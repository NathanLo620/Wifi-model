/*
 * Copyright (c) 2020 Universita' degli Studi di Napoli Federico II
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#include "qos-frame-exchange-manager.h"

#include "ap-wifi-mac.h"
#include "channel-access-manager.h"
#include "wifi-mac-queue.h"
#include "wifi-mac-trailer.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/wifi-mac.h"
#include "ns3/qos-utils.h" // Added for P-EDCA logic
#include "wifi-protection.h"
#include "wifi-protection-manager.h"

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT WIFI_FEM_NS_LOG_APPEND_CONTEXT

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("QosFrameExchangeManager");

NS_OBJECT_ENSURE_REGISTERED(QosFrameExchangeManager);

TypeId
QosFrameExchangeManager::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::QosFrameExchangeManager")
            .SetParent<FrameExchangeManager>()
            .AddConstructor<QosFrameExchangeManager>()
            .SetGroupName("Wifi")
            .AddTraceSource("PedcaTx",
                            "A P-EDCA Stage 2 transmission",
                            MakeTraceSourceAccessor(&QosFrameExchangeManager::m_pedcaTxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("EdcaTx",
                            "An EDCA transmission",
                            MakeTraceSourceAccessor(&QosFrameExchangeManager::m_edcaTxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("PedcaAttempt",
                            "A P-EDCA Stage 1 attempt (DS-CTS sent)",
                            MakeTraceSourceAccessor(&QosFrameExchangeManager::m_pedcaAttemptTrace),
                            "ns3::Packet::TracedCallback")
            .AddAttribute("PifsRecovery",
                          "Perform a PIFS recovery as a response to transmission failure "
                          "within a TXOP",
                          BooleanValue(true),
                          MakeBooleanAccessor(&QosFrameExchangeManager::m_pifsRecovery),
                          MakeBooleanChecker())
            .AddAttribute("SetQueueSize",
                          "Whether to set the Queue Size subfield of the QoS Control field "
                          "of QoS data frames sent by non-AP stations",
                          BooleanValue(false),
                          MakeBooleanAccessor(&QosFrameExchangeManager::m_setQosQueueSize),
                          MakeBooleanChecker())
            .AddAttribute("ProtectSingleExchange",
                          "Whether the Duration/ID field in frames establishing protection only "
                          "covers the immediate frame exchange instead of rest of the TXOP limit "
                          "when the latter is non-zero",
                          BooleanValue(false),
                          MakeBooleanAccessor(&QosFrameExchangeManager::m_protectSingleExchange),
                          MakeBooleanChecker())
            .AddAttribute(
                "SingleExchangeProtectionSurplus",
                "Additional time to protect beyond end of the immediate frame exchange in case of "
                "non-zero TXOP limit when a single frame exchange is protected",
                TimeValue(Time(0)),
                MakeTimeAccessor(&QosFrameExchangeManager::m_singleExchangeProtectionSurplus),
                MakeTimeChecker());
    return tid;
}

QosFrameExchangeManager::QosFrameExchangeManager()
    : m_initialFrame(false),
      m_pedcaPending(false),  // P-EDCA state
      m_psrc(0),              // P-EDCA STA Retry Counter
      m_qsrc(0)               // Queue Size Retry Counter
{
    NS_LOG_FUNCTION(this);
}

QosFrameExchangeManager::~QosFrameExchangeManager()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
QosFrameExchangeManager::SetWifiPhy(Ptr<WifiPhy> phy)
{
    FrameExchangeManager::SetWifiPhy(phy);
    // Connect PhyTxEnd trace permanently so PedcaPhyTxEndCallback can detect DS-CTS TX end
    // without the per-transmission polling loop.  The flag m_pedcaTxEndPending gates activity.
    phy->TraceConnectWithoutContext(
        "PhyTxEnd",
        MakeCallback(&QosFrameExchangeManager::PedcaPhyTxEndCallback, this));
}

void
QosFrameExchangeManager::ResetPhy()
{
    if (m_phy)
    {
        m_phy->TraceDisconnectWithoutContext(
            "PhyTxEnd",
            MakeCallback(&QosFrameExchangeManager::PedcaPhyTxEndCallback, this));
    }
    FrameExchangeManager::ResetPhy();
}

void
QosFrameExchangeManager::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_edca = nullptr;
    m_edcaBackingOff = nullptr;
    m_pifsRecoveryEvent.Cancel();
    FrameExchangeManager::DoDispose();
}

bool
QosFrameExchangeManager::SendCfEndIfNeeded()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_edca);

    WifiMacHeader cfEnd;
    cfEnd.SetType(WIFI_MAC_CTL_END);
    cfEnd.SetDsNotFrom();
    cfEnd.SetDsNotTo();
    cfEnd.SetNoRetry();
    cfEnd.SetNoMoreFragments();
    cfEnd.SetDuration(Seconds(0));
    cfEnd.SetAddr1(Mac48Address::GetBroadcast());
    cfEnd.SetAddr2(m_self);

    WifiTxVector cfEndTxVector =
        GetWifiRemoteStationManager()->GetRtsTxVector(cfEnd.GetAddr1(), m_allowedWidth);

    auto mpdu = Create<WifiMpdu>(Create<Packet>(), cfEnd);
    auto txDuration =
        WifiPhy::CalculateTxDuration(mpdu->GetSize(), cfEndTxVector, m_phy->GetPhyBand());

    // Send the CF-End frame if the remaining TXNAV is long enough to transmit this frame
    if (m_txNav > Simulator::Now() + txDuration)
    {
        NS_LOG_DEBUG("Send CF-End frame");
        ForwardMpduDown(mpdu, cfEndTxVector);
        Simulator::Schedule(txDuration,
                            &QosFrameExchangeManager::NotifyChannelReleased,
                            this,
                            m_edca);
        ResetTxNav();
        return true;
    }

    NotifyChannelReleased(m_edca);
    m_edca = nullptr;
    return false;
}

void
QosFrameExchangeManager::PifsRecovery(bool forceCurrentCw)
{
    NS_LOG_FUNCTION(this << forceCurrentCw);
    NS_ASSERT(m_edca);
    NS_ASSERT(m_edca->GetTxopStartTime(m_linkId).has_value());

    // Release the channel if it has not been idle for the last PIFS interval
    m_allowedWidth = std::min(
        m_allowedWidth,
        m_channelAccessManager->GetLargestIdlePrimaryChannel(m_phy->GetPifs(), Simulator::Now()));

    if (m_allowedWidth == MHz_u{0})
    {
        // PIFS recovery failed, TXOP is terminated
        NotifyChannelReleased(m_edca);
        if (!forceCurrentCw)
        {
            m_edca->UpdateFailedCw(m_linkId);
        }
        m_edca = nullptr;
    }
    else
    {
        // the txopDuration parameter is unused because we are not starting a new TXOP
        StartTransmission(m_edca, Seconds(0));
    }
}

void
QosFrameExchangeManager::CancelPifsRecovery()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_pifsRecoveryEvent.IsPending());
    NS_ASSERT(m_edca);

    NS_LOG_DEBUG("Cancel PIFS recovery being attempted by EDCAF " << m_edca);
    m_pifsRecoveryEvent.Cancel();
    NotifyChannelReleased(m_edca);
}

bool
QosFrameExchangeManager::StartTransmission(Ptr<Txop> edca, MHz_u allowedWidth)
{
    NS_LOG_FUNCTION(this << edca << allowedWidth);

    if (m_pifsRecoveryEvent.IsPending())
    {
        // Another AC (having AIFS=1 or lower, if the user changed the default settings)
        // gained channel access while performing PIFS recovery. Abort PIFS recovery
        CancelPifsRecovery();
    }

    // TODO This will become an assert once no Txop is installed on a QoS station
    if (!edca->IsQosTxop())
    {
        m_edca = nullptr;
        return FrameExchangeManager::StartTransmission(edca, allowedWidth);
    }

    m_allowedWidth = allowedWidth;
    auto qosTxop = StaticCast<QosTxop>(edca);
    return StartTransmission(qosTxop, qosTxop->GetTxopLimit(m_linkId));
}

bool
QosFrameExchangeManager::StartTransmission(Ptr<QosTxop> edca, Time txopDuration)
{
    NS_LOG_FUNCTION(this << edca << txopDuration);

    if (m_pifsRecoveryEvent.IsPending())
    {
        // Another AC (having AIFS=1 or lower, if the user changed the default settings)
        // gained channel access while performing PIFS recovery. Abort PIFS recovery
        CancelPifsRecovery();
    }

    // Capture state before cancelling timer
    bool waitingForResponse = m_txTimer.IsRunning();

    if (m_txTimer.IsRunning())
    {
        m_txTimer.Cancel();
    }
    m_dcf = edca;
    m_edca = edca;

    // We check if this EDCAF invoked the backoff procedure (without terminating
    // the TXOP) because the transmission of a non-initial frame of a TXOP failed
    bool backingOff = (m_edcaBackingOff == m_edca);

    if (backingOff)
    {
        NS_ASSERT(m_edca->GetTxopLimit(m_linkId).IsStrictlyPositive());
        NS_ASSERT(m_edca->GetTxopStartTime(m_linkId));
        NS_ASSERT(!m_pifsRecovery);
        NS_ASSERT(!m_initialFrame);

        // clear the member variable
        m_edcaBackingOff = nullptr;
    }

    if (m_edca->GetTxopLimit(m_linkId).IsStrictlyPositive())
    {
        // TXOP limit is not null. We have to check if this EDCAF is starting a
        // new TXOP. This includes the case when the transmission of a non-initial
        // frame of a TXOP failed and backoff was invoked without terminating the
        // TXOP. In such a case, we assume that a new TXOP is being started if it
        // elapsed more than TXOPlimit since the start of the paused TXOP. Note
        // that GetRemainingTxop returns 0 iff Now - TXOPstart >= TXOPlimit
        if (!m_edca->GetTxopStartTime(m_linkId) ||
            (backingOff && m_edca->GetRemainingTxop(m_linkId).IsZero()))
        {
            // P-EDCA Implementation: Strict P-EDCA (Two-Stage Access)
            // We only intervene if we are starting a NEW TXOP.
            if (m_mac->GetPedcaSupported() && m_edca->GetAccessCategory() == AC_VO)
            {
                // Comment Resolution #4555, #7657, #12651:
                // dot11ShortRetryLimit must be configured higher than dot11PEDCARetryThreshold.
                // If not, P-EDCA would be unreachable (MPDU dropped before QSRC hits threshold).
                // Auto-raise FrameRetryLimit if violated.
                uint32_t frameRetryLimit = m_mac->GetFrameRetryLimit();
                if (frameRetryLimit <= PEDCA_RETRY_THRESHOLD)
                {
                    uint32_t newLimit = PEDCA_RETRY_THRESHOLD + 1;
                    std::clog << "[P-EDCA CONFIG] WARNING: dot11ShortRetryLimit ("
                              << frameRetryLimit << ") <= dot11PEDCARetryThreshold ("
                              << PEDCA_RETRY_THRESHOLD << "). "
                              << "Auto-raising FrameRetryLimit to " << newLimit
                              << " (per CR #4555, #7657, #12651)" << std::endl;
                    m_mac->SetFrameRetryLimit(newLimit);
                }

                // P-EDCA Trigger Check (per 802.11bn D1.3):
                // 1. QSRC[AC_VO] >= dot11PEDCARetryThreshold
                // 2. PSRC[AC_VO] < dot11PEDCAConsecutiveAttempt
                // 3. dot11ShortRetryLimit > dot11PEDCARetryThreshold (CIDs 4555/7657/12651)
                // 4. AIFSN[AC_VO] is set to a nonzero value (CIDs 7112/11411/11759)
                //    Rationale: P-EDCA slot boundary = SIFS + (2+DSr)*aSlotTime requires AIFSN≥2;
                //    if AIFSN=0 the slot boundary is undefined.

                std::clog << "[P-EDCA TRIGGER CHECK] t=" << Simulator::Now().GetMicroSeconds()
                          << "us QSRC=" << m_qsrc << " PSRC=" << +m_psrc
                          << " AIFSN=" << +m_edca->GetAifsn(m_linkId)
                          << " m_pedcaPending=" << (m_pedcaPending ? "TRUE" : "FALSE")
                          << std::endl;

                bool qsrcOk = (m_qsrc >= PEDCA_RETRY_THRESHOLD);
                bool psrcOk = (m_psrc < PEDCA_CONSECUTIVE_ATTEMPT);
                bool retryLimitOk = (m_mac->GetFrameRetryLimit() > PEDCA_RETRY_THRESHOLD);
                // CIDs 7112/11411/11759: AIFSN[AC_VO] must be nonzero for P-EDCA slot boundary to be defined
                bool aifsn_nonzero = (m_edca->GetAifsn(m_linkId) > 0);
                
                // DEFERRAL RULES: Check EIFS (implicit), CTS/Ack Timeout, and NAV
                bool navActive = !VirtualCsMediumIdle();
                bool phyBusy = (m_phy->IsStateTx() || m_phy->IsStateRx() || m_phy->IsStateSwitching() || m_phy->IsStateCcaBusy());

                bool deferralRequired = waitingForResponse || navActive || phyBusy;

                // Explicitly check if we SHOULD defer P-EDCA
                if (!m_pedcaPending && deferralRequired)
                {
                     m_pedcaFailDeferral++;  // TRACE: deferral count
                     std::clog << "[P-EDCA DEFERRAL] Condition met at t=" << Simulator::Now().GetMicroSeconds() << "us: "
                               << (waitingForResponse ? "WaitingForResponse " : "")
                               << (navActive ? "NAV>0 " : "")
                               << (phyBusy ? "PHY-Busy " : "")
                               << "-> Aborting DS-CTS schedule (Fallback/Defer)" << std::endl;
                     
                     // If deferral is required, we effectively "do NOT schedule DS-CTS".
                     // We must abort this access attempt to respect the deferral rules.
                     NotifyChannelReleased(m_edca);
                     
                     // P-EDCA TIMING FIX:
                     // If we defer P-EDCA (e.g. due to busy medium), we should retry using Stage 1 backoff.
                     // Generate DSr (0~CWds) backoff slots if P-EDCA conditions are met.
                     if (qsrcOk && psrcOk && retryLimitOk && aifsn_nonzero)
                     {
                         std::clog << "[P-EDCA PRIORITY] Deferred: Generating Stage 1 backoff (CWds=0) for ASAP retry" << std::endl;
                         m_edca->GeneratePedcaStage1Backoff(0, m_linkId);
                     }
                     
                     m_edca = nullptr;
                     return false;
                }

                if (!m_pedcaPending && qsrcOk && psrcOk && retryLimitOk && aifsn_nonzero)
                {
                    // === P-EDCA Stage 1: Send DS-CTS ===
                    //
                    // DS-CTS is sent at the P-EDCA slot boundary (D1.3 terminology, replacing DSAIFS):
                    //   P-EDCA slot boundary = SIFS + (2 + DSr) × aSlotTime
                    //   With DSr=0: = 16µs + 2×9µs = 34µs after last medium-busy-end.
                    // This is the EDCA slot boundary with AIFSN[AC_VO] set to 2+DSr
                    // (per CIDs 5780/5782/6457/11751/11757/10258).
                    //
                    // P-EDCA VO has higher internal priority than normal EDCA VO.
                    // TransmissionFailed forces backoff=0 when P-EDCA conditions are met,
                    // so the STA accesses the channel after only AIFS (no random backoff).
                    // No virtual collision bypass is needed.
                    
                    Time nowTime = Simulator::Now();
                    
                    // Get the access grant start (last busy end + SIFS)
                    // This tells us when the channel became idle for contention
                    Ptr<ChannelAccessManager> cam = m_mac->GetChannelAccessManager(m_linkId);
                    Time accessGrantStart = Seconds(0);
                    if (cam)
                    {
                        accessGrantStart = cam->GetAccessGrantStart();
                    }
                    
                    // Calculate expected P-EDCA slot boundary timing (D1.3 terminology, replacing DSAIFS).
                    // P-EDCA slot boundary = SIFS + (2 + DSr) × aSlotTime (from last medium-busy-end).
                    // accessGrantStart = lastBusy + SIFS (already includes SIFS).
                    Time sifs = m_phy->GetSifs();    // 16µs for 5GHz OFDM
                    Time slotTime = m_phy->GetSlot(); // 9µs for OFDM
                    uint8_t aifsn = 2;               // DS-CTS sent at AIFSN=2 P-EDCA slot boundary
                    uint32_t dsr = m_edca->GetBackoffSlots(m_linkId); // DSr drawn from [0, CWds]

                    // P-EDCA slot boundary from lastBusy = SIFS + (2+DSr)*Slot
                    Time lastBusyPlusSifs = accessGrantStart;  // accessGrantStart = lastBusy + SIFS
                    Time expectedDsCtsTime = lastBusyPlusSifs + (aifsn + dsr) * slotTime;
                    
                    // Verify DS-CTS timing against P-EDCA slot boundary.
                    // Expected: DS-CTS sent exactly at P-EDCA slot boundary = (2+DSr)*Slot from accessGrant.
                    Time gapFromAccessGrant = nowTime - accessGrantStart;
                    Time targetGapFromGrant = (aifsn + dsr) * slotTime;  // (2+DSr)×9µs

                    std::clog << "[P-EDCA STAGE1] Sending DS-CTS at t="
                              << nowTime.GetMicroSeconds() << "us"
                              << " (P-EDCA slot boundary: +" << (sifs + targetGapFromGrant).GetMicroSeconds()
                              << "µs from lastBusy, DSr=" << dsr << ")" << std::endl;

                    if (gapFromAccessGrant > targetGapFromGrant + MicroSeconds(200)) {
                         std::clog << "[P-EDCA TIMING] Channel was idle; gap from grant="
                                   << gapFromAccessGrant.GetMicroSeconds() << "us (target="
                                   << targetGapFromGrant.GetMicroSeconds() << "us). (OK — long idle)" << std::endl;
                    } else {
                        int64_t delta = gapFromAccessGrant.GetMicroSeconds() - targetGapFromGrant.GetMicroSeconds();
                        if (std::abs(delta) <= 1) {
                            std::clog << "[P-EDCA TIMING] DS-CTS at exact P-EDCA slot boundary. (OK)" << std::endl;
                        } else {
                            // Warn if timing is off from expected P-EDCA slot boundary
                            double lateSlots = (double)delta / slotTime.GetMicroSeconds();
                            std::clog << "[P-EDCA WARNING] DS-CTS late by " << lateSlots
                                      << " slots (gap=" << gapFromAccessGrant.GetMicroSeconds()
                                      << "us, target=" << targetGapFromGrant.GetMicroSeconds() << "us)" << std::endl;
                        }
                    }

                    // 2. Construct DS-CTS (CTS-to-Self)
                    WifiMacHeader ctsHeader;
                    ctsHeader.SetType(WIFI_MAC_CTL_CTS);
                    ctsHeader.SetDsNotFrom();
                    ctsHeader.SetDsNotTo();
                    ctsHeader.SetNoMoreFragments();
                    ctsHeader.SetNoRetry();
                    ctsHeader.SetAddr1(Mac48Address("00:0F:AC:47:43:00")); // P-EDCA fixed RA (per draft)
                    
                    // P-EDCA DS-CTS Duration: 77µs (per P-EDCA standard/draft resolutions)
                    // Reserves a 77us contention window:
                    //   - NAV freezes non-P-EDCA STAs
                    //   - P-EDCA STAs contend with (CW=7, AIFSN=2) 
                    Time pedcaWindowDuration = MicroSeconds(77);
                    ctsHeader.SetDuration(pedcaWindowDuration);

                    // 3. Transmit DS-CTS using non-HT OFDM 6 Mbps (per P-EDCA draft 3.5)
                    // Standard requires: non-HT PPDU, 6 Mb/s, scrambler seed=32
                    WifiTxVector ctsTxVector;
                    ctsTxVector.SetMode(WifiMode("OfdmRate6Mbps"));  // non-HT 6 Mbps
                    ctsTxVector.SetPreambleType(WIFI_PREAMBLE_LONG); // non-HT preamble
                    ctsTxVector.SetTxPowerLevel(0);
                    ctsTxVector.SetChannelWidth(20);

                    Ptr<WifiMpdu> mpdu = Create<WifiMpdu>(Create<Packet>(), ctsHeader);

                    // PHY Busy check moved to deferralRequired block above for robustness

                    ForwardMpduDown(mpdu, ctsTxVector);
                    
                    m_dsCtsCount++;  // TRACE: DS-CTS sent count
                    
                    // Fire P-EDCA Attempt trace (DS-CTS sent = one P-EDCA attempt)
                    m_pedcaAttemptTrace(mpdu->GetPacket());
                    
                    Time ctsAirtime = m_phy->CalculateTxDuration(mpdu->GetPacketSize(),
                                                                ctsTxVector,
                                                                m_phy->GetPhyBand());
                    Time ctsTxEnd = Simulator::Now() + ctsAirtime;
                    
                    // PSRC++ on DS-CTS transmission (per draft 3.1)
                    m_psrc++;
                    std::clog << "[P-EDCA TIMING] DS-CTS TX end at t=" 
                              << ctsTxEnd.GetMicroSeconds() << "us (airtime=" 
                              << ctsAirtime.GetMicroSeconds() << "us) PSRC=" << +m_psrc << std::endl;

                    // 4. Disable THIS STA's non-VO ACs during P-EDCA window
                    // This prevents internal collision from VI/BE/BK on this STA
                    // Note: cam was already declared above for timing verification
                    if (cam)
                    {
                        for (const auto& ac : {AC_BE, AC_BK, AC_VI})
                        {
                            Ptr<QosTxop> other = m_mac->GetQosTxop(ac);
                            if (other)
                            {
                                // P-EDCA Support: Use explicit suspend instead of DisableEdcaFor (timer-based)
                                // Standard requires other ACs to remain suspended across consecutive DS-CTS attempts.
                                other->SetPedcaSuspended(true, m_linkId);
                            }
                        }
                    }

                    // 5. Switch to P-EDCA Parameters for Stage 2 contention
                    // NOTE: Do NOT set m_pedcaPending = true here! 
                    // We only set it in the callback when we truly begin Stage 2 contention.
                    // This prevents StartTransmission from mistakenly entering Stage 2 during CTS transmission.
                    
                    // Pre-calculate CTS end time for the callback locally
                    Time pedcaCtsTxEnd = Simulator::Now() + ctsAirtime;
                    
                    // CRITICAL: Use m_linkId when setting P-EDCA parameters!
                    // The default SetMinCw(7) only sets linkId=0 which may be wrong.
                    m_edca->SetMinCw(7, m_linkId);
                    m_edca->SetMaxCw(7, m_linkId);
                    m_edca->SetAifsn(2, m_linkId);
                    
                    // Force reset CW to ensure P-EDCA uses CW=7
                    m_edca->ResetCw(m_linkId);
                    
                    /*NS_LOG_DEBUG("P-EDCA: Switched to P-EDCA params on link " << +m_linkId 
                                 << " (CWmin=7, CWmax=7, AIFSN=2, CW=" << m_edca->GetCw(m_linkId) << ")");*/

                    // 6. Connect PhyTxEnd trace for zero-delay Stage 2 entry.
                    //
                    // PhyTxEnd fires from WifiPhy::TxDone at the EXACT CTS TX end time,
                    // with no polling delay.  From that moment, Stage 2 timing is:
                    //   AIFS(34µs) + backoff(0–63µs) = [34, 97]µs from ctsTxEnd.
                    //   NAV window = 77µs → backoff 0–4 (gap 34–70µs) are protected (5/8 = 62.5%).
                    //   Backoff 5–7 (gap 79–97µs) exceed 77µs → TIMING EXPIRED (3/8 = 37.5%).
                    m_pedcaEdca = m_edca;
                    m_pedcaTxEndPending = true;  // arm the permanently-connected PhyTxEnd callback

                    // Prevent immediate data transmission in this call stack
                    m_edca = nullptr;
                    
                    // Return false - we sent CTS but not data yet
                    return false;
                }
                else if (m_pedcaPending)
                {
                     // === P-EDCA Stage 2: Backoff Complete, Transmit Data ===
                     Time payloadStart = Simulator::Now();
                     bool stage2Valid = false;

                     // Determine if this is a valid P-EDCA transmission within the 77us window
                     if (m_pedcaCtsTxEnd > Seconds(0))
                     {
                         Time gap = payloadStart - m_pedcaCtsTxEnd;
                         double gapUs = gap.GetMicroSeconds();
                         
                         std::clog << "[P-EDCA STAGE2] Payload TX start at t=" << payloadStart.GetMicroSeconds()
                                   << "us, CTS TX end=" << m_pedcaCtsTxEnd.GetMicroSeconds()
                                   << "us, gap=" << gapUs << "us";

                         // NAV window = 77µs (802.11bn draft).  Gap from ctsTxEnd:
                         //   AIFS(34µs) + backoff(0–63µs) = [34, 97]µs.
                         //   Backoff 0–4 → gap 34–70µs ≤ 77µs → protected (TIMING OK, 5/8 probability).
                         //   Backoff 5–7 → gap 79–97µs > 77µs → TIMING EXPIRED (3/8 probability).
                         if (gapUs <= 77.0) {
                             std::clog << " ✓ TIMING OK" << std::endl;
                              m_stage2TxStartCount++;  // TRACE: Stage 2 valid TX
                             stage2Valid = true;
                         } else {
                             // gap > 77µs: P-EDCA NAV reservation expired at AP/others.
                             // But the STA itself still continues and finishes its Stage 2 backoff.
                             // It is still a P-EDCA transmission, just unprotected by DS-CTS NAV.
                             std::clog << " ⚠ TIMING EXPIRED (gap=" << gapUs << "us > 77us) - NAV expired, but continuing Stage 2" << std::endl;
                              m_pedcaFailTimingExpired++;  // TRACE: timing expired
                              m_stage2TxStartCount++;      // TRACE: Still counts as Stage 2 TX start
                             stage2Valid = true;          // DO NOT fallback to EDCA
                         }
                     }
                     else
                     {
                         // No valid timestamp -> Stale state, force clear
                         std::clog << "[P-EDCA STATE ERROR] No timestamp - Fallback to EDCA" << std::endl;
                         stage2Valid = false;
                     }

                     // Always clear P-EDCA pending flags when leaving this block (either success or expiry)
                     m_pedcaPending = false;
                     m_pedcaCtsTxEnd = Seconds(0);
                     
                     // CRITICAL FIX: Restore VO default parameters from dot11EDCATable!
                     // Per 802.11 Table 9-155: AC_VO: CWmin=3, CWmax=7, AIFSN=2
                     // Whether Stage 2 was valid or expired, we must return to normal EDCA rules.
                     constexpr uint32_t VO_DEFAULT_CWMIN = 3;   // 2^(ECWmin=1+1) - 1 = 3
                     constexpr uint32_t VO_DEFAULT_CWMAX = 7;   // 2^(ECWmax=2+1) - 1 = 7
                     constexpr uint8_t  VO_DEFAULT_AIFSN = 2;
                     m_edca->SetMinCw(VO_DEFAULT_CWMIN, m_linkId);
                     m_edca->SetMaxCw(VO_DEFAULT_CWMAX, m_linkId);
                     m_edca->SetAifsn(VO_DEFAULT_AIFSN, m_linkId);

                     if (stage2Valid)
                     {
                         // Mark Stage 2 as active for specific collision handling
                         m_pedcaStage2Active = true;
                         // Continue logic below (StartFrameExchange will be called)
                     }
                     else
                     {
                         // Fallback to normal EDCA: Active flag false
                         m_pedcaStage2Active = false;
                         // Continue logic below (will just be a normal EDCA StartFrameExchange)
                     }
                }
                // else: conditions not met (QSRC < threshold or PSRC >= limit) - proceed with normal EDCA
            }

            // starting a new TXOP
            m_edca->NotifyChannelAccessed(m_linkId, txopDuration);

            if (StartFrameExchange(m_edca, txopDuration, true))
            {
                m_initialFrame = true;
                return true;
            }

            // TXOP not even started, return false
            NS_LOG_DEBUG("No frame transmitted");
            NotifyChannelReleased(m_edca);
            m_edca = nullptr;
            return false;
        }

        // We are continuing a TXOP, check if we can transmit another frame
        NS_ASSERT(!m_initialFrame);

        if (!StartFrameExchange(m_edca, m_edca->GetRemainingTxop(m_linkId), false))
        {
            NS_LOG_DEBUG("Not enough remaining TXOP time");
            return SendCfEndIfNeeded();
        }

        return true;
    }

    // we get here if TXOP limit is null
    m_initialFrame = true;

    if (StartFrameExchange(m_edca, Time::Min(), true))
    {
        m_edca->NotifyChannelAccessed(m_linkId, Seconds(0));
        return true;
    }

    NS_LOG_DEBUG("No frame transmitted");
    NotifyChannelReleased(m_edca);
    m_edca = nullptr;
    return false;
}

bool
QosFrameExchangeManager::StartFrameExchange(Ptr<QosTxop> edca,
                                            Time availableTime,
                                            bool initialFrame)
{
    NS_LOG_FUNCTION(this << edca << availableTime << initialFrame);

    Ptr<WifiMpdu> mpdu = edca->PeekNextMpdu(m_linkId);

    // Even though channel access is requested when the queue is not empty, at
    // the time channel access is granted the lifetime of the packet might be
    // expired and the queue might be empty.
    if (!mpdu)
    {
        NS_LOG_DEBUG("Queue empty");
        return false;
    }

    mpdu = CreateAliasIfNeeded(mpdu);
    WifiTxParameters txParams;
    txParams.m_txVector =
        GetWifiRemoteStationManager()->GetDataTxVector(mpdu->GetHeader(), m_allowedWidth);

    Ptr<WifiMpdu> item = edca->GetNextMpdu(m_linkId, mpdu, txParams, availableTime, initialFrame);

    if (!item)
    {
        NS_LOG_DEBUG("Not enough time to transmit a frame");
        return false;
    }

    NS_ASSERT_MSG(!item->GetHeader().IsQosData() || !item->GetHeader().IsQosAmsdu(),
                  "We should not get an A-MSDU here");

    // check if the MSDU needs to be fragmented
    item = GetFirstFragmentIfNeeded(item);

    // update the protection method if the frame was fragmented
    if (item->IsFragment() && item->GetSize() != mpdu->GetSize())
    {
        WifiTxParameters fragmentTxParams;
        fragmentTxParams.m_txVector = txParams.m_txVector;
        fragmentTxParams.AddMpdu(item);
        UpdateTxDuration(item->GetHeader().GetAddr1(), fragmentTxParams);
        txParams.m_protection = GetProtectionManager()->TryAddMpdu(item, fragmentTxParams);
        NS_ASSERT(txParams.m_protection);
    }

    SendMpduWithProtection(item, txParams);

    if (item->GetPacket())
    {
        m_edcaTxTrace(item->GetPacket());
        if (m_pedcaStage2Active)
        {
            m_pedcaTxTrace(item->GetPacket());
        }
    }

    return true;
}

Ptr<WifiMpdu>
QosFrameExchangeManager::CreateAliasIfNeeded(Ptr<WifiMpdu> mpdu) const
{
    return mpdu;
}

bool
QosFrameExchangeManager::TryAddMpdu(Ptr<const WifiMpdu> mpdu,
                                    WifiTxParameters& txParams,
                                    Time availableTime) const
{
    NS_ASSERT(mpdu);
    NS_LOG_FUNCTION(this << *mpdu << &txParams << availableTime);

    // tentatively add the given MPDU
    auto prevTxDuration = txParams.m_txDuration;
    txParams.AddMpdu(mpdu);
    UpdateTxDuration(mpdu->GetHeader().GetAddr1(), txParams);

    // check if adding the given MPDU requires a different protection method
    std::optional<Time> protectionTime; // uninitialized
    if (txParams.m_protection)
    {
        protectionTime = txParams.m_protection->protectionTime;
    }

    std::unique_ptr<WifiProtection> protection;
    protection = GetProtectionManager()->TryAddMpdu(mpdu, txParams);

    // Force RTS for P-EDCA Stage 2, if no RTS protection is scheduled
    if (m_pedcaStage2Active && m_edca && m_edca->GetAccessCategory() == AC_VO)
    {
        bool currentHasRts = (txParams.m_protection && txParams.m_protection->method == WifiProtection::RTS_CTS);
        bool newHasRts = (protection && protection->method == WifiProtection::RTS_CTS);
        if (!currentHasRts && !newHasRts)
        {
            auto p = std::make_unique<WifiRtsCtsProtection>();
            p->rtsTxVector = GetWifiRemoteStationManager()->GetRtsTxVector(
                mpdu->GetHeader().GetAddr1(), txParams.m_txVector.GetChannelWidth());
            p->ctsTxVector = GetWifiRemoteStationManager()->GetCtsTxVector(
                mpdu->GetHeader().GetAddr1(), p->rtsTxVector.GetMode());
            protection = std::move(p);
        }
    }

    bool protectionSwapped = false;

    if (protection)
    {
        // the protection method has changed, calculate the new protection time
        CalculateProtectionTime(protection.get());
        protectionTime = protection->protectionTime;
        // swap unique pointers, so that the txParams that is passed to the next
        // call to IsWithinLimitsIfAddMpdu is the most updated one
        txParams.m_protection.swap(protection);
        protectionSwapped = true;
    }
    NS_ASSERT(protectionTime.has_value());
    NS_LOG_DEBUG("protection time=" << *protectionTime);

    // check if adding the given MPDU requires a different acknowledgment method
    std::optional<Time> acknowledgmentTime; // uninitialized
    if (txParams.m_acknowledgment)
    {
        acknowledgmentTime = txParams.m_acknowledgment->acknowledgmentTime;
    }

    std::unique_ptr<WifiAcknowledgment> acknowledgment;
    acknowledgment = GetAckManager()->TryAddMpdu(mpdu, txParams);
    bool acknowledgmentSwapped = false;

    if (acknowledgment)
    {
        // the acknowledgment method has changed, calculate the new acknowledgment time
        CalculateAcknowledgmentTime(acknowledgment.get());
        acknowledgmentTime = acknowledgment->acknowledgmentTime;
        // swap unique pointers, so that the txParams that is passed to the next
        // call to IsWithinLimitsIfAddMpdu is the most updated one
        txParams.m_acknowledgment.swap(acknowledgment);
        acknowledgmentSwapped = true;
    }
    NS_ASSERT(acknowledgmentTime.has_value());
    NS_LOG_DEBUG("acknowledgment time=" << *acknowledgmentTime);

    Time ppduDurationLimit = Time::Min();
    if (availableTime != Time::Min())
    {
        ppduDurationLimit = availableTime - *protectionTime - *acknowledgmentTime;
    }

    if (!IsWithinLimitsIfAddMpdu(mpdu, txParams, ppduDurationLimit))
    {
        // adding MPDU failed, undo the addition of the MPDU and restore protection and
        // acknowledgment methods if they were swapped
        txParams.UndoAddMpdu();
        txParams.m_txDuration = prevTxDuration;
        if (protectionSwapped)
        {
            txParams.m_protection.swap(protection);
        }
        if (acknowledgmentSwapped)
        {
            txParams.m_acknowledgment.swap(acknowledgment);
        }
        return false;
    }

    return true;
}

bool
QosFrameExchangeManager::IsWithinLimitsIfAddMpdu(Ptr<const WifiMpdu> mpdu,
                                                 const WifiTxParameters& txParams,
                                                 Time ppduDurationLimit) const
{
    NS_ASSERT(mpdu);
    NS_LOG_FUNCTION(this << *mpdu << &txParams << ppduDurationLimit);

    // A QoS station only has to check that the MPDU transmission time does not
    // exceed the given limit
    return IsWithinSizeAndTimeLimits(mpdu->GetSize(),
                                     mpdu->GetHeader().GetAddr1(),
                                     txParams,
                                     ppduDurationLimit);
}

bool
QosFrameExchangeManager::IsWithinSizeAndTimeLimits(uint32_t ppduPayloadSize,
                                                   Mac48Address receiver,
                                                   const WifiTxParameters& txParams,
                                                   Time ppduDurationLimit) const
{
    NS_LOG_FUNCTION(this << ppduPayloadSize << receiver << &txParams << ppduDurationLimit);

    if (ppduDurationLimit != Time::Min() && ppduDurationLimit.IsNegative())
    {
        NS_LOG_DEBUG("ppduDurationLimit is null or negative, time limit is trivially exceeded");
        return false;
    }

    if (ppduPayloadSize > WifiPhy::GetMaxPsduSize(txParams.m_txVector.GetModulationClass()))
    {
        NS_LOG_DEBUG("the frame exceeds the max PSDU size");
        return false;
    }

    // Get the maximum PPDU Duration based on the preamble type
    Time maxPpduDuration = GetPpduMaxTime(txParams.m_txVector.GetPreambleType());

    NS_ASSERT_MSG(txParams.m_txDuration, "TX duration not yet computed");
    auto txTime = txParams.m_txDuration.value();
    NS_LOG_DEBUG("PPDU duration: " << txTime.As(Time::MS));

    if ((ppduDurationLimit.IsStrictlyPositive() && txTime > ppduDurationLimit) ||
        (maxPpduDuration.IsStrictlyPositive() && txTime > maxPpduDuration))
    {
        NS_LOG_DEBUG(
            "the frame does not meet the constraint on max PPDU duration or PPDU duration limit");
        return false;
    }

    return true;
}

Time
QosFrameExchangeManager::GetFrameDurationId(const WifiMacHeader& header,
                                            uint32_t size,
                                            const WifiTxParameters& txParams,
                                            Ptr<Packet> fragmentedPacket) const
{
    NS_LOG_FUNCTION(this << header << size << &txParams << fragmentedPacket);

    const auto singleDurationId =
        FrameExchangeManager::GetFrameDurationId(header, size, txParams, fragmentedPacket);

    // TODO This will be removed once no Txop is installed on a QoS station
    if (!m_edca)
    {
        return singleDurationId;
    }

    if (m_edca->GetTxopLimit(m_linkId).IsZero())
    {
        return singleDurationId;
    }

    NS_ASSERT(txParams.m_acknowledgment &&
              txParams.m_acknowledgment->acknowledgmentTime.has_value());

    // under multiple protection settings, if the TXOP limit is not null, Duration/ID
    // is set to cover the remaining TXOP time (Sec. 9.2.5.2 of 802.11-2016).
    // The TXOP holder may exceed the TXOP limit in some situations (Sec. 10.22.2.8
    // of 802.11-2016)
    auto duration =
        std::max(m_edca->GetRemainingTxop(m_linkId) -
                     WifiPhy::CalculateTxDuration(size, txParams.m_txVector, m_phy->GetPhyBand()),
                 *txParams.m_acknowledgment->acknowledgmentTime);

    if (m_protectSingleExchange)
    {
        duration = std::min(duration, singleDurationId + m_singleExchangeProtectionSurplus);
    }

    return duration;
}

Time
QosFrameExchangeManager::GetRtsDurationId(const WifiTxVector& rtsTxVector,
                                          Time txDuration,
                                          Time response) const
{
    NS_LOG_FUNCTION(this << rtsTxVector << txDuration << response);

    const auto singleDurationId =
        FrameExchangeManager::GetRtsDurationId(rtsTxVector, txDuration, response);

    // TODO This will be removed once no Txop is installed on a QoS station
    if (!m_edca)
    {
        return singleDurationId;
    }

    if (m_edca->GetTxopLimit(m_linkId).IsZero())
    {
        return singleDurationId;
    }

    // under multiple protection settings, if the TXOP limit is not null, Duration/ID
    // is set to cover the remaining TXOP time (Sec. 9.2.5.2 of 802.11-2016).
    // The TXOP holder may exceed the TXOP limit in some situations (Sec. 10.22.2.8
    // of 802.11-2016)
    auto duration =
        std::max(m_edca->GetRemainingTxop(m_linkId) -
                     WifiPhy::CalculateTxDuration(GetRtsSize(), rtsTxVector, m_phy->GetPhyBand()),
                 Seconds(0));

    if (m_protectSingleExchange)
    {
        duration = std::min(duration, singleDurationId + m_singleExchangeProtectionSurplus);
    }

    return duration;
}

Time
QosFrameExchangeManager::GetCtsToSelfDurationId(const WifiTxVector& ctsTxVector,
                                                Time txDuration,
                                                Time response) const
{
    NS_LOG_FUNCTION(this << ctsTxVector << txDuration << response);

    const auto singleDurationId =
        FrameExchangeManager::GetCtsToSelfDurationId(ctsTxVector, txDuration, response);

    // TODO This will be removed once no Txop is installed on a QoS station
    if (!m_edca)
    {
        return singleDurationId;
    }

    if (m_edca->GetTxopLimit(m_linkId).IsZero())
    {
        return singleDurationId;
    }

    // under multiple protection settings, if the TXOP limit is not null, Duration/ID
    // is set to cover the remaining TXOP time (Sec. 9.2.5.2 of 802.11-2016).
    // The TXOP holder may exceed the TXOP limit in some situations (Sec. 10.22.2.8
    // of 802.11-2016)
    auto duration =
        std::max(m_edca->GetRemainingTxop(m_linkId) -
                     WifiPhy::CalculateTxDuration(GetCtsSize(), ctsTxVector, m_phy->GetPhyBand()),
                 Seconds(0));

    if (m_protectSingleExchange)
    {
        duration = std::min(duration, singleDurationId + m_singleExchangeProtectionSurplus);
    }

    return duration;
}

void
QosFrameExchangeManager::ForwardMpduDown(Ptr<WifiMpdu> mpdu, WifiTxVector& txVector)
{
    NS_LOG_FUNCTION(this << *mpdu << txVector);

    WifiMacHeader& hdr = mpdu->GetHeader();

    if (hdr.IsQosData() && m_mac->GetTypeOfStation() == STA &&
        (m_setQosQueueSize || hdr.IsQosEosp()))
    {
        uint8_t tid = hdr.GetQosTid();
        hdr.SetQosEosp();
        hdr.SetQosQueueSize(
            m_mac->GetQosTxop(tid)->GetQosQueueSize(tid,
                                                    mpdu->GetOriginal()->GetHeader().GetAddr1()));
    }
    FrameExchangeManager::ForwardMpduDown(mpdu, txVector);
}

void
QosFrameExchangeManager::TransmissionSucceeded()
{
    NS_LOG_DEBUG(this);

    // P-EDCA Success Logic: Reset counters for VO success
    if (m_mac && m_mac->GetPedcaSupported() && m_edca && m_edca->GetAccessCategory() == AC_VO)
    {
        std::clog << "[P-EDCA VO SUCCESS] VO TX succeeded at t="
                  << Simulator::Now().GetMicroSeconds() << "us"
                  << " QSRC=" << m_qsrc << " PSRC=" << +m_psrc;
        if (m_pedcaStage2Active)
        {
            m_pedcaSuccessCount++;  // TRACE: P-EDCA success
            std::clog << " (Stage2-PEDCA)";
        }
        else
        {
            m_edcaVoSuccessCount++;  // TRACE: normal EDCA VO success
            std::clog << " (EDCA)";
        }
        std::clog << " -> Resetting counters" << std::endl;
        
        // Reset QSRC and PSRC on successful TXOP (per spec)
        m_qsrc = 0;
        m_psrc = 0;
        m_pedcaStage2Active = false;
        m_pedcaPending = false;
        ResumePedcaSuspendedACs();
    }

    // TODO This will be removed once no Txop is installed on a QoS station
    if (!m_edca)
    {
        FrameExchangeManager::TransmissionSucceeded();
        return;
    }

    if (m_edca->GetTxopLimit(m_linkId).IsStrictlyPositive() &&
        m_edca->GetRemainingTxop(m_linkId) > m_phy->GetSifs())
    {
        NS_LOG_DEBUG("Schedule another transmission in a SIFS");
        bool (QosFrameExchangeManager::*fp)(Ptr<QosTxop>, Time) =
            &QosFrameExchangeManager::StartTransmission;

        // we are continuing a TXOP, hence the txopDuration parameter is unused
        Simulator::Schedule(m_phy->GetSifs(), fp, this, m_edca, Seconds(0));

        if (m_protectedIfResponded)
        {
            m_protectedStas.merge(m_sentFrameTo);
        }
    }
    else
    {
        NotifyChannelReleased(m_edca);
        m_edca = nullptr;
    }
    m_initialFrame = false;
    m_sentFrameTo.clear();

    if (m_pifsRecovery)
    {
        m_pifsRecovery = false;
        // restore the contention window that was reset when granting access for PIFS recovery
        // m_edcaBackingOff->SetCw(m_edcaBackingOff->GetCw(m_linkId)); // Error: SetCw does not exist
        m_edcaBackingOff = nullptr;
    }
}

void
QosFrameExchangeManager::TransmissionFailed(bool forceCurrentCw)
{
    NS_LOG_FUNCTION(this << forceCurrentCw);

    // P-EDCA Stage 2 Collision Detection
    if (m_pedcaStage2Active && m_edca && m_edca->GetAccessCategory() == AC_VO)
    {
        // Stage 2 failure: Could be RTS collision (two P-EDCA STAs), CTS timeout (AP didn't reply), 
        // or ACK timeout (data frame collided). All are P-EDCA Stage 2 failures.
        // forceCurrentCw=true typically means CTS timeout path
        if (forceCurrentCw)
        {
            m_pedcaFailRtsCtsTimeout++;  // TRACE: CTS timeout (AP didn't reply to RTS)
            std::clog << "[P-EDCA STAGE2 FAIL:CTS_TIMEOUT] TX failed during Stage 2 at t="
                      << Simulator::Now().GetMicroSeconds() << "us"
                      << " PSRC=" << +m_psrc << " QSRC=" << m_qsrc << std::endl;
        }
        else
        {
            m_pedcaFailRtsCollision++;  // TRACE: RTS/data collision in Stage 2
            std::clog << "[P-EDCA STAGE2 FAIL:COLLISION] TX failed during Stage 2 at t="
                      << Simulator::Now().GetMicroSeconds() << "us"
                      << " PSRC=" << +m_psrc << " QSRC=" << m_qsrc << std::endl;
        }
        
        // Stage 2 failure: Apply CW expansion using QSRC formula (per spec 5.3)
        // CW[AC_VO] = min(CWmax[AC_VO], 2^QSRC[AC_VO] × (CWmin[AC_VO] + 1) - 1)
        // Use dot11EDCATable values for CWmin/CWmax
        constexpr uint32_t VO_DEFAULT_CWMIN = 3;
        constexpr uint32_t VO_DEFAULT_CWMAX = 7;
        
        uint16_t qsrcCapped = std::min(m_qsrc, static_cast<uint16_t>(15));
        uint32_t newCw = std::min(VO_DEFAULT_CWMAX, 
            static_cast<uint32_t>((1UL << qsrcCapped) * (VO_DEFAULT_CWMIN + 1) - 1));
        m_edca->SetCw(newCw, m_linkId);
        std::clog << "[P-EDCA CW EXPANSION] CW set by formula: min(" << VO_DEFAULT_CWMAX 
                  << ", 2^" << qsrcCapped << " × (" << VO_DEFAULT_CWMIN << "+1) - 1) = " 
                  << m_edca->GetCw(m_linkId) << std::endl;
        
        // Increment QSRC for this failure (Stage 2 collision is also a retry)
        m_qsrc++;
        std::clog << "[P-EDCA QSRC++] QSRC incremented to " << m_qsrc << " after Stage 2 collision" << std::endl;
        
        // Check if PSRC exhausted (reached dot11PEDCAConsecutiveAttempt)
        if (m_psrc >= PEDCA_CONSECUTIVE_ATTEMPT)
        {
            // Per 802.11bn D1.3 §5.4: when PSRC reaches dot11PEDCAConsecutiveAttempt, the STA
            // shall NOT attempt P-EDCA again until QSRC[AC_VO] is reset (i.e., successful TX).
            // FIX: Do NOT reset PSRC to 0 here. Keeping PSRC >= PEDCA_CONSECUTIVE_ATTEMPT makes
            // psrcOk=false in the trigger check, blocking P-EDCA re-entry.
            // PSRC is only reset to 0 in TransmissionSucceeded() when QSRC is also reset.
            std::clog << "[P-EDCA PSRC EXHAUSTED] PSRC=" << +m_psrc << " >= " << +PEDCA_CONSECUTIVE_ATTEMPT
                      << " - P-EDCA blocked until next QSRC reset (successful TX)" << std::endl;
            ResumePedcaSuspendedACs();
        }
        else
        {
             // P-EDCA Priority Override: Stage 2 failed but PSRC < limit — retry Stage 1.
             // Check all D1.3 start conditions (including AIFSN nonzero, CIDs 7112/11411/11759).
             bool qsrcOk = (m_qsrc >= PEDCA_RETRY_THRESHOLD);
             bool retryLimitOk = (m_mac->GetFrameRetryLimit() > PEDCA_RETRY_THRESHOLD);
             bool aifsn_nonzero = (m_edca->GetAifsn(m_linkId) > 0);

             if (qsrcOk && retryLimitOk && aifsn_nonzero)
             {
                  std::clog << "[P-EDCA PRIORITY] Stage 2 fail, PSRC=" << +m_psrc
                            << " < limit, generating P-EDCA slot boundary backoff for Stage 1 retry" << std::endl;
                  m_edca->GeneratePedcaStage1Backoff(0, m_linkId);
             }
             else
             {
                  ResumePedcaSuspendedACs();
             }
        }
        
        // Return to normal EDCA
        m_pedcaStage2Active = false;
        m_pedcaPending = false;
        std::clog << "[P-EDCA RESTORE] Returning to normal EDCA after Stage 2 collision" << std::endl;
    }
    // P-EDCA Failure Logic: Increment QSRC for VO retries (non-Stage 2)
    // Only execute for P-EDCA enabled STAs
    else if (m_mac->GetPedcaSupported() && m_edca && m_edca->GetAccessCategory() == AC_VO)
    {
        // Increment QSRC (Queue Size Retry Counter) for VO failures
        m_qsrc++;
        std::clog << "[P-EDCA QSRC++] VO transmission failed, QSRC=" << m_qsrc 
                  << " at t=" << Simulator::Now().GetMicroSeconds() << "us" << std::endl;
        
        // If we were in P-EDCA mode, reset the pending flag
        if (m_pedcaPending)
        {
            m_pedcaPending = false;
            std::clog << "[P-EDCA RESTORE] Returning to normal EDCA after failure" << std::endl;
        }

        // P-EDCA Priority Override: If all start conditions are still met after failure,
        // pre-generate DSr=0 backoff so Stage 1 fires at the P-EDCA slot boundary immediately.
        bool qsrcOk = (m_qsrc >= PEDCA_RETRY_THRESHOLD);
        bool psrcOk = (m_psrc < PEDCA_CONSECUTIVE_ATTEMPT);
        bool retryLimitOk = (m_mac->GetFrameRetryLimit() > PEDCA_RETRY_THRESHOLD);
        bool aifsn_nonzero = (m_edca->GetAifsn(m_linkId) > 0);  // CIDs 7112/11411/11759

        if (qsrcOk && psrcOk && retryLimitOk && aifsn_nonzero)
        {
             std::clog << "[P-EDCA PRIORITY] Conditions met (QSRC=" << m_qsrc
                       << "), generating P-EDCA slot boundary backoff (DSr=0) for immediate Stage 1 retry" << std::endl;
             m_edca->GeneratePedcaStage1Backoff(0, m_linkId);
        }
        else
        {
             ResumePedcaSuspendedACs();
        }
    }

    // TODO This will be removed once no Txop is installed on a QoS station
    if (!m_edca)
    {
        FrameExchangeManager::TransmissionFailed(forceCurrentCw);
        return;
    }

    if (m_initialFrame)
    {
        // The backoff procedure shall be invoked by an EDCAF when the transmission
        // of an MPDU in the initial PPDU of a TXOP fails (Sec. 10.22.2.2 of 802.11-2016)
        NS_LOG_DEBUG("TX of the initial frame of a TXOP failed: terminate TXOP");
        if (!forceCurrentCw)
        {
            m_edca->UpdateFailedCw(m_linkId);
        }
        NotifyChannelReleased(m_edca);
        
        // P-EDCA TIMING FIX:
        // If P-EDCA conditions are met, we must NOT use the random backoff generated by NotifyChannelReleased.
        // Instead, we must use DSr (random in [0, CWds], currently CWds=0 -> DSr=0).
        // So we force backoff to 0 slots and AIFS to AIFSN.
        if (m_mac->GetPedcaSupported() && m_edca->GetAccessCategory() == AC_VO && 
            m_qsrc >= PEDCA_RETRY_THRESHOLD && m_psrc < PEDCA_CONSECUTIVE_ATTEMPT)
        {
             // std::clog << "[P-EDCA OVERRIDE] Forcing Backoff=0 for P-EDCA (Stage 1) retry" << std::endl;
             
             Ptr<ChannelAccessManager> cam = m_mac->GetChannelAccessManager(m_linkId);
             if (cam) 
             {
                 Time slot = m_phy->GetSlot();
                 // Target DS-CTS time = LastBusy + SIFS + AIFSN*Slot + DSr*Slot
                 // cam->GetAccessGrantStart() returns LastBusy + SIFS
                 Time accessGrantStart = cam->GetAccessGrantStart(); 
                 
                 // Use current AIFSN (should be 2 for VO/P-EDCA)
                 Time backoffEnd = accessGrantStart + m_edca->GetAifsn(m_linkId) * slot;
                 
                 // Force 0 slots (DSr=0 for now)
                 m_edca->UpdateBackoffSlotsNow(0, backoffEnd, m_linkId);
             }
        }

        m_edca = nullptr;
    }
    else
    {
        // some STA(s) did not respond, they are no longer protected
        for (const auto& address : m_txTimer.GetStasExpectedToRespond())
        {
            NS_LOG_DEBUG(address << " did not respond, hence it is no longer protected");
            m_protectedStas.erase(address);
        }

        NS_ASSERT_MSG(m_edca->GetTxopLimit(m_linkId).IsStrictlyPositive(),
                      "Cannot transmit more than one frame if TXOP Limit is zero");

        // A STA can perform a PIFS recovery or perform a backoff as a response to
        // transmission failure within a TXOP. How it chooses between these two is
        // implementation dependent. (Sec. 10.22.2.2 of 802.11-2016)
        if (m_pifsRecovery)
        {
            // we can continue the TXOP if the carrier sense mechanism indicates that
            // the medium is idle in a PIFS
            NS_LOG_DEBUG("TX of a non-initial frame of a TXOP failed: perform PIFS recovery");
            NS_ASSERT(!m_pifsRecoveryEvent.IsPending());
            m_pifsRecoveryEvent = Simulator::Schedule(m_phy->GetPifs(),
                                                      &QosFrameExchangeManager::PifsRecovery,
                                                      this,
                                                      forceCurrentCw);
        }
        else
        {
            // In order not to terminate (yet) the TXOP, we call the NotifyChannelReleased
            // method of the Txop class, which only generates a new backoff value and
            // requests channel access if needed,
            NS_LOG_DEBUG("TX of a non-initial frame of a TXOP failed: invoke backoff");
            m_edca->Txop::NotifyChannelReleased(m_linkId);
            // CW and QSRC shall be updated in this case (see Section 10.23.2.2 of 802.11-2020)
            if (!forceCurrentCw)
            {
                m_edca->UpdateFailedCw(m_linkId);
            }
            m_edcaBackingOff = m_edca;
            m_edca = nullptr;
        }
    }
    m_initialFrame = false;
    m_sentFrameTo.clear();
    // reset TXNAV because transmission failed
    ResetTxNav();
}

void
QosFrameExchangeManager::ReceivedMacHdr(const WifiMacHeader& macHdr,
                                        const WifiTxVector& txVector,
                                        Time psduDuration)
{
    NS_LOG_FUNCTION(this << macHdr << txVector << psduDuration.As(Time::MS));
    FrameExchangeManager::ReceivedMacHdr(macHdr, txVector, psduDuration);
    SetTxopHolder(macHdr, txVector);
}

void
QosFrameExchangeManager::PreProcessFrame(Ptr<const WifiPsdu> psdu, const WifiTxVector& txVector)
{
    NS_LOG_FUNCTION(this << psdu << txVector);

    // APs store buffer size report of associated stations
    if (m_mac->GetTypeOfStation() == AP && psdu->GetAddr1() == m_self)
    {
        for (const auto& mpdu : *PeekPointer(psdu))
        {
            const WifiMacHeader& hdr = mpdu->GetHeader();

            if (hdr.IsQosData() && hdr.IsQosEosp())
            {
                NS_LOG_DEBUG("Station " << hdr.GetAddr2() << " reported a buffer status of "
                                        << +hdr.GetQosQueueSize()
                                        << " for tid=" << +hdr.GetQosTid());
                m_apMac->SetBufferStatus(hdr.GetQosTid(),
                                         mpdu->GetOriginal()->GetHeader().GetAddr2(),
                                         hdr.GetQosQueueSize());
            }
        }
    }

    // before updating the NAV, check if the NAV counted down to zero. In such a
    // case, clear the saved TXOP holder address.
    ClearTxopHolderIfNeeded();

    FrameExchangeManager::PreProcessFrame(psdu, txVector);
}

void
QosFrameExchangeManager::PostProcessFrame(Ptr<const WifiPsdu> psdu, const WifiTxVector& txVector)
{
    NS_LOG_FUNCTION(this << psdu << txVector);

    SetTxopHolder(psdu->GetHeader(0), txVector);
    FrameExchangeManager::PostProcessFrame(psdu, txVector);
}

void
QosFrameExchangeManager::SetTxopHolder(const WifiMacHeader& hdr, const WifiTxVector& txVector)
{
    NS_LOG_FUNCTION(this << hdr << txVector);
    if (auto txopHolder = FindTxopHolder(hdr, txVector))
    {
        m_txopHolder = *txopHolder;
    }
}

std::optional<Mac48Address>
QosFrameExchangeManager::GetTxopHolder() const
{
    return m_txopHolder;
}

std::optional<Mac48Address>
QosFrameExchangeManager::FindTxopHolder(const WifiMacHeader& hdr, const WifiTxVector& txVector)
{
    NS_LOG_FUNCTION(this << hdr << txVector);

    // A STA shall save the TXOP holder address for the BSS in which it is associated.
    // The TXOP holder address is the MAC address from the Address 2 field of the frame
    // that initiated a frame exchange sequence, except if this is a CTS frame, in which
    // case the TXOP holder address is the Address 1 field. (Sec. 10.23.2.4 of 802.11-2020)
    if ((hdr.IsQosData() || hdr.IsMgt() || hdr.IsRts() || hdr.IsBlockAckReq()) &&
        (hdr.GetAddr1() == m_bssid || hdr.GetAddr2() == m_bssid))
    {
        return hdr.GetAddr2();
    }
    if (hdr.IsCts() && hdr.GetAddr1() == m_bssid)
    {
        return hdr.GetAddr1();
    }
    return std::nullopt;
}

void
QosFrameExchangeManager::ClearTxopHolderIfNeeded()
{
    NS_LOG_FUNCTION(this);
    if (m_navEnd <= Simulator::Now())
    {
        m_txopHolder.reset();
    }
}

void
QosFrameExchangeManager::UpdateNav(const WifiMacHeader& hdr,
                                   const WifiTxVector& txVector,
                                   const Time& surplus)
{
    NS_LOG_FUNCTION(this << hdr << txVector << surplus.As(Time::US));
    if (hdr.IsCfEnd())
    {
        NS_LOG_DEBUG("Received CF-End, resetting NAV");
        NavResetTimeout();
        return;
    }

    FrameExchangeManager::UpdateNav(hdr, txVector, surplus);
}

void
QosFrameExchangeManager::NavResetTimeout()
{
    NS_LOG_FUNCTION(this);
    FrameExchangeManager::NavResetTimeout();
    ClearTxopHolderIfNeeded();
}

void
QosFrameExchangeManager::ReceiveMpdu(Ptr<const WifiMpdu> mpdu,
                                     RxSignalInfo rxSignalInfo,
                                     const WifiTxVector& txVector,
                                     bool inAmpdu)
{
    NS_LOG_FUNCTION(this << *mpdu << rxSignalInfo << txVector << inAmpdu);

    // The received MPDU is either broadcast or addressed to this station
    NS_ASSERT(mpdu->GetHeader().GetAddr1().IsGroup() || mpdu->GetHeader().GetAddr1() == m_self);

    double rxSnr = rxSignalInfo.snr;
    const WifiMacHeader& hdr = mpdu->GetHeader();

    if (hdr.IsRts())
    {
        NS_ABORT_MSG_IF(inAmpdu, "Received RTS as part of an A-MPDU");

        // If a non-VHT STA receives an RTS frame with the RA address matching the
        // MAC address of the STA and the MAC address in the TA field in the RTS
        // frame matches the saved TXOP holder address, then the STA shall send the
        // CTS frame after SIFS, without regard for, and without resetting, its NAV.
        // (sec. 10.22.2.4 of 802.11-2016)
        if (hdr.GetAddr2() == m_txopHolder || VirtualCsMediumIdle())
        {
            NS_LOG_DEBUG("Received RTS from=" << hdr.GetAddr2() << ", schedule CTS");
            m_sendCtsEvent = Simulator::Schedule(m_phy->GetSifs(),
                                                 &QosFrameExchangeManager::SendCtsAfterRts,
                                                 this,
                                                 hdr,
                                                 txVector,
                                                 rxSnr);
        }
        else
        {
            NS_LOG_DEBUG("Received RTS from=" << hdr.GetAddr2() << ", cannot schedule CTS");
        }
        return;
    }

    if (hdr.IsQosData())
    {
        if (hdr.GetAddr1() == m_self && hdr.GetQosAckPolicy() == WifiMacHeader::NORMAL_ACK)
        {
            NS_LOG_DEBUG("Received " << hdr.GetTypeString() << " from=" << hdr.GetAddr2()
                                     << ", schedule ACK");
            Simulator::Schedule(m_phy->GetSifs(),
                                &QosFrameExchangeManager::SendNormalAck,
                                this,
                                hdr,
                                txVector,
                                rxSnr);
        }

        // Forward up the frame
        m_rxMiddle->Receive(mpdu, m_linkId);

        // the received data frame has been processed
        return;
    }

    return FrameExchangeManager::ReceiveMpdu(mpdu, rxSignalInfo, txVector, inAmpdu);
}

void
QosFrameExchangeManager::PedcaPhyTxEndCallback(Ptr<const Packet> pkt)
{
    // Guard: only fire once (m_pedcaTxEndPending is true only while we are waiting for our DS-CTS)
    if (!m_pedcaTxEndPending)
    {
        return;
    }
    // Disarm: subsequent PhyTxEnd firings (from data/RTS/etc.) will return immediately
    m_pedcaTxEndPending = false;

    // Simulator::Now() == m_pedcaCtsTxEnd exactly (fired from WifiPhy::TxDone)
    uint32_t pre = m_pedcaEdca->GetBackoffSlots(m_linkId);
    std::clog << "[P-EDCA STAGE2 ENTER] t=" << Simulator::Now().GetMicroSeconds()
              << "us (= CTS TX end) pre-regen backoff=" << pre
              << " slots (CW=" << m_pedcaEdca->GetCw(m_linkId) << ")" << std::endl;

    m_stage2EntryCount++;
    m_pedcaPending   = true;
    m_pedcaCtsTxEnd  = Simulator::Now();  // exact TX end, no polling offset

    // Defer Stage 2 entry out of the WifiPhy::TxDone call stack to avoid re-entrant
    // PHY/MAC state machine calls.  ScheduleNow preserves the exact simulation time.
    Simulator::ScheduleNow(&QosFrameExchangeManager::PedcaStage2Enter, this);
}

void
QosFrameExchangeManager::PedcaStage2Enter()
{
    m_pedcaEdca->SetPedcaBypassBackoff(true, m_linkId);
    NotifyChannelReleased(m_pedcaEdca);
}

void
QosFrameExchangeManager::ResumePedcaSuspendedACs()
{
    if (m_mac && m_mac->GetPedcaSupported())
    {
        for (const auto& ac : {AC_BE, AC_BK, AC_VI})
        {
            Ptr<QosTxop> other = m_mac->GetQosTxop(ac);
            if (other && other->IsPedcaSuspended(m_linkId))
            {
                other->SetPedcaSuspended(false, m_linkId);
                // std::clog << "[P-EDCA RESUME] Resumed " << ac << std::endl;
            }
        }
    }
}

} // namespace ns3
