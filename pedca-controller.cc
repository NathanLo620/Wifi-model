/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * AP-side adaptive P-EDCA controller.
 */

#include "pedca-controller.h"

#include "ap-wifi-mac.h"
#include "qos-frame-exchange-manager.h"
#include "wifi-net-device.h"
#include "wifi-phy.h"
#include "wifi-phy-state-helper.h"
#include "wifi-ppdu.h"
#include "wifi-psdu.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("PedcaController");

NS_OBJECT_ENSURE_REGISTERED(PedcaController);

/// TIDs belonging to AC_VO, the access category P-EDCA acts on.
static constexpr std::array<uint8_t, 2> PEDCA_VO_TIDS = {6, 7};

TypeId
PedcaController::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::PedcaController")
            .SetParent<Object>()
            .SetGroupName("Wifi")
            .AddConstructor<PedcaController>()
            .AddAttribute("Policy",
                          "Which rule set chooses the parameters: kdriven (fitted to the "
                          "2026-08-01 sweeps), loaddriven (the original airtime integrator), "
                          "burstadaptive (population prior plus filtered overload guard), v2 "
                          "(the original collision/busy/per-station rules), or fixed (push one "
                          "constant triple)",
                          EnumValue(PedcaPolicy::KDRIVEN),
                          MakeEnumAccessor<PedcaPolicy>(&PedcaController::m_policy),
                          MakeEnumChecker(PedcaPolicy::KDRIVEN,
                                          "kdriven",
                                          PedcaPolicy::LOADDRIVEN,
                                          "loaddriven",
                                          PedcaPolicy::BURSTADAPTIVE,
                                          "burstadaptive",
                                          PedcaPolicy::V2,
                                          "v2",
                                          PedcaPolicy::FIXED,
                                          "fixed"))
            .AddAttribute("OverheadHigh",
                          "Fraction of airtime spent on P-EDCA Stage-1 above which the "
                          "load-driven policy raises QSRC",
                          DoubleValue(0.06),
                          MakeDoubleAccessor(&PedcaController::m_overheadHigh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("OverheadLow",
                          "Stage-1 airtime below which the load-driven policy is willing to "
                          "lower QSRC, provided stations are short of delay budget",
                          DoubleValue(0.03),
                          MakeDoubleAccessor(&PedcaController::m_overheadLow),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("UrgencyHigh",
                          "Fraction of arriving AC_VO frames carrying the low latency "
                          "indication above which stations count as short of delay budget",
                          DoubleValue(0.15),
                          MakeDoubleAccessor(&PedcaController::m_urgencyHigh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("UrgencyLow",
                          "LLI ratio below which the load-driven policy reclaims medium time "
                          "by raising QSRC, because nothing is short of delay budget",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&PedcaController::m_urgencyLow),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("QsrcFloor",
                          "QSRC never goes below this. 0 lets every transmission attempt open "
                          "a Stage 1 and measures worst or near-worst at every load.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&PedcaController::m_qsrcFloor),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("BurstCost",
                          "Medium time one Stage-1 burst occupies: two DS-CTS a SIFS apart "
                          "plus the 81 us reservation",
                          TimeValue(MicroSeconds(185)),
                          MakeTimeAccessor(&PedcaController::m_burstCost),
                          MakeTimeChecker())
            .AddAttribute("BurstEwmaAlpha",
                          "Weight of the newest Stage-1 airtime, urgency and DS-CTS loss "
                          "samples in the burst-adaptive policy",
                          DoubleValue(0.25),
                          MakeDoubleAccessor(&PedcaController::m_burstEwmaAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BurstOverheadHigh",
                          "Filtered Stage-1 airtime above which burst-adaptive votes to raise "
                          "QSRC. This is separate from the legacy load-driven threshold.",
                          DoubleValue(0.12),
                          MakeDoubleAccessor(&PedcaController::m_burstOverheadHigh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BurstOverheadLow",
                          "Filtered Stage-1 airtime below which high urgency may lower QSRC "
                          "from the q5 overload state to its population prior",
                          DoubleValue(0.09),
                          MakeDoubleAccessor(&PedcaController::m_burstOverheadLow),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BurstCollisionHigh",
                          "Filtered DS-CTS burst-loss fraction above which burst-adaptive "
                          "votes to raise QSRC",
                          DoubleValue(0.50),
                          MakeDoubleAccessor(&PedcaController::m_burstCollisionHigh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BurstMinBursts",
                          "Minimum DS-CTS bursts in one period before its loss ratio may vote "
                          "to change QSRC",
                          UintegerValue(20),
                          MakeUintegerAccessor(&PedcaController::m_burstMinBursts),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("BurstUpHoldPeriods",
                          "Consecutive overload periods before burst-adaptive raises QSRC",
                          UintegerValue(2),
                          MakeUintegerAccessor(&PedcaController::m_burstUpHoldPeriods),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("BurstDownHoldPeriods",
                          "Consecutive safe-but-urgent periods before burst-adaptive lowers "
                          "QSRC toward its population prior",
                          UintegerValue(5),
                          MakeUintegerAccessor(&PedcaController::m_burstDownHoldPeriods),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("BurstSmallKMax",
                          "Largest estimated P-EDCA population assigned BurstQsrcSmall",
                          UintegerValue(8),
                          MakeUintegerAccessor(&PedcaController::m_burstSmallKMax),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("BurstMediumKMax",
                          "Largest estimated P-EDCA population assigned BurstQsrcMedium",
                          UintegerValue(22),
                          MakeUintegerAccessor(&PedcaController::m_burstMediumKMax),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("BurstQsrcSmall",
                          "QSRC prior for a small bursty P-EDCA population",
                          UintegerValue(3),
                          MakeUintegerAccessor(&PedcaController::m_burstQsrcSmall),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("BurstQsrcMedium",
                          "QSRC prior for a medium bursty P-EDCA population",
                          UintegerValue(4),
                          MakeUintegerAccessor(&PedcaController::m_burstQsrcMedium),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("BurstQsrcLarge",
                          "QSRC prior for a large bursty P-EDCA population",
                          UintegerValue(4),
                          MakeUintegerAccessor(&PedcaController::m_burstQsrcLarge),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("BurstQsrcFloor",
                          "Hard QSRC floor of burst-adaptive. The default excludes QSRC 0 "
                          "and the collision-conditioned QSRC 1 operating point.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&PedcaController::m_burstQsrcFloor),
                          MakeUintegerChecker<uint8_t>(2, PEDCA_QSRC_MAX))
            .AddAttribute("KOverride",
                          "Use this value as the number of P-EDCA stations instead of estimating "
                          "it. Negative means estimate. Set it to the true count to measure what "
                          "the policy is worth independently of the estimator.",
                          IntegerValue(-1),
                          MakeIntegerAccessor(&PedcaController::m_kOverride),
                          MakeIntegerChecker<int32_t>(-1, 2007))
            .AddAttribute("QsrcIntercept",
                          "Intercept of the k-driven QSRC law, QSRC = round(intercept + slope*k)",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&PedcaController::m_qsrcIntercept),
                          MakeDoubleChecker<double>())
            .AddAttribute("QsrcSlope",
                          "Slope per P-EDCA station of the k-driven QSRC law",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&PedcaController::m_qsrcSlope),
                          MakeDoubleChecker<double>())
            .AddAttribute("CwdsKDriven",
                          "CWds advertised by the k-driven policy. The sweeps make this a "
                          "don't-care, so it is held constant rather than controlled.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&PedcaController::m_cwdsKDriven),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_CWDS_MAX))
            .AddAttribute("Psrc3MaxK",
                          "Largest estimated k that still receives PSRC 3",
                          UintegerValue(10),
                          MakeUintegerAccessor(&PedcaController::m_psrc3MaxK),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("Psrc2MaxK",
                          "Largest estimated k that still receives PSRC 2",
                          UintegerValue(22),
                          MakeUintegerAccessor(&PedcaController::m_psrc2MaxK),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("Period",
                          "How often the controller re-chooses the P-EDCA parameter table",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&PedcaController::m_period),
                          MakeTimeChecker())
            .AddAttribute("DsCtsBurstGap",
                          "Inter-arrival gap above which a DS-CTS observation is treated as a new "
                          "Stage-1 burst rather than the continuation of the current one",
                          TimeValue(MicroSeconds(100)),
                          MakeTimeAccessor(&PedcaController::m_dsCtsBurstGap),
                          MakeTimeChecker())
            .AddAttribute("BusyHigh",
                          "Channel busy fraction above which every station is pushed to the "
                          "conservative corner",
                          DoubleValue(0.85),
                          MakeDoubleAccessor(&PedcaController::m_busyHigh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("CollHigh",
                          "DS-CTS collision rate above which CWds jumps to CwdsMax. Calibrated "
                          "against the single DS-CTS sender; re-measure before trusting it with "
                          "dual DS-CTS, where a burst survives losing its first frame.",
                          DoubleValue(0.90),
                          MakeDoubleAccessor(&PedcaController::m_collHigh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("CollLow",
                          "DS-CTS collision rate below which CWds drops back to zero",
                          DoubleValue(0.30),
                          MakeDoubleAccessor(&PedcaController::m_collLow),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("LliHigh",
                          "Number of low-latency indications from one station in one period that "
                          "marks it as running out of delay budget",
                          UintegerValue(5),
                          MakeUintegerAccessor(&PedcaController::m_lliHigh),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("BsrPerStaHigh",
                          "Buffer status of one station, in units of 256 octets, that marks it as "
                          "backlogged",
                          DoubleValue(8.0),
                          MakeDoubleAccessor(&PedcaController::m_bsrPerStaHigh),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("CwdsMax",
                          "Largest CWds the controller may choose",
                          UintegerValue(PEDCA_CWDS_MAX),
                          MakeUintegerAccessor(&PedcaController::m_cwdsMax),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_CWDS_MAX))
            .AddAttribute("QsrcAggressive",
                          "QSRC threshold given to a station under delay or queue pressure",
                          UintegerValue(1),
                          MakeUintegerAccessor(&PedcaController::m_qsrcAggressive),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("PsrcAggressive",
                          "PSRC limit given to a station under delay or queue pressure. The dual "
                          "DS-CTS sender has not been re-calibrated for this corner: a value of 3 "
                          "was measured harmful there at high P-EDCA penetration.",
                          UintegerValue(3),
                          MakeUintegerAccessor(&PedcaController::m_psrcAggressive),
                          MakeUintegerChecker<uint8_t>(PEDCA_PSRC_MIN, PEDCA_PSRC_MAX))
            .AddAttribute("QsrcConservative",
                          "QSRC threshold given to every station while the congestion gate is on",
                          UintegerValue(PEDCA_QSRC_MAX),
                          MakeUintegerAccessor(&PedcaController::m_qsrcConservative),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("PsrcConservative",
                          "PSRC limit given to every station while the congestion gate is on",
                          UintegerValue(PEDCA_PSRC_MIN),
                          MakeUintegerAccessor(&PedcaController::m_psrcConservative),
                          MakeUintegerChecker<uint8_t>(PEDCA_PSRC_MIN, PEDCA_PSRC_MAX))
            .AddAttribute("UseBurstCollRate",
                          "Base the CWds decision on the fraction of DS-CTS bursts of which no "
                          "frame decoded, rather than on the fraction of individual DS-CTS frames "
                          "lost. With dual DS-CTS the latter reads about 0.5 for a burst that "
                          "worked exactly as designed.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&PedcaController::m_useBurstCollRate),
                          MakeBooleanChecker())
            .AddTraceSource("ControlStep",
                            "A control step was taken",
                            MakeTraceSourceAccessor(&PedcaController::m_controlStepTrace),
                            "ns3::PedcaController::ControlStepCallback");
    return tid;
}

PedcaController::PedcaController()
{
    NS_LOG_FUNCTION(this);
}

PedcaController::~PedcaController()
{
    NS_LOG_FUNCTION(this);
}

void
PedcaController::DoDispose()
{
    m_stepEvent.Cancel();
    m_apMac = nullptr;
    m_phy = nullptr;
    m_fem = nullptr;
    Object::DoDispose();
}

void
PedcaController::Setup(Ptr<WifiNetDevice> apDevice)
{
    NS_LOG_FUNCTION(this << apDevice);
    NS_ABORT_MSG_IF(!apDevice, "PedcaController::Setup needs an AP device");

    m_apMac = DynamicCast<ApWifiMac>(apDevice->GetMac());
    NS_ABORT_MSG_IF(!m_apMac, "PedcaController must be attached to an AP");
    NS_ABORT_MSG_IF(m_apMac->GetNLinks() != 1, "PedcaController supports single-link APs only");
    NS_ABORT_MSG_IF(!m_apMac->GetPedcaControl(),
                    "PedcaController needs ApWifiMac::PedcaControl enabled, otherwise the "
                    "parameter table is never advertised");

    m_phy = apDevice->GetPhy();
    NS_ABORT_MSG_IF(!m_phy, "AP device has no PHY");

    m_fem = DynamicCast<QosFrameExchangeManager>(m_apMac->GetFrameExchangeManager(0));
    NS_ABORT_MSG_IF(!m_fem, "AP link 0 has no QoS frame exchange manager");

    // The station applying a QSRC threshold at or above its frame retry limit would make the
    // sender raise that limit permanently, silently changing retransmission behaviour.
    NS_ABORT_MSG_IF(m_qsrcConservative >= m_apMac->GetFrameRetryLimit() ||
                        m_qsrcAggressive >= m_apMac->GetFrameRetryLimit(),
                    "QSRC corners must stay below WifiMac::FrameRetryLimit ("
                        << m_apMac->GetFrameRetryLimit() << ")");

    m_phy->GetState()->TraceConnectWithoutContext(
        "State",
        MakeCallback(&PedcaController::PhyStateCb, this));
    m_phy->TraceConnectWithoutContext("PhyRxPpduDrop",
                                      MakeCallback(&PedcaController::RxDropCb, this));
    m_fem->TraceConnectWithoutContext("DsCtsRx", MakeCallback(&PedcaController::DsCtsRxCb, this));
}

void
PedcaController::SetInitialTheta(const PedcaTheta& theta)
{
    m_initialTheta = theta;
    m_fixedTheta = theta;
}

uint32_t
PedcaController::EstimatePedcaStaCount()
{
    // The LLI bit is set only by a P-EDCA station, so one marked frame is proof. Sticky:
    // capability does not come and go between periods.
    for (const auto& [aid, addr] : m_apMac->GetStaList(0))
    {
        if (m_fem->GetLliRxCount(addr) > 0)
        {
            m_provenPedcaStas.insert(addr);
        }
    }
    return static_cast<uint32_t>(m_provenPedcaStas.size());
}

PedcaTheta
PedcaController::KDrivenTheta(uint32_t k) const
{
    PedcaTheta theta;
    theta.cwds = m_cwdsKDriven;

    const auto raw = std::lround(m_qsrcIntercept + m_qsrcSlope * static_cast<double>(k));
    theta.qsrcThreshold =
        static_cast<uint8_t>(std::clamp<long>(raw, 0, static_cast<long>(PEDCA_QSRC_MAX)));

    theta.psrcLimit = (k <= m_psrc3MaxK) ? 3 : ((k <= m_psrc2MaxK) ? 2 : 1);
    return theta;
}

uint8_t
PedcaController::BurstQsrcPrior(uint32_t k) const
{
    // k=0 means the LLI-based capability estimator has not observed a P-EDCA station yet;
    // stay at the safe floor until it has evidence instead of pretending the BSS is small.
    if (k == 0)
    {
        return m_burstQsrcFloor;
    }
    if (k <= m_burstSmallKMax)
    {
        return std::max(m_burstQsrcFloor, m_burstQsrcSmall);
    }
    if (k <= m_burstMediumKMax)
    {
        return std::max(m_burstQsrcFloor, m_burstQsrcMedium);
    }
    return std::max(m_burstQsrcFloor, m_burstQsrcLarge);
}

void
PedcaController::Start(Time when)
{
    NS_LOG_FUNCTION(this << when.As(Time::S));
    NS_ABORT_MSG_IF(!m_apMac, "PedcaController::Start called before Setup");

    Simulator::Schedule(when, [this]() {
        m_cwds = std::min<uint8_t>(m_initialTheta.cwds, m_cwdsMax);
        m_burstAdaptiveQsrc = std::clamp(std::max(m_initialTheta.qsrcThreshold,
                                                  m_burstQsrcFloor),
                                         m_burstQsrcFloor,
                                         static_cast<uint8_t>(PEDCA_QSRC_MAX));
        m_lastBurstQsrcPrior = m_burstQsrcFloor;
        m_burstEwmaReady = false;
        m_burstCollisionEwmaReady = false;
        m_overheadEwma = 0.0;
        m_urgencyEwma = 0.0;
        m_collisionEwma = 0.0;
        m_burstUpVotes = 0;
        m_burstDownVotes = 0;
        ResetObservations();

        // The BSS-wide deltas must start at the same boundary as the PHY observations.
        // Otherwise the first control period includes all feedback accumulated in warmup.
        m_lastVoRx = m_fem->GetVoRxCount();
        m_lastLliTotal = m_fem->GetLliRxCount();

        // Snapshot the per-station LLI counters so that the first step sees a delta over the
        // period rather than everything accumulated during warmup.
        m_lastLliByAddr.clear();
        for (const auto& [aid, addr] : m_apMac->GetStaList(0))
        {
            m_lastLliByAddr[addr] = m_fem->GetLliRxCount(addr);
        }

        m_stepEvent = Simulator::Schedule(m_period, &PedcaController::Step, this);
    });
}

void
PedcaController::Stop()
{
    m_stepEvent.Cancel();
}

uint32_t
PedcaController::GetStepCount() const
{
    return m_stepCount;
}

void
PedcaController::ResetObservations()
{
    m_busyAccum = Time(0);
    m_framesDecoded = 0;
    m_framesDropped = 0;
    m_bursts = 0;
    m_burstsLost = 0;
    m_curBurstFrames = 0;
    m_curBurstDecoded = 0;
    m_lastDsCtsEvent = Time(0);
}

void
PedcaController::PhyStateCb(Time start, Time duration, WifiPhyState state)
{
    switch (state)
    {
    case WifiPhyState::TX:
    case WifiPhyState::RX:
    case WifiPhyState::CCA_BUSY:
    case WifiPhyState::SWITCHING:
        m_busyAccum += duration;
        break;
    default:
        break;
    }
}

void
PedcaController::RxDropCb(Ptr<const WifiPpdu> ppdu, WifiPhyRxfailureReason reason)
{
    // A DS-CTS missed because the AP itself was transmitting is self-inflicted, not evidence
    // of a collision on air.
    if (reason == TXING || !ppdu || !ppdu->GetPsdu())
    {
        return;
    }

    const auto& hdr = ppdu->GetPsdu()->GetHeader(0);
    if (hdr.IsCts() && hdr.GetAddr1() == FrameExchangeManager::GetDsCtsAddress())
    {
        FeedDsCtsEvent(false);
    }
}

void
PedcaController::DsCtsRxCb(Time now)
{
    FeedDsCtsEvent(true);
}

void
PedcaController::FeedDsCtsEvent(bool decoded)
{
    const auto now = Simulator::Now();
    if (m_curBurstFrames > 0 && (now - m_lastDsCtsEvent) > m_dsCtsBurstGap)
    {
        CloseBurst();
    }

    m_curBurstFrames++;
    if (decoded)
    {
        m_curBurstDecoded++;
        m_framesDecoded++;
    }
    else
    {
        m_framesDropped++;
    }
    m_lastDsCtsEvent = now;
}

void
PedcaController::CloseBurst()
{
    if (m_curBurstFrames == 0)
    {
        return;
    }
    m_bursts++;
    if (m_curBurstDecoded == 0)
    {
        m_burstsLost++;
    }
    m_curBurstFrames = 0;
    m_curBurstDecoded = 0;
}

void
PedcaController::Step()
{
    NS_LOG_FUNCTION(this);

    CloseBurst();

    // ---- BSS-wide observations of the elapsed period ----
    const auto periodUs = static_cast<double>(m_period.GetMicroSeconds());
    const auto busyFrac =
        std::min(1.0, static_cast<double>(m_busyAccum.GetMicroSeconds()) / periodUs);

    const auto frames = m_framesDecoded + m_framesDropped;
    const auto collRateFrame =
        (frames > 0) ? static_cast<double>(m_framesDropped) / frames : 0.0;
    const auto collRateBurst =
        (m_bursts > 0) ? static_cast<double>(m_burstsLost) / m_bursts : 0.0;
    const auto collRate = m_useBurstCollRate ? collRateBurst : collRateFrame;
    const bool dsActive = (m_bursts > 0) || (frames > 0);

    const auto dsCtsFrames = frames;
    const auto dsCtsBursts = m_bursts;

    // ---- How many stations are actually using P-EDCA ----
    const auto kHat = (m_kOverride >= 0) ? static_cast<uint32_t>(m_kOverride)
                                         : EstimatePedcaStaCount();

    // ---- Runtime load signals, both independent of how many stations are configured ----
    // What share of the medium Stage-1 is consuming right now.
    const auto overhead =
        std::min(1.0,
                 static_cast<double>(m_bursts) *
                     static_cast<double>(m_burstCost.GetMicroSeconds()) / periodUs);
    // What share of the voice frames arriving are already close to their delay bound.
    const auto voRxTotal = m_fem->GetVoRxCount();
    const auto voRx = voRxTotal - m_lastVoRx;
    m_lastVoRx = voRxTotal;
    const auto lliRxTotal = m_fem->GetLliRxCount();
    const auto lliRx = lliRxTotal - m_lastLliTotal;
    m_lastLliTotal = lliRxTotal;
    const auto urgency = (voRx > 0) ? static_cast<double>(lliRx) / voRx : 0.0;

    if (m_policy == PedcaPolicy::LOADDRIVEN)
    {
        // One step per period in each direction, so the loop integrates rather than chasing
        // a single noisy observation to an extreme.
        //
        // Raising is driven by Stage-1 airtime alone: once P-EDCA is consuming too much of
        // the medium it is competing with the very traffic it is meant to protect. Lowering
        // additionally requires that something is actually short of delay budget. That
        // asymmetry is what keeps the loop out of trouble at light load, where Stage-1
        // airtime is near zero but nothing needs help: an airtime-only rule would read the
        // idle medium as headroom, drop QSRC to the floor and start firing P-EDCA on sparse
        // traffic, which measures +45% on the tail. Regulating urgency directly instead was
        // tried and is worse at low contention (see the notes in the commit).
        if (overhead > m_overheadHigh)
        {
            m_qsrcLoad = std::min<uint8_t>(m_qsrcLoad + 1, PEDCA_QSRC_MAX);
        }
        else if (urgency > m_urgencyHigh && overhead < m_overheadLow)
        {
            m_qsrcLoad = (m_qsrcLoad > m_qsrcFloor) ? static_cast<uint8_t>(m_qsrcLoad - 1)
                                                    : m_qsrcFloor;
        }
        m_qsrcLoad = std::clamp(m_qsrcLoad, m_qsrcFloor, static_cast<uint8_t>(PEDCA_QSRC_MAX));
    }
    else if (m_policy == PedcaPolicy::BURSTADAPTIVE)
    {
        // The population prior represents the peak number of simultaneously eligible
        // stations. It rises immediately when the sticky capability estimate enters a new
        // bucket. The exact On/Off sweep puts q4 and q5 very close but selects q4 for the
        // best complete triple; q5 is therefore reserved for measured overload instead of
        // being entered merely because the population is large.
        const auto qsrcPrior = BurstQsrcPrior(kHat);
        if (qsrcPrior > m_lastBurstQsrcPrior)
        {
            m_burstAdaptiveQsrc = std::max(m_burstAdaptiveQsrc, qsrcPrior);
        }
        m_lastBurstQsrcPrior = qsrcPrior;
        const auto qsrcLower = qsrcPrior;

        if (!m_burstEwmaReady)
        {
            m_overheadEwma = overhead;
            m_urgencyEwma = urgency;
            m_burstEwmaReady = true;
        }
        else
        {
            const auto a = m_burstEwmaAlpha;
            m_overheadEwma = a * overhead + (1.0 - a) * m_overheadEwma;
            m_urgencyEwma = a * urgency + (1.0 - a) * m_urgencyEwma;
        }

        // A loss ratio based on a handful of bursts is too quantised to control from (one
        // missed burst can mean 25-50%).  Only fold adequately sampled periods into this
        // EWMA, and only let such a period cast an overload vote.
        const bool collisionSampleValid = m_bursts >= m_burstMinBursts;
        if (collisionSampleValid)
        {
            if (!m_burstCollisionEwmaReady)
            {
                m_collisionEwma = collRate;
                m_burstCollisionEwmaReady = true;
            }
            else
            {
                const auto a = m_burstEwmaAlpha;
                m_collisionEwma = a * collRate + (1.0 - a) * m_collisionEwma;
            }
        }

        const bool overloaded = (m_overheadEwma > m_burstOverheadHigh) ||
                                (collisionSampleValid &&
                                 m_collisionEwma > m_burstCollisionHigh);
        const bool safeButUrgent = (m_urgencyEwma > m_urgencyHigh) &&
                                   (m_overheadEwma < m_burstOverheadLow) &&
                                   (!m_burstCollisionEwmaReady ||
                                    m_collisionEwma < m_burstCollisionHigh);

        if (overloaded && m_burstAdaptiveQsrc < PEDCA_QSRC_MAX)
        {
            m_burstUpVotes++;
            m_burstDownVotes = 0;
            if (m_burstUpVotes >= m_burstUpHoldPeriods)
            {
                m_burstAdaptiveQsrc++;
                m_burstUpVotes = 0;
            }
        }
        else if (safeButUrgent && m_burstAdaptiveQsrc > qsrcLower)
        {
            m_burstDownVotes++;
            m_burstUpVotes = 0;
            if (m_burstDownVotes >= m_burstDownHoldPeriods)
            {
                m_burstAdaptiveQsrc--;
                m_burstDownVotes = 0;
            }
        }
        else
        {
            m_burstUpVotes = 0;
            m_burstDownVotes = 0;
        }

        m_burstAdaptiveQsrc = std::clamp(m_burstAdaptiveQsrc,
                                         qsrcLower,
                                         static_cast<uint8_t>(PEDCA_QSRC_MAX));
    }

    // ---- BSS-wide CWds: jump straight to the target, no step limit ----
    if (dsActive && collRate > m_collHigh)
    {
        m_cwds = m_cwdsMax;
    }
    else if (dsActive && collRate < m_collLow)
    {
        m_cwds = 0;
    }

    // ---- Global conservative gate: congestion guard, or collisions that CWds cannot fix ----
    const bool globalConservativeGate =
        (busyFrac > m_busyHigh && dsActive) ||
        (dsActive && collRate > m_collHigh && m_cwds >= m_cwdsMax);

    // ---- Per-station decisions ----
    uint32_t bsrSum = 0;
    uint32_t lliTotal = 0;
    uint32_t nAggressive = 0;
    uint32_t nConservative = 0;
    uint32_t nUnchanged = 0;
    bool anyChanged = false;
    std::map<uint16_t, PedcaTheta> newTheta;

    // Only V2 differentiates per station; the others advertise one triple to everyone.
    PedcaTheta bssWide = m_fixedTheta;
    if (m_policy == PedcaPolicy::KDRIVEN)
    {
        bssWide = KDrivenTheta(kHat);
    }
    else if (m_policy == PedcaPolicy::LOADDRIVEN)
    {
        // CWds is a measured don't-care and PSRC 3 is the best single choice once QSRC is
        // free to move, so QSRC carries the whole loop.
        bssWide = PedcaTheta{m_cwdsKDriven, m_qsrcLoad, 3};
    }
    else if (m_policy == PedcaPolicy::BURSTADAPTIVE)
    {
        // The On/Off sweep consistently favours PSRC 3; CWds is a weak dimension, so one
        // slot of randomisation is retained as cheap protection against Stage-1 ties.
        bssWide = PedcaTheta{m_cwdsKDriven, m_burstAdaptiveQsrc, 3};
    }

    for (const auto& [aid, addr] : m_apMac->GetStaList(0))
    {
        uint32_t bsrSta = 0;
        for (const auto tid : PEDCA_VO_TIDS)
        {
            // 255 means the station never reported, or its report has expired: raise
            // ApWifiMac::BsrLifetime above the control period if this is always the case.
            if (const auto queueSize = m_apMac->GetBufferStatus(tid, addr); queueSize != 255)
            {
                bsrSta += queueSize;
            }
        }
        bsrSum += bsrSta;

        const auto lliCumulative = m_fem->GetLliRxCount(addr);
        const auto lliIt = m_lastLliByAddr.find(addr);
        const uint32_t lliSta =
            (lliIt == m_lastLliByAddr.end()) ? 0 : (lliCumulative - lliIt->second);
        m_lastLliByAddr[addr] = lliCumulative;
        lliTotal += lliSta;

        const auto previous = m_apMac->GetPedcaParametersFor(aid);
        auto next = previous;

        switch (m_policy)
        {
        case PedcaPolicy::KDRIVEN:
        case PedcaPolicy::LOADDRIVEN:
        case PedcaPolicy::BURSTADAPTIVE:
        case PedcaPolicy::FIXED:
            next = bssWide;
            nUnchanged++;
            break;

        case PedcaPolicy::V2:
            next.cwds = m_cwds; // CWds is a shared Stage-1 resource, not a per-station property
            if (globalConservativeGate)
            {
                next.qsrcThreshold = m_qsrcConservative;
                next.psrcLimit = m_psrcConservative;
                nConservative++;
            }
            else if (lliSta >= m_lliHigh || static_cast<double>(bsrSta) >= m_bsrPerStaHigh)
            {
                next.qsrcThreshold = m_qsrcAggressive;
                next.psrcLimit = m_psrcAggressive;
                nAggressive++;
            }
            else
            {
                // Nothing worth reacting to: leave this station where it was.
                nUnchanged++;
            }
            break;
        }

        if (!(next == previous))
        {
            anyChanged = true;
        }
        newTheta[aid] = next;
    }

    m_apMac->SetPedcaParametersBulk(newTheta);
    m_stepCount++;

    const auto shownCwds = (m_policy == PedcaPolicy::V2) ? m_cwds : bssWide.cwds;
    const auto shownQsrc = (m_policy == PedcaPolicy::V2) ? 0 : bssWide.qsrcThreshold;
    const auto shownPsrc = (m_policy == PedcaPolicy::V2) ? 0 : bssWide.psrcLimit;

    if (anyChanged)
    {
        std::clog << "[P-EDCA CTRL] t=" << Simulator::Now().GetMicroSeconds()
                  << "us kHat=" << kHat << " overhead=" << overhead
                  << " urgency=" << urgency
                  << " cwds=" << +shownCwds << " qsrc=" << +shownQsrc
                  << " psrc=" << +shownPsrc << " aggr=" << nAggressive
                  << " cons=" << nConservative << " keep=" << nUnchanged
                  << " busyFrac=" << busyFrac << " collRate=" << collRate
                  << " collRateFrame=" << collRateFrame << " dsCtsFrames=" << dsCtsFrames
                  << " dsCtsBursts=" << dsCtsBursts << " bsrSum=" << bsrSum
                  << " lli=" << lliTotal << std::endl;
    }

    m_controlStepTrace(Simulator::Now(),
                       kHat,
                       shownCwds,
                       shownQsrc,
                       shownPsrc,
                       nAggressive,
                       nConservative,
                       nUnchanged,
                       busyFrac,
                       collRate,
                       collRateFrame,
                       dsCtsFrames,
                       dsCtsBursts,
                       bsrSum,
                       lliTotal,
                       anyChanged);

    ResetObservations();
    m_stepEvent = Simulator::Schedule(m_period, &PedcaController::Step, this);
}

} // namespace ns3
