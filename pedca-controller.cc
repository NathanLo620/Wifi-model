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
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>

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
}

void
PedcaController::Start(Time when)
{
    NS_LOG_FUNCTION(this << when.As(Time::S));
    NS_ABORT_MSG_IF(!m_apMac, "PedcaController::Start called before Setup");

    Simulator::Schedule(when, [this]() {
        m_cwds = std::min<uint8_t>(m_initialTheta.cwds, m_cwdsMax);
        ResetObservations();

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

        if (!(next == previous))
        {
            anyChanged = true;
        }
        newTheta[aid] = next;
    }

    m_apMac->SetPedcaParametersBulk(newTheta);
    m_stepCount++;

    if (anyChanged)
    {
        std::clog << "[P-EDCA CTRL] t=" << Simulator::Now().GetMicroSeconds()
                  << "us cwds=" << +m_cwds << " aggr=" << nAggressive
                  << " cons=" << nConservative << " keep=" << nUnchanged
                  << " busyFrac=" << busyFrac << " collRate=" << collRate
                  << " collRateFrame=" << collRateFrame << " dsCtsFrames=" << dsCtsFrames
                  << " dsCtsBursts=" << dsCtsBursts << " bsrSum=" << bsrSum
                  << " lli=" << lliTotal << std::endl;
    }

    m_controlStepTrace(Simulator::Now(),
                       m_cwds,
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
