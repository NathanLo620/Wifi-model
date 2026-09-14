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
#include <optional>
#include <tuple>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("PedcaController");

NS_OBJECT_ENSURE_REGISTERED(PedcaController);

static_assert(std::tuple_size_v<decltype(PedcaSrcStats::qsrcHist)> == 16,
              "the controller histogram must match the one the frame exchange manager fills");

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
                          "burstadaptive (urgency-driven loop on an airtime cost proxy), hillclimb "
                          "(measure-and-revert search on the delay objective itself), modelpredictive "
                          "(predict both delay terms for every candidate and jump to the argmin), "
                          "srcfeedback (the same prediction, but from the QSRC/PSRC counters the "
                          "stations piggyback on every voice frame), v2 "
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
                                          PedcaPolicy::HILLCLIMB,
                                          "hillclimb",
                                          PedcaPolicy::MODELPREDICTIVE,
                                          "modelpredictive",
                                          PedcaPolicy::SRCFEEDBACK,
                                          "srcfeedback",
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
                          "samples in the burst-adaptive policy. 1.0 disables the filter and "
                          "controls on each period's raw measurement; the consecutive-period "
                          "hold counters then provide all of the damping.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&PedcaController::m_burstEwmaAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BurstOverheadHigh",
                          "Filtered Stage-1 airtime above which burst-adaptive becomes more "
                          "selective. The measured airtime stays between 2% and 13% over the "
                          "whole On/Off population range, so a threshold near the upper end "
                          "of that band leaves the loop unable to fire at all.",
                          DoubleValue(0.085),
                          MakeDoubleAccessor(&PedcaController::m_burstOverheadHigh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BurstOverheadLow",
                          "Filtered Stage-1 airtime below which burst-adaptive may admit more "
                          "stations when the delay tail needs help",
                          DoubleValue(0.055),
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
            .AddAttribute("BurstQsrcPrior",
                          "QSRC the loop returns to when nothing is asking for a move. The "
                          "R(10 ms) optimum over the dual-DS sweeps is 4 for On/Off and 3 for "
                          "MMPP and Poisson, so no single constant is right everywhere; the "
                          "AP cannot tell those cases apart from its own observations.",
                          UintegerValue(4),
                          MakeUintegerAccessor(&PedcaController::m_burstQsrcPrior),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("BurstQsrcFloor",
                          "Hard QSRC floor of the burst-adaptive loop",
                          UintegerValue(0),
                          MakeUintegerAccessor(&PedcaController::m_burstQsrcFloor),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("BurstAvoidQsrc1",
                          "Skip QSRC 1 when stepping. Eligibility at QSRC 1 is conditioned "
                          "on the immediately preceding EDCA collision, so it preferentially "
                          "admits the stations that just collided with each other.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&PedcaController::m_burstAvoidQsrc1),
                          MakeBooleanChecker())
            .AddAttribute("BurstCwds",
                          "CWds advertised by the burst-adaptive policy. 0, not the k-driven "
                          "1: the R(10 ms) selection over the On/Off dual-DS sweep picks "
                          "CWds 0 at every population.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&PedcaController::m_burstCwds),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("BurstPsrc",
                          "PSRC advertised by the burst-adaptive policy. Constant: PSRC 3 is "
                          "the sweep optimum in 14 of 15 traffic-model/population cells, and "
                          "no runtime signal has been shown to select it better.",
                          UintegerValue(3),
                          MakeUintegerAccessor(&PedcaController::m_burstPsrc),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("BurstDecayPeriods",
                          "Consecutive quiet periods (no overload, no delay pressure) after "
                          "which QSRC steps one back towards its prior. Without this the "
                          "loop ratchets: the airtime deadband can straddle the operating "
                          "point, so a step up is never undone.",
                          UintegerValue(8),
                          MakeUintegerAccessor(&PedcaController::m_burstDecayPeriods),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("HillTrialPeriods",
                          "Control periods averaged into one hill-climbing trial. Longer "
                          "trials measure the objective more precisely but leave fewer "
                          "trials inside a run.",
                          UintegerValue(6),
                          MakeUintegerAccessor(&PedcaController::m_hillTrialPeriods),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("HillDeadband",
                          "Relative improvement a candidate must show before it is adopted. "
                          "Guards against adopting measurement noise.",
                          DoubleValue(0.03),
                          MakeDoubleAccessor(&PedcaController::m_hillDeadband),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("HillCostFloor",
                          "Urgency below which the delay tail is considered healthy and the "
                          "search holds still rather than wandering on noise",
                          DoubleValue(0.02),
                          MakeDoubleAccessor(&PedcaController::m_hillCostFloor),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("HillQsrcMin",
                          "Smallest QSRC the hill climber may propose",
                          UintegerValue(0),
                          MakeUintegerAccessor(&PedcaController::m_hillQsrcMin),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("HillQsrcMax",
                          "Largest QSRC the hill climber may propose",
                          UintegerValue(PEDCA_QSRC_MAX),
                          MakeUintegerAccessor(&PedcaController::m_hillQsrcMax),
                          MakeUintegerChecker<uint8_t>(0, PEDCA_QSRC_MAX))
            .AddAttribute("HillPsrcMin",
                          "Smallest PSRC the hill climber may propose",
                          UintegerValue(1),
                          MakeUintegerAccessor(&PedcaController::m_hillPsrcMin),
                          MakeUintegerChecker<uint8_t>(1))
            .AddAttribute("HillPsrcMax",
                          "Largest PSRC the hill climber may propose",
                          UintegerValue(3),
                          MakeUintegerAccessor(&PedcaController::m_hillPsrcMax),
                          MakeUintegerChecker<uint8_t>(1))
            .AddAttribute("MpAlpha",
                          "EWMA weight on the primitives the model-predictive policy "
                          "estimates each period",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&PedcaController::m_mpAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("MpUtilMax",
                          "Predicted Stage-1 utilisation above which a candidate operating "
                          "point is rejected outright",
                          DoubleValue(0.35),
                          MakeDoubleAccessor(&PedcaController::m_mpUtilMax),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("MpHysteresis",
                          "Relative predicted gain required before the advertised operating "
                          "point is changed",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&PedcaController::m_mpHysteresis),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("MpMinBursts",
                          "DS-CTS bursts needed in a period before its loss ratio is used to "
                          "fit the Stage-1 collision hazard",
                          UintegerValue(10),
                          MakeUintegerAccessor(&PedcaController::m_mpMinBursts),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("MpWarmupPeriods",
                          "Periods the model-predictive policy observes before its first "
                          "switch, so the primitives are filtered before they are trusted",
                          UintegerValue(5),
                          MakeUintegerAccessor(&PedcaController::m_mpWarmupPeriods),
                          MakeUintegerChecker<uint32_t>()) 
            .AddAttribute("SrcAlpha",
                          "EWMA weight on the reported QSRC histogram in the srcfeedback "
                          "policy. 1 disables the filter.",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&PedcaController::m_srcAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("SrcHoldPeriods",
                          "Consecutive control periods that must agree on the direction "
                          "before the srcfeedback policy moves the threshold.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&PedcaController::m_srcHoldPeriods),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("SrcMaxStep",
                          "Largest change in QSRC the srcfeedback policy may make in one "
                          "control period. The inversion is only trustworthy near where the "
                          "distribution was measured.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&PedcaController::m_srcMaxStep),
                          MakeUintegerChecker<uint8_t>(1, PEDCA_QSRC_MAX))
            .AddAttribute("SrcMinReports",
                          "P-EDCA Status Reports a control period must carry before the "
                          "srcfeedback policy folds it into the histogram. Below this the "
                          "period is skipped, not averaged in.",
                          UintegerValue(20),
                          MakeUintegerAccessor(&PedcaController::m_srcMinReports),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SrcWarmupPeriods",
                          "Periods the srcfeedback policy observes before its first move.",
                          UintegerValue(3),
                          MakeUintegerAccessor(&PedcaController::m_srcWarmupPeriods),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("AlignToBeacon",
                          "Run exactly one control step per beacon interval, placed "
                          "BeaconLead ahead of the beacon, so that a decision reaches the "
                          "stations in the very next beacon instead of waiting out most of "
                          "one. Overrides Period.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&PedcaController::m_alignToBeacon),
                          MakeBooleanChecker())
            .AddAttribute("BeaconLead",
                          "How far ahead of the beacon an aligned control step is placed.",
                          TimeValue(MilliSeconds(2)),
                          MakeTimeAccessor(&PedcaController::m_beaconLead),
                          MakeTimeChecker())
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
        // A P-EDCA Status Report is the stronger proof of the two, and it arrives on the
        // station's first delivered voice frame rather than waiting for one to run short of
        // delay budget, so it closes the low-load blind spot the LLI evidence has.
        if (m_fem->GetLliRxCount(addr) > 0 || m_fem->GetPedcaSrcStats(addr).reports > 0)
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
PedcaController::BurstQsrcStep(uint8_t qsrc, int direction) const
{
    if (direction > 0)
    {
        if (qsrc >= PEDCA_QSRC_MAX)
        {
            return qsrc;
        }
        const uint8_t next = qsrc + 1;
        return (m_burstAvoidQsrc1 && next == 1) ? static_cast<uint8_t>(2) : next;
    }
    if (direction < 0)
    {
        if (qsrc == 0)
        {
            return qsrc;
        }
        const uint8_t next = qsrc - 1;
        return (m_burstAvoidQsrc1 && next == 1) ? static_cast<uint8_t>(0) : next;
    }
    return qsrc;
}

PedcaController::MpPrediction
PedcaController::MpPredict(uint8_t qsrc, uint8_t psrc) const
{
    MpPrediction out;
    out.qsrc = qsrc;
    out.psrc = psrc;

    // Qualifying means losing `qsrc` EDCA attempts in a row, so the eligible population
    // scales as p^qsrc. Only the ratio against the operating point actually being measured
    // is needed, which is what makes this observable: DS-CTS frames carry no transmitter
    // address, so the AP can count bursts but never the stations behind them.
    const auto dQsrc = static_cast<double>(qsrc) - static_cast<double>(m_mpQsrc);
    // PSRC lets a station keep the privilege over consecutive attempts, extending how long
    // it contributes to Stage-1 load.
    const auto kappa = [this](uint8_t p) {
        return 1.0 + (static_cast<double>(p) - 1.0) * m_mpP;
    };
    const auto psrcScale = kappa(psrc) / std::max(1e-9, kappa(m_mpPsrc));
    out.bursts = std::max(0.0, m_mpBursts * std::pow(m_mpP, dQsrc) * psrcScale);

    const auto periodSec = m_period.GetSeconds();
    const auto burstSec = m_burstCost.GetSeconds();
    out.util = (periodSec > 0.0) ? out.bursts * burstSec / periodSec : 0.0;

    // Term 1, rising in qsrc: the frame has to lose `qsrc` attempts before P-EDCA will take
    // it, and every station's attempt cycle stretches as Stage-1 eats into the medium.
    const auto inflation = 1.0 - std::min(out.util, 0.95);
    out.wQual = static_cast<double>(qsrc) * m_mpTau0 / std::max(0.05, inflation);

    // Term 2, falling in qsrc: reservations collide with each other, and the hazard was
    // fitted to the burst loss measured at the current point.
    const auto succ = std::exp(-m_mpGamma * out.bursts);
    out.wStage1 = burstSec / std::max(1e-3, succ);

    out.delay = out.wQual + out.wStage1;
    return out;
}

double
PedcaController::SrcSurvival(uint8_t qsrc) const
{
    double tail = 0.0;
    for (std::size_t q = qsrc; q < SRC_QSRC_BINS; q++)
    {
        tail += m_srcPmf[q];
    }
    return std::clamp(tail, 0.0, 1.0);
}

void
PedcaController::SrcObserve(double busyFrac)
{
    // Deltas against the previous step, so that a period is a period and not everything
    // since the station associated.
    const auto& stats = m_fem->GetPedcaSrcStats();
    const auto reports = stats.reports - m_lastSrcReports;
    const auto qsrcSum = stats.qsrcSum - m_lastSrcQsrcSum;
    const auto psrcSum = stats.psrcSum - m_lastSrcPsrcSum;
    const auto stage2 = stats.stage2 - m_lastSrcStage2;

    std::array<uint32_t, SRC_QSRC_BINS> hist{};
    for (std::size_t q = 0; q < SRC_QSRC_BINS; q++)
    {
        hist[q] = stats.qsrcHist[q] - m_lastSrcHist[q];
        m_lastSrcHist[q] = stats.qsrcHist[q];
    }
    m_lastSrcReports = stats.reports;
    m_lastSrcQsrcSum = stats.qsrcSum;
    m_lastSrcPsrcSum = stats.psrcSum;
    m_lastSrcStage2 = stats.stage2;

    // Urgency over the stations that are actually running P-EDCA. Only they ever set LLI,
    // so dividing by every station's voice frames -- which is all the AP could do before --
    // reads low by roughly the penetration ratio. The per-station report count is the
    // denominator that ratio wanted all along.
    uint32_t lliDelta = 0;
    uint32_t reportDelta = 0;
    for (const auto& [aid, addr] : m_apMac->GetStaList(0))
    {
        const auto r = m_fem->GetPedcaSrcStats(addr).reports;
        if (r == 0)
        {
            continue; // never reported, so not a P-EDCA station
        }
        auto& prevR = m_lastSrcReportsByAddr[addr];
        auto& prevL = m_lastSrcLliByAddr[addr];
        const auto l = m_fem->GetLliRxCount(addr);
        reportDelta += r - prevR;
        lliDelta += l - prevL;
        prevR = r;
        prevL = l;
    }
    if (reportDelta > 0)
    {
        m_srcUrgency = static_cast<double>(lliDelta) / reportDelta;
    }

    m_srcReports = reports;
    if (reports == 0)
    {
        m_srcMeanQsrc = 0.0;
        m_srcMeanPsrc = 0.0;
        m_srcStage2Frac = 0.0;
        return;
    }

    const auto n = static_cast<double>(reports);
    m_srcMeanQsrc = static_cast<double>(qsrcSum) / n;
    m_srcMeanPsrc = static_cast<double>(psrcSum) / n;
    m_srcStage2Frac = static_cast<double>(stage2) / n;

    // The exact number of EDCA attempts the reporting stations made: one per delivery plus
    // the failures each of them owns up to. MODELPREDICTIVE can only write
    // deliveries + (deliveries that carry a set Retry bit), which counts a frame that failed
    // five times exactly once, so its tau0 reads long wherever retries cluster.
    const auto attempts = n + static_cast<double>(qsrcSum);
    const auto tau0Sample = busyFrac * m_period.GetSeconds() / attempts;

    // A handful of reports quantises the histogram into uselessness: with 5 deliveries the
    // survival function can only take the values 0, 0.2, 0.4 ... Skip such a period rather
    // than filtering it in, so a quiet period cannot drag the estimate.
    if (reports < m_srcMinReports)
    {
        return;
    }

    const auto a = m_srcReady ? m_srcAlpha : 1.0;
    for (std::size_t q = 0; q < SRC_QSRC_BINS; q++)
    {
        const auto share = static_cast<double>(hist[q]) / n;
        m_srcPmf[q] = a * share + (1.0 - a) * m_srcPmf[q];
    }
    m_srcTau0 = a * tau0Sample + (1.0 - a) * m_srcTau0;
    m_srcReady = true;
    m_srcPeriods++;
}

uint8_t
PedcaController::SrcThresholdFor(double admit) const
{
    // G is non-increasing in T, so the first threshold that fits the budget is the most
    // aggressive one that does. This is the whole payoff of the piggyback: without the
    // reported distribution there is no such table to read, and the budget can only be met
    // by stepping and re-measuring.
    for (uint8_t t = 0; t <= PEDCA_QSRC_MAX; t++)
    {
        if (m_burstAvoidQsrc1 && t == 1)
        {
            continue;
        }
        if (SrcSurvival(t) <= admit)
        {
            return t;
        }
    }
    return PEDCA_QSRC_MAX;
}

void
PedcaController::Start(Time when)
{
    NS_LOG_FUNCTION(this << when.As(Time::S));
    NS_ABORT_MSG_IF(!m_apMac, "PedcaController::Start called before Setup");

    if (m_alignToBeacon)
    {
        // One decision per beacon, so that what the loop decides is advertised immediately
        // rather than sitting in the table for a fraction of a beacon interval first. The
        // stations' reports are accumulated over exactly the same window.
        m_period = m_apMac->GetBeaconInterval();
    }

    Simulator::Schedule(when, [this]() {
        m_cwds = std::min<uint8_t>(m_initialTheta.cwds, m_cwdsMax);
        // Start on the configured triple, not on the prior. The controller has to reach a
        // good operating point by adapting; seeding it at the answer would measure the prior
        // rather than the loop.
        m_burstAdaptiveQsrc = std::clamp(m_initialTheta.qsrcThreshold,
                                         m_burstQsrcFloor,
                                         static_cast<uint8_t>(PEDCA_QSRC_MAX));
        m_burstAdaptivePsrc = m_initialTheta.psrcLimit;

        // The hill climber starts wherever it was configured to start and searches from
        // there; it is never seeded at a known-good operating point.
        m_hillQsrc = std::clamp(m_initialTheta.qsrcThreshold, m_hillQsrcMin, m_hillQsrcMax);
        m_hillPsrc = std::clamp(m_initialTheta.psrcLimit, m_hillPsrcMin, m_hillPsrcMax);
        m_hillBestQsrc = m_hillQsrc;
        m_hillBestPsrc = m_hillPsrc;
        m_hillBestCost = -1.0;
        m_hillTrialCost = 0.0;
        m_hillCostAccum = 0.0;
        m_hillSamples = 0;
        m_hillDir = 1;
        m_hillFailures = 0;
        m_hillTrials = 0;
        m_hillDim = HillDim::QSRC;
        m_hillPhase = HillPhase::PROBE;

        m_mpQsrc = m_initialTheta.qsrcThreshold;
        m_mpPsrc = m_initialTheta.psrcLimit;
        m_mpP = 0.5;
        m_mpTau0 = 0.0;
        m_mpGamma = 0.0;
        m_mpBursts = 0.0;
        m_mpPredDelay = 0.0;
        m_mpLittleDelay = 0.0;
        m_mpReady = false;
        m_mpPeriods = 0;
        m_lastVoRetryRx = m_fem->GetVoRetryRxCount();

        m_srcQsrc = m_initialTheta.qsrcThreshold;
        m_srcPsrc = m_initialTheta.psrcLimit;
        m_srcPmf.fill(0.0);
        m_srcReady = false;
        m_srcPeriods = 0;
        m_srcReports = 0;
        m_srcMeanQsrc = 0.0;
        m_srcMeanPsrc = 0.0;
        m_srcStage2Frac = 0.0;
        m_srcTau0 = 0.0;
        m_srcTargetAdmit = 0.0;
        m_srcUpVotes = 0;
        m_srcDownVotes = 0;
        m_srcReleaseVotes = 0;
        m_srcUrgency = 0.0;
        m_lastSrcReportsByAddr.clear();
        m_lastSrcLliByAddr.clear();
        // Seeded, not just cleared: a default-constructed entry would make the first period's
        // delta the whole of warmup.
        for (const auto& [aid, addr] : m_apMac->GetStaList(0))
        {
            m_lastSrcReportsByAddr[addr] = m_fem->GetPedcaSrcStats(addr).reports;
            m_lastSrcLliByAddr[addr] = m_fem->GetLliRxCount(addr);
        }

        m_burstEwmaReady = false;
        m_burstCollisionEwmaReady = false;
        m_overheadEwma = 0.0;
        m_urgencyEwma = 0.0;
        m_collisionEwma = 0.0;
        m_burstUpVotes = 0;
        m_burstDownVotes = 0;
        m_burstDecayVotes = 0;
        ResetObservations();

        // The BSS-wide deltas must start at the same boundary as the PHY observations.
        // Otherwise the first control period includes all feedback accumulated in warmup.
        m_lastVoRx = m_fem->GetVoRxCount();
        m_lastLliTotal = m_fem->GetLliRxCount();
        m_lastVoRetryRx = m_fem->GetVoRetryRxCount();

        // Snapshot the per-station LLI counters so that the first step sees a delta over the
        // period rather than everything accumulated during warmup.
        m_lastLliByAddr.clear();
        for (const auto& [aid, addr] : m_apMac->GetStaList(0))
        {
            m_lastLliByAddr[addr] = m_fem->GetLliRxCount(addr);
        }

        // Same for the piggybacked retry reports: everything the stations sent during warmup
        // belongs to a different operating point.
        const auto& src = m_fem->GetPedcaSrcStats();
        m_lastSrcReports = src.reports;
        m_lastSrcQsrcSum = src.qsrcSum;
        m_lastSrcPsrcSum = src.psrcSum;
        m_lastSrcStage2 = src.stage2;
        std::copy(src.qsrcHist.begin(), src.qsrcHist.end(), m_lastSrcHist.begin());

        // Land the first step just before a beacon; every later one inherits the phase,
        // because the period is the beacon interval.
        auto first = m_period;
        if (m_alignToBeacon)
        {
            const auto toBeacon = m_apMac->GetTimeToNextBeacon(0);
            if (toBeacon.IsStrictlyPositive())
            {
                first = toBeacon;
                while (first < m_beaconLead)
                {
                    first += m_period;
                }
                first -= m_beaconLead;
            }
        }
        m_stepEvent = Simulator::Schedule(first, &PedcaController::Step, this);
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
    m_lastUrgency = urgency;

    // ---- Model-predictive primitives -------------------------------------------------
    // Pure measurement, so it runs whatever policy is active: that keeps the estimates
    // comparable across policies and lets them be validated against a static operating
    // point before any controller acts on them.
    m_mpPeriods++;

    // ---- Phase I/II: estimate the primitives from this period's observations ----
    const auto retryTotal = m_fem->GetVoRetryRxCount();
    const auto retryRx = retryTotal - m_lastVoRetryRx;
    m_lastVoRetryRx = retryTotal;
    // Fraction of voice frames that already lost at least one EDCA attempt. Under the
    // geometric model that fraction is the per-attempt failure probability.
    const auto pSample =
        (voRx > 0) ? std::clamp(static_cast<double>(retryRx) / voRx, m_mpPMin, m_mpPMax)
                   : m_mpP;

    // Mean time one EDCA attempt occupies, from the medium the AP watched go by.
    const auto periodSec = m_period.GetSeconds();
    const auto attempts = static_cast<double>(voRx) + static_cast<double>(retryRx);
    const auto tau0Sample =
        (attempts > 0.0) ? busyFrac * periodSec / attempts : m_mpTau0;

    const auto burstsSample = static_cast<double>(m_bursts);
    // One-point fit of the Stage-1 collision hazard: S = exp(-gamma * bursts), measured
    // where the system is actually running. Too few bursts and the ratio is noise.
    double gammaSample = m_mpGamma;
    if (m_bursts >= m_mpMinBursts && burstsSample > 0.0)
    {
        const auto succ = std::clamp(1.0 - collRateBurst, 1e-3, 1.0);
        gammaSample = -std::log(succ) / burstsSample;
    }

    if (!m_mpReady)
    {
        m_mpP = pSample;
        m_mpTau0 = tau0Sample;
        m_mpGamma = gammaSample;
        m_mpBursts = burstsSample;
        m_mpReady = true;
    }
    else
    {
        const auto a = m_mpAlpha;
        m_mpP = a * pSample + (1.0 - a) * m_mpP;
        m_mpTau0 = a * tau0Sample + (1.0 - a) * m_mpTau0;
        m_mpGamma = a * gammaSample + (1.0 - a) * m_mpGamma;
        m_mpBursts = a * burstsSample + (1.0 - a) * m_mpBursts;
    }

    // The piggybacked retry reports, folded in the same way and for the same reason: measure
    // under every policy, so that the estimates can be validated against a static operating
    // point before any controller is allowed to act on them.
    SrcObserve(busyFrac);

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
        // The operating point the loop returns to. Constant: see BurstQsrcPrior.
        const auto qsrcPrior = std::clamp(m_burstQsrcPrior,
                                          m_burstQsrcFloor,
                                          static_cast<uint8_t>(PEDCA_QSRC_MAX));

        // ---- Filtered runtime signals ----
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

        // ---- Urgency-driven loop around the prior ----
        // The objective is the delay tail, so the signal that decides *whether* to act is
        // urgency: the fraction of arriving voice frames already past 70% of their delay
        // bound. Stage-1 airtime then decides *which way* to move. Raising QSRC makes
        // eligibility more selective, which buys back medium time; lowering it admits more
        // stations, which is only affordable when Stage-1 is not already the expense.
        const bool tailInTrouble = m_urgencyEwma > m_urgencyHigh;
        const bool stage1Expensive = m_overheadEwma > m_burstOverheadHigh;
        const bool stage1Cheap = m_overheadEwma < m_burstOverheadLow;
        const bool collisionSevere =
            collisionSampleValid && m_collisionEwma > m_burstCollisionHigh;

        int direction = 0;
        if (collisionSevere || (stage1Expensive && !stage1Cheap))
        {
            // Either P-EDCA is eating the medium it is meant to protect, or Stage 1 is
            // losing bursts outright. Both are cured by admitting fewer stations.
            direction = +1;
        }
        else if (tailInTrouble && stage1Cheap)
        {
            // Frames are running out of budget while Stage 1 is cheap: there is room to let
            // more of them reserve.
            direction = -1;
        }

        if (direction > 0 && m_burstAdaptiveQsrc < PEDCA_QSRC_MAX)
        {
            m_burstUpVotes++;
            m_burstDownVotes = 0;
            m_burstDecayVotes = 0;
            if (m_burstUpVotes >= m_burstUpHoldPeriods)
            {
                m_burstAdaptiveQsrc = BurstQsrcStep(m_burstAdaptiveQsrc, +1);
                m_burstUpVotes = 0;
            }
        }
        else if (direction < 0 && m_burstAdaptiveQsrc > m_burstQsrcFloor)
        {
            m_burstDownVotes++;
            m_burstUpVotes = 0;
            m_burstDecayVotes = 0;
            if (m_burstDownVotes >= m_burstDownHoldPeriods)
            {
                m_burstAdaptiveQsrc = BurstQsrcStep(m_burstAdaptiveQsrc, -1);
                m_burstDownVotes = 0;
            }
        }
        else
        {
            m_burstUpVotes = 0;
            m_burstDownVotes = 0;
            // Nothing is asking for a move. Decay back towards the prior so that a step
            // taken under transient pressure is given back: the airtime deadband can sit
            // either side of the steady-state operating point, in which case an increase
            // would otherwise be permanent.
            if (m_burstAdaptiveQsrc != qsrcPrior || m_burstAdaptivePsrc != m_burstPsrc)
            {
                m_burstDecayVotes++;
                if (m_burstDecayVotes >= m_burstDecayPeriods)
                {
                    if (m_burstAdaptiveQsrc != qsrcPrior)
                    {
                        m_burstAdaptiveQsrc = BurstQsrcStep(
                            m_burstAdaptiveQsrc, (m_burstAdaptiveQsrc > qsrcPrior) ? -1 : +1);
                    }
                    // PSRC has no loop of its own; it only walks from the configured start
                    // to its constant target, one step per quiet interval.
                    if (m_burstAdaptivePsrc != m_burstPsrc)
                    {
                        m_burstAdaptivePsrc += (m_burstAdaptivePsrc > m_burstPsrc) ? -1 : +1;
                    }
                    m_burstDecayVotes = 0;
                }
            }
            else
            {
                m_burstDecayVotes = 0;
            }
        }

        // The loop may move either side of the prior now. The prior is where a class change
        // puts it, not a floor: under smooth traffic the whole point is that QSRC 0 is
        // reachable, and under bursty traffic QSRC 5 must stay reachable above the q4 prior.
        m_burstAdaptiveQsrc = std::clamp(m_burstAdaptiveQsrc,
                                         m_burstQsrcFloor,
                                         static_cast<uint8_t>(PEDCA_QSRC_MAX));
    }

    else if (m_policy == PedcaPolicy::HILLCLIMB)
    {
        // The objective. Urgency is the fraction of arriving voice frames already past 70%
        // of their delay bound, so lowering it is lowering the delay tail. No airtime or
        // collision term appears here on purpose: those say what P-EDCA costs, not whether
        // the tail got shorter, and optimising the proxy is what the previous policy did.
        m_hillCostAccum += urgency;
        m_hillSamples++;

        if (m_hillSamples >= m_hillTrialPeriods)
        {
            const auto cost = m_hillCostAccum / m_hillSamples;
            m_hillCostAccum = 0.0;
            m_hillSamples = 0;
            m_hillTrialCost = cost;
            m_hillTrials++;

            if (m_hillPhase == HillPhase::REFRESH)
            {
                // We were sitting on the incumbent purely to re-measure it.
                m_hillBestCost = cost;
                m_hillPhase = HillPhase::PROBE;
            }
            else if (m_hillBestCost < 0.0)
            {
                // First trial: nothing to compare against, so this is the incumbent.
                m_hillBestCost = cost;
                m_hillBestQsrc = m_hillQsrc;
                m_hillBestPsrc = m_hillPsrc;
            }
            else if (cost < m_hillBestCost * (1.0 - m_hillDeadband))
            {
                // The candidate is better. Adopt it and keep going the same way.
                m_hillBestCost = cost;
                m_hillBestQsrc = m_hillQsrc;
                m_hillBestPsrc = m_hillPsrc;
                m_hillFailures = 0;
            }
            else
            {
                // No improvement: try the other side next.
                m_hillDir = -m_hillDir;
                m_hillFailures++;
                if (m_hillFailures >= 2)
                {
                    // Both sides lost. Re-measure the incumbent before believing that, then
                    // move the search to the other parameter.
                    m_hillFailures = 0;
                    m_hillPhase = HillPhase::REFRESH;
                    m_hillDim = (m_hillDim == HillDim::QSRC) ? HillDim::PSRC : HillDim::QSRC;
                }
            }

            // Choose the point to hold for the next trial.
            m_hillQsrc = m_hillBestQsrc;
            m_hillPsrc = m_hillBestPsrc;
            const bool tailHealthy = (m_hillBestCost >= 0.0) &&
                                     (m_hillBestCost < m_hillCostFloor);
            if (m_hillPhase == HillPhase::PROBE && !tailHealthy)
            {
                // Hold still when the tail is already healthy: with nothing to gain, every
                // step is a coin flip on measurement noise.
                if (m_hillDim == HillDim::QSRC)
                {
                    auto next = static_cast<int>(m_hillBestQsrc) + m_hillDir;
                    // QSRC 1 admits precisely the stations that just collided together.
                    if (m_burstAvoidQsrc1 && next == 1)
                    {
                        next += m_hillDir;
                    }
                    m_hillQsrc = static_cast<uint8_t>(
                        std::clamp(next, static_cast<int>(m_hillQsrcMin),
                                   static_cast<int>(m_hillQsrcMax)));
                }
                else
                {
                    const auto next = static_cast<int>(m_hillBestPsrc) + m_hillDir;
                    m_hillPsrc = static_cast<uint8_t>(
                        std::clamp(next, static_cast<int>(m_hillPsrcMin),
                                   static_cast<int>(m_hillPsrcMax)));
                }
            }
        }
    }

    else if (m_policy == PedcaPolicy::MODELPREDICTIVE)
    {
        // ---- Phase III: predict every candidate and take the argmin ----
        const auto current = MpPredict(m_mpQsrc, m_mpPsrc);
        m_mpPredDelay = current.delay;

        if (m_mpPeriods > m_mpWarmupPeriods && m_mpTau0 > 0.0)
        {
            std::optional<MpPrediction> best;
            for (uint8_t q = 0; q <= PEDCA_QSRC_MAX; q++)
            {
                // QSRC 1 admits precisely the stations that just collided with each other.
                if (m_burstAvoidQsrc1 && q == 1)
                {
                    continue;
                }
                for (uint8_t l = m_hillPsrcMin; l <= m_hillPsrcMax; l++)
                {
                    const auto cand = MpPredict(q, l);
                    if (cand.util >= m_mpUtilMax)
                    {
                        continue;
                    }
                    if (!best || cand.delay < best->delay)
                    {
                        best = cand;
                    }
                }
            }

            // ---- Phase IV: actuate, with hysteresis so noise cannot start a cycle ----
            if (best && best->delay < current.delay * (1.0 - m_mpHysteresis))
            {
                m_mpQsrc = best->qsrc;
                m_mpPsrc = best->psrc;
                m_mpPredDelay = best->delay;
            }
        }

        // Consistency guard: Little's law over the reported backlog gives a delay estimate
        // that never passes through the LLI bit, so it is not censored the way the delay
        // percentiles and the urgency ratio both are. It is recorded rather than acted on;
        // a persistent gap means the two-term model has drifted.
        uint32_t bsrBytes = 0;
        for (const auto& [aid, addr] : m_apMac->GetStaList(0))
        {
            for (const auto tid : PEDCA_VO_TIDS)
            {
                // 255 means the station never reported, or its report has expired.
                if (const auto qs = m_apMac->GetBufferStatus(tid, addr); qs != 255)
                {
                    bsrBytes += static_cast<uint32_t>(qs) * 256;
                }
            }
        }
        const auto deliveredBytes = static_cast<double>(voRx) * 1000.0;
        m_mpLittleDelay =
            (deliveredBytes > 0.0) ? bsrBytes * periodSec / deliveredBytes : 0.0;
    }

    else if (m_policy == PedcaPolicy::SRCFEEDBACK)
    {
        // Same objective and the same target band BURSTADAPTIVE was tuned to: hold the share
        // of airtime Stage-1 consumes inside [BurstOverheadLow, BurstOverheadHigh]. What
        // changes is how the threshold that achieves it is found.
        const auto gCur = SrcSurvival(m_srcQsrc);
        m_srcTargetAdmit = gCur;

        const bool usable = m_srcReady && m_srcPeriods > m_srcWarmupPeriods && gCur > 1e-4;
        const bool stage1Expensive = overhead > m_burstOverheadHigh;
        // An idle medium is not evidence that anything needs help. Lowering the threshold on
        // airtime alone is what makes P-EDCA fire on sparse traffic, which costs more tail
        // than it buys; so the loosening direction additionally needs frames actually short
        // of delay budget. See the same asymmetry in LOADDRIVEN.
        const bool stage1Cheap = overhead < m_burstOverheadLow;
        const bool tailInTrouble = m_srcUrgency > m_urgencyHigh;
        // The mirror image, and the reason the airtime budget alone is not enough: Stage-1
        // airtime measures what P-EDCA costs the P-EDCA stations, not what a Stage-2 TXOP
        // costs the legacy stations it pre-empts. At low penetration Stage 1 stays well
        // inside its budget however many frames are admitted, so nothing ever asks the
        // threshold to rise, and P-EDCA keeps being spent on frames that were in no danger.
        // Releasing the medium when no delivery is near its delay bound is the only thing
        // that stops that, and it needs the per-station urgency above to be visible at all.
        const bool tailHealthy = m_srcUrgency < m_urgencyLow;

        int direction = 0;
        if (stage1Expensive || tailHealthy)
        {
            direction = +1;
        }
        else if (stage1Cheap && tailInTrouble)
        {
            direction = -1;
        }

        // A release is a one-notch step, not a solve: the airtime ratio that sizes a solve
        // says nothing about how far to back off when airtime was never the problem.
        const bool release = (direction > 0) && !stage1Expensive;

        if (direction > 0)
        {
            (release ? m_srcReleaseVotes : m_srcUpVotes)++;
            if (release)
            {
                m_srcUpVotes = 0;
            }
            else
            {
                m_srcReleaseVotes = 0;
            }
            m_srcDownVotes = 0;
        }
        else if (direction < 0)
        {
            m_srcDownVotes++;
            m_srcUpVotes = 0;
            m_srcReleaseVotes = 0;
        }
        else
        {
            m_srcUpVotes = 0;
            m_srcDownVotes = 0;
            m_srcReleaseVotes = 0;
            // Nothing is asking for a move, so walk PSRC the one step it is allowed towards
            // its constant target. PSRC has no loop of its own here either: the sweeps pick
            // PSRC 3 in 14 of 15 traffic-model/population cells.
            if (m_srcPsrc != m_burstPsrc)
            {
                m_srcPsrc += (m_srcPsrc > m_burstPsrc) ? -1 : +1;
            }
        }

        // A release holds longer than an airtime correction: it is giving up a mechanism
        // that is currently working, on the evidence that nothing needs it.
        const auto votes =
            release ? m_srcReleaseVotes : ((direction > 0) ? m_srcUpVotes : m_srcDownVotes);
        const auto hold = release ? m_burstDecayPeriods : m_srcHoldPeriods;
        if (usable && direction != 0 && votes >= hold)
        {
            // Stage-1 airtime is very nearly proportional to how many deliveries qualify, so
            // the admitted fraction that lands in the middle of the band is the current one
            // scaled by how far off budget the measurement is. Then read the threshold that
            // admits it off the measured survival function -- one step, not a hunt.
            if (!release)
            {
                const auto targetOverhead = 0.5 * (m_burstOverheadLow + m_burstOverheadHigh);
                const auto scale = targetOverhead / std::max(overhead, 1e-4);
                m_srcTargetAdmit = std::clamp(gCur * scale, 0.0, 1.0);
            }

            auto next = release ? static_cast<uint8_t>(std::min<int>(m_srcQsrc + 1,
                                                                      PEDCA_QSRC_MAX))
                                : SrcThresholdFor(m_srcTargetAdmit);
            // The inversion is only trustworthy near where the distribution was measured: a
            // period that happens to carry an odd sample should not be able to throw the
            // threshold across its whole range.
            const auto lo = static_cast<int>(m_srcQsrc) - static_cast<int>(m_srcMaxStep);
            const auto hi = static_cast<int>(m_srcQsrc) + static_cast<int>(m_srcMaxStep);
            auto clamped = std::clamp(static_cast<int>(next), lo, hi);
            if (m_burstAvoidQsrc1 && clamped == 1)
            {
                // QSRC 1 admits precisely the stations that just collided with each other.
                clamped += direction;
            }
            m_srcQsrc = static_cast<uint8_t>(
                std::clamp(clamped,
                           static_cast<int>(m_burstQsrcFloor),
                           static_cast<int>(PEDCA_QSRC_MAX)));
            m_srcUpVotes = 0;
            m_srcDownVotes = 0;
            m_srcReleaseVotes = 0;
        }
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
        bssWide = PedcaTheta{m_burstCwds, m_burstAdaptiveQsrc, m_burstAdaptivePsrc};
    }
    else if (m_policy == PedcaPolicy::HILLCLIMB)
    {
        bssWide = PedcaTheta{m_burstCwds, m_hillQsrc, m_hillPsrc};
    }
    else if (m_policy == PedcaPolicy::MODELPREDICTIVE)
    {
        bssWide = PedcaTheta{m_burstCwds, m_mpQsrc, m_mpPsrc};
    }
    else if (m_policy == PedcaPolicy::SRCFEEDBACK)
    {
        bssWide = PedcaTheta{m_burstCwds, m_srcQsrc, m_srcPsrc};
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
        case PedcaPolicy::HILLCLIMB:
        case PedcaPolicy::MODELPREDICTIVE:
        case PedcaPolicy::SRCFEEDBACK:
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
                  << " lli=" << lliTotal;
        if (m_policy == PedcaPolicy::SRCFEEDBACK)
        {
            std::clog << " srcReports=" << m_srcReports
                      << " meanQsrc=" << m_srcMeanQsrc
                      << " meanPsrc=" << m_srcMeanPsrc
                      << " stage2Frac=" << m_srcStage2Frac
                      << " tau0=" << m_srcTau0
                      << " G(T)=" << SrcSurvival(m_srcQsrc)
                      << " srcUrgency=" << m_srcUrgency
                      << " admit=" << m_srcTargetAdmit;
        }
        std::clog << std::endl;
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
