/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * AP-side adaptive P-EDCA controller.
 */

#ifndef PEDCA_CONTROLLER_H
#define PEDCA_CONTROLLER_H

#include "pedca-parameter-set.h"
#include "wifi-phy-common.h"
#include "wifi-phy-state.h"

#include "ns3/event-id.h"
#include "ns3/mac48-address.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/traced-callback.h"

#include <map>
#include <set>

namespace ns3
{

class ApWifiMac;
class QosFrameExchangeManager;
class WifiNetDevice;
class WifiPhy;
class WifiPpdu;

/**
 * @ingroup wifi
 * Which rule set PedcaController uses to choose parameters.
 */
enum class PedcaPolicy
{
    /**
     * Choose one BSS-wide parameter triple from the estimated number of stations actually
     * using P-EDCA. Derived from the 2026-08-01 CWds x QSRC x PSRC sweeps over five traffic
     * models and three penetrations; see PedcaController::KDrivenTheta.
     */
    KDRIVEN,
    /**
     * Regulate QSRC against what is happening on the medium right now, with no reference to
     * how many stations are configured for P-EDCA.
     *
     * Two observables, both re-measured every period and both of which move with the offered
     * load rather than with the configuration: the fraction of airtime Stage-1 is consuming,
     * and the fraction of arriving voice frames already close to their delay bound. Too much
     * airtime means P-EDCA is congesting itself, so QSRC goes up; frames running out of
     * budget while airtime is cheap means there is room to be more aggressive, so QSRC comes
     * down. Anything in between holds.
     *
     * This is the policy to use for bursty traffic, where the number of stations actually
     * contending changes continuously and a configured count says nothing about it.
     */
    LOADDRIVEN,
    /**
     * Bursty-traffic policy calibrated from the On/Off saturation sweep.
     *
     * The estimated active P-EDCA population supplies a conservative QSRC prior (3 or 4
     * as the population grows). Runtime Stage-1 airtime, DS-CTS loss and LLI urgency
     * refine that operating point with EWMA filtering and asymmetric hold times; in the
     * largest bucket q4 is the prior and q5 is reserved for measured overload. QSRC 1 is
     * deliberately excluded: it selects exactly
     * the stations that have just collided and is therefore the operating point most
     * exposed to collision-conditioned re-synchronisation.
     */
    BURSTADAPTIVE,
    /**
     * The original rule set: BSS-wide CWds from the DS-CTS collision rate, a global gate on
     * the channel busy fraction, and a per-station choice between an aggressive and a
     * conservative corner. Kept for comparison.
     */
    V2,
    /** Push one constant triple to every station. The control arm with the loop switched off. */
    FIXED
};

/**
 * @ingroup wifi
 *
 * Closed-loop controller that periodically re-chooses the P-EDCA parameters of every
 * associated station and hands them to the AP for advertisement.
 *
 * The object is standalone: a simulation script creates it, points it at an AP device and
 * starts it. Nothing in the MAC depends on it, so a run without a controller behaves exactly
 * as it did before.
 *
 * Every Period it reads what happened on air during the elapsed period -- the fraction of
 * time the channel was busy, and how many P-EDCA DS-CTS bursts were heard versus lost -- plus
 * the per-station buffer status and low-latency-indication feedback that stations piggyback
 * on their uplink frames. From those it computes one parameter triple per station and pushes
 * the whole table to ApWifiMac::SetPedcaParametersBulk(), which advertises it in the next
 * beacon.
 *
 * The decision rules are the ones calibrated against the earlier single-DS-CTS sender: a
 * BSS-wide CWds driven by the DS-CTS collision rate, a global gate that pushes every station
 * to a conservative corner when the channel is congested, and otherwise a per-station choice
 * between an aggressive corner (this station is running out of delay budget or has a backlog)
 * and keeping what it already has. Both corners are exposed as attributes, because the dual
 * DS-CTS sender has not been re-calibrated against them yet.
 */
class PedcaController : public Object
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    PedcaController();
    ~PedcaController() override;

    /**
     * Attach the controller to an AP. Grabs the AP MAC, PHY and frame exchange manager of
     * link 0 and subscribes to the traces the control loop reads. Single-link only.
     *
     * @param apDevice the AP device to control
     */
    void Setup(Ptr<WifiNetDevice> apDevice);

    /**
     * Tell the controller which parameters the stations start out with, so that its first
     * decision is a delta from the real starting point. The AP's own frame exchange manager
     * cannot be used for this: the AP is not a P-EDCA sender, so its parameters are unrelated
     * to whatever the script configured on the stations.
     *
     * @param theta the parameters the stations were configured with
     */
    void SetInitialTheta(const PedcaTheta& theta);

    /**
     * Start the control loop. Observations accumulated before this instant are discarded, so
     * that a warmup period does not leak into the first decision. The first decision is taken
     * one Period later.
     *
     * @param when when to start observing
     */
    void Start(Time when);

    /** Stop the control loop. No further parameter tables are pushed. */
    void Stop();

    /** @return the number of control steps taken so far */
    uint32_t GetStepCount() const;

  private:
    void DoDispose() override;

    /** One control step: observe, decide, push, reschedule. */
    void Step();

    /**
     * Estimate how many associated stations are actually using P-EDCA.
     *
     * The AP has no way to ask: PedcaSupported is a station-local flag with no over-the-air
     * capability report. What it does have is the Low Latency Indication bit, which only a
     * P-EDCA station ever sets, so a station that has sent even one LLI-marked frame is
     * proven to be running P-EDCA with no false positives. The set is sticky, because a
     * station does not stop being P-EDCA-capable between periods.
     *
     * Known blind spot: LLI only fires once a frame's head-of-line age passes a fraction of
     * the delay bound, so under light load the estimate reads low, possibly zero, even
     * though P-EDCA stations are present. Use SetKOverride() to separate policy error from
     * estimator error when validating.
     *
     * @return the number of stations proven to be using P-EDCA
     */
    uint32_t EstimatePedcaStaCount();

    /**
     * The k-driven parameter law, fitted to the 2026-08-01 sweeps.
     *
     * CWds is held at 1 (best in 13 of the 15 traffic x penetration cells, and worth under
     * 3% in the other two). QSRC rises linearly with k, which is what bounds how often a
     * station may enter Stage 1 as contention grows. PSRC falls in steps, because extra
     * consecutive attempts pay off while the medium has room and become pure overhead once
     * it does not.
     *
     * @param k the estimated number of stations using P-EDCA
     * @return the parameter triple to advertise to every station
     */
    PedcaTheta KDrivenTheta(uint32_t k) const;

    /** Return the nominal QSRC prior used by the burst-adaptive policy for population k. */
    uint8_t BurstQsrcPrior(uint32_t k) const;

    /**
     * PHY state trace sink, accumulating the time the channel was not idle.
     *
     * @param start when the state was entered
     * @param duration how long the state lasts
     * @param state the state that was entered
     */
    void PhyStateCb(Time start, Time duration, WifiPhyState state);

    /**
     * PHY drop trace sink, counting DS-CTS frames that failed to decode.
     *
     * @param ppdu the dropped PPDU
     * @param reason why reception failed
     */
    void RxDropCb(Ptr<const WifiPpdu> ppdu, WifiPhyRxfailureReason reason);

    /**
     * Frame exchange manager trace sink, counting DS-CTS frames that decoded successfully.
     *
     * @param now when the frame was decoded
     */
    void DsCtsRxCb(Time now);

    /**
     * Fold one DS-CTS observation into the burst timeline. Frames less than the burst gap
     * apart belong to the same Stage-1 attempt: with dual DS-CTS a single attempt puts two
     * frames on air, and the second one exists precisely to survive a collision that destroys
     * the first, so a burst only counts as lost if neither frame decoded.
     *
     * @param decoded whether this DS-CTS was decoded or dropped
     */
    void FeedDsCtsEvent(bool decoded);

    /** Close the burst currently being accumulated, if any. */
    void CloseBurst();

    /** Discard everything observed so far and start a fresh observation period. */
    void ResetObservations();

    Ptr<ApWifiMac> m_apMac;                 //!< the AP being controlled
    Ptr<WifiPhy> m_phy;                     //!< the AP PHY of link 0
    Ptr<QosFrameExchangeManager> m_fem;     //!< the AP frame exchange manager of link 0
    EventId m_stepEvent;                    //!< the next scheduled control step
    uint32_t m_stepCount{0};                //!< number of control steps taken

    // Observation accumulators, cleared at the end of every step.
    Time m_busyAccum{0};             //!< time the channel was not idle this period
    uint32_t m_framesDecoded{0};     //!< DS-CTS frames decoded this period
    uint32_t m_framesDropped{0};     //!< DS-CTS frames dropped this period
    uint32_t m_bursts{0};            //!< DS-CTS bursts observed this period
    uint32_t m_burstsLost{0};        //!< bursts of which no frame decoded
    uint32_t m_curBurstFrames{0};    //!< frames seen in the burst being accumulated
    uint32_t m_curBurstDecoded{0};   //!< of which decoded
    Time m_lastDsCtsEvent{0};        //!< when the last DS-CTS observation arrived
    std::map<Mac48Address, uint32_t> m_lastLliByAddr; //!< per-station LLI count at last step
    std::set<Mac48Address> m_provenPedcaStas; //!< stations proven to be running P-EDCA

    PedcaTheta m_initialTheta;  //!< parameters the stations start out with
    uint8_t m_cwds{0};          //!< current BSS-wide CWds

    // Attributes.
    PedcaPolicy m_policy{PedcaPolicy::KDRIVEN}; //!< which rule set to apply
    int32_t m_kOverride{-1};       //!< force k instead of estimating it; negative = estimate
    double m_qsrcIntercept{0.5};   //!< QSRC law intercept
    double m_qsrcSlope{0.2};       //!< QSRC law slope per P-EDCA station
    uint8_t m_cwdsKDriven{1};      //!< CWds the k-driven law advertises
    uint32_t m_psrc3MaxK{10};      //!< largest k that still gets PSRC 3
    uint32_t m_psrc2MaxK{22};      //!< largest k that still gets PSRC 2
    PedcaTheta m_fixedTheta{};     //!< the triple pushed by the FIXED policy
    double m_overheadHigh{0.06};   //!< Stage-1 airtime above which QSRC is raised
    double m_overheadLow{0.03};    //!< Stage-1 airtime below which QSRC may be lowered
    double m_urgencyHigh{0.15};    //!< LLI ratio above which stations count as short of budget
    double m_urgencyLow{0.05};     //!< LLI ratio below which the medium can be reclaimed
    uint8_t m_qsrcFloor{1};        //!< QSRC never goes below this; 0 is bad at every load
    Time m_burstCost{MicroSeconds(185)}; //!< medium time one Stage-1 burst costs
    uint8_t m_qsrcLoad{2};         //!< the QSRC the load-driven loop currently holds
    uint8_t m_burstAdaptiveQsrc{2}; //!< QSRC held by the burst-adaptive loop
    uint8_t m_lastBurstQsrcPrior{2}; //!< previous population prior, for bucket transitions
    bool m_burstEwmaReady{false};  //!< whether the burst-adaptive EWMAs have a sample
    bool m_burstCollisionEwmaReady{false}; //!< whether DS-CTS loss has enough samples
    double m_overheadEwma{0.0};    //!< filtered Stage-1 airtime fraction
    double m_urgencyEwma{0.0};     //!< filtered fraction of LLI-marked VO frames
    double m_collisionEwma{0.0};   //!< filtered DS-CTS burst-loss fraction
    uint32_t m_burstUpVotes{0};    //!< consecutive overload observations
    uint32_t m_burstDownVotes{0};  //!< consecutive safe-but-urgent observations
    uint32_t m_lastVoRx{0};        //!< AC_VO receive count at the previous step
    uint32_t m_lastLliTotal{0};    //!< BSS-wide LLI count at the previous step
    double m_burstEwmaAlpha{0.25}; //!< weight of the newest burst-adaptive observation
    double m_burstOverheadHigh{0.12}; //!< Stage-1 airtime that indicates real overload
    double m_burstOverheadLow{0.09}; //!< headroom allowing q5 to retreat to q4
    double m_burstCollisionHigh{0.50}; //!< severe DS-CTS loss that asks for throttling
    uint32_t m_burstMinBursts{20}; //!< minimum DS-CTS bursts for a collision decision
    uint32_t m_burstUpHoldPeriods{2}; //!< overload periods required before QSRC is raised
    uint32_t m_burstDownHoldPeriods{5}; //!< headroom periods required before QSRC is lowered
    uint32_t m_burstSmallKMax{8};  //!< largest k assigned the small-population QSRC prior
    uint32_t m_burstMediumKMax{22}; //!< largest k assigned the medium-population QSRC prior
    uint8_t m_burstQsrcSmall{3};   //!< QSRC prior for a small P-EDCA population
    uint8_t m_burstQsrcMedium{4};  //!< QSRC prior for a medium P-EDCA population
    uint8_t m_burstQsrcLarge{4};   //!< QSRC prior for a large P-EDCA population
    uint8_t m_burstQsrcFloor{2};   //!< hard floor; keeps burst-adaptive out of QSRC 0/1
    Time m_period{MilliSeconds(100)};  //!< control period
    Time m_dsCtsBurstGap{MicroSeconds(100)}; //!< gap above which a DS-CTS starts a new burst
    double m_busyHigh{0.85};           //!< busy fraction above which the congestion gate fires
    double m_collHigh{0.90};           //!< collision rate above which CWds jumps to its maximum
    double m_collLow{0.30};            //!< collision rate below which CWds drops to zero
    uint32_t m_lliHigh{5};             //!< per-station LLI count that marks delay pressure
    double m_bsrPerStaHigh{8.0};       //!< per-station buffer status that marks queue pressure
    uint8_t m_cwdsMax{PEDCA_CWDS_MAX}; //!< largest CWds the controller may choose
    uint8_t m_qsrcAggressive{1};       //!< QSRC threshold of the aggressive corner
    uint8_t m_psrcAggressive{3};       //!< PSRC limit of the aggressive corner
    uint8_t m_qsrcConservative{5};     //!< QSRC threshold of the conservative corner
    uint8_t m_psrcConservative{1};     //!< PSRC limit of the conservative corner
    bool m_useBurstCollRate{true};     //!< whether to decide on burst- or frame-level collisions

    /// Per-step trace: time, kHat, cwds, qsrc, psrc, nAggressive, nConservative, nUnchanged,
    /// busyFrac, collRate, collRateFrame, dsCtsFrames, dsCtsBursts, bsrSum, lliCount, anyChanged
    TracedCallback<Time,
                   uint32_t,
                   uint8_t,
                   uint8_t,
                   uint8_t,
                   uint32_t,
                   uint32_t,
                   uint32_t,
                   double,
                   double,
                   double,
                   uint32_t,
                   uint32_t,
                   uint32_t,
                   uint32_t,
                   bool>
        m_controlStepTrace;
};

} // namespace ns3

#endif /* PEDCA_CONTROLLER_H */
