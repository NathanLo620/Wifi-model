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

#include <array>
#include <cstddef>
#include <map>
#include <utility>
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
     * Feedback loop on QSRC alone, around a constant prior.
     *
     * Whether to move is decided by the LLI urgency ratio, the fraction of arriving voice
     * frames already past 70% of their delay bound and so the closest available proxy for
     * the delay tail this policy exists to shorten. Which way to move is decided by Stage-1
     * airtime: raising QSRC makes eligibility more selective and buys medium time back,
     * lowering it admits more stations. Severe DS-CTS burst loss forces the selective
     * direction. All three signals are EWMA filtered, and each direction needs several
     * consecutive periods before it moves, so one noisy 100 ms sample cannot steer.
     *
     * CWds and PSRC are constants: the sweeps make CWds a measured don't-care and pick
     * PSRC 3 in 14 of 15 traffic-model/population cells.
     *
     * QSRC 1 is skipped in both directions: eligibility is conditioned on the immediately
     * preceding EDCA collision, so QSRC 1 preferentially admits exactly the stations that
     * just collided with each other.
     *
     * @note The QSRC prior is a configured constant. An earlier revision tried to pick it
     *       from a runtime burstiness classifier; that was removed because the AP cannot
     *       observe the offered traffic shape. See the measurement in
     *       scratch/delay_pdf/11be/adaptive_compare_v3/POLICY.md.
     */
    BURSTADAPTIVE,
    /**
     * Measure-and-revert hill climbing directly on the delay objective.
     *
     * Every other policy here decides which way to move from a *cost* proxy: Stage-1
     * airtime says how expensive P-EDCA is, not whether the delay tail got shorter, and
     * the relationship between the two is not monotonic. This policy drops the proxy and
     * closes the loop on the objective itself.
     *
     * One trial holds a candidate (QSRC, PSRC) for HillTrialPeriods control periods and
     * averages the LLI urgency ratio over them -- the fraction of arriving voice frames
     * already past 70% of their delay bound, which is the AP's only direct view of the
     * tail. A candidate that beats the incumbent by more than HillDeadband is adopted; one
     * that does not causes the search direction to flip, so the next probe tries the other
     * side. After two consecutive failures the search returns to the incumbent for one
     * trial to re-measure its cost, which keeps a stale baseline from locking the search,
     * and then alternates to the other dimension.
     */
    HILLCLIMB,
    /**
     * Model-predictive selection of QSRC and PSRC (MP-PEDCA).
     *
     * The delay a voice frame sees under threshold T decomposes into two terms that move in
     * opposite directions, and the sum has an interior minimum. Every signal the AP can
     * measure directly is monotone in T, so no local search on a single proxy can find that
     * minimum; instead this policy measures the primitives, predicts both terms for every
     * candidate operating point, and jumps to the predicted argmin.
     *
     *   W_qual(T)   = T * tau0 / (1 - u(T))    stations must fail T times to qualify
     *   W_stage1(T) = C_b / S(T)               reservations collide when many qualify
     *
     * The eligible population scales geometrically in T, because qualifying means failing T
     * EDCA attempts in a row: b(T) = b_obs * p^(T - T_cur), with p the per-attempt failure
     * probability read off the Retry bit. Only the ratio is needed, so the absolute eligible
     * count -- which the AP cannot observe, DS-CTS carrying no transmitter address -- never
     * enters. The Stage-1 success model S is a one-parameter hazard fitted each period to
     * the DS-CTS burst loss actually measured at the current operating point.
     *
     * Nothing here is calibrated offline: a change in population, offered load or traffic
     * shape moves p, b_obs and the fitted hazard, and the predicted argmin moves with them.
     */
    MODELPREDICTIVE,
    /**
     * Solve for the threshold directly, from the retry counters the stations report.
     *
     * LOADDRIVEN and BURSTADAPTIVE regulate the same thing this policy does -- the share of
     * airtime Stage-1 consumes -- and they regulate it by stepping QSRC one notch, waiting
     * a few periods to see what happened, and stepping again. They have to: the AP knows
     * what the current threshold admits, because it can watch DS-CTS go by, but it has no
     * way to know what a different one would admit. Several hundred milliseconds of hunting
     * is the price of that ignorance, and under bursty traffic the answer has moved before
     * the hunt arrives.
     *
     * The stations can simply say. Every voice frame carries a P-EDCA Status Report -- the
     * QSRC and PSRC in force when it was transmitted -- and because both counters clear on
     * success, a delivered frame reports exactly what that delivery cost: how many EDCA
     * attempts were lost first, and how many Stage-1 reservations were spent. Tallied
     * between one beacon and the next, the reports give the AP the distribution of QSRC at
     * success, and from it the survival function
     *
     *     G(T) = Pr[QSRC >= T]
     *
     * which is precisely the fraction of deliveries a threshold T would admit to Stage 1 --
     * measured, over the whole candidate range, with no assumption about its shape. Meeting
     * an airtime budget then stops being a search and becomes an inversion: scale the
     * admitted fraction by however far off budget the measurement is, and read the threshold
     * that admits it off G. One control period instead of five, with the same target band
     * BURSTADAPTIVE was tuned to.
     *
     * The direction has to agree for SrcHoldPeriods consecutive periods before anything
     * moves, and a move is capped at SrcMaxStep notches, so one noisy period cannot throw
     * the threshold across its range. Lowering additionally requires that something is
     * actually short of delay budget: an idle medium is not evidence that P-EDCA would help,
     * and reading it as headroom is what makes an airtime-only rule fire P-EDCA on sparse
     * traffic. PSRC has no loop, walking to its constant target as it does in BURSTADAPTIVE.
     *
     * Piggybacking also fixes the attempt-cycle time the other policies estimate: tau0 can
     * divide medium time by the exact number of attempts, deliveries plus the QSRC they
     * report, instead of by deliveries plus those carrying a set Retry bit, which counts a
     * frame that failed five times exactly once.
     *
     * @note The reported distribution is censored by the threshold currently in force:
     *       deliveries that qualified were helped by P-EDCA, so the mass above T_cur is the
     *       assisted tail and reads slightly light. Candidates below T_cur are therefore
     *       read off uncensored data, and candidates above it conservatively.
     */
    SRCFEEDBACK,
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

    /** @return the LLI urgency ratio measured in the last control period */
    double GetLastUrgency() const { return m_lastUrgency; }

    /** @return the cost of the most recently completed hill-climbing trial */
    double GetHillTrialCost() const { return m_hillTrialCost; }

    /** @return the incumbent cost of the hill-climbing search */
    double GetHillBestCost() const { return m_hillBestCost; }

    /** @return the incumbent (QSRC, PSRC) of the hill-climbing search */
    std::pair<uint8_t, uint8_t> GetHillBest() const { return {m_hillBestQsrc, m_hillBestPsrc}; }

    /** @return how many hill-climbing trials have completed */
    uint32_t GetHillTrials() const { return m_hillTrials; }

    /** @return the filtered per-attempt EDCA failure probability estimate */
    double GetMpP() const { return m_mpP; }

    /** @return the filtered EDCA attempt-cycle time estimate, in seconds */
    double GetMpTau0() const { return m_mpTau0; }

    /** @return the filtered Stage-1 collision hazard per DS-CTS burst */
    double GetMpGamma() const { return m_mpGamma; }

    /** @return the predicted delay of the operating point currently advertised */
    double GetMpPredDelay() const { return m_mpPredDelay; }

    /** @return the Little's law delay estimate used as the model-drift guard */
    double GetMpLittleDelay() const { return m_mpLittleDelay; }

    /** @return the mean QSRC the stations reported in the last control period */
    double GetSrcMeanQsrc() const { return m_srcMeanQsrc; }

    /** @return the mean PSRC the stations reported in the last control period */
    double GetSrcMeanPsrc() const { return m_srcMeanPsrc; }

    /** @return the fraction of reported deliveries that came through a P-EDCA TXOP */
    double GetSrcStage2Frac() const { return m_srcStage2Frac; }

    /** @return the attempt-cycle time estimated from the exact reported attempt count */
    double GetSrcTau0() const { return m_srcTau0; }

    /** @return the admitted fraction the last control step solved for */
    double GetSrcTargetAdmit() const { return m_srcTargetAdmit; }

    /**
     * The urgency ratio restricted to the stations that are actually running P-EDCA.
     *
     * GetLastUrgency() divides LLI-marked frames by *every* station's voice frames, but only
     * a P-EDCA station ever sets LLI, so at low penetration it reads low by roughly the
     * penetration ratio -- at 5 of 30 stations, six times too low. The reports fix both
     * halves: a station that has sent one is proven to be running P-EDCA, and its report
     * count is the per-station voice-frame denominator the ratio actually wants.
     *
     * @return LLI-marked frames over delivered frames, over the reporting stations only
     */
    double GetSrcUrgency() const { return m_srcUrgency; }

    /** @return P-EDCA Status Reports received in the last control period */
    uint32_t GetSrcReports() const { return m_srcReports; }

    /**
     * The measured fraction of deliveries that would qualify for P-EDCA at a threshold.
     *
     * @param qsrc the candidate dot11PEDCARetryThreshold
     * @return Pr[QSRC >= qsrc] over the filtered report histogram
     */
    double SrcSurvival(uint8_t qsrc) const;

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

    /**
     * Step the burst-adaptive QSRC one operating point up or down.
     *
     * QSRC 1 is skipped in both directions: eligibility is conditioned on the immediately
     * preceding EDCA collision, so QSRC 1 preferentially admits exactly the stations that
     * just collided with each other.
     */
    uint8_t BurstQsrcStep(uint8_t qsrc, int direction) const;

    /** Predicted delay of one candidate operating point, and the terms behind it. */
    struct MpPrediction
    {
        uint8_t qsrc{0};
        uint8_t psrc{0};
        double bursts{0.0};  //!< predicted DS-CTS bursts in one period
        double util{0.0};    //!< predicted Stage-1 utilisation
        double wQual{0.0};   //!< predicted qualification delay, seconds
        double wStage1{0.0}; //!< predicted Stage-1 reservation delay, seconds
        double delay{0.0};   //!< their sum
    };

    /**
     * Predict the delay of one candidate operating point from the measured primitives.
     *
     * @param qsrc the candidate QSRC threshold
     * @param psrc the candidate PSRC limit
     * @return the prediction, including the two terms behind it
     */
    MpPrediction MpPredict(uint8_t qsrc, uint8_t psrc) const;

    /**
     * The most aggressive threshold whose admitted fraction stays within a budget.
     *
     * G is non-increasing in T, so this is the smallest T with G(T) <= admit. Returns
     * PEDCA_QSRC_MAX if even the most selective threshold admits more than the budget.
     *
     * @param admit the largest admitted fraction the budget allows
     * @return the threshold to advertise
     */
    uint8_t SrcThresholdFor(double admit) const;

    /**
     * Fold one control period's worth of P-EDCA Status Reports into the filtered histogram
     * and the scalar summaries derived from it.
     *
     * @param busyFrac the fraction of the period the medium was not idle
     */
    void SrcObserve(double busyFrac);


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

    // ---- Hill-climbing search state ----
    /// Which parameter the hill climber is currently probing.
    enum class HillDim
    {
        QSRC,
        PSRC
    };
    /// What the current trial is for.
    enum class HillPhase
    {
        PROBE,  //!< holding a candidate that differs from the incumbent
        REFRESH //!< holding the incumbent to re-measure its cost
    };
    uint32_t m_hillTrialPeriods{6}; //!< control periods averaged into one trial
    double m_hillDeadband{0.03};    //!< relative improvement required to adopt a candidate
    double m_hillCostFloor{0.02};   //!< cost below which the tail is fine and search pauses
    uint8_t m_hillQsrcMin{0};       //!< smallest QSRC the search may propose
    uint8_t m_hillQsrcMax{PEDCA_QSRC_MAX}; //!< largest QSRC the search may propose
    uint8_t m_hillPsrcMin{1};       //!< smallest PSRC the search may propose
    uint8_t m_hillPsrcMax{3};       //!< largest PSRC the search may propose
    uint8_t m_hillQsrc{2};          //!< QSRC currently advertised
    uint8_t m_hillPsrc{1};          //!< PSRC currently advertised
    uint8_t m_hillBestQsrc{2};      //!< incumbent QSRC
    uint8_t m_hillBestPsrc{1};      //!< incumbent PSRC
    double m_hillBestCost{-1.0};    //!< incumbent cost; negative means not measured yet
    double m_hillTrialCost{0.0};    //!< cost of the trial that just finished, for tracing
    double m_hillCostAccum{0.0};    //!< running sum of this trial's per-period urgency
    uint32_t m_hillSamples{0};      //!< periods folded into the current trial
    int m_hillDir{1};               //!< which way the next probe steps
    uint32_t m_hillFailures{0};     //!< consecutive probes that did not improve
    uint32_t m_hillTrials{0};       //!< trials completed, for tracing
    HillDim m_hillDim{HillDim::QSRC};      //!< dimension being probed
    HillPhase m_hillPhase{HillPhase::PROBE}; //!< what this trial is for

    // ---- Model-predictive (MP-PEDCA) state ----
    double m_mpAlpha{0.3};          //!< EWMA weight on the estimated primitives
    double m_mpUtilMax{0.35};       //!< Stage-1 utilisation a candidate may not exceed
    double m_mpHysteresis{0.05};    //!< relative gain needed before switching
    double m_mpPMin{0.05};          //!< clamp on the estimated failure probability
    double m_mpPMax{0.95};          //!< clamp on the estimated failure probability
    uint32_t m_mpMinBursts{10};     //!< bursts needed before the hazard fit is trusted
    uint32_t m_mpWarmupPeriods{5};  //!< periods observed before the first switch
    uint8_t m_mpQsrc{2};            //!< QSRC currently advertised
    uint8_t m_mpPsrc{1};            //!< PSRC currently advertised
    double m_mpP{0.5};              //!< filtered per-attempt EDCA failure probability
    double m_mpTau0{0.0};           //!< filtered EDCA attempt-cycle time, seconds
    double m_mpGamma{0.0};          //!< filtered Stage-1 collision hazard per burst
    double m_mpBursts{0.0};         //!< filtered DS-CTS bursts per period
    double m_mpPredDelay{0.0};      //!< predicted delay of the point being advertised
    double m_mpLittleDelay{0.0};    //!< Little's law delay estimate, for the guard
    bool m_mpReady{false};          //!< whether the filtered primitives have a sample
    uint32_t m_mpPeriods{0};        //!< control periods observed in this policy
    uint32_t m_lastVoRetryRx{0};    //!< Retry-bit count at the previous step

    // ---- Piggybacked retry-report state (SRCFEEDBACK) ----
    /// One more than the largest QSRC a P-EDCA Status Report can carry (4-bit subfield).
    static constexpr std::size_t SRC_QSRC_BINS = 16;
    double m_srcAlpha{0.3};          //!< EWMA weight on the reported histogram
    uint32_t m_srcMinReports{20};    //!< reports needed in a period before it is trusted
    uint32_t m_srcWarmupPeriods{3};  //!< periods observed before the first move
    uint32_t m_srcHoldPeriods{2};    //!< consecutive periods a direction must agree
    uint8_t m_srcMaxStep{2};         //!< notches the threshold may move in one period
    uint8_t m_srcQsrc{2};            //!< QSRC currently advertised
    uint8_t m_srcPsrc{1};            //!< PSRC currently advertised
    std::array<double, SRC_QSRC_BINS> m_srcPmf{}; //!< filtered distribution of reported QSRC
    bool m_srcReady{false};          //!< whether the filtered histogram has a sample
    uint32_t m_srcPeriods{0};        //!< control periods with a usable report sample
    uint32_t m_srcReports{0};        //!< reports received in the last period
    double m_srcMeanQsrc{0.0};       //!< mean reported QSRC over the last period
    double m_srcMeanPsrc{0.0};       //!< mean reported PSRC over the last period
    double m_srcStage2Frac{0.0};     //!< share of deliveries that used a P-EDCA TXOP
    double m_srcTau0{0.0};           //!< attempt-cycle time from the exact attempt count
    double m_srcTargetAdmit{0.0};    //!< admitted fraction the last step solved for
    double m_srcUrgency{0.0};        //!< urgency over the reporting stations only
    uint32_t m_srcReleaseVotes{0};   //!< consecutive periods with nothing short of budget
    std::map<Mac48Address, uint32_t> m_lastSrcReportsByAddr; //!< per-station report count
    std::map<Mac48Address, uint32_t> m_lastSrcLliByAddr;     //!< per-station LLI count
    uint32_t m_srcUpVotes{0};        //!< consecutive periods asking for a stricter threshold
    uint32_t m_srcDownVotes{0};      //!< consecutive periods asking for a looser one
    uint32_t m_lastSrcReports{0};    //!< report count at the previous step
    uint64_t m_lastSrcQsrcSum{0};    //!< reported QSRC sum at the previous step
    uint64_t m_lastSrcPsrcSum{0};    //!< reported PSRC sum at the previous step
    uint32_t m_lastSrcStage2{0};     //!< Stage-2 delivery count at the previous step
    std::array<uint32_t, SRC_QSRC_BINS> m_lastSrcHist{}; //!< histogram at the previous step

    bool m_alignToBeacon{false};     //!< run one control step per beacon interval
    Time m_beaconLead{MilliSeconds(2)}; //!< how far ahead of a beacon a step is placed
    double m_urgencyLow{0.05};     //!< LLI ratio below which the medium can be reclaimed
    uint8_t m_qsrcFloor{1};        //!< QSRC never goes below this; 0 is bad at every load
    Time m_burstCost{MicroSeconds(185)}; //!< medium time one Stage-1 burst costs
    uint8_t m_qsrcLoad{2};         //!< the QSRC the load-driven loop currently holds
    uint8_t m_burstAdaptiveQsrc{2}; //!< QSRC held by the burst-adaptive loop
    bool m_burstEwmaReady{false};  //!< whether the burst-adaptive EWMAs have a sample
    bool m_burstCollisionEwmaReady{false}; //!< whether DS-CTS loss has enough samples
    double m_overheadEwma{0.0};    //!< filtered Stage-1 airtime fraction
    double m_urgencyEwma{0.0};     //!< filtered fraction of LLI-marked VO frames
    double m_collisionEwma{0.0};   //!< filtered DS-CTS burst-loss fraction
    uint8_t m_burstCwds{0};        //!< CWds advertised by the burst-adaptive policy
    uint32_t m_burstDecayPeriods{8}; //!< quiet periods before QSRC decays towards its prior
    uint32_t m_burstDecayVotes{0}; //!< consecutive quiet observations
    uint32_t m_burstUpVotes{0};    //!< consecutive overload observations
    uint32_t m_burstDownVotes{0};  //!< consecutive safe-but-urgent observations
    uint32_t m_lastVoRx{0};        //!< AC_VO receive count at the previous step
    uint32_t m_lastLliTotal{0};    //!< BSS-wide LLI count at the previous step
    double m_burstEwmaAlpha{1.0};  //!< weight of the newest observation; 1 = no filter
    double m_burstOverheadHigh{0.085}; //!< Stage-1 airtime that indicates real overload
    double m_burstOverheadLow{0.055}; //!< headroom allowing more stations to be admitted
    double m_burstCollisionHigh{0.50}; //!< severe DS-CTS loss that asks for throttling
    uint32_t m_burstMinBursts{20}; //!< minimum DS-CTS bursts for a collision decision
    uint32_t m_burstUpHoldPeriods{2}; //!< overload periods required before QSRC is raised
    uint32_t m_burstDownHoldPeriods{5}; //!< headroom periods required before QSRC is lowered
    uint8_t m_burstQsrcPrior{4};    //!< QSRC the loop returns to when nothing asks for a move
    uint8_t m_burstQsrcFloor{0};    //!< hard floor of the burst-adaptive loop
    uint8_t m_burstPsrc{3};         //!< PSRC the policy walks towards from its start value
    uint8_t m_burstAdaptivePsrc{1}; //!< PSRC currently advertised
    double m_lastUrgency{0.0};      //!< LLI urgency ratio of the last control period
    bool m_burstAvoidQsrc1{true};   //!< skip QSRC 1 when stepping through the range
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
