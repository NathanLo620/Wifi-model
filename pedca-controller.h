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

namespace ns3
{

class ApWifiMac;
class QosFrameExchangeManager;
class WifiNetDevice;
class WifiPhy;
class WifiPpdu;

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

    PedcaTheta m_initialTheta;  //!< parameters the stations start out with
    uint8_t m_cwds{0};          //!< current BSS-wide CWds

    // Attributes.
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

    /// Per-step trace: time, cwds, nAggressive, nConservative, nUnchanged, busyFrac,
    /// collRate, collRateFrame, dsCtsFrames, dsCtsBursts, bsrSum, lliCount, anyChanged
    TracedCallback<Time,
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
