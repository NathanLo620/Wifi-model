/*
 * Copyright (c) 2020 Universita' degli Studi di Napoli Federico II
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#ifndef QOS_FRAME_EXCHANGE_MANAGER_H
#define QOS_FRAME_EXCHANGE_MANAGER_H

#include "frame-exchange-manager.h"

#include <optional>
#include <vector>
#include <string>

namespace ns3
{

/**
 * @ingroup wifi
 *
 * QosFrameExchangeManager handles the frame exchange sequences
 * for QoS stations.
 * Note that Basic Block Ack is not supported.
 */
class QosFrameExchangeManager : public FrameExchangeManager
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    QosFrameExchangeManager();
    ~QosFrameExchangeManager() override;

    bool StartTransmission(Ptr<Txop> edca, MHz_u allowedWidth) override;

    // P-EDCA Detailed Trace Getters
    uint32_t GetDsCtsCount() const { return m_dsCtsCount; }
    uint32_t GetStage2EntryCount() const { return m_stage2EntryCount; }
    uint32_t GetStage2TxStartCount() const { return m_stage2TxStartCount; }
    uint32_t GetPedcaSuccessCount() const { return m_pedcaSuccessCount; }
    uint32_t GetEdcaVoSuccessCount() const { return m_edcaVoSuccessCount; }
    uint32_t GetPedcaFailRtsCtsTimeout() const { return m_pedcaFailRtsCtsTimeout; }
    uint32_t GetPedcaFailRtsCollision() const { return m_pedcaFailRtsCollision; }
    uint32_t GetPedcaFailTimingExpired() const { return m_pedcaFailTimingExpired; }
    uint32_t GetPedcaFailDeferral() const { return m_pedcaFailDeferral; }

    /** Set CWds (Stage-1 contention window).  0=immediate, 1=random[0,1], … */
    void SetCwds(uint32_t cwds) { m_cwds = cwds; }
    /** Set dot11PEDCARetryThreshold (QSRC must reach this value to trigger P-EDCA). */
    void SetQsrc(uint16_t qsrc) { m_qsrc_threshold = qsrc; }
    /** Set dot11PEDCAConsecutiveAttempt (max consecutive P-EDCA attempts per QSRC cycle). */
    void SetPsrc(uint8_t psrc)  { m_psrc_limit = psrc; }
    uint16_t GetQsrcThreshold() const { return m_qsrc_threshold; }
    uint8_t  GetPsrcLimit()     const { return m_psrc_limit; }

    // Per-attempt P-EDCA record for backoff vs failure analysis
    struct PedcaAttemptRecord
    {
        double dsCtsEndUs;       //!< Time when DS-CTS TX ended (microseconds)
        double gapUs;            //!< Effective gap from DS-CTS end to TX start (microseconds)
        int    backoffSlots;     //!< Reconstructed backoff slots = (gap-AIFS)/slotTime; -1 if unknown
        std::string outcome;     //!< SUCCESS / RTS_CTS_TIMEOUT / RTS_COLLISION / TIMING_EXPIRED / DEFERRAL
    };
    const std::vector<PedcaAttemptRecord>& GetPedcaAttempts() const { return m_pedcaAttempts; }

    /**
     * Recompute the protection and acknowledgment methods to use if the given MPDU
     * is added to the frame being built (as described by the given TX parameters)
     * and check whether the duration of the frame exchange sequence (including
     * protection and acknowledgment) does not exceed the given available time.
     * The protection and acknowledgment methods held by the given TX parameters are
     * only updated if the given MPDU can be added.
     *
     * @param mpdu the MPDU to add to the frame being built
     * @param txParams the TX parameters describing the frame being built
     * @param availableTime the time limit on the frame exchange sequence
     * @return true if the given MPDU can be added to the frame being built
     */
    bool TryAddMpdu(Ptr<const WifiMpdu> mpdu, WifiTxParameters& txParams, Time availableTime) const;

    /**
     * Check whether the given MPDU can be added to the frame being built (as described
     * by the given TX parameters) without violating the given constraint on the
     * PPDU transmission duration.
     *
     * @param mpdu the MPDU to add to the frame being built
     * @param txParams the TX parameters describing the frame being built
     * @param ppduDurationLimit the time limit on the PPDU transmission duration
     * @return true if the given MPDU can be added to the frame being built
     */
    virtual bool IsWithinLimitsIfAddMpdu(Ptr<const WifiMpdu> mpdu,
                                         const WifiTxParameters& txParams,
                                         Time ppduDurationLimit) const;

    /**
     * Check whether the transmission time of the frame being built (as described
     * by the given TX parameters) does not exceed the given PPDU duration limit
     * if the size of the PSDU addressed to the given receiver becomes
     * <i>ppduPayloadSize</i>. Also check that the PSDU size does not exceed the
     * max PSDU size for the modulation class being used.
     *
     * @param ppduPayloadSize the new PSDU size
     * @param receiver the MAC address of the receiver of the PSDU
     * @param txParams the TX parameters describing the frame being built
     * @param ppduDurationLimit the limit on the PPDU duration
     * @return true if the constraints on the PPDU duration limit and the maximum PSDU size are met
     */
    virtual bool IsWithinSizeAndTimeLimits(uint32_t ppduPayloadSize,
                                           Mac48Address receiver,
                                           const WifiTxParameters& txParams,
                                           Time ppduDurationLimit) const;

    /**
     * Create an alias of the given MPDU for transmission by this Frame Exchange Manager.
     * This is required by 11be MLDs to support translation of MAC addresses. For single
     * link devices, the given MPDU is simply returned.
     *
     * @param mpdu the given MPDU
     * @return the alias of the given MPDU for transmission on this link
     */
    virtual Ptr<WifiMpdu> CreateAliasIfNeeded(Ptr<WifiMpdu> mpdu) const;

    /**
     * @return the TXOP holder (if any)
     */
    std::optional<Mac48Address> GetTxopHolder() const;

  protected:
    void DoDispose() override;

    void ReceiveMpdu(Ptr<const WifiMpdu> mpdu,
                     RxSignalInfo rxSignalInfo,
                     const WifiTxVector& txVector,
                     bool inAmpdu) override;
    void PreProcessFrame(Ptr<const WifiPsdu> psdu, const WifiTxVector& txVector) override;
    void PostProcessFrame(Ptr<const WifiPsdu> psdu, const WifiTxVector& txVector) override;
    void NavResetTimeout() override;
    void UpdateNav(const WifiMacHeader& hdr,
                   const WifiTxVector& txVector,
                   const Time& surplus = Time{0}) override;
    Time GetFrameDurationId(const WifiMacHeader& header,
                            uint32_t size,
                            const WifiTxParameters& txParams,
                            Ptr<Packet> fragmentedPacket) const override;
    Time GetRtsDurationId(const WifiTxVector& rtsTxVector,
                          Time txDuration,
                          Time response) const override;
    Time GetCtsToSelfDurationId(const WifiTxVector& ctsTxVector,
                                Time txDuration,
                                Time response) const override;
    void TransmissionSucceeded() override;
    void TransmissionFailed(bool forceCurrentCw = false) override;
    void ForwardMpduDown(Ptr<WifiMpdu> mpdu, WifiTxVector& txVector) override;
    void ReceivedMacHdr(const WifiMacHeader& macHdr,
                        const WifiTxVector& txVector,
                        Time psduDuration) override;

    /**
     * Request the FrameExchangeManager to start a frame exchange sequence.
     *
     * @param edca the EDCA that gained channel access
     * @param txopDuration the duration of a TXOP. This value is only used when a
     *                     new TXOP is started (and hence the TXOP limit for the
     *                     given EDCAF is non-zero)
     * @return true if a frame exchange sequence was started, false otherwise
     */
    virtual bool StartTransmission(Ptr<QosTxop> edca, Time txopDuration);

    /**
     * Start a frame exchange (including protection frames and acknowledgment frames
     * as needed) that fits within the given <i>availableTime</i> (if different than
     * Time::Min()).
     *
     * @param edca the EDCAF which has been granted the opportunity to transmit
     * @param availableTime the amount of time allowed for the frame exchange. Pass
     *                      Time::Min() in case the TXOP limit is null
     * @param initialFrame true if the frame being transmitted is the initial frame
     *                     of the TXOP. This is used to determine whether the TXOP
     *                     limit can be exceeded
     * @return true if a frame exchange is started, false otherwise
     */
    virtual bool StartFrameExchange(Ptr<QosTxop> edca, Time availableTime, bool initialFrame);

    /**
     * Perform a PIFS recovery as a response to transmission failure within a TXOP.
     * If the carrier sense indicates that the medium is idle, continue the TXOP.
     * Otherwise, release the channel.
     *
     * @param forceCurrentCw whether to force the contention window to stay equal to the current
     *                       value if PIFs recovery fails (normally, contention window is updated)
     */
    void PifsRecovery(bool forceCurrentCw);

    /**
     * Send a CF-End frame to indicate the completion of the TXOP, provided that
     * the remaining duration is long enough to transmit this frame.
     *
     * @return true if a CF-End frame was sent, false otherwise
     */
    virtual bool SendCfEndIfNeeded();

    void SetWifiPhy(Ptr<WifiPhy> phy) override;
    void ResetPhy() override;

    /**
     * Determine the holder of the TXOP, if possible, based on the received frame
     *
     * @param hdr the MAC header of an MPDU included in the received PSDU
     * @param txVector TX vector of the received PSDU
     * @return the holder of the TXOP, if one was found
     */
    virtual std::optional<Mac48Address> FindTxopHolder(const WifiMacHeader& hdr,
                                                       const WifiTxVector& txVector);

    /**
     * Clear the TXOP holder if the NAV counted down to zero (includes the case of NAV reset).
     */
    virtual void ClearTxopHolderIfNeeded();

    Ptr<QosTxop> m_edca;                      //!< the EDCAF that gained channel access
    std::optional<Mac48Address> m_txopHolder; //!< MAC address of the TXOP holder
    bool m_setQosQueueSize;                   /**< whether to set the Queue Size subfield of the
                                                   QoS Control field of QoS data frames */
    bool m_protectSingleExchange; /**< true if the Duration/ID field in frames establishing
                                     protection only covers the immediate frame exchange instead of
                                     rest of the TXOP limit when the latter is non-zero */
    Time m_singleExchangeProtectionSurplus; /**< additional time to protect beyond end of the
                                               immediate frame exchange in case of non-zero TXOP
                                               limit when a single frame exchange is protected */

  private:
    /**
     * Set the TXOP holder, if needed, based on the received frame
     *
     * @param hdr the MAC header of the received PSDU
     * @param txVector TX vector of the received PSDU
     */
    void SetTxopHolder(const WifiMacHeader& hdr, const WifiTxVector& txVector);

    /**
     * @brief Resume standard EDCA access for AC_BE, AC_BK, AC_VI.
     */
    void ResumePedcaSuspendedACs();

    /**
     * Cancel the PIFS recovery event and have the EDCAF attempting PIFS recovery
     * release the channel.
     */
    void CancelPifsRecovery();

    bool m_initialFrame;         //!< true if transmitting the initial frame of a TXOP
    bool m_pifsRecovery;         //!< true if performing a PIFS recovery after failure
    EventId m_pifsRecoveryEvent; //!< event associated with an attempt of PIFS recovery
    Ptr<Txop> m_edcaBackingOff;  //!< channel access function that invoked backoff during TXOP

    // P-EDCA state variables
    bool m_pedcaPending{false};              //!< True if DS-CTS was sent and we are in P-EDCA backoff
    
    // P-EDCA PSRC/QSRC counters (per 802.11bn spec)
    uint8_t m_psrc{0};                       //!< P-EDCA STA Retry Counter (consecutive DS-CTS attempts)
    uint16_t m_qsrc{0};                      //!< Queue Size Retry Counter (tracks VO retry conditions)
    uint32_t m_cwds{0};                      //!< CWds: Stage-1 contention window (0 = ASAP, 1+ = random backoff)

    // P-EDCA thresholds — runtime-configurable (set via SetQsrc/SetPsrc before sim start,
    // or dynamically from AP via future signalling).  Defaults match 802.11be draft D1.1.
    uint16_t m_qsrc_threshold{2};   //!< dot11PEDCARetryThreshold   (QSRC must reach this to trigger P-EDCA)
    uint8_t  m_psrc_limit{1};       //!< dot11PEDCAConsecutiveAttempt (max consecutive P-EDCA attempts)
    
    // P-EDCA timing tracking for verification
    Time m_pedcaCtsTxEnd{0};  //!< DS-CTS transmission end time for timing verification

    // P-EDCA PhyTxEnd trace helpers (zero-delay Stage 2 entry)
    bool m_pedcaTxEndPending{false};  //!< True while PhyTxEnd is connected waiting for our DS-CTS
    Ptr<QosTxop> m_pedcaEdca;         //!< Saved AC_VO Txop for use inside PedcaPhyTxEndCallback

    /**
     * PhyTxEnd trace callback: fires at the exact moment our DS-CTS finishes TX.
     * Replaces the polling-loop scheduler so Stage 2 entry has zero delay after CTS TX end.
     */
    void PedcaPhyTxEndCallback(Ptr<const Packet> pkt);

    /**
     * Deferred Stage 2 entry scheduled by PedcaPhyTxEndCallback via ScheduleNow so that
     * the call runs after WifiPhy::TxDone returns (avoids re-entrant MAC/PHY calls).
     */
    void PedcaStage2Enter();

    // P-EDCA Stage 2 collision tracking
    bool m_pedcaStage2Active{false};  //!< True when in P-EDCA Stage 2 contention
    TracedCallback<Ptr<const Packet>> m_pedcaTxTrace;      //!< Trace for P-EDCA Stage 2 transmissions
    TracedCallback<Ptr<const Packet>> m_edcaTxTrace;       //!< Trace for all EDCA transmissions
    TracedCallback<Ptr<const Packet>> m_pedcaAttemptTrace;  //!< Trace for P-EDCA Stage 1 attempts (DS-CTS sent)

    // ── P-EDCA Detailed Trace Counters ──
    // Failure reason counters
    uint32_t m_pedcaFailRtsCtsTimeout{0};   //!< RTS sent during Stage 2 but AP didn't reply CTS
    uint32_t m_pedcaFailRtsCollision{0};    //!< RTS collision during Stage 2 (two P-EDCA STAs)
    uint32_t m_pedcaFailTimingExpired{0};   //!< P-EDCA 77us window expired (legacy steal / slow backoff)
    uint32_t m_pedcaFailDeferral{0};        //!< P-EDCA deferred (medium busy before DS-CTS)
    // Event counters
    uint32_t m_dsCtsCount{0};              //!< Number of DS-CTS frames sent
    uint32_t m_stage2EntryCount{0};        //!< Number of times Stage 2 contention entered
    uint32_t m_stage2TxStartCount{0};      //!< Number of times Stage 2 data TX started
    // Success counters
    uint32_t m_pedcaSuccessCount{0};       //!< P-EDCA Stage 2 TX succeeded (ACK received)
    uint32_t m_edcaVoSuccessCount{0};      //!< Normal EDCA VO TX succeeded

    // Per-attempt log: gap and outcome of each P-EDCA Stage 2 attempt
    std::vector<PedcaAttemptRecord> m_pedcaAttempts;
    double m_lastStage2GapUs{-1.0};   //!< Most recent Stage 2 gap (used to back-fill outcome)
    int    m_lastBackoffSlots{-1};    //!< Reconstructed backoff slots for the in-flight attempt
};

} // namespace ns3

#endif /* QOS_FRAME_EXCHANGE_MANAGER_H */
