/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * P-EDCA Parameter Set information element (adaptive P-EDCA closed loop).
 */

#ifndef PEDCA_PARAMETER_SET_H
#define PEDCA_PARAMETER_SET_H

#include "wifi-information-element.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace ns3
{

/// Largest CWds an AP may advertise in a P-EDCA Parameter Set.
constexpr uint8_t PEDCA_CWDS_MAX = 2;
/// Largest dot11PEDCARetryThreshold an AP may advertise. Must stay below the MAC frame retry
/// limit (7 by default): QosFrameExchangeManager raises the retry limit to match a larger
/// threshold and never lowers it again.
constexpr uint8_t PEDCA_QSRC_MAX = 5;
/// Smallest dot11PEDCAConsecutiveAttempt an AP may advertise.
constexpr uint8_t PEDCA_PSRC_MIN = 1;
/// Largest dot11PEDCAConsecutiveAttempt an AP may advertise.
constexpr uint8_t PEDCA_PSRC_MAX = 3;

/**
 * @ingroup wifi
 * The P-EDCA parameters controlling one station's Stage-1/Stage-2 channel access.
 *
 * Kept at namespace scope (rather than nested in ApWifiMac) so that the controller
 * and the IE can pass it around without depending on ap-wifi-mac.h.
 */
struct PedcaTheta
{
    uint8_t cwds{0};          //!< CWds: Stage-1 contention window (0 = ASAP)
    uint8_t qsrcThreshold{2}; //!< dot11PEDCARetryThreshold
    uint8_t psrcLimit{1};     //!< dot11PEDCAConsecutiveAttempt

    /// @return whether two parameter triples are equal
    friend bool operator==(const PedcaTheta&, const PedcaTheta&) = default;
};

/**
 * @ingroup wifi
 * One row of the P-EDCA Parameter Set: the parameters destined for a single AID.
 */
struct PedcaStaEntry
{
    uint16_t aid{0};          //!< association ID this row applies to
    uint8_t cwds{0};          //!< CWds
    uint8_t qsrcThreshold{0}; //!< dot11PEDCARetryThreshold
    uint8_t psrcLimit{0};     //!< dot11PEDCAConsecutiveAttempt
};

/**
 * @ingroup wifi
 *
 * The P-EDCA Parameter Set information element, carried in Beacon, Probe Response and
 * Association Response frames by an AP that has PedcaControl enabled. It advertises one
 * parameter row per associated station; each station applies the row matching its own AID.
 *
 * Information field layout (1 + 5*N octets):
 * @verbatim
 *   octet 0     : Update Count (incremented on every push, wraps at 256)
 *   octet 1..5N : N entries of AID (2 octets, LSB first), CWds, QSRC threshold, PSRC limit
 * @endverbatim
 */
class PedcaParameterSet : public WifiInformationElement
{
  public:
    WifiInformationElementId ElementId() const override;

    /**
     * Set the Update Count field.
     *
     * @param updateCount the number of times the AP has pushed a parameter table
     */
    void SetUpdateCount(uint8_t updateCount);
    /** @return the Update Count field */
    uint8_t GetUpdateCount() const;

    /**
     * Replace the parameter table carried by this element.
     *
     * @param entries the per-station parameter rows
     */
    void SetEntries(std::vector<PedcaStaEntry> entries);
    /** @return the per-station parameter rows */
    const std::vector<PedcaStaEntry>& GetEntries() const;

    /**
     * Look up the row destined for a given station.
     *
     * @param aid the association ID to look for
     * @return the matching row, or std::nullopt if the table has no row for this AID
     */
    std::optional<PedcaStaEntry> GetEntryFor(uint16_t aid) const;

  private:
    uint16_t GetInformationFieldSize() const override;
    void SerializeInformationField(Buffer::Iterator start) const override;
    uint16_t DeserializeInformationField(Buffer::Iterator start, uint16_t length) override;
    void Print(std::ostream& os) const override;

    uint8_t m_updateCount{0};              ///< update count
    std::vector<PedcaStaEntry> m_entries;  ///< per-station parameter rows
};

} // namespace ns3

#endif /* PEDCA_PARAMETER_SET_H */
