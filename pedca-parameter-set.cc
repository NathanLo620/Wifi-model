/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * P-EDCA Parameter Set information element (adaptive P-EDCA closed loop).
 */

#include "pedca-parameter-set.h"

namespace ns3
{

/// Octets occupied by one parameter row on the air: AID (2) + CWds + QSRC + PSRC.
static constexpr uint16_t PEDCA_ENTRY_SIZE = 5;

WifiInformationElementId
PedcaParameterSet::ElementId() const
{
    return IE_PEDCA_PARAMETER_SET;
}

void
PedcaParameterSet::SetUpdateCount(uint8_t updateCount)
{
    m_updateCount = updateCount;
}

uint8_t
PedcaParameterSet::GetUpdateCount() const
{
    return m_updateCount;
}

void
PedcaParameterSet::SetEntries(std::vector<PedcaStaEntry> entries)
{
    m_entries = std::move(entries);
}

const std::vector<PedcaStaEntry>&
PedcaParameterSet::GetEntries() const
{
    return m_entries;
}

std::optional<PedcaStaEntry>
PedcaParameterSet::GetEntryFor(uint16_t aid) const
{
    for (const auto& entry : m_entries)
    {
        if (entry.aid == aid)
        {
            return entry;
        }
    }
    return std::nullopt;
}

uint16_t
PedcaParameterSet::GetInformationFieldSize() const
{
    return 1 + PEDCA_ENTRY_SIZE * static_cast<uint16_t>(m_entries.size());
}

void
PedcaParameterSet::SerializeInformationField(Buffer::Iterator start) const
{
    start.WriteU8(m_updateCount);
    for (const auto& entry : m_entries)
    {
        start.WriteHtolsbU16(entry.aid);
        start.WriteU8(entry.cwds);
        start.WriteU8(entry.qsrcThreshold);
        start.WriteU8(entry.psrcLimit);
    }
}

uint16_t
PedcaParameterSet::DeserializeInformationField(Buffer::Iterator start, uint16_t length)
{
    Buffer::Iterator i = start;
    m_entries.clear();

    if (length < 1)
    {
        return length;
    }

    m_updateCount = i.ReadU8();

    uint16_t remaining = length - 1;
    while (remaining >= PEDCA_ENTRY_SIZE)
    {
        PedcaStaEntry entry;
        entry.aid = i.ReadLsbtohU16();
        entry.cwds = i.ReadU8();
        entry.qsrcThreshold = i.ReadU8();
        entry.psrcLimit = i.ReadU8();
        m_entries.push_back(entry);
        remaining -= PEDCA_ENTRY_SIZE;
    }
    return length;
}

void
PedcaParameterSet::Print(std::ostream& os) const
{
    os << "P-EDCA Parameter Set: UpdateCount=" << +m_updateCount
       << " Entries=" << m_entries.size();
}

} // namespace ns3
