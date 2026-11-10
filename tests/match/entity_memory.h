#pragma once

// The reference CLIENT ENTITY MEMORY of tests is the one the game client uses (core/match/entity_memory.h); this header
// only makes it available as match_test::EntityMemory and adds the comparison helper the host tests share.

#include <cstdint>
#include <map>

#include "wpiemu/core/match/entity_memory.h"
#include "wpiemu/sdk/match/snapshot_messages.h"

namespace match_test {

using EntityMemory = wpiemu::core::match::EntityMemory;

// Every entity of `truth` is shown within `cm` of it and nothing else is shown (a rest-state upsert is exact, so `cm` is the
// rest tolerance).
inline bool sameEntities(const std::map<std::uint32_t, wpiemu::sdk::match::EntityState>& shown,
                         const std::map<std::uint32_t, wpiemu::sdk::match::EntityState>& truth, int cm) {
    if (shown.size() != truth.size()) return false;
    for (const auto& [id, e] : truth) {
        const auto it = shown.find(id);
        if (it == shown.end()) return false;
        const int dx = it->second.xCm - e.xCm, dy = it->second.yCm - e.yCm, dz = it->second.zCm - e.zCm;
        if (dx * dx + dy * dy + dz * dz > cm * cm) return false;
    }
    return true;
}

}  // namespace match_test
