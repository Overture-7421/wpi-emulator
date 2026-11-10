#pragma once

// The part of a MatchSnapshot that is the same for every client: the quantized robots, the game HUD and the entity groups
// of one telemetry snapshot (sdk::ISimSnapshot). The host builds it once per snapshot instant and then completes it per
// client (acknowledgement fields, entity deltas). Quantization uses sdk/match/quantize.h only, so it is deterministic.

#include <cstdint>
#include <optional>
#include <vector>

#include "wpiemu/core/match/entity_delta.h"
#include "wpiemu/core/match/hud_keys.h"
#include "wpiemu/sdk/match/input_messages.h"
#include "wpiemu/sdk/match/snapshot_messages.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::core::match {

struct WireFrame {
    std::uint64_t tick = 0;
    std::int64_t simTimeUs = 0;
    sdk::match::MatchInfo info;
    // Participant 0 (the host's robot) first, then the remote robots by ascending participant id.
    std::vector<sdk::match::RobotWireState> robots;
    std::vector<sdk::match::HudValue> gameHud;   // numeric and boolean HUD values with a key id, by ascending key id
    // Entity group i of the telemetry snapshot has groupId i; a group without stable ids cannot be delta-encoded and is
    // skipped.
    std::vector<EntityGroupFrame> groups;
    std::vector<RobotAnchor> anchors;            // robot positions in cm, in `robots` order
    std::size_t hudKeysUnknown = 0;              // numeric HUD values not sent because their key is not in the table
    std::size_t groupsWithoutIds = 0;
};

// What the HUD value of a telemetry snapshot becomes on the wire: numbers as themselves, booleans as 0 or 1; text is not
// carried (nullopt).
std::optional<float> hudWireValue(const sdk::Value& value);

WireFrame buildWireFrame(const sdk::ISimSnapshot& snapshot, const HudKeyTable& hudKeys, const sdk::match::MatchInfo& info);

}  // namespace wpiemu::core::match
