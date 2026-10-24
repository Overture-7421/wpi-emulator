#pragma once

#include <vector>

#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::plugins::game_2026 {

// Centres of the fuel staged at match start: the NEUTRAL ZONE block (12 columns, rows along Y,
// centred on the field) and each depot (4 x 6 grid against the alliance wall). Neutral-zone fuel
// is capped at 408 by the config schema; depots at 24. Order is deterministic.
std::vector<sdk::Vec3> initialFuelPositions(const GameConfig& config, const FieldLayout& layout);

}  // namespace wpiemu::plugins::game_2026
