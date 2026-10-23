#pragma once

// Numbers the 2026 game state publishes each tick live in seasons/2026/game/README.md.
// IGameModule::contributeGameValues defines the shared contract (docs/reference/game-plugin.md).
// Read back through Game2026State::gameValue.

#include <vector>

#include "wpiemu/sdk/game/game_value.h"

namespace wpiemu::plugins::game_2026 {

namespace game_value {
inline constexpr const char* kFuelCount = "game_2026.fuel.count";        // fuel currently held by the robot
inline constexpr const char* kFuelCapacity = "game_2026.fuel.capacity";  // robot fuel capacity
inline constexpr const char* kFuelFraction = "game_2026.fuel.fraction";  // count / capacity, 0..1
}  // namespace game_value

std::vector<sdk::GameValueInfo> gameValueInfos();

}  // namespace wpiemu::plugins::game_2026
