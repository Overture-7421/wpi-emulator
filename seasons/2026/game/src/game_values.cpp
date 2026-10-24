#include "wpiemu/plugins/game_2026/game_values.h"

namespace wpiemu::plugins::game_2026 {

std::vector<sdk::GameValueInfo> gameValueInfos() {
    return {
        {game_value::kFuelCount, "Fuel in robot", "Number of fuel currently held by the robot.", sdk::PropertyUnit::Count},
        {game_value::kFuelCapacity, "Fuel capacity", "Maximum number of fuel the robot can hold.", sdk::PropertyUnit::Count},
        {game_value::kFuelFraction, "Fuel fill", "Fuel held as a fraction of the capacity (0 to 1).", sdk::PropertyUnit::Ratio},
    };
}

}  // namespace wpiemu::plugins::game_2026
