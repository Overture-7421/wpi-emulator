#pragma once

// Game values: named numbers a game plugin publishes every tick (fuel in the robot, ...) so that generic
// core features (sensors, see config/sensor.h) can be driven by game state without knowing the season.
// Declared by IGameModule::contributeGameValues(), read by IGameState::gameValue(id).
// Not to be confused with device signals (devices/signals.h) or HUD keys (presentation, not per tick).

#include <string>

#include "wpiemu/sdk/config/property_schema.h"

namespace wpiemu::sdk {

struct GameValueInfo {
    std::string id;           // prefixed with the plugin id: "game_2026.fuel.count"
    std::string label;        // UI text
    std::string description;
    PropertyUnit unit = PropertyUnit::None;
};

}  // namespace wpiemu::sdk
