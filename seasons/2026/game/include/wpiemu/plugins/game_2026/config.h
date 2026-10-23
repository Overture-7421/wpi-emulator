#pragma once

// Plugin section "game_2026" of robot.json: parsing, defaults and the PropertySchema the generic
// inspector renders, all from one sdk::ConfigBindings table (config.cpp). Per-intake / per-shooter parameters are NOT
// here: they are properties of the robot zones (fuel_zones.h) so several of each can exist.

#include <string>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/core/value.h"

namespace wpiemu::plugins::game_2026 {

// Section schema version (the SDK stays at version 1 until release, docs/CONTRIBUTING.md: no migration).
inline constexpr int kConfigSchemaVersion = 1;

enum class FirstInactiveMode { FromAutoResult, Red, Blue };
enum class HumanPlayerMode { Off, Timed };

struct FuelPhysicsConfig {
    float massKg = 0.215f;
    float friction = 0.6f;
    float restitution = 0.55f;
    float rollingResistance = 0.02f;
    bool sleeping = true;
};

struct GameConfig {
    // fuel meter
    int capacity = 30;
    int preloadFuel = 8;
    // initial layout (field staging)
    int neutralFuel = 360;
    int depotFuel = 24;          // per depot
    int outpostChuteFuel = 24;   // per outpost, held as a stock (not as bodies) until a human player feeds it
    int maxFuelOnField = 800;
    FuelPhysicsConfig fuel;
    float shotGhostS = 0.15f;    // shot fuel ignores the robot and other shot fuel for this long
    // rules
    bool enforceHubShifts = true;
    FirstInactiveMode firstInactive = FirstInactiveMode::FromAutoResult;
    double hubProcessingDelayS = 1.0;
    bool recycleScoredFuel = true;
    HumanPlayerMode humanPlayer = HumanPlayerMode::Off;
    double humanPlayerPeriodS = 2.0;
};

// All plugin config properties with defaults, ranges and groups.
sdk::PluginConfigSchema configSchema();

// Missing keys take defaults; out-of-range values are clamped to the schema range. Never fails.
GameConfig parseConfig(const sdk::Value& data);
// Every key, in schema order.
sdk::Value toValue(const GameConfig& config);

}  // namespace wpiemu::plugins::game_2026
