#include "wpiemu/plugins/game_2026/config.h"

#include <algorithm>

#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/sdk/config/config_bindings.h"
#include "wpiemu/sdk/config/property_builders.h"

namespace wpiemu::plugins::game_2026 {

namespace {

using sdk::PropertyUnit;
namespace props = sdk::props;

constexpr const char* kGroupMeter = "Fuel meter";
constexpr const char* kGroupStaging = "Field staging";
constexpr const char* kGroupPhysics = "Fuel physics";
constexpr const char* kGroupRules = "Match rules";

// Each key is declared here once: the schema, parseConfig and toValue all follow this table.
sdk::ConfigBindings<GameConfig> bindings() {
    sdk::ConfigBindings<GameConfig> b;
    b.add(props::integer("capacity", "Fuel capacity", kGroupMeter, 30, 0, 500,
                         "Maximum fuel the robot holds (the fuel meter). Intakes stop absorbing when full."),
          &GameConfig::capacity);
    b.add(props::integer("preloadFuel", "Preload", kGroupMeter, dim::kMaxPreload, 0, 500,
                         "Fuel in the robot at match start (manual 6.3.4 C allows up to 8)."),
          &GameConfig::preloadFuel);

    b.add(props::integer("neutralFuel", "Neutral zone fuel", kGroupStaging, 360, 0, dim::kMaxNeutralFuel,
                         "Fuel staged in the neutral zone (manual 6.3.4: 360-408)."),
          &GameConfig::neutralFuel);
    b.add(props::integer("depotFuel", "Fuel per depot", kGroupStaging, dim::kFuelPerDepot, 0,
                         dim::kDepotColumns * dim::kDepotRows, "Fuel staged in each depot (manual 6.3.4 A: 24)."),
          &GameConfig::depotFuel);
    b.add(props::integer("outpostChuteFuel", "Fuel per outpost chute", kGroupStaging, dim::kFuelPerChute, 0, 100,
                         "Fuel held by each outpost chute for the human player (manual 6.3.4 B: 24)."),
          &GameConfig::outpostChuteFuel);
    b.choice(props::choice("humanPlayer", "Human player", kGroupStaging, "off",
                           {{"off", "Off (chute fuel is never released)"}, {"timed", "Timed (one fuel per period)"}},
                           "Whether the human players release chute fuel onto the field."),
             &GameConfig::humanPlayer, {HumanPlayerMode::Off, HumanPlayerMode::Timed});
    b.add(props::real("humanPlayerPeriodS", "Human player period", kGroupStaging, 2.0, 0.1, 60.0, PropertyUnit::Seconds,
                      "Seconds between chute releases while the match is running."),
          &GameConfig::humanPlayerPeriodS);
    b.add(props::integer("maxFuelOnField", "Max fuel bodies", kGroupStaging, 800, 0, 5000,
                         "Safety cap on simultaneously simulated fuel bodies."),
          &GameConfig::maxFuelOnField);

    b.add(props::real("fuelMassKg", "Fuel mass", kGroupPhysics, dim::kFuelMassKg, 0.05, 1.0, PropertyUnit::Kilograms,
                      "Manual 5.10.1: 0.203 - 0.227 kg."),
          &GameConfig::fuel, &FuelPhysicsConfig::massKg);
    b.add(props::real("fuelFriction", "Fuel friction", kGroupPhysics, dim::kFuelFriction, 0.0, 2.0, PropertyUnit::None,
                      "Coulomb friction coefficient of fuel (foam on carpet)."),
          &GameConfig::fuel, &FuelPhysicsConfig::friction);
    b.add(props::real("fuelRestitution", "Fuel restitution", kGroupPhysics, dim::kFuelRestitution, 0.0, 1.0,
                      PropertyUnit::None, "Bounciness of fuel."),
          &GameConfig::fuel, &FuelPhysicsConfig::restitution);
    b.add(props::real("fuelRollingResistance", "Fuel rolling resistance", kGroupPhysics, dim::kFuelRollingResistance,
                      0.0, 1.0, PropertyUnit::None, "Rolling resistance of fuel."),
          &GameConfig::fuel, &FuelPhysicsConfig::rollingResistance);
    b.add(props::flag("fuelSleeping", "Fuel sleeping", kGroupPhysics, true,
                      "Let resting fuel sleep (large performance win; disable only for debugging)."),
          &GameConfig::fuel, &FuelPhysicsConfig::sleeping);
    b.add(props::real("shotGhostS", "Shot ghost time", kGroupPhysics, 0.15, 0.0, 1.0, PropertyUnit::Seconds,
                      "Shot fuel ignores the robot and other just-shot fuel for this long after leaving the shooter."),
          &GameConfig::shotGhostS);

    b.add(props::flag("enforceHubShifts", "Enforce hub shifts", kGroupRules, true,
                      "Apply the active/inactive hub schedule (manual 6.4.1). Off: both hubs score whenever the "
                      "robot is enabled."),
          &GameConfig::enforceHubShifts);
    b.choice(props::choice("firstInactiveHub", "First inactive hub", kGroupRules, "auto",
                           {{"auto", "Auto winner (FMS rule)"}, {"red", "Red"}, {"blue", "Blue"}},
                           "Which alliance's hub is inactive in shift 1. Auto: the alliance that scored more fuel "
                           "in AUTO (ties: random, seeded)."),
             &GameConfig::firstInactive,
             {FirstInactiveMode::FromAutoResult, FirstInactiveMode::Red, FirstInactiveMode::Blue});
    b.add(props::real("hubProcessingDelayS", "Hub processing delay", kGroupRules, 1.0, 0.0, 10.0, PropertyUnit::Seconds,
                      "Time between fuel entering a hub and leaving through its exits."),
          &GameConfig::hubProcessingDelayS);
    b.add(props::flag("recycleScoredFuel", "Recycle scored fuel", kGroupRules, true,
                      "Scored fuel is released from the hub exits into the neutral zone (manual 5.4). Off: it "
                      "is removed."),
          &GameConfig::recycleScoredFuel);
    return b;
}

}  // namespace

sdk::PluginConfigSchema configSchema() {
    sdk::PluginConfigSchema s;
    s.schemaVersion = kConfigSchemaVersion;
    s.properties = bindings().schema();
    s.groupOrder = {kGroupMeter, kGroupStaging, kGroupPhysics, kGroupRules};
    return s;
}

GameConfig parseConfig(const sdk::Value& data) {
    GameConfig c = bindings().decode(data);
    c.preloadFuel = std::min(c.preloadFuel, c.capacity);
    return c;
}

sdk::Value toValue(const GameConfig& c) { return bindings().encode(c); }

}  // namespace wpiemu::plugins::game_2026
