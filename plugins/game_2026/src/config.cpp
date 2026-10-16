#include "wpiemu/plugins/game_2026/config.h"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <utility>

#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/property_helpers.h"

namespace wpiemu::plugins::game_2026 {

namespace {

using sdk::PropertyUnit;

constexpr const char* kGroupMeter = "Fuel meter";
constexpr const char* kGroupStaging = "Field staging";
constexpr const char* kGroupPhysics = "Fuel physics";
constexpr const char* kGroupRules = "Match rules";

int clampedInt(const sdk::Value* v, const sdk::PropertySpec& spec, int fallback) {
    if (v == nullptr || !v->isNumber()) return fallback;
    const double lo = spec.min.value_or(-1e18), hi = spec.max.value_or(1e18);
    return static_cast<int>(std::clamp(static_cast<double>(v->asInt()), lo, hi));
}

double clampedDouble(const sdk::Value* v, const sdk::PropertySpec& spec, double fallback) {
    if (v == nullptr || !v->isNumber() || std::isnan(v->asDouble())) return fallback;
    return std::clamp(v->asDouble(), spec.min.value_or(-1e18), spec.max.value_or(1e18));
}

bool boolOf(const sdk::Value* v, bool fallback) { return v != nullptr && v->isBool() ? v->asBool() : fallback; }

}  // namespace

sdk::PropertySchema configPropertySchema() {
    sdk::PropertySchema s;
    auto& p = s.properties;
    p.push_back(props::integer("capacity", "Fuel capacity", kGroupMeter, 30, 0, 500,
                               "Maximum fuel the robot holds (the fuel meter). Intakes stop absorbing when full."));
    p.push_back(props::integer("preloadFuel", "Preload", kGroupMeter, dim::kMaxPreload, 0, 500,
                               "Fuel in the robot at match start (manual 6.3.4 C allows up to 8)."));

    p.push_back(props::integer("neutralFuel", "Neutral zone fuel", kGroupStaging, 360, 0, dim::kMaxNeutralFuel,
                               "Fuel staged in the neutral zone (manual 6.3.4: 360-408)."));
    p.push_back(props::integer("depotFuel", "Fuel per depot", kGroupStaging, dim::kFuelPerDepot, 0,
                               dim::kDepotColumns * dim::kDepotRows, "Fuel staged in each depot (manual 6.3.4 A: 24)."));
    p.push_back(props::integer("outpostChuteFuel", "Fuel per outpost chute", kGroupStaging, dim::kFuelPerChute, 0, 100,
                               "Fuel held by each outpost chute for the human player (manual 6.3.4 B: 24)."));
    p.push_back(props::choice("humanPlayer", "Human player", kGroupStaging, "off",
                              {{"off", "Off (chute fuel is never released)"}, {"timed", "Timed (one fuel per period)"}},
                              "Whether the human players release chute fuel onto the field."));
    p.push_back(props::real("humanPlayerPeriodS", "Human player period", kGroupStaging, 2.0, 0.1, 60.0,
                            PropertyUnit::Seconds, "Seconds between chute releases while the match is running."));
    p.push_back(props::integer("maxFuelOnField", "Max fuel bodies", kGroupStaging, 800, 0, 5000,
                               "Safety cap on simultaneously simulated fuel bodies."));

    p.push_back(props::real("fuelMassKg", "Fuel mass", kGroupPhysics, dim::kFuelMassKg, 0.05, 1.0,
                            PropertyUnit::Kilograms, "Manual 5.10.1: 0.203 - 0.227 kg."));
    p.push_back(props::real("fuelFriction", "Fuel friction", kGroupPhysics, dim::kFuelFriction, 0.0, 2.0,
                            PropertyUnit::None, "Coulomb friction coefficient of fuel (foam on carpet)."));
    p.push_back(props::real("fuelRestitution", "Fuel restitution", kGroupPhysics, dim::kFuelRestitution, 0.0, 1.0,
                            PropertyUnit::None, "Bounciness of fuel."));
    p.push_back(props::real("fuelRollingResistance", "Fuel rolling resistance", kGroupPhysics,
                            dim::kFuelRollingResistance, 0.0, 1.0, PropertyUnit::None, "Rolling resistance of fuel.",
                            true));
    p.push_back(props::flag("fuelSleeping", "Fuel sleeping", kGroupPhysics, true,
                            "Let resting fuel sleep (large performance win; disable only for debugging).", true));
    p.push_back(props::real("shotGhostS", "Shot ghost time", kGroupPhysics, 0.15, 0.0, 1.0, PropertyUnit::Seconds,
                            "Shot fuel ignores the robot's collision shapes for this long after leaving the shooter.",
                            true));

    p.push_back(props::flag("enforceHubShifts", "Enforce hub shifts", kGroupRules, true,
                            "Apply the active/inactive hub schedule (manual 6.4.1). Off: both hubs score whenever the "
                            "robot is enabled."));
    p.push_back(props::choice("firstInactiveHub", "First inactive hub", kGroupRules, "auto",
                              {{"auto", "Auto winner (FMS rule)"}, {"red", "Red"}, {"blue", "Blue"}},
                              "Which alliance's hub is inactive in shift 1. Auto: the alliance that scored more fuel "
                              "in AUTO (ties: random, seeded)."));
    p.push_back(props::real("hubProcessingDelayS", "Hub processing delay", kGroupRules, 1.0, 0.0, 10.0,
                            PropertyUnit::Seconds, "Time between fuel entering a hub and leaving through its exits."));
    p.push_back(props::flag("recycleScoredFuel", "Recycle scored fuel", kGroupRules, true,
                            "Scored fuel is released from the hub exits into the neutral zone (manual 5.4). Off: it "
                            "is removed."));
    return s;
}

sdk::PluginConfigSchema configSchema() {
    sdk::PluginConfigSchema s;
    s.schemaVersion = kConfigSchemaVersion;
    s.properties = configPropertySchema();
    s.groupOrder = {kGroupMeter, kGroupStaging, kGroupPhysics, kGroupRules};
    return s;
}

GameConfig parseConfig(const sdk::Value& data) {
    const sdk::PropertySchema schema = configPropertySchema();
    GameConfig c;
    auto spec = [&](std::string_view key) -> const sdk::PropertySpec& { return *schema.find(key); };
    auto get = [&](std::string_view key) -> const sdk::Value* { return data.find(key); };

    c.capacity = clampedInt(get("capacity"), spec("capacity"), c.capacity);
    c.preloadFuel = clampedInt(get("preloadFuel"), spec("preloadFuel"), c.preloadFuel);
    c.preloadFuel = std::min(c.preloadFuel, c.capacity);
    c.neutralFuel = clampedInt(get("neutralFuel"), spec("neutralFuel"), c.neutralFuel);
    c.depotFuel = clampedInt(get("depotFuel"), spec("depotFuel"), c.depotFuel);
    c.outpostChuteFuel = clampedInt(get("outpostChuteFuel"), spec("outpostChuteFuel"), c.outpostChuteFuel);
    c.maxFuelOnField = clampedInt(get("maxFuelOnField"), spec("maxFuelOnField"), c.maxFuelOnField);
    c.humanPlayerPeriodS = clampedDouble(get("humanPlayerPeriodS"), spec("humanPlayerPeriodS"), c.humanPlayerPeriodS);
    if (const sdk::Value* v = get("humanPlayer"); v != nullptr && v->asString() == "timed")
        c.humanPlayer = HumanPlayerMode::Timed;

    c.fuel.massKg = static_cast<float>(clampedDouble(get("fuelMassKg"), spec("fuelMassKg"), c.fuel.massKg));
    c.fuel.friction = static_cast<float>(clampedDouble(get("fuelFriction"), spec("fuelFriction"), c.fuel.friction));
    c.fuel.restitution =
        static_cast<float>(clampedDouble(get("fuelRestitution"), spec("fuelRestitution"), c.fuel.restitution));
    c.fuel.rollingResistance = static_cast<float>(
        clampedDouble(get("fuelRollingResistance"), spec("fuelRollingResistance"), c.fuel.rollingResistance));
    c.fuel.sleeping = boolOf(get("fuelSleeping"), c.fuel.sleeping);
    c.shotGhostS = static_cast<float>(clampedDouble(get("shotGhostS"), spec("shotGhostS"), c.shotGhostS));

    c.enforceHubShifts = boolOf(get("enforceHubShifts"), c.enforceHubShifts);
    if (const sdk::Value* v = get("firstInactiveHub")) {
        const std::string s = v->asString();
        if (s == "red") c.firstInactive = FirstInactiveMode::Red;
        else if (s == "blue") c.firstInactive = FirstInactiveMode::Blue;
    }
    c.hubProcessingDelayS = clampedDouble(get("hubProcessingDelayS"), spec("hubProcessingDelayS"), c.hubProcessingDelayS);
    c.recycleScoredFuel = boolOf(get("recycleScoredFuel"), c.recycleScoredFuel);
    return c;
}

sdk::Value toValue(const GameConfig& c) {
    sdk::Value o = sdk::Value::makeObject();
    o["capacity"] = c.capacity;
    o["preloadFuel"] = c.preloadFuel;
    o["neutralFuel"] = c.neutralFuel;
    o["depotFuel"] = c.depotFuel;
    o["outpostChuteFuel"] = c.outpostChuteFuel;
    o["humanPlayer"] = std::string(humanPlayerName(c.humanPlayer));
    o["humanPlayerPeriodS"] = c.humanPlayerPeriodS;
    o["maxFuelOnField"] = c.maxFuelOnField;
    o["fuelMassKg"] = c.fuel.massKg;
    o["fuelFriction"] = c.fuel.friction;
    o["fuelRestitution"] = c.fuel.restitution;
    o["fuelRollingResistance"] = c.fuel.rollingResistance;
    o["fuelSleeping"] = c.fuel.sleeping;
    o["shotGhostS"] = c.shotGhostS;
    o["enforceHubShifts"] = c.enforceHubShifts;
    o["firstInactiveHub"] = std::string(firstInactiveName(c.firstInactive));
    o["hubProcessingDelayS"] = c.hubProcessingDelayS;
    o["recycleScoredFuel"] = c.recycleScoredFuel;
    return o;
}

const char* firstInactiveName(FirstInactiveMode mode) {
    switch (mode) {
        case FirstInactiveMode::FromAutoResult: return "auto";
        case FirstInactiveMode::Red: return "red";
        case FirstInactiveMode::Blue: return "blue";
    }
    return "auto";
}

const char* humanPlayerName(HumanPlayerMode mode) { return mode == HumanPlayerMode::Timed ? "timed" : "off"; }

}  // namespace wpiemu::plugins::game_2026
