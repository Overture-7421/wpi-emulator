#include <doctest/doctest.h>

#include <algorithm>
#include <string>

#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/migration.h"
#include "wpiemu/plugins/game_2026/plugin.h"
#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/game/plugin_registry.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;

TEST_CASE("config schema defaults validate and round-trip through parseConfig") {
    const sdk::PluginConfigSchema schema = configSchema();
    CHECK(schema.schemaVersion == kConfigSchemaVersion);
    CHECK(schema.properties.properties.size() >= 15);

    const sdk::Value defaults = sdk::defaultsOf(schema.properties);
    CHECK(sdk::validateProperties(schema.properties, defaults).empty());

    const GameConfig c = parseConfig(defaults);
    CHECK(c.capacity == 30);
    CHECK(c.preloadFuel == 8);
    CHECK(c.neutralFuel == 360);
    CHECK(c.depotFuel == 24);
    CHECK(c.outpostChuteFuel == 24);
    CHECK(c.enforceHubShifts);
    CHECK(c.firstInactive == FirstInactiveMode::FromAutoResult);
    CHECK(c.humanPlayer == HumanPlayerMode::Off);
    CHECK(c.fuel.massKg == doctest::Approx(0.215f));  // 0.203 - 0.227 kg per manual 5.10.1
    CHECK(c.fuel.sleeping);

    // toValue/parseConfig round trip and the result still validates
    const sdk::Value out = toValue(c);
    CHECK(sdk::validateProperties(schema.properties, out).empty());
    const GameConfig again = parseConfig(out);
    CHECK(again.capacity == c.capacity);
    CHECK(again.fuel.restitution == doctest::Approx(c.fuel.restitution));
    CHECK(again.hubProcessingDelayS == doctest::Approx(c.hubProcessingDelayS));
}

TEST_CASE("every schema group is listed in groupOrder and every key is unique") {
    const sdk::PluginConfigSchema schema = configSchema();
    std::vector<std::string> keys;
    for (const sdk::PropertySpec& p : schema.properties.properties) {
        CHECK_FALSE(p.label.empty());
        CHECK_FALSE(p.group.empty());
        CHECK(std::find(schema.groupOrder.begin(), schema.groupOrder.end(), p.group) != schema.groupOrder.end());
        CHECK(std::find(keys.begin(), keys.end(), p.key) == keys.end());
        keys.push_back(p.key);
    }
}

TEST_CASE("parseConfig clamps out-of-range values and ignores garbage") {
    sdk::Value v = sdk::Value::makeObject();
    v["capacity"] = 100000;
    v["preloadFuel"] = 900;       // above capacity => limited to it
    v["neutralFuel"] = -5;
    v["depotFuel"] = 1000;
    v["fuelMassKg"] = 0.0;
    v["fuelRestitution"] = 7.0;
    v["humanPlayer"] = "timed";
    v["humanPlayerPeriodS"] = "fast";    // wrong type => default
    v["firstInactiveHub"] = "blue";
    v["enforceHubShifts"] = "yes";       // wrong type => default (true)
    v["unknownKey"] = 3;
    const GameConfig c = parseConfig(v);
    CHECK(c.capacity == 500);
    CHECK(c.preloadFuel == 500);
    CHECK(c.neutralFuel == 0);
    CHECK(c.depotFuel == 24);
    CHECK(c.fuel.massKg == doctest::Approx(0.05f));
    CHECK(c.fuel.restitution == doctest::Approx(1.0f));
    CHECK(c.humanPlayer == HumanPlayerMode::Timed);
    CHECK(c.humanPlayerPeriodS == doctest::Approx(2.0));
    CHECK(c.firstInactive == FirstInactiveMode::Blue);
    CHECK(c.enforceHubShifts);

    CHECK(parseConfig(sdk::Value{}).capacity == 30);                // null section
    CHECK(parseConfig(sdk::Value("not an object")).capacity == 30);  // wrong shape
    sdk::Value small = sdk::Value::makeObject();
    small["capacity"] = 5;
    CHECK(parseConfig(small).preloadFuel == 5);  // default preload 8 is limited to the capacity
}

TEST_CASE("schema validation rejects bad persisted values the way the inspector would") {
    const sdk::PluginConfigSchema schema = configSchema();
    sdk::Value v = sdk::defaultsOf(schema.properties);
    v["capacity"] = -1;
    v["humanPlayer"] = "robot";
    const auto issues = sdk::validateProperties(schema.properties, v);
    CHECK(issues.size() == 2);
}

TEST_CASE("v1 -> v2 section migration renames the draft keys through the SDK chain") {
    sdk::Value data = sdk::Value::makeObject();
    data["fuelCapacity"] = 45;
    data["preload"] = 3;
    data["neutralZoneFuel"] = 300;
    data["hubShifts"] = false;
    data["firstInactive"] = "red";
    data["capacity"] = 99;     // already new: the stale draft key must not win
    data["depotCount"] = 12;

    const auto migrators = makeConfigMigrators();
    int version = 1;
    REQUIRE(sdk::migrateToVersion(data, version, kConfigSchemaVersion, migrators).ok());
    CHECK(version == 2);
    CHECK(data.find("fuelCapacity") == nullptr);
    CHECK(data.find("capacity")->asInt() == 99);
    CHECK(data.find("preloadFuel")->asInt() == 3);
    CHECK(data.find("neutralFuel")->asInt() == 300);
    CHECK(data.find("depotFuel")->asInt() == 12);
    CHECK(data.find("enforceHubShifts")->asBool(true) == false);
    CHECK(data.find("firstInactiveHub")->asString() == "red");

    const GameConfig c = parseConfig(data);
    CHECK(c.capacity == 99);
    CHECK_FALSE(c.enforceHubShifts);
    CHECK(c.firstInactive == FirstInactiveMode::Red);
}

TEST_CASE("section migration failure and version edge cases") {
    const auto migrators = makeConfigMigrators();
    sdk::Value notObject("scalar");
    int v1 = 1;
    CHECK(sdk::migrateToVersion(notObject, v1, 2, migrators).error().code == sdk::ErrorCode::InvalidArgument);

    sdk::Value data = sdk::Value::makeObject();
    int newer = 3;
    CHECK(sdk::migrateToVersion(data, newer, 2, migrators).error().code == sdk::ErrorCode::VersionMismatch);
    int current = 2;
    CHECK(sdk::migrateToVersion(data, current, 2, migrators).ok());  // already current: no-op
}

TEST_CASE("migrateRobotConfig renames only this plugin's draft mechanisms") {
    sdk::RobotConfig config;
    auto add = [&](const char* id, const char* type, sdk::Value props) {
        sdk::MechanismConfig m;
        m.id = id;
        m.type = type;
        m.properties = std::move(props);
        config.mechanisms.push_back(std::move(m));
    };
    sdk::Value intakeProps = sdk::Value::makeObject();
    intakeProps["zone"] = "z";
    intakeProps["intakeRate"] = 9;
    sdk::Value shooterProps = sdk::Value::makeObject();
    shooterProps["exitEfficiency"] = 0.5;
    shooterProps["feedMechanismId"] = "indexer";
    shooterProps["efficiency"] = 0.7;  // new key present: stale one is dropped, new one kept
    sdk::Value foreign = sdk::Value::makeObject();
    foreign["zone"] = "keep";
    add("intake", "game_2026.intake", intakeProps);
    add("shooter", "game_2026.shooter", shooterProps);
    add("coreRoller", "roller", foreign);
    add("otherYear", "game_2027.intake", foreign);
    add("modern", "game_2026.fuel_shooter", sdk::Value::makeObject());

    REQUIRE(migrateGame2026Config(config, 1).ok());
    CHECK(config.mechanisms[0].type == "game_2026.fuel_intake");
    CHECK(config.mechanisms[0].properties.find("pickupZone")->asString() == "z");
    CHECK(config.mechanisms[0].properties.find("intakeRateHz")->asInt() == 9);
    CHECK(config.mechanisms[1].type == "game_2026.fuel_shooter");
    CHECK(config.mechanisms[1].properties.find("feedMechanism")->asString() == "indexer");
    CHECK(config.mechanisms[1].properties.find("efficiency")->asDouble() == doctest::Approx(0.7));
    CHECK(config.mechanisms[1].properties.find("exitEfficiency") == nullptr);

    // ownership rule: core and foreign mechanisms are untouched
    CHECK(config.mechanisms[2].type == "roller");
    CHECK(config.mechanisms[2].properties.find("zone") != nullptr);
    CHECK(config.mechanisms[3].type == "game_2027.intake");
    CHECK(config.mechanisms[3].properties.find("zone") != nullptr);

    // idempotent and a no-op for current sections
    const sdk::RobotConfig before = config;
    REQUIRE(migrateGame2026Config(config, 1).ok());
    REQUIRE(migrateGame2026Config(config, 2).ok());
    CHECK(config.mechanisms[0].type == before.mechanisms[0].type);
    CHECK(config.mechanisms[1].properties == before.mechanisms[1].properties);
}

TEST_CASE("the module wires the migrators and the whole-config hook") {
    sdk::GamePluginRegistry registry;
    REQUIRE(registerPlugin(registry).ok());
    sdk::IGameModule* module = registry.find("game_2026");
    REQUIRE(module != nullptr);
    CHECK(module->configMigrators().size() == 1);

    sdk::RobotConfig config;
    sdk::MechanismConfig m;
    m.id = "i";
    m.type = "game_2026.intake";
    config.mechanisms.push_back(m);
    REQUIRE(module->migrateRobotConfig(config, 1).ok());
    CHECK(config.mechanisms[0].type == "game_2026.fuel_intake");
}
