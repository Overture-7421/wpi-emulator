#include <doctest/doctest.h>

#include <algorithm>

#include "runtime_fixtures.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/core/runtime/sensor_system.h"
#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/sdk/devices/signals.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/fake_device_hub.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core;
using namespace wpiemu::core::config;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;

namespace {

SensorConfig sensor(const std::string& id, SensorKind kind, const std::string& source = "fake_game.level") {
    SensorConfig s;
    s.id = id;
    s.kind = kind;
    s.source = source;
    s.properties = defaultsOf(sensorResponseSchema(kind));
    return s;
}

std::vector<Issue> validate(const RobotConfig& c, Registries& r) {
    IGameModule* p = selectPlugin(c, r.plugins);
    REQUIRE(p != nullptr);
    REQUIRE(r.registrar.ensureRegistered(*p).ok());
    return config::ConfigValidator{}.validate(c, r.mechanisms, r.drivetrains, r.plugins);
}

bool has(const std::vector<Issue>& issues, const char* code, const std::string& path, Severity sev = Severity::Error) {
    return std::any_of(issues.begin(), issues.end(),
                       [&](const Issue& i) { return i.code == code && i.path == path && i.severity == sev; });
}

}  // namespace

TEST_CASE("sensors: the config codec round trips them, defaults absent sensors to none and rejects unknown kinds") {
    const config::RobotConfigCodec codec;
    RobotConfig c = sampleConfig();
    SensorConfig s = sensor("hopper", SensorKind::Distance);
    s.label = "Hopper";
    s.properties["mode"] = "linear";
    s.mappings.push_back({{"hopper", "sensor"}, {"phoenix6", "canrange", 33, "rio", -1}, false});
    c.sensors.push_back(s);
    c.sensors.push_back(sensor("gate", SensorKind::BeamBreak));

    const Result<RobotConfig> back = codec.fromValue(codec.toValue(c));
    REQUIRE(back.ok());
    REQUIRE(back.value().sensors.size() == 2);
    CHECK(back.value().sensors[0] == s);
    CHECK(back.value().sensors[1] == c.sensors[1]);
    CHECK(back.value().allMappings().size() == c.allMappings().size());

    Value bare = codec.toValue(sampleConfig());
    bare.object().erase("sensors");
    CHECK(codec.fromValue(bare).value().sensors.empty());

    Value bad = codec.toValue(c);
    bad["sensors"].array()[0]["kind"] = "radar";
    const Result<RobotConfig> err = codec.fromValue(bad);
    REQUIRE_FALSE(err.ok());
    CHECK(err.error().message.find("sensors[0].kind") != std::string::npos);
}

TEST_CASE("sensors: validator checks ids, sources, response properties and slot mappings") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.sensors.push_back(sensor("ok", SensorKind::Distance));
    CHECK_FALSE(config::ConfigValidator::hasErrors(validate(c, reg)));

    c.sensors.push_back(sensor("", SensorKind::Distance));                   // 1
    c.sensors.push_back(sensor("ok", SensorKind::BeamBreak));                // 2: duplicate
    c.sensors.push_back(sensor("intake", SensorKind::Distance));             // 3: mechanism id
    c.sensors.push_back(sensor("a.b", SensorKind::Distance));                // 4
    c.sensors.push_back(sensor("robot", SensorKind::Distance));              // 5: reserved
    c.sensors.push_back(sensor("none", SensorKind::Distance, ""));           // 6: Warning
    c.sensors.push_back(sensor("foreign", SensorKind::Distance, "other.x")); // 7
    c.sensors.push_back(sensor("unknown", SensorKind::Distance, "fake_game.nope"));  // 8: Warning
    SensorConfig linear = sensor("flat", SensorKind::Distance);              // 9
    linear.properties["mode"] = "linear";
    linear.properties["inputMin"] = 2.0;
    linear.properties["inputMax"] = 2.0;
    c.sensors.push_back(linear);
    SensorConfig badMode = sensor("badMode", SensorKind::Distance);          // 10
    badMode.properties["mode"] = "cubic";
    c.sensors.push_back(badMode);
    SensorConfig badSlot = sensor("badSlot", SensorKind::BeamBreak);         // 11
    badSlot.mappings.push_back({{"badSlot", "motor"}, {"phoenix6", "canrange", 40, "rio", -1}, false});
    c.sensors.push_back(badSlot);

    const auto issues = validate(c, reg);
    CHECK(has(issues, validation_code::kSensorIdInvalid, "sensors[1].id"));
    CHECK(has(issues, validation_code::kSensorIdDuplicate, "sensors[2].id"));
    CHECK(has(issues, validation_code::kSensorIdDuplicate, "sensors[3].id"));
    CHECK(has(issues, validation_code::kSensorIdInvalid, "sensors[4].id"));
    CHECK(has(issues, validation_code::kSensorIdInvalid, "sensors[5].id"));
    CHECK(has(issues, validation_code::kSensorSourceMissing, "sensors[6].source", Severity::Warning));
    CHECK(has(issues, validation_code::kSensorSourceNotOwned, "sensors[7].source"));
    CHECK(has(issues, validation_code::kSensorSourceUnknown, "sensors[8].source", Severity::Warning));
    CHECK(has(issues, validation_code::kSensorPropertyInvalid, "sensors[9].properties.inputMax"));
    CHECK(has(issues, validation_code::kSensorPropertyInvalid, "sensors[10].properties.mode"));
    CHECK(has(issues, config::issue_code::kSlotUnknown, "sensors[11].mappings[0]"));
}

TEST_CASE("sensors: a source needs a game plugin") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.project.wpilibYear = 1999;  // no plugin for this season
    c.sensors.push_back(sensor("hopper", SensorKind::Distance));
    const auto issues = config::ConfigValidator{}.validate(c, reg.mechanisms, reg.drivetrains, reg.plugins);
    CHECK(has(issues, validation_code::kSensorSourceNotOwned, "sensors[0].source"));
}

TEST_CASE("sensors: readings follow the response mode") {
    using runtime::beamBreakReading;
    using runtime::distanceReadingM;
    Value threshold = defaultsOf(sensorResponseSchema(SensorKind::Distance));
    threshold["threshold"] = 6.0;
    threshold["nearM"] = 0.1;
    threshold["farM"] = 1.0;
    CHECK(distanceReadingM(threshold, 5.0) == doctest::Approx(1.0));
    CHECK(distanceReadingM(threshold, 6.0) == doctest::Approx(0.1));
    CHECK(runtime::idleDistanceM(threshold) == doctest::Approx(1.0));

    Value linear = defaultsOf(sensorResponseSchema(SensorKind::Distance));
    linear["mode"] = "linear";
    linear["inputMin"] = 0.0;
    linear["inputMax"] = 10.0;
    linear["distanceAtMinM"] = 1.0;
    linear["distanceAtMaxM"] = 0.2;
    CHECK(distanceReadingM(linear, 5.0) == doctest::Approx(0.6));
    CHECK(distanceReadingM(linear, -3.0) == doctest::Approx(1.0));  // clamped
    CHECK(distanceReadingM(linear, 99.0) == doctest::Approx(0.2));

    Value beam = defaultsOf(sensorResponseSchema(SensorKind::BeamBreak));
    beam["threshold"] = 2.0;
    CHECK_FALSE(beamBreakReading(beam, 1.0));
    CHECK(beamBreakReading(beam, 2.0));
    beam["brokenValue"] = false;  // active-low wiring
    CHECK(beamBreakReading(beam, 2.0) == false);
    CHECK(beamBreakReading(beam, 1.0) == true);
}


TEST_CASE("sensors: a session feeds game values to the mapped device and reports unavailable sources") {
    Registries reg;
    auto* module = dynamic_cast<testing::FakeGameModule*>(reg.plugins.find("fake_game"));
    REQUIRE(module != nullptr);

    proto::DeviceAnnounce rangeDevice;
    rangeDevice.id = DeviceId{201};
    rangeDevice.descriptor = {"phoenix6", "canrange", "ctre", 33, kBusUnknown, -1, {}, "", ""};
    rangeDevice.descriptor.capabilities = Capability::Distance;
    rangeDevice.signalSpecs = {SignalSpec{signal::kDistance, SignalDirection::Input, SignalType::Double, "m"}};

    RobotConfig cfg = sampleConfig();
    SensorConfig range = sensor("range", SensorKind::Distance, "fake_game.level");
    range.properties["threshold"] = 2.0;
    range.properties["nearM"] = 0.1;
    range.properties["farM"] = 0.9;
    range.mappings.push_back({{"range", kSensorSlot}, rangeDevice.descriptor.key(), false});
    cfg.sensors.push_back(range);
    cfg.sensors.push_back(sensor("gate", SensorKind::BeamBreak, "fake_game.missing"));

    testing::CapturingLogger logger;
    runtime::SessionParts parts;
    parts.world = std::make_unique<testing::FakePhysicsWorld>(cfg.sim.physics);
    parts.config = std::move(cfg);
    parts.mechanisms = &reg.mechanisms;
    parts.drivetrains = &reg.drivetrains;
    parts.tireModels = &reg.tires;
    parts.plugins = &reg.plugins;
    parts.logger = &logger;
    auto built = runtime::SessionBuilder::build(std::move(parts));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    runtime::SimulationSession& session = *built.value();
    session.hub().onAnnounce(rangeDevice);

    const auto distance = [&](double level) {
        module->counters()->level = level;
        proto::SensorFrame out;
        session.tick(out, true);
        for (const auto& s : out.samples)
            if (s.device == DeviceId{201}) return s.value;
        return -999.0;
    };
    CHECK(distance(1.0) == doctest::Approx(0.9));
    CHECK(distance(2.0) == doctest::Approx(0.1));  // same tick as the game state: no lag
    CHECK(distance(0.0) == doctest::Approx(0.9));

    const auto snapshot = session.buildSnapshot(false, true);
    const auto warnings = snapshot->warnings();
    const auto unavailable = std::count_if(warnings.begin(), warnings.end(),
                                           [](const HubWarning& w) { return w.code == "sensor_source_unavailable"; });
    CHECK(unavailable == 1);  // "gate": its value is not published; "range" resolves

    const auto states = snapshot->sensors();
    REQUIRE(states.size() == 2);
    CHECK(states[0].id == "range");
    REQUIRE(states[0].sourceValue.has_value());
    CHECK(*states[0].sourceValue == doctest::Approx(0.0));
    CHECK(states[0].reading == doctest::Approx(0.9));
    CHECK(states[1].id == "gate");
    CHECK_FALSE(states[1].sourceValue.has_value());
    CHECK(states[1].reading == doctest::Approx(0.0));  // idle: beam intact
}
