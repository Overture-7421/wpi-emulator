#include <doctest/doctest.h>

#include <cmath>
#include <limits>

#include "runtime_fixtures.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"

using namespace wpiemu::sdk;
using wpiemu::core::config::JsonValueCodec;
using wpiemu::core::config::RobotConfigCodec;
using wpiemu::test_runtime::sampleConfig;

namespace {

Value roundTripValue(const RobotConfig& c) {
    const RobotConfigCodec codec;
    return codec.toValue(c);
}

}  // namespace

TEST_CASE("config codec: a full config survives config -> Value -> JSON text -> Value -> config") {
    RobotConfig c = sampleConfig();
    c.chassis.sizeM = Vec3{0.7f, 0.65f, 0.12f};
    c.chassis.comOffsetM = Vec3{0.01f, -0.02f, 0.f};
    c.chassis.primitives.push_back({"bumper", PrimitiveKind::Box, Pose3{Vec3{0.f, 0.35f, 0.f}, Quat::fromYaw(0.5f)}, Vec3{0.7f, 0.05f, 0.1f}, 0.f, 0.f, "#ff0000", true, 1.5f});
    c.drivetrain.spec.wheels[0].kind = WheelKind::Mecanum;
    c.drivetrain.spec.wheels[0].tire.mu = 1.3f;
    c.drivetrain.spec.wheels[1].steered = true;
    c.mechanisms[0].properties["gearRatio"] = 2.5;
    c.mechanisms[0].mappings.push_back({{"intake", "motor2"}, DeviceKey{"phoenix6", "talonfx", 11, "rio", -1}, true});
    c.mechanisms[0].geometry.push_back({"roller", PrimitiveKind::Cylinder, Pose3{}, Vec3{}, 0.04f, 0.3f, "#00ff00", false, 0.f});
    c.sim.seed = 12345;
    c.sim.physics.gravity = Vec3{0.f, 0.f, -9.81f};
    c.plugins["fake_game"] = PluginSection{1, Value::makeObject()};
    c.plugins["fake_game"].data["capacity"] = 7;
    c.gamePluginId = "fake_game";

    const RobotConfigCodec codec;
    const JsonValueCodec json;
    const Value v = codec.toValue(c);
    const auto parsed = json.parse(json.dump(v, true));
    REQUIRE(parsed.ok());
    const auto back = codec.fromValue(parsed.value());
    REQUIRE_MESSAGE(back.ok(), (back.ok() ? "" : back.error().message));

    const RobotConfig& r = back.value();
    CHECK(r.name == c.name);
    CHECK(r.project.wpilibYear == 2999);
    CHECK(r.chassis.sizeM == c.chassis.sizeM);
    CHECK(r.chassis.comOffsetM == c.chassis.comOffsetM);
    REQUIRE(r.chassis.primitives.size() == 1);
    CHECK(r.chassis.primitives[0].collides);
    CHECK(r.chassis.primitives[0].local.rotation.yaw() == doctest::Approx(0.5f).epsilon(1e-5));
    REQUIRE(r.drivetrain.spec.wheels.size() == 4);
    CHECK(r.drivetrain.spec.wheels[0].kind == WheelKind::Mecanum);
    CHECK(r.drivetrain.spec.wheels[0].tire.mu == 1.3f);
    CHECK(r.drivetrain.spec.wheels[1].steered);
    CHECK(r.drivetrain.mappings == c.drivetrain.mappings);
    REQUIRE(r.mechanisms.size() == 1);
    CHECK(r.mechanisms[0].properties.find("gearRatio")->asDouble() == 2.5);
    CHECK(r.mechanisms[0].mappings == c.mechanisms[0].mappings);
    CHECK(r.mechanisms[0].mappings.back().reversed);
    CHECK_FALSE(r.mechanisms[0].mappings.front().reversed);
    CHECK(r.mechanisms[0].mount.position == c.mechanisms[0].mount.position);
    CHECK(r.mechanisms[0].geometry[0].kind == PrimitiveKind::Cylinder);
    CHECK(r.robotMappings == c.robotMappings);
    CHECK(r.sim.seed == 12345);
    CHECK(r.sim.physics.gravity.z == -9.81f);
    CHECK(r.gamePluginId == "fake_game");
    REQUIRE(r.findPlugin("fake_game") != nullptr);
    CHECK(r.findPlugin("fake_game")->schemaVersion == 1);
    CHECK(r.findPlugin("fake_game")->data.find("capacity")->asInt() == 7);
    // and the second generation is textually identical
    CHECK(json.dump(codec.toValue(r), true) == json.dump(v, true));
}

TEST_CASE("config codec: floats are written as short decimals") {
    RobotConfig c = sampleConfig();
    c.chassis.sizeM = Vec3{0.7f, 0.7f, 0.12f};
    const std::string text = JsonValueCodec{}.dump(roundTripValue(c), true);
    CHECK(text.find("0.7") != std::string::npos);
    CHECK(text.find("0.699999") == std::string::npos);
}

TEST_CASE("config codec: the structured attachment is the only form written and read") {
    RobotConfig c = sampleConfig();
    MechanismConfig arm;
    arm.id = "arm";
    arm.type = "roller";
    arm.attachTo("intake", "tip");
    c.mechanisms.push_back(arm);

    const Value v = roundTripValue(c);
    const Value& m0 = v.find("mechanisms")->array()[0];
    CHECK(m0.find("parent") == nullptr);
    CHECK(m0.find("attachment")->find("parentLink")->asString() == "root");
    const Value& m1 = v.find("mechanisms")->array()[1];
    CHECK(m1.find("parent") == nullptr);
    CHECK(m1.find("attachment")->find("parentMechanismId")->asString() == "intake");
    CHECK(m1.find("attachment")->find("parentLink")->asString() == "tip");

    const auto r = RobotConfigCodec{}.fromValue(v);
    REQUIRE(r.ok());
    CHECK(r.value().mechanisms[1].attachment.parentMechanismId == "intake");
    CHECK(r.value().mechanisms[1].attachment.parentLink == "tip");

    // a file without any attachment mounts the mechanism on the chassis root
    Value bare = v;
    bare["mechanisms"].array()[1].object().erase("attachment");
    const auto b = RobotConfigCodec{}.fromValue(bare);
    REQUIRE(b.ok());
    CHECK(b.value().mechanisms[1].attachment.onChassis());
}

TEST_CASE("config codec: unknown fields are ignored") {
    Value v = roundTripValue(sampleConfig());
    v["futureRootField"] = Value::makeObject();
    v["chassis"]["wedgeAngle"] = 12.5;
    v["mechanisms"].array()[0]["notes"] = "hand written";

    const RobotConfigCodec codec;
    const auto r = codec.fromValue(v);
    REQUIRE(r.ok());
    const Value out = codec.toValue(r.value());
    CHECK(out.find("futureRootField") == nullptr);
    CHECK(out.find("chassis")->find("wedgeAngle") == nullptr);
    CHECK(out.find("mechanisms")->array()[0].find("notes") == nullptr);
}
TEST_CASE("config codec: absent fields default, wrong types are reported with their path") {
    Value minimal = Value::makeObject();
    minimal["schemaVersion"] = 1;
    const auto d = RobotConfigCodec{}.fromValue(minimal);
    REQUIRE(d.ok());
    CHECK(d.value().name == "Robot");
    CHECK(d.value().sim.stepUs == 5000);
    CHECK(d.value().chassis.massKg == 50.f);

    Value bad = roundTripValue(sampleConfig());
    bad["chassis"]["massKg"] = "heavy";
    bad["drivetrain"]["wheels"].array()[1]["kind"] = "hover";
    bad["mechanisms"].array()[0]["mappings"] = 5;
    const auto r = RobotConfigCodec{}.fromValue(bad);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::ParseError);
    CHECK(r.error().message.find("chassis.massKg") != std::string::npos);
    CHECK(r.error().message.find("drivetrain.wheels[1].kind") != std::string::npos);
    CHECK(r.error().message.find("mechanisms[0].mappings") != std::string::npos);

    CHECK_FALSE(RobotConfigCodec{}.fromValue(Value(3)).ok());
    Value notNumber = roundTripValue(sampleConfig());
    notNumber["schemaVersion"] = "one";
    CHECK_FALSE(RobotConfigCodec{}.fromValue(notNumber).ok());
    Value badMapping = roundTripValue(sampleConfig());
    badMapping["robotMappings"].array()[0].object().erase("device");
    CHECK_FALSE(RobotConfigCodec{}.fromValue(badMapping).ok());
    Value zeroQuat = roundTripValue(sampleConfig());
    zeroQuat["mechanisms"].array()[0]["mount"]["rotation"] = Value::makeObject();
    zeroQuat["mechanisms"].array()[0]["mount"]["rotation"]["x"] = 0;
    zeroQuat["mechanisms"].array()[0]["mount"]["rotation"]["y"] = 0;
    zeroQuat["mechanisms"].array()[0]["mount"]["rotation"]["z"] = 0;
    zeroQuat["mechanisms"].array()[0]["mount"]["rotation"]["w"] = 0;
    CHECK_FALSE(RobotConfigCodec{}.fromValue(zeroQuat).ok());
}

TEST_CASE("config codec: only the current schema version is accepted, there is no migration") {
    for (const int version : {0, kRobotConfigSchemaVersion + 1}) {
        Value other = roundTripValue(sampleConfig());
        other["schemaVersion"] = version;
        const auto refused = RobotConfigCodec{}.fromValue(other);
        REQUIRE_FALSE(refused.ok());
        CHECK(refused.error().code == ErrorCode::VersionMismatch);
    }
    // a hand-written file may omit the version
    Value bare = roundTripValue(sampleConfig());
    bare.object().erase("schemaVersion");
    const auto r = RobotConfigCodec{}.fromValue(bare);
    REQUIRE(r.ok());
    CHECK(r.value().schemaVersion == kRobotConfigSchemaVersion);
}
TEST_CASE("config codec: numeric integer boundaries are checked before conversion") {
    const RobotConfigCodec codec;
    const JsonValueCodec json;
    auto rejected = [&](const std::string& fields, const std::string& path) {
        const auto parsed = json.parse("{" + fields + "}");
        REQUIRE(parsed.ok());
        const auto result = codec.fromValue(parsed.value());
        REQUIRE_FALSE(result.ok());
        CHECK(result.error().code == ErrorCode::ParseError);
        CHECK(result.error().message.find(path) != std::string::npos);
    };
    for (const char* version : {"1.9", "1e100", "2147483648", "-2147483649"}) {
        rejected(std::string("\"schemaVersion\":") + version, "schemaVersion");

    }
    for (const char* step : {"1e100", "9223372036854775808", "-9223372036854777856", "1.5"})
        rejected(std::string("\"sim\":{\"stepUs\":") + step + "}", "sim.stepUs");
    const std::string mapping =
        "\"robotMappings\":[{\"owner\":\"robot\",\"slot\":\"imu\",\"device\":{";
    for (const char* id : {"2147483648", "-2147483649", "4294967297", "1e100", "1.5"}) {
        rejected(mapping + "\"id\":" + id + "}}]", "robotMappings[0].device.id");
        rejected(mapping + "\"channel\":" + id + "}}]", "robotMappings[0].device.channel");
    }
    rejected("\"plugins\":{\"fake\":{\"schemaVersion\":1.9}}", "plugins.fake.schemaVersion");
    rejected("\"plugins\":{\"fake\":{\"schemaVersion\":1e100}}", "plugins.fake.schemaVersion");
    rejected("\"sim\":{\"seed\":18446744073709551615}", "sim.seed");

    Value v = codec.toValue(sampleConfig());
    v["sim"]["stepUs"] = std::numeric_limits<std::int64_t>::max();
    v["project"]["wpilibYear"] = std::numeric_limits<int>::max();
    v["robotMappings"].array()[0]["device"]["id"] = std::numeric_limits<int>::min();
    v["robotMappings"].array()[0]["device"]["channel"] = std::numeric_limits<int>::max();
    auto result = codec.fromValue(v);
    REQUIRE(result.ok());
    CHECK(result.value().sim.stepUs == std::numeric_limits<std::int64_t>::max());
    CHECK(result.value().project.wpilibYear == std::numeric_limits<int>::max());
    CHECK(result.value().robotMappings[0].device.id == std::numeric_limits<int>::min());
    CHECK(result.value().robotMappings[0].device.channel == std::numeric_limits<int>::max());
    v["sim"]["stepUs"] = -std::ldexp(1.0, 63);
    v["project"]["wpilibYear"] = static_cast<double>(std::numeric_limits<int>::min());
    result = codec.fromValue(v);
    REQUIRE(result.ok());
    CHECK(result.value().sim.stepUs == std::numeric_limits<std::int64_t>::min());
    CHECK(result.value().project.wpilibYear == std::numeric_limits<int>::min());
    v["sim"]["stepUs"] = std::nextafter(std::ldexp(1.0, 63), 0.0);
    result = codec.fromValue(v);
    REQUIRE(result.ok());
    CHECK(result.value().sim.stepUs == std::numeric_limits<std::int64_t>::max() - 1023);
}

TEST_CASE("config codec: seeds retain their signed int64 persisted representation") {
    const RobotConfigCodec codec;
    const JsonValueCodec json;
    for (const std::uint64_t seed :
         {std::uint64_t{0}, std::uint64_t{1}, std::uint64_t{1} << 63,
          std::numeric_limits<std::uint64_t>::max()}) {
        RobotConfig cfg = sampleConfig();
        cfg.sim.seed = seed;
        const Value stored = codec.toValue(cfg);
        CHECK(stored.find("sim")->find("seed")->isInt());
        const auto parsed = json.parse(json.dump(stored, false));
        REQUIRE(parsed.ok());
        const auto result = codec.fromValue(parsed.value());
        REQUIRE(result.ok());
        CHECK(result.value().sim.seed == seed);
    }
    const auto result = codec.fromValue(json.parse("{\"sim\":{\"seed\":-1.0}}").value());
    REQUIRE(result.ok());
    CHECK(result.value().sim.seed == std::numeric_limits<std::uint64_t>::max());
}

TEST_CASE("config codec: nonfinite and out-of-range float components are parse errors") {
    const RobotConfigCodec codec;
    for (const double number : {1e100, -1e100, std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::quiet_NaN()}) {
        for (const char* field : {"massKg", "sizeM", "rotation"}) {
            Value v = codec.toValue(sampleConfig());
            std::string path;
            if (std::string(field) == "massKg") {
                v["chassis"]["massKg"] = number;
                path = "chassis.massKg";
            } else if (std::string(field) == "sizeM") {
                v["chassis"]["sizeM"]["x"] = number;
                path = "chassis.sizeM.x";
            } else {
                v["mechanisms"].array()[0]["mount"]["rotation"]["w"] = number;
                path = "mechanisms[0].mount.rotation.w";
            }
            const auto result = codec.fromValue(v);
            REQUIRE_FALSE(result.ok());
            CHECK(result.error().code == ErrorCode::ParseError);
            CHECK(result.error().message.find(path) != std::string::npos);
        }
    }
    for (const double number : {std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::quiet_NaN()}) {
        Value v = codec.toValue(sampleConfig());
        v["schemaVersion"] = number;
        CHECK(codec.fromValue(v).error().code == ErrorCode::ParseError);
        v["schemaVersion"] = 1;
        v["sim"]["stepUs"] = number;
        CHECK(codec.fromValue(v).error().code == ErrorCode::ParseError);
        v["sim"]["stepUs"] = 5000;
        v["battery"]["nominalVoltageV"] = number;
        const auto result = codec.fromValue(v);
        REQUIRE_FALSE(result.ok());
        CHECK(result.error().message.find("battery.nominalVoltageV") != std::string::npos);
    }
    Value valid = codec.toValue(sampleConfig());
    valid["chassis"]["massKg"] = static_cast<double>(std::numeric_limits<float>::max());
    valid["chassis"]["sizeM"]["x"] = -static_cast<double>(std::numeric_limits<float>::max());
    valid["mechanisms"].array()[0]["mount"]["rotation"]["w"] =
        static_cast<double>(std::numeric_limits<float>::max());
    const auto result = codec.fromValue(valid);
    REQUIRE(result.ok());
    CHECK(result.value().chassis.massKg == std::numeric_limits<float>::max());
    CHECK(result.value().chassis.sizeM.x == -std::numeric_limits<float>::max());
    CHECK(result.value().mechanisms[0].mount.rotation.w == 1.f);
}
