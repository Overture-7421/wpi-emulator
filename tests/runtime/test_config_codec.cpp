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

TEST_CASE("config codec: optional start pose round trips and rejects malformed coordinates") {
    const RobotConfigCodec codec;
    const JsonValueCodec json;
    RobotConfig c;
    CHECK(codec.toValue(c).find("startPose") == nullptr);
    c.startPose = RobotStartPose{4.51f, 0.52654f, 1.25f};
    const auto back = codec.fromValue(codec.toValue(c));
    REQUIRE(back.ok());
    CHECK(back.value().startPose == c.startPose);
    for (const auto* text : {R"({"startPose":null})", R"({"startPose":{"xM":1,"yM":2}})",
                             R"({"startPose":{"xM":"1","yM":2,"headingRad":0}})"}) {
        const auto parsed = json.parse(text);
        REQUIRE(parsed.ok());
        CHECK_FALSE(codec.fromValue(parsed.value()).ok());
    }
}

TEST_CASE("config codec: start positions round trip and reject an unknown alliance") {
    const RobotConfigCodec codec;
    const JsonValueCodec json;
    RobotConfig c;
    c.startPositions = {{"blue_right", "Blue right", Alliance::Blue, {4.51f, 0.52654f, 0.f}},
                        {"red_right", "Red right", Alliance::Red, {12.03f, 7.54f, 3.1f}}};
    const auto back = codec.fromValue(codec.toValue(c));
    REQUIRE(back.ok());
    CHECK(back.value().startPositions == c.startPositions);
    CHECK(codec.fromValue(codec.toValue(RobotConfig{})).value().startPositions.empty());
    for (const auto* text : {R"({"startPositions":[{"id":"a","alliance":"green","xM":1,"yM":2,"headingRad":0}]})",
                             R"({"startPositions":[{"id":"a","alliance":"red","xM":1,"yM":2}]})",
                             R"({"startPositions":{}})"}) {
        const auto parsed = json.parse(text);
        REQUIRE(parsed.ok());
        CHECK_FALSE(codec.fromValue(parsed.value()).ok());
    }
}

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

TEST_CASE("config codec: zones survive a roundtrip with their attachment and activation") {
    RobotConfig c = sampleConfig();
    ZoneConfig z;
    z.id = "pickup";
    z.kind = "fake_game.fake_zone";
    z.label = "Floor pickup";
    z.attachment.parentMechanismId = "intake";
    z.attachment.parentLink = "tip";
    z.local = Pose3{Vec3{0.1f, -0.2f, 0.3f}, Quat::fromYaw(0.25f)};
    z.sizeM = Vec3{0.5f, 0.4f, 0.2f};
    z.properties["capacity"] = 12;
    z.activation.mode = ActivationMode::Any;
    z.activation.conditions.push_back({"intake", "velocity", ConditionOp::Within, 12.5, 0.25, false});
    z.activation.conditions.push_back({"intake", "position", ConditionOp::Below, 1.5, 0.1, true});
    c.zones.push_back(z);
    ZoneConfig onChassis;
    onChassis.id = "muzzle";
    onChassis.kind = "fake_game.fake_zone";
    c.zones.push_back(onChassis);

    const RobotConfigCodec codec;
    const JsonValueCodec json;
    const Value v = codec.toValue(c);
    const Value& z0 = v.find("zones")->array()[0];
    CHECK(z0.find("attachment")->find("parentLink")->asString() == "tip");
    CHECK(z0.find("activation")->find("mode")->asString() == "any");
    CHECK(z0.find("activation")->find("conditions")->array()[0].find("op")->asString() == "within");
    CHECK(v.find("zones")->array()[1].find("attachment")->find("parentMechanismId")->asString() == "chassis");

    const auto parsed = json.parse(json.dump(v, true));
    REQUIRE(parsed.ok());
    const auto back = codec.fromValue(parsed.value());
    REQUIRE_MESSAGE(back.ok(), (back.ok() ? "" : back.error().message));
    REQUIRE(back.value().zones.size() == 2);
    CHECK(back.value().zones[0] == z);
    CHECK(back.value().zones[1] == onChassis);
    CHECK(json.dump(codec.toValue(back.value()), true) == json.dump(v, true));

    // a missing key reads as no zones; malformed zones are reported with their path
    Value bare = v;
    bare.object().erase("zones");
    CHECK(codec.fromValue(bare).value().zones.empty());
    Value bad = v;
    bad["zones"].array()[0]["activation"]["conditions"].array()[0]["op"] = "sideways";
    const auto err = codec.fromValue(bad);
    REQUIRE_FALSE(err.ok());
    CHECK(err.error().message.find("zones[0].activation.conditions[0].op") != std::string::npos);
}

TEST_CASE("config codec: cameras survive a roundtrip, default when fields are absent, and report paths") {
    RobotConfig c = sampleConfig();
    CameraConfig cam;
    cam.id = "front";
    cam.label = "Front limelight";
    cam.name = "front";
    cam.kind = CameraKind::PhotonVision;
    cam.attachment.parentMechanismId = "intake";
    cam.attachment.parentLink = "tip";
    cam.local = Pose3{Vec3{0.1f, -0.2f, 0.3f}, Quat::fromYaw(0.25f)};
    cam.hFovDeg = 62.5;
    cam.vFovDeg = 48.0;
    cam.maxRangeM = 7.5;
    cam.maxIncidenceDeg = 60.0;
    cam.fps = 45.0;
    cam.latencyMs = 25.0;
    cam.widthPx = 960;
    cam.heightPx = 720;
    cam.positionNoiseM = 0.02;
    cam.yawNoiseDeg = 1.5;
    c.cameras.push_back(cam);
    CameraConfig onChassis;
    onChassis.id = "rear";
    c.cameras.push_back(onChassis);

    const RobotConfigCodec codec;
    const JsonValueCodec json;
    const Value v = codec.toValue(c);
    const Value& c0 = v.find("cameras")->array()[0];
    CHECK(c0.find("kind")->asString() == "photonvision");
    CHECK(v.find("cameras")->array()[1].find("kind")->asString() == "limelight");
    CHECK(v.find("cameras")->array()[1].find("attachment")->find("parentMechanismId")->asString() == "chassis");

    const auto parsed = json.parse(json.dump(v, true));
    REQUIRE(parsed.ok());
    const auto back = codec.fromValue(parsed.value());
    REQUIRE_MESSAGE(back.ok(), (back.ok() ? "" : back.error().message));
    REQUIRE(back.value().cameras.size() == 2);
    CHECK(back.value().cameras[0] == cam);
    CHECK(back.value().cameras[1] == onChassis);
    CHECK(json.dump(codec.toValue(back.value()), true) == json.dump(v, true));

    // absent = no cameras; absent fields take the CameraConfig defaults
    Value bare = v;
    bare.object().erase("cameras");
    CHECK(codec.fromValue(bare).value().cameras.empty());
    Value sparse = v;
    sparse["cameras"] = Value::makeArray();
    Value minimal = Value::makeObject();
    minimal["id"] = "min";
    sparse["cameras"].push_back(std::move(minimal));
    const auto defaults = codec.fromValue(sparse);
    REQUIRE(defaults.ok());
    CameraConfig expected;
    expected.id = "min";
    CHECK(defaults.value().cameras[0] == expected);

    for (const auto& [key, wrong] : std::vector<std::pair<std::string, Value>>{
             {"kind", Value("laser")}, {"fps", Value("fast")}, {"widthPx", Value(1.5)}, {"pose", Value(3)}}) {
        Value bad = v;
        bad["cameras"].array()[0][key] = wrong;
        const auto err = codec.fromValue(bad);
        REQUIRE_FALSE(err.ok());
        CHECK(err.error().message.find("cameras[0]." + key) != std::string::npos);
    }
    Value notArray = v;
    notArray["cameras"] = Value::makeObject();
    CHECK(codec.fromValue(notArray).error().message.find("cameras: expected an array") != std::string::npos);
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

TEST_CASE("config codec: the physics worker count is a machine choice and never reaches the model file") {
    RobotConfig config = sampleConfig();
    config.sim.physics.workerCount = 4;
    const RobotConfigCodec codec;
    CHECK(codec.toValue(config).find("sim")->find("physics")->find("workerCount") == nullptr);

    Value legacy = roundTripValue(sampleConfig());
    legacy["sim"]["physics"]["workerCount"] = 4;
    const auto r = codec.fromValue(legacy);
    REQUIRE(r.ok());
    CHECK(r.value().sim.physics.workerCount == 1);
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
