#include <doctest/doctest.h>

#include "runtime_fixtures.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"

using namespace wpiemu::sdk;
using wpiemu::core::config::DecodeInfo;
using wpiemu::core::config::JsonValueCodec;
using wpiemu::core::config::RobotConfigCodec;
using wpiemu::test_runtime::sampleConfig;

namespace {

// Moves a v0 file ("wheelBase" key) to v1 ("name"): used to prove the core migration chain runs first.
class RenameMigrator final : public IConfigMigrator {
public:
    int fromVersion() const override { return 0; }
    int toVersion() const override { return 1; }
    Status migrate(Value& data) const override {
        if (!data.isObject()) return makeError(ErrorCode::ParseError, "not an object");
        if (const Value* old = data.find("robotName")) {
            const Value moved = *old;
            data.object().erase("robotName");
            data["name"] = moved;
        }
        data["schemaVersion"] = 1;
        return okStatus();
    }
};

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
    c.mechanisms[0].geometry.push_back({"roller", PrimitiveKind::Cylinder, Pose3{}, Vec3{}, 0.04f, 0.3f, "#00ff00", false, 0.f, "arm"});
    c.sim.seed = 12345;
    c.sim.physics.gravity = Vec3{0.f, 0.f, -9.81f};
    c.plugins["fake_game"] = PluginSection{2, Value::makeObject()};
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
    CHECK(r.mechanisms[0].mount.position == c.mechanisms[0].mount.position);
    CHECK(r.mechanisms[0].geometry[0].kind == PrimitiveKind::Cylinder);
    CHECK(r.mechanisms[0].geometry[0].link == "arm");
    CHECK(r.chassis.primitives[0].link.empty());  // default: derived from the name prefix by the run view
    CHECK(r.robotMappings == c.robotMappings);
    CHECK(r.sim.seed == 12345);
    CHECK(r.sim.physics.gravity.z == -9.81f);
    CHECK(r.gamePluginId == "fake_game");
    REQUIRE(r.findPlugin("fake_game") != nullptr);
    CHECK(r.findPlugin("fake_game")->schemaVersion == 2);
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

TEST_CASE("config codec: writes both legacy parent and structured attachment, structured wins on read") {
    RobotConfig c = sampleConfig();
    MechanismConfig arm;
    arm.id = "arm";
    arm.type = "roller";
    arm.attachTo("intake", "tip");
    c.mechanisms.push_back(arm);
    c.mechanisms[0].parent = "chassis";  // legacy only, no structured form

    const Value v = roundTripValue(c);
    const Value& m0 = v.find("mechanisms")->array()[0];
    CHECK(m0.find("parent")->asString() == "chassis");
    CHECK(m0.find("attachment")->find("parentLink")->asString() == "root");
    const Value& m1 = v.find("mechanisms")->array()[1];
    CHECK(m1.find("parent")->asString() == "intake");
    CHECK(m1.find("attachment")->find("parentMechanismId")->asString() == "intake");
    CHECK(m1.find("attachment")->find("parentLink")->asString() == "tip");

    // a hand-edited file where the two disagree: the structured form wins and `parent` is re-synced
    Value edited = v;
    edited["mechanisms"].array()[1]["parent"] = "chassis";
    const auto r = RobotConfigCodec{}.fromValue(edited);
    REQUIRE(r.ok());
    CHECK(r.value().mechanisms[1].parent == "intake");
    REQUIRE(r.value().mechanisms[1].attachment.has_value());
    CHECK(r.value().mechanisms[1].attachment->parentLink == "tip");

    // a 1.0 file with only `parent`: no structured attachment is invented
    Value legacy = v;
    legacy["mechanisms"].array()[1].object().erase("attachment");
    legacy["mechanisms"].array()[1]["parent"] = "intake";
    const auto l = RobotConfigCodec{}.fromValue(legacy);
    REQUIRE(l.ok());
    CHECK_FALSE(l.value().mechanisms[1].attachment.has_value());
    CHECK(l.value().mechanisms[1].resolvedAttachment().parentMechanismId == "intake");
}

TEST_CASE("config codec: unknown fields are preserved at every level") {
    Value v = roundTripValue(sampleConfig());
    v["futureRootField"] = Value::makeObject();
    v["futureRootField"]["x"] = 1;
    v["chassis"]["wedgeAngle"] = 12.5;
    v["drivetrain"]["wheels"].array()[0]["rollerCount"] = 9;
    v["mechanisms"].array()[0]["notes"] = "hand written";
    v["plugins"] = Value::makeObject();
    Value section = Value::makeObject();
    section["schemaVersion"] = 1;
    section["data"] = Value::makeObject();
    section["extra"] = "kept";
    v["plugins"].object().set("other_game", section);

    const RobotConfigCodec codec;
    const auto r = codec.fromValue(v);
    REQUIRE(r.ok());
    const Value out = codec.toValue(r.value());
    CHECK(out.find("futureRootField")->find("x")->asInt() == 1);
    CHECK(out.find("chassis")->find("wedgeAngle")->asDouble() == 12.5);
    CHECK(out.find("drivetrain")->find("wheels")->array()[0].find("rollerCount")->asInt() == 9);
    CHECK(out.find("mechanisms")->array()[0].find("notes")->asString() == "hand written");
    CHECK(out.find("plugins")->find("other_game")->find("extra")->asString() == "kept");
    CHECK(out == v);  // the whole document is unchanged by a load/save cycle
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

TEST_CASE("config codec: newer schema is refused, older runs the migration chain") {
    Value newer = roundTripValue(sampleConfig());
    newer["schemaVersion"] = kRobotConfigSchemaVersion + 1;
    const auto refused = RobotConfigCodec{}.fromValue(newer);
    REQUIRE_FALSE(refused.ok());
    CHECK(refused.error().code == ErrorCode::VersionMismatch);

    // no migrator for v0 -> VersionMismatch
    Value old = roundTripValue(sampleConfig());
    old["schemaVersion"] = 0;
    CHECK(RobotConfigCodec{}.fromValue(old).error().code == ErrorCode::VersionMismatch);

    // with a v0 -> v1 migrator the old key is renamed first, then decoded
    std::vector<std::unique_ptr<IConfigMigrator>> chain;
    chain.push_back(std::make_unique<RenameMigrator>());
    const RobotConfigCodec codec(std::move(chain));
    Value v0 = Value::makeObject();
    v0["schemaVersion"] = 0;
    v0["robotName"] = "Legacy";
    DecodeInfo info;
    const auto r = codec.fromValue(v0, &info);
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
    CHECK(r.value().name == "Legacy");
    CHECK(r.value().schemaVersion == kRobotConfigSchemaVersion);
    CHECK(info.migrated);
    CHECK(info.persistedVersion == 0);

    DecodeInfo current;
    REQUIRE(codec.fromValue(roundTripValue(sampleConfig()), &current).ok());
    CHECK_FALSE(current.migrated);
    CHECK(RobotConfigCodec::storedVersion(Value::makeObject()) == 1);
    CHECK(RobotConfigCodec::storedVersion(Value(5)) == 0);
}

TEST_CASE("config codec: failing migrator aborts the load") {
    class Failing final : public IConfigMigrator {
    public:
        int fromVersion() const override { return 0; }
        int toVersion() const override { return 1; }
        Status migrate(Value&) const override { return makeError(ErrorCode::InvalidState, "boom"); }
    };
    std::vector<std::unique_ptr<IConfigMigrator>> chain;
    chain.push_back(std::make_unique<Failing>());
    Value v0 = Value::makeObject();
    v0["schemaVersion"] = 0;
    const auto r = RobotConfigCodec(std::move(chain)).fromValue(v0);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message == "boom");
}
