#include <doctest/doctest.h>

#include <limits>

#include "runtime_fixtures.h"
#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
#include "wpiemu/core/physics/drivetrains.h"
#include "wpiemu/core/physics/tire_models.h"
#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core;

namespace {

struct Rig {
    MechanismFactory mechanisms;
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;

    Rig() {
        REQUIRE(wpiemu::core::mechanisms::registerGenericMechanisms(mechanisms).ok());
        REQUIRE(physics::registerDrivetrains(drivetrains).ok());
        REQUIRE(physics::registerTireModels(tires).ok());
    }

    std::unique_ptr<runtime::SimulationSession> build(RobotConfig config) {
        runtime::SessionParts parts;
        parts.world = std::make_unique<testing::FakePhysicsWorld>(config.sim.physics);
        parts.config = std::move(config);
        parts.mechanisms = &mechanisms;
        parts.drivetrains = &drivetrains;
        parts.tireModels = &tires;
        auto result = runtime::SessionBuilder::build(std::move(parts));
        REQUIRE_MESSAGE(result.ok(), (result.ok() ? "" : result.error().message));
        return std::move(result).value();
    }
};

RobotConfig tank() {
    RobotConfig c;
    c.motionLinksConfigured = true;
    c.drivetrain.spec.kind = drivetrain_kind::kTank;
    for (const int side : {1, -1})
        for (int i = 0; i < 3; ++i) {
            WheelSpec wheel;
            wheel.name = std::string(side > 0 ? "left" : "right") + std::to_string(i);
            wheel.positionM = {0.2f * static_cast<float>(i - 1), 0.3f * static_cast<float>(side),
                               wheel.radiusM};
            c.drivetrain.spec.wheels.push_back(wheel);
        }
    c.drivetrain.mappings = {
        {{slot_owner::kDrivetrain, "left1.drive"}, {"phoenix6", "talonfx", 1, "", -1}},
        {{slot_owner::kDrivetrain, "right1.drive"}, {"phoenix6", "talonfx", 2, "", -1}}};
    return c;
}

PlantState state(runtime::SimulationSession& session, const SlotAddress& address) {
    if (address.owner == slot_owner::kDrivetrain) {
        for (auto& axis : session.drivetrain()->motionAxes())
            if (axis.info.address == address) return axis.state();
    } else {
        for (auto& axis : session.mechanisms().find(address.owner)->motionAxes())
            if (axis.info.address == address) return axis.state();
    }
    FAIL("Unknown motion axis " << address.toString());
    return {};
}

}  // namespace

TEST_CASE("motion links: tank conversion keeps two motors and six wheels") {
    auto c = tank();
    const auto result = tankMotionLinks(c);
    REQUIRE(result.ok());
    c.motionLinks = result.value();
    CHECK(c.motionLinks.size() == 4);
    CHECK(c.drivetrain.mappings.size() == 2);
    CHECK(c.drivetrain.spec.wheels.size() == 6);
    CHECK(motionLinkTo(c, {slot_owner::kDrivetrain, "left0.drive"})->source.slot == "left1.drive");
    auto again = tankMotionLinks(c);
    REQUIRE(again.ok());
    CHECK(again.value().size() == 4);
    c.drivetrain.mappings.push_back(
        {{slot_owner::kDrivetrain, "left0.drive"}, {"phoenix6", "talonfx", 3, "", -1}});
    CHECK_FALSE(tankMotionLinks(c).ok());
    CHECK(c.drivetrain.mappings.size() == 3);
}

TEST_CASE(
    "motion links: validation rejects cycles, duplicate targets, invalid ratios and mapped "
    "followers") {
    auto c = tank();
    c.motionLinks = tankMotionLinks(c).value();
    const auto axes = motionAxisInfos(
        c, [](std::string_view) -> const MechanismTemplateInfo* { return nullptr; });
    CHECK(motionLinkIssues(c, axes).empty());
    SUBCASE("cycle") {
        c.motionLinks.push_back({"cycle",
                                 {slot_owner::kDrivetrain, "left0.drive"},
                                 {slot_owner::kDrivetrain, "left1.drive"},
                                 1});
    }
    SUBCASE("duplicate target") {
        c.motionLinks.push_back({"duplicate", c.motionLinks[0].source, c.motionLinks[0].target, 1});
    }
    SUBCASE("invalid ratio") {
        c.motionLinks[0].ratio = std::numeric_limits<double>::infinity();
    }
    SUBCASE("zero ratio") {
        c.motionLinks[0].ratio = 0;
    }
    SUBCASE("missing endpoint") {
        c.motionLinks[0].source.slot = "gone";
    }
    SUBCASE("mapped follower") {
        c.motionLinks[0].target.slot = "right1.drive";
    }
    CHECK_FALSE(motionLinkIssues(c, axes).empty());
}

TEST_CASE("motion links: linked wheels follow the source in the same tick without extra current") {
    Rig rig;
    auto c = tank();
    c.motionLinks = tankMotionLinks(c).value();
    c.motionLinks[0].ratio = -2.0;
    auto s = rig.build(c);
    s->hub().onAnnounce(wpiemu::test_runtime::talonAnnounce(101, 1));
    s->hub().onAnnounce(wpiemu::test_runtime::talonAnnounce(102, 2));
    proto::ActuatorFrame outputs;
    outputs.samples = {{DeviceId{101}, 1, 4.0}, {DeviceId{102}, 1, 4.0}};
    s->hub().onActuatorFrame(outputs);
    proto::SensorFrame frame;
    for (int i = 0; i < 20; ++i) {
        s->tick(frame, true);
        const auto source = state(*s, {slot_owner::kDrivetrain, "left1.drive"});
        const auto follower = state(*s, {slot_owner::kDrivetrain, "left0.drive"});
        CHECK(source.position > 0.0);
        CHECK(follower.position == doctest::Approx(-2.0 * source.position));
        CHECK(follower.velocity == doctest::Approx(-2.0 * source.velocity));
        CHECK(s->drivetrain()->wheelStates()[0].driveMotorCurrentA == 0);
    }
    s->reset();
    CHECK(state(*s, {slot_owner::kDrivetrain, "left0.drive"}).position == 0.0);
    s->tick(frame, true);
    CHECK(state(*s, {slot_owner::kDrivetrain, "left0.drive"}).velocity == 0.0);
}

TEST_CASE(
    "motion links: cross-owner chains use final motion, preserve starts and clamp only followers") {
    Rig rig;
    RobotConfig c;
    c.motionLinksConfigured = true;
    MechanismConfig source;
    source.id = "source";
    source.type = "roller";
    source.mappings = {{{"source", "motor"}, {"phoenix6", "talonfx", 1, "", -1}}};
    MechanismConfig follower;
    follower.id = "follower";
    follower.type = "elevator";
    follower.properties["startPosition"] = 0.2;
    follower.properties["drumRadiusM"] = 0.05;
    follower.mappings = {{{"follower", "encoder"}, {"phoenix6", "cancoder", 4, "", -1}}};
    MechanismConfig chained;
    chained.id = "chained";
    chained.type = "roller";
    // Deliberately opposite to dependency order.
    c.mechanisms = {chained, follower, source};
    c.motionLinks = {{"linear", {"source", "axis"}, {"follower", "axis"}, 0.1},
                     {"reverse", {"follower", "axis"}, {"chained", "axis"}, -2.0}};
    auto s = rig.build(c);
    s->hub().onAnnounce(wpiemu::test_runtime::talonAnnounce(101, 1));
    s->hub().onAnnounce(wpiemu::test_runtime::cancoderAnnounce(104, 4));
    CHECK_FALSE(s->bindSlot({"follower", "motor"}, {"phoenix6", "talonfx", 5, "", -1}).ok());
    proto::ActuatorFrame outputs;
    outputs.samples = {{DeviceId{101}, 1, 4.0}};
    s->hub().onActuatorFrame(outputs);
    proto::SensorFrame frame;
    for (int i = 0; i < 200; ++i) {
        s->tick(frame, true);
        const auto a = state(*s, {"source", "axis"});
        const auto b = state(*s, {"follower", "axis"});
        const auto d = state(*s, {"chained", "axis"});
        CHECK(b.position == doctest::Approx(std::min(0.8, 0.2 + 0.1 * a.position)));
        CHECK(d.position == doctest::Approx(-2.0 * (b.position - 0.2)));
        CHECK(d.velocity == doctest::Approx(-2.0 * b.velocity));
        CHECK(s->mechanisms().find("follower")->batteryCurrentA() == 0.0);
        bool positionSeen = false, velocitySeen = false;
        for (const auto& sample : frame.samples)
            if (sample.device == DeviceId{104}) {
                if (sample.signal == 0) {
                    CHECK(sample.value == doctest::Approx(b.position / (0.05 * kTwoPi)));
                    positionSeen = true;
                } else if (sample.signal == 1) {
                    CHECK(sample.value == doctest::Approx(b.velocity / (0.05 * kTwoPi)));
                    velocitySeen = true;
                }
            }
        CHECK(positionSeen);
        CHECK(velocitySeen);
    }
    CHECK(state(*s, {"follower", "axis"}).position == doctest::Approx(0.8));
    CHECK(state(*s, {"follower", "axis"}).velocity == 0.0);
    CHECK(state(*s, {"source", "axis"}).velocity > 0.0);
    s->reset();
    CHECK(state(*s, {"follower", "axis"}).position == doctest::Approx(0.2));
}

TEST_CASE("motion links: codec preserves link and endpoint unknown fields") {
    config::RobotConfigCodec codec;
    auto c = tank();
    c.motionLinks = tankMotionLinks(c).value();
    auto stored = codec.toValue(c);
    stored["motionLinks"].array()[0]["future"] = "kept";
    stored["motionLinks"].array()[0]["source"]["future"] = 42;
    auto decoded = codec.fromValue(stored);
    REQUIRE(decoded.ok());
    CHECK(codec.toValue(decoded.value()) == stored);
}

TEST_CASE(
    "motion links: pipeline upgrades tank layouts once and preserves mappings and unrelated data") {
    Rig rig;
    config::RobotConfigCodec codec;
    config::ConfigValidator validator(&rig.tires, {});
    config::PluginRegistrar registrar(rig.mechanisms);
    GamePluginRegistry plugins;
    config::ConfigPipeline pipeline(codec, validator, registrar, rig.drivetrains, plugins);
    auto c = tank();
    c.motionLinksConfigured = false;
    c.unknownFields[""]["futureSetting"] = 17;
    auto loaded = pipeline.process(c, {});
    REQUIRE(loaded.ok());
    CHECK(loaded.value().valid());
    CHECK(loaded.value().migrated());
    CHECK(loaded.value().config.motionLinks.size() == 4);
    CHECK(loaded.value().config.drivetrain.mappings.size() == 2);
    CHECK(loaded.value().config.drivetrain.mappings[0].slot == c.drivetrain.mappings[0].slot);
    CHECK(loaded.value().config.unknownFields == c.unknownFields);
    auto again = pipeline.load(codec.toValue(loaded.value().config));
    REQUIRE(again.ok());
    CHECK(again.value().valid());
    CHECK_FALSE(again.value().migrated());
    CHECK(again.value().config.motionLinks.size() == 4);

    SUBCASE("explicit independent mode is retained") {
        c.motionLinksConfigured = true;
        auto independent = pipeline.process(c, {});
        REQUIRE(independent.ok());
        CHECK(independent.value().config.motionLinks.empty());
        CHECK_FALSE(independent.value().migrated());
    }
    SUBCASE("conflicts preserve every motor mapping") {
        c.drivetrain.mappings.push_back(
            {{slot_owner::kDrivetrain, "left0.drive"}, {"phoenix6", "talonfx", 3, "", -1}});
        auto conflict = pipeline.process(c, {});
        REQUIRE(conflict.ok());
        CHECK_FALSE(conflict.value().valid());
        CHECK(conflict.value().config.motionLinks.empty());
        CHECK(conflict.value().config.drivetrain.mappings.size() == 3);
        CHECK_FALSE(conflict.value().config.motionLinksConfigured);
    }
}

TEST_CASE("motion links: wheel and mechanism chains have no cross-owner tick delay") {
    Rig rig;
    auto c = tank();
    MechanismConfig source;
    source.id = "source";
    source.type = "roller";
    source.mappings = {{{"source", "motor"}, {"phoenix6", "talonfx", 3, "", -1}}};
    MechanismConfig follower;
    follower.id = "follower";
    follower.type = "roller";
    c.mechanisms = {follower, source};
    c.drivetrain.mappings.clear();
    c.motionLinks = {
        {"toWheel", {"source", "axis"}, {slot_owner::kDrivetrain, "left0.drive"}, 2},
        {"fromWheel", {slot_owner::kDrivetrain, "left0.drive"}, {"follower", "axis"}, -0.5}};
    auto s = rig.build(c);
    s->hub().onAnnounce(wpiemu::test_runtime::talonAnnounce(103, 3));
    proto::ActuatorFrame outputs;
    outputs.samples = {{DeviceId{103}, 1, 4.0}};
    s->hub().onActuatorFrame(outputs);
    proto::SensorFrame frame;
    for (int i = 0; i < 20; ++i) {
        s->tick(frame, true);
        const auto a = state(*s, {"source", "axis"});
        const auto b = state(*s, {slot_owner::kDrivetrain, "left0.drive"});
        const auto d = state(*s, {"follower", "axis"});
        CHECK(a.position > 0);
        CHECK(b.position == doctest::Approx(2 * a.position));
        CHECK(d.position == doctest::Approx(-a.position));
        CHECK(d.velocity == doctest::Approx(-a.velocity));
    }
}
