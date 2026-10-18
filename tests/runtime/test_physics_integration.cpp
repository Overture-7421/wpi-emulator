// Wiring test: the session over the REAL physics module (Box3D world, brush tire, swerve drivetrain, chassis coupling).
// Compiled only where core/physics is part of the build; the unit tests above use the sdk fakes.
#if __has_include("wpiemu/core/physics/box3d_world.h")

#include <doctest/doctest.h>

#include "runtime_fixtures.h"
#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/core/runtime/physics_wiring.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
namespace physics = wpiemu::core::physics;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;
using wpiemu::test_runtime::talonAnnounce;

TEST_CASE("integration: a start pose and a teleport put the real Box3D robot upright on the floor at the right spot") {
    Registries reg;
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    REQUIRE(physics::registerPhysicsComponents(drivetrains, tires).ok());

    RobotConfig c = sampleConfig();
    c.sim.physics = PhysicsConfig{};  // real gravity
    c.drivetrain.spec.kind = "swerve";
    c.drivetrain.spec.tireModel = "brush";
    c.drivetrain.mappings.clear();
    for (WheelSpec& w : c.drivetrain.spec.wheels) {
        w.kind = WheelKind::Swerve;
        w.steered = true;
        w.positionM.z = w.radiusM;
    }
    c.mechanisms.clear();
    c.robotMappings.clear();
    c.sim.startPose = StartPose{6.f, -3.f, 1.0f};

    SessionParts parts;
    parts.world = physics::createBox3dWorld(c.sim.physics);
    REQUIRE(parts.world != nullptr);
    parts.config = c;
    parts.mechanisms = &reg.mechanisms;
    parts.drivetrains = &drivetrains;
    parts.tireModels = &tires;
    parts.chassisBuilder = physicsChassisBuilder(physics::ChassisPlacement{1.f, 1.f, 0.f});   // the config wins over this placement
    parts.coupling = std::make_unique<PhysicsCouplingDriver>();
    auto built = SessionBuilder::build(std::move(parts));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    SimulationSession& s = *built.value();
    BodyDesc floor;
    floor.type = BodyType::Static;
    floor.pose.position = Vec3{0.f, 0.f, -0.5f};
    ShapeDesc slab;
    slab.geometry = BoxShape{Vec3{30.f, 30.f, 0.5f}};
    slab.material = Material{0.8f, 0.f, 0.f};
    slab.category = layer::Field;
    REQUIRE(s.world().addShape(s.world().createBody(floor), slab).valid());

    const auto settle = [&](int ticks) {
        proto::SensorFrame out;
        for (int i = 0; i < ticks; ++i) s.tick(out, i == 0);
    };
    const Pose3 spawn = s.spawnPose();
    CHECK(spawn.position.x == 6.f);
    CHECK(spawn.position.y == -3.f);
    CHECK(spawn.rotation.yaw() == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(spawn.position.z > 0.f);   // the builder's rest height is kept: the chassis does not start inside the floor
    settle(300);
    Pose3 pose = s.world().pose(s.chassisBody());
    CHECK(std::abs(pose.position.x - 6.f) < 0.05f);
    CHECK(std::abs(pose.position.y + 3.f) < 0.05f);
    CHECK(std::abs(pose.rotation.yaw() - 1.0f) < 0.05f);
    CHECK(std::abs(pose.position.z - spawn.position.z) < 0.05f);   // resting on its wheels

    s.setRobotPose(Pose3{Vec3{-4.f, 2.f, spawn.position.z}, Quat::fromYaw(-2.0f)});
    CHECK(s.world().linearVelocity(s.chassisBody()) == Vec3{});
    settle(300);
    pose = s.world().pose(s.chassisBody());
    CHECK(std::abs(pose.position.x + 4.f) < 0.05f);
    CHECK(std::abs(pose.position.y - 2.f) < 0.05f);
    CHECK(std::abs(pose.rotation.yaw() + 2.0f) < 0.05f);
    CHECK(std::abs(pose.position.z - spawn.position.z) < 0.05f);

    s.reset();
    CHECK(s.world().pose(s.chassisBody()) == spawn);
}

TEST_CASE("integration: the session drives a real Box3D swerve robot from hub voltages") {
    Registries reg;  // brings the roller mechanism + plugin registry; drivetrains/tires come from core/physics
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    REQUIRE(physics::registerPhysicsComponents(drivetrains, tires).ok());

    RobotConfig c = sampleConfig();
    c.sim.physics = PhysicsConfig{};  // real gravity
    c.drivetrain.spec.kind = "swerve";
    c.drivetrain.spec.tireModel = "brush";
    c.drivetrain.mappings.clear();
    int can = 1;
    for (WheelSpec& w : c.drivetrain.spec.wheels) {
        w.kind = WheelKind::Swerve;
        w.steered = true;
        w.positionM.z = w.radiusM;
        c.drivetrain.mappings.push_back({{slot_owner::kDrivetrain, w.name + ".drive"}, DeviceKey{"phoenix6", "talonfx", can, "rio", -1}});
        c.drivetrain.mappings.push_back({{slot_owner::kDrivetrain, w.name + ".steer"}, DeviceKey{"phoenix6", "talonfx", can + 10, "rio", -1}});
        ++can;
    }
    c.mechanisms.clear();
    c.robotMappings.clear();

    SessionParts parts;
    parts.world = physics::createBox3dWorld(c.sim.physics);
    REQUIRE(parts.world != nullptr);
    parts.config = c;
    parts.mechanisms = &reg.mechanisms;
    parts.drivetrains = &drivetrains;
    parts.tireModels = &tires;
    parts.chassisBuilder = physicsChassisBuilder();
    parts.coupling = std::make_unique<PhysicsCouplingDriver>();
    auto built = SessionBuilder::build(std::move(parts));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    SimulationSession& s = *built.value();

    BodyDesc floor;
    floor.type = BodyType::Static;
    floor.pose.position = Vec3{0.f, 0.f, -0.5f};
    const BodyId floorBody = s.world().createBody(floor);
    ShapeDesc slab;
    slab.geometry = BoxShape{Vec3{30.f, 30.f, 0.5f}};
    slab.material = Material{0.8f, 0.f, 0.f};
    slab.category = layer::Field;
    REQUIRE(s.world().addShape(floorBody, slab).valid());

    // devices: drive talons 1..4 (runtime ids 101..104), steer talons 11..14 (111..114)
    for (std::uint32_t i = 0; i < 4; ++i) {
        s.hub().onAnnounce(talonAnnounce(101 + i, static_cast<int>(i) + 1));
        s.hub().onAnnounce(talonAnnounce(111 + i, static_cast<int>(i) + 11));
    }
    CHECK(s.hub().loopbackSet().loopback.empty());

    proto::ActuatorFrame outputs;
    for (std::uint32_t i = 0; i < 4; ++i) {
        outputs.samples.push_back({DeviceId{101 + i}, 1, 6.0});  // drive motorVoltage
        outputs.samples.push_back({DeviceId{111 + i}, 1, 0.0});  // steer
    }
    s.hub().onActuatorFrame(outputs);

    proto::SensorFrame out;
    double firstDrivePosition = 0.0, lastDrivePosition = 0.0;
    for (int i = 0; i < 200; ++i) {
        s.tick(out, i == 0);
        for (const auto& sample : out.samples)
            if (sample.device == DeviceId{101} && sample.signal == 3) {  // rotorPosition of the front-left drive motor
                if (firstDrivePosition == 0.0) firstDrivePosition = sample.value;
                lastDrivePosition = sample.value;
            }
    }
    const Vec3 v = s.world().linearVelocity(s.chassisBody());
    const Pose3 pose = s.world().pose(s.chassisBody());
    CHECK(v.x > 0.2f);                      // driving forward
    CHECK(pose.position.x > 0.05f);
    CHECK(std::abs(pose.position.y) < 0.3f);
    CHECK(pose.position.z > 0.05f);         // still on the floor, not through it or flying
    CHECK(pose.position.z < 0.5f);
    CHECK(lastDrivePosition > firstDrivePosition);  // wheel feedback reaches the hub as rotor rotations
    CHECK(s.battery().voltageV() < 12.6);
    CHECK(s.mechanisms().all().empty());
    const auto snap = s.buildSnapshot(false, true);
    CHECK(snap->wheels().size() == 4);
    CHECK(snap->chassis().pose.position.x == pose.position.x);

    s.reset();
    CHECK(std::abs(s.world().pose(s.chassisBody()).position.x) < 0.01f);
    CHECK(s.clock().tickIndex() == 0);
}

TEST_CASE("integration: two motors move and turn a six-contact linked tank in Box3D") {
    Registries reg;
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    REQUIRE(physics::registerPhysicsComponents(drivetrains, tires).ok());
    RobotConfig c;
    c.drivetrain.spec.kind = drivetrain_kind::kTank;
    for (int side : {1, -1})
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
    c.motionLinks = tankMotionLinks(c).value();
    SessionParts parts;
    parts.world = physics::createBox3dWorld(c.sim.physics);
    parts.config = c;
    parts.mechanisms = &reg.mechanisms;
    parts.drivetrains = &drivetrains;
    parts.tireModels = &tires;
    parts.chassisBuilder = physicsChassisBuilder();
    parts.coupling = std::make_unique<PhysicsCouplingDriver>();
    auto built = SessionBuilder::build(std::move(parts));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    auto& s = *built.value();
    BodyDesc floor;
    floor.type = BodyType::Static;
    floor.pose.position.z = -0.5f;
    ShapeDesc slab;
    slab.geometry = BoxShape{Vec3{30.f, 30.f, 0.5f}};
    slab.material = Material{0.8f, 0.f, 0.f};
    slab.category = layer::Field;
    REQUIRE(s.world().addShape(s.world().createBody(floor), slab).valid());
    s.hub().onAnnounce(talonAnnounce(101, 1));
    s.hub().onAnnounce(talonAnnounce(102, 2));
    proto::ActuatorFrame outputs;
    outputs.samples = {{DeviceId{101}, 1, 4.0}, {DeviceId{102}, 1, 4.0}};
    s.hub().onActuatorFrame(outputs);
    proto::SensorFrame frame;
    for (int i = 0; i < 200; ++i) s.tick(frame, true);
    CHECK(s.world().pose(s.chassisBody()).position.x > 0.05f);
    const auto wheels = s.drivetrain()->wheelStates();
    REQUIRE(wheels.size() == 6);
    for (std::size_t i : {0u, 2u, 3u, 5u}) {
        const std::size_t source = i < 3 ? 1 : 4;
        CHECK(wheels[i].driveWheelAngleRad == doctest::Approx(wheels[source].driveWheelAngleRad));
        CHECK(wheels[i].driveWheelOmegaRadPerSec ==
              doctest::Approx(wheels[source].driveWheelOmegaRadPerSec));
        CHECK(wheels[i].driveMotorCurrentA == 0);
        CHECK(wheels[i].normalLoadN > 0);
    }
    outputs.samples[0].value = -3.0;
    outputs.samples[1].value = 3.0;
    s.hub().onActuatorFrame(outputs);
    for (int i = 0; i < 200; ++i) s.tick(frame, true);
    CHECK(std::abs(s.world().angularVelocity(s.chassisBody()).z) > 0.01f);
}

#endif
