// Wiring test: the session over the REAL physics module (Box3D world, brush tire, swerve drivetrain, chassis coupling).
// Compiled only where core/physics is part of the build; the unit tests above use the sdk fakes.
#if __has_include("wpiemu/core/physics/box3d_world.h")

#include <doctest/doctest.h>

#include "runtime_fixtures.h"
#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/core/runtime/physics_wiring.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
namespace physics = wpiemu::core::physics;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;
using wpiemu::test_runtime::talonAnnounce;

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

#endif
