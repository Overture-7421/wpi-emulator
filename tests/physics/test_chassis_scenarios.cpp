#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "physics_test_support.h"
#include "refusing_world.h"

// ADR 0003 validation scenarios as automated tests. Setup: 50 kg, 0.5 m x 0.5 m wheel square, COM 0.2 m,
// mu 1.1, 4 in wheels, Kraken X60 at 6.75:1, 200 Hz, 4 substeps. mu g = 10.79 m/s^2, sliding 0.85 mu g = 9.17.

using namespace physics_test;

namespace {

constexpr double kWheelRadius = 0.0508;

// Voltage of an ideal motor that turns the wheel so its rim moves at `speed` m/s.
double voltsForRim(const WheelSpec& w, double speed) { return steadyVoltage(w, speed / kWheelRadius); }

// Mecanum inverse kinematics (WPILib X layout, ADR 0003): wheel rim speed = (vc . c) / cos(phi).
std::array<double, 4> mecanumRimSpeeds(const DrivetrainSpec& d, double vx, double vy, double wz) {
    std::array<double, 4> out{};
    for (std::size_t i = 0; i < 4; ++i) {
        const double x = d.wheels[i].positionM.x, y = d.wheels[i].positionM.y;
        const double phi = (x * y > 0 ? -1.0 : 1.0) * 0.7853981633974483;
        const double vcx = vx - wz * y, vcy = vy + wz * x;
        out[i] = (vcx * std::cos(phi) + vcy * std::sin(phi)) / std::cos(phi);
    }
    return out;
}

void commandMecanum(Rig& rig, double vx, double vy, double wz) {
    const auto rim = mecanumRimSpeeds(rig.drivetrainSpec, vx, vy, wz);
    for (std::size_t i = 0; i < 4; ++i) rig.cmd[i].driveVoltageV = voltsForRim(rig.drivetrainSpec.wheels[i], rim[i]);
}

float speedOver(Rig& rig, float seconds) {
    rig.run(seconds);
    return rig.bodyVelocity().x;
}

}  // namespace

TEST_CASE("chassis: rests on its wheel pads with the static load split") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    rig.run(1.0f);
    const Vec3 p = rig.position();
    CHECK(p.z == doctest::Approx(0.2f).epsilon(0.02));
    CHECK(length(rig.world->linearVelocity(rig.sim->body())) < 0.02f);

    float sum = 0.f;
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) {
        CHECK_FALSE(s.airborne);
        CHECK(s.normalLoadN == doctest::Approx(122.6f).epsilon(0.03));
        sum += s.normalLoadN;
    }

    CHECK(sum == doctest::Approx(490.5f).epsilon(0.01));
    CHECK(rig.sim->imu().accelerationMps2.z == doctest::Approx(9.81f).epsilon(0.05));
}

TEST_CASE("chassis builder rolls back every late shape failure and permits retry") {
    for (const int budget : {0, 1, 2, 3, 4, 5, 6}) {
        INFO(budget);
        RefusingWorld world;
        const auto existingBody = world.createBody({});
        sdk::ShapeDesc existingShape;
        existingShape.geometry = sdk::SphereShape{0.1f};
        REQUIRE(world.addShape(existingBody, existingShape).valid());
        const auto bodies = world.stats().bodies;
        const auto shapes = world.inner.shapeCount();
        auto chassis = testChassis();
        for (const char* name : {"first", "second"}) {
            sdk::PrimitiveSpec primitive;
            primitive.name = name;
            primitive.kind = sdk::PrimitiveKind::Box;
            primitive.sizeM = {0.1f, 0.1f, 0.1f};
            primitive.collides = true;
            chassis.primitives.push_back(primitive);
        }
        const auto drivetrain = swerveSpec();
        world.shapeBudget = budget;
        CHECK_FALSE(phys::buildChassisBody(world, chassis, drivetrain, {}).ok());
        CHECK(world.stats().bodies == bodies);
        CHECK(world.inner.shapeCount() == shapes);
        CHECK(world.isValid(existingBody));
        world.shapeBudget = 100;
        auto retry = phys::buildChassisBody(world, chassis, drivetrain, {});
        REQUIRE(retry.ok());
        CHECK(world.stats().bodies == bodies + 1);
        CHECK(world.inner.shapeCount() == shapes + 7);
        world.destroyBody(retry.value().body);
        CHECK(world.stats().bodies == bodies);
        CHECK(world.inner.shapeCount() == shapes);
    }
}

TEST_CASE("chassis builder rejects late invalid primitive dimensions without changing the world") {
    RefusingWorld world;
    auto chassis = testChassis();
    sdk::PrimitiveSpec first;
    first.kind = sdk::PrimitiveKind::Box;
    first.sizeM = {0.1f, 0.1f, 0.1f};
    first.collides = true;
    chassis.primitives.push_back(first);
    auto invalid = first;
    invalid.name = "bad_later_primitive";
    invalid.sizeM = {};
    chassis.primitives.push_back(invalid);
    CHECK_FALSE(phys::buildChassisBody(world, chassis, swerveSpec(), {}).ok());
    CHECK(world.stats().bodies == 0);
    CHECK(world.inner.shapeCount() == 0);
    chassis.primitives.back().sizeM = {0.1f, 0.1f, 0.1f};
    auto retry = phys::buildChassisBody(world, chassis, swerveSpec(), {});
    REQUIRE(retry.ok());
    world.destroyBody(retry.value().body);
    CHECK(world.stats().bodies == 0);
    CHECK(world.inner.shapeCount() == 0);
}

TEST_CASE("chassis: 1 V stays below the traction limit with no slip") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    rig.run(0.5f);  // settle
    rig.setDrive(1.0);
    const float v01 = speedOver(rig, 0.1f);
    CHECK(v01 / 0.1f > 2.0f);
    CHECK(v01 / 0.1f < 3.6f);
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) CHECK(std::fabs(s.slipRatio) < 0.2f);
    rig.run(3.0f);
    CHECK(rig.bodyVelocity().x == doctest::Approx(0.397f).epsilon(0.08));  // free speed of the gearbox at 1 V
    CHECK(std::fabs(rig.bodyVelocity().y) < 0.01f);
}

TEST_CASE("chassis: acceleration with excess motor torque is limited by mu N at the sliding level") {
    Rig rig(swerveSpec(0.f));  // no current limit
    REQUIRE(rig.sim);
    rig.run(0.5f);
    rig.setDrive(12.0);
    const float v = speedOver(rig, 0.3f);
    const float a = v / 0.3f;
    CHECK(a > 8.4f);
    CHECK(a < 10.79f);  // never above mu g
    float maxSlip = 0.f;
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) maxSlip = std::max(maxSlip, s.slipRatio);
    CHECK(maxSlip > 0.2f);  // the driven wheels are skidding
    // Skidding wheels spin much faster than the body moves.
    const auto states = rig.sim->drivetrain().wheelStates();
    CHECK(states[0].driveWheelOmegaRadPerSec * kWheelRadius > 1.3 * rig.bodyVelocity().x);
}

TEST_CASE("chassis: a stator current limit lowers the acceleration below the traction limit") {
    Rig limited(swerveSpec(60.f));
    REQUIRE(limited.sim);
    limited.run(0.5f);
    limited.setDrive(12.0);
    const float a = speedOver(limited, 0.3f) / 0.3f;
    CHECK(a > 6.0f);
    CHECK(a < 8.2f);
}

TEST_CASE("chassis: load transfer unloads the front wheels under hard acceleration") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    rig.run(0.5f);
    rig.setDrive(12.0);
    rig.run(0.3f);
    const auto s = rig.sim->drivetrain().wheelStates();  // fl fr rl rr
    CHECK(s[0].normalLoadN < 70.f);
    CHECK(s[1].normalLoadN < 70.f);
    CHECK(s[2].normalLoadN > 170.f);
    CHECK(s[3].normalLoadN > 170.f);
    CHECK(s[0].normalLoadN + s[1].normalLoadN + s[2].normalLoadN + s[3].normalLoadN == doctest::Approx(490.5f).epsilon(0.02));
    // The chassis pitches nose-up a little under the transfer; the IMU reports it (pitch < 0 = nose up).
    CHECK(rig.sim->imu().pitchRad < 0.f);
}

TEST_CASE("chassis: plug braking skids at the sliding friction level") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    rig.run(0.5f);
    rig.world->setLinearVelocity(rig.sim->body(), {4.f, 0.f, 0.f});
    rig.setDrive(-12.0);
    rig.run(0.3f);
    const float decel = (4.f - rig.bodyVelocity().x) / 0.3f;
    CHECK(decel > 8.0f);
    CHECK(decel < 10.8f);
    CHECK(rig.sim->drivetrain().wheelStates()[0].driveMotorCurrentA > 1.f);  // plugging draws from the battery
}

TEST_CASE("chassis: a coasting motor (0 V) at speed draws no battery current") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    rig.run(0.5f);
    rig.setDrive(5.0);
    rig.run(0.3f);
    CHECK(rig.sim->drivetrain().wheelStates()[0].driveMotorCurrentA > 0.f);
    rig.setDrive(0.0);
    rig.run(0.05f);
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) CHECK(s.driveMotorCurrentA == 0.0);
}

TEST_CASE("swerve: steering all modules to 90 degrees strafes like a forward drive") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    const std::array<double, 4> ninety{kPiF / 2, kPiF / 2, kPiF / 2, kPiF / 2};
    for (int i = 0; i < 200; ++i) {
        steerToward(rig, ninety);
        rig.step();
    }
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) CHECK(s.steerAngleRad == doctest::Approx(kPiF / 2).epsilon(0.02));
    rig.setDrive(voltsForRim(rig.drivetrainSpec.wheels[0], 2.0));
    for (int i = 0; i < 600; ++i) {
        steerToward(rig, ninety);
        rig.step();
    }
    const Vec3 v = rig.bodyVelocity();
    CHECK(v.y == doctest::Approx(2.0f).epsilon(0.03));
    CHECK(std::fabs(v.x) < 0.05f);
    CHECK(std::fabs(rig.yawRate()) < 0.05f);
}

TEST_CASE("swerve: modules in the tangent configuration spin the robot in place at the kinematic rate") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    std::array<double, 4> tangent{};
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec3 p = rig.drivetrainSpec.wheels[i].positionM;
        tangent[i] = std::atan2(p.x, -p.y);
    }
    for (int i = 0; i < 200; ++i) {
        steerToward(rig, tangent);
        rig.step();
    }
    const double idealRate = 8.485;
    rig.setDrive(voltsForRim(rig.drivetrainSpec.wheels[0], idealRate * std::sqrt(2.0) * 0.25));
    for (int i = 0; i < 800; ++i) {
        steerToward(rig, tangent);
        rig.step();
    }
    CHECK(rig.yawRate() == doctest::Approx(static_cast<float>(idealRate)).epsilon(0.03));
    CHECK(length(rig.world->linearVelocity(rig.sim->body())) < 0.15f);
    CHECK(rig.sim->imu().angularVelocityRadPerSec.z == doctest::Approx(rig.yawRate()).epsilon(0.02));
}

TEST_CASE("swerve: azimuth motors reach their targets and expose slot specs") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    const auto slots = rig.sim->drivetrain().slotSpecs();
    REQUIRE(slots.size() == 12);
    CHECK(slots[0].name == "fl.drive");
    CHECK(slots[1].name == "fl.steer");
    CHECK(slots[2].name == "fl.steerEncoder");
    CHECK(slots[2].type == SlotType::PositionSensor);
    CHECK_FALSE(slots[2].required);
    CHECK(slots[0].type == SlotType::Motor);
    CHECK(rig.sim->drivetrain().kind() == "swerve");
    const std::array<double, 4> targets{0.5, -0.5, 1.0, -1.0};
    for (int i = 0; i < 300; ++i) {
        steerToward(rig, targets);
        rig.step();
    }
    const auto st = rig.sim->drivetrain().wheelStates();
    for (std::size_t i = 0; i < 4; ++i) CHECK(st[i].steerAngleRad == doctest::Approx(targets[i]).epsilon(0.02));
    // A steer command with the motor unpowered stays put.
    for (WheelCommand& c : rig.cmd) c.steerVoltageV = 0.0;
    rig.run(0.5f);
    CHECK(std::fabs(rig.sim->drivetrain().wheelStates()[0].steerOmegaRadPerSec) < 0.1);
}

TEST_CASE("tank: four wheels drive straight at the commanded rim speed") {
    Rig rig(tankSpec(2, WheelKind::Standard));
    REQUIRE(rig.sim);
    CHECK(rig.sim->drivetrain().slotSpecs().size() == 4);  // no steer slots
    rig.run(0.5f);
    rig.setDrive(voltsForRim(rig.drivetrainSpec.wheels[0], 3.0));
    rig.run(4.0f);
    const Vec3 v = rig.bodyVelocity();
    CHECK(v.x == doctest::Approx(3.0f).epsilon(0.03));
    CHECK(std::fabs(v.y) < 0.05f);
    CHECK(std::fabs(rig.yawRate()) < 0.02f);
}

TEST_CASE("tank: six wheels (three per side) also drive straight") {
    Rig rig(tankSpec(3, WheelKind::Standard));
    REQUIRE(rig.sim);
    REQUIRE(rig.drivetrainSpec.wheels.size() == 6);
    rig.run(0.5f);
    rig.setDrive(voltsForRim(rig.drivetrainSpec.wheels[0], 2.0));
    rig.run(3.0f);
    CHECK(rig.bodyVelocity().x == doctest::Approx(2.0f).epsilon(0.03));
    CHECK(std::fabs(rig.yawRate()) < 0.02f);
    float sum = 0.f;
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) sum += s.normalLoadN;
    CHECK(sum == doctest::Approx(490.5f).epsilon(0.02));
}

TEST_CASE("tank: turning scrubs traction wheels but omni wheels turn nearly kinematically") {
    auto yawFor = [](WheelKind kind, float lateralScale) {
        Rig rig(tankSpec(2, kind, lateralScale));
        REQUIRE(rig.sim);
        rig.run(0.5f);
        const double v = voltsForRim(rig.drivetrainSpec.wheels[0], 1.0);
        for (std::size_t i = 0; i < rig.cmd.size(); ++i) rig.cmd[i].driveVoltageV = rig.drivetrainSpec.wheels[i].positionM.y > 0 ? -v : v;
        rig.run(3.0f);
        return rig.yawRate();  // ideal: 1 m/s on each side, track 0.5 m => 4 rad/s
    };
    const float traction = yawFor(WheelKind::Standard, 1.0f);
    const float omni = yawFor(WheelKind::Omni, 1.0f);  // Omni caps lateralScale at 0.15
    CHECK(traction > 0.8f);
    CHECK(traction < 2.6f);
    CHECK(omni > 3.2f);
    CHECK(omni < 4.3f);
    CHECK(omni > traction);
}

TEST_CASE("mecanum: wheel speeds from the WPILib kinematics reproduce the commanded chassis motion") {
    struct Case { double vx, vy, wz; };
    for (const Case c : {Case{2.0, 0.0, 0.0}, Case{0.0, 2.0, 0.0}, Case{0.0, 0.0, 3.0}, Case{1.5, 1.5, 1.0}}) {
        Rig rig(mecanumSpec());
        REQUIRE(rig.sim);
        rig.run(0.5f);
        commandMecanum(rig, c.vx, c.vy, c.wz);
        rig.run(4.0f);
        const Vec3 v = rig.bodyVelocity();
        const auto tol = [](double ref) { return std::max(0.12, 0.12 * std::fabs(ref)); };
        CHECK(std::fabs(v.x - c.vx) < tol(c.vx));
        CHECK(std::fabs(v.y - c.vy) < tol(c.vy));
        CHECK(std::fabs(rig.yawRate() - c.wz) < std::max(0.15, 0.06 * std::fabs(c.wz)));
    }
}

TEST_CASE("mecanum: traction-limited acceleration is the same forward and sideways and below swerve") {
    auto accelOf = [](double vx, double vy) {
        Rig rig(mecanumSpec());
        REQUIRE(rig.sim);
        rig.run(0.5f);
        const auto rim = mecanumRimSpeeds(rig.drivetrainSpec, vx, vy, 0.0);
        double peak = 0.0;
        for (double r : rim) peak = std::max(peak, std::fabs(r));
        for (std::size_t i = 0; i < 4; ++i) rig.cmd[i].driveVoltageV = 12.0 * rim[i] / peak;
        rig.run(0.3f);
        const Vec3 v = rig.bodyVelocity();
        return static_cast<float>(std::sqrt(v.x * v.x + v.y * v.y) / 0.3);
    };
    const float forward = accelOf(1.0, 0.0);
    const float strafe = accelOf(0.0, 1.0);
    CHECK(forward > 4.0f);
    CHECK(forward < 7.5f);
    CHECK(strafe == doctest::Approx(forward).epsilon(0.2));
}

TEST_CASE("chassis: driving up a 15 degree ramp pitches the chassis and keeps the total load at m g cos") {
    const float angle = 15.f * kPiF / 180.f;
    Rig rig(swerveSpec(), testChassis(), {}, {-0.5f, 0.f, 0.f});
    REQUIRE(rig.sim);
    addRamp(*rig.world, 1.0f, angle);
    rig.run(0.5f);
    rig.setDrive(4.5);
    float minZ = 10.f;
    for (int i = 0; i < 1000; ++i) {  // 5 s
        rig.step();
        minZ = std::min(minZ, rig.position().z);
    }
    const Vec3 p = rig.position();
    CHECK(p.x > 2.0f);
    CHECK(p.z > 0.6f);  // well up the ramp
    const auto imu = rig.sim->imu();
    CHECK(imu.pitchRad == doctest::Approx(-angle).epsilon(0.08));  // nose up = negative pitch (Rotation3d convention)
    CHECK(std::fabs(imu.rollRad) < 0.03f);
    float sum = 0.f;
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) sum += s.normalLoadN;
    CHECK(sum == doctest::Approx(490.5f * std::cos(angle)).epsilon(0.04));
    CHECK(rig.bodyVelocity().x > 0.8f);
}

TEST_CASE("chassis: sensor volumes added to the chassis body keep its mass and detect fuel while driving") {
    Rig rig(swerveSpec(), testChassis(), {}, {-1.0f, 0.f, 0.f});
    REQUIRE(rig.sim);
    ShapeDesc intake;
    intake.geometry = BoxShape{{0.15f, 0.3f, 0.1f}};
    intake.local.position = {0.45f, 0.f, -0.1f};
    intake.density = 500.f;  // would add mass if the explicit chassis mass were not authoritative
    intake.sensor = true;
    intake.sensorEvents = true;
    intake.category = layer::Sensor;
    intake.userTag = 4242;
    const ShapeId zone = rig.world->addShape(rig.sim->body(), intake);
    REQUIRE(zone.valid());
    addBall(*rig.world, {0.5f, 0.f, 0.075f}, 0.075f, 113.2f, layer::Fuel, layer::All, 99);

    rig.run(1.0f);
    float sum = 0.f;
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) sum += s.normalLoadN;
    CHECK(sum == doctest::Approx(490.5f).epsilon(0.01));  // mass unchanged by the extra shape
    CHECK(rig.position().z == doctest::Approx(0.2f).epsilon(0.03));

    rig.setDrive(steadyVoltage(rig.drivetrainSpec.wheels[0], 1.5 / 0.0508));
    bool began = false;
    for (int i = 0; i < 600; ++i) {
        rig.step();
        for (const SensorEvent& e : rig.world->sensorEvents())
            if (e.begin && e.sensorTag == 4242 && e.visitorTag == 99) began = true;
    }
    CHECK(began);
}

TEST_CASE("chassis: wheels report airborne while dropping and ground contact after landing") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    rig.world->setPose(rig.sim->body(), Pose3{{0.f, 0.f, 1.0f}, Quat::identity()});
    rig.step();
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) {
        CHECK(s.airborne);
        CHECK(s.normalLoadN == 0.f);
        CHECK(s.force == Vec3{});
    }
    rig.setDrive(6.0);
    rig.run(0.2f);
    // Free-falling wheels spin up without any chassis force.
    CHECK(rig.sim->drivetrain().wheelStates()[0].driveWheelOmegaRadPerSec > 5.0);
    rig.setDrive(0.0);
    rig.run(1.5f);
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) CHECK_FALSE(s.airborne);
    CHECK(rig.position().z == doctest::Approx(0.2f).epsilon(0.05));
}

TEST_CASE("chassis simulator: reset puts the robot back at rest on its placement") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    rig.setDrive(8.0);
    rig.run(1.0f);
    CHECK(rig.position().x > 0.2f);
    rig.sim->reset({3.f, -2.f, 1.0f});
    CHECK(rig.position().x == doctest::Approx(3.f));
    CHECK(rig.position().y == doctest::Approx(-2.f));
    CHECK(rig.world->pose(rig.sim->body()).rotation.yaw() == doctest::Approx(1.0f).epsilon(0.001));
    CHECK(length(rig.world->linearVelocity(rig.sim->body())) == 0.f);
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) {
        CHECK(s.driveWheelOmegaRadPerSec == 0.0);
        CHECK(s.driveWheelAngleRad == 0.0);
    }
    rig.setDrive(0.0);
    rig.run(1.0f);
    CHECK(rig.position().x == doctest::Approx(3.f).epsilon(0.02));
}

TEST_CASE("chassis simulator: creation fails cleanly for unknown kinds, models and bad specs") {
    PhysicsConfig cfg;
    auto world = makeWorld(cfg);
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    REQUIRE(phys::registerPhysicsComponents(drivetrains, tires).ok());
    const ChassisSpec chassis = testChassis();

    auto make = [&](DrivetrainSpec ds, ChassisSpec cs) {
        return phys::ChassisSimulator::create({*world, cs, ds, drivetrains, tires, {}, {}, cfg.gravity});
    };
    DrivetrainSpec unknownKind = swerveSpec();
    unknownKind.kind = "hovercraft";
    CHECK(make(unknownKind, chassis).error().code == ErrorCode::NotFound);
    DrivetrainSpec unknownTire = swerveSpec();
    unknownTire.tireModel = "wooden";
    CHECK(make(unknownTire, chassis).error().code == ErrorCode::NotFound);
    ChassisSpec heavyless = chassis;
    heavyless.massKg = 0.f;
    CHECK(make(swerveSpec(), heavyless).error().code == ErrorCode::InvalidArgument);
    ChassisSpec flat = chassis;
    flat.sizeM.z = 0.f;
    CHECK(make(swerveSpec(), flat).error().code == ErrorCode::InvalidArgument);
    ChassisSpec noCom = chassis;
    noCom.comHeightM = 0.f;
    CHECK(make(swerveSpec(), noCom).error().code == ErrorCode::InvalidArgument);
    DrivetrainSpec noRadius = swerveSpec();
    noRadius.wheels[0].radiusM = 0.f;
    CHECK(make(noRadius, chassis).error().code == ErrorCode::InvalidArgument);
    CHECK(world->stats().bodies == 0);  // failed creations leave no body behind
}

TEST_CASE("chassis simulator: destroying it removes the chassis body from the world") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    const BodyId body = rig.sim->body();
    CHECK(rig.world->isValid(body));
    rig.sim.reset();
    CHECK_FALSE(rig.world->isValid(body));
}

TEST_CASE("chassis builder: spec geometry, pads and mass properties") {
    ChassisSpec c = testChassis();
    c.comOffsetM = {0.05f, 0.f, 0.f};
    PrimitiveSpec bumper;
    bumper.name = "bumper";
    bumper.kind = PrimitiveKind::Cylinder;
    bumper.radiusM = 0.1f;
    bumper.heightM = 0.1f;
    bumper.collides = true;
    PrimitiveSpec visual;
    visual.name = "decal";
    c.primitives = {bumper, visual};
    Rig rig(swerveSpec(), c);
    REQUIRE(rig.sim);
    const phys::ChassisBody& cb = rig.sim->chassisBody();
    CHECK(cb.wheelPads.size() == 4);
    CHECK(cb.extraShapes.size() == 1);  // the non-colliding primitive adds nothing
    CHECK(cb.baseMass.massKg == 50.f);
    CHECK(cb.baseMass.inertiaDiagLocal.z > cb.baseMass.inertiaDiagLocal.x);
    CHECK(cb.restHeightM == doctest::Approx(0.2f));
    CHECK(rig.world->shapeBody(cb.hull) == cb.body);

    // Explicit inertia wins over the derived box inertia.
    ChassisSpec e = testChassis();
    e.inertiaDiagKgM2 = {1.f, 2.f, 3.f};
    CHECK(phys::chassisMassProperties(e).inertiaDiagLocal == Vec3{1.f, 2.f, 3.f});
    // Hub heights: z <= 0 means one radius; the body sits higher when a hub is above its radius.
    WheelSpec w = testWheel("w", 0.1f, 0.2f, WheelKind::Standard, false, 0.f);
    CHECK(phys::wheelHubLocal(e, w).z == doctest::Approx(0.0508f - 0.2f));
    w.positionM.z = 0.06f;
    CHECK(phys::wheelHubLocal(e, w).z == doctest::Approx(0.06f - 0.2f));
}

TEST_CASE("drivetrain registry: the shipped kinds can be registered, created and are unique") {
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    REQUIRE(phys::registerPhysicsComponents(drivetrains, tires).ok());
    CHECK(drivetrains.contains("swerve"));
    CHECK(drivetrains.contains("tank"));
    CHECK(drivetrains.contains("mecanum"));
    CHECK_FALSE(phys::registerDrivetrains(drivetrains).ok());
    CHECK(drivetrains.info("tank")->displayName.find("Tank") != std::string::npos);

    auto world = makeWorld();
    addFloor(*world);
    const ChassisSpec chassis = testChassis();
    const DrivetrainSpec spec = mecanumSpec();
    auto tire = tires.create("brush");
    REQUIRE(tire.ok());
    auto body = phys::buildChassisBody(*world, chassis, spec, {});
    REQUIRE(body.ok());
    const DrivetrainBuildArgs args{spec, chassis, *world, body.value().body, *tire.value()};
    for (const char* kind : {"swerve", "tank", "mecanum"}) {
        auto d = drivetrains.create(kind, args);
        REQUIRE(d.ok());
        CHECK(d.value()->kind() == kind);
        CHECK(d.value()->wheelStates().size() == 4);
        // Missing commands are treated as zero volts and nothing crashes with an empty command list.
        d.value()->step({}, kDt);
        d.value()->step({}, 0.f);
        d.value()->reset();
    }
}

TEST_CASE("drivetrain: non-finite commands are ignored") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    for (WheelCommand& c : rig.cmd) {
        c.driveVoltageV = std::numeric_limits<double>::quiet_NaN();
        c.steerVoltageV = std::numeric_limits<double>::infinity();
    }
    rig.run(0.5f);
    CHECK(std::isfinite(rig.position().x));
    CHECK(std::fabs(rig.bodyVelocity().x) < 0.05f);
}

namespace {

// A misbehaving third-party model: every output is NaN.
class NanTireModel final : public ITireModel {
public:
    std::string_view id() const override { return "nan_tire"; }
    TireOutput solve(const TireParams&, const TireInput&, const WheelDrive&) const override {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        return {nan, nan, nan, nan, nan, false};
    }
};

}  // namespace

TEST_CASE("drivetrain: a tire model returning NaN neither moves the body nor corrupts the wheel state") {
    PhysicsConfig cfg;
    auto world = makeWorld(cfg);
    addFloor(*world);
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    REQUIRE(phys::registerPhysicsComponents(drivetrains, tires).ok());
    REQUIRE(tires.registerModel("nan_tire", [] { return std::make_unique<NanTireModel>(); }).ok());
    DrivetrainSpec spec = tankSpec(2, WheelKind::Standard);
    spec.tireModel = "nan_tire";
    const ChassisSpec chassis = testChassis();
    phys::ChassisSimulatorArgs args{*world, chassis, spec, drivetrains, tires, {}, {}, cfg.gravity};
    auto created = phys::ChassisSimulator::create(args);
    REQUIRE(created.ok());
    auto sim = std::move(created).value();

    std::vector<WheelCommand> cmd(spec.wheels.size());
    for (WheelCommand& c : cmd) c.driveVoltageV = 6.0;
    for (int i = 0; i < 100; ++i) sim->step(cmd, kDt);

    const Vec3 p = world->pose(sim->body()).position;
    CHECK(std::isfinite(p.x));
    CHECK(std::isfinite(p.z));
    for (const WheelState& s : sim->drivetrain().wheelStates()) {
        CHECK(std::isfinite(s.driveWheelOmegaRadPerSec));
        CHECK(s.driveWheelOmegaRadPerSec == 0.0);  // the previous (initial) spin is kept
        CHECK(std::isfinite(s.driveWheelAngleRad));
        CHECK(std::isfinite(s.force.x));
        CHECK(s.force.x == 0.f);
        CHECK(std::isfinite(s.slipRatio));
    }
    CHECK(std::fabs(world->linearVelocity(sim->body()).x) < 0.05f);
}

namespace {

void driveMotors(Rig& rig, std::initializer_list<double> volts) {
    for (WheelCommand& c : rig.cmd) {
        c.driveMotors = static_cast<int>(volts.size());
        std::size_t i = 0;
        for (const double v : volts) c.driveMotorVoltagesV[i++] = v;
    }
}

DrivetrainSpec multiMotorTank(int motors) {
    DrivetrainSpec spec = tankSpec(2, WheelKind::Standard);
    for (WheelSpec& w : spec.wheels) w.driveMotorCount = motors;
    return spec;
}

}  // namespace

TEST_CASE("wheels with several motors: each motor adds its own voltage, mirrored motors match the old scalar command") {
    Rig scalar(multiMotorTank(3));
    scalar.setDrive(6.0);
    const float expected = speedOver(scalar, 0.5f);
    CHECK(expected > 0.1f);

    Rig mirrored(multiMotorTank(3));
    driveMotors(mirrored, {6.0});   // motors beyond the listed ones repeat motor 0
    CHECK(speedOver(mirrored, 0.5f) == doctest::Approx(expected).epsilon(1e-4));

    Rig mixed(multiMotorTank(3));
    driveMotors(mixed, {12.0, 6.0, 0.0});   // same mean voltage, so the same torque
    CHECK(speedOver(mixed, 0.5f) == doctest::Approx(expected).epsilon(1e-3));
}

TEST_CASE("wheels with opposed motors do not move the robot but still draw current") {
    Rig rig(multiMotorTank(2));
    driveMotors(rig, {12.0, -12.0});   // a motor that is reversed relative to its partner
    rig.run(0.5f);
    CHECK(std::fabs(rig.bodyVelocity().x) < 0.01f);
    for (const WheelState& s : rig.sim->drivetrain().wheelStates()) CHECK(s.driveMotorCurrentA > 1.0);
}

TEST_CASE("every drive motor of a wheel gets its own slot") {
    Rig rig(multiMotorTank(3));
    std::vector<std::string> names;
    for (const SlotSpec& s : rig.sim->drivetrain().slotSpecs()) names.push_back(s.name);
    const std::vector<std::string> left0 = {"left0.drive", "left0.drive2", "left0.drive3"};
    CHECK(std::vector<std::string>(names.begin(), names.begin() + 3) == left0);
    CHECK(names.size() == 12);
}