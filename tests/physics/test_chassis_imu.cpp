#include <doctest/doctest.h>

#include <cmath>

#include "physics_test_support.h"
#include "wpiemu/core/physics/chassis_imu.h"

using namespace physics_test;

TEST_CASE("imu: Euler extraction matches Quat::fromEuler and stays finite at gimbal lock") {
    for (float roll : {-0.7f, 0.f, 0.3f})
        for (float pitch : {-0.9f, 0.f, 0.4f})
            for (float yaw : {-3.0f, 0.f, 2.0f}) {
                const phys::EulerAngles e = phys::eulerZyx(Quat::fromEuler(roll, pitch, yaw));
                CHECK(e.rollRad == doctest::Approx(roll).epsilon(1e-4));
                CHECK(e.pitchRad == doctest::Approx(pitch).epsilon(1e-4));
                CHECK(e.yawRad == doctest::Approx(yaw).epsilon(1e-4));
            }
    const phys::EulerAngles lock = phys::eulerZyx(Quat::fromEuler(0.1f, 0.5f * kPiF, 0.2f));
    CHECK(std::isfinite(lock.pitchRad));
    CHECK(std::isfinite(lock.rollRad));
    CHECK(lock.pitchRad == doctest::Approx(0.5f * kPiF).epsilon(1e-3));
}

TEST_CASE("imu: at rest the accelerometer reads +g up in the chassis frame, in free fall it reads zero") {
    auto w = makeWorld();
    addFloor(*w);
    const BodyId ball = addBall(*w, {0.f, 0.f, 2.f}, 0.1f, 100.f, layer::Fuel, layer::All, 0, false);
    phys::ChassisImuSensor imu;
    for (int i = 0; i < 20; ++i) {
        w->step(kDt);
        imu.update(*w, ball, kDt);
    }
    CHECK(length(imu.last().accelerationMps2) < 0.6f);  // free fall: specific force is ~0
    for (int i = 0; i < 600; ++i) {
        w->step(kDt);
        imu.update(*w, ball, kDt);
    }
    CHECK(imu.last().accelerationMps2.z == doctest::Approx(9.81f).epsilon(0.1));
    CHECK(std::fabs(imu.last().accelerationMps2.x) < 0.5f);
    CHECK(imu.last().pitchRad == doctest::Approx(0.f).epsilon(0.1));
}

TEST_CASE("imu: attitude, body-frame gyro and tilted gravity") {
    PhysicsConfig zero;
    zero.gravity = {0.f, 0.f, 0.f};
    auto w = makeWorld(zero);
    const BodyId b = addBall(*w, {0.f, 0.f, 0.f}, 0.1f, 100.f, layer::Fuel, layer::All, 0, false);
    phys::ChassisImuSensor imu;  // reports specific force against the default -9.81 gravity

    const float yaw = 0.8f, pitch = 0.3f, roll = -0.2f;
    w->setPose(b, Pose3{{}, Quat::fromEuler(roll, pitch, yaw)});
    w->setAngularVelocity(b, {0.f, 0.f, 0.f});
    imu.update(*w, b, kDt);
    const ImuFeedback f = imu.update(*w, b, kDt);
    CHECK(f.yawRad == doctest::Approx(yaw).epsilon(1e-3));
    CHECK(f.pitchRad == doctest::Approx(pitch).epsilon(1e-3));
    CHECK(f.rollRad == doctest::Approx(roll).epsilon(1e-3));
    // R^T (0,0,g): x = -g sin(pitch), z = g cos(pitch) cos(roll)
    CHECK(f.accelerationMps2.x == doctest::Approx(-9.81f * std::sin(pitch)).epsilon(0.01));
    CHECK(f.accelerationMps2.z == doctest::Approx(9.81f * std::cos(pitch) * std::cos(roll)).epsilon(0.01));

    // A world-frame spin about +Z shows up in the chassis frame rotated by the attitude.
    w->setAngularVelocity(b, {0.f, 0.f, 2.f});
    const ImuFeedback g = imu.update(*w, b, kDt);
    const Vec3 expected = Quat::fromEuler(roll, pitch, yaw).conjugate().rotate({0.f, 0.f, 2.f});
    CHECK(g.angularVelocityRadPerSec.x == doctest::Approx(expected.x).epsilon(1e-3));
    CHECK(g.angularVelocityRadPerSec.z == doctest::Approx(expected.z).epsilon(1e-3));
}

TEST_CASE("imu: invalid bodies give neutral output and reset clears the history") {
    auto w = makeWorld();
    phys::ChassisImuSensor imu;
    const ImuFeedback none = imu.update(*w, BodyId{}, kDt);
    CHECK(none.accelerationMps2 == Vec3{});
    CHECK(none.yawRad == 0.0);

    const BodyId b = addBall(*w, {0.f, 0.f, 5.f}, 0.1f, 100.f, layer::Fuel, layer::All, 0, false);
    w->step(kDt);
    imu.update(*w, b, kDt);
    imu.reset();
    CHECK(imu.last().yawRad == 0.0);
    // First sample after a reset has no velocity history: acceleration = reaction to gravity only.
    const ImuFeedback first = imu.update(*w, b, kDt);
    CHECK(first.accelerationMps2.z == doctest::Approx(9.81f).epsilon(0.01));
}

TEST_CASE("imu: the chassis simulator publishes yaw and yaw rate truth") {
    Rig rig(withMu(swerveSpec(), 0.001f));  // free-spinning on the floor: tire scrub would stop the yaw otherwise
    REQUIRE(rig.sim);
    rig.world->setAngularVelocity(rig.sim->body(), {0.f, 0.f, 1.0f});
    rig.run(0.2f);
    CHECK(rig.sim->imu().yawRad > 0.f);
    CHECK(rig.sim->imu().angularVelocityRadPerSec.z > 0.1f);
    CHECK(rig.sim->imu().yawRad == doctest::Approx(rig.world->pose(rig.sim->body()).rotation.yaw()).epsilon(1e-3));
}
