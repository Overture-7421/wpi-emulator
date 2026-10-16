#pragma once

// Ground-truth IMU outputs of the chassis body for the sensor layer (the device bridge applies the device's
// own zeroing / mounting / noise on top). Conventions:
//   * yaw / pitch / roll: intrinsic Z-Y-X like sdk::Quat::fromEuler and WPILib Rotation3d (yaw CCW+ about +Z,
//     pitch about +Y: positive = nose DOWN, roll about +X: positive = right side down). Radians.
//   * angularVelocityRadPerSec: gyro rates in the CHASSIS frame.
//   * accelerationMps2: accelerometer reading (specific force) in the chassis frame, i.e. it includes the
//     reaction to gravity: (0, 0, +g) at rest on level ground.
// The IMU sits at the body origin (the chassis COM).

#include "wpiemu/sdk/devices/frames.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::core::physics {

struct EulerAngles {
    float yawRad = 0.f;
    float pitchRad = 0.f;
    float rollRad = 0.f;
};

EulerAngles eulerZyx(const sdk::Quat& q);

class ChassisImuSensor {
public:
    explicit ChassisImuSensor(sdk::Vec3 gravity = {0.f, 0.f, -9.81f}) : gravity_(gravity) {}

    // Call once after every IPhysicsWorld::step(dt). Returns neutral values for an invalid body.
    sdk::ImuFeedback update(const sdk::IPhysicsWorld& world, sdk::BodyId body, float dt);
    const sdk::ImuFeedback& last() const { return last_; }
    void reset();

private:
    sdk::Vec3 gravity_;
    sdk::Vec3 lastVelocity_;
    bool hasLastVelocity_ = false;
    sdk::ImuFeedback last_;
};

}  // namespace wpiemu::core::physics
