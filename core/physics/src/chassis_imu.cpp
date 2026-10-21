#include "wpiemu/core/physics/chassis_imu.h"

namespace wpiemu::core::physics {

void ChassisImuSensor::reset() {
    hasLastVelocity_ = false;
    lastVelocity_ = {};
    last_ = {};
}

sdk::ImuFeedback ChassisImuSensor::update(const sdk::IPhysicsWorld& world, sdk::BodyId body, float dt) {
    if (!world.isValid(body)) return last_ = {};
    const sdk::Quat rot = world.pose(body).rotation;
    const sdk::Quat inv = rot.conjugate();
    const sdk::Vec3 v = world.linearVelocity(body);

    const sdk::Vec3 a = (hasLastVelocity_ && dt > 0.f) ? (v - lastVelocity_) / dt : sdk::Vec3{};
    lastVelocity_ = v;
    hasLastVelocity_ = true;

    const sdk::Euler e = rot.toEuler();
    sdk::ImuFeedback out;
    out.yawRad = e.yawRad;
    out.pitchRad = e.pitchRad;
    out.rollRad = e.rollRad;
    out.angularVelocityRadPerSec = inv.rotate(world.angularVelocity(body));
    out.accelerationMps2 = inv.rotate(a - gravity_);
    last_ = out;
    return out;
}

}  // namespace wpiemu::core::physics
