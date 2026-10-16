#include "wpiemu/core/physics/chassis_imu.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::core::physics {

EulerAngles eulerZyx(const sdk::Quat& q) {
    const float r11 = 1.f - 2.f * (q.y * q.y + q.z * q.z);
    const float r21 = 2.f * (q.x * q.y + q.w * q.z);
    const float r31 = 2.f * (q.x * q.z - q.w * q.y);
    const float r32 = 2.f * (q.y * q.z + q.w * q.x);
    const float r33 = 1.f - 2.f * (q.x * q.x + q.y * q.y);
    EulerAngles e;
    e.yawRad = std::atan2(r21, r11);
    e.pitchRad = std::asin(std::clamp(-r31, -1.f, 1.f));
    e.rollRad = std::atan2(r32, r33);
    return e;
}

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

    const EulerAngles e = eulerZyx(rot);
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
