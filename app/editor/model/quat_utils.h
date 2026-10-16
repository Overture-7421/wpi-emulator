#pragma once

// Euler <-> quaternion helpers for the editor's numeric entry (roll/pitch/yaw in degrees). The SDK
// only provides fromEuler (intrinsic Z-Y-X); the inverse lives here.

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::editor {

struct Euler {
    float rollRad = 0.f;
    float pitchRad = 0.f;
    float yawRad = 0.f;
};

inline Euler toEuler(const sdk::Quat& qIn) {
    const sdk::Quat q = qIn.normalized();
    Euler e;
    const float sinr = 2.f * (q.w * q.x + q.y * q.z);
    const float cosr = 1.f - 2.f * (q.x * q.x + q.y * q.y);
    e.rollRad = std::atan2(sinr, cosr);
    const float sinp = std::clamp(2.f * (q.w * q.y - q.z * q.x), -1.f, 1.f);
    e.pitchRad = std::asin(sinp);
    const float siny = 2.f * (q.w * q.z + q.x * q.y);
    const float cosy = 1.f - 2.f * (q.y * q.y + q.z * q.z);
    e.yawRad = std::atan2(siny, cosy);
    return e;
}

inline sdk::Quat fromEuler(const Euler& e) { return sdk::Quat::fromEuler(e.rollRad, e.pitchRad, e.yawRad); }

inline float degToRadF(float deg) { return static_cast<float>(sdk::degToRad(static_cast<double>(deg))); }
inline float radToDegF(float rad) { return static_cast<float>(sdk::radToDeg(static_cast<double>(rad))); }

// Rotation of `angleRad` about a unit axis, applied after `q` in the parent frame.
inline sdk::Quat rotatedAbout(const sdk::Quat& q, const sdk::Vec3& axis, float angleRad) {
    return (sdk::Quat::fromAxisAngle(axis, angleRad) * q).normalized();
}

}  // namespace wpiemu::app::editor
