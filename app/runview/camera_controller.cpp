#include "camera_controller.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::app::runview {
namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kFov = 45.f;

float wrapPi(float a) {
    while (a > kPi) a -= 2.f * kPi;
    while (a < -kPi) a += 2.f * kPi;
    return a;
}

sdk::Vec3 sphericalOffset(float yaw, float pitch, float dist) {
    return {dist * std::cos(pitch) * std::cos(yaw), dist * std::cos(pitch) * std::sin(yaw), dist * std::sin(pitch)};
}

}  // namespace

CameraController::CameraController() { reset(); }

void CameraController::setField(const sdk::Vec3& center, float lengthM, float widthM) {
    fieldCenter_ = center;
    fieldLength_ = lengthM > 0.f ? lengthM : fieldLength_;
    fieldWidth_ = widthM > 0.f ? widthM : fieldWidth_;
    reset();
}

void CameraController::setRobot(const sdk::Vec3& position, float yawRad) {
    robotPos_ = position;
    robotYaw_ = yawRad;
}

void CameraController::setMode(CameraMode mode) {
    if (mode == mode_) return;
    mode_ = mode;
    if (mode_ == CameraMode::Follow) {
        followTarget_ = robotPos_;
        followYaw_ = robotYaw_;
    }
}

void CameraController::reset() {
    const float tanHalf = std::tan(kFov * 0.5f * kPi / 180.f);
    target_ = fieldCenter_;
    yaw_ = -kPi * 0.5f;   // side-on, so both alliance walls are visible
    pitch_ = 0.4f;
    distance_ = std::max(fieldLength_, fieldWidth_) * 1.05f;
    followYawOffset_ = 0.f;
    followPitch_ = 0.45f;
    followDistance_ = 3.5f;
    followTarget_ = robotPos_;
    followYaw_ = robotYaw_;
    topCenter_ = fieldCenter_;
    topHeight_ = std::max(fieldLength_ / (2.f * tanHalf * 1.6f), fieldWidth_ / (2.f * tanHalf)) * 1.05f;
}

void CameraController::clampState() {
    // Pitch is unrestricted: it wraps so the camera can pass over the poles and under the ground.
    pitch_ = wrapPi(pitch_);
    followPitch_ = wrapPi(followPitch_);
    const float span = std::max(fieldLength_, fieldWidth_);
    distance_ = std::clamp(distance_, 0.2f, span * 6.f);
    followDistance_ = std::clamp(followDistance_, 0.5f, 60.f);
    topHeight_ = std::clamp(topHeight_, 3.f, span * 3.f);
}

void CameraController::orbit(float dxPx, float dyPx) {
    switch (mode_) {
        case CameraMode::Orbit:
            yaw_ = wrapPi(yaw_ - dxPx * 0.0075f);
            pitch_ += dyPx * 0.0075f;
            break;
        case CameraMode::Follow:
            followYawOffset_ = wrapPi(followYawOffset_ - dxPx * 0.0075f);
            followPitch_ += dyPx * 0.0075f;
            break;
        case CameraMode::TopDown:
            pan(dxPx, dyPx);
            return;
    }
    clampState();
}

void CameraController::pan(float dxPx, float dyPx) {
    if (mode_ == CameraMode::Follow) return;
    if (mode_ == CameraMode::TopDown) {
        const float perPx = topHeight_ * 0.0017f;
        topCenter_.x -= dxPx * perPx;
        topCenter_.y += dyPx * perPx;
        return;
    }
    // Move the target in the ground plane relative to the camera heading.
    const float perPx = distance_ * 0.0015f;
    const sdk::Vec3 right{-std::sin(yaw_), std::cos(yaw_), 0.f};
    const sdk::Vec3 forward{-std::cos(yaw_), -std::sin(yaw_), 0.f};
    target_ += right * (-dxPx * perPx) + forward * (dyPx * perPx);
}

void CameraController::zoom(float steps) {
    const float f = std::pow(0.9f, steps);
    switch (mode_) {
        case CameraMode::Orbit: distance_ *= f; break;
        case CameraMode::Follow: followDistance_ *= f; break;
        case CameraMode::TopDown: topHeight_ *= f; break;
    }
    clampState();
}

void CameraController::centerOnRobot() {
    target_ = robotPos_;
    topCenter_ = robotPos_;
}

void CameraController::advance(double dt) {
    const float alpha = static_cast<float>(1.0 - std::exp(-std::max(0.0, dt) / 0.12));
    followTarget_ += (robotPos_ - followTarget_) * alpha;
    followYaw_ = wrapPi(followYaw_ + wrapPi(robotYaw_ - followYaw_) * alpha);
}

sdk::Quat CameraController::lookRotation(const sdk::Vec3& forwardIn, const sdk::Vec3& upHint) {
    const sdk::Vec3 f = sdk::normalized(forwardIn);
    sdk::Vec3 right = sdk::cross(f, upHint);
    if (sdk::length(right) < 1e-5f) right = sdk::cross(f, sdk::Vec3{0.f, 1.f, 0.f});
    right = sdk::normalized(right);
    const sdk::Vec3 up = sdk::cross(right, f);
    const sdk::Vec3 z = -f;
    const float m00 = right.x, m01 = up.x, m02 = z.x;
    const float m10 = right.y, m11 = up.y, m12 = z.y;
    const float m20 = right.z, m21 = up.z, m22 = z.z;
    const float trace = m00 + m11 + m22;
    sdk::Quat q;
    if (trace > 0.f) {
        const float s = std::sqrt(trace + 1.f) * 2.f;
        q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.f + m00 - m11 - m22) * 2.f;
        q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
    } else if (m11 > m22) {
        const float s = std::sqrt(1.f + m11 - m00 - m22) * 2.f;
        q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
    } else {
        const float s = std::sqrt(1.f + m22 - m00 - m11) * 2.f;
        q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
    }
    return q.normalized();
}

CameraPose CameraController::pose() const {
    CameraPose p;
    p.fieldOfViewDeg = kFov;
    const sdk::Vec3 up{0.f, 0.f, 1.f};
    // Past the poles the camera is upside down; flip the up hint so the view rolls over smoothly.
    const sdk::Vec3 orbitUp = std::cos(pitch_) < 0.f ? sdk::Vec3{0.f, 0.f, -1.f} : up;
    const sdk::Vec3 followUp = std::cos(followPitch_) < 0.f ? sdk::Vec3{0.f, 0.f, -1.f} : up;
    switch (mode_) {
        case CameraMode::Orbit: {
            p.position = target_ + sphericalOffset(yaw_, pitch_, distance_);
            p.rotation = lookRotation(target_ - p.position, orbitUp);
            break;
        }
        case CameraMode::Follow: {
            const sdk::Vec3 focus = followTarget_ + sdk::Vec3{0.f, 0.f, 0.25f};
            p.position = focus + sphericalOffset(followYaw_ + kPi + followYawOffset_, followPitch_, followDistance_);
            p.rotation = lookRotation(focus - p.position, followUp);
            break;
        }
        case CameraMode::TopDown: {
            p.position = {topCenter_.x, topCenter_.y, topHeight_};
            p.rotation = lookRotation({0.f, 0.f, -1.f}, {0.f, 1.f, 0.f});
            break;
        }
    }
    return p;
}

}  // namespace wpiemu::app::runview
