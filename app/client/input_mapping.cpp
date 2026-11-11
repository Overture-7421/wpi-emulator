#include "client/input_mapping.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/physics/remote_spec_limits.h"

namespace wpiemu::app::client {
namespace {

constexpr float kDegToRad = 3.14159265358979323846f / 180.f;

float axisOf(const sdk::proto::JoystickState& stick, int axis) {
    if (axis < 0 || static_cast<std::size_t>(axis) >= stick.axes.size()) return 0.f;
    const float v = stick.axes[static_cast<std::size_t>(axis)];
    return std::isfinite(v) ? std::clamp(v, -1.f, 1.f) : 0.f;
}

}  // namespace

DriveLimits driveLimitsOf(const sdk::RobotConfig& spec) {
    DriveLimits limits;
    limits.maxLinearMps = sdk::maxLinearSpeedMps(spec.drivetrain.spec);
    float radius = 0.f;
    for (const sdk::WheelSpec& wheel : spec.drivetrain.spec.wheels)
        radius = std::max(radius, std::hypot(wheel.positionM.x, wheel.positionM.y));
    limits.maxAngularRadPerS = radius > 0.05f ? limits.maxLinearMps / radius : 0.f;
    return limits;
}

DriveInputConfig normalizedDriveConfig(DriveInputConfig config) {
    const auto finiteOr = [](float v, float fallback) { return std::isfinite(v) ? v : fallback; };
    config.deadband = std::clamp(finiteOr(config.deadband, 0.10f), 0.f, 0.9f);
    config.linearScale = std::clamp(finiteOr(config.linearScale, 1.f), 0.05f, 1.f);
    config.angularScale = std::clamp(finiteOr(config.angularScale, 1.f), 0.05f, 1.f);
    return config;
}

float applyDeadband(float value, float deadband) {
    if (!std::isfinite(value)) return 0.f;
    const float db = std::clamp(deadband, 0.f, 0.99f);
    const float magnitude = std::fabs(value);
    if (magnitude <= db) return 0.f;
    const float scaled = std::min(1.f, (magnitude - db) / (1.f - db));
    return value < 0.f ? -scaled : scaled;
}

sdk::ChassisSpeeds mapDrive(const sdk::proto::JoystickState& stick, const DriveInputConfig& config, const DriveLimits& limits) {
    sdk::ChassisSpeeds out;
    if (!stick.present) return out;
    const DriveInputConfig cfg = normalizedDriveConfig(config);
    // WPILib convention: pushing a stick up or left reads negative.
    const float forward = -axisOf(stick, kForwardAxis);
    const float left = -axisOf(stick, kStrafeAxis);
    const float magnitude = std::hypot(forward, left);
    if (magnitude > 0.f) {
        const float scaled = applyDeadband(std::min(magnitude, 1.f), cfg.deadband);
        out.vx = forward / magnitude * scaled * limits.maxLinearMps * cfg.linearScale;
        out.vy = left / magnitude * scaled * limits.maxLinearMps * cfg.linearScale;
    }
    out.omega = applyDeadband(-axisOf(stick, kRotateAxis), cfg.deadband) * limits.maxAngularRadPerS * cfg.angularScale;
    return out;
}

std::uint32_t zoneFlagsOf(const sdk::proto::JoystickState& stick, std::size_t zoneCount) {
    if (!stick.present) return 0;
    std::uint32_t flags = 0;
    const std::size_t count = std::min({zoneCount, static_cast<std::size_t>(kMaxZoneButtons), static_cast<std::size_t>(stick.buttonCount)});
    for (std::size_t i = 0; i < count; ++i)
        if ((stick.buttons >> i) & 1u) flags |= 1u << i;
    return flags;
}

void ZonePoseControls::configure(const sdk::ChassisSpec& chassis, std::vector<sdk::Pose3> restPoses) {
    chassis_ = chassis;
    rest_ = std::move(restPoses);
    pose_ = rest_;
    yaw_.assign(rest_.size(), 0.f);
    moved_.assign(rest_.size(), 0);
    selected_ = 0;
    const sdk::ZoneEnvelope box = sdk::remoteZoneEnvelope(chassis);
    min_ = box.min;
    max_ = box.max;
}

void ZonePoseControls::select(std::size_t zone) {
    if (zone < rest_.size()) selected_ = zone;
}

const sdk::Pose3& ZonePoseControls::pose(std::size_t zone) const {
    static const sdk::Pose3 none;
    return zone < pose_.size() ? pose_[zone] : none;
}

const sdk::Pose3& ZonePoseControls::restPose(std::size_t zone) const {
    static const sdk::Pose3 none;
    return zone < rest_.size() ? rest_[zone] : none;
}

void ZonePoseControls::apply(std::size_t zone, const sdk::Vec3& position, float yaw) {
    sdk::Pose3 wanted;
    wanted.position = position;
    wanted.rotation = sdk::Quat::fromYaw(yaw) * rest_[zone].rotation;
    pose_[zone] = sdk::clampRemoteZonePose(chassis_, wanted).pose;
    yaw_[zone] = yaw;
    moved_[zone] = 1;
}

bool ZonePoseControls::set(std::size_t zone, const sdk::Vec3& position, float yawOffsetRad) {
    if (zone >= rest_.size() || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !std::isfinite(yawOffsetRad))
        return false;
    apply(zone, position, yawOffsetRad);
    return true;
}

bool ZonePoseControls::reset(std::size_t zone) {
    if (zone >= rest_.size()) return false;
    const bool was = moved_[zone] != 0;
    pose_[zone] = rest_[zone];
    yaw_[zone] = 0.f;
    moved_[zone] = 0;
    return was;
}

int ZonePoseControls::nudge(double dtSeconds, const sdk::proto::JoystickState& stick, float deadband) {
    if (!stick.present || rest_.empty() || !(dtSeconds > 0.0)) return -1;
    const float dt = static_cast<float>(std::min(dtSeconds, 0.1));
    float dx = 0.f;
    float dy = 0.f;
    if (!stick.povs.empty() && stick.povs[0] >= 0) {
        const float angle = static_cast<float>(stick.povs[0]) * kDegToRad;   // clockwise from "up"
        dx = std::cos(angle);
        dy = -std::sin(angle);
    }
    const float dz = -applyDeadband(axisOf(stick, kPoseZAxis), deadband);
    const float yaw = applyDeadband(axisOf(stick, kPoseYawLeftAxis), deadband) - applyDeadband(axisOf(stick, kPoseYawRightAxis), deadband);
    if (std::fabs(dx) < 1e-3f && std::fabs(dy) < 1e-3f && dz == 0.f && yaw == 0.f) return -1;

    const std::size_t zone = selected_;
    const sdk::Vec3 position = pose_[zone].position + sdk::Vec3{dx, dy, dz} * (kNudgeSpeedMps * dt);
    apply(zone, position, yaw_[zone] + yaw * kNudgeYawRadPerS * dt);
    return static_cast<int>(zone);
}

}  // namespace wpiemu::app::client
