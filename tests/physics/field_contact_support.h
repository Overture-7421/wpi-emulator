#pragma once

// Shared by the wall-contact tests: the wizard's default bumpered swerve robot and a field-oriented
// driver that commands the robot at an angle into a wall
// (docs/reference/physics.md#contact-materials).

#include <algorithm>
#include <cmath>

#include "physics_test_support.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/physics/wheel_layout.h"

namespace physics_test {

// 27 in square frame with four 3.25 in bumper boxes, like the setup wizard builds.
inline ChassisSpec bumperedChassis() {
    ChassisSpec c;
    const float len = 0.6858f, wid = 0.6858f, t = 0.0826f, h = 0.1397f;
    c.sizeM = {len, wid, 0.10f};
    c.massKg = 54.f;
    c.comHeightM = 0.20f;
    c.groundClearanceM = 0.04f;
    const float z = 0.1207f - c.comHeightM;
    auto bumper = [&](const char* name, Vec3 centre, Vec3 size) {
        PrimitiveSpec p;
        p.name = name;
        p.local = Pose3(centre, Quat{});
        p.sizeM = size;
        p.collides = true;
        c.primitives.push_back(p);
    };
    bumper("front", {len / 2.f + t / 2.f, 0.f, z}, {t, wid + 2.f * t, h});
    bumper("back", {-(len / 2.f + t / 2.f), 0.f, z}, {t, wid + 2.f * t, h});
    bumper("left", {0.f, wid / 2.f + t / 2.f, z}, {len, t, h});
    bumper("right", {0.f, -(wid / 2.f + t / 2.f), z}, {len, t, h});
    return c;
}

inline DrivetrainSpec bumperedSwerve() {
    WheelSpec base;
    base.radiusM = static_cast<float>(kDefaultWheelRadiusM);
    base.driveMotor = motors::krakenX60();
    base.steerMotor = motors::neo550();
    base.driveGearRatio = 6.75f;
    base.steerGearRatio = 12.8f;
    DrivetrainSpec spec;
    spec.kind = drivetrain_kind::kSwerve;
    const float half = 0.6858f / 2.f - 0.0635f;
    spec.wheels = swerveWheels(base, half, half);
    return spec;
}

// Field-oriented swerve command: chassis velocity (vx, vy) in the world frame while holding
// `yawTarget`.
inline void driveFieldOriented(Rig& rig, double vx, double vy, double yawTarget) {
    const double yaw = rig.world->pose(rig.sim->body()).rotation.yaw();
    double err = yawTarget - yaw;
    while (err > kPi) err -= 2.0 * kPi;
    while (err < -kPi) err += 2.0 * kPi;
    const double omega = std::clamp(6.0 * err - 0.5 * rig.yawRate(), -4.0, 4.0);
    const double c = std::cos(-yaw), s = std::sin(-yaw);
    const double vxr = c * vx - s * vy, vyr = s * vx + c * vy;

    const auto states = rig.sim->drivetrain().wheelStates();
    for (std::size_t i = 0; i < rig.cmd.size(); ++i) {
        const WheelSpec& w = rig.drivetrainSpec.wheels[i];
        const double vix = vxr - omega * w.positionM.y;
        const double viy = vyr + omega * w.positionM.x;
        const double speed = std::hypot(vix, viy);
        const double current = states[i].steerAngleRad;
        double target = speed > 1e-3 ? std::atan2(viy, vix) : current;
        target += 2.0 * kPi * std::round((current - target) / (2.0 * kPi));
        const double steerErr = target - current;
        rig.cmd[i].steerVoltageV =
            std::clamp(12.0 * steerErr - 0.8 * states[i].steerOmegaRadPerSec, -12.0, 12.0);
        rig.cmd[i].driveVoltageV =
            steadyVoltage(w, speed / w.radiusM) * std::max(0.0, std::cos(steerErr));
    }
}

struct WallRun {
    double alongMps = 0.0;       // mean speed along the wall over the last half second
    double maxReboundMps = 0.0;  // fastest movement away from the wall
    double maxTiltDeg = 0.0;
};

// Commands `speedMps` in a direction `angleDeg` away from the wall normal (0 = head-on) with the
// heading held. `intoWall` and `alongWall` are horizontal unit vectors.
inline WallRun driveIntoWall(Rig& rig, const Vec3& intoWall, const Vec3& alongWall, double angleDeg,
                             double speedMps, double seconds, double yawTarget) {
    const double a = angleDeg * kPi / 180.0;
    const double vx = speedMps * (std::cos(a) * intoWall.x + std::sin(a) * alongWall.x);
    const double vy = speedMps * (std::cos(a) * intoWall.y + std::sin(a) * alongWall.y);
    const int steps = static_cast<int>(seconds / kDt + 0.5);
    const int tail = static_cast<int>(0.5 / kDt);
    WallRun out;
    for (int i = 0; i < steps; ++i) {
        driveFieldOriented(rig, vx, vy, yawTarget);
        rig.step();
        const Vec3 v = rig.world->linearVelocity(rig.sim->body());
        out.maxReboundMps = std::max(out.maxReboundMps, -static_cast<double>(dot(v, intoWall)));
        const Vec3 up = rig.world->pose(rig.sim->body()).rotation.rotate({0.f, 0.f, 1.f});
        out.maxTiltDeg =
            std::max(out.maxTiltDeg, std::acos(std::clamp(up.z, -1.f, 1.f)) * 180.0 / kPi);
        if (i >= steps - tail) out.alongMps += dot(v, alongWall) / tail;
    }
    return out;
}

}  // namespace physics_test
