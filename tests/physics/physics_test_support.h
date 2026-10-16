#pragma once

// Shared helpers for the physics tests: floor/ramp builders, canonical FRC-like chassis and drivetrain specs
// (ADR 0003 validation setup), and a small rig that owns a Box3D world + ChassisSimulator.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace sdk = wpiemu::sdk;

namespace physics_test {

using namespace wpiemu::sdk;
namespace phys = wpiemu::core::physics;

constexpr float kDt = 0.005f;
constexpr float kPiF = 3.14159265358979323846f;

inline std::unique_ptr<IPhysicsWorld> makeWorld(PhysicsConfig cfg = {}) {
    auto w = phys::createBox3dWorld(cfg);
    return w;
}

inline BodyId addStaticBox(IPhysicsWorld& w, Vec3 center, Vec3 halfExtents, Quat rot = Quat::identity(),
                           std::uint64_t tag = 0, float friction = 0.6f) {
    BodyDesc bd;
    bd.type = BodyType::Static;
    bd.pose = Pose3{center, rot};
    const BodyId b = w.createBody(bd);
    ShapeDesc sd;
    sd.geometry = BoxShape{halfExtents};
    sd.material.friction = friction;
    sd.category = layer::Field;
    sd.mask = layer::All;
    sd.userTag = tag;
    w.addShape(b, sd);
    return b;
}

// Large floor whose top surface is z = 0.
inline BodyId addFloor(IPhysicsWorld& w) { return addStaticBox(w, {0.f, 0.f, -1.f}, {40.f, 40.f, 1.f}); }

// Inclined plane rising along +X, passing through the floor at x = startX.
inline BodyId addRamp(IPhysicsWorld& w, float startX, float angleRad, float length = 8.f) {
    const Quat q = Quat::fromAxisAngle({0.f, 1.f, 0.f}, -angleRad);
    const Vec3 n = q.rotate({0.f, 0.f, 1.f});
    const float halfThickness = 0.2f;
    const Vec3 surfaceStart{startX, 0.f, 0.f};
    const Vec3 center = surfaceStart - n * halfThickness + q.rotate({0.5f * length, 0.f, 0.f});
    return addStaticBox(w, center, {0.5f * length, 3.f, halfThickness}, q);
}

inline BodyId addBall(IPhysicsWorld& w, Vec3 at, float radius = 0.075f, float density = 113.2f,
                      std::uint64_t category = layer::Fuel, std::uint64_t mask = layer::All, std::uint64_t tag = 0,
                      bool sleep = true) {
    BodyDesc bd;
    bd.type = BodyType::Dynamic;
    bd.pose.position = at;
    bd.allowSleep = sleep;
    const BodyId b = w.createBody(bd);
    ShapeDesc sd;
    sd.geometry = SphereShape{radius};
    sd.density = density;
    sd.material = {0.6f, 0.5f, 0.02f};
    sd.category = category;
    sd.mask = mask;
    sd.sensorEvents = true;
    sd.contactEvents = true;
    sd.userTag = tag;
    w.addShape(b, sd);
    return b;
}

// ---- canonical chassis / drivetrains (ADR 0003 validation setup) ----------------------------------------

inline ChassisSpec testChassis() {
    ChassisSpec c;
    c.sizeM = {0.7f, 0.7f, 0.12f};
    c.massKg = 50.f;
    c.comHeightM = 0.20f;
    c.groundClearanceM = 0.03f;
    return c;
}

inline WheelSpec testWheel(const char* name, float x, float y, WheelKind kind, bool steered, float currentLimitA) {
    WheelSpec w;
    w.name = name;
    w.kind = kind;
    w.positionM = {x, y, 0.f};  // z = 0 => hub at one radius
    w.steered = steered;
    w.driveCurrentLimitA = currentLimitA;
    w.tire.mu = 1.1f;
    return w;
}

// Square layout, wheels at (+-half, +-half): fl, fr, rl, rr.
inline DrivetrainSpec squareDrivetrain(const char* kind, WheelKind wheelKind, bool steered, float currentLimitA = 0.f,
                                       float half = 0.25f) {
    DrivetrainSpec d;
    d.kind = kind;
    d.wheels = {testWheel("fl", half, half, wheelKind, steered, currentLimitA),
                testWheel("fr", half, -half, wheelKind, steered, currentLimitA),
                testWheel("rl", -half, half, wheelKind, steered, currentLimitA),
                testWheel("rr", -half, -half, wheelKind, steered, currentLimitA)};
    return d;
}

inline DrivetrainSpec swerveSpec(float currentLimitA = 0.f) {
    return squareDrivetrain(drivetrain_kind::kSwerve, WheelKind::Swerve, true, currentLimitA);
}
inline DrivetrainSpec withMu(DrivetrainSpec d, float mu) {
    for (WheelSpec& w : d.wheels) w.tire.mu = mu;
    return d;
}
inline DrivetrainSpec mecanumSpec(float currentLimitA = 0.f) {
    DrivetrainSpec d = squareDrivetrain(drivetrain_kind::kMecanum, WheelKind::Mecanum, false, currentLimitA);
    return d;
}
// N wheels per side, spaced along the chassis; rows are [left0..leftN-1, right0..rightN-1].
inline DrivetrainSpec tankSpec(int perSide, WheelKind kind, float lateralScale = 1.f, float track = 0.5f,
                               float currentLimitA = 0.f) {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kTank;
    static const char* const left[] = {"left0", "left1", "left2", "left3"};
    static const char* const right[] = {"right0", "right1", "right2", "right3"};
    for (int side = 0; side < 2; ++side) {
        for (int i = 0; i < perSide; ++i) {
            const float x = perSide == 1 ? 0.f : -0.25f + 0.5f * static_cast<float>(i) / static_cast<float>(perSide - 1);
            WheelSpec w = testWheel(side == 0 ? left[i] : right[i], x, side == 0 ? 0.5f * track : -0.5f * track, kind,
                                    false, currentLimitA);
            w.tire.lateralScale = lateralScale;
            d.wheels.push_back(w);
        }
    }
    return d;
}

// Volts that make an ideal motor turn the wheel at `wheelOmega` (no load).
inline double steadyVoltage(const WheelSpec& w, double wheelOmega) {
    return w.driveGearRatio * wheelOmega / w.driveMotor.kvRadPerSecPerVolt();
}

// ---- rig -----------------------------------------------------------------------------------------------------

struct Rig {
    std::unique_ptr<IPhysicsWorld> world;
    DrivetrainRegistry drivetrains;
    TireModelRegistry tireModels;
    DrivetrainSpec drivetrainSpec;
    ChassisSpec chassisSpec;
    std::unique_ptr<phys::ChassisSimulator> sim;
    std::vector<WheelCommand> cmd;

    explicit Rig(DrivetrainSpec ds, ChassisSpec cs = testChassis(), PhysicsConfig cfg = {},
                 phys::ChassisPlacement placement = {}, bool floor = true)
        : world(makeWorld(cfg)), drivetrainSpec(std::move(ds)), chassisSpec(std::move(cs)) {
        (void)phys::registerPhysicsComponents(drivetrains, tireModels);
        if (floor) addFloor(*world);
        phys::ChassisSimulatorArgs args{*world, chassisSpec, drivetrainSpec, drivetrains, tireModels, placement, {}, cfg.gravity};
        auto r = phys::ChassisSimulator::create(args);
        if (r.ok()) sim = std::move(r).value();
        cmd.assign(drivetrainSpec.wheels.size(), WheelCommand{});
    }

    void step() { sim->step(cmd, kDt); }
    void run(float seconds) {
        const int n = static_cast<int>(seconds / kDt + 0.5f);
        for (int i = 0; i < n; ++i) step();
    }
    void setDrive(double volts) {
        for (WheelCommand& c : cmd) c.driveVoltageV = volts;
    }

    // Velocity of the chassis COM in the chassis frame.
    Vec3 bodyVelocity() const {
        const Pose3 p = world->pose(sim->body());
        return p.rotation.conjugate().rotate(world->linearVelocity(sim->body()));
    }
    float yawRate() const { return world->angularVelocity(sim->body()).z; }
    Vec3 position() const { return world->pose(sim->body()).position; }
};

// PD azimuth control for swerve modules (tests only): sets cmd[i].steerVoltageV towards targets[i].
inline void steerToward(Rig& rig, std::span<const double> targets, double kp = 12.0, double kd = 0.8) {
    const auto states = rig.sim->drivetrain().wheelStates();
    for (std::size_t i = 0; i < rig.cmd.size(); ++i) {
        const double v = kp * (targets[i] - states[i].steerAngleRad) - kd * states[i].steerOmegaRadPerSec;
        rig.cmd[i].steerVoltageV = std::max(-12.0, std::min(12.0, v));
    }
}

}  // namespace physics_test
