// Joystick to chassis speeds with a deadband, the zone buttons and the zone pose controls.
#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include "client/input_mapping.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"

using namespace wpiemu;
using namespace wpiemu::app::client;
namespace proto = wpiemu::sdk::proto;

namespace {

proto::JoystickState stick(float lx, float ly, float rx = 0.f) {
    proto::JoystickState s;
    s.present = true;
    s.axes = {lx, ly, 0.f, 0.f, rx, 0.f};
    s.buttonCount = 12;
    s.povs = {-1};
    return s;
}

const DriveLimits kLimits{4.f, 8.f};

sdk::ChassisSpec chassis() {
    sdk::ChassisSpec c;
    c.sizeM = {0.7f, 0.7f, 0.12f};
    c.comHeightM = 0.2f;
    return c;
}

}  // namespace

TEST_CASE("deadband: nothing inside it, then a straight line to full travel") {
    CHECK(applyDeadband(0.f, 0.1f) == 0.f);
    CHECK(applyDeadband(0.05f, 0.1f) == 0.f);
    CHECK(applyDeadband(-0.1f, 0.1f) == 0.f);
    CHECK(applyDeadband(0.1001f, 0.1f) < 0.01f);
    CHECK(applyDeadband(0.55f, 0.1f) == doctest::Approx(0.5f));
    CHECK(applyDeadband(-0.55f, 0.1f) == doctest::Approx(-0.5f));
    CHECK(applyDeadband(1.f, 0.1f) == doctest::Approx(1.f));
    CHECK(applyDeadband(2.f, 0.1f) == doctest::Approx(1.f));
    CHECK(applyDeadband(0.5f, 0.f) == doctest::Approx(0.5f));
    CHECK(applyDeadband(std::nanf(""), 0.1f) == 0.f);
}

TEST_CASE("drive mapping: stick up is forward, stick left is left, right stick left turns counter-clockwise") {
    DriveInputConfig cfg;
    cfg.deadband = 0.f;
    sdk::ChassisSpeeds s = mapDrive(stick(0.f, -1.f), cfg, kLimits);
    CHECK(s.vx == doctest::Approx(4.f));
    CHECK(s.vy == doctest::Approx(0.f));
    CHECK(s.omega == doctest::Approx(0.f));

    s = mapDrive(stick(-1.f, 0.f), cfg, kLimits);
    CHECK(s.vx == doctest::Approx(0.f));
    CHECK(s.vy == doctest::Approx(4.f));

    s = mapDrive(stick(0.f, 0.f, -1.f), cfg, kLimits);
    CHECK(s.omega == doctest::Approx(8.f));
    s = mapDrive(stick(0.f, 0.f, 0.5f), cfg, kLimits);
    CHECK(s.omega == doctest::Approx(-4.f));
}

TEST_CASE("drive mapping: a diagonal is not faster than straight ahead") {
    DriveInputConfig cfg;
    cfg.deadband = 0.f;
    const sdk::ChassisSpeeds s = mapDrive(stick(-1.f, -1.f), cfg, kLimits);
    CHECK(std::hypot(s.vx, s.vy) == doctest::Approx(4.f));
}

TEST_CASE("drive mapping: the deadband zeroes a resting stick and the scale limits the top speed") {
    DriveInputConfig cfg;   // the default deadband
    sdk::ChassisSpeeds s = mapDrive(stick(0.04f, -0.06f, 0.05f), cfg, kLimits);
    CHECK(s.vx == 0.f);
    CHECK(s.vy == 0.f);
    CHECK(s.omega == 0.f);

    cfg.deadband = 0.2f;
    s = mapDrive(stick(0.f, -0.6f), cfg, kLimits);
    CHECK(s.vx == doctest::Approx(4.f * 0.5f));   // (0.6 - 0.2) / 0.8

    cfg.deadband = 0.f;
    cfg.linearScale = 0.5f;
    cfg.angularScale = 0.25f;
    s = mapDrive(stick(0.f, -1.f, -1.f), cfg, kLimits);
    CHECK(s.vx == doctest::Approx(2.f));
    CHECK(s.omega == doctest::Approx(2.f));
}

TEST_CASE("drive mapping: no joystick, short axis lists and junk values drive nothing") {
    const DriveInputConfig cfg;
    CHECK(mapDrive(proto::JoystickState{}, cfg, kLimits).vx == 0.f);
    proto::JoystickState partial;
    partial.present = true;
    partial.axes = {0.f};
    const sdk::ChassisSpeeds s = mapDrive(partial, cfg, kLimits);
    CHECK(s.vx == 0.f);
    CHECK(s.omega == 0.f);
    const sdk::ChassisSpeeds junk = mapDrive(stick(std::nanf(""), std::numeric_limits<float>::infinity()), cfg, kLimits);
    CHECK(std::isfinite(junk.vx));
    CHECK(std::isfinite(junk.vy));
}

TEST_CASE("drive mapping: stored settings are brought into range") {
    DriveInputConfig wild;
    wild.deadband = 5.f;
    wild.linearScale = 0.f;
    wild.angularScale = std::nanf("");
    const DriveInputConfig n = normalizedDriveConfig(wild);
    CHECK(n.deadband == doctest::Approx(0.9f));
    CHECK(n.linearScale == doctest::Approx(0.05f));
    CHECK(n.angularScale == doctest::Approx(1.f));
}

TEST_CASE("drive limits: the slowest wheel sets the top speed and the farthest wheel the top spin") {
    sdk::RobotConfig spec;
    sdk::WheelSpec w;
    w.radiusM = 0.05f;
    w.positionM = {0.3f, 0.4f, 0.f};
    spec.drivetrain.spec.kind = sdk::drivetrain_kind::kSwerve;
    spec.drivetrain.spec.wheels = {w, w};
    const DriveLimits limits = driveLimitsOf(spec);
    CHECK(limits.maxLinearMps == doctest::Approx(sdk::maxLinearSpeedMps(spec.drivetrain.spec)));
    CHECK(limits.maxAngularRadPerS == doctest::Approx(limits.maxLinearMps / 0.5f));
}

TEST_CASE("zone buttons: button i holds zone i, only for zones the robot has") {
    proto::JoystickState s = stick(0, 0);
    s.buttons = 0b101u;   // buttons 1 and 3 (A and X)
    CHECK(zoneFlagsOf(s, 3) == 0b101u);
    CHECK(zoneFlagsOf(s, 2) == 0b001u);   // no third zone
    CHECK(zoneFlagsOf(s, 0) == 0u);
    s.buttons = 0xFFFFu;
    CHECK(zoneFlagsOf(s, 20) == 0xFFFu);   // the profile has 12 buttons
    s.present = false;
    CHECK(zoneFlagsOf(s, 3) == 0u);
}

TEST_CASE("zone poses: the d-pad moves the selected zone inside the envelope and a reset forgets it") {
    ZonePoseControls zones;
    sdk::Pose3 rest;
    rest.position = {0.4f, 0.f, 0.2f};
    zones.configure(chassis(), {rest, rest});
    REQUIRE(zones.zoneCount() == 2);
    CHECK_FALSE(zones.moved(0));
    CHECK(zones.nudge(0.1, stick(0, 0), 0.1f) == -1);   // nothing pressed

    zones.select(1);
    proto::JoystickState pad = stick(0, 0);
    pad.povs = {0};   // up: forward
    CHECK(zones.nudge(0.1, pad, 0.1f) == 1);
    CHECK(zones.moved(1));
    CHECK_FALSE(zones.moved(0));
    CHECK(zones.pose(1).position.x == doctest::Approx(0.4f + ZonePoseControls::kNudgeSpeedMps * 0.1f));
    CHECK(std::fabs(zones.pose(1).position.y) < 1e-4f);

    pad.povs = {90};   // right: toward the robot's right, negative y
    for (int i = 0; i < 5; ++i) zones.nudge(0.1, pad, 0.1f);
    CHECK(zones.pose(1).position.y < -0.1f);

    // Held far beyond the reach limit: clamped to the envelope, never farther.
    pad.povs = {0};
    for (int i = 0; i < 200; ++i) zones.nudge(0.1, pad, 0.1f);
    CHECK(zones.pose(1).position.x == doctest::Approx(zones.maxPosition().x));

    CHECK(zones.reset(1));
    CHECK_FALSE(zones.moved(1));
    CHECK(zones.pose(1).position.x == doctest::Approx(0.4f));
    CHECK_FALSE(zones.reset(1));
}

TEST_CASE("zone poses: the right stick raises, the triggers turn, the sliders set exactly") {
    ZonePoseControls zones;
    zones.configure(chassis(), {sdk::Pose3{}});
    proto::JoystickState s = stick(0, 0);
    s.axes[kPoseZAxis] = -1.f;   // up
    CHECK(zones.nudge(0.1, s, 0.1f) == 0);
    CHECK(zones.pose(0).position.z > 0.f);

    s = stick(0, 0);
    s.axes[kPoseYawLeftAxis] = 1.f;   // counter-clockwise
    zones.nudge(0.1, s, 0.1f);
    CHECK(zones.yawOffsetRad(0) > 0.f);

    CHECK(zones.set(0, {0.25f, -0.1f, 0.3f}, 0.5f));
    CHECK(zones.pose(0).position.x == doctest::Approx(0.25f));
    CHECK(zones.pose(0).position.y == doctest::Approx(-0.1f));
    CHECK(zones.yawOffsetRad(0) == doctest::Approx(0.5f));
    CHECK(zones.pose(0).rotation.yaw() == doctest::Approx(0.5f));

    CHECK_FALSE(zones.set(5, {0, 0, 0}, 0.f));                       // no such zone
    CHECK_FALSE(zones.set(0, {std::nanf(""), 0, 0}, 0.f));           // junk is refused
    CHECK(zones.set(0, {9.f, 9.f, 9.f}, 0.f));                       // out of reach: clamped
    CHECK(zones.pose(0).position.x == doctest::Approx(zones.maxPosition().x));
    CHECK(zones.pose(0).position.z == doctest::Approx(zones.maxPosition().z));
}

TEST_CASE("zone poses: a new robot forgets the moved zones and the selection") {
    ZonePoseControls zones;
    zones.configure(chassis(), {sdk::Pose3{}, sdk::Pose3{}});
    zones.select(1);
    zones.set(1, {0.1f, 0.f, 0.f}, 0.f);
    zones.configure(chassis(), {sdk::Pose3{}});
    CHECK(zones.zoneCount() == 1);
    CHECK(zones.selected() == 0);
    CHECK_FALSE(zones.moved(0));
    zones.select(7);   // out of range is ignored
    CHECK(zones.selected() == 0);
}
