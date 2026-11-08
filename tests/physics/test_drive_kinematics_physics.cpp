#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "physics_test_support.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"

// The conventions of sdk/physics/drive_kinematics.h must match the traction drivetrains of core/physics: a robot driven
// open loop with the wheel setpoints of a chassis command moves the way the kinematics predict. A sign or handedness
// mistake in either place shows up here as a robot driving or turning the wrong way.

using namespace physics_test;

namespace {

struct Outcome {
    Vec3 bodyVelocity;   // chassis frame
    float yawRate = 0.f;
    std::vector<WheelSetpoint> setpoints;
};

// Open-loop voltage that holds the wheel at `omega`, limited to the motor's nominal voltage.
double voltsFor(const WheelSpec& w, double omega) {
    const double limit = w.driveMotor.nominalVoltageV;
    return std::clamp(steadyVoltage(w, omega), -limit, limit);
}

std::string describe(const char* label, ChassisSpeeds cmd, const Outcome& r) {
    return std::string(label) + ": commanded (vx " + std::to_string(cmd.vx) + ", vy " + std::to_string(cmd.vy) + ", omega " +
           std::to_string(cmd.omega) + "), simulated (vx " + std::to_string(r.bodyVelocity.x) + ", vy " +
           std::to_string(r.bodyVelocity.y) + ", omega " + std::to_string(r.yawRate) + ")";
}

// Settles the rig, points the modules (swerve only), then drives the setpoints for 1.5 s of simulated time.
Outcome drive(const DrivetrainSpec& spec, ChassisSpeeds cmd) {
    Rig rig(spec);
    REQUIRE(rig.sim);
    auto created = DriveKinematics::create(spec);
    REQUIRE(created.ok());
    DriveKinematics kin = std::move(created).value();
    auto sp = kin.update(cmd);
    REQUIRE(sp.ok());

    const bool steered = spec.kind == drivetrain_kind::kSwerve;
    std::vector<double> targets;
    for (const WheelSetpoint& s : sp.value()) targets.push_back(s.steerAngleRad);

    rig.run(0.3f);
    if (steered)
        for (int i = 0; i < 120; ++i) {
            steerToward(rig, targets);
            rig.step();
        }
    for (std::size_t i = 0; i < rig.cmd.size(); ++i)
        rig.cmd[i].driveVoltageV = voltsFor(spec.wheels[i], sp.value()[i].wheelSpeedRadPerSec);
    for (int i = 0; i < 300; ++i) {
        if (steered) steerToward(rig, targets);
        rig.step();
    }
    return {rig.bodyVelocity(), rig.yawRate(), sp.value()};
}

// The robot translates along the commanded direction at about the commanded speed and does not turn.
void expectTranslation(const char* label, const DrivetrainSpec& spec, ChassisSpeeds cmd) {
    const Outcome r = drive(spec, cmd);
    const float speed = std::hypot(cmd.vx, cmd.vy);
    const float along = (r.bodyVelocity.x * cmd.vx + r.bodyVelocity.y * cmd.vy) / speed;
    const float across = (-r.bodyVelocity.x * cmd.vy + r.bodyVelocity.y * cmd.vx) / speed;
    INFO(describe(label, cmd, r));
    CHECK_MESSAGE(along > 0.75f * speed, label << ": the robot does not move in the direction the kinematics predicted");
    CHECK_MESSAGE(along < 1.2f * speed, label << ": the robot moves much faster than the kinematics predicted");
    CHECK_MESSAGE(std::fabs(across) < 0.2f * speed, label << ": the robot drifts sideways of the predicted direction");
    CHECK_MESSAGE(std::fabs(r.yawRate) < 0.25f, label << ": the robot turns although no rotation was commanded");
}

// The robot turns the commanded way at about the commanded rate and stays near its place.
void expectSpin(const char* label, const DrivetrainSpec& spec, float omega) {
    const ChassisSpeeds cmd{0.f, 0.f, omega};
    const Outcome r = drive(spec, cmd);
    INFO(describe(label, cmd, r));
    CHECK_MESSAGE(r.yawRate * omega > 0.f, label << ": the robot turns the wrong way (rotation sign mismatch)");
    CHECK_MESSAGE(std::fabs(r.yawRate) > 0.7f * std::fabs(omega), label << ": the robot turns much slower than predicted");
    CHECK_MESSAGE(std::fabs(r.yawRate) < 1.2f * std::fabs(omega), label << ": the robot turns much faster than predicted");
    CHECK_MESSAGE(length(r.bodyVelocity) < 0.3f, label << ": the robot translates although only rotation was commanded");
}

DrivetrainSpec explicitAngleMecanum() {
    DrivetrainSpec d = mecanumSpec();
    // The X pattern at +-0.6 rad (34 degrees) given on the tires instead of the 45 degrees derived from the positions.
    for (WheelSpec& w : d.wheels) w.tire.tractionAngleRad = -0.6f * (w.positionM.x * w.positionM.y > 0.f ? 1.f : -1.f);
    return d;
}

}  // namespace

TEST_CASE("kinematics vs simulator: swerve modules drive forward, strafe left, spin and flip as predicted") {
    const DrivetrainSpec spec = swerveSpec();
    expectTranslation("swerve forward", spec, {1.5f, 0.f, 0.f});
    expectTranslation("swerve strafe left (modules at 90 degrees)", spec, {0.f, 1.5f, 0.f});
    expectTranslation("swerve backward (modules flipped or reversed)", spec, {-1.5f, 0.f, 0.f});
    expectTranslation("swerve diagonal", spec, {1.f, 1.f, 0.f});
    expectTranslation("swerve flipped module (backward and left, heading beyond 90 degrees)", spec, {-1.2f, 0.5f, 0.f});
    expectSpin("swerve spin counter-clockwise", spec, 3.f);
    expectSpin("swerve spin clockwise", spec, -3.f);

    const ChassisSpeeds combined{1.f, 0.8f, 1.f};
    const Outcome r = drive(spec, combined);
    INFO(describe("swerve combined", combined, r));
    CHECK(r.bodyVelocity.x == doctest::Approx(combined.vx).epsilon(0.25));
    CHECK(r.bodyVelocity.y == doctest::Approx(combined.vy).epsilon(0.25));
    CHECK(r.yawRate == doctest::Approx(combined.omega).epsilon(0.25));
}

TEST_CASE("kinematics vs simulator: tank sides drive forward and turn the predicted way") {
    expectTranslation("tank forward", tankSpec(2, WheelKind::Standard), {1.5f, 0.f, 0.f});
    expectTranslation("tank six wheels backward", tankSpec(3, WheelKind::Standard), {-1.5f, 0.f, 0.f});
    // Omni wheels turn nearly kinematically, so the rotation is comparable with the prediction.
    expectSpin("tank spin counter-clockwise", tankSpec(2, WheelKind::Omni), 2.f);
    expectSpin("tank spin clockwise", tankSpec(2, WheelKind::Omni), -2.f);
}

TEST_CASE("kinematics vs simulator: mecanum X layout drives forward, strafes left and spins as predicted") {
    const DrivetrainSpec spec = mecanumSpec();
    expectTranslation("mecanum forward", spec, {1.5f, 0.f, 0.f});
    expectTranslation("mecanum strafe left", spec, {0.f, 1.5f, 0.f});
    expectTranslation("mecanum strafe right", spec, {0.f, -1.5f, 0.f});
    expectTranslation("mecanum diagonal", spec, {1.f, 1.f, 0.f});
    expectSpin("mecanum spin counter-clockwise", spec, 3.f);
    expectSpin("mecanum spin clockwise", spec, -3.f);
}

TEST_CASE("kinematics vs simulator: mecanum wheels with explicit roller angles follow their own angles") {
    const DrivetrainSpec spec = explicitAngleMecanum();
    expectTranslation("explicit-angle mecanum forward", spec, {1.5f, 0.f, 0.f});
    expectTranslation("explicit-angle mecanum strafe left", spec, {0.f, 1.5f, 0.f});
    expectSpin("explicit-angle mecanum spin counter-clockwise", spec, 3.f);
}
