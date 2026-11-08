#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "wpiemu/sdk/physics/drive_kinematics.h"
#include "wpiemu/sdk/physics/wheel_layout.h"

using namespace wpiemu::sdk;

namespace {

constexpr float kTol = 1e-4f;

DrivetrainSpec swerve(float halfL = 0.3f, float halfW = 0.25f) {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kSwerve;
    d.wheels = swerveWheels(WheelSpec{}, halfL, halfW);
    return d;
}

DrivetrainSpec mecanum(float halfL = 0.3f, float halfW = 0.25f) {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kMecanum;
    d.wheels = mecanumWheels(WheelSpec{}, halfL, halfW);
    for (WheelSpec& w : d.wheels) w.tire.tractionAngleRad = 0.f;   // the X layout comes from the wheel positions
    return d;
}

DrivetrainSpec tank(int perSide, float halfWidth = 0.3f) {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kTank;
    d.wheels = tankWheels(WheelSpec{}, perSide, 0.3f, halfWidth);
    return d;
}

// Deterministic values in [-1, 1].
class Lcg {
public:
    explicit Lcg(std::uint32_t seed) : state_(seed) {}
    float next() {
        state_ = state_ * 1664525u + 1013904223u;
        return static_cast<float>(state_ >> 8) / static_cast<float>(1u << 23) - 1.f;
    }

private:
    std::uint32_t state_;
};

std::vector<WheelSetpoint> raw(const DrivetrainSpec& d, ChassisSpeeds s, std::span<const float> prev = {}) {
    auto r = wheelSetpointsRaw(d, s, prev);
    REQUIRE(r.ok());
    return std::move(r).value();
}

void checkRoundTrip(const DrivetrainSpec& d, ChassisSpeeds s, const char* what) {
    INFO(what);
    const auto sp = raw(d, s);
    auto back = chassisSpeedsFromWheels(d, sp);
    REQUIRE(back.ok());
    CHECK(back.value().vx == doctest::Approx(s.vx).epsilon(1e-3).scale(1.0));
    CHECK(back.value().vy == doctest::Approx(s.vy).epsilon(1e-3).scale(1.0));
    CHECK(back.value().omega == doctest::Approx(s.omega).epsilon(1e-3).scale(1.0));
}

}  // namespace

TEST_CASE("swerve kinematics: straight, strafe and spin use the module headings of the simulator") {
    const DrivetrainSpec d = swerve();
    const float r = d.wheels[0].radiusM;

    for (const WheelSetpoint& sp : raw(d, {2.f, 0.f, 0.f})) {
        CHECK(sp.wheelSpeedRadPerSec == doctest::Approx(2.f / r).epsilon(kTol));
        CHECK(sp.steerAngleRad == doctest::Approx(0.f).scale(1.0).epsilon(kTol));
    }
    for (const WheelSetpoint& sp : raw(d, {0.f, 1.5f, 0.f})) {
        CHECK(sp.wheelSpeedRadPerSec == doctest::Approx(1.5f / r).epsilon(kTol));
        CHECK(sp.steerAngleRad == doctest::Approx(kPiF / 2).epsilon(kTol));
    }

    const auto spin = raw(d, {0.f, 0.f, 2.f});
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec3 p = d.wheels[i].positionM;
        INFO("wheel " << d.wheels[i].name);
        CHECK(spin[i].steerAngleRad == doctest::Approx(std::atan2(p.x, -p.y)).epsilon(kTol));
        CHECK(spin[i].wheelSpeedRadPerSec == doctest::Approx(2.f * std::hypot(p.x, p.y) / r).epsilon(kTol));
    }
    // Counter-clockwise: the front-left module's velocity (-omega*y, omega*x) points back and to the left.
    CHECK(spin[0].steerAngleRad > kPiF / 2);
}

TEST_CASE("swerve kinematics: a combined motion sets each module to its own point velocity") {
    const DrivetrainSpec d = swerve();
    const ChassisSpeeds s{1.f, -0.5f, 1.2f};
    const auto sp = raw(d, s);
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec3 p = d.wheels[i].positionM;
        const float vx = s.vx - s.omega * p.y, vy = s.vy + s.omega * p.x;
        const float speed = sp[i].wheelSpeedRadPerSec * d.wheels[i].radiusM;
        CHECK(speed * std::cos(sp[i].steerAngleRad) == doctest::Approx(vx).epsilon(kTol));
        CHECK(speed * std::sin(sp[i].steerAngleRad) == doctest::Approx(vy).epsilon(kTol));
    }
}

TEST_CASE("tank kinematics: sides turn at vx minus omega times their lateral offset") {
    const DrivetrainSpec d = tank(2);
    const float r = d.wheels[0].radiusM;
    for (const WheelSetpoint& sp : raw(d, {2.f, 0.f, 0.f})) {
        CHECK(sp.wheelSpeedRadPerSec == doctest::Approx(2.f / r).epsilon(kTol));
        CHECK(sp.steerAngleRad == 0.f);
    }
    // Counter-clockwise spin: the left side (y > 0) runs backward, the right side forward.
    const auto spin = raw(d, {0.f, 0.f, 2.f});
    for (std::size_t i = 0; i < d.wheels.size(); ++i) {
        const float expected = -2.f * d.wheels[i].positionM.y / r;
        INFO("wheel " << d.wheels[i].name);
        CHECK(spin[i].wheelSpeedRadPerSec == doctest::Approx(expected).epsilon(kTol));
    }
    // A tank cannot strafe: vy is ignored.
    const auto strafe = raw(d, {1.f, 5.f, 0.f});
    for (const WheelSetpoint& sp : strafe) CHECK(sp.wheelSpeedRadPerSec == doctest::Approx(1.f / r).epsilon(kTol));
}

TEST_CASE("tank kinematics: six wheels share one speed per side") {
    const DrivetrainSpec d = tank(3);
    REQUIRE(d.wheels.size() == 6);
    const float r = d.wheels[0].radiusM;
    const auto sp = raw(d, {1.f, 0.f, 1.5f});
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(sp[i].wheelSpeedRadPerSec == doctest::Approx((1.f - 1.5f * 0.3f) / r).epsilon(kTol));
        CHECK(sp[i + 3].wheelSpeedRadPerSec == doctest::Approx((1.f + 1.5f * 0.3f) / r).epsilon(kTol));
    }
    checkRoundTrip(d, {1.f, 0.f, 1.5f}, "six wheel tank");
}

TEST_CASE("tank kinematics: uneven wheels on a side still share that side's speed") {
    DrivetrainSpec d = tank(2);
    d.wheels[0].positionM.y = 0.35f;   // left wheels at 0.35 and 0.3
    const auto sp = raw(d, {1.f, 0.f, 1.f});
    CHECK(sp[0].wheelSpeedRadPerSec == doctest::Approx(sp[1].wheelSpeedRadPerSec).epsilon(kTol));
    checkRoundTrip(d, {1.f, 0.f, 1.f}, "uneven tank");
}

TEST_CASE("mecanum kinematics: the X layout of the simulator matches WPILib") {
    const DrivetrainSpec d = mecanum();
    const float r = d.wheels[0].radiusM;
    // wheels: fl, fr, bl, br
    for (const WheelSetpoint& sp : raw(d, {1.f, 0.f, 0.f})) CHECK(sp.wheelSpeedRadPerSec == doctest::Approx(1.f / r).epsilon(kTol));

    // Strafe left: FL and BR roll backward, FR and BL forward.
    const auto strafe = raw(d, {0.f, 1.f, 0.f});
    CHECK(strafe[0].wheelSpeedRadPerSec == doctest::Approx(-1.f / r).epsilon(kTol));
    CHECK(strafe[1].wheelSpeedRadPerSec == doctest::Approx(1.f / r).epsilon(kTol));
    CHECK(strafe[2].wheelSpeedRadPerSec == doctest::Approx(1.f / r).epsilon(kTol));
    CHECK(strafe[3].wheelSpeedRadPerSec == doctest::Approx(-1.f / r).epsilon(kTol));

    // Counter-clockwise spin: left wheels backward, right wheels forward, scaled by (halfL + halfW).
    const auto spin = raw(d, {0.f, 0.f, 1.f});
    const float k = (0.3f + 0.25f) / r;
    CHECK(spin[0].wheelSpeedRadPerSec == doctest::Approx(-k).epsilon(kTol));
    CHECK(spin[1].wheelSpeedRadPerSec == doctest::Approx(k).epsilon(kTol));
    CHECK(spin[2].wheelSpeedRadPerSec == doctest::Approx(-k).epsilon(kTol));
    CHECK(spin[3].wheelSpeedRadPerSec == doctest::Approx(k).epsilon(kTol));
}

TEST_CASE("mecanum kinematics: an explicit tire traction angle overrides the position rule") {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kMecanum;
    d.wheels = mecanumWheels(WheelSpec{}, 0.3f, 0.25f);
    // The same X pattern as the rule gives, at +-0.6 rad (about 34 degrees) instead of 45 degrees.
    for (WheelSpec& w : d.wheels) w.tire.tractionAngleRad = -0.6f * (w.positionM.x * w.positionM.y > 0.f ? 1.f : -1.f);
    const float r = d.wheels[0].radiusM;
    const auto forward = raw(d, {1.f, 0.f, 0.f});
    CHECK(forward[0].wheelSpeedRadPerSec == doctest::Approx(1.f / r).epsilon(kTol));
    // Strafe left: ground speed along the axis of "fl" (at -0.6 rad) is -sin(0.6), divided by r * cos(0.6).
    const auto strafe = raw(d, {0.f, 1.f, 0.f});
    CHECK(strafe[0].wheelSpeedRadPerSec == doctest::Approx(-std::tan(0.6f) / r).epsilon(kTol));
    CHECK(strafe[1].wheelSpeedRadPerSec == doctest::Approx(std::tan(0.6f) / r).epsilon(kTol));
    checkRoundTrip(d, {0.7f, -0.4f, 0.9f}, "explicit mecanum angles");
}

TEST_CASE("mecanum kinematics: a layout whose roller axes all point at the centre is refused") {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kMecanum;
    d.wheels = mecanumWheels(WheelSpec{}, 0.25f, 0.25f);
    // "+45 deg on fl" (the opposite of the X layout): on square geometry no wheel has a torque arm, so the robot cannot spin.
    for (WheelSpec& w : d.wheels) w.tire.tractionAngleRad = 0.78539816339f * (w.positionM.x * w.positionM.y > 0.f ? 1.f : -1.f);
    auto r = wheelSetpointsRaw(d, {0.f, 0.f, 1.f});
    CHECK_FALSE(r.ok());
    CHECK_FALSE(DriveKinematics::create(d).ok());
}

TEST_CASE("mecanum kinematics: the wizard layout is the X layout and can spin on square geometry") {
    DrivetrainSpec d;
    d.kind = drivetrain_kind::kMecanum;
    d.wheels = mecanumWheels(WheelSpec{}, 0.25f, 0.25f);
    CHECK(d.wheels[0].tire.tractionAngleRad < 0.f);   // "fl"
    CHECK(d.wheels[1].tire.tractionAngleRad > 0.f);   // "fr"
    const auto spin = wheelSetpointsRaw(d, {0.f, 0.f, 1.f});
    REQUIRE(spin.ok());
    CHECK(DriveKinematics::create(d).ok());
    checkRoundTrip(d, {0.5f, -0.3f, 0.8f}, "wizard mecanum layout");
}

TEST_CASE("forward and inverse kinematics round trip for every kind with varied inputs") {
    Lcg rng(1234u);
    DrivetrainSpec offset = swerve();
    for (WheelSpec& w : offset.wheels) w.positionM.x += 0.2f;   // origin (the COM) away from the wheel centroid
    DrivetrainSpec offsetMecanum = mecanum();
    for (WheelSpec& w : offsetMecanum.wheels) w.positionM.x += 0.15f;
    DrivetrainSpec threeSwerve = swerve();
    threeSwerve.wheels.pop_back();

    for (int i = 0; i < 50; ++i) {
        const ChassisSpeeds s{2.f * rng.next(), 2.f * rng.next(), 3.f * rng.next()};
        checkRoundTrip(swerve(), s, "swerve");
        checkRoundTrip(offset, s, "swerve with offset origin");
        checkRoundTrip(threeSwerve, s, "three modules");
        checkRoundTrip(mecanum(), s, "mecanum");
        checkRoundTrip(offsetMecanum, s, "mecanum with offset origin");
        const ChassisSpeeds t{s.vx, 0.f, s.omega};
        checkRoundTrip(tank(2), t, "tank");
        checkRoundTrip(tank(3, 0.25f), t, "six wheel tank");
    }
}

TEST_CASE("inverse kinematics: more than four wheels use a least squares fit") {
    DrivetrainSpec d = mecanum();
    WheelSpec extra = d.wheels[0];
    extra.name = "extra";
    extra.positionM = {0.f, 0.4f, extra.positionM.z};   // x * y == 0: a plain forward wheel
    d.wheels.push_back(extra);
    auto sp = raw(d, {1.f, 0.5f, 0.3f});
    auto exact = chassisSpeedsFromWheels(d, sp);
    REQUIRE(exact.ok());
    CHECK(exact.value().vx == doctest::Approx(1.f).epsilon(1e-3));
    CHECK(exact.value().vy == doctest::Approx(0.5f).epsilon(1e-3));

    // A disagreeing fifth wheel pulls the estimate, but only by a share of the error.
    sp.back().wheelSpeedRadPerSec += 5.f;
    auto fit = chassisSpeedsFromWheels(d, sp);
    REQUIRE(fit.ok());
    CHECK(std::fabs(fit.value().vx - 1.f) > 1e-3f);
    CHECK(std::fabs(fit.value().vx - 1.f) < 1.f);
}

TEST_CASE("inverse kinematics: degenerate wheel sets are refused") {
    DrivetrainSpec twoMecanum = mecanum();
    twoMecanum.wheels.resize(2);
    CHECK_FALSE(chassisSpeedsFromWheels(twoMecanum, std::vector<WheelSetpoint>(2)).ok());

    DrivetrainSpec oneSide = tank(2);
    oneSide.wheels.resize(2);   // both on the left
    CHECK_FALSE(chassisSpeedsFromWheels(oneSide, std::vector<WheelSetpoint>(2)).ok());

    CHECK_FALSE(chassisSpeedsFromWheels(swerve(), std::vector<WheelSetpoint>(3)).ok());   // wrong count
}

TEST_CASE("wheel speed limit follows the drive motor free speed and the gearing") {
    WheelSpec w;
    w.driveGearRatio = 6.75f;
    const double free = w.driveMotor.freeSpeedRadPerSec;
    CHECK(maxWheelSpeedRadPerSec(w) == doctest::Approx(free / 6.75).epsilon(1e-6));
    CHECK(maxWheelSpeedRadPerSec(w, w.driveMotor.nominalVoltageV / 2) == doctest::Approx(free / 6.75 / 2).epsilon(1e-6));
    CHECK(maxWheelSpeedRadPerSec(w, std::numeric_limits<double>::quiet_NaN()) == doctest::Approx(free / 6.75).epsilon(1e-6));
    w.driveGearRatio = 0.f;
    CHECK(maxWheelSpeedRadPerSec(w) == 0.f);
    w.driveGearRatio = 6.75f;
    w.driveMotor.freeSpeedRadPerSec = -1.0;
    CHECK(maxWheelSpeedRadPerSec(w) == 0.f);
    CHECK(maxLinearSpeedMps(swerve()) == doctest::Approx(maxWheelSpeedRadPerSec(WheelSpec{}) * WheelSpec{}.radiusM).epsilon(1e-6));
}

TEST_CASE("desaturation scales all wheel speeds uniformly and keeps their ratios") {
    const DrivetrainSpec d = mecanum();
    const float limit = maxWheelSpeedRadPerSec(d.wheels[0]);
    std::vector<WheelSetpoint> sp = raw(d, {6.f, 3.f, 4.f});
    const std::vector<WheelSetpoint> before = sp;
    float peak = 0.f;
    for (const WheelSetpoint& s : before) peak = std::max(peak, std::fabs(s.wheelSpeedRadPerSec));
    REQUIRE(peak > limit);

    const float scale = desaturateWheelSetpoints(d, sp);
    CHECK(scale == doctest::Approx(limit / peak).epsilon(1e-5));
    float newPeak = 0.f;
    for (std::size_t i = 0; i < sp.size(); ++i) {
        newPeak = std::max(newPeak, std::fabs(sp[i].wheelSpeedRadPerSec));
        CHECK(sp[i].wheelSpeedRadPerSec == doctest::Approx(before[i].wheelSpeedRadPerSec * scale).epsilon(1e-5));
        CHECK(sp[i].steerAngleRad == before[i].steerAngleRad);
    }
    CHECK(newPeak == doctest::Approx(limit).epsilon(1e-5));

    std::vector<WheelSetpoint> slow = raw(d, {0.5f, 0.f, 0.f});
    const auto slowBefore = slow;
    CHECK(desaturateWheelSetpoints(d, slow) == 1.f);
    CHECK(slow[0].wheelSpeedRadPerSec == slowBefore[0].wheelSpeedRadPerSec);
}

TEST_CASE("the full pipeline desaturates before it sets module headings") {
    const DrivetrainSpec d = swerve();
    auto fast = wheelSetpoints(d, {50.f, 0.f, 0.f});
    REQUIRE(fast.ok());
    const float limit = maxWheelSpeedRadPerSec(d.wheels[0]);
    for (const WheelSetpoint& sp : fast.value()) {
        CHECK(sp.wheelSpeedRadPerSec == doctest::Approx(limit).epsilon(1e-5));
        CHECK(sp.steerAngleRad == doctest::Approx(0.f).scale(1.0).epsilon(kTol));
    }
}

TEST_CASE("swerve optimisation flips a module that would turn more than 90 degrees") {
    const WheelSetpoint target{10.f, 170.f * kPiF / 180.f};
    const WheelSetpoint flipped = optimizeSwerveSetpoint(target, 0.f);
    CHECK(flipped.wheelSpeedRadPerSec == doctest::Approx(-10.f));
    CHECK(flipped.steerAngleRad == doctest::Approx(-10.f * kPiF / 180.f).epsilon(1e-4));

    const WheelSetpoint kept = optimizeSwerveSetpoint({10.f, 80.f * kPiF / 180.f}, 0.f);
    CHECK(kept.wheelSpeedRadPerSec == doctest::Approx(10.f));
    CHECK(kept.steerAngleRad == doctest::Approx(80.f * kPiF / 180.f).epsilon(1e-4));

    // Exactly 90 degrees is not flipped.
    const WheelSetpoint edge = optimizeSwerveSetpoint({10.f, kPiF / 2}, 0.f);
    CHECK(edge.wheelSpeedRadPerSec == doctest::Approx(10.f));
}

TEST_CASE("swerve optimisation takes the shortest turn across the wrap") {
    const float deg = kPiF / 180.f;
    const WheelSetpoint across = optimizeSwerveSetpoint({4.f, -179.f * deg}, 179.f * deg);
    CHECK(across.wheelSpeedRadPerSec == doctest::Approx(4.f));
    // 2 degrees away through the wrap: the angle is the same direction as the target.
    CHECK(std::fabs(std::remainder(across.steerAngleRad - (-179.f * deg), 2.f * kPiF)) < 1e-4f);

    const WheelSetpoint back = optimizeSwerveSetpoint({4.f, 179.f * deg}, -179.f * deg);
    CHECK(back.wheelSpeedRadPerSec == doctest::Approx(4.f));

    // Previous at 179 deg and a target of 1 deg: 178 deg apart through the wrap, so the module flips.
    const WheelSetpoint nearWrap = optimizeSwerveSetpoint({4.f, 1.f * deg}, 179.f * deg);
    CHECK(nearWrap.wheelSpeedRadPerSec == doctest::Approx(-4.f));   // 178 deg away: the flipped module is 2 deg from the target + pi
    CHECK(std::fabs(std::remainder(nearWrap.steerAngleRad - (1.f + 180.f) * deg, 2.f * kPiF)) < 1e-4f);

    // An unwrapped previous angle (several turns) gives the same shortest turn.
    const WheelSetpoint spun = optimizeSwerveSetpoint({4.f, 0.2f}, 6.f * kPiF + 0.1f);
    CHECK(spun.wheelSpeedRadPerSec == doctest::Approx(4.f));
    CHECK(std::fabs(std::remainder(spun.steerAngleRad - (6.f * kPiF + 0.1f), 2.f * kPiF)) < 0.101f);
}

TEST_CASE("swerve optimisation never turns a module by more than 90 degrees") {
    Lcg rng(99u);
    for (int i = 0; i < 200; ++i) {
        const float prev = kPiF * rng.next();
        const WheelSetpoint target{3.f * rng.next() + 3.5f, kPiF * rng.next()};
        const WheelSetpoint out = optimizeSwerveSetpoint(target, prev);
        const double turn = wrapAngleRad(static_cast<double>(out.steerAngleRad) - prev);
        CHECK(std::fabs(turn) <= 0.5 * kPi + 2e-4);
        // The wheel still moves the same way on the ground.
        const double dir = target.wheelSpeedRadPerSec * std::cos(target.steerAngleRad);
        const double dirOut = out.wheelSpeedRadPerSec * std::cos(out.steerAngleRad);
        CHECK(dir == doctest::Approx(dirOut).epsilon(1e-4).scale(1.0));
    }
}

TEST_CASE("zero speed keeps the previous steer angle instead of snapping to zero") {
    auto created = DriveKinematics::create(swerve());
    REQUIRE(created.ok());
    DriveKinematics kin = std::move(created).value();

    auto strafe = kin.update({0.f, 1.f, 0.f});
    REQUIRE(strafe.ok());
    for (const WheelSetpoint& sp : strafe.value()) CHECK(sp.steerAngleRad == doctest::Approx(kPiF / 2).epsilon(kTol));

    auto stopped = kin.update({0.f, 0.f, 0.f});
    REQUIRE(stopped.ok());
    for (const WheelSetpoint& sp : stopped.value()) {
        CHECK(sp.wheelSpeedRadPerSec == 0.f);
        CHECK(sp.steerAngleRad == doctest::Approx(kPiF / 2).epsilon(kTol));
    }
    // And a restart straight ahead turns by 90 degrees rather than flipping through a stale state.
    auto forward = kin.update({1.f, 0.f, 0.f});
    REQUIRE(forward.ok());
    for (const WheelSetpoint& sp : forward.value()) {
        CHECK(sp.wheelSpeedRadPerSec == doctest::Approx(1.f / WheelSpec{}.radiusM).epsilon(kTol));
        CHECK(sp.steerAngleRad == doctest::Approx(0.f).scale(1.0).epsilon(kTol));
    }

    kin.resetSteer();
    for (const float a : kin.previousSteerRad()) CHECK(a == 0.f);
}

TEST_CASE("driving backward from rest flips the modules instead of steering them half a turn") {
    auto kin = DriveKinematics::create(swerve()).value();
    auto reverse = kin.update({-1.f, 0.f, 0.f});
    REQUIRE(reverse.ok());
    for (const WheelSetpoint& sp : reverse.value()) {
        CHECK(sp.wheelSpeedRadPerSec < 0.f);
        CHECK(std::fabs(sp.steerAngleRad) < 1e-4f);
    }
}

TEST_CASE("kinematics refuse non-finite input and unusable specs") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (const DrivetrainSpec& d : {swerve(), tank(2), mecanum()}) {
        CHECK_FALSE(wheelSetpointsRaw(d, {nan, 0.f, 0.f}).ok());
        CHECK_FALSE(wheelSetpointsRaw(d, {0.f, inf, 0.f}).ok());
        CHECK_FALSE(wheelSetpoints(d, {0.f, 0.f, -inf}).ok());
        std::vector<WheelSetpoint> bad(d.wheels.size());
        bad[0].wheelSpeedRadPerSec = nan;
        CHECK_FALSE(chassisSpeedsFromWheels(d, bad).ok());
        bad[0] = {0.f, inf};
        CHECK_FALSE(chassisSpeedsFromWheels(d, bad).ok());
    }
    DrivetrainSpec badPos = swerve();
    badPos.wheels[1].positionM.x = nan;
    CHECK_FALSE(wheelSetpointsRaw(badPos, {1.f, 0.f, 0.f}).ok());
    DrivetrainSpec badRadius = swerve();
    badRadius.wheels[1].radiusM = 0.f;
    CHECK_FALSE(wheelSetpointsRaw(badRadius, {1.f, 0.f, 0.f}).ok());
    DrivetrainSpec none;
    CHECK_FALSE(wheelSetpointsRaw(none, {1.f, 0.f, 0.f}).ok());
    DrivetrainSpec noMotor = swerve();
    noMotor.wheels[0].driveGearRatio = 0.f;
    CHECK_FALSE(wheelSetpoints(noMotor, {1.f, 0.f, 0.f}).ok());
    CHECK_FALSE(DriveKinematics::create(noMotor).ok());
    const std::vector<float> wrongSize(2, 0.f);
    CHECK_FALSE(wheelSetpointsRaw(swerve(), {1.f, 0.f, 0.f}, wrongSize).ok());
}

TEST_CASE("an unknown drivetrain kind is refused rather than guessed") {
    DrivetrainSpec d = swerve();
    d.kind = "crab";
    auto r = wheelSetpointsRaw(d, {1.f, 0.f, 0.f});
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::Unsupported);
    CHECK_FALSE(wheelSetpoints(d, {1.f, 0.f, 0.f}).ok());
    CHECK_FALSE(chassisSpeedsFromWheels(d, std::vector<WheelSetpoint>(4)).ok());
    CHECK_FALSE(DriveKinematics::create(d).ok());
}

TEST_CASE("a fixed wheel in a swerve drivetrain projects the point velocity on its axis") {
    DrivetrainSpec d = swerve();
    d.wheels[2].kind = WheelKind::Standard;
    d.wheels[2].steered = false;
    const float r = d.wheels[2].radiusM;
    const auto sp = raw(d, {1.f, 0.4f, 0.8f});
    CHECK(sp[2].steerAngleRad == 0.f);
    CHECK(sp[2].wheelSpeedRadPerSec == doctest::Approx((1.f - 0.8f * d.wheels[2].positionM.y) / r).epsilon(kTol));
    checkRoundTrip(d, {1.f, 0.4f, 0.8f}, "swerve with a fixed wheel");
}
