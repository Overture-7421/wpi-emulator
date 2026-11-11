#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "physics_test_support.h"
#include "wpiemu/core/physics/wheel_setpoint_controller.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"

// WheelSetpointController on real ChassisSimulator rigs: step responses, current limits, braking, hostile inputs,
// kinematics-driven tank / mecanum / swerve robots, two controlled robots pushing each other in one world, determinism.

using namespace physics_test;

namespace {

constexpr double kSteerSettleBandRad = 0.02;   // about 1.1 degrees

// A single robot and its controller.
struct Loop {
    Rig rig;
    phys::WheelSetpointController ctl;
    std::vector<WheelSetpoint> sp;

    explicit Loop(DrivetrainSpec ds, ChassisSpec cs = testChassis(), PhysicsConfig cfg = {},
                  phys::ChassisPlacement placement = {})
        : rig(std::move(ds), std::move(cs), cfg, placement), ctl(rig.drivetrainSpec), sp(rig.cmd.size()) {
        REQUIRE(rig.sim);
    }

    void set(float wheelSpeed, float steerAngle = 0.f) {
        for (WheelSetpoint& s : sp) s = {wheelSpeed, steerAngle};
        ctl.setSetpoints(sp);
    }
    void step() {
        ctl.step(rig.sim->drivetrain().wheelStates(), kDt, rig.cmd);
        rig.step();
    }
    void run(float seconds) {
        const int n = static_cast<int>(seconds / kDt + 0.5f);
        for (int i = 0; i < n; ++i) step();
    }
    std::span<const WheelState> states() const { return rig.sim->drivetrain().wheelStates(); }
};

struct Response {
    double settleS = 0.0;      // last time the error left the band
    double overshoot = 0.0;    // fraction of the initial error
    int crossings = 0;         // sign changes of the error outside a tiny band
    bool settled = false;
};

// Runs `seconds` and analyses the error series err(t) = target - value (wrapped by the caller) of one signal.
template <typename ErrorFn>
Response analyse(Loop& loop, float seconds, double band, ErrorFn error) {
    Response r;
    const double e0 = error();
    double last = e0;
    const int n = static_cast<int>(seconds / kDt + 0.5f);
    double worstOvershoot = 0.0;
    for (int i = 0; i < n; ++i) {
        loop.step();
        const double e = error();
        if (std::fabs(e) > band) r.settleS = (i + 1) * static_cast<double>(kDt);
        if (e * e0 < 0.0) worstOvershoot = std::max(worstOvershoot, std::fabs(e));
        if (std::fabs(e) > 0.1 * band && std::fabs(last) > 0.1 * band && e * last < 0.0) ++r.crossings;
        if (std::fabs(e) > 0.1 * band) last = e;
    }
    r.overshoot = worstOvershoot / std::max(1e-9, std::fabs(e0));
    r.settled = std::fabs(error()) <= band && r.settleS < seconds - 0.1;
    return r;
}

double meanOmega(const Loop& l) {
    double s = 0.0;
    for (const WheelState& w : l.states()) s += w.driveWheelOmegaRadPerSec;
    return s / static_cast<double>(l.states().size());
}

float forwardSpeed(const Loop& l) { return l.rig.bodyVelocity().x; }

DrivetrainSpec swerveWith(const MotorSpec& steerMotor, float steerGear) {
    DrivetrainSpec d = swerveSpec(60.f);
    for (WheelSpec& w : d.wheels) {
        w.steerMotor = steerMotor;
        w.steerGearRatio = steerGear;
    }
    return d;
}

}  // namespace

TEST_CASE("wheel setpoint: drive step response settles quickly without overshoot on a swerve rig") {
    Loop loop(swerveSpec(60.f));
    loop.run(0.3f);
    const double target = 40.0;   // rad/s, about 2 m/s
    loop.set(static_cast<float>(target));
    const Response r = analyse(loop, 1.5f, 0.05 * target, [&] { return target - meanOmega(loop); });
    MESSAGE("drive step to " << target << " rad/s: settle " << r.settleS << " s, overshoot " << r.overshoot * 100.0
                             << " %, crossings " << r.crossings);
    CHECK(r.settled);
    CHECK(r.settleS < 0.6);
    CHECK(r.overshoot < 0.05);
    CHECK(forwardSpeed(loop) == doctest::Approx(target * loop.rig.drivetrainSpec.wheels[0].radiusM).epsilon(0.1));

    // Stopping: the setpoint back to zero brakes the robot to rest.
    loop.set(0.f);
    loop.run(1.0f);
    CHECK(std::fabs(forwardSpeed(loop)) < 0.1f);
}

TEST_CASE("wheel setpoint: drive gains follow the motor, the gearing and the inertia") {
    DrivetrainSpec spec = swerveSpec(60.f);
    spec.wheels[1].driveGearRatio = 4.f;
    spec.wheels[2].driveMotor = motors::neo();
    spec.wheels[3].driveMotorCount = 2;
    const phys::WheelSetpointController ctl(spec);
    REQUIRE(ctl.wheelCount() == 4);
    for (std::size_t i = 0; i < 4; ++i) {
        const WheelSpec& w = spec.wheels[i];
        CHECK(ctl.gains(i).driveVoltsPerRadPerSec == doctest::Approx(w.driveGearRatio / w.driveMotor.kvRadPerSecPerVolt()));
        CHECK(ctl.gains(i).driveKp > 0.0);
        CHECK(ctl.gains(i).driveLimitV == w.driveMotor.nominalVoltageV);
    }
    CHECK(ctl.gains(1).driveVoltsPerRadPerSec < ctl.gains(0).driveVoltsPerRadPerSec);
}

TEST_CASE("wheel setpoint: steer step responses settle without sustained oscillation for every motor preset") {
    const std::array<float, 2> gearings{12.8f, 150.f / 7.f};
    double worstSettle = 0.0, worstOvershoot = 0.0;
    for (const MotorPreset& preset : motors::presets()) {
        for (const float gear : gearings) {
            // Physically unable to swing a module quickly (the Romi motor at 12.8:1 tops out near 1 rad/s): no controller
            // can settle it within the bound, so it is not a steer motor for this test.
            if (preset.spec.freeSpeedRadPerSec / gear < 20.0) continue;
            Loop loop(swerveWith(preset.spec, gear));
            loop.run(0.2f);
            const std::string label = preset.key + " at " + std::to_string(gear) + ":1";
            INFO(label);
            for (std::size_t i = 0; i < 4; ++i) {
                CHECK(loop.ctl.gains(i).steerDampingRatio >= 0.9);
                CHECK(loop.ctl.gains(i).steerNaturalFreqRadPerSec > 5.0);
            }

            // Steps of 90 degrees, 179 degrees (the wrap), another 179 the other way, and 20 degrees across +-180.
            const std::array<double, 6> targets{0.5 * kPi, -89.0 * kPi / 180.0, 0.5 * kPi, 170.0 * kPi / 180.0,
                                                190.0 * kPi / 180.0, 0.0};
            for (const double target : targets) {
                loop.set(0.f, static_cast<float>(target));
                const Response r = analyse(loop, 0.8f, kSteerSettleBandRad, [&] {
                    double worst = 0.0;
                    for (const WheelState& w : loop.states()) {
                        const double e = wrapAngleRad(target - w.steerAngleRad);
                        if (std::fabs(e) > std::fabs(worst)) worst = e;
                    }
                    return worst;
                });
                INFO("target " << target << " rad");
                CHECK_MESSAGE(r.settled, label << ": steer did not settle to " << target);
                CHECK_MESSAGE(r.settleS < 0.5, label << ": settle time " << r.settleS << " s");
                CHECK_MESSAGE(r.overshoot < 0.02, label << ": overshoot " << r.overshoot);
                CHECK_MESSAGE(r.crossings <= 1, label << ": " << r.crossings << " zero crossings");
                worstSettle = std::max(worstSettle, r.settleS);
                worstOvershoot = std::max(worstOvershoot, r.overshoot);
            }
        }
    }
    MESSAGE("steer over all presets, 12.8:1 and 150/7:1: worst settle " << worstSettle << " s, worst overshoot "
                                                                       << worstOvershoot * 100.0 << " %");
}

TEST_CASE("wheel setpoint: steer step response report for the default modules") {
    Loop loop(swerveSpec(60.f));
    loop.run(0.2f);
    for (const double deg : {90.0, -89.0, 179.0 - 89.0}) {
        const double target = deg * kPi / 180.0;
        loop.set(0.f, static_cast<float>(target));
        const Response r = analyse(loop, 0.8f, kSteerSettleBandRad,
                                   [&] { return wrapAngleRad(target - loop.states()[0].steerAngleRad); });
        MESSAGE("default module to " << deg << " deg: settle " << r.settleS << " s, overshoot " << r.overshoot * 100.0
                                     << " %");
        CHECK(r.settled);
    }
    const auto& g = loop.ctl.gains(0);
    MESSAGE("default steer gains: Kp " << g.steerKp << " V/rad, Kd " << g.steerKd << " V/(rad/s), wn "
                                       << g.steerNaturalFreqRadPerSec << ", zeta " << g.steerDampingRatio
                                       << "; drive Kp " << g.driveKp << " V/(rad/s)");
}

TEST_CASE("wheel setpoint: the stator current limit bounds the acceleration") {
    const double target = 90.0;   // rad/s, about 4.6 m/s: far above what the robot reaches in 0.3 s
    const auto speedAfter = [&](float limitA) {
        Loop loop(swerveSpec(limitA));
        loop.run(0.3f);
        loop.set(static_cast<float>(target));
        loop.run(0.3f);
        return std::pair{forwardSpeed(loop), loop.states()[0].driveMotorCurrentA};
    };
    const auto [limited, limitedCurrent] = speedAfter(30.f);
    const auto [unlimited, unlimitedCurrent] = speedAfter(0.f);
    // 4 wheels x eta x gear x Kt x I / r / mass over 0.3 s.
    const WheelSpec w = swerveSpec(30.f).wheels[0];
    const double accel = 4.0 * w.driveEfficiency * w.driveGearRatio * w.driveMotor.ktNmPerAmp() * 30.0 / w.radiusM / testChassis().massKg;
    MESSAGE("speed after 0.3 s: limited " << limited << " m/s (model " << accel * 0.3 << "), unlimited " << unlimited
                                          << " m/s; wheel current " << limitedCurrent << " vs " << unlimitedCurrent);
    CHECK(limited < 0.75 * unlimited);
    CHECK(limited == doctest::Approx(accel * 0.3).epsilon(0.25));
    CHECK(limitedCurrent <= 30.0 + 1e-6);
}

TEST_CASE("wheel setpoint: disabled outputs zero volts and the robot brakes") {
    Loop loop(swerveSpec(60.f));
    loop.run(0.2f);
    loop.set(40.f);
    loop.run(1.0f);
    const float cruising = forwardSpeed(loop);
    REQUIRE(cruising > 1.5f);

    loop.ctl.setEnabled(false);
    CHECK_FALSE(loop.ctl.enabled());
    loop.step();
    for (const WheelCommand& c : loop.rig.cmd) {
        CHECK(c.driveVoltageV == 0.0);
        CHECK(c.steerVoltageV == 0.0);
    }
    loop.run(0.7f);
    for (const WheelCommand& c : loop.rig.cmd) CHECK(c.driveVoltageV == 0.0);
    CHECK(forwardSpeed(loop) < 0.1f * cruising);

    // Enabled again: the standing setpoint drives the robot once more.
    loop.ctl.setEnabled(true);
    loop.run(1.0f);
    CHECK(forwardSpeed(loop) > 0.8f * cruising);
}

TEST_CASE("wheel setpoint: zeroSpeeds brakes and keeps the module angles") {
    Loop loop(swerveSpec(60.f));
    loop.run(0.2f);
    loop.set(30.f, 0.5f);
    loop.run(1.0f);
    loop.ctl.zeroSpeeds();
    loop.run(1.0f);
    CHECK(length(loop.rig.bodyVelocity()) < 0.1f);
    for (const WheelState& w : loop.states()) CHECK(w.steerAngleRad == doctest::Approx(0.5).epsilon(0.05));
}

TEST_CASE("wheel setpoint: non-finite and wrong-size inputs never reach a wheel command") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const float fnan = std::numeric_limits<float>::quiet_NaN();
    const float finf = std::numeric_limits<float>::infinity();
    const DrivetrainSpec spec = swerveSpec(60.f);
    phys::WheelSetpointController ctl(spec);
    std::vector<WheelState> states(4);
    std::vector<WheelCommand> out(4);

    const auto allFinite = [&] {
        for (const WheelCommand& c : out) {
            CHECK(std::isfinite(c.driveVoltageV));
            CHECK(std::isfinite(c.steerVoltageV));
            CHECK(std::fabs(c.driveVoltageV) <= 12.0);
            CHECK(std::fabs(c.steerVoltageV) <= 12.0);
        }
    };

    std::vector<WheelSetpoint> sp(4, WheelSetpoint{20.f, 1.f});
    ctl.setSetpoints(sp);
    ctl.step(states, kDt, out);
    const double goodDrive = out[0].driveVoltageV;
    const double goodSteer = out[0].steerVoltageV;
    CHECK(goodDrive > 0.0);
    CHECK(goodSteer > 0.0);

    // Non-finite setpoints: speed becomes 0, the angle target is kept.
    sp = {{fnan, fnan}, {finf, finf}, {-finf, -finf}, {fnan, 1.f}};
    ctl.setSetpoints(sp);
    ctl.step(states, kDt, out);
    allFinite();
    CHECK(out[0].driveVoltageV == 0.0);
    CHECK(out[1].driveVoltageV == 0.0);
    CHECK(out[2].driveVoltageV == 0.0);
    CHECK(out[0].steerVoltageV == goodSteer);
    CHECK(ctl.rejectedInputs() == 0);

    // Non-finite states count as zero.
    ctl.setSetpoints(std::vector<WheelSetpoint>(4, WheelSetpoint{20.f, 1.f}));
    for (WheelState& s : states) {
        s.driveWheelOmegaRadPerSec = nan;
        s.steerAngleRad = nan;
        s.steerOmegaRadPerSec = std::numeric_limits<double>::infinity();
    }
    ctl.step(states, kDt, out);
    allFinite();
    CHECK(out[0].driveVoltageV == goodDrive);
    CHECK(out[0].steerVoltageV == goodSteer);

    // Absurd finite setpoints clamp to the nominal voltage.
    ctl.setSetpoints(std::vector<WheelSetpoint>(4, WheelSetpoint{1e30f, 1e30f}));
    ctl.step(std::vector<WheelState>(4), kDt, out);
    allFinite();
    CHECK(out[0].driveVoltageV == 12.0);

    // A setpoint list of the wrong size is ignored and counted.
    ctl.setSetpoints(std::vector<WheelSetpoint>(4, WheelSetpoint{20.f, 1.f}));
    ctl.setSetpoints(std::vector<WheelSetpoint>(3, WheelSetpoint{-20.f, 0.f}));
    ctl.setSetpoints(std::vector<WheelSetpoint>{});
    ctl.setSetpoints(std::vector<WheelSetpoint>(5, WheelSetpoint{-20.f, 0.f}));
    CHECK(ctl.rejectedInputs() == 3);
    ctl.step(std::vector<WheelState>(4), kDt, out);
    CHECK(out[0].driveVoltageV == goodDrive);

    // Wrong-size states or output: every output is zeroed.
    for (WheelCommand& c : out) c.driveVoltageV = 5.0;
    const std::size_t before = ctl.rejectedInputs();
    ctl.step(std::vector<WheelState>(3), kDt, out);
    for (const WheelCommand& c : out) CHECK(c.driveVoltageV == 0.0);
    std::vector<WheelCommand> small(2);
    small[0].driveVoltageV = 5.0;
    ctl.step(std::vector<WheelState>(4), kDt, small);
    CHECK(small[0].driveVoltageV == 0.0);
    CHECK(ctl.rejectedInputs() == before + 2);

    ctl.reset();
    CHECK(ctl.rejectedInputs() == 0);
    for (const WheelSetpoint& s : ctl.setpoints()) CHECK(s.wheelSpeedRadPerSec == 0.f);
}

TEST_CASE("wheel setpoint: a controller for a drivetrain without wheels is inert") {
    phys::WheelSetpointController ctl{DrivetrainSpec{}};
    std::vector<WheelCommand> out;
    ctl.step({}, kDt, out);
    ctl.setSetpoints({});
    CHECK(ctl.wheelCount() == 0);
    CHECK(ctl.rejectedInputs() == 0);
}

namespace {

struct Motion {
    Vec3 velocity;   // chassis frame
    float yawRate = 0.f;
};

// Settles, then drives the setpoints of `cmd` for `seconds` and reports the chassis motion.
Motion driveWithKinematics(const DrivetrainSpec& spec, ChassisSpeeds cmd, float seconds = 2.0f) {
    Loop loop(spec);
    auto created = DriveKinematics::create(spec);
    REQUIRE(created.ok());
    DriveKinematics kin = std::move(created).value();
    auto sp = kin.update(cmd);
    REQUIRE(sp.ok());
    loop.run(0.3f);
    loop.ctl.setSetpoints(sp.value());
    loop.run(seconds);
    return {loop.rig.bodyVelocity(), loop.rig.yawRate()};
}

void expectTranslation(const char* label, const DrivetrainSpec& spec, ChassisSpeeds cmd) {
    const Motion m = driveWithKinematics(spec, cmd);
    const float speed = std::hypot(cmd.vx, cmd.vy);
    const float along = (m.velocity.x * cmd.vx + m.velocity.y * cmd.vy) / speed;
    const float across = (-m.velocity.x * cmd.vy + m.velocity.y * cmd.vx) / speed;
    INFO(label << ": commanded (" << cmd.vx << ", " << cmd.vy << "), simulated (" << m.velocity.x << ", "
               << m.velocity.y << "), yaw " << m.yawRate);
    CHECK_MESSAGE(along > 0.85f * speed, label << ": too slow along the commanded direction");
    CHECK_MESSAGE(along < 1.1f * speed, label << ": too fast along the commanded direction");
    CHECK_MESSAGE(std::fabs(across) < 0.15f * speed, label << ": drifts sideways");
    CHECK_MESSAGE(std::fabs(m.yawRate) < 0.2f, label << ": turns unasked");
}

void expectSpin(const char* label, const DrivetrainSpec& spec, float omega) {
    const Motion m = driveWithKinematics(spec, {0.f, 0.f, omega});
    INFO(label << ": commanded " << omega << " rad/s, simulated " << m.yawRate << ", drift " << length(m.velocity));
    CHECK_MESSAGE(m.yawRate * omega > 0.f, label << ": wrong turning direction");
    CHECK_MESSAGE(std::fabs(m.yawRate) > 0.8f * std::fabs(omega), label << ": turns too slowly");
    CHECK_MESSAGE(std::fabs(m.yawRate) < 1.15f * std::fabs(omega), label << ": turns too fast");
    CHECK_MESSAGE(length(m.velocity) < 0.3f, label << ": translates although only rotation was commanded");
}

}  // namespace

TEST_CASE("wheel setpoint: tank, mecanum and swerve robots follow the setpoints of the kinematics") {
    expectTranslation("tank forward", tankSpec(2, WheelKind::Standard), {1.5f, 0.f, 0.f});
    expectTranslation("tank backward, six wheels", tankSpec(3, WheelKind::Standard), {-1.5f, 0.f, 0.f});
    expectSpin("tank spin", tankSpec(2, WheelKind::Omni), 2.f);

    const DrivetrainSpec mecanum = mecanumSpec(60.f);
    expectTranslation("mecanum forward", mecanum, {1.5f, 0.f, 0.f});
    expectTranslation("mecanum strafe left", mecanum, {0.f, 1.5f, 0.f});
    expectTranslation("mecanum diagonal", mecanum, {1.f, -1.f, 0.f});
    expectSpin("mecanum spin", mecanum, -3.f);

    const DrivetrainSpec swerve = swerveSpec(60.f);
    expectTranslation("swerve forward", swerve, {1.5f, 0.f, 0.f});
    expectTranslation("swerve strafe left (modules turn 90 degrees)", swerve, {0.f, 1.5f, 0.f});
    expectTranslation("swerve backward and left (module flip)", swerve, {-1.2f, 0.5f, 0.f});
    expectSpin("swerve spin", swerve, 3.f);
}

namespace {

// Several controlled robots sharing ONE world: every robot prepares, the world steps once, every robot completes.
struct Fleet {
    struct Robot {
        DrivetrainSpec spec;
        ChassisSpec chassis;
        std::unique_ptr<phys::ChassisSimulator> sim;
        std::unique_ptr<phys::WheelSetpointController> ctl;
        std::vector<WheelCommand> cmd;
        std::vector<WheelSetpoint> sp;

        void set(float wheelSpeed, float steerAngle = 0.f) {
            for (WheelSetpoint& s : sp) s = {wheelSpeed, steerAngle};
            ctl->setSetpoints(sp);
        }
        Vec3 position() const { return sim->pose().position; }
    };

    std::unique_ptr<IPhysicsWorld> world;
    DrivetrainRegistry drivetrains;
    TireModelRegistry tireModels;
    std::vector<std::unique_ptr<Robot>> robots;
    PhysicsConfig cfg;

    explicit Fleet(PhysicsConfig c = {}) : world(makeWorld(c)), cfg(c) {
        (void)phys::registerPhysicsComponents(drivetrains, tireModels);
        addFloor(*world);
    }

    Robot& add(DrivetrainSpec ds, ChassisSpec cs, phys::ChassisPlacement placement, std::uint64_t tag) {
        auto r = std::make_unique<Robot>();
        r->spec = std::move(ds);
        r->chassis = std::move(cs);
        phys::ChassisSimulatorArgs args{*world, r->chassis, r->spec, drivetrains, tireModels, placement, {tag}, cfg.gravity};
        auto created = phys::ChassisSimulator::create(args);
        REQUIRE(created.ok());
        r->sim = std::move(created).value();
        r->ctl = std::make_unique<phys::WheelSetpointController>(r->spec);
        r->cmd.assign(r->spec.wheels.size(), WheelCommand{});
        r->sp.assign(r->spec.wheels.size(), WheelSetpoint{});
        robots.push_back(std::move(r));
        return *robots.back();
    }

    void step() {
        for (auto& r : robots) {
            r->ctl->step(r->sim->drivetrain().wheelStates(), kDt, r->cmd);
            r->sim->prepareStep(r->cmd, kDt);
        }
        world->step(kDt);
        for (auto& r : robots) r->sim->completeStep(kDt);
    }
    void run(float seconds) {
        const int n = static_cast<int>(seconds / kDt + 0.5f);
        for (int i = 0; i < n; ++i) step();
    }
};

ChassisSpec withMass(float massKg) {
    ChassisSpec c = testChassis();
    c.massKg = massKg;
    return c;
}

struct Pushed {
    float winnerX = 0.f, loserX = 0.f, loserVx = 0.f;
    bool tagsSeen = false;
};

// Robot A (+x facing) at x = -1.2 and robot B (facing -x) at x = +1.2 both drive forward at 3 m/s.
Pushed collide(float massA, float massB) {
    Fleet fleet;
    constexpr std::uint64_t kTagA = 1001, kTagB = 1002;
    Fleet::Robot& a = fleet.add(swerveSpec(60.f), withMass(massA), {-1.2f, 0.f, 0.f}, kTagA);
    Fleet::Robot& b = fleet.add(swerveSpec(60.f), withMass(massB), {1.2f, 0.f, static_cast<float>(kPi)}, kTagB);
    CHECK(fleet.world->shapeTag(a.sim->chassisBody().hull) == kTagA);
    CHECK(fleet.world->shapeTag(b.sim->chassisBody().hull) == kTagB);
    CHECK(fleet.world->shapeTag(b.sim->chassisBody().wheelPads[0]) == kTagB);
    fleet.run(0.3f);
    const float wheelSpeed = 3.f / a.spec.wheels[0].radiusM;
    a.set(wheelSpeed);
    b.set(wheelSpeed);

    Pushed out;
    for (int i = 0; i < 800; ++i) {
        fleet.step();
        for (const ContactEvent& e : fleet.world->contactEvents())
            if ((e.tagA == kTagA && e.tagB == kTagB) || (e.tagA == kTagB && e.tagB == kTagA)) out.tagsSeen = true;
    }
    const float ax = a.position().x, bx = b.position().x;
    CHECK(std::isfinite(ax));
    CHECK(std::isfinite(bx));
    CHECK(bx - ax > 0.6f);   // the hulls do not pass through each other (0.7 m square, centres at least 0.7 apart)
    if (massA > massB) {
        out.winnerX = ax;
        out.loserX = bx;
        out.loserVx = fleet.world->linearVelocity(b.sim->body()).x;
    } else {
        out.winnerX = bx;
        out.loserX = ax;
        out.loserVx = -fleet.world->linearVelocity(a.sim->body()).x;
    }
    return out;
}

}  // namespace

TEST_CASE("wheel setpoint: two robots in one world push each other, the heavier one wins") {
    // Heavy A drives at light B: B starts at +x and ends up shoved further to +x.
    const Pushed aWins = collide(80.f, 30.f);
    MESSAGE("A (80 kg) vs B (30 kg): A at " << aWins.winnerX << " m, B at " << aWins.loserX << " m, B speed "
                                           << aWins.loserVx << " m/s");
    CHECK(aWins.tagsSeen);
    CHECK(aWins.winnerX > -0.3f);              // the winner got past the middle
    CHECK(aWins.loserX > 1.2f + 1.0f);         // the loser was shoved back beyond where it started
    CHECK(aWins.loserVx > 1.0f);               // along the winner's direction

    // Swap the masses: B wins and shoves A towards -x.
    const Pushed bWins = collide(30.f, 80.f);
    MESSAGE("A (30 kg) vs B (80 kg): B at " << bWins.winnerX << " m, A at " << bWins.loserX << " m, A speed "
                                           << bWins.loserVx << " m/s (towards -x)");
    CHECK(bWins.tagsSeen);
    CHECK(bWins.winnerX < 0.3f);
    CHECK(bWins.loserX < -1.2f - 1.0f);
    CHECK(bWins.loserVx > 1.0f);
}

TEST_CASE("wheel setpoint: two robots of equal mass stall against each other") {
    Fleet fleet;
    Fleet::Robot& a = fleet.add(swerveSpec(60.f), testChassis(), {-1.2f, 0.f, 0.f}, 1);
    Fleet::Robot& b = fleet.add(swerveSpec(60.f), testChassis(), {1.2f, 0.f, static_cast<float>(kPi)}, 2);
    fleet.run(0.3f);
    a.set(60.f);
    b.set(60.f);
    fleet.run(3.0f);
    CHECK(std::fabs(a.position().x + b.position().x) < 0.4f);   // the meeting point stays near the middle
    CHECK(b.position().x - a.position().x > 0.65f);
}

namespace {

std::uint64_t twoRobotHash(int workers) {
    PhysicsConfig cfg;
    cfg.workerCount = workers;
    Fleet fleet(cfg);
    Fleet::Robot& a = fleet.add(swerveSpec(60.f), withMass(70.f), {-1.0f, -0.2f, 0.2f}, 11);
    Fleet::Robot& b = fleet.add(swerveSpec(60.f), withMass(45.f), {1.0f, 0.1f, static_cast<float>(kPi) - 0.1f}, 12);
    for (int i = 0; i < 80; ++i) {
        const float x = -0.6f + 0.17f * static_cast<float>(i % 10);
        const float y = -0.8f + 0.17f * static_cast<float>(i / 10);
        addBall(*fleet.world, {x, y, 0.1f});
    }
    fleet.run(0.3f);
    for (int i = 0; i < 600; ++i) {
        const double t = i * static_cast<double>(kDt);
        a.set(55.f, static_cast<float>(0.3 * std::sin(t)));
        b.set(50.f, static_cast<float>(-0.3 * std::sin(1.3 * t)));
        fleet.step();
    }
    return fleet.world->stateHash();
}

}  // namespace

TEST_CASE("wheel setpoint: two controlled robots hash identically for every physics worker count") {
    const std::uint64_t one = twoRobotHash(1);
    CHECK(one == twoRobotHash(1));
    CHECK(one == twoRobotHash(2));
    CHECK(one == twoRobotHash(4));
}
