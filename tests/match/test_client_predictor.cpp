#include "match_test_support.h"

#include <cmath>

#include "wpiemu/core/match/own_predictor.h"
#include "wpiemu/sdk/game/default_field.h"

using namespace match_test;
using wpiemu::core::match::OwnAuthoritative;
using wpiemu::core::match::OwnBodyState;
using wpiemu::core::match::OwnPredictor;
using wpiemu::core::match::OwnPredictorConfig;
using wpiemu::core::match::PredictedInput;
using wpiemu::core::match::PredictionWorld;
using wpiemu::core::match::PredictionWorldArgs;
using wpiemu::core::match::ReconcileKind;

namespace {

// The authoritative state a host snapshot would carry for `s`: through the wire's quantization, as a client receives it.
OwnAuthoritative wireRoundTrip(const OwnBodyState& s) {
    wire::RobotWireState r;
    r.xMm = wire::quantizeMilliI32(s.pose.position.x);
    r.yMm = wire::quantizeMilliI32(s.pose.position.y);
    r.yawQ = wire::quantizeAngle(s.pose.rotation.yaw());
    r.vxMmps = wire::quantizeMilliI16(s.linearVelocity.x);
    r.vyMmps = wire::quantizeMilliI16(s.linearVelocity.y);
    r.omegaMradps = wire::quantizeMilliI16(s.angularVelocity.z);
    for (const WheelKinematics& w : s.wheels) r.wheels.push_back({wire::quantizeAngle(w.steerAngleRad), wire::quantizeAngle(w.driveWheelAngleRad)});
    return core::match::authoritativeFromWire(r);
}

struct Bench {
    PipelineKit kit;
    RobotConfig spec = swerveSpec("Predicted");
    sdk::DriveKinematics kin = sdk::DriveKinematics::create(spec.drivetrain.spec).value();

    std::unique_ptr<OwnPredictor> make(float x = 4.f, float y = 4.f, float heading = 0.f, OwnPredictorConfig config = {}) {
        PredictionWorldArgs args{spec, nullptr, kit.drivetrains, kit.tires, {}, RobotStartPose{x, y, heading}, 1, 60};
        auto world = PredictionWorld::create(args);
        REQUIRE_MESSAGE(world.ok(), (world.ok() ? "" : world.error().message));
        return std::make_unique<OwnPredictor>(std::move(world).value(), config);
    }

    std::vector<sdk::WheelSetpoint> setpoints(float vx, float vy = 0.f, float omega = 0.f) {
        auto sp = kin.update({vx, vy, omega});
        REQUIRE(sp.ok());
        return std::move(sp).value();
    }

    PredictedInput input(std::uint32_t seq, std::uint64_t tick, float vx, float vy = 0.f, float omega = 0.f) {
        PredictedInput in;
        in.seq = seq;
        in.targetTick = tick;
        in.setpoints = setpoints(vx, vy, omega);
        return in;
    }
};

OwnAuthoritative restAt(float x, float y, float yaw) {
    OwnAuthoritative a;
    a.x = x;
    a.y = y;
    a.yaw = yaw;
    a.steerRad.assign(4, 0.0);
    a.spinRad.assign(4, 0.0);
    return a;
}

double distance(const OwnBodyState& a, const OwnBodyState& b) {
    return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

// Runs `ticks` ticks of the script: input 1 (vx = 1.5) at tick 0 and input 2 (vx = 0.5, turning) at tick `second`.
void runScript(Bench& b, OwnPredictor& p, std::uint64_t ticks, std::uint64_t second, std::vector<OwnBodyState>* record = nullptr) {
    for (std::uint64_t t = p.tick(); t < ticks; ++t) {
        PredictedInput in;
        const bool first = t == 0, change = t == second;
        if (first) in = b.input(1, t, 1.5f);
        if (change) in = b.input(2, t, 0.5f, 0.f, 0.6f);
        p.stepTick(first || change ? &in : nullptr, true);
        if (record != nullptr) record->push_back(p.newest());
    }
}

}  // namespace

TEST_CASE("own predictor: the same inputs from the same start give the same states and the robot moves") {
    Bench b;
    auto a = b.make();
    auto c = b.make();
    a->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    c->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    runScript(b, *a, 200, 120);
    runScript(b, *c, 200, 120);
    CHECK(a->tick() == 200);
    CHECK(distance(a->newest(), c->newest()) == 0.0);
    CHECK(a->newest().pose.position.x > 4.8f);   // 0.6 s at up to 1.5 m/s
    CHECK(a->newest().pose.rotation.yaw() > 0.05f);   // the second input turns it
}

TEST_CASE("own predictor: an authoritative state identical to the prediction is in band and costs no replay") {
    Bench b;
    auto client = b.make();
    auto server = b.make();
    client->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    server->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    std::vector<OwnBodyState> truth;
    runScript(b, *server, 160, 100, &truth);
    runScript(b, *client, 190, 100);
    // truth[i] is the server after i + 1 ticks.
    const wpiemu::core::match::ReconcileResult r = client->reconcile(wireRoundTrip(truth[149]), 150, 2, true);
    CHECK(r.kind == ReconcileKind::InBand);
    CHECK(r.replayedTicks == 0);
    CHECK(r.posErrM < 0.002);
    CHECK(r.yawErrRad < 0.001);
    CHECK(client->correctionMagnitudeM() == 0.0);
    CHECK(client->inputCount() <= 2);   // acknowledged inputs are pruned down to the newest one
}

namespace {

// Contact the client does not know about: pushes the robot of `p` by (dx, dy) metres, velocities kept.
void shove(OwnPredictor& p, float dx, float dy) {
    OwnBodyState s = p.world().capture();
    s.pose.position.x += dx;
    s.pose.position.y += dy;
    REQUIRE(p.world().restore(s).ok());
}

}  // namespace

TEST_CASE("own predictor: a shove of the host-side robot is replayed and the displayed pose stays continuous") {
    Bench b;
    auto client = b.make();
    auto server = b.make();
    client->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    server->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    runScript(b, *server, 150, 1000);
    runScript(b, *client, 150, 1000);
    shove(*server, 0.f, 0.3f);
    std::vector<OwnBodyState> truth;   // truth[i] = the server after 151 + i ticks
    for (std::uint64_t t = 150; t < 180; ++t) {
        server->stepTick(nullptr, true);
        truth.push_back(server->newest());
    }
    runScript(b, *client, 180, 1000);
    CHECK(distance(client->newest(), server->newest()) > 0.25);   // the client does not know

    const wpiemu::core::match::OwnShown before = client->shown(1.0);
    // The snapshot is for tick 170: 20 ticks behind the client's present.
    const wpiemu::core::match::ReconcileResult r = client->reconcile(wireRoundTrip(truth[19]), 170, 1, true);
    CHECK(r.kind == ReconcileKind::Corrected);
    CHECK(r.replayedTicks == 10);
    CHECK(r.posErrM > 0.25);
    // The present of the client is the server's present again, to within the quantization and the unknown wheel speeds.
    CHECK(distance(client->newest(), server->newest()) < 0.01);
    // What is drawn did not jump: the shift went into the correction offset.
    const wpiemu::core::match::OwnShown after = client->shown(1.0);
    CHECK(std::hypot(after.pose.position.x - before.pose.position.x, after.pose.position.y - before.pose.position.y) < 1e-4);
    CHECK(client->correctionMagnitudeM() == doctest::Approx(0.3).epsilon(0.05));
    // And it decays: after 300 ms (more than six time constants) the picture is the prediction.
    for (int i = 0; i < 60; ++i) client->decayCorrection(5000);
    CHECK(client->correctionMagnitudeM() < 0.001);
    const wpiemu::core::match::OwnShown settled = client->shown(1.0);
    CHECK(std::fabs(settled.pose.position.y - server->newest().pose.position.y) < 0.012);
}

TEST_CASE("own predictor: a correction beyond the limits is shown at once") {
    Bench b;
    auto client = b.make();
    auto server = b.make();
    client->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    server->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    runScript(b, *server, 120, 1000);
    runScript(b, *client, 120, 1000);
    shove(*server, 0.f, 1.6f);
    server->stepTick(nullptr, true);
    client->stepTick(nullptr, true);
    const wpiemu::core::match::ReconcileResult r = client->reconcile(wireRoundTrip(server->newest()), 121, 1, true);
    CHECK(r.kind == ReconcileKind::Snapped);
    CHECK(client->correctionMagnitudeM() == 0.0);
    CHECK(distance(client->newest(), server->newest()) < 0.01);
}

TEST_CASE("own predictor: unacknowledged inputs are replayed on their target ticks") {
    Bench b;
    auto client = b.make();
    auto server = b.make();
    client->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    server->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    // Both apply input 1 at tick 0 and input 2 at tick 130; the server's robot is also shoved at tick 100.
    runScript(b, *server, 100, 130);
    runScript(b, *client, 100, 130);
    shove(*server, 0.2f, -0.2f);
    std::vector<OwnBodyState> truth;
    for (std::uint64_t t = 100; t < 150; ++t) {
        PredictedInput in;
        if (t == 130) in = b.input(2, t, 0.5f, 0.f, 0.6f);
        server->stepTick(t == 130 ? &in : nullptr, true);
        truth.push_back(server->newest());   // truth[i]: after 101 + i ticks
    }
    for (std::uint64_t t = 100; t < 150; ++t) {
        PredictedInput in;
        if (t == 130) in = b.input(2, t, 0.5f, 0.f, 0.6f);
        client->stepTick(t == 130 ? &in : nullptr, true);
    }
    // The snapshot is for tick 120: input 1 acknowledged, input 2 (tick 130) still in flight from the host's point of view.
    const wpiemu::core::match::ReconcileResult r = client->reconcile(wireRoundTrip(truth[19]), 120, 1, true);
    CHECK(r.kind == ReconcileKind::Corrected);
    CHECK(r.replayedTicks == 30);
    CHECK(client->inputCount() == 2);   // input 1 (the acknowledged one) and input 2
    CHECK(distance(client->newest(), server->newest()) < 0.02);
    CHECK(std::fabs(wpiemu::sdk::match::angleDifferenceRad(client->newest().pose.rotation.yaw(), server->newest().pose.rotation.yaw())) < 0.02);
}

TEST_CASE("own predictor: an input the host applied late is replayed from the snapshot's tick") {
    Bench b;
    auto client = b.make();
    auto server = b.make();
    client->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    server->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    // Input 1 targets tick 5; the client applies it there, the host got it 35 ticks (175 ms) late and applies it at 40.
    std::vector<OwnBodyState> truth;
    for (std::uint64_t t = 0; t < 100; ++t) {
        PredictedInput in;
        const bool late = t == 40;
        if (late) in = b.input(1, t, 1.2f);
        server->stepTick(late ? &in : nullptr, true);
        truth.push_back(server->newest());   // after t + 1 ticks
    }
    for (std::uint64_t t = 0; t < 100; ++t) {
        PredictedInput in;
        if (t == 5) in = b.input(1, t, 1.2f);
        client->stepTick(t == 5 ? &in : nullptr, true);
    }
    CHECK(distance(client->newest(), server->newest()) > 0.1);   // a head start the host never gave
    // The snapshot of tick 80 acknowledges input 1 (applied at 40).
    const wpiemu::core::match::ReconcileResult r = client->reconcile(wireRoundTrip(truth[79]), 80, 1, true);
    CHECK(r.kind == ReconcileKind::Corrected);
    CHECK(distance(client->newest(), server->newest()) < 0.02);
}

TEST_CASE("own predictor: snapshots from the future or beyond the history are refused") {
    Bench b;
    auto p = b.make(4.f, 4.f, 0.f, OwnPredictorConfig{.historyTicks = 40});
    CHECK(p->reconcile(restAt(4, 4, 0), 0, 0, true).kind == ReconcileKind::NotStarted);
    p->start(restAt(4.f, 4.f, 0.f), 100, 100, true);
    CHECK(p->tick() == 100);
    runScript(b, *p, 160, 1000);
    CHECK(p->reconcile(restAt(4, 4, 0), 200, 0, true).kind == ReconcileKind::Ahead);
    CHECK(p->reconcile(restAt(4, 4, 0), 100, 0, true).kind == ReconcileKind::TooOld);
}

TEST_CASE("own predictor: a wrong steer angle and wheel spin are restored from the host and smoothed like the pose") {
    Bench b;
    auto p = b.make();
    p->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    PredictedInput in = b.input(1, 0, 0.f, 1.0f);   // a strafe: the modules turn to 90 degrees
    p->stepTick(&in, true);
    for (int i = 0; i < 59; ++i) p->stepTick(nullptr, true);
    REQUIRE(p->tick() == 60);
    const OwnBodyState before = p->newest();
    CHECK(std::fabs(before.wheels[0].steerAngleRad) > 1.0);

    OwnAuthoritative auth = wireRoundTrip(before);
    const double steerBefore = before.wheels[0].steerAngleRad, spinBefore = before.wheels[0].driveWheelAngleRad;
    auth.steerRad[0] += 0.4;
    auth.spinRad[0] += 1.0;
    const wpiemu::core::match::OwnShown shownBefore = p->shown(1.0);
    const wpiemu::core::match::ReconcileResult r = p->reconcile(auth, 60, 1, true);
    CHECK(r.kind == ReconcileKind::Corrected);
    CHECK(r.steerErrRad == doctest::Approx(0.4).epsilon(0.02));
    // The simulation holds the host's angles now (spin is restored too) ...
    CHECK(p->newest().wheels[0].steerAngleRad == doctest::Approx(steerBefore + 0.4).epsilon(0.01));
    CHECK(wire::angleDifferenceRad(p->newest().wheels[0].driveWheelAngleRad, spinBefore + 1.0) == doctest::Approx(0.0).epsilon(0.01).scale(1.0));
    // ... and the picture did not jump: the difference went into the offsets.
    const wpiemu::core::match::OwnShown shownAfter = p->shown(1.0);
    CHECK(std::fabs(shownAfter.wheels[0].steerRad - shownBefore.wheels[0].steerRad) < 1e-3);
    CHECK(std::fabs(shownAfter.wheels[0].spinRad - shownBefore.wheels[0].spinRad) < 1e-3);
    for (int i = 0; i < 60; ++i) p->decayCorrection(5000);
    const wpiemu::core::match::OwnShown settled = p->shown(1.0);
    CHECK(settled.wheels[0].steerRad == doctest::Approx(steerBefore + 0.4).epsilon(0.01));
}

TEST_CASE("prediction world: an unknown drivetrain is an error, remote bodies follow the roster") {
    Bench b;
    RobotConfig bad = b.spec;
    bad.drivetrain.spec.kind = "no_such_drivetrain";
    PredictionWorldArgs args{bad, nullptr, b.kit.drivetrains, b.kit.tires, {}, RobotStartPose{4.f, 4.f, 0.f}, 1, 5};
    CHECK_FALSE(PredictionWorld::create(args).ok());

    auto world = PredictionWorld::create(PredictionWorldArgs{b.spec, nullptr, b.kit.drivetrains, b.kit.tires, {}, RobotStartPose{4.f, 4.f, 0.f}, 1, 5});
    REQUIRE(world.ok());
    PredictionWorld& w = *world.value();
    CHECK(w.wheelCount() == 4);
    CHECK(w.remoteCount() == 0);
    wire::RobotAppearance look;
    look.sizeXMm = look.sizeYMm = 700;
    look.sizeZMm = 120;
    w.setRemote(3, look, {{8.f, 4.f, 0.2f}, sdk::Quat{}}, {}, 0.f);
    w.setRemote(3, look, {{8.1f, 4.f, 0.2f}, sdk::Quat{}}, {1.f, 0.f, 0.f}, 0.f);   // the same body, moved
    CHECK(w.remoteCount() == 1);
    look.sizeXMm = 900;                                                             // a new look is a new body
    w.setRemote(3, look, {{8.1f, 4.f, 0.2f}, sdk::Quat{}}, {}, 0.f);
    CHECK(w.remoteCount() == 1);
    w.setRemote(4, look, {{9.f, 4.f, 0.2f}, sdk::Quat{}}, {}, 0.f);
    CHECK(w.remoteCount() == 2);
    w.removeRemote(3);
    CHECK(w.remoteCount() == 1);
    w.clearRemotes();
    CHECK(w.remoteCount() == 0);

    // The robot can be captured and put back exactly.
    const OwnBodyState s = w.capture();
    CHECK(s.wheels.size() == 4);
    OwnBodyState moved = s;
    moved.pose.position.x += 1.f;
    REQUIRE(w.restore(moved).ok());
    CHECK(w.capture().pose.position.x == doctest::Approx(moved.pose.position.x));
    moved.wheels.pop_back();
    CHECK_FALSE(w.restore(moved).ok());
}

TEST_CASE("own predictor: shown() interpolates between the last two predicted states") {
    Bench b;
    auto p = b.make();
    p->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    PredictedInput in = b.input(1, 0, 2.0f);
    p->stepTick(&in, true);
    for (int i = 0; i < 40; ++i) p->stepTick(nullptr, true);
    const auto a = p->shown(0.0);
    const auto m = p->shown(0.5);
    const auto z = p->shown(1.0);
    CHECK(m.pose.position.x == doctest::Approx(0.5f * (a.pose.position.x + z.pose.position.x)).epsilon(1e-4));
    CHECK(z.pose.position.x > a.pose.position.x);
    CHECK(z.linearVelocity.x >= a.linearVelocity.x);
    CHECK(z.wheels.size() == 4);
}

TEST_CASE("own predictor: the other robots are kinematic bodies the own robot cannot drive through") {
    Bench b;
    auto p = b.make(4.f, 4.f, 0.f);
    p->start(restAt(4.f, 4.f, 0.f), 0, 0, true);
    wire::RobotAppearance look;
    look.sizeXMm = look.sizeYMm = 800;
    look.sizeZMm = 120;
    look.comHeightMm = 200;
    look.centreZMm = -110;   // the box spans the height of the own robot's hull
    look.wheels.push_back({});
    p->setTickHook([&](PredictionWorld& w, std::uint64_t) {
        w.setRemote(7, look, {{5.2f, 4.f, 0.2f}, sdk::Quat{}}, {}, 0.f);
    });
    PredictedInput in = b.input(1, 0, 2.0f);
    p->stepTick(&in, true);
    for (int i = 0; i < 300; ++i) p->stepTick(nullptr, true);
    // 1.5 s at 2 m/s would be 3 m of travel: it stopped against the 0.8 m wide box whose near face is at x = 4.8.
    CHECK(p->newest().pose.position.x < 4.6f);
    CHECK(p->newest().pose.position.x > 4.2f);
    CHECK(p->world().remoteCount() == 1);
}

