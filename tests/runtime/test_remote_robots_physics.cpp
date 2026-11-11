// Remote robots through the whole stack: client wheel setpoints -> session -> WheelSetpointController -> traction
// drivetrain -> the shared Box3D world. Movement, gating, contacts, determinism, teardown and cost.
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "remote_robot_rig.h"

using namespace remote_test;
using proto::AllianceStation;

namespace {

// Both alliances' stations are free on the default field; a robot far from the walls drives clean.
struct Outcome {
    Vec3 bodyVelocity;
    float yawRate = 0.f;
};

Outcome driveAndMeasure(const RobotConfig& spec, AllianceStation station, const ChassisSpeeds& speeds, int ticks = 300) {
    Rig rig;
    rig.add(1, station, spec);
    ScriptedClient client(rig, 1, spec);
    client.drive(speeds, ticks);
    return {rig.bodyVelocity(1), rig.session->world().angularVelocity(rig.bodyOf(1)).z};
}

}  // namespace

// ---- movement ---------------------------------------------------------------------------------------------------

TEST_CASE("remote robot: a swerve robot follows DriveKinematics setpoints open loop") {
    const RobotConfig spec = swerveRobot();
    SUBCASE("forward") {
        const Outcome o = driveAndMeasure(spec, AllianceStation::Blue1, {1.5f, 0.f, 0.f});
        CHECK(o.bodyVelocity.x == doctest::Approx(1.5f).epsilon(0.1));
        CHECK(std::abs(o.bodyVelocity.y) < 0.15f);
        CHECK(std::abs(o.yawRate) < 0.1f);
    }
    SUBCASE("strafe left") {
        const Outcome o = driveAndMeasure(spec, AllianceStation::Blue3, {0.f, 1.f, 0.f});   // blue 3 has the width of the field to its left
        CHECK(o.bodyVelocity.y == doctest::Approx(1.f).epsilon(0.1));
        CHECK(std::abs(o.bodyVelocity.x) < 0.15f);
    }
    SUBCASE("rotate counter-clockwise in place") {
        const Outcome o = driveAndMeasure(spec, AllianceStation::Blue1, {0.f, 0.f, 1.5f});
        CHECK(o.yawRate == doctest::Approx(1.5f).epsilon(0.1));
        CHECK(std::abs(o.bodyVelocity.x) < 0.15f);
        CHECK(std::abs(o.bodyVelocity.y) < 0.15f);
    }
    SUBCASE("a red robot drives toward the blue wall") {
        Rig rig;
        rig.add(1, AllianceStation::Red2, spec);
        ScriptedClient client(rig, 1, spec);
        const float startX = rig.poseOf(1).position.x;
        client.drive({1.5f, 0.f, 0.f}, 300);
        CHECK(rig.poseOf(1).position.x < startX - 1.f);
        CHECK(rig.session->world().linearVelocity(rig.bodyOf(1)).x < -1.2f);
    }
}

TEST_CASE("remote robot: tank and mecanum robots follow DriveKinematics setpoints open loop") {
    SUBCASE("tank forward") {
        const Outcome o = driveAndMeasure(tankRobot(), AllianceStation::Blue1, {1.5f, 0.f, 0.f});
        CHECK(o.bodyVelocity.x == doctest::Approx(1.5f).epsilon(0.1));
        CHECK(std::abs(o.bodyVelocity.y) < 0.15f);
    }
    SUBCASE("tank turns (omni wheels scrub little)") {
        RobotConfig spec = tankRobot();
        for (WheelSpec& w : spec.drivetrain.spec.wheels) w.kind = WheelKind::Omni;
        const Outcome o = driveAndMeasure(spec, AllianceStation::Blue1, {0.f, 0.f, 2.f});
        CHECK(o.yawRate == doctest::Approx(2.f).epsilon(0.3));
        CHECK(std::abs(o.bodyVelocity.x) < 0.3f);
    }
    SUBCASE("mecanum forward") {
        const Outcome o = driveAndMeasure(mecanumRobot(), AllianceStation::Blue1, {1.5f, 0.f, 0.f});
        CHECK(o.bodyVelocity.x == doctest::Approx(1.5f).epsilon(0.15));
        CHECK(std::abs(o.bodyVelocity.y) < 0.25f);
    }
    SUBCASE("mecanum strafe") {
        const Outcome o = driveAndMeasure(mecanumRobot(), AllianceStation::Blue3, {0.f, 1.f, 0.f});
        CHECK(o.bodyVelocity.y > 0.6f);
        CHECK(std::abs(o.bodyVelocity.x) < 0.25f);
    }
}

TEST_CASE("remote robot: the wheels of the snapshot steer and spin") {
    Rig rig;
    const RobotConfig spec = swerveRobot();
    rig.add(1, AllianceStation::Blue1, spec);
    ScriptedClient client(rig, 1, spec);
    client.drive({0.f, 1.f, 0.f}, 200);   // strafe: every module points 90 degrees left
    const auto snap = rig.session->buildSnapshot(false, true);
    REQUIRE(snap->remoteRobots().size() == 1);
    for (const WheelSample& w : snap->remoteRobots()[0].wheels) {
        CHECK(w.steerAngleRad == doctest::Approx(kPi / 2.0).epsilon(0.05));
        CHECK(w.spinAngleRad > 1.0);
    }
}

// ---- gating -----------------------------------------------------------------------------------------------------

TEST_CASE("remote robot: disabled and e-stopped robots do not move and a moving one brakes") {
    const RobotConfig spec = swerveRobot();
    SUBCASE("disabled from the start") {
        Rig rig;
        rig.setEnabled(false);
        rig.add(1, AllianceStation::Blue1, spec);
        ScriptedClient client(rig, 1, spec);
        const float startX = rig.poseOf(1).position.x;
        client.drive({1.5f, 0.f, 0.f}, 200);
        CHECK(rig.poseOf(1).position.x == doctest::Approx(startX).epsilon(0.001));
        CHECK(rig.session->remoteRobotInfo(1)->stats.applied > 0);   // the inputs are still consumed and acknowledged
        CHECK_FALSE(rig.session->remoteRobotInfo(1)->enabled);
    }
    SUBCASE("disabling a moving robot stops it, enabling again lets it drive on") {
        Rig rig;
        rig.add(1, AllianceStation::Blue1, spec);
        ScriptedClient client(rig, 1, spec);
        client.drive({1.5f, 0.f, 0.f}, 300);
        REQUIRE(rig.groundSpeed(1) > 1.2f);
        rig.setEnabled(false);
        client.drive({1.5f, 0.f, 0.f}, 200);   // the client keeps commanding full speed
        CHECK(rig.groundSpeed(1) < 0.1f);
        rig.setEnabled(true);
        client.drive({1.5f, 0.f, 0.f}, 300);
        CHECK(rig.groundSpeed(1) > 1.2f);
    }
    SUBCASE("an e-stop stops a moving robot") {
        Rig rig;
        rig.add(1, AllianceStation::Blue1, spec);
        ScriptedClient client(rig, 1, spec);
        client.drive({1.5f, 0.f, 0.f}, 300);
        REQUIRE(rig.groundSpeed(1) > 1.2f);
        rig.setEnabled(true, true);
        client.drive({1.5f, 0.f, 0.f}, 200);
        CHECK(rig.groundSpeed(1) < 0.1f);
    }
    SUBCASE("a browned-out host battery gives the remote robots 0 V as well") {
        RigOptions options;
        options.brownoutVoltageV = 20.0;
        Rig rig(options);
        rig.add(1, AllianceStation::Blue1, spec);
        ScriptedClient client(rig, 1, spec);
        const float startX = rig.poseOf(1).position.x;
        rig.tick(2);   // the battery latches the brownout after the first tick
        REQUIRE(rig.session->battery().brownedOut());
        client.drive({1.5f, 0.f, 0.f}, 200);
        CHECK(rig.poseOf(1).position.x == doctest::Approx(startX).epsilon(0.001));
        CHECK_FALSE(rig.session->remoteRobotInfo(1)->enabled);
        CHECK_FALSE(rig.session->robots().find(1)->enabled);
    }
}

TEST_CASE("remote robot: no input for 250 ms brakes the robot") {
    const RobotConfig spec = swerveRobot();
    Rig rig;
    rig.add(1, AllianceStation::Blue1, spec);
    ScriptedClient client(rig, 1, spec);
    client.drive({1.5f, 0.f, 0.f}, 300);
    const float cruise = rig.groundSpeed(1);
    REQUIRE(cruise > 1.2f);

    const int timeoutTicks = static_cast<int>(match::kInputTimeoutUs / rig.session->clock().fixedStepUs());
    rig.tick(timeoutTicks / 2);   // silence, but not for long yet: still cruising
    CHECK(rig.groundSpeed(1) > 0.9f * cruise);
    rig.tick(timeoutTicks / 2 + 1 + 200);   // past the timeout plus one second of braking
    CHECK(rig.groundSpeed(1) < 0.15f);

    // A new input drives the robot again.
    client.drive({1.5f, 0.f, 0.f}, 300);
    CHECK(rig.groundSpeed(1) > 1.2f);
}

// ---- contacts ----------------------------------------------------------------------------------------------------

namespace {

// Every robot tag seen in a contact event of the last `ticks` ticks, and the highest approach speed per tag pair.
struct ContactLog {
    std::vector<std::pair<std::uint64_t, std::uint64_t>> pairs;
    void collect(const Rig& rig) {
        for (const ContactEvent& e : rig.session->world().contactEvents())
            if (isRobotTag(e.tagA) && isRobotTag(e.tagB)) pairs.emplace_back(e.tagA, e.tagB);
    }
    bool seen(ParticipantId a, ParticipantId b) const {
        return std::any_of(pairs.begin(), pairs.end(), [&](const auto& p) {
            return (p.first == makeRobotTag(a) && p.second == makeRobotTag(b)) || (p.first == makeRobotTag(b) && p.second == makeRobotTag(a));
        });
    }
};

}  // namespace

TEST_CASE("remote robot: pushing the host robot produces contact events with both robot tags") {
    RigOptions options;
    options.hostX = 5.5f;   // in the way of blue 2, which starts on the same line
    options.hostY = default_field::kWidthM * 0.5f;
    Rig rig(options);
    const RobotConfig spec = tankRobot();
    rig.add(3, AllianceStation::Blue2, spec);
    ScriptedClient client(rig, 3, spec);
    ContactLog log;
    for (int i = 0; i < 500; ++i) {
        client.drive({2.f, 0.f, 0.f}, 1);
        log.collect(rig);
    }
    CHECK(log.seen(match::kHostParticipantId, 3));
    // The host really was shoved down the field.
    CHECK(rig.session->world().pose(rig.session->chassisBody()).position.x > 6.f);
}

TEST_CASE("remote robot: two remote robots colliding produce contact events with both robot tags") {
    RigOptions options;
    options.hostY = 1.f;   // out of the way
    Rig rig(options);
    rig.add(1, AllianceStation::Blue2, swerveRobot());
    rig.add(2, AllianceStation::Red2, tankRobot());
    ScriptedClient a(rig, 1, swerveRobot());
    ScriptedClient b(rig, 2, tankRobot());
    ContactLog log;
    for (int i = 0; i < 1000; ++i) {
        if (i % 4 == 0) {
            REQUIRE(a.send({1.5f, 0.f, 0.f}, rig.nextTick()).ok());
            REQUIRE(b.send({1.5f, 0.f, 0.f}, rig.nextTick()).ok());
        }
        rig.tick();
        log.collect(rig);
    }
    CHECK(log.seen(1, 2));
    // They met head on in the middle and neither got through the other.
    CHECK(rig.poseOf(1).position.x < rig.poseOf(2).position.x);
}

// ---- determinism --------------------------------------------------------------------------------------------------

namespace {

// Three remote robots drive a scripted pattern (with contacts) and the world hash at the end is returned. Inputs are
// submitted up front in `order` (0 = in order, 1 = reversed, 2 = interleaved) with fixed target ticks.
std::uint64_t scenarioHash(int workerCount, int order) {
    RigOptions options;
    options.workerCount = workerCount;
    options.hostX = 3.6f;   // just ahead of blue 2, so the scenario includes a push
    options.hostY = default_field::kWidthM * 0.5f;
    Rig rig(options);
    const RobotConfig specs[] = {swerveRobot(), tankRobot(), swerveRobot()};
    const AllianceStation stations[] = {AllianceStation::Blue2, AllianceStation::Red2, AllianceStation::Blue3};
    std::vector<std::vector<match::MatchInput>> inputs;
    for (ParticipantId id = 1; id <= 3; ++id) {
        const RobotConfig& spec = specs[id - 1];
        rig.add(id, stations[id - 1], spec);
        ScriptedClient client(rig, id, spec);
        std::vector<match::MatchInput> list;
        for (std::uint32_t s = 1; s <= 40; ++s) {
            const float t = static_cast<float>(s);
            const ChassisSpeeds speeds{1.2f + 0.3f * std::sin(0.4f * t + static_cast<float>(id)), 0.4f * std::cos(0.3f * t), 0.5f * std::sin(0.2f * t)};
            list.push_back(inputOf(s, 4ull * s, client.setpointsFor(speeds), s % 5 == 0 ? 1u : 0u));
        }
        inputs.push_back(std::move(list));
    }
    for (ParticipantId id = 1; id <= 3; ++id) {
        std::vector<std::size_t> idx(40);
        for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
        if (order == 1) std::reverse(idx.begin(), idx.end());
        if (order == 2) std::stable_partition(idx.begin(), idx.end(), [](std::size_t i) { return i % 2 == 1; });
        for (const std::size_t i : idx) REQUIRE(rig.session->submitRemoteInput(id, inputs[id - 1][i]).ok());
    }
    rig.tick(260);
    return rig.hash();
}

}  // namespace

TEST_CASE("remote robots: identical runs hash equal and arrival order does not matter") {
    const std::uint64_t reference = scenarioHash(1, 0);
    CHECK(scenarioHash(1, 0) == reference);
    CHECK(scenarioHash(1, 1) == reference);
    CHECK(scenarioHash(1, 2) == reference);
}

TEST_CASE("remote robots: the world hash does not depend on the physics worker count") {
    const std::uint64_t reference = scenarioHash(1, 0);
    CHECK(scenarioHash(2, 0) == reference);
    CHECK(scenarioHash(4, 0) == reference);
}

// ---- lifetime -----------------------------------------------------------------------------------------------------

TEST_CASE("remote robots: a robot removed during a run leaves the others untouched") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    rig.add(2, AllianceStation::Red1, tankRobot());
    ScriptedClient a(rig, 1, swerveRobot());
    ScriptedClient b(rig, 2, tankRobot());
    const int bodiesWithTwo = rig.session->world().stats().bodies;
    for (int i = 0; i < 100; ++i) {
        if (i % 4 == 0) {
            REQUIRE(a.send({1.f, 0.f, 0.f}, rig.nextTick()).ok());
            REQUIRE(b.send({1.f, 0.f, 0.f}, rig.nextTick()).ok());
        }
        rig.tick();
    }
    const BodyId removed = rig.bodyOf(2);
    REQUIRE(rig.session->removeRemoteRobot(2).ok());
    CHECK_FALSE(rig.session->world().isValid(removed));
    CHECK(rig.session->world().stats().bodies == bodiesWithTwo - 1);
    CHECK_FALSE(rig.session->submitRemoteInput(2, inputOf(99, 0, {})).ok());
    for (int i = 0; i < 100; ++i) {
        if (i % 4 == 0) REQUIRE(a.send({1.f, 0.f, 0.f}, rig.nextTick()).ok());
        rig.tick();
    }
    CHECK(rig.groundSpeed(1) > 0.8f);
    CHECK(rig.session->robots().robots().size() == 2);
}

TEST_CASE("remote robots: destroying a session with robots alive is clean") {
    for (int round = 0; round < 3; ++round) {
        Rig rig;
        rig.add(1, AllianceStation::Blue1, swerveRobot());
        rig.add(2, AllianceStation::Red3, tankRobot());
        rig.add(3, AllianceStation::Red1, mecanumRobot());
        ScriptedClient c(rig, 1, swerveRobot());
        c.drive({1.f, 0.f, 0.f}, 40);
        rig.session.reset();   // robots, then world; nothing may touch a destroyed world
    }
    CHECK(true);
}

// ---- cost -----------------------------------------------------------------------------------------------------------

TEST_CASE("remote robots: wall time per tick with five remote robots and 200 fuel") {
    const auto measure = [](int remoteRobots) {
        Rig rig;
        for (int i = 0; i < 200; ++i) {
            BodyDesc body;
            body.type = BodyType::Dynamic;
            body.pose.position = Vec3{7.5f + 0.17f * static_cast<float>(i % 10), 3.4f + 0.17f * static_cast<float>(i / 10), 0.1f};
            const BodyId b = rig.session->world().createBody(body);
            ShapeDesc ball;
            ball.geometry = SphereShape{0.075f};
            ball.density = 113.2f;
            ball.material = {0.6f, 0.5f, 0.02f};
            ball.category = layer::Fuel;
            ball.mask = layer::All;
            rig.session->world().addShape(b, ball);
        }
        const AllianceStation stations[] = {AllianceStation::Blue1, AllianceStation::Blue3, AllianceStation::Red1, AllianceStation::Red2, AllianceStation::Red3};
        std::vector<std::unique_ptr<ScriptedClient>> clients;
        for (int i = 0; i < remoteRobots; ++i) {
            const RobotConfig spec = i % 2 == 0 ? swerveRobot() : tankRobot();
            rig.add(static_cast<ParticipantId>(i + 1), stations[i], spec);
            clients.push_back(std::make_unique<ScriptedClient>(rig, static_cast<ParticipantId>(i + 1), spec));
        }
        rig.tick(60);   // let the fuel settle
        constexpr int kTicks = 800;   // 4 s: the robots cross the field and plough through the fuel
        const auto start = std::chrono::steady_clock::now();
        for (int t = 0; t < kTicks; ++t) {
            if (t % 4 == 0)
                for (auto& c : clients) REQUIRE(c->send({2.5f, 0.f, 0.3f}, rig.nextTick()).ok());
            rig.tick();
        }
        const auto elapsed = std::chrono::steady_clock::now() - start;
        return std::chrono::duration<double, std::milli>(elapsed).count() / kTicks;
    };
    const double withoutRobots = measure(0);
    const double withRobots = measure(5);
    MESSAGE("ms per tick, host + 200 fuel: ", withoutRobots, "; plus 5 remote robots with 4 wheels: ", withRobots,
            " (remote robots add ", withRobots - withoutRobots, " ms)");
    // The gate of the plan (docs: remote robots must stay a small share of the 5 ms tick). Generous on purpose: wall time on a
    // loaded CI machine is noisy; the measured cost is about 0.15 ms.
    CHECK(std::isfinite(withRobots));
#ifdef NDEBUG
    CHECK(withRobots < 2.5);                       // half of the 200 Hz tick budget, with the host and 200 fuel bodies
    CHECK(withRobots - withoutRobots < 1.25);      // the remote robots alone take at most a quarter of it
#else
    CHECK(withRobots < 50.0);
#endif
}
