// Remote robots in SimulationSession: roster rules, placement, input scheduling, game hooks, directory and snapshot. The
// session runs over the real Box3D world and drivetrain registries (remote_robot_rig.h); movement, gating, contacts and
// determinism are in test_remote_robots_physics.cpp.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <tuple>

#include "remote_robot_rig.h"

using namespace remote_test;
using proto::AllianceStation;

namespace {

constexpr float kFieldL = default_field::kLengthM;
constexpr float kFieldW = default_field::kWidthM;
constexpr AllianceStation kAllStations[] = {AllianceStation::Red1,  AllianceStation::Red2,  AllianceStation::Red3,
                                            AllianceStation::Blue1, AllianceStation::Blue2, AllianceStation::Blue3};

float distance(const RobotStartPose& a, const RobotStartPose& b) { return std::hypot(a.xM - b.xM, a.yM - b.yM); }

}  // namespace

// ---- roster -----------------------------------------------------------------------------------------------------

TEST_CASE("remote robots: add, query and remove") {
    Rig rig;
    CHECK(rig.session->remoteRobotIds().empty());
    rig.add(7, AllianceStation::Blue1, swerveRobot("Alpha"));
    rig.add(3, AllianceStation::Red2, tankRobot("Beta"));

    CHECK(rig.session->remoteRobotIds() == std::vector<ParticipantId>{3, 7});   // ascending id
    const auto info = rig.session->remoteRobotInfo(7);
    REQUIRE(info.has_value());
    CHECK(info->name == "Alpha-7");
    CHECK(info->station == AllianceStation::Blue1);
    CHECK(info->stats.lastAppliedSeq == match::kNoInputSeq);
    CHECK(info->stats.ackTick == proto::kNoAckTick);
    CHECK_FALSE(rig.session->remoteRobotInfo(99).has_value());

    CHECK(rig.session->removeRemoteRobot(7).ok());
    CHECK(rig.session->remoteRobotIds() == std::vector<ParticipantId>{3});
    const Status again = rig.session->removeRemoteRobot(7);
    REQUIRE_FALSE(again.ok());
    CHECK(again.error().code == ErrorCode::NotFound);
    rig.add(7, AllianceStation::Blue1, swerveRobot("Alpha"));   // the id and station are free again
    CHECK(rig.session->remoteRobotIds().size() == 2);
}

TEST_CASE("remote robots: roster limits and duplicates are rejected without side effects") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());

    SUBCASE("reserved ids") {
        for (const ParticipantId id : {match::kHostParticipantId, match::kNoParticipant}) {
            const Status s = rig.session->addRemoteRobot(requestFor(id, AllianceStation::Blue2, swerveRobot()));
            REQUIRE_FALSE(s.ok());
            CHECK(s.error().code == ErrorCode::InvalidArgument);
        }
    }
    SUBCASE("duplicate id and duplicate station") {
        const Status id = rig.session->addRemoteRobot(requestFor(1, AllianceStation::Blue2, swerveRobot()));
        REQUIRE_FALSE(id.ok());
        CHECK(id.error().code == ErrorCode::AlreadyExists);
        const Status station = rig.session->addRemoteRobot(requestFor(2, AllianceStation::Blue1, swerveRobot()));
        REQUIRE_FALSE(station.ok());
        CHECK(station.error().code == ErrorCode::AlreadyExists);
    }
    SUBCASE("at most kMaxRemoteRobots") {
        const AllianceStation stations[] = {AllianceStation::Blue2, AllianceStation::Blue3, AllianceStation::Red1, AllianceStation::Red2};
        for (std::size_t i = 0; i < kMaxRemoteRobots - 1; ++i) rig.add(static_cast<ParticipantId>(2 + i), stations[i], tankRobot());
        CHECK(rig.session->remoteRobotIds().size() == kMaxRemoteRobots);
        const Status s = rig.session->addRemoteRobot(requestFor(20, AllianceStation::Red3, swerveRobot()));
        REQUIRE_FALSE(s.ok());
        CHECK(s.error().code == ErrorCode::OutOfRange);
    }
    SUBCASE("unknown drivetrain kind and tire model") {
        RobotConfig kind = swerveRobot();
        kind.drivetrain.spec.kind = "hovercraft";
        const Status k = rig.session->addRemoteRobot(requestFor(2, AllianceStation::Blue2, kind));
        REQUIRE_FALSE(k.ok());
        CHECK(k.error().code == ErrorCode::InvalidArgument);
        CHECK(k.error().message.find("hovercraft") != std::string::npos);

        RobotConfig tire = swerveRobot();
        tire.drivetrain.spec.tireModel = "slick";
        const Status t = rig.session->addRemoteRobot(requestFor(2, AllianceStation::Blue2, tire));
        REQUIRE_FALSE(t.ok());
        CHECK(t.error().message.find("slick") != std::string::npos);
    }
    SUBCASE("a spec outside the fairness limits lists its violation codes and is not clamped") {
        RobotConfig heavy = swerveRobot();
        heavy.chassis.massKg = 5000.f;
        heavy.drivetrain.spec.wheels[0].driveCurrentLimitA = 0.f;
        const Status s = rig.session->addRemoteRobot(requestFor(2, AllianceStation::Blue2, heavy));
        REQUIRE_FALSE(s.ok());
        CHECK(s.error().code == ErrorCode::InvalidArgument);
        CHECK(s.error().message.find(remote_spec_code::kOutOfRange) != std::string::npos);
        CHECK(s.error().message.find(remote_spec_code::kUnlimitedCurrent) != std::string::npos);
    }
    SUBCASE("too many wheels and too many zones") {
        RobotConfig wheels = swerveRobot();
        for (int i = 0; i < 6; ++i) wheels.drivetrain.spec.wheels.push_back(makeWheel(("x" + std::to_string(i)).c_str(), 0.f, 0.f, WheelKind::Standard, false));
        CHECK_FALSE(rig.session->addRemoteRobot(requestFor(2, AllianceStation::Blue2, wheels)).ok());
        CHECK_FALSE(rig.session->addRemoteRobot(requestFor(2, AllianceStation::Blue2, swerveRobot("Z", match::kMaxZonesPerRobot + 1))).ok());
        CHECK(rig.session->addRemoteRobot(requestFor(2, AllianceStation::Blue2, swerveRobot("Z", match::kMaxZonesPerRobot))).ok());
    }
    SUBCASE("a session without a chassis factory does not support remote robots") {
        RigOptions options;
        options.remoteSupport = false;
        Rig bare(options);
        const Status s = bare.session->addRemoteRobot(requestFor(1, AllianceStation::Blue1, swerveRobot()));
        REQUIRE_FALSE(s.ok());
        CHECK(s.error().code == ErrorCode::Unsupported);
    }
    CHECK(rig.session->remoteRobotIds().size() >= 1);   // the failures left the roster intact
    CHECK(rig.session->robots().robots().size() == rig.session->remoteRobotIds().size() + 1);
}

// ---- placement --------------------------------------------------------------------------------------------------

TEST_CASE("remote start poses face the right way and spread the stations like the driver station numbering") {
    const FieldFrame frame{kFieldL, kFieldW, FieldSymmetry::Rotational};
    const float setback = std::min(2.f, kFieldL * 0.15f);
    for (int number = 1; number <= 3; ++number) {
        const RobotStartPose blue = remoteStartPose(frame, proto::stationOf(Alliance::Blue, number));
        const RobotStartPose red = remoteStartPose(frame, proto::stationOf(Alliance::Red, number));
        CHECK(blue.xM == doctest::Approx(setback));
        CHECK(blue.headingRad == doctest::Approx(0.f));
        CHECK(red.xM == doctest::Approx(kFieldL - setback));
        CHECK(red.headingRad == doctest::Approx(static_cast<float>(kPi)));
        // The same lateral slot as the driver station spots the field draws.
        CHECK(blue.yM == doctest::Approx(driverStationSpot(defaultFieldDescription(), Alliance::Blue, number)->eyeM.y));
        CHECK(red.yM == doctest::Approx(driverStationSpot(defaultFieldDescription(), Alliance::Red, number)->eyeM.y));
    }
    // Left to right as seen by the alliance: blue looks along +X (left is +Y), red along -X (left is -Y).
    CHECK(remoteStartPose(frame, AllianceStation::Blue1).yM > remoteStartPose(frame, AllianceStation::Blue2).yM);
    CHECK(remoteStartPose(frame, AllianceStation::Blue2).yM > remoteStartPose(frame, AllianceStation::Blue3).yM);
    CHECK(remoteStartPose(frame, AllianceStation::Red1).yM < remoteStartPose(frame, AllianceStation::Red2).yM);
    CHECK(remoteStartPose(frame, AllianceStation::Red2).yM < remoteStartPose(frame, AllianceStation::Red3).yM);
    CHECK(remoteStartPose(frame, AllianceStation::Blue2).yM == doctest::Approx(kFieldW * 0.5f));
}

TEST_CASE("remote start poses never overlap the host start or each other and stay inside the field") {
    const FieldFrame frame{kFieldL, kFieldW, FieldSymmetry::Rotational};
    const float radius = 0.6f;
    std::vector<StartDisc> placed{{kHostStartX, kHostStartY, radius}};   // the host's default start
    for (const AllianceStation station : kAllStations) {
        const RobotStartPose p = remoteStartPoseClear(frame, station, radius, placed);
        CHECK(p.xM >= radius);
        CHECK(p.xM <= kFieldL - radius);
        CHECK(p.yM >= radius);
        CHECK(p.yM <= kFieldW - radius);
        for (const StartDisc& d : placed) CHECK(std::hypot(p.xM - d.xM, p.yM - d.yM) >= radius + d.radiusM);
        placed.push_back({p.xM, p.yM, radius});
        // Sliding away from the alliance wall never turns the robot around.
        CHECK(p.headingRad == doctest::Approx(remoteStartPose(frame, station).headingRad));
    }
    // Blue 2 stands where the host does by default, so it is the one that slid down the field.
    CHECK(remoteStartPoseClear(frame, AllianceStation::Blue2, radius, std::vector<StartDisc>{{kHostStartX, kHostStartY, radius}}).xM > kHostStartX + radius);
    // Without anything in the way the plain station pose is used.
    CHECK(remoteStartPoseClear(frame, AllianceStation::Blue1, radius, {}).xM == doctest::Approx(remoteStartPose(frame, AllianceStation::Blue1).xM));
}

TEST_CASE("remote start poses fit a small field") {
    const FieldFrame frame{5.f, 2.f, FieldSymmetry::None};
    for (const AllianceStation station : kAllStations) {
        const RobotStartPose p = remoteStartPoseClear(frame, station, 0.5f, {});
        CHECK(p.xM >= 0.5f);
        CHECK(p.xM <= 4.5f);
        CHECK(p.yM >= 0.5f);
        CHECK(p.yM <= 1.5f);
    }
}

TEST_CASE("remote robots are placed on their stations without overlapping anybody in the session") {
    Rig rig;
    ParticipantId id = 1;
    for (const AllianceStation station : {AllianceStation::Blue1, AllianceStation::Blue2, AllianceStation::Blue3, AllianceStation::Red1, AllianceStation::Red2})
        rig.add(id++, station, swerveRobot());
    const float radius = chassisFootprintRadiusM(swerveRobot().chassis);
    std::vector<RemoteRobotInfo> infos;
    for (const ParticipantId rid : rig.session->remoteRobotIds()) infos.push_back(*rig.session->remoteRobotInfo(rid));
    for (std::size_t i = 0; i < infos.size(); ++i) {
        const RobotStartPose& p = infos[i].placement;
        CHECK(std::hypot(p.xM - kHostStartX, p.yM - kHostStartY) >= 2.f * radius);
        for (std::size_t j = i + 1; j < infos.size(); ++j) CHECK(distance(p, infos[j].placement) >= 2.f * radius);
        // The body really is where the placement says.
        const Pose3 pose = rig.poseOf(infos[i].participantId);
        CHECK(pose.position.x == doctest::Approx(p.xM).epsilon(0.001));
        CHECK(pose.position.y == doctest::Approx(p.yM).epsilon(0.001));
        CHECK(std::abs(pose.rotation.yaw()) == doctest::Approx(std::abs(p.headingRad)).epsilon(0.001));
    }
    // Blue 1 and blue 3 are on the blue half facing +X, red 1 on the red half facing -X.
    CHECK(rig.session->remoteRobotInfo(1)->placement.headingRad == doctest::Approx(0.f));
    CHECK(std::abs(rig.session->remoteRobotInfo(4)->placement.headingRad) == doctest::Approx(static_cast<float>(kPi)));
}

// ---- input scheduling -----------------------------------------------------------------------------------------------

TEST_CASE("remote inputs apply exactly on their target tick") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    ScriptedClient client(rig, 1, swerveRobot());
    const ChassisSpeeds forward{1.f, 0.f, 0.f};

    REQUIRE(client.send(forward, 10).value() == RemoteInputOutcome::Queued);
    for (int t = 0; t < 10; ++t) {
        CHECK(rig.session->remoteRobotInfo(1)->stats.applied == 0);
        rig.tick();   // ticks 0..9
    }
    CHECK(rig.session->remoteRobotInfo(1)->stats.applied == 0);
    CHECK(rig.session->remoteRobotInfo(1)->stats.queueDepth == 1);
    rig.tick();       // tick 10
    const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.applied == 1);
    CHECK(stats.lastAppliedSeq == 1);
    CHECK(stats.ackTick == 10);
    CHECK(stats.queueDepth == 0);
}

TEST_CASE("remote inputs: the newest sequence that is due wins and older ones are superseded") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    ScriptedClient client(rig, 1, swerveRobot());
    REQUIRE(client.send({1.f, 0.f, 0.f}, 5).ok());   // seq 1
    REQUIRE(client.send({1.f, 0.f, 0.f}, 6).ok());   // seq 2
    REQUIRE(client.send({1.f, 0.f, 0.f}, 50).ok());  // seq 3, far later
    rig.tick(8);                                      // ticks 0..7: seq 1 applies on tick 5, seq 2 on tick 6
    const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.lastAppliedSeq == 2);
    CHECK(stats.applied == 2);
    rig.tick(50);
    CHECK(rig.session->remoteRobotInfo(1)->stats.lastAppliedSeq == 3);
    CHECK(rig.session->remoteRobotInfo(1)->stats.ackTick == 50);

    // Two due on the same tick: only the newest sequence is applied.
    Rig other;
    other.add(1, AllianceStation::Blue1, swerveRobot());
    ScriptedClient c2(other, 1, swerveRobot());
    REQUIRE(c2.send({1.f, 0.f, 0.f}, 3).ok());
    REQUIRE(c2.send({0.5f, 0.f, 0.f}, 3).ok());
    other.tick(4);
    const RemoteInputStats s2 = other.session->remoteRobotInfo(1)->stats;
    CHECK(s2.lastAppliedSeq == 2);
    CHECK(s2.applied == 1);
    CHECK(s2.superseded == 1);
}

TEST_CASE("remote inputs: arrival order does not change the result") {
    const auto run = [](const std::vector<int>& order) {
        Rig rig;
        rig.add(1, AllianceStation::Blue1, swerveRobot());
        std::vector<match::MatchInput> inputs;
        ScriptedClient client(rig, 1, swerveRobot());
        for (int i = 1; i <= 8; ++i) {
            const ChassisSpeeds speeds{0.4f * static_cast<float>(i % 4), 0.2f * static_cast<float>(i % 3), 0.3f};
            inputs.push_back(inputOf(static_cast<std::uint32_t>(i), static_cast<std::uint64_t>(3 * i), client.setpointsFor(speeds)));
        }
        for (const int i : order) REQUIRE(rig.session->submitRemoteInput(1, inputs[static_cast<std::size_t>(i)]).ok());
        rig.tick(60);
        const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
        return std::make_tuple(rig.hash(), stats.applied, stats.lastAppliedSeq, stats.superseded);
    };
    const auto inOrder = run({0, 1, 2, 3, 4, 5, 6, 7});
    CHECK(run({7, 6, 5, 4, 3, 2, 1, 0}) == inOrder);
    CHECK(run({3, 0, 6, 1, 7, 2, 5, 4}) == inOrder);
    CHECK(std::get<1>(inOrder) == 8);
}

TEST_CASE("remote inputs: late, duplicate, stale and rejected inputs are counted") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    ScriptedClient client(rig, 1, swerveRobot());
    const auto sp = client.setpointsFor({1.f, 0.f, 0.f});

    rig.tick(20);   // the next tick is 20
    CHECK(rig.session->submitRemoteInput(1, inputOf(1, 25, sp)).value() == RemoteInputOutcome::Queued);
    CHECK(rig.session->submitRemoteInput(1, inputOf(1, 25, sp)).value() == RemoteInputOutcome::Duplicate);
    CHECK(rig.session->submitRemoteInput(1, inputOf(2, 12, sp)).value() == RemoteInputOutcome::QueuedLate);   // target already passed
    rig.tick();   // tick 20: seq 2 (late) applies at once
    CHECK(rig.session->remoteRobotInfo(1)->stats.lastAppliedSeq == 2);
    CHECK(rig.session->submitRemoteInput(1, inputOf(2, 30, sp)).value() == RemoteInputOutcome::Duplicate);   // the one applied last
    CHECK(rig.session->submitRemoteInput(1, inputOf(1, 30, sp)).value() == RemoteInputOutcome::Stale);       // older than the applied one
    CHECK(rig.session->submitRemoteInput(1, inputOf(3, 40, sp)).value() == RemoteInputOutcome::Queued);
    CHECK(rig.session->submitRemoteInput(1, inputOf(3, 41, sp)).value() == RemoteInputOutcome::Duplicate);   // queued
    rig.tick(10);
    CHECK(rig.session->submitRemoteInput(1, inputOf(1, 40, sp)).value() == RemoteInputOutcome::Stale);

    CHECK_FALSE(rig.session->submitRemoteInput(1, inputOf(0, 40, sp)).ok());                                 // sequence 0 is reserved
    auto fewer = sp;
    fewer.pop_back();
    const auto mismatch = rig.session->submitRemoteInput(1, inputOf(9, 40, fewer));                          // wrong wheel count
    REQUIRE_FALSE(mismatch.ok());
    CHECK(mismatch.error().code == ErrorCode::InvalidArgument);
    const auto tooFar = rig.session->submitRemoteInput(1, inputOf(10, rig.nextTick() + 100000, sp));
    REQUIRE_FALSE(tooFar.ok());
    CHECK(tooFar.error().code == ErrorCode::OutOfRange);
    CHECK(rig.session->submitRemoteInput(77, inputOf(11, 40, sp)).error().code == ErrorCode::NotFound);

    const RemoteInputStats s = rig.session->remoteRobotInfo(1)->stats;
    CHECK(s.late == 1);
    CHECK(s.duplicate == 3);
    CHECK(s.stale == 2);
    CHECK(s.rejected == 3);
    CHECK(s.superseded == 1);
    CHECK(s.received == 11);   // the unknown robot's input is not counted
}

TEST_CASE("remote inputs: the input lead is the newest sequence's target minus its arrival tick") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    ScriptedClient client(rig, 1, swerveRobot());
    const auto sp = client.setpointsFor({0.f, 0.f, 0.f});
    rig.tick(100);
    REQUIRE(rig.session->submitRemoteInput(1, inputOf(5, 112, sp)).ok());
    CHECK(rig.session->remoteRobotInfo(1)->stats.inputLeadTicks == 12);
    REQUIRE(rig.session->submitRemoteInput(1, inputOf(4, 140, sp)).ok());   // older sequence: no new lead measurement
    CHECK(rig.session->remoteRobotInfo(1)->stats.inputLeadTicks == 12);
    REQUIRE(rig.session->submitRemoteInput(1, inputOf(6, 95, sp)).ok());    // newer and late: negative
    CHECK(rig.session->remoteRobotInfo(1)->stats.inputLeadTicks == -5);
}

TEST_CASE("remote inputs: the queue is bounded however many inputs are flooded") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    ScriptedClient client(rig, 1, swerveRobot());
    const auto sp = client.setpointsFor({0.5f, 0.f, 0.f});
    const std::uint64_t target = rig.nextTick() + 150;   // within the lead limit, so every one is a valid input
    for (std::uint32_t seq = 1; seq <= 5000; ++seq) (void)rig.session->submitRemoteInput(1, inputOf(seq, target, sp));
    const RemoteRobotInfo info = *rig.session->remoteRobotInfo(1);
    CHECK(info.stats.queueDepth <= kMaxQueuedRemoteInputs);
    CHECK(info.stats.queueDepth == kMaxQueuedRemoteInputs);
    CHECK(info.stats.evicted == 5000 - kMaxQueuedRemoteInputs);
    CHECK(info.stats.queued == 5000);
    // The oldest were evicted, the newest kept: the sequence that applies is the last one.
    rig.tick(151);
    CHECK(rig.session->remoteRobotInfo(1)->stats.lastAppliedSeq == 5000);

    // An input older than everything in a full queue is dropped, not queued.
    Rig full;
    full.add(1, AllianceStation::Blue1, swerveRobot());
    for (std::uint32_t seq = 1000; seq < 1000 + kMaxQueuedRemoteInputs; ++seq) (void)full.session->submitRemoteInput(1, inputOf(seq, 100, sp));
    CHECK(full.session->submitRemoteInput(1, inputOf(5, 100, sp)).value() == RemoteInputOutcome::Stale);
    CHECK(full.session->remoteRobotInfo(1)->stats.queueDepth == kMaxQueuedRemoteInputs);
}

TEST_CASE("remote inputs: sequence numbers wrap") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    ScriptedClient client(rig, 1, swerveRobot());
    const auto sp = client.setpointsFor({0.f, 0.f, 0.f});
    for (const std::uint32_t seq : {0xFFFFFFFEu, 0xFFFFFFFFu, 1u, 2u}) REQUIRE(rig.session->submitRemoteInput(1, inputOf(seq, 0, sp)).ok());
    rig.tick();
    CHECK(rig.session->remoteRobotInfo(1)->stats.lastAppliedSeq == 2);   // 2 is newer than 0xFFFFFFFF in serial order
    CHECK(rig.session->submitRemoteInput(1, inputOf(0xFFFFFFFFu, 0, sp)).value() == RemoteInputOutcome::Stale);
}

// ---- zones ----------------------------------------------------------------------------------------------------------

TEST_CASE("remote zone flags are masked and active only while enabled and fed") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot("Z", 3));
    ScriptedClient client(rig, 1, swerveRobot("Z", 3));
    const auto zoneFlags = [&] { return rig.session->robots().find(1)->zoneFlags; };

    REQUIRE(client.send({}, rig.nextTick(), 0xFFFFu).ok());   // far more bits than the three zones
    rig.tick();
    CHECK(zoneFlags() == 0b111u);
    CHECK(rig.session->robots().find(1)->zoneFlagActive(2));
    CHECK_FALSE(rig.session->robots().find(1)->zoneFlagActive(3));
    CHECK(rig.session->remoteRobotInfo(1)->zoneFlags == 0b111u);

    rig.setEnabled(false);
    rig.tick();
    CHECK(zoneFlags() == 0u);   // disabled: forced off
    rig.setEnabled(true);
    rig.tick();
    CHECK(zoneFlags() == 0b111u);   // the input is still the current one
    rig.setEnabled(true, true);     // e-stop
    rig.tick();
    CHECK(zoneFlags() == 0u);
    rig.setEnabled(true);
    rig.tick();

    // No input for kInputTimeoutUs of sim time clears the flags. Five ticks have run (0..4), the input applied on tick 0.
    const int timeoutTicks = static_cast<int>(match::kInputTimeoutUs / rig.session->clock().fixedStepUs());
    rig.tick(timeoutTicks - 5);   // up to tick timeoutTicks - 1
    CHECK(zoneFlags() == 0b111u);
    rig.tick(1);                  // tick timeoutTicks: 250 ms after the input applied
    CHECK(zoneFlags() == 0u);
    REQUIRE(client.send({}, rig.nextTick(), 0b010u).ok());   // a new input revives them
    rig.tick();
    CHECK(zoneFlags() == 0b010u);
}

// ---- reset ------------------------------------------------------------------------------------------------------

TEST_CASE("remote robots: reset restores placement, clears queues and keeps the roster") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    rig.add(2, AllianceStation::Red3, tankRobot());
    ScriptedClient client(rig, 1, swerveRobot());
    const RobotStartPose placement = rig.session->remoteRobotInfo(1)->placement;
    client.drive({1.5f, 0.f, 0.f}, 200);
    REQUIRE(client.send({1.f, 0.f, 0.f}, rig.nextTick() + 100).ok());   // a queued, not yet applied input
    CHECK(rig.poseOf(1).position.x > placement.xM + 0.5f);
    CHECK(rig.session->remoteRobotInfo(1)->stats.applied > 0);

    rig.session->reset();
    CHECK(rig.session->clock().tickIndex() == 0);
    CHECK(rig.session->remoteRobotIds() == std::vector<ParticipantId>{1, 2});
    const Pose3 pose = rig.poseOf(1);
    CHECK(pose.position.x == doctest::Approx(placement.xM).epsilon(0.001));
    CHECK(pose.position.y == doctest::Approx(placement.yM).epsilon(0.001));
    CHECK(rig.session->world().linearVelocity(rig.bodyOf(1)).x == doctest::Approx(0.f));
    const RemoteRobotInfo info = *rig.session->remoteRobotInfo(1);
    CHECK(info.stats.applied == 0);
    CHECK(info.stats.queueDepth == 0);
    CHECK(info.stats.lastAppliedSeq == match::kNoInputSeq);
    CHECK(info.stats.ackTick == proto::kNoAckTick);

    // A fresh epoch accepts sequences from the start again and the robot drives as before.
    ScriptedClient again(rig, 1, swerveRobot());
    again.drive({1.5f, 0.f, 0.f}, 100);
    CHECK(rig.poseOf(1).position.x > placement.xM + 0.3f);
}

// ---- hooks, directory and snapshot ----------------------------------------------------------------------------------------

TEST_CASE("remote robots: the game hooks run in the documented order") {
    using Event = testing::RecordingGameState::Event;
    RigOptions options;
    options.recordingGame = true;
    Rig rig(options);
    REQUIRE(rig.game != nullptr);
    REQUIRE(rig.game->recording != nullptr);
    auto& rec = *rig.game->recording;

    // The host robot is announced right after the game state exists, before its first reset, and is in the directory.
    REQUIRE(rec.events.size() == 1);
    CHECK(rec.events[0] == Event{Event::Kind::Added, match::kHostParticipantId, 1});
    CHECK(rec.resets == 1);

    rig.add(4, AllianceStation::Blue1, swerveRobot());
    REQUIRE(rec.events.size() == 2);
    CHECK(rec.events[1] == Event{Event::Kind::Added, 4, 2});   // listed in the directory by then
    rig.add(9, AllianceStation::Red1, tankRobot());
    CHECK(rec.events.back() == Event{Event::Kind::Added, 9, 3});

    rig.tick();
    CHECK(rec.events.back() == Event{Event::Kind::Stepped, match::kNoParticipant, 3});
    const std::size_t afterTick = rec.events.size();
    REQUIRE(rig.session->removeRemoteRobot(4).ok());
    REQUIRE(rec.events.size() == afterTick + 1);
    CHECK(rec.events.back() == Event{Event::Kind::Removed, 4, 2});   // already gone from the directory

    // A failed add announces nothing. Neither does tick() add or remove anyone.
    CHECK_FALSE(rig.session->addRemoteRobot(requestFor(9, AllianceStation::Red2, tankRobot())).ok());
    rig.tick(3);
    CHECK(rec.events.size() == afterTick + 1 + 3);
    CHECK(std::count_if(rec.events.begin(), rec.events.end(), [](const Event& e) { return e.kind == Event::Kind::Added; }) == 3);

    // reset() restarts the game but keeps the roster: no re-announcement.
    const int resets = rec.resets;
    rig.session->reset();
    CHECK(rec.resets == resets + 1);
    CHECK(std::count_if(rec.events.begin(), rec.events.end(), [](const Event& e) { return e.kind == Event::Kind::Added; }) == 3);

    // publishRobot runs for remote robots only, after publish.
    rec.published.clear();
    (void)rig.session->buildSnapshot(false, true);
    CHECK(rec.published == std::vector<ParticipantId>{9});
}

TEST_CASE("remote robots: the directory lists the host first, then the remote robots by id") {
    RigOptions options;
    options.recordingGame = true;
    Rig rig(options);
    rig.add(8, AllianceStation::Red2, tankRobot("Tanky", 2));
    rig.add(2, AllianceStation::Blue3, swerveRobot("Swervy"));

    const auto robots = rig.session->robots().robots();
    REQUIRE(robots.size() == 3);
    CHECK(robots[0].participantId == match::kHostParticipantId);
    CHECK(robots[0].isHost);
    CHECK(robots[0].name == "Host");
    CHECK(robots[0].body == rig.session->chassisBody());
    CHECK(robots[0].zones == nullptr);
    CHECK(robots[1].participantId == 2);
    CHECK(robots[2].participantId == 8);
    CHECK_FALSE(robots[1].isHost);
    CHECK(robots[2].station == AllianceStation::Red2);
    CHECK(robots[2].alliance() == Alliance::Red);
    REQUIRE(robots[2].zones != nullptr);
    CHECK(robots[2].zoneCount() == 2);
    CHECK((*robots[2].zones)[1].id == "zone1");
    CHECK(robots[2].body.valid());
    CHECK(robots[1].body != robots[2].body);
    CHECK(rig.session->robots().find(2) == &robots[1]);
    CHECK(rig.session->robots().find(5) == nullptr);

    // The enabled flag follows the host Driver Station for every robot, the host included.
    rig.setEnabled(false);
    rig.tick();
    for (const RobotView& v : rig.session->robots().robots()) CHECK_FALSE(v.enabled);
    rig.setEnabled(true);
    rig.tick();
    for (const RobotView& v : rig.session->robots().robots()) CHECK(v.enabled);
    proto::DsState ds = rig.session->driverStation();
    ds.station = AllianceStation::Blue3;
    rig.session->setDriverStation(ds);
    rig.tick();
    CHECK(rig.session->robots().robots()[0].station == AllianceStation::Blue3);
}

TEST_CASE("remote robots: snapshots carry one sample per remote robot and leave the host chassis alone") {
    RigOptions options;
    options.recordingGame = true;
    Rig rig(options);
    const auto before = rig.session->buildSnapshot(false, true);
    CHECK(before->remoteRobots().empty());

    rig.add(5, AllianceStation::Red3, swerveRobot("Sw", 2));
    rig.add(6, AllianceStation::Blue1, tankRobot("Tk"));
    ScriptedClient client(rig, 5, swerveRobot("Sw", 2));
    client.drive({1.f, 0.f, 0.3f}, 80, 4, 0b01u);

    const auto snap = rig.session->buildSnapshot(false, true);
    CHECK(snap->chassis().name == "chassis");
    CHECK(snap->chassis().pose.position.x == doctest::Approx(kHostStartX).epsilon(0.05));
    CHECK(snap->wheels().empty());   // the host has no drivetrain in this rig
    REQUIRE(snap->remoteRobots().size() == 2);
    const RobotSample& a = snap->remoteRobots()[0];
    const RobotSample& b = snap->remoteRobots()[1];
    CHECK(a.participantId == 5);
    CHECK(b.participantId == 6);
    CHECK(a.name == "Sw-5");
    CHECK(a.station == AllianceStation::Red3);
    CHECK_FALSE(a.isHost);
    const Pose3 pose = rig.poseOf(5);
    CHECK(a.body.pose.position.x == doctest::Approx(pose.position.x));
    CHECK(a.body.linearVelocity.x == doctest::Approx(rig.session->world().linearVelocity(rig.bodyOf(5)).x));
    CHECK(a.body.angularVelocity.z == doctest::Approx(rig.session->world().angularVelocity(rig.bodyOf(5)).z));
    REQUIRE(a.wheels.size() == 4);
    CHECK(a.wheels[0].name == "fl");
    CHECK(a.wheels[0].normalLoadN > 10.f);
    CHECK_FALSE(a.wheels[0].airborne);
    CHECK(std::abs(a.wheels[0].spinAngleRad) > 0.5);   // the wheels have turned
    REQUIRE(a.zones.size() == 2);
    CHECK(a.zones[0].id == "zone0");
    CHECK(a.zones[0].active);
    CHECK_FALSE(a.zones[1].active);
    REQUIRE(b.wheels.size() == 4);
    CHECK(b.zones.empty());
    // The per-robot HUD comes from the game's publishRobot.
    REQUIRE(a.hud.size() == 1);
    CHECK(a.hud[0].key == "recording.seen");
    CHECK(b.hud.size() == 1);
}

TEST_CASE("remote robots: a session without remote robots has an unchanged snapshot") {
    Rig rig;
    rig.tick(20);
    const auto snap = rig.session->buildSnapshot(false, true);
    CHECK(snap->remoteRobots().empty());
    CHECK(snap->bodies().empty());
    CHECK(rig.session->robots().robots().size() == 1);   // just the host
    CHECK(rig.session->robots().robots()[0].isHost);
}
