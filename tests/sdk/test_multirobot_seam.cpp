#include <doctest/doctest.h>

#include <memory>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/game/null_objects.h"
#include "wpiemu/sdk/game/robot_view.h"
#include "wpiemu/sdk/game/zone_pose.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/testing/fake_game_module.h"
#include "wpiemu/sdk/testing/fake_robot_directory.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

using namespace wpiemu::sdk;

namespace {

RobotView makeView(ParticipantId id, proto::AllianceStation station, bool host = false) {
    RobotView v;
    v.participantId = id;
    v.name = host ? "host" : "remote" + std::to_string(id);
    v.station = station;
    v.isHost = host;
    return v;
}

}  // namespace

TEST_CASE("multirobot seam: the null directory is empty and finds nothing") {
    NullRobotDirectory dir;
    CHECK(dir.robots().empty());
    CHECK(dir.find(match::kHostParticipantId) == nullptr);
    CHECK(dir.find(7) == nullptr);
}

TEST_CASE("multirobot seam: a robot view derives alliance and reads zone flags") {
    RobotView blue = makeView(2, proto::AllianceStation::Blue2);
    CHECK(blue.alliance() == Alliance::Blue);
    CHECK(makeView(3, proto::AllianceStation::Red3).alliance() == Alliance::Red);

    CHECK(blue.zoneCount() == 0);
    blue.zoneFlags = 0b101;
    CHECK_FALSE(blue.zoneFlagActive(0));   // no zones configured: nothing is active

    const std::vector<ZoneConfig> zones(3);
    blue.zones = &zones;
    CHECK(blue.zoneCount() == 3);
    CHECK(blue.zoneFlagActive(0));
    CHECK_FALSE(blue.zoneFlagActive(1));
    CHECK(blue.zoneFlagActive(2));
    CHECK_FALSE(blue.zoneFlagActive(3));    // out of range
}

TEST_CASE("multirobot seam: the fake directory finds robots by participant id") {
    testing::FakeRobotDirectory dir;
    dir.add(makeView(match::kHostParticipantId, proto::AllianceStation::Red1, true));
    dir.add(makeView(4, proto::AllianceStation::Blue1));
    REQUIRE(dir.robots().size() == 2);
    REQUIRE(dir.find(4) != nullptr);
    CHECK(dir.find(4)->station == proto::AllianceStation::Blue1);
    CHECK(dir.find(match::kHostParticipantId)->isHost);
    CHECK(dir.find(5) == nullptr);
    CHECK(dir.remove(4));
    CHECK_FALSE(dir.remove(4));
    CHECK(dir.find(4) == nullptr);
}

TEST_CASE("multirobot seam: SimContext robots is never null and follows bindRobots") {
    testing::FakeSimEnvironment env;
    CHECK(env.ctx.robots().robots().empty());   // nothing bound by the fake environment

    testing::FakeRobotDirectory dir;
    dir.add(makeView(1, proto::AllianceStation::Red2));
    env.ctx.bindRobots(&dir);
    REQUIRE(env.ctx.robots().robots().size() == 1);
    CHECK(env.ctx.robots().find(1) == &dir.list[0]);

    env.ctx.bindRobots(nullptr);
    CHECK(env.ctx.robots().robots().empty());
}

TEST_CASE("multirobot seam: SimContextServices accepts a trailing robot directory") {
    testing::FakeSimEnvironment env;
    testing::FakeRobotDirectory dir;
    dir.add(makeView(2, proto::AllianceStation::Blue3));

    SimContext without(SimContextServices{env.physics, env.clock, env.hub, env.logger, env.config, env.rng,
                                          env.driverStation, BodyId{}});
    CHECK(without.robots().robots().empty());

    SimContext with(SimContextServices{env.physics, env.clock, env.hub, env.logger, env.config, env.rng,
                                       env.driverStation, BodyId{}, nullptr, nullptr, nullptr, &dir});
    CHECK(with.robots().robots().size() == 1);
    const SimContext& constView = with;
    CHECK(constView.robots().find(2) != nullptr);
}

TEST_CASE("multirobot seam: the defaulted game state hooks are no-ops") {
    testing::FakeSimEnvironment env;
    NullGameState state;
    IGameState& game = state;
    const RobotView view = makeView(1, proto::AllianceStation::Red1);
    game.onRobotAdded(env.ctx, view);
    game.onRobotRemoved(env.ctx, 1);

    SnapshotBuilder builder;
    game.publishRobot(builder, view);
    CHECK(builder.data().remoteRobots.empty());
    CHECK(builder.data().hud.empty());
}

TEST_CASE("multirobot seam: the recording game state captures hook order") {
    testing::FakeSimEnvironment env;
    testing::FakeRobotDirectory dir;
    env.ctx.bindRobots(&dir);
    testing::RecordingGameState state;
    using Kind = testing::RecordingGameState::Event::Kind;

    RobotView& added = dir.add(makeView(3, proto::AllianceStation::Red3));
    state.onRobotAdded(env.ctx, added);
    state.step(env.ctx, 0.005);
    dir.remove(3);
    state.onRobotRemoved(env.ctx, 3);

    REQUIRE(state.events.size() == 3);
    CHECK(state.events[0] == testing::RecordingGameState::Event{Kind::Added, 3, 1});
    CHECK(state.events[1] == testing::RecordingGameState::Event{Kind::Stepped, match::kNoParticipant, 1});
    CHECK(state.events[2] == testing::RecordingGameState::Event{Kind::Removed, 3, 0});
}

TEST_CASE("multirobot seam: the snapshot builder round trips remote robots and match info") {
    SnapshotBuilder builder;

    RobotSample robot;
    robot.participantId = 2;
    robot.name = "bravo";
    robot.station = proto::AllianceStation::Blue1;
    robot.body.name = "chassis";
    robot.body.pose.position = {1.f, 2.f, 0.f};
    robot.wheels.resize(4);
    robot.wheels[1].steerAngleRad = 0.5;
    robot.zones.push_back({"intake", true});
    builder.addRobot(robot);

    testing::RecordingGameState state;
    RobotView view = makeView(2, proto::AllianceStation::Blue1);
    state.publishRobot(builder, view);
    CHECK(builder.setRobotHud(2, "recording.seen", 9));    // overwrites, does not duplicate
    CHECK_FALSE(builder.setRobotHud(9, "x", 1));           // unknown robot: no-op

    match::MatchInfo info;
    info.state = match::MatchState::Paused;
    info.enabled = true;
    info.matchTimeS = 42.5f;
    info.phaseKey = "phase.teleop";
    builder.setMatchInfo(info);

    const std::shared_ptr<const ISimSnapshot> snap = builder.build();
    REQUIRE(snap->remoteRobots().size() == 1);
    const RobotSample& got = snap->remoteRobots()[0];
    CHECK(got.participantId == 2);
    CHECK(got.name == "bravo");
    CHECK(got.station == proto::AllianceStation::Blue1);
    CHECK(got.body.pose.position.x == doctest::Approx(1.f));
    REQUIRE(got.wheels.size() == 4);
    CHECK(got.wheels[1].steerAngleRad == doctest::Approx(0.5));
    REQUIRE(got.zones.size() == 1);
    CHECK(got.zones[0].active);
    REQUIRE(got.hud.size() == 1);
    CHECK(got.hud[0].key == "recording.seen");
    CHECK_FALSE(got.isHost);

    REQUIRE(snap->matchInfo() != nullptr);
    CHECK(*snap->matchInfo() == info);
    CHECK(snap->hud().empty());   // per-robot HUD stays out of the host HUD
}

TEST_CASE("multirobot seam: a snapshot without a match has no match info and no remote robots") {
    const SimSnapshot snap{SimSnapshotData{}};
    CHECK(snap.remoteRobots().empty());
    CHECK(snap.matchInfo() == nullptr);

    // An implementer of ISimSnapshot that predates the additions keeps the defaults.
    const ISimSnapshot& base = snap;
    CHECK(base.zones().empty());
}

TEST_CASE("multirobot seam: entity ids are optional and must match the positions") {
    EntityGroupSample group;
    group.group = "fuel";
    group.positions.resize(3);
    CHECK(group.ids.empty());
    CHECK(entityIdsConsistent(group));       // unidentified

    group.ids = {10, 11, 12};
    CHECK(entityIdsConsistent(group));

    group.ids.pop_back();
    CHECK_FALSE(entityIdsConsistent(group));

    group.positions.clear();
    group.ids.clear();
    CHECK(entityIdsConsistent(group));       // empty group

    SnapshotBuilder builder;
    EntityGroupSample identified;
    identified.positions.resize(2);
    identified.ids = {5, 6};
    builder.addEntityGroup(identified);
    const std::shared_ptr<const ISimSnapshot> snap = builder.build();
    REQUIRE(snap->entityGroups().size() == 1);
    CHECK(snap->entityGroups()[0].ids == std::vector<std::uint32_t>{5, 6});
}

TEST_CASE("multirobot seam: restPoseOfZone resolves chassis zones and refuses a missing mechanism") {
    ZoneConfig zone;
    zone.local.position = {0.3f, 0.1f, 0.2f};

    const std::optional<Pose3> chassis = restPoseOfZone(zone);
    REQUIRE(chassis.has_value());
    CHECK(chassis->position.x == doctest::Approx(0.3f));

    zone.attachment.parentMechanismId = "arm";
    zone.attachment.parentLink = "tip";
    CHECK_FALSE(restPoseOfZone(zone).has_value());

    RobotConfig config;   // no mechanism "arm": the chain is broken
    const LinkPoseSource none = [](std::string_view, std::string_view) { return std::optional<Pose3>{}; };
    CHECK_FALSE(restPoseOfZone(config, zone, none).has_value());

    zone.attachment = MechanismAttachment{};
    const std::optional<Pose3> viaConfig = restPoseOfZone(config, zone, none);
    REQUIRE(viaConfig.has_value());
    CHECK(viaConfig->position.z == doctest::Approx(0.2f));
}

TEST_CASE("multirobot seam: the fake game module state still works unchanged") {
    testing::FakeSimEnvironment env;
    testing::FakeGameModule module;
    auto state = module.createGameState(env.ctx);
    REQUIRE(state.ok());
    IGameState& game = *state.value();
    game.step(env.ctx, 0.005);
    game.onRobotAdded(env.ctx, makeView(1, proto::AllianceStation::Red1));   // default: ignored
    SnapshotBuilder builder;
    game.publish(builder);
    game.publishRobot(builder, makeView(1, proto::AllianceStation::Red1));
    CHECK(builder.data().hud.size() == 1);
    CHECK(builder.data().remoteRobots.empty());
}
