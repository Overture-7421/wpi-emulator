// The adapter from a client's picture of the match to what the run view's scene draws.
#include <doctest/doctest.h>

#include "client/scene_feed.h"
#include "match/match_test_support.h"
#include "wpiemu/core/match/robot_appearance.h"
#include "wpiemu/sdk/ipc/driver_station.h"

using namespace wpiemu;
using namespace wpiemu::app;
using namespace wpiemu::app::client;
namespace mc = wpiemu::core::match;

namespace {

std::shared_ptr<const sdk::match::RobotAppearance> appearance(float x = 0.7f, std::size_t zones = 0) {
    sdk::RobotConfig spec = match_test::swerveSpec("Bot");
    spec.chassis.sizeM = {x, 0.7f, 0.12f};
    for (std::size_t i = 0; i < zones; ++i) {
        sdk::ZoneConfig z;
        z.id = "zone" + std::to_string(i);
        z.kind = "pickup";
        z.local.position = {0.4f, 0.f, 0.2f};
        z.sizeM = {0.2f, 0.4f, 0.3f};
        spec.zones.push_back(z);
    }
    return std::make_shared<const sdk::match::RobotAppearance>(mc::appearanceOf(spec));
}

mc::MatchClientView view() {
    mc::MatchClientView v;
    v.revision = 7;
    v.connection.state = mc::ConnectionState::Live;
    v.renderSimTimeUs = 123000;
    v.hasOwn = true;
    v.own.id = 2;
    v.own.name = "Me";
    v.own.station = sdk::proto::AllianceStation::Blue2;
    v.own.appearance = appearance(0.7f, 2);
    v.own.pose = {{3.f, 4.f, 0.2f}, sdk::Quat::fromYaw(0.5f)};
    v.own.linearVelocity = {1.f, 0.f, 0.f};
    v.own.wheels = {{0.1f, 1.f}, {0.2f, 2.f}, {0.3f, 3.f}, {0.4f, 4.f}};
    v.own.zoneFlags = 0b10;
    v.own.zonePoses = {sdk::Pose3{{0.4f, 0.f, 0.2f}, {}}, sdk::Pose3{{0.5f, 0.1f, 0.3f}, {}}};
    v.own.hud = {{1, "fuel.count", 12.f}, {2, "shots", 3.f}};

    mc::RemoteRobotView host;
    host.id = 0;
    host.name = "Host";
    host.isHost = true;
    host.station = sdk::proto::AllianceStation::Red1;
    host.appearance = appearance(0.9f);
    host.pose = {{8.f, 4.f, 0.2f}, {}};
    host.wheels = {{0.f, 0.5f}, {0.f, 0.6f}, {0.f, 0.7f}, {0.f, 0.8f}};
    host.zoneFlags = 1;
    host.zonePoses = {sdk::Pose3{}};
    v.remotes.push_back(host);
    mc::RemoteRobotView other = host;
    other.id = 3;
    other.name = "Other";
    other.isHost = false;
    v.remotes.push_back(other);

    v.entities = {{0, {{10, 1.f, 2.f, 0.1f}, {11, 3.f, 4.f, 0.1f}}}};
    v.hud.state = sdk::match::MatchState::Running;
    v.hud.enabled = true;
    v.hud.matchTimeS = 95.f;
    v.hud.phaseKey = "TELEOP";
    v.hud.game = {{4, "score.red", 17.f}, {1, "fuel.count", 99.f}};
    return v;
}

}  // namespace

TEST_CASE("scene feed: the own robot is the snapshot's chassis, wheels and zones") {
    const mc::MatchClientView v = view();
    const auto config = sceneConfigOf(v.own);
    REQUIRE(config);
    const auto snap = snapshotOf(v, config.get(), "phase");
    REQUIRE(snap);
    CHECK(snap->chassis().pose.position.x == doctest::Approx(3.f));
    CHECK(snap->chassis().pose.rotation.yaw() == doctest::Approx(0.5f));
    CHECK(snap->chassis().linearVelocity.x == doctest::Approx(1.f));
    REQUIRE(snap->wheels().size() == 4);
    CHECK(snap->wheels()[2].name == "wheel2");   // the names the scene's rig for this appearance gives its wheels
    CHECK(snap->wheels()[2].steerAngleRad == doctest::Approx(0.3f));
    CHECK(snap->wheels()[2].spinAngleRad == doctest::Approx(3.f));
    REQUIRE(snap->zones().size() == 2);
    CHECK_FALSE(snap->zones()[0].active);
    CHECK(snap->zones()[1].active);
    REQUIRE(snap->zones()[1].poseInChassis.has_value());
    CHECK(snap->zones()[1].poseInChassis->position.y == doctest::Approx(0.1f));
    CHECK(snap->zones()[0].id == config->zones[0].id);   // matched to the scene's zone rig by id
    CHECK(snap->robotConnected());
}

TEST_CASE("scene feed: the other robots, the host's included, are remote samples drawn by index") {
    const auto snap = snapshotOf(view(), nullptr, "");
    REQUIRE(snap);
    REQUIRE(snap->remoteRobots().size() == 2);
    const sdk::RobotSample& host = snap->remoteRobots()[0];
    CHECK(host.participantId == 0);
    CHECK(host.name == "Host");
    CHECK_FALSE(host.isHost);   // the remote rigs skip a sample flagged as the host's own
    CHECK(host.body.pose.position.x == doctest::Approx(8.f));
    CHECK(host.wheels.size() == 4);
    REQUIRE(host.zones.size() == 1);
    CHECK(host.zones[0].active);
    CHECK(snap->remoteRobots()[1].participantId == 3);
}

TEST_CASE("scene feed: the roster lists everyone else with how they look") {
    const mc::MatchClientView v = view();
    const auto roster = rosterOf(v);
    REQUIRE(roster->size() == 2);
    CHECK((*roster)[0].id == 0);
    CHECK((*roster)[0].appearance == v.remotes[0].appearance);
    CHECK((*roster)[1].name == "Other");
    CHECK(*roster == *rosterOf(v));   // equal for an unchanged match, so the scene is not rebuilt
}

TEST_CASE("scene feed: game and own HUD values merge by key, the phase text is published under the game's key") {
    const auto snap = snapshotOf(view(), nullptr, "game.phase");
    REQUIRE(snap);
    const sdk::Value* score = snap->hudValue("score.red");
    REQUIRE(score != nullptr);
    CHECK(score->asDouble() == doctest::Approx(17.0));
    const sdk::Value* fuel = snap->hudValue("fuel.count");
    REQUIRE(fuel != nullptr);
    CHECK(fuel->asDouble() == doctest::Approx(12.0));   // the own robot's value wins over the game's
    const sdk::Value* phase = snap->hudValue("game.phase");
    REQUIRE(phase != nullptr);
    CHECK(phase->asString() == "TELEOP");
    CHECK(snapshotOf(view(), nullptr, "")->hudValue("game.phase") == nullptr);   // a game with no phase key shows none
}

TEST_CASE("scene feed: entities keep their ids and the Driver Station fields follow the host's") {
    const auto snap = snapshotOf(view(), nullptr, "");
    REQUIRE(snap);
    REQUIRE(snap->entityGroups().size() == 1);
    const sdk::EntityGroupSample& g = snap->entityGroups()[0];
    CHECK(g.positions.size() == 2);
    CHECK(g.ids == std::vector<std::uint32_t>{10, 11});
    CHECK(g.radiusM == doctest::Approx(kEntityRadiusM));
    CHECK(snap->driverStation().enabled);
    CHECK(snap->driverStation().matchTimeS == doctest::Approx(95.0));
    CHECK(snap->driverStation().station == sdk::proto::AllianceStation::Blue2);
    CHECK_FALSE(snap->paused());
}

TEST_CASE("scene feed: no own robot yet means no picture, and a missing appearance means no scene config") {
    mc::MatchClientView v = view();
    v.hasOwn = false;
    CHECK(snapshotOf(v, nullptr, "") == nullptr);
    v = view();
    v.own.appearance.reset();
    CHECK(sceneConfigOf(v.own) == nullptr);
    v.connection.state = mc::ConnectionState::Paused;
    v.hud.state = sdk::match::MatchState::Paused;
    CHECK(snapshotOf(v, nullptr, "")->paused());
}

TEST_CASE("scene feed: the scene carries the game's HUD, the field and the own robot") {
    auto field = std::make_shared<const sdk::FieldDescription>();
    const auto own = sceneConfigOf(view().own);
    const HostScene scene = makeClientScene(nullptr, field, own, sdk::proto::AllianceStation::Blue2);
    CHECK(scene.field == field);
    CHECK(scene.config == own);
    CHECK(scene.defaultStation == sdk::proto::AllianceStation::Blue2);
    CHECK(scene.projectPath.isEmpty());
}
