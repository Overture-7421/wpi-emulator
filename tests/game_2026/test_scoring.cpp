#include <doctest/doctest.h>

#include <algorithm>
#include <string>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/game_state.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26test::Session;

namespace {

void emptyField(Session& s) {
    s.section()["neutralFuel"] = 0;
    s.section()["depotFuel"] = 0;
}

sdk::Vec3 hubSensorCentre(Alliance a) {
    const FieldLayout layout = buildFieldLayout();
    const HubSpec& hub = layout.hub(a);
    return {hub.center.x, hub.center.y, dim::kHubSensorCenterZM};
}

void autoEnabled(Session& s) {
    s.env.driverStation.enabled = true;
    s.env.driverStation.mode = sdk::proto::RobotMode::Autonomous;
    s.env.driverStation.matchTimeS = 15.0;   // a timed match (Practice); without it the game is in free play
}

// DS in TELEOP showing `elapsed` seconds into the period (WPILib reports the remaining time).
void teleopAt(Session& s, double elapsed) {
    s.env.driverStation.enabled = true;
    s.env.driverStation.mode = sdk::proto::RobotMode::Teleop;
    s.env.driverStation.matchTimeS = 140.0 - elapsed;
}

}  // namespace

TEST_CASE("fuel dropped into a hub scores for that alliance, leaves the field and is recycled") {
    Session s;
    emptyField(s);
    s.section()["hubProcessingDelayS"] = 0.1;
    s.start();
    autoEnabled(s);

    s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {});
    s.game->pool().spawn(hubSensorCentre(Alliance::Red), {});
    s.game->pool().spawn(hubSensorCentre(Alliance::Red), {});
    s.tick(1);
    CHECK(s.game->score(Alliance::Blue).points() == 1);
    CHECK(s.game->score(Alliance::Red).points() == 2);
    CHECK(s.game->score(Alliance::Blue).autoFuel == 1);  // scored during AUTO
    CHECK(s.game->fuelOnField() == 0);

    // after the processing delay each fuel comes out of a hub exit of its own hub
    s.tick(30);
    CHECK(s.game->fuelOnField() == 3);
    const FieldLayout layout = buildFieldLayout();
    int atBlueExit = 0, atRedExit = 0;
    s.game->pool().forEach([&](std::uint32_t, const sdk::Vec3& p) {
        for (const HubExit& e : layout.hub(Alliance::Blue).exits)
            if (sdk::length(p - e.position) < 1.5f) { ++atBlueExit; break; }
        for (const HubExit& e : layout.hub(Alliance::Red).exits)
            if (sdk::length(p - e.position) < 1.5f) { ++atRedExit; break; }
    });
    CHECK(atBlueExit == 1);
    CHECK(atRedExit == 2);
}

TEST_CASE("scored fuel is removed when recycling is off") {
    Session s;
    emptyField(s);
    s.section()["recycleScoredFuel"] = false;
    s.start();
    autoEnabled(s);
    s.game->pool().spawn(hubSensorCentre(Alliance::Red), {});
    s.tick(400);
    CHECK(s.game->score(Alliance::Red).points() == 1);
    CHECK(s.game->fuelOnField() == 0);
}

TEST_CASE("recycling retains due fuel across a full field cap and conserves it on retry") {
    Session s;
    emptyField(s);
    s.section()["maxFuelOnField"] = 2;
    s.section()["hubProcessingDelayS"] = 0.02;
    s.start();
    autoEnabled(s);
    REQUIRE(s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {}) != 0);
    REQUIRE(s.game->pool().spawn(hubSensorCentre(Alliance::Red), {}) != 0);
    s.tick();
    REQUIRE(s.game->fuelOnField() == 0);
    REQUIRE(s.game->score(Alliance::Blue).points() + s.game->score(Alliance::Red).points() == 2);
    const auto filler1 = s.game->pool().spawn({8.f, 2.f, 0.075f}, {});
    const auto filler2 = s.game->pool().spawn({9.f, 2.f, 0.075f}, {});
    REQUIRE(filler1 != 0);
    REQUIRE(filler2 != 0);
    s.tick(50);
    CHECK(s.game->fuelOnField() == 2);
    CHECK(s.game->pool().spawn({10.f, 2.f, 0.075f}, {}) == 0);
    s.game->pool().despawn(filler1);
    s.tick();
    CHECK(s.game->fuelOnField() == 2);
    s.game->pool().despawn(filler2);
    s.tick();
    CHECK(s.game->fuelOnField() == 2);  // Both scored fuel survived the earlier refused releases.
    CHECK(s.game->fuelInRobot() == 8);
    CHECK(s.game->fuelLost() == 0);
    s.tick();
    CHECK(s.game->fuelOnField() == 2);  // No duplicate releases.
}

TEST_CASE("recycling retries temporary body or shape refusal with bounded spawn attempts") {
    for (const bool refuseBody : {false, true}) {
        INFO(refuseBody);
        g26test::ForwardingWorld world;
        sdk::testing::FakeSimEnvironment env;
        sdk::SimContext ctx(
            {world, env.clock, env.hub, env.logger, env.config, env.rng, env.driverStation, {}});
        Session moduleSource;
        auto fieldResult = moduleSource.module().createField(world);
        REQUIRE(fieldResult.ok());
        auto field = std::move(fieldResult).value();
        GameConfig config;
        config.neutralFuel = config.depotFuel = 0;
        config.maxFuelOnField = 3;
        config.hubProcessingDelayS = 0.02;
        Game2026State game(ctx, config, buildFieldLayout());
        game.reset(ctx);
        for (int i = 0; i < 3; ++i)
            REQUIRE(game.pool().spawn(hubSensorCentre(Alliance::Blue), {}) != 0);
        world.step(0.005f);
        game.step(ctx, 0.005);
        REQUIRE(game.fuelOnField() == 0);
        const auto bodies = world.stats().bodies;
        const auto shapes = world.inner.shapeCount();
        world.refuseBodies = refuseBody;
        if (!refuseBody) world.shapeBudget = 0;
        // Clear the scoring events before reusing the world in the next game step.
        for (int i = 0; i < 20; ++i) {
            world.step(0.005f);
            const int attempts = world.bodyAttempts;
            game.step(ctx, 0.005);
            CHECK(world.bodyAttempts - attempts <= 1);
            CHECK(game.fuelOnField() == 0);
            CHECK(world.stats().bodies == bodies);
            CHECK(world.inner.shapeCount() == shapes);
        }
        world.refuseBodies = false;
        world.shapeBudget = 100;
        world.step(0.005f);
        game.step(ctx, 0.005);
        CHECK(game.fuelOnField() == 3);
        CHECK(game.fuelLost() == 0);
        CHECK(world.stats().bodies == bodies + 3);
        CHECK(world.inner.shapeCount() == shapes + 3);
        world.step(0.005f);
        game.step(ctx, 0.005);
        CHECK(game.fuelOnField() == 3);
        game.reset(ctx);
        CHECK(game.fuelOnField() == 0);
    }
}

TEST_CASE("fuel in a hub that is not active earns nothing, a disabled robot scores nothing") {
    Session s;
    emptyField(s);
    s.section()["firstInactiveHub"] = "red";
    s.start();

    // robot disabled, match not started: both hubs inactive, fuel is processed but not scored
    s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {});
    s.tick(1);
    CHECK(s.game->score(Alliance::Blue).points() == 0);
    CHECK(s.game->score(Alliance::Blue).inactiveFuel == 1);
    CHECK_FALSE(s.game->hubActive(Alliance::Blue));

    // TELEOP, shift 1: the red hub is inactive, blue is active
    teleopAt(s, 20.0);
    s.tick(1);
    CHECK(s.game->match().phase == MatchPhase::Shift1);
    CHECK_FALSE(s.game->hubActive(Alliance::Red));
    CHECK(s.game->hubActive(Alliance::Blue));
    // let the 3 s grace of the red hub expire (it was never active, so there is none)
    s.game->pool().spawn(hubSensorCentre(Alliance::Red), {});
    s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {});
    s.tick(1);
    CHECK(s.game->score(Alliance::Red).points() == 0);
    CHECK(s.game->score(Alliance::Red).inactiveFuel == 1);
    CHECK(s.game->score(Alliance::Blue).points() == 1);
    CHECK(s.game->score(Alliance::Blue).teleopFuel == 1);

    // shift 2 flips the hubs
    teleopAt(s, 40.0);
    s.tick(1);
    CHECK(s.game->match().phase == MatchPhase::Shift2);
    CHECK(s.game->hubActive(Alliance::Red));
    CHECK_FALSE(s.game->hubActive(Alliance::Blue));
    s.game->pool().spawn(hubSensorCentre(Alliance::Red), {});
    s.tick(1);
    CHECK(s.game->score(Alliance::Red).points() == 1);
}

TEST_CASE("hub grace period: fuel arriving within 3 s of deactivation counts, later fuel does not") {
    Session s;
    emptyField(s);
    s.section()["firstInactiveHub"] = "red";  // red inactive in shifts 1 and 3, active in 2 and 4
    s.start();
    teleopAt(s, 40.0);  // shift 2: red active
    s.tick(1);
    REQUIRE(s.game->hubActive(Alliance::Red));
    teleopAt(s, 60.0);  // shift 3: red deactivates now
    s.tick(1);
    REQUIRE_FALSE(s.game->hubActive(Alliance::Red));

    s.game->pool().spawn(hubSensorCentre(Alliance::Red), {});
    s.tick(1);
    CHECK(s.game->score(Alliance::Red).points() == 1);  // inside the 3 s grace

    s.tick(700);  // 3.5 s later
    s.game->pool().spawn(hubSensorCentre(Alliance::Red), {});
    s.tick(1);
    CHECK(s.game->score(Alliance::Red).points() == 1);
    CHECK(s.game->score(Alliance::Red).inactiveFuel == 1);
}

TEST_CASE("the auto winner's hub is the first to go inactive, fuel scored in auto decides it") {
    Session s;
    emptyField(s);
    s.start();
    autoEnabled(s);
    s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {});
    s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {});
    s.game->pool().spawn(hubSensorCentre(Alliance::Red), {});
    s.tick(1);

    teleopAt(s, 12.0);  // shift 1
    s.tick(1);
    REQUIRE(s.game->hubRules().firstInactiveKnown());
    CHECK(s.game->hubRules().firstInactive() == Alliance::Blue);  // blue scored 2 > 1 in auto
    REQUIRE(s.game->gameSpecificMessage().has_value());
    CHECK(*s.game->gameSpecificMessage() == "B");  // forwarded to robot code as FMS game data
    CHECK_FALSE(s.game->hubActive(Alliance::Blue));
    CHECK(s.game->hubActive(Alliance::Red));
}

TEST_CASE("shift enforcement can be switched off for quick testing") {
    Session s;
    emptyField(s);
    s.section()["enforceHubShifts"] = false;
    s.start();
    teleopAt(s, 20.0);
    s.tick(1);
    CHECK(s.game->hubActive(Alliance::Red));
    CHECK(s.game->hubActive(Alliance::Blue));
}

TEST_CASE("fuel pushed into an outpost corral is stored, a timed human player feeds it back") {
    Session s;
    emptyField(s);
    s.section()["humanPlayer"] = "timed";
    s.section()["humanPlayerPeriodS"] = 0.5;
    s.section()["outpostChuteFuel"] = 1;
    s.start();
    teleopAt(s, 20.0);

    const sdk::Vec3 corral{-0.3f, dim::kOutpostCenterYM, 0.1f};  // behind the blue alliance wall
    s.game->pool().spawn(corral, {});
    s.tick(1);
    CHECK(s.game->fuelOnField() == 0);
    CHECK(s.game->outpostStock(Alliance::Blue) == 2);  // 1 staged + 1 delivered
    CHECK(s.game->outpostStock(Alliance::Red) == 1);

    s.tick(110);  // > 0.5 s: one release per period per alliance
    CHECK(s.game->outpostStock(Alliance::Blue) == 1);
    CHECK(s.game->outpostStock(Alliance::Red) == 0);
    CHECK(s.game->fuelOnField() == 2);
    s.tick(1000);
    CHECK(s.game->outpostStock(Alliance::Blue) == 0);
    CHECK(s.game->fuelOnField() == 3);
    s.tick(400);  // nothing left to release
    CHECK(s.game->fuelOnField() == 3);
}

TEST_CASE("the human player is idle when switched off or while the robot is disabled") {
    Session s;
    emptyField(s);
    s.start();
    teleopAt(s, 20.0);
    s.tick(1000);
    CHECK(s.game->fuelOnField() == 0);  // default: off

    Session t;
    emptyField(t);
    t.section()["humanPlayer"] = "timed";
    t.start();
    t.tick(1000);  // robot disabled
    CHECK(t.game->fuelOnField() == 0);
    CHECK(t.game->outpostStock(Alliance::Red) == 24);
}

TEST_CASE("fuel that leaves the playing area is despawned and counted as lost") {
    Session s;
    emptyField(s);
    s.start();
    s.game->pool().spawn({40.f, 3.f, 1.f}, {});
    s.game->pool().spawn({3.f, 3.f, 1.f}, {});
    s.game->pool().spawn({-40.f, 3.f, 1.f}, {});
    s.tick(1);
    CHECK(s.game->fuelOnField() == 1);
    CHECK(s.game->fuelLost() == 2);
}

TEST_CASE("reset restores the initial layout, the meter and every counter") {
    Session s;
    s.start();
    autoEnabled(s);
    s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {});
    s.game->pool().spawn({50.f, 0.f, 0.f}, {});
    s.tick(5);
    REQUIRE(s.game->score(Alliance::Blue).points() == 1);
    REQUIRE(s.game->fuelLost() == 1);

    s.stateOwner->reset(s.env.ctx);
    CHECK(s.game->fuelOnField() == 360 + 48);
    CHECK(s.game->fuelInRobot() == 8);
    CHECK(s.game->score(Alliance::Blue).points() == 0);
    CHECK(s.game->fuelLost() == 0);
    CHECK(s.game->shotsFired() == 0);
    CHECK(s.game->match().phase == MatchPhase::PreMatch);
    CHECK_FALSE(s.game->hubRules().firstInactiveKnown());
    CHECK_FALSE(s.game->gameSpecificMessage().has_value());
    CHECK(s.game->outpostStock(Alliance::Red) == 24);

    // a second reset does not accumulate bodies
    s.stateOwner->reset(s.env.ctx);
    CHECK(s.game->fuelOnField() == 360 + 48);
    CHECK(s.env.physics.stats().bodies == 1 /*field*/ + 1 /*chassis*/ + 360 + 48);
}

TEST_CASE("without a timed match the game is free play: no phases, no match HUD, both hubs score") {
    Session s;
    s.start();
    s.env.driverStation.enabled = true;
    s.env.driverStation.mode = sdk::proto::RobotMode::Teleop;   // matchTimeS stays -1
    s.tick(2);
    CHECK(s.game->match().phase == MatchPhase::Test);
    CHECK(s.game->hubActive(Alliance::Red));
    CHECK(s.game->hubActive(Alliance::Blue));

    sdk::SnapshotBuilder out;
    s.stateOwner->publish(out);
    const auto snap = out.build();
    CHECK(snap->hudValue(hud::kFuelCount) != nullptr);
    CHECK(snap->hudValue(hud::kMatchPhase) == nullptr);
    CHECK(snap->hudValue(hud::kMatchTime) == nullptr);
    CHECK(snap->hudValue(hud::kScoreRed) == nullptr);
    CHECK(snap->hudValue(hud::kHubRedActive) == nullptr);
}

TEST_CASE("the final score stays on the HUD after a timed match stops until reset, a new match or free play") {
    Session s;
    emptyField(s);
    s.start();
    autoEnabled(s);
    s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {});
    s.tick(2);
    const auto published = [&] {
        sdk::SnapshotBuilder out;
        s.stateOwner->publish(out);
        return out.build();
    };
    CHECK(published()->hudValue(hud::kScoreBlue)->asInt() == 1);

    s.env.driverStation.enabled = false;
    s.env.driverStation.matchTimeS = -1.0;   // the match ended
    s.tick(2);
    auto snap = published();
    REQUIRE(snap->hudValue(hud::kScoreBlue) != nullptr);
    CHECK(snap->hudValue(hud::kScoreBlue)->asInt() == 1);
    CHECK(snap->hudValue(hud::kMatchPhase)->asString() == "FINAL");
    CHECK(snap->hudValue(hud::kHubBlueStatus) == nullptr);

    s.env.driverStation.enabled = true;   // free play hides it again
    s.env.driverStation.mode = sdk::proto::RobotMode::Teleop;
    s.tick(2);
    CHECK(published()->hudValue(hud::kScoreBlue) == nullptr);

    s.env.driverStation.enabled = false;
    autoEnabled(s);   // a new match starts from zero
    s.tick(2);
    CHECK(published()->hudValue(hud::kScoreBlue)->asInt() == 0);
    s.env.driverStation.enabled = false;
    s.env.driverStation.matchTimeS = -1.0;
    s.tick(2);
    REQUIRE(published()->hudValue(hud::kScoreBlue) != nullptr);
    s.stateOwner->reset(s.env.ctx);
    CHECK(published()->hudValue(hud::kScoreBlue) == nullptr);
}

TEST_CASE("destroying the game state removes its fuel from the world") {
    Session s;
    s.start();
    const int withFuel = s.env.physics.stats().bodies;
    CHECK(withFuel == 1 + 1 + 408);  // field + chassis + staged fuel
    s.stateOwner.reset();
    s.game = nullptr;
    CHECK(s.env.physics.stats().bodies == 2);
    s.fieldOwner.reset();
    CHECK(s.env.physics.stats().bodies == 1);
}

TEST_CASE("publish exposes the fuel entity group and every HUD value the widgets bind to") {
    Session s;
    s.start();
    autoEnabled(s);
    s.env.driverStation.station = sdk::proto::AllianceStation::Blue2;
    s.tick(2);

    sdk::SnapshotBuilder out;
    s.stateOwner->publish(out);
    const auto snap = out.build();

    REQUIRE(snap->entityGroups().size() == 1);
    CHECK(snap->entityGroups()[0].group == "fuel");
    CHECK(snap->entityGroups()[0].positions.size() == 408);
    CHECK(snap->entityGroups()[0].radiusM == doctest::Approx(0.075f));

    CHECK(snap->hudValue(hud::kFuelCount)->asInt() == 8);
    CHECK(snap->hudValue(hud::kFuelCapacity)->asInt() == 30);
    CHECK(snap->hudValue(hud::kFuelOnField)->asInt() == 408);
    CHECK(snap->hudValue(hud::kHubBlueActive)->asBool());
    CHECK(snap->hudValue(hud::kHubOwnActive)->asBool());   // blue station => blue hub
    CHECK(snap->hudValue(hud::kMatchPhase)->asString() == "AUTO");
    // AUTO: both hubs stay active for the remainder of the period.
    CHECK(snap->hudValue(hud::kHubBlueStatus)->asString().rfind("Active ", 0) == 0);
    CHECK(snap->hudValue(hud::kHubRedStatus)->asString().rfind("Active ", 0) == 0);
    CHECK(snap->hudValue(hud::kFirstInactive)->asString() == "unknown");

    for (const sdk::HudWidgetDescriptor& w : s.module().contributeHud()) {
        CHECK_MESSAGE(snap->hudValue(w.valueKey) != nullptr, w.id);
        if (!w.maxKey.empty()) CHECK_MESSAGE(snap->hudValue(w.maxKey) != nullptr, w.id);
        for (const sdk::HudScoreSide& side : w.sides) {
            CHECK_MESSAGE(snap->hudValue(side.scoreKey) != nullptr, w.id);
            if (!side.statusKey.empty()) CHECK_MESSAGE(snap->hudValue(side.statusKey) != nullptr, w.id);
        }
    }
}
