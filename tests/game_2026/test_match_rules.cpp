#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2026/hub_rules.h"
#include "wpiemu/plugins/game_2026/match_clock.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;

namespace {

sdk::proto::DsState ds(sdk::proto::RobotMode mode, bool enabled, double matchTimeS = -1.0) {
    sdk::proto::DsState s;
    s.mode = mode;
    s.enabled = enabled;
    s.matchTimeS = matchTimeS;
    return s;
}

// Runs `seconds` of sim at 5 ms steps with a constant DS state.
MatchStatus run(MatchClock& clock, const sdk::proto::DsState& state, double seconds) {
    MatchStatus last;
    const int steps = static_cast<int>(seconds / 0.005 + 0.5);
    for (int i = 0; i < steps; ++i) last = clock.update(state, 0.005);
    return last;
}

}  // namespace

TEST_CASE("teleop phase table matches manual Table 6-2") {
    CHECK(teleopStatusAt(0.0).phase == MatchPhase::Transition);
    CHECK(teleopStatusAt(9.99).phase == MatchPhase::Transition);
    CHECK(teleopStatusAt(10.0).phase == MatchPhase::Shift1);
    CHECK(teleopStatusAt(34.9).phase == MatchPhase::Shift1);
    CHECK(teleopStatusAt(35.0).phase == MatchPhase::Shift2);
    CHECK(teleopStatusAt(60.0).phase == MatchPhase::Shift3);
    CHECK(teleopStatusAt(85.0).phase == MatchPhase::Shift4);
    CHECK(teleopStatusAt(110.0).phase == MatchPhase::EndGame);
    CHECK(teleopStatusAt(139.99).phase == MatchPhase::EndGame);
    CHECK(teleopStatusAt(140.0).phase == MatchPhase::PostMatch);
    CHECK(teleopStatusAt(500.0).phase == MatchPhase::PostMatch);   // clamped
    CHECK(teleopStatusAt(-3.0).phase == MatchPhase::Transition);   // clamped

    const MatchStatus s = teleopStatusAt(12.0);
    CHECK(s.phaseRemainingS == doctest::Approx(23.0));
    CHECK(s.periodRemainingS == doctest::Approx(128.0));
    CHECK(s.periodElapsedS == doctest::Approx(12.0));
}

TEST_CASE("hub schedule alternates after the auto winner's hub goes inactive first (Table 6-3)") {
    // red scored more in auto -> red inactive in shifts 1 and 3
    const Alliance firstInactive = Alliance::Red;
    for (const MatchPhase p : {MatchPhase::Auto, MatchPhase::Transition, MatchPhase::EndGame}) {
        CHECK(hubActiveInPhase(Alliance::Red, p, firstInactive));
        CHECK(hubActiveInPhase(Alliance::Blue, p, firstInactive));
    }
    CHECK_FALSE(hubActiveInPhase(Alliance::Red, MatchPhase::Shift1, firstInactive));
    CHECK(hubActiveInPhase(Alliance::Blue, MatchPhase::Shift1, firstInactive));
    CHECK(hubActiveInPhase(Alliance::Red, MatchPhase::Shift2, firstInactive));
    CHECK_FALSE(hubActiveInPhase(Alliance::Blue, MatchPhase::Shift2, firstInactive));
    CHECK_FALSE(hubActiveInPhase(Alliance::Red, MatchPhase::Shift3, firstInactive));
    CHECK(hubActiveInPhase(Alliance::Red, MatchPhase::Shift4, firstInactive));
    // blue won: mirrored
    CHECK(hubActiveInPhase(Alliance::Red, MatchPhase::Shift1, Alliance::Blue));
    CHECK_FALSE(hubActiveInPhase(Alliance::Blue, MatchPhase::Shift1, Alliance::Blue));
    // not scoring outside the match
    for (const MatchPhase p : {MatchPhase::PreMatch, MatchPhase::AutoEnded, MatchPhase::PostMatch})
        CHECK_FALSE(hubActiveInPhase(Alliance::Red, p, firstInactive));
}

TEST_CASE("match clock integrates enabled time per mode") {
    MatchClock clock;
    CHECK(clock.update(ds(sdk::proto::RobotMode::Teleop, false), 0.005).phase == MatchPhase::PreMatch);

    MatchStatus s = run(clock, ds(sdk::proto::RobotMode::Autonomous, true), 5.0);
    CHECK(s.phase == MatchPhase::Auto);
    CHECK(s.running);
    CHECK(s.autoPeriod);
    CHECK(s.periodRemainingS == doctest::Approx(15.0).epsilon(0.01));

    s = run(clock, ds(sdk::proto::RobotMode::Autonomous, true), 20.0);  // overshoot: auto is capped at 20 s
    CHECK(s.phase == MatchPhase::AutoEnded);
    CHECK_FALSE(s.running);
    CHECK(s.autoPeriod);

    s = run(clock, ds(sdk::proto::RobotMode::Teleop, false), 3.0);  // disabled gap: nothing advances
    CHECK(s.phase == MatchPhase::AutoEnded);

    s = run(clock, ds(sdk::proto::RobotMode::Teleop, true), 12.0);
    CHECK(s.phase == MatchPhase::Shift1);
    CHECK(s.running);
    CHECK_FALSE(s.autoPeriod);
    CHECK(s.teleopStarted);

    s = run(clock, ds(sdk::proto::RobotMode::Teleop, false), 50.0);  // disabled mid-match: frozen, not running
    CHECK(s.phase == MatchPhase::Shift1);
    CHECK_FALSE(s.running);

    s = run(clock, ds(sdk::proto::RobotMode::Teleop, true), 200.0);
    CHECK(s.phase == MatchPhase::PostMatch);
    CHECK_FALSE(s.running);

    clock.reset();
    CHECK(clock.update(ds(sdk::proto::RobotMode::Teleop, false), 0.005).phase == MatchPhase::PreMatch);
}

TEST_CASE("DS match time (WPILib countdown) overrides the integrated time") {
    MatchClock clock;
    MatchStatus s = clock.update(ds(sdk::proto::RobotMode::Teleop, true, 125.0), 0.005);  // 15 s elapsed
    CHECK(s.phase == MatchPhase::Shift1);
    CHECK(s.periodElapsedS == doctest::Approx(15.0));
    s = clock.update(ds(sdk::proto::RobotMode::Teleop, true, 25.0), 0.005);  // 115 s elapsed
    CHECK(s.phase == MatchPhase::EndGame);
    s = clock.update(ds(sdk::proto::RobotMode::Teleop, true, 999.0), 0.005);  // garbage clamps to the start
    CHECK(s.phase == MatchPhase::Transition);
    s = clock.update(ds(sdk::proto::RobotMode::Teleop, true, 0.0), 0.005);
    CHECK(s.phase == MatchPhase::PostMatch);
}

TEST_CASE("test mode is a practice mode: running with both hubs active, no match structure") {
    MatchClock clock;
    const MatchStatus s = clock.update(ds(sdk::proto::RobotMode::Test, true), 0.005);
    CHECK(s.phase == MatchPhase::Test);
    CHECK(s.running);
    CHECK(hubActiveInPhase(Alliance::Red, s.phase, Alliance::Red));
    CHECK(hubActiveInPhase(Alliance::Blue, s.phase, Alliance::Blue));
}

TEST_CASE("hub rules: points, inactive fuel, grace period and the auto-winner latch") {
    sdk::Rng rng(42);
    HubRules hubs;
    MatchClock clock;
    const auto autoDs = ds(sdk::proto::RobotMode::Autonomous, true);

    MatchStatus st = clock.update(autoDs, 0.005);
    hubs.update(st, autoDs, 0.005, rng);
    CHECK(hubs.active(Alliance::Red));
    CHECK(hubs.active(Alliance::Blue));
    CHECK_FALSE(hubs.firstInactiveKnown());
    CHECK(hubs.recordFuel(Alliance::Red, st));
    CHECK(hubs.recordFuel(Alliance::Red, st));
    CHECK(hubs.recordFuel(Alliance::Blue, st));
    CHECK(hubs.score(Alliance::Red).autoFuel == 2);
    CHECK(hubs.score(Alliance::Red).points() == 2);

    // auto ends, robot disabled: hubs stop being active but fuel in flight still counts for 3 s
    const auto disabled = ds(sdk::proto::RobotMode::Autonomous, false);
    st = clock.update(disabled, 0.005);
    hubs.update(st, disabled, 0.005, rng);
    CHECK_FALSE(hubs.active(Alliance::Red));
    CHECK(hubs.scoringOpen(Alliance::Red));
    CHECK(hubs.recordFuel(Alliance::Red, st));
    CHECK(hubs.score(Alliance::Red).autoFuel == 3);  // attributed to AUTO
    for (int i = 0; i < 700; ++i) hubs.update(st, disabled, 0.005, rng);  // 3.5 s
    CHECK_FALSE(hubs.scoringOpen(Alliance::Red));
    CHECK_FALSE(hubs.recordFuel(Alliance::Red, st));
    CHECK(hubs.score(Alliance::Red).inactiveFuel == 1);

    // teleop starts: red won auto (3 > 1) -> red hub inactive in shift 1
    const auto teleop = ds(sdk::proto::RobotMode::Teleop, true);
    st = clock.update(teleop, 0.005);
    hubs.update(st, teleop, 0.005, rng);
    REQUIRE(hubs.firstInactiveKnown());
    CHECK(hubs.firstInactive() == Alliance::Red);
    CHECK(hubs.active(Alliance::Red));  // transition shift: both active

    MatchStatus later = clock.update(ds(sdk::proto::RobotMode::Teleop, true, 120.0), 0.005);  // 20 s: shift 1
    hubs.update(later, teleop, 0.005, rng);
    CHECK_FALSE(hubs.active(Alliance::Red));
    CHECK(hubs.active(Alliance::Blue));
    CHECK(hubs.scoringOpen(Alliance::Red));  // grace: it was active a moment ago
    CHECK(hubs.recordFuel(Alliance::Red, later));
    CHECK(hubs.score(Alliance::Red).teleopFuel == 1);

    // the latch does not change when scores change afterwards
    (void)hubs.recordFuel(Alliance::Blue, later);
    (void)hubs.recordFuel(Alliance::Blue, later);
    hubs.update(later, teleop, 0.005, rng);
    CHECK(hubs.firstInactive() == Alliance::Red);
}

TEST_CASE("first inactive hub: config override, FMS game data, tie break") {
    const auto teleop = ds(sdk::proto::RobotMode::Teleop, true);
    MatchClock clock;
    const MatchStatus st = clock.update(teleop, 0.005);
    sdk::Rng rng(1);

    HubRules forcedBlue(true, FirstInactiveMode::Blue);
    forcedBlue.update(st, teleop, 0.005, rng);
    CHECK(forcedBlue.firstInactive() == Alliance::Blue);

    sdk::proto::DsState withData = teleop;
    withData.gameSpecificMessage = "R";
    HubRules fromData;
    fromData.update(st, withData, 0.005, rng);
    CHECK(fromData.firstInactive() == Alliance::Red);
    withData.gameSpecificMessage = "B";
    HubRules fromDataBlue;
    fromDataBlue.update(st, withData, 0.005, rng);
    CHECK(fromDataBlue.firstInactive() == Alliance::Blue);

    withData.gameSpecificMessage = "garbage";  // ignored: falls back to the auto result (0:0 tie -> random)
    HubRules tie;
    sdk::Rng rngA(7), rngB(7);
    tie.update(st, withData, 0.005, rngA);
    HubRules tie2;
    tie2.update(st, withData, 0.005, rngB);
    CHECK(tie.firstInactiveKnown());
    CHECK(tie.firstInactive() == tie2.firstInactive());  // deterministic for a seed
}

TEST_CASE("disabling shift enforcement keeps both hubs active while running, never outside the match") {
    HubRules relaxed(false, FirstInactiveMode::FromAutoResult);
    sdk::Rng rng(1);
    MatchClock clock;
    const auto teleop = ds(sdk::proto::RobotMode::Teleop, true, 120.0);  // shift 1 by the clock
    MatchStatus st = clock.update(teleop, 0.005);
    relaxed.update(st, teleop, 0.005, rng);
    CHECK(relaxed.active(Alliance::Red));
    CHECK(relaxed.active(Alliance::Blue));

    const auto off = ds(sdk::proto::RobotMode::Teleop, false);
    st = clock.update(off, 0.005);
    relaxed.update(st, off, 0.005, rng);
    CHECK_FALSE(relaxed.active(Alliance::Red));
}
