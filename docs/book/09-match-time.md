# 9. Match time and scoring windows

## 9.1 Objectives

After this chapter the reader can classify Driver Station state, distinguish a
scoring phase from an enabled test mode, and test exact time boundaries. Chapter
8 is the prerequisite. The clock classifier and tests address R7.

## 9.2 Problem statement

A 15-second autonomous timer inside the plugin would disagree with a Driver
Station configured for a shorter practice period. It would also keep running
after a disable unless every scoring path remembered to check another flag.
The plugin therefore classifies the current Driver Station state once.

## 9.3 Background and definitions

**Definition 9.1 (scoring window).** A scoring window is an enabled autonomous
or teleoperated interval with no emergency stop and a nonzero remaining time.
The value -1 denotes untimed operation. Test mode can enable mechanisms
without belonging to a scoring window.

Endgame is the final 30 seconds of teleoperation, not a third Driver Station
mode. Untimed teleoperation has no defined final 30 seconds and is classified
as Teleop. The clock classifier has no mutable timer.

## 9.4 Design

One design reconstructs a match from elapsed simulation time. Another treats
the Driver Station as authoritative and uses elapsed simulation time only to
measure durations within an active window. The second design supports practice
settings and disabled intervals without inventing a parallel match controller.

**Invariant 9.1 (freeze).** Disabled, emergency-stop and zero-remaining-time
inputs cannot increase a match score. Physics may continue to settle while
points remain unchanged.

## 9.5 Implementation

`MatchPhase` gives the human-readable phases a typed representation.
`phaseOf` checks disabling conditions before mode; `scoresIn` excludes Test.
`phaseName` supplies publication text without making strings control the rules.

Listing 9.1. Phase classification interface. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\match_clock.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\match_clock.h -->
```cpp
#pragma once

#include "wpiemu/sdk/ipc/driver_station.h"

namespace wpiemu::plugins::game_2027 {

enum class MatchPhase { Disabled, Autonomous, Teleop, Endgame, Test };
MatchPhase phaseOf(const sdk::proto::DsState& ds);
const char* phaseName(MatchPhase phase);
bool scoresIn(MatchPhase phase);

}  // namespace wpiemu::plugins::game_2027
```

Listing 9.2. Driver Station classification. <code>seasons\2027\game\src\match_clock.cpp</code>.

<!-- book-file: seasons\2027\game\src\match_clock.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/match_clock.h"

namespace wpiemu::plugins::game_2027 {

MatchPhase phaseOf(const sdk::proto::DsState& ds) {
    if (!ds.enabled || ds.eStop || ds.matchTimeS == 0.0) return MatchPhase::Disabled;
    if (ds.mode == sdk::proto::RobotMode::Autonomous) return MatchPhase::Autonomous;
    if (ds.mode == sdk::proto::RobotMode::Test) return MatchPhase::Test;
    return ds.matchTimeS >= 0.0 && ds.matchTimeS <= 30.0 ? MatchPhase::Endgame : MatchPhase::Teleop;
}

const char* phaseName(MatchPhase phase) {
    switch (phase) {
        case MatchPhase::Disabled:
            return "Disabled";
        case MatchPhase::Autonomous:
            return "Autonomous";
        case MatchPhase::Teleop:
            return "Teleop";
        case MatchPhase::Endgame:
            return "Endgame";
        case MatchPhase::Test:
            return "Test";
    }
    return "Disabled";
}

bool scoresIn(MatchPhase phase) {
    return phase == MatchPhase::Autonomous || phase == MatchPhase::Teleop ||
           phase == MatchPhase::Endgame;
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 9.3. Library sources through Chapter 9. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp
    src/field_layout.cpp
    src/cube_layout.cpp
    src/cube_pool.cpp
    src/plate_build.cpp
    src/plate_rules.cpp
    src/plates.cpp
    src/match_clock.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

The initial assembled game in Chapter 12 integrates ownership at two points
per second in autonomous and one in teleoperation. That continuous baseline
helps isolate physical delivery from score-accounting transitions. Chapter 15
replaces it with a one-second acquisition test and a documented rounding
policy. The baseline is not
classified as official scoring.

## 9.6 Verification

The boundary table checks 30.001 s, 30 s, -1 s and 0 s, as well as disabled,
emergency-stop and Test states. The oracle is Definition 9.1, so a change in
implementation branch order cannot redefine the expected result.

Listing 9.4. Scoring-window boundary tests. <code>tests\game_2027\test_match_clock.cpp</code>.

<!-- book-file: tests\game_2027\test_match_clock.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/match_clock.h"

namespace pu = wpiemu::plugins::game_2027;
namespace s = wpiemu::sdk;

TEST_CASE("Power UP: Driver Station state defines every scoring boundary") {
    s::proto::DsState ds;
    CHECK(pu::phaseOf(ds) == pu::MatchPhase::Disabled);
    ds.enabled = true;
    ds.mode = s::proto::RobotMode::Autonomous;
    ds.matchTimeS = 15.0;
    CHECK(pu::phaseOf(ds) == pu::MatchPhase::Autonomous);
    ds.mode = s::proto::RobotMode::Teleop;
    ds.matchTimeS = 30.001;
    CHECK(pu::phaseOf(ds) == pu::MatchPhase::Teleop);
    ds.matchTimeS = 30.0;
    CHECK(pu::phaseOf(ds) == pu::MatchPhase::Endgame);
    ds.matchTimeS = -1.0;
    CHECK(pu::phaseOf(ds) == pu::MatchPhase::Teleop);
    ds.matchTimeS = 0.0;
    CHECK_FALSE(pu::scoresIn(pu::phaseOf(ds)));
    ds.matchTimeS = 1.0;
    ds.eStop = true;
    CHECK_FALSE(pu::scoresIn(pu::phaseOf(ds)));
    ds.eStop = false;
    ds.mode = s::proto::RobotMode::Test;
    CHECK_FALSE(pu::scoresIn(pu::phaseOf(ds)));
}
```

## 9.7 Reproduction

Extract through Chapter 9 and run:

```powershell
ctest --test-dir build\book -R "Power UP: Driver Station" --no-tests=error --output-on-failure
```

## 9.8 Discussion

F-06 describes match timing and F-09 distinguishes the baseline from final
accounting. A countdown supplies the current phase; integer elapsed intervals
inside a rule state machine supply dwell and power-up durations.

## 9.9 Summary

The plugin has one definition of a scoring window. No game timer competes
with the Driver Station. The later scoring modules can rely on a tested phase
classification.

## 9.10 Exercises

1. State the expected phase for enabled Test mode with 10 s remaining.
2. Extend the test with disabled autonomous and untimed autonomous.
3. Explain why physics need not stop when Invariant 9.1 freezes scoring.

[Chapter 10](10-interaction.md)
