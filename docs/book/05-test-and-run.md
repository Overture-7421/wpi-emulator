# Week 6: bag and tag

**In 30 seconds:** a plugin that compiles is not yet a game. Register it through the season
aggregator, run rule tests with fixed time, then launch an ordinary robot project and inspect the
field in the app.

## 1. Wire the plugin once

In the scratch copy, set the 2027 manifest's optional game entry to `game_2027`, add the plugin
target to the season CMake aggregation, and call its `registerPlugin` function from the built-in
plugin aggregator. Do not add a direct dependency from the app to `game_2027`; the aggregator is
the composition seam.

Build with warnings-as-errors and run the existing season, plugin, and session tests. Then run:

```powershell
python tools\check_seasons.py
python tools\check_boundaries.py
python tools\check_docs.py
```

The season checker catches identity and manifest mistakes. The boundary checker catches forbidden
dependencies, such as a plugin that includes Qt or Box3D. Neither one proves the plates line up with
the field drawing.

## 2. Test rules at the edges

Write focused tests around the plugin's public SDK interfaces. At minimum, cover:

- registering exactly once and selecting the game from a 2027 project config;
- field construction and teardown, repeated reset, and two independent game-state instances;
- cube pickup/deposit, duplicate overlap events, and scoring once per simulation tick;
- auto versus teleop ownership rates, disabled behavior, and endgame boundaries;
- each of the four legal game-data strings, malformed input, and Reset;
- each power-up's cost, target, duration, and expiration;
- HUD and game-value publication, plus any sensor response the robot uses;
- the climber's success, failure, and reset paths.

Use a fake physics world for rule transitions. Use the real Box3D backend for contacts, ramps,
overlaps, and a climber whose success depends on support. Fix the simulation step and random seed.
Never use a sleep to make a deterministic simulation test “more realistic.”

The existing tests include a newly registered year plugin, a template plugin, zone activation,
game-fed sensors, and game-data delivery through the robot-facing state. Read those before adding
another fake; they show which behavior already belongs to core.

## 3. Run a robot like a driver would

Select the 2027 project in the app. Confirm that the plugin is selectable, the field and pieces
appear, Driver Station state changes scoring phase, and the robot sees the expected game data in
`autonomousInit`. Drive slowly first. Check that the plate visual and collision remain level and
coincident when ownership changes; verify reset returns every cube and counter.

The run view is a Qt Quick 3D scene, so an offscreen widget test does not prove the field renders.
If the book's Qt-free build was all you configured, you have not tested the picture. Use a matching
Qt kit and follow the app build instructions in [Build and test](../BUILDING.md), then inspect the
scene in the actual app.

For an end-to-end check, use a real-HAL build and the normal 2027 GradleRIO launcher with JDK 25.
An integration test must launch robot code, deliver game data before autonomous, step the match, and
observe the robot's response. A sample that compiles but never loads HALSim is not an end-to-end
test.

## 4. Existing 2027 assumptions are test cases, not obstacles

The repository currently treats 2027 as core-only. A real local plugin changes that scenario. In the
scratch copy, update the exact-count assumption in
[`test_new_year_plugin.cpp`](../../tests/plugins/test_new_year_plugin.cpp) and the core-only
expectation in [`e2e_season_test.cpp`](../../app/host/tests/e2e_season_test.cpp). Revisit the
statements in the [2027 season README](../../seasons/2027/README.md),
[user guide](../USER_GUIDE.md), [season reference](../reference/seasons.md), and
[2027 sample README](../../samples/season_2027/README.md). Do not weaken the test or bake the
stand-in into tracked code. Keep the platform-only 2027 behavior covered as a separate
configuration if that remains a supported setup.

## You now have…

A plugin the app can select, a test plan that covers reset and scoring edges, and a manual run
checklist. A successful build proves the code links. A successful match proves the pieces, clock,
robot, and referee can have a conversation.

Next: [Next kickoff—reuse the seam for the actual game](06-next-kickoff.md).
