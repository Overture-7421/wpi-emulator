# Kickoff: meet the machine

**In 30 seconds:** the robot process runs ordinary WPILib code; the desktop app owns simulation;
the plugin is the season-specific referee. You will build the 2027 preview profile before changing
anything, so later failures have somewhere to report.

## 1. Find the owners before opening an editor

Start at the [architecture](../ARCHITECTURE.md), then read the [codebase tour](../CODEBASE_TOUR.md)
and [game-plugin contract](../reference/game-plugin.md). The picture to keep is small:

- The robot sends motor commands through WPILib.
- HALSim translates those commands and receives simulated sensors.
- The runtime advances mechanisms and physics on its simulation thread.
- The game plugin updates game-specific rules after the physics step.
- The UI reads immutable snapshots; it never reaches into a live physics world.

That last point is why this book does not ask you to add a Qt widget for a score counter. The plugin
describes HUD values, and the existing run view draws them.

**Student asks:** “So is the plugin the robot code?” No. The robot still decides what to do. The
plugin decides what the field does when the robot does it. One is the driver; the other is the
referee with a clipboard.

## 2. Open the right season

The current 2027 season pack declares `systemcore`, the `gradlerio-modern` launch profile, Java 25,
and WPILib `2027.0.0-alpha-7`. It has no game plugin today. That is deliberate: ABI support and game
rules are separate features. A project can use a new WPILib year without the emulator pretending to
know that year's game.

For the full Windows/Ninja setup, follow [Build and test](../BUILDING.md). The 2027 profile requires
the current Visual Studio 2026 toolchain for C++23; use the WPILib-bundled JDK 25 for Java. To build
the Qt-free core and plugin targets in a separate tree:

```powershell
cmake -S . -B build\power-up-book -G "Visual Studio 18 2026" -A x64 `
  -DWPIEMU_BUILD_APP=OFF -DWPIEMU_BUILD_TESTS=ON `
  -DWPIEMU_WPILIB_SEASON=2027 -DWPIEMU_WPILIB_VERSION=2027.0.0-alpha-7 `
  -DWPIEMU_WPILIB_DOWNLOAD=ON -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build build\power-up-book --config Release --parallel
ctest --test-dir build\power-up-book -C Release -R "new season|new plugin|template plugin" --output-on-failure
```

The `--config Release` option matters for Visual Studio generators. For a Qt app build, use the Qt
kit and commands in [Building](../BUILDING.md); do not turn off Qt checks and then claim the 3D view
was tested.

**Quick win:** a clean build plus the template tests tells you the checkout, compiler, SDK, and plugin
contract agree before Power UP enters the conversation. If this baseline fails, fix the baseline
first; season code is innocent until proven otherwise.

## 3. Make a scratch pit

Experiments belong under the git-ignored `spikes\` directory, not in generated files under `build\`
and not in the live sample projects. For a reproducible baseline, export the current tracked source
to `spikes\pu`, then configure a distinct build directory there. Keep the WPILib version and the
compiler in your notes; “latest” is not a version number.

Do not point two Ninja build trees at the same FetchContent build directory. Share only the cached
dependency **sources**, or give each build its own `_deps` folder. Sharing configured build output is
how one clean build turns into an archaeology exercise.

## 4. Copy the season template, not the 2026 game

Read the [game-plugin guide](../guides/game-plugins.md) and the [season template checklist](../../seasons/_template/game/README.md).
Copy the template game into <code>seasons/2027/game</code> in the scratch export. Rename its target, alias,
include directory, namespace, and plugin id consistently to `game_2027`. Set the metadata year to
2027. Keep the target's only dependency `wpiemu_sdk`.

Make these changes in this order:

1. In `plugin_info.cpp`, return `game_2027` and year `2027`; keep the ID in one place.
2. In `game_module.cpp`, set `GameMetadata`, create the field and game state, and publish plugin-
   prefixed HUD, config, zone, game-value, and sound descriptors.
3. In `seasons/2027/season.json`, set the game entry to plugin `game_2027`; preserve its WPILib,
   ABI, and launch-profile data.
4. In `seasons/CMakeLists.txt`, add the plugin target to the season aggregation. In
   `seasons/builtin_plugins.cpp`, include its public entry point and call `registerPlugin`.
5. Add a focused registration/session test; do not make the app link directly to the new target.

The minimal plugin has three owners: a stateless `IGameModule` describes the game and constructs
per-session objects; an `IField` owns static bodies and their lifetime; an `IGameState` owns one
match's rules, timers, and game pieces.

Do not store cubes or score in the module. The module can outlive a session; a match cannot.

The app links the aggregator, never the individual year plugin. Run the season and boundary checks
before moving on:

```powershell
python tools\check_seasons.py
python tools\check_boundaries.py
```

**Inspection:** if you catch yourself adding `if (year == 2027)` to core or the app, stop. The
manifest and registry are the season switch. A year-specific branch outside the season pack is a
future cleanup ticket pretending to be architecture.

## You now have…

A baseline, a private scratch pit, and a clear ownership map. The plugin can be selected without a
year-specific change in core. Next, we pour a carpet that the physics engine and the run view agree
on.

Next: [Week 1—field coordinates and the first static geometry](02-field-and-physics.md).
