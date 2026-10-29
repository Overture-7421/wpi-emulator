# Appendix B: where the work goes

These are actual destinations assembled by [Replay the book](appendix-replay.md), not a proposed
file layout. All paths are relative to a scratch source export. The tracked checkout stays
documentation-only.

| Destination | What you write there |
|---|---|
| <code>seasons\2027\game\CMakeLists.txt</code> | SDK-only static-library target |
| <code>seasons\2027\game\include\wpiemu\plugins\game_2027\plugin.h</code> | Public registration declaration |
| <code>seasons\2027\game\src\game_module.cpp</code> | All implementation fragments from field through registration workshops |
| <code>seasons\2027\season.json</code> | Existing season pin, with optional game enabled |
| <code>seasons\builtin_plugins.cpp</code> | Composition through the public registration function |
| <code>tests\plugins\test_power_up.cpp</code> | Five real-physics test cases |
| <code>tests\plugins\test_new_year_plugin.cpp</code> | Existing registry assertions updated for the extra built-in |
| <code>app\host\tests\e2e_season_test.cpp</code> | Opt-in real-HAL match, both languages and reset |
| <code>samples\season_2027\cpp\src\main\cpp\Robot.cpp</code> | Three-PWM C++ robot |
| <code>samples\season_2027\java\src\main\java\frc\robot\Main.java</code> | Equivalent Java robot |
| <code>samples\season_2027\cpp\.wpiemu\robot.json</code> | C++ fixture, low-Y start and RLR assignment |
| <code>samples\season_2027\java\.wpiemu\robot.json</code> | Java fixture, high-Y start and LRL assignment |

Within the single implementation file, `Bodies` owns collider lifetimes, `Field` owns static
geometry, `State` owns match-local pieces/rules, and `Module` supplies factories and descriptors.
They remain private to the translation unit. Once this prototype grows, those are natural seams
for private source files; inventing empty files for future vaults does not help the first build.

The template's [file list](../../seasons/_template/game/README.md) shows a more split-up skeleton.
The [2026 plugin](../../seasons/2026/game/README.md) is a mature reference for config, zones,
sensors, sounds and field layouts, not a Power Up source tree to rename.
