# Appendix B: where the work goes

The paths below are the scratch implementation's destination map. Some Power UP-specific source
files do not exist in the repository today; create them only in the season plugin you add.

| Work | Destination | Owner |
|---|---|---|
| Year, ABI, WPILib pin, plugin ID | <code>seasons/2027/season.json</code> | Season facts; no core year branch |
| Target and SDK-only dependency | <code>seasons/2027/game/CMakeLists.txt</code> | Plugin compilation |
| Registration declaration and identity | <code>game/include/.../plugin.h</code>, <code>plugin_info.h</code> | Public plugin entry point |
| Module, factories, and descriptors | <code>game/src/game_module.cpp</code> | Stateless description and session setup |
| Field geometry and visuals | <code>game/src/field_layout.cpp</code> | Measured colliders and render poses |
| Cube pool and entity IDs | <code>game/src/power_cubes.cpp</code> | Dynamic pieces and reset |

| Work | Destination | Owner |
|---|---|---|
| Switch/scale plate rules | <code>game/src/plate_rules.cpp</code> | Ownership bands and plate targets |
| Score, clock, and power-ups | <code>game/src/game_state.cpp</code>, <code>scoring.cpp</code> | Match-local rule state |
| Robot zones and inventory | <code>game/src/zones.cpp</code> | Existing SDK interaction seams |
| Config and property bindings | <code>game/src/config.cpp</code> | Plugin-owned settings |
| HUD, values, and sounds | <code>game/src/game_ui.cpp</code>, <code>game/src/game_sounds.cpp</code> | Qt-free descriptors |
| Linking and rule tests | <code>seasons/CMakeLists.txt</code>, <code>seasons/builtin_plugins.cpp</code>, <code>tests/plugins/</code> | Aggregation and contract checks |

Use the real [template file list](../../seasons/_template/game/README.md) for the minimum plugin
skeleton. The existing [2026 plugin](../../seasons/2026/game/README.md) demonstrates mature config,
zones, sensors, sounds, and field layouts, but its game rules and ABI are not Power UP examples.
Treat it as a seam reference, not a source tree to rename.
