# plugins/_template - skeleton for a new season

A complete, compiling year plugin that does nothing useful yet: it registers, builds an (empty) field, steps a
game state, publishes one HUD value and passes the plugin contract tests. Copy it, rename it, fill it in. The full
guide is [docs/new-year-plugin.md](../../docs/new-year-plugin.md); the finished reference implementation is
[plugins/game_2026](../game_2026) ([docs/game_2026.md](../../docs/game_2026.md)).

## Add season 2027 in 5 steps

1. **Copy and rename.** `plugins/_template` -> `plugins/game_2027`, then rename
   * folder / include path: `include/wpiemu/plugins/_template` -> `include/wpiemu/plugins/game_2027`
   * namespace `wpiemu::plugins::_template` -> `wpiemu::plugins::game_2027` (all files)
   * CMake target `wpiemu_plugin_template` -> `wpiemu_plugin_game_2027` and the alias `wpiemu::plugin_template`
   * `pluginId()` -> `"game_2027"`, `gameYear()` -> `2027` (`src/plugin_info.cpp`)
2. **Register it** - two one-line edits, nothing else in the repository changes:
   * `plugins/CMakeLists.txt`: `add_subdirectory(game_2027)` and add `wpiemu_plugin_game_2027` to the
     `target_link_libraries(wpiemu_plugins_builtin ...)` line;
   * `plugins/builtin_plugins.cpp`: `#include "wpiemu/plugins/game_2027/plugin.h"` and
     `keepFirstError(plugins::game_2027::registerPlugin(registry));`
3. **Fill in `src/game_module.cpp`**, numbered like the contract: `metadata`, `registerMechanisms`, `createField`,
   `createGameState`, `contributeConfigSchema` (+ migrators), optional editor panels / HUD, `onTick`.
4. **Test it** under `tests/game_2027/` (doctest; they are globbed into `wpiemu_tests`). Use
   `sdk/testing/fake_sim_environment.h` + `fake_physics_world.h`; `tests/game_2026/test_support.h` has a ready-made
   session helper. Run `python tools/check_boundaries.py`.
5. **Document it** in `docs/game_<year>.md`: sources for every dimension and rule, and an honest simplification list.

## Rules that are enforced

| Rule | Enforced by |
|---|---|
| The plugin includes only `wpiemu/sdk/**` and links only `wpiemu_sdk` | `tools/check_boundaries.py` (CTest `boundaries.check`) |
| Core, app and other plugins never include a plugin; only `plugins/builtin_plugins.cpp` knows plugin targets | same |
| No Qt, no Box3D, no globals/singletons, no exceptions across SDK interfaces | review + `docs/ARCHITECTURE.md` |
| Mechanism template keys are `"<pluginId>.<name>"`; a plugin edits only its own config section and mechanisms | `sdk::ownsMechanismType` + the config pipeline |
| Per-session state lives in the `IField` / `IGameState` objects, never in the module | `IGameModule` contract |

This folder is built by default (`WPIEMU_BUILD_PLUGIN_TEMPLATE=ON`) so the skeleton cannot rot, but it is never part of
`wpiemu_plugins_builtin`: the application never loads it.

## Files

```
CMakeLists.txt                                      one STATIC target linking wpiemu_sdk only
include/wpiemu/plugins/_template/plugin.h           registerPlugin() - what the aggregation seam calls
include/wpiemu/plugins/_template/plugin_info.h      pluginId(), gameYear()
src/plugin_info.cpp                                 identity (the two TODO(new season) constants)
src/game_module.cpp                                 IGameModule facade + placeholder IField / IGameState
```

Grow the plugin the way `plugins/game_2026` did: one header + source pair per concern (field layout, game state,
config, UI descriptors, migration, mechanisms) and keep `game_module.cpp` a thin facade.
