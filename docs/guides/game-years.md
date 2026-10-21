# Bring up a new game year

Read this when adding a season without changing core simulation or adding season switches to UI.
Prerequisites: [Architecture](../ARCHITECTURE.md), [a working build](../BUILDING.md), and the
[game-plugin contract](../reference/game-plugin.md).

The initial goal is a selectable year with a field, resettable state and one observable HUD value.
Field fidelity, scoring rules and game-piece behavior come after that integration works.

## Copy the buildable skeleton

1. Copy [`plugins/_template`](../../plugins/_template) to a new season folder under `plugins`.
   Follow its [rename checklist](../../plugins/_template/README.md): include tree, namespace,
   library/alias, `pluginId()`, `gameYear()` and metadata must agree.
2. Use a plugin-prefixed identity for custom templates, zones, game values and HUD keys.
   Keep persisted plugin data in that plugin's `RobotConfig::plugins` section.
3. Retain SDK-only dependencies. The template's
   [`CMakeLists.txt`](../../plugins/_template/CMakeLists.txt) explicitly lists
   `src/plugin_info.cpp` and `src/game_module.cpp`, exports `include`, and links `wpiemu_sdk`.
   Add sources to your copied list as the implementation grows.

The template itself deliberately does not ship in the built-in set. Copying it or enabling its
build option is not enough to make your new season selectable.

## Establish the three owners

- The module implements `IGameModule` metadata, construction and descriptors; it holds no mutable
  session state.
- The `IField` owns field physics bodies/shapes and destroys them before its world is destroyed.
- The `IGameState` owns per-session rules, entities and timers, resets them, and publishes snapshots.
  Use context simulation time/RNG; do not introduce wall-clock season timers.

Read the complete miniature season in
[`test_new_year_plugin.cpp`](../../tests/plugins/test_new_year_plugin.cpp).
Its `Field`, `State` and `Module` are compiled alongside the real config pipeline and session.
This exact `State` excerpt increments one counter and makes it visible:

```cpp
void reset(SimContext&) override { steps = 0; }
void step(SimContext&, double) override { ++steps; }
void publish(SnapshotBuilder& out) const override { out.setHud("game_2998.steps", steps); }
```

This excerpt belongs inside that test's `State` class, with its `steps` member and includes.
The integration assertion expects HUD value `game_2998.steps` to equal five after five ticks.
For production error handling and richer field construction, follow
[`game_2026/game_module.cpp`](../../plugins/game_2026/src/game_module.cpp).

## Register through the plugin aggregator

1. Place `WPIEMU_REGISTER_GAME_PLUGIN` with your module type **inside its namespace**.
   The macro defines a `registerPlugin` function; declare that function in the public plugin header.
2. Add your season subdirectory in
   [`plugins/CMakeLists.txt`](../../plugins/CMakeLists.txt) and its library to
   `wpiemu_plugins_builtin`'s private links.
3. Include the public header and add its namespaced `registerPlugin` call in
   [`plugins/builtin_plugins.cpp`](../../plugins/builtin_plugins.cpp).
   Use the existing first-error aggregation pattern, not ignored registration results.
4. Link the new plugin into `wpiemu_tests` in
   [`tests/CMakeLists.txt`](../../tests/CMakeLists.txt) for direct plugin tests.
   The app continues linking only the aggregator.
5. Reconfigure and build using [Building](../BUILDING.md). The boundary checker enforces
   directory/target/aggregator wiring; it is not a substitute for constructing the season.

## Grow from bring-up to playable behavior

1. Add typed config using `ConfigBindings`, as in
   [`game_2026/config.cpp`](../../plugins/game_2026/src/config.cpp).
   Contribute the schema, decode/default/validate, then construct game state. The
   [property guide](editor-properties.md) explains shared keys, units and persistence checks.
2. Prefer generic mechanisms with [zones and game values](game-interactions.md).
   Register a plugin mechanism only when generic dynamics cannot express the part.
3. Return Qt-free HUD/editor descriptors and publish every HUD key they bind to. Numeric game
   values for sensors are a separate interface, not parsed HUD strings.
4. Supply `aprilTagLayout()` with tag poses and field dimensions for vision. Supply `fieldFrame()`
   with the correct symmetry for alliance-copy start poses. Neither feature is inferred from a mesh.
5. Source rule/dimension data at the implementation and document deliberate limitations in the
   [game-plugin reference](../reference/game-plugin.md), rather than turning this recipe into a
   season-specific rules manual.

## Run the bring-up checks

```powershell
ctest --preset windows-ninja-release -R "new season|new plugin|template plugin"
```

These cases verify duplicate registration rejection, year selection, config defaults, a constructed
field/game state/mechanism, and the five-step HUD result above. The copied skeleton also has its
own [template test](../../tests/plugins/test_template_plugin.cpp).
For your real season add field cleanup, repeated reset, rule boundaries, HUD/game-value publication,
and seeded determinism tests; a miniature integration test is not proof of rule fidelity.

If the year is absent, check both aggregator files and metadata year. If it loads but config is
ignored, inspect the selected plugin ID and section, not an app-side special case.

Next: [implement pickups, launchers and game-fed sensors](game-interactions.md).
