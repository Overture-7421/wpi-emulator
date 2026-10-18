# Writing a year plugin (new FRC season)

Core and UI are season-agnostic. A season is **one folder** under `plugins/` that implements
`sdk::IGameModule` and registers with one line. Adding 2027 changes nothing in `core/`, `app/`, the SDK or
another plugin - `tests/plugins/test_new_year_plugin.cpp` proves it by registering a complete extra season next to
2026 and driving it through the real config pipeline and simulation session.

Start from [`plugins/_template`](../plugins/_template/README.md). The finished example is
[`plugins/game_2026`](../plugins/game_2026) with its write-up in [game_2026.md](game_2026.md).

## 1. Checklist

| # | Step | Where |
|---|---|---|
| 1 | Copy `plugins/_template` to `plugins/game_<year>`; rename folder/include path, namespace `wpiemu::plugins::game_<year>`, target `wpiemu_plugin_game_<year>`, `pluginId()`, `gameYear()` | the new folder |
| 2 | `add_subdirectory(game_<year>)` + add the target to the `wpiemu_plugins_builtin` link line | `plugins/CMakeLists.txt` |
| 3 | `#include ".../game_<year>/plugin.h"` + `keepFirstError(plugins::game_<year>::registerPlugin(registry));` | `plugins/builtin_plugins.cpp` |
| 4 | Implement `IGameModule` (section 2) | `plugins/game_<year>/src/` |
| 5 | Write tests (section 4) | `tests/game_<year>/` |
| 6 | `python tools/check_boundaries.py`, then build + run `wpiemu_tests` | repo root, `docs/BUILDING.md` |
| 7 | Write `docs/game_<year>.md` with sources and simplifications | `docs/` |

The app selects the plugin from the project's GradleRIO year (`RobotConfig::project.wpilibYear`, or the explicit
`gamePluginId`); an unknown year runs with core features only. No `if (year == ...)` exists anywhere outside plugins.

## 2. The `IGameModule` contract, method by method

| Method | You provide | Notes |
|---|---|---|
| `metadata()` | `id` (`game_<year>`), `displayName`, `year`, `description`, `minSdkMinor` | `year` must be > 0 (only ids starting with `_` may use 0). Set `minSdkMinor` only if you need a newer SDK minor |
| `registerMechanisms(factory)` | season-specific mechanism templates | keys **must** be `"<id>.<name>"`. Prefer thin wrappers that read the user's generic mechanisms (roller, flywheel, ...) through `IMechanismDirectory` and their published `MechanismState` values over re-implementing physics (game_2026 has none: its pickup and shooter are zones, see `fuel_zones.h`) |
| `createField(world)` | static colliders + sensor volumes in the `IPhysicsWorld`, `FieldDescription` visuals | return an `IField` that owns them (RAII); it is destroyed before the world. Use `layer::Field`, `layer::Sensor`, `layer::Fuel`/own bits >= `layer::kPluginBase`; remember sensors need `sensorEvents` on **both** shapes and matching masks (ADR 0003) |
| `createGameState(ctx)` | the rules and entities for one session | `reset` restores the initial layout, `step` runs after the physics step, `publish` adds HUD values / entity groups to the snapshot |
| `contributeConfigSchema()` | `PluginConfigSchema{schemaVersion, PropertySchema, groupOrder}` | persisted under `RobotConfig::plugins["game_<year>"]`; rendered by the generic inspector; core validates it |
| `contributeEditorPanels()` / `contributeHud()` | Qt-free descriptors | `PanelScope::PluginConfig` or `Mechanism` (with viewport overlays); HUD widgets bind to `SnapshotBuilder::setHud` keys |
| `contributeZoneKinds()` | `std::vector<ZoneKindInfo>` | optional; kinds are `"<id>.<name>"`, persisted as `RobotConfig::zones` (ADR 0016); core validates ownership, attachment and properties |
| `onTick(ctx, dt)` | rarely needed | per-tick hook after `IGameState::step`; keep logic in the game state |

Per-tick order (docs/ARCHITECTURE.md §4): hub ingest -> mechanisms -> drivetrain -> physics step -> `IGameState::step`
-> `onTick` -> snapshot. Mechanisms therefore publish their state *before* the game state reads it.

## 3. Design rules

* **Dependencies**: `wpiemu/sdk/**` only (no core, app, ipc, bridges, Qt, Box3D, other plugin). One target, links
  `wpiemu_sdk`. Enforced by `tools/check_boundaries.py`.
* **No singletons or globals.** The module object is created at registration and must be stateless; session state
  belongs to the `IField` / `IGameState` it creates. Randomness only from `SimContext::rng()` (reproducible runs).
* **Errors are values** (`Status` / `Result`); no exception may cross an SDK interface.
* **Units** SI, frame Z-up NWU (+X to the opposing wall, +Y left); name scalar fields with unit suffixes.
* **Data over code**: put dimensions in constexpr tables with a source tag per number (see `field_dimensions.h`), build
  the field from a pure `FieldLayout` that tests can inspect without a physics world.
* **Keep the module a facade**; one header/source pair per concern.
* **Be honest**: write down what is approximated and why.

## 4. Testing a plugin

* Use the SDK fakes: `sdk/testing/fake_sim_environment.h` (physics, clock, hub, logger, config in one object),
  `fake_physics_world.h` (point-in-volume sensors, floor), `fake_mechanism_directory.h`. `tests/game_2026/test_support.h`
  wires module + field + mechanisms + game state like the runtime and can be copied.
* Cover, at least: metadata + registration (duplicate id, year lookup), template registration (twice -> error), schema
  defaults validate (`validateProperties(schema, defaultsOf(schema))`) and the ownership rule, field creation/destruction (`stats().bodies` back to the start),
  scoring / rules with edge cases (disabled robot, boundaries, caps), reset, and that every HUD key the widgets bind to
  is published.
* Integration (optional but valuable): link `wpiemu_core_physics` in the *test* (tests may link anything) and run the
  field against the real Box3D adapter - ray casts against the field surfaces and a sensor-scoring test catch
  geometry mistakes the fakes cannot (`tests/game_2026/test_box3d_integration.cpp`).
* Zero-core-change proof for your season: copy `tests/plugins/test_new_year_plugin.cpp`'s pattern - register next to the
  built-ins and run the real `ConfigPipeline` + `SimulationSession`.

## 5. Config schema version

`PluginConfigSchema::schemaVersion` is 1 until release (ADR 0013). The core refuses a persisted section with another
version (`VersionMismatch`); there are no migrators and no whole-config hook, so change your schema freely and
update the defaults, tests and docs together.
## 6. Distribution notes

v1 supports **static built-in** plugins only (one toolchain/CRT): the SDK exposes C++ virtual interfaces that are not a
stable ABI across separately built libraries (docs/ARCHITECTURE.md §4.7). A shared-library loader would need a versioned
`extern "C"` entry descriptor first; do not export C++ types across a DLL boundary.
