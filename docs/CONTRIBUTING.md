# Contributing

Read [ARCHITECTURE.md](ARCHITECTURE.md) first (modules, dependency rule, threading, per-tick data flow) and
[BUILDING.md](BUILDING.md) (build, test, options). Decisions live in [decisions/](decisions/README.md); a change
that crosses a module boundary or the SDK needs an ADR (see [Changing the SDK](#changing-the-sdk)).

## Before you push

```powershell
python tools/check_boundaries.py                       # module dependency rules (also CTest boundaries.check)
cmake --preset windows-ninja -DCMAKE_PREFIX_PATH=C:/Qt/6.8.3/msvc2022_64 -DWPIEMU_WARNINGS_AS_ERRORS=ON   # in a vcvars64 shell
cmake --build --preset windows-ninja-release
ctest --preset windows-ninja-release
```

Use the preset of your platform (BUILDING.md). First-party code builds at `/W4` (MSVC) or `-Wall -Wextra -Wpedantic`;
the tree is clean with `-DWPIEMU_WARNINGS_AS_ERRORS=ON` (the CI workflow does not set it yet), so use it locally to keep
it that way.
Do not commit build output, per-project `.wpiemu/` folders, screenshots or logs (`.gitignore` covers `build/`, `out/`,
`.wpiemu/`, `spikes/`, IDE folders and `*.log`). Experiments belong in an ignored `spikes/` folder.

## Module map and the boundary check

| Module | Depends on | Target |
|---|---|---|
| `sdk/` | std only | `wpiemu_sdk` (header-only) |
| `core/` (`src`, `physics`, `mechanisms`, `runtime`) | sdk, Box3D (private) | `wpiemu_core` |
| `ipc/` | sdk | `wpiemu_ipc` |
| `halsim_ext/` | sdk, ipc, the bridges aggregator, WPILib HAL | `halsim_wpiemu` |
| `bridges/hal`, `bridges/vendor` | sdk (not each other) | `wpiemu_bridges_hal`, `wpiemu_bridges_vendor` |
| `bridges/` (aggregator) | sdk + every bridge module | `wpiemu_bridges_builtin` (only `halsim_ext` links it) |
| `plugins/<name>` | sdk only | `wpiemu_plugin_<name>` |
| `plugins/` (aggregator) | sdk + every plugin | `wpiemu_plugins_builtin` (only `app` links it) |
| `app/` | sdk, core, ipc, plugins aggregator, Qt | `wpiemu_app` and its static libs |

`tools/check_boundaries.py` scans `#include` directives and `target_link_libraries` calls and fails on a rule
violation (plus headers that use `signals`, `slots`, `emit` or `foreach` as identifiers, because Qt turns them into
macros). `python tools/check_boundaries.py --self-test` tests the checker. Public headers live at
`<module>/include/wpiemu/<module>/...` (`wpiemu/bridges/<hal|vendor>/...`, `wpiemu/plugins/<name>/...`); target
names follow `wpiemu_<module>`. If you need a new dependency, you probably need a new SDK interface instead.
Implementation subtargets and aliases must have entries in `TARGET_OWNERS`; unknown first-party
dependencies and missing/incorrect declaration ownership fail the check. The plugin and bridge
aggregators have separate owners: app can link only the plugin seam, extension only the bridge seam.
The imported WPILib targets are robot-process backends available to the extension, not public SDK edges.

## Coding conventions

* C++20, no compiler extensions. `.clang-format` (Google-based, 4 spaces) is the reference style; keep new code
  consistent with the surrounding file.
* **Errors are values**: `sdk::Result<T>` / `sdk::Status`. No exception may cross an SDK interface; catch at the edge
  of third-party code (JSON, Qt, regex) and convert. Contract violations abort via `contractViolation`.
* **No singletons or globals**; ownership with `std::unique_ptr`, collaborators by constructor injection or
  `SimContext`. Composition roots (`main()`, `AppServices`, tests) create everything.
* **Units**: SI everywhere internally, right-handed Z-up NWU frame (+X toward the opposing wall, +Y left, +Z up),
  quaternions (x,y,z,w). Suffix plain scalars with units (`massKg`, `radiusM`, `timeUs`). Sim time is integer
  microseconds from `ISimClock`; never read a wall clock in simulation code. Randomness only from `SimContext::rng()`.
* **Threads**: the sim thread owns physics, mechanisms, hub and game state; the UI reads immutable snapshots and sends
  commands through `ISimController`; never block in a HAL callback (ARCHITECTURE section 3).
* **UI (`app/`)**: MVVM (widgets talk to view-models; only view-models touch the config repository or the runtime),
  user-visible strings through `tr()` (`ETR()` in the editor model code), colours from `ThemeTokens`, display units
  through `UnitFormatter`. No Qt in `sdk`, `core`, `ipc`, bridges or plugins.
* **Wire protocol / SDK**: the SDK stays at version 1 until release (ADR 0013): no compatibility shims, migrations or
  version bumps; keep the protocol layout and its tests consistent when you change it.
* Comment only what needs clarifying. Prefer small header/source pairs per concern.

## Tests

* Framework: doctest. Every `tests/**/*.cpp` is globbed into `wpiemu_tests` (re-run CMake after adding a file); each
  `TEST_CASE` becomes one CTest test via `doctest_discover_tests`.
* **Test case names must not contain `;`**: CTest splits the discovery list on it and registers bogus half-named
  tests. Use `,` or `-` in names (`TEST_CASE("hub: one device, one slot, rebinding is a no-op")`). Keep names unique.
* Layout mirrors the source: `tests/sdk`, `tests/ipc`, `tests/physics`, `tests/mechanisms`, `tests/runtime`,
  `tests/bridges_hal`, `tests/bridges_vendor`, `tests/game_2026`, `tests/plugins`. Header-only fakes for isolated tests
  are in `sdk/include/wpiemu/sdk/testing/`.
* Qt suites have their own executables (they need an event loop and run with `QT_QPA_PLATFORM=offscreen`):
  `app/tests` (`app.ui_tests`), `tests/app_editor` (`app_editor.tests`, files are `*.cc` so the Qt-free glob does not
  pick them up), `tests/app_runview` (`runview.ui_tests`), `app/host/tests` (`host.*`), `halsim_ext/tests`.
  Offscreen cannot render Qt Quick 3D; check visuals with the demo executables (`wpiemu_editor_demo`,
  `wpiemu_runview_demo`).
* New SDK headers need no test wiring: `tests/sdk` generates a one-TU-per-header self-contained-include check.
* The real-robot end-to-end test (`e2e.shelby_2026`) is opt-in, see BUILDING.md.
* Add a regression test with every bug fix; test through the public interface with fakes first, add a Box3D
  integration test only for geometry that fakes cannot judge.

## Recipes

### Add a year plugin
The checklist with method-by-method contract is [new-year-plugin.md](new-year-plugin.md); the skeleton is
[plugins/_template](../plugins/_template/README.md); the worked example is [game_2026.md](game_2026.md). In short:

1. Copy `plugins/_template` to `plugins/game_<year>`; rename the include path, namespace
   `wpiemu::plugins::game_<year>`, target `wpiemu_plugin_game_<year>`, `pluginId()` and `gameYear()`. The target links
   only `wpiemu_sdk`.
2. Implement `IGameModule` (`metadata`, `registerMechanisms` with keys `"game_<year>.<name>"`, `createField`,
   `createGameState`, `contributeConfigSchema`, optional panels/HUD) and end the file with
   `WPIEMU_REGISTER_GAME_PLUGIN(...)`.
3. Register: `add_subdirectory(game_<year>)` and the link line in `plugins/CMakeLists.txt`; one `registerPlugin(registry)`
   call in `plugins/builtin_plugins.cpp`. Nothing else in the repository changes.
4. Tests in `tests/game_<year>/` (and run `tests/plugins/test_new_year_plugin.cpp`'s pattern for the zero-core-change
   proof), then `python tools/check_boundaries.py`.
5. Write `docs/game_<year>.md` with sources for every number and an honest simplification list.

### Add a device bridge (new device family)
1. Create `bridges/<name>/` linking `wpiemu_sdk` only (plus vendor/HAL headers there), public headers at
   `include/wpiemu/bridges/<name>/`. Add `<name>` to `ALLOWED`/`MODULE_OWNERS`/`TARGET_OWNERS` in
   `tools/check_boundaries.py`, including the bridge aggregator's outgoing edge, and to
   ADR 0004's table (rule changes require amending that ADR). Do not grant the extension
   a direct dependency on the new module.
2. Choose the base: a `SimDeviceBridge` with a `std::vector<SimDeviceRule>` table (name regex, bindings to canonical
   signals with `scale`/`offset`), a `ChannelFamilyBridge` for per-index HAL families, or a plain `IDeviceBridge`.
   Signal names and units are in `sdk/devices/signals.h`; capabilities decide which slot types accept the device.
   Never invent a CAN bus label (`busIsStatic = false`).
3. Expose `registerXBridges(sdk::DeviceBridgeRegistry&)` and wire it into the aggregation seam:
   `bridges/CMakeLists.txt` (`add_subdirectory`, link into `wpiemu_bridges_builtin`) and one call in
   `bridges/builtin_bridges.cpp`. The extension itself does not change.
4. Test with the fakes (`FakeSimBackend`-style fixtures, `tests/bridges_vendor/fakes.h`, `testing/fake_hal_api.h`) and
   document the capability table in [bridges-hal.md](bridges-hal.md) or [bridges-vendor.md](bridges-vendor.md).

### Add a vendor (REV, NavX, ...) to the vendor bridge
Follow "Extension recipe: adding a vendor" in [bridges-vendor.md](bridges-vendor.md): discover the real SimDevice
names from the vendor wrapper, write a rule table (`src/<vendor>_rules.cpp`), add guarded values only for values the
vendor firmware integrates, create the bridge in `SharedSetup` and add one `registerBridge` call in
`vendor_bridges.cpp`, test with the fakes, document the table. REV is not shipped in v1.

### Add a mechanism template
1. Implement `IMechanism` (constructor takes `MechanismBuildContext`) and declare `MechanismTemplateInfo`: `type`, a
   display name and category, `slotSpecs`, a `PropertySchema`, default primitives and optional attachable `links`.
   Talk to devices only through slots (`ctx.devices()`), integrate motion only through plants (`IPlant`), push on the
   chassis only through `ctx.coupling()`, and publish moving links and values in `MechanismState`.
2. Register with `MechanismFactory::registerTemplate`: generic templates in `core/mechanisms`
   (`registerGenericMechanisms`, see [mechanisms.md](mechanisms.md)); season-specific ones in the plugin's
   `registerMechanisms`, keys prefixed with the plugin id.
3. The editor builds its context-menu entry, inspector and slot panel from that metadata: no UI change.
4. Test with `FakeSimEnvironment` + `FakeDeviceHub::autoBind` (`tests/mechanisms`).

### Add a zone kind (game plugin)
1. In the plugin's `IGameModule::contributeZoneKinds()` return a `ZoneKindInfo` per kind: `kind` prefixed with the
   plugin id (`"game_<year>.<name>"`, other prefixes are rejected by the validator), display name, `shape`
   (`Volume` / `Muzzle`), the kind's `PropertySchema`, default size/pose and gizmo color.
2. Read the persisted `RobotConfig::zones` of your kinds in the game state, resolve their pose with
   `IMechanismDirectory::linkPoseInChassis(zone.attachment...)` * `zone.local` and gate them with
   `evaluateActivation(zone.activation, ctx.mechanisms())` (`sdk/game/zone_activation.h`). Never add season code to core or UI.
3. The editor builds the add-zone menu, inspector and gizmo from the metadata. Test with `FakeGameModule` and
   `FakeMechanismDirectory`; see [ADR 0016](decisions/0016-zones.md).

### Other extension points
Drivetrain (`IDrivetrain`, `DrivetrainRegistry`), tire model (`ITireModel`), plant (`IPlant`, `PlantFactory`),
transport/codec (`ITransport`, `IMessageCodec`): see the table in ARCHITECTURE section 5.

## Changing the SDK
Until the first release the SDK is version 1.0 and the IPC protocol is 1 ([ADR 0013](decisions/0013-pre-release-single-sdk-version.md)):
change it freely (including removals and renames) without version bumps, migrations or compatibility fields, and update the
callers, tests and docs in the same PR. Add or update a test in `tests/sdk`. Versioned evolution rules return with the first release.

## Documentation
Update the doc next to the code you change (`docs/` per module, ADRs for decisions, the README of `app/`, `app/editor`,
`app/runview`). Keep claims verifiable: say what is tested and what is not, as `packaging.md` and `game_2026.md` do.
User-facing behaviour belongs in [USER_GUIDE.md](USER_GUIDE.md).
