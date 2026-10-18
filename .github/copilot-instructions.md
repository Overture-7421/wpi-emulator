# Repository guidance

WPI Emulator is a C++20 desktop simulator for **unmodified** WPILib Java/C++ robot
projects. The Qt app owns simulation; a HALSim extension injected by the Gradle
launcher runs inside the robot process. Season-specific behavior belongs in game
plugins, not in the core or UI.

Read `docs/ARCHITECTURE.md` and `docs/CONTRIBUTING.md` for dependency and extension
contracts; `docs/BUILDING.md` documents prerequisites and options. Decisions live
in `docs/decisions/`. Module-specific guidance is in `docs/runtime.md`,
`docs/mechanisms.md`, `docs/halsim-extension.md`, `docs/bridges-hal.md`,
`docs/bridges-vendor.md`, and the READMEs under `app/`.

## Build, test, and checks

Run commands from the repository root. Prerequisites: CMake 3.25+ for presets,
a C++20 compiler, Python 3, and Qt 6.8 with Quick 3D and Shader Tools. Box3D and
doctest are fetched by CMake. Windows uses MSVC 2022; non-VS presets require Ninja.

```powershell
# Windows: Visual Studio generator, no developer shell required.
# Substitute the actual Qt kit location.
cmake --preset windows-vs2022 -DCMAKE_PREFIX_PATH="C:\Qt\6.8.3\msvc2022_64" -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build --preset windows-vs2022-release --parallel
ctest --preset windows-vs2022-release

# List tests, then run one exact CTest test (the filter is a regex).
ctest --preset windows-vs2022-release -N
ctest --preset windows-vs2022-release -R "^runner: handshake assigns session, epoch, step and the negotiated watchdog$"

# Run a Qt suite.
ctest --preset windows-vs2022-release -R "^app_editor[.]tests$"

# Architecture / header-macro checks and the checker's own tests.
python tools\check_boundaries.py
python tools\check_boundaries.py --self-test
```

Other configure presets are `windows-ninja`, `linux`, and `macos`; corresponding
build/test presets append `-debug` or `-release`. Presets also read `QT_ROOT_DIR`.
With Windows Ninja, run `vcvars64.bat` and CMake in the **same** `cmd.exe` process.
Build trees are `build\<preset>` and executables are in `bin\<Config>` beneath
that tree; do not mix generators in one tree.

Useful configure switches:

- `-DWPIEMU_BUILD_APP=OFF` builds without Qt.
- `-DWPIEMU_WPILIB_DOWNLOAD=ON` downloads HAL dependencies when no WPILib Maven
  cache is available; alternatively set `WPIEMU_WPILIB_ROOT`. Without HAL,
  configuration can succeed but real-HAL support/tests are absent.
- `-DWPIEMU_RUNVIEW_FETCH_SDL2=ON` enables fetching SDL2 for gamepads.

The project uses doctest. `tests/**/*.cpp` feeds `wpiemu_tests`; CMake discovers
each `TEST_CASE` as a CTest entry. Reconfigure after adding tests. Names must be
unique and must **not contain semicolons**, which break CTest discovery.
SDK headers automatically receive self-contained-include checks.

Qt suites have separate executables and CTest entries (`app.ui_tests`,
`app_editor.tests`, `runview.ui_tests`); editor tests use `tests/app_editor/*.cc`
to avoid the Qt-free `.cpp` glob. For an individual case inside a suite, run its
binary with `--test-case="exact case name"`. Direct Qt binary runs need
`QT_QPA_PLATFORM=offscreen` and the Qt DLL directory on Windows `PATH`; CTest
sets these for the UI suites. Offscreen tests do not validate Quick 3D rendering:
use `wpiemu_editor_demo` or `wpiemu_runview_demo` for visuals.

The real-project test is opt-in: set `WPIEMU_E2E=1` and
`WPIEMU_E2E_PROJECT` to a WPILib robot project, then run
`ctest --preset windows-vs2022-release -L e2e`. It requires the WPILib toolchain
and runs against a temporary project copy.

Formatting is defined by `.clang-format`: Google-based C++20, four spaces,
100 columns. First-party warnings are `/W4` or `-Wall -Wextra -Wpedantic`;
`WPIEMU_WARNINGS_AS_ERRORS=ON` promotes them to errors.

## Architecture and dependency boundaries

- `sdk` is header-only and standard-library-only: public interfaces, data models,
  registries, protocol types, and testing fakes.
- `core` is game-agnostic and Qt-free. Physics, mechanisms/plants, and runtime/config
  form one `wpiemu_core` target; Box3D stays behind SDK physics interfaces.
- `ipc` implements SDK transport/codec interfaces with Windows named pipes or
  Unix sockets. `halsim_ext` lives in the robot process and uses SDK, IPC, and
  the bridge aggregator, not core.
- Bridge modules depend on SDK, not each other. Each game plugin depends **only**
  on SDK. No Qt in SDK, core, IPC, bridges, or plugins.
- `plugins/builtin_plugins.cpp` / `wpiemu_plugins_builtin` is the app's only
  built-in plugin aggregation seam. `bridges/builtin_bridges.cpp` /
  `wpiemu_bridges_builtin` is the corresponding extension-side seam.
  Do not add direct app-to-season or extension-to-individual-bridge coupling.

`tools/check_boundaries.py` checks includes and CMake links. Public headers use
`<module>/include/wpiemu/...`; targets use `wpiemu_...`. Headers must not use
Qt macro names (`signals`, `slots`, `emit`, `foreach`) as identifiers.

`AppServices` is the composition root. The editor and run view share an
`IRuntimeHost`; `--demo` selects `DemoRuntimeHost`, otherwise `RealRuntimeHost`
delegates Qt-free process/session orchestration to `app/host/RobotSession`.
`SimulationSession` is synchronous and deterministic, with no I/O or threads;
`SessionRunner` adds command queues, lock-step IPC, and pacing around it.

The emulator is clock master (default 5 ms step, dividing the 20 ms robot period).
Each tick consumes prior actuator outputs, advances mechanisms, drivetrain,
physics, and game state, then emits sensor feedback and an immutable snapshot.
The extension applies sensors, steps HAL timing, and returns actuator outputs.
Only **one tick may be outstanding**. Preserve `sessionId`/`runEpoch`, stale-frame
rejection, reset/reconnect semantics, command acknowledgements, and the
extension's independent watchdog unfreezing behavior; see Architecture section 4.

The sim thread owns all mutable simulation state. UI code reads immutable
`ISimSnapshot`s and sends commands through `ISimController`. HAL callbacks only
record events; never block or perform socket I/O there.

## Repository-specific conventions

- Recoverable errors use `sdk::Result<T>` / `sdk::Status`; exceptions must not
  cross SDK interfaces. Contract violations use `contractViolation`.
- Owners use `std::unique_ptr`; collaborators are injected through constructors
  or `SimContext`. Registries are instances, not globals or singletons.
- Internal units are SI in right-handed Z-up NWU coordinates; quaternions are
  `(x,y,z,w)`. Suffix plain scalars with units (`massKg`, `radiusM`, `timeUs`).
  Sim time is integer microseconds from `ISimClock`, randomness from
  `SimContext::rng()`, never wall-clock time in simulation code.
- UI follows MVVM. Widgets do not access config persistence or runtime directly.
  Use `tr()` (`ETR()` in editor models), `ThemeTokens`, and `UnitFormatter`.
  Editor changes go through undoable config operations. Keep world-frame
  conversion to Qt's Y-up scene in QML, not in persisted or physics coordinates.
- Persistence belongs to core's `ProjectConfigRepository`, behind app's
  `ConfigRepository`. Preserve mechanisms, mappings, and plugin sections when
  updating wizard basics; do not replace the whole model.
- Config loading order is decode (current schema only, no migration), defaults,
  validation, then construction. A plugin owns only its config section and
  mechanisms whose type starts with its plugin id plus `.`.
  Validation issue codes are stable SDK keys. JSON numeric stream fallbacks
  must use `std::locale::classic()`.
- Distinguish declared `DeviceKey` addresses from physical identity: vendor
  simulation cannot necessarily distinguish CAN buses. Never invent a bus
  label from a static rule (`busIsStatic=false`); preserve alias-conflict
  validation and neutral/loopback behavior.
- Until release the SDK is version 1.0 and the protocol is 1 (ADR 0013): no
  compatibility, migrations, legacy fields or version bumps; change the SDK
  freely. `schemaVersion` in robot.json and plugin sections is 1 and anything
  else is rejected.
- New seasons use `plugins/_template`, SDK interfaces, and the plugin aggregator
  (`docs/new-year-plugin.md`). Mechanisms use slots, plants, and coupling
  interfaces; context-menu/inspector UI is generated from template metadata.
  New bridges use canonical signals in `sdk/devices/signals.h` and register via
  the bridge aggregator. Use the extension recipes in `docs/CONTRIBUTING.md`.
- Tests mirror module layout and can use header-only fakes under
  `sdk/include/wpiemu/sdk/testing/`. Keep module behavior documented in its
  existing doc; user-facing behavior belongs in `docs/USER_GUIDE.md`, and
  boundary/SDK decisions belong in ADRs.
