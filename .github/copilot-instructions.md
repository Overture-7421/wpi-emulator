# Repository guidance

WPI Emulator runs unmodified WPILib Java/C++ projects. The Qt app owns simulation;
a HALSim extension inside the robot process exchanges feedback through local IPC.
Season behavior belongs in game plugins, never core or UI.

## Read before changing code

- [Architecture](../docs/ARCHITECTURE.md): dependency graph, ownership and tick order.
- [Building](../docs/BUILDING.md): prerequisites, presets, focused tests and opt-in integration.
- [Contributing](../docs/CONTRIBUTING.md): errors, units, formatting, tests and documentation policy.
- [Extending](../docs/extending.md): the single home for add-a-feature recipes.
- [Runtime](../docs/reference/runtime.md), [mechanisms](../docs/reference/mechanisms.md),
  [physics](../docs/reference/physics.md), [extension/IPC](../docs/reference/extension-ipc.md),
  [bridges](../docs/reference/bridges.md), [games](../docs/reference/game-plugin.md) and
  [app](../docs/reference/app.md) describe subsystem contracts.
- [User guide](../docs/USER_GUIDE.md), [releasing](../docs/releasing.md) and the
  [decision index](../docs/decisions/README.md) serve users, maintainers and design rationale.

## Preserve the boundaries

SDK is header-only and standard-library-only. Keep Qt out of core, IPC, bridges and plugins.
Plugins depend only on SDK. The app uses the plugin aggregator; the extension uses the
bridge aggregator. Do not introduce direct app-to-season or extension-to-bridge coupling.
Keep non-app headers safe from Qt macro identifiers.

`AppServices` composes the app; editor and run view share one `IRuntimeHost`.
`RobotSession` is Qt-free process orchestration. `SimulationSession` owns synchronous
simulation; `SessionRunner` adds queues, IPC and pacing. UI reads immutable snapshots.
The emulator is clock master with one outstanding tick. Preserve session/epoch rejection,
reset/reconnect, acknowledgements and independent watchdog recovery. HAL callbacks never block.

Use `sdk::Result<T>` / `sdk::Status`; do not let exceptions cross SDK interfaces.
Simulation uses SI/NWU, `(x,y,z,w)` quaternions, integer clock microseconds and context RNG.
Keep scene-frame conversion in QML. UI edits use undoable operations, translations,
`ThemeTokens` and `UnitFormatter`, not direct runtime or persistence access.
Preserve unrelated config sections when changing wizard basics; core owns persistence.
Decode, default, validate, then construct; keep validation keys stable and numeric stream
fallbacks in the classic locale. Keep the single pre-release schema/SDK/protocol policy.
Do not infer physical CAN bus identity from a declared address or invert mapped motors twice.

## Checks and documentation

Run from the repository root, using the commands in Building. Set warnings-as-errors
when configuring, not when building. Windows Ninja needs vcvars and CMake in one process.
Use focused doctest/Qt selectors; CTest sets the UI environment. Offscreen checks do not
validate Quick 3D rendering. SDK headers receive automatic self-contained include checks.
Boundary and documentation checks are `python tools\check_boundaries.py`,
its `--self-test`, and `python tools\check_docs.py`.

Keep one home per fact: recipes in Extending, contracts in the owning reference,
user behavior in the User guide, rationale in decisions. Link instead of copying.
Verify named APIs, paths and commands; keep links current and remove obsolete material.
