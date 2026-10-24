# Repository guidance

WPI Emulator runs unmodified WPILib Java and C++ robot projects. The desktop app owns the
simulation. A HALSim extension in the robot process exchanges data with it through local IPC.
Season-specific game behavior belongs in game plugins; WPILib support for a year is data in its season pack
(`seasons/<year>/season.json`).

## Read first

- [Documentation home](../docs/README.md) for reading paths.
- [Architecture](../docs/ARCHITECTURE.md) and [codebase tour](../docs/CODEBASE_TOUR.md) for ownership
  and call flows.
- [Contributing](../docs/CONTRIBUTING.md) for code, testing, and documentation conventions.
- [Extending](../docs/extending.md) for the right extension seam and its guide.
- The relevant subsystem reference listed from the [documentation home](../docs/README.md).

## Keep the boundaries

- Season and WPILib facts live in the season packs and per-ABI sources. Never write a season year or an `#if`
  on a year/ABI outside `seasons/`, tests, samples and docs; read the generated `SeasonRegistry`, put HAL
  differences in `abi/<abi>/` sources, and run `python tools\check_seasons.py`.
- The SDK is header-only and standard-library-only. Keep Qt out of the SDK, core, IPC, bridges,
  and plugins. Plugins depend only on the SDK. The app uses the plugin aggregator; the extension
  uses the bridge aggregator. Do not couple the app directly to a season plugin or the extension
  directly to a bridge. Non-app public headers must avoid Qt macro names such as `signals`,
  `slots`, `emit`, and `foreach`.
- `RobotSession` owns Qt-free process orchestration. `SimulationSession` owns synchronous
  simulation. `SessionRunner` adds queues, IPC, and pacing. The emulator owns the clock and allows
  one outstanding tick. Preserve session/epoch rejection, acknowledgements, reset/reconnect, and
  independent watchdog recovery. HAL callbacks must not block.
- Use `sdk::Result<T>` and `sdk::Status` at SDK boundaries. Exceptions must not cross them.
- Simulation uses SI units, right-handed NWU coordinates, `(x,y,z,w)` quaternions, integer
  microseconds, and context-provided randomness. Do not use wall clocks in simulation logic.
- UI reads immutable snapshots and edits configuration through undoable operations. Keep
  persistence behind its owning service, use translations and theme/unit helpers, and perform
  scene-frame conversion in QML.
- Decode, default, validate, then construct. Preserve unrelated configuration sections and stable
  validation keys. Use `std::locale::classic()` for numeric stream fallbacks. Keep the current
  schema, SDK, and protocol policy. Do not infer physical CAN identity from a declared address or
  invert mapped motors twice.

## Verify changes

Run checks from the repository root using [Building](../docs/BUILDING.md). Configure warnings as
errors; Windows Ninja setup needs the Visual Studio environment and CMake in the same process.
Use focused tests for fast feedback, then run the checks required by the changed subsystem.
Offscreen tests do not validate Quick 3D rendering.
