# ADR 0010 - Runtime, configuration and launcher (`core/runtime`)

Status: **Accepted** (implemented in `core/runtime/`, tested on Windows/MSVC and Linux/g++; macOS shares the POSIX code).
Details of the API: [docs/runtime.md](../runtime.md).

## Decisions

1. **JSON: in-house codec, no dependency.** `sdk::Value` already is a JSON-shaped tree with insertion-ordered objects, so the only
   work is text <-> `Value`. A ~400-line parser/writer gives diff-friendly output (key order kept, `0.7` stays `0.7`, int vs double
   distinct), exact error positions and no FetchContent in `core`. nlohmann/json was rejected: its objects sort keys (or need
   `ordered_json`), a second tree type would need conversion code anyway, and it adds a build-time dependency to a module that otherwise
   needs none. The codec lives behind `IValueTextCodec`, so swapping it later is a one-file change. Hostile input is bounded (nesting 128).
2. **Unknown-field preservation needs one additive SDK field.** `RobotConfig::unknownFields` (a `Value` object keyed by owner path, defaulted
   empty) is the only SDK 1.1 addition. The codec fills it; editors never touch it. Without it a file written by a newer build would silently lose
   data when an older build saved it. Unknown keys inside arrays of mappings/primitives are not preserved (they have no stable identity).
3. **The coordinator is a class, not a convention.** `ConfigPipeline` implements ARCHITECTURE 4.4 once (core migration -> plugin section chain ->
   plugin `migrateRobotConfig` -> defaults -> validation) and *enforces* the ownership rule by diffing everything except `plugins[id]` and the plugin's
   own mechanisms; a violating plugin makes the load fail instead of corrupting the model. Plugin mechanism templates are registered through
   `PluginRegistrar` (idempotent) so pipeline, session builder and editor cannot double-register.
4. **The validator knows no devices.** `IConfigValidator::validate` receives registries, not a device catalogue, so slot/device capability checks use an optional
   injected `DeviceCapabilityLookup` (unknown device => skipped, not an error) and drivetrain slots follow the naming convention `<wheel>.drive|steer`.
   Required-but-unmapped slots are Errors for mechanisms (template says required) and Warnings for the drivetrain (follower motors are not separate devices).
5. **Persistence.** Atomic temp+rename with flush; per-user working copy keyed by a hash of the canonical project path, shareable export in
   `<project>/.wpiemu/robot.json`; upgrading an older file backs the original up first and never overwrites a backup. App-data paths are computed from
   an injectable environment (no Qt) so all three OS layouts are unit-tested on one machine.
6. **Hub = pure sim-thread object, mailbox for the IPC thread.** The SDK says hub calls are sim-thread only; thread safety is therefore a `MessageInbox`
   (multi-producer, single-consumer) in front of it, not locks inside it. All hub warnings are *derived* from state after every change, which makes
   de-duplication and "warning goes away when fixed" automatic. The hub deviates from `FakeDeviceHub` only where the fake is silent (retired devices are
   never resolved or looped back, announces under a new `DeviceId` re-key the same physical device).
7. **Session = deterministic and synchronous; runner = lock-step I/O.** `SimulationSession::tick` has no threads, clocks or I/O (same inputs => same
   `stateHash`), `SessionRunner::poll` is one bounded iteration with an injectable wall clock, so the whole protocol (reset epochs, watchdog, pacing,
   reconnect) is tested deterministically against a scripted extension on the SDK's in-memory transport. Battery voltage uses the previous tick's current
   (one-tick lag breaks the motor<->voltage loop without iteration).
8. **Chassis and coupling are seams, not hard dependencies.** The runtime only knows `ChassisBuilder` (default: a reference hull+pads builder) and
   `ICouplingDriver` (bind / attachDrivetrain / beforeStep / afterStep / reset). The header-only `runtime/physics_wiring.h` adapts `core/physics`
   (`buildChassisBody`, `ChassisPhysicsCoupling`) for the composition root; the runtime sources never include it, so the runtime builds and tests
   without physics, and an integration test proves the real wiring.
9. **Launcher: Gradle init script by default, external direct launch on request.** Both paths of ADR 0001 are implemented. The init script is generated, constant
   (paths travel in `WPIEMU_EXT`), and passed with `-I` only. Gradle always runs with `--no-daemon` so the process tree stays ours; the robot PID from `Hello` plus
   `killProcessTree` covers the documented case where Gradle's single-use daemon outlives a killed client.
10. **Process supervision per OS.** Windows: suspended create + kill-on-close job object + exactly three inherited handles; batch files through `cmd /d /s /c`
    with metacharacter-bearing arguments *refused* (BatBadBut class of bugs). POSIX: own process group, exec errors via a CLOEXEC pipe, no non-async-signal-safe
    work after `fork`. A graceful stop on Windows GUI processes does not exist without a console, so the polite path is the protocol's `Stop`; `requestStop()` is best effort.
11. **Inspection is read-only and tolerant.** `ProjectInspector` never executes Gradle; it parses text (regexes bounded to build scripts) and JSON. A directory
    that is not a WPILib project is a report with notes, not an error. `ToolchainReport` is plain data (`Ok/Info/Warning/Error` + fix hint) for the wizard.

## Consequences / limits

* `WheelSpec::positionM.z` is the hub height above the floor (<= 0 means one radius), matching `physics::wheelHubLocal`; the reference chassis follows the same convention.
* Linux/macOS Java simulation may need the JNI directory on `LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH` (ADR 0001 risk); `ExternalDirect` sets it, `GradleInitScript` relies on GradleRIO.
* Gradle arguments for `.bat` wrappers cannot contain `" % ^ & | < > !`; an app-data path with such characters fails the launch with a clear error.
* The Windows tree kill falls back to a PID walk when the job cannot be assigned (very old parent jobs); descendants that deliberately escape the job are not caught.
* The POSIX process code is exercised on Linux (g++, WSL); macOS is untested here.
