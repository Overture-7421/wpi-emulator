# Runtime and configuration

**Read this when:** changing robot configuration, building a simulation, choosing Java for a
robot project, or tracing a reset or feedback problem.
**Prerequisites:** [Architecture](../ARCHITECTURE.md) for ownership and tick order;
[Contributing](../CONTRIBUTING.md) for project conventions.

This page owns the core runtime contract: `robot.json`, load and construction order, project
setup persistence, Java selection, and robot placement. Follow [Editor properties](../guides/editor-properties.md)
to add a model field; use [Maintenance](../guides/maintenance.md) to trace a failure.
Subsystem details live in [mechanisms](mechanisms.md), [physics](physics.md),
[bridges](bridges.md), and [extension/IPC](extension-ipc.md).
For orientation, see [simulation and physics](../concepts/simulation-and-physics.md) and
[WPILib simulation](../concepts/wpilib-simulation.md).

## Configuration and loading

[`RobotConfig`](../../sdk/include/wpiemu/sdk/config/robot_config.h) is the model and
[`RobotConfigCodec`](../../core/runtime/include/wpiemu/core/config/robot_config_codec.h)
defines its JSON representation. The core schema is version `1`: an omitted version means
`1`; other versions are rejected, not migrated. Plugin sections have their own version.

The root object stores robot identity and project metadata; chassis, drivetrain, mechanisms
and mappings; plugin data, zones, cameras and sensors; start placement; and simulation and
battery settings. `project` records language, WPILib year and GradleRIO version, not a
filesystem path. A mapping's declared device address is not necessarily a unique physical
device; retain the identity and alias rules in [Bridges](bridges.md).

Missing fields receive defaults. Unknown structural members are ignored and are not
round-tripped; property bags and plugin data remain values owned by their respective
features. Invalid types or out-of-range numbers produce path-specific parse errors.
`sim.seed` is signed 64-bit in JSON, so negative values can encode unsigned seeds.
Numeric stream fallbacks use the classic locale.

[`ConfigPipeline`](../../core/runtime/src/config/config_pipeline.cpp) always decodes, selects
and registers the plugin, fills missing defaults, then validates. Defaults do not replace
present or unknown property keys. Decode, registration, or unsupported plugin-schema failures
return errors; semantic problems are issues on the loaded model. A successful load does not
mean the configuration is valid, and validation does not modify it. Preserve stable issue
codes and paths.

The editor and runtime use the same SDK validation rules. Required unmapped mechanism slots
are errors; missing drivetrain mappings are warnings. Motion-link followers do not require
motors. If a device is not known, optional capability checks are skipped so projects remain
editable offline.

[`SessionBuilder`](../../core/runtime/include/wpiemu/core/runtime/simulation_session.h) builds
only from defaulted and validated input, with its collaborators already created. It reports
construction failures rather than exposing a partial session. Without a selected game plugin,
the app can run core simulation on a floor.

## Persistence responsibilities

[`ProjectConfigRepository`](../../core/runtime/include/wpiemu/core/config/config_store.h)
owns disk behavior; the app repository adapts it and runs loaded models through the pipeline.
The project's only robot setup file is `<project>/.wpiemu/robot.json`.

Opening an existing project file never falls back to an older copy if that file is unreadable.
For projects created by older versions, a setup under the platform app-data directory at
`projects/<project-key>/robot.json` is moved into the project once. After a successful write,
the old file is retained as `robot.json.migrated.bak` and the app-data copy is removed. If
the project write fails, the app-data setup remains available for a later move; if preserving
the old file fails, the original is left in app data.

Saving writes a flushed sibling scratch file and replaces the target, avoiding partial JSON.
If the existing project file cannot be loaded, it is first preserved as
`robot.json.unreadable.bak` (or a numbered variant if that name exists). If making that backup
fails, the save fails too. A wizard edit must preserve mechanisms, mappings and plugin
sections rather than replacing the whole model. See the [User guide](../USER_GUIDE.md) for
user-facing file behavior.

## Launcher boundary

The same JDK resolution is used by the setup check and by a run. Candidates are tried in this
order: the JDK folder explicitly selected in Settings; the project's season WPILib JDK; another
installed season's WPILib JDK; `JAVA_HOME`; `java` on `PATH`; then Java downloaded by the
emulator. An explicit Settings choice is never silently replaced: if it fails validation,
the launch is blocked until the choice is changed or cleared.

Candidates must be runnable JDKs with `javac`, and meet that WPILib season's minimum Java
major. They must also be no newer than the project's Gradle wrapper supports (defaulting to
Gradle 8.11 when its version is unknown). The project's own season WPILib JDK is trusted for
Gradle compatibility, compiler presence and Visual C++ runtime; it still must run and meet
the minimum Java major. Other candidates must pass all checks. On Windows, if the JDK's
`java.exe` loads a Visual C++ runtime, it must be version 14.40 or newer. These rules are
shared by [`kJdkSources`](../../core/runtime/include/wpiemu/core/launcher/project_inspector.h)
and [`jdk_policy`](../../core/runtime/include/wpiemu/core/launcher/jdk_policy.h).

[`ProjectInspector`](../../core/runtime/include/wpiemu/core/launcher/project_inspector.h)
reads project metadata without running Gradle. The default launcher injects a generated
Gradle init script; direct-launch mode uses Gradle to describe the simulation target before
starting its JVM or native program. Neither mode edits the robot project's sources or build
scripts. The selected JDK is passed to Gradle so the project's `gradle.properties` cannot
override it. The wizard's Java result and the launch must agree; if no candidate works, do
not start Gradle.

The live GradleRIO version selects the HAL extension and default game module, rather than the
saved setup's year. Extensions are installed in year-qualified directories. Known handshake
year mismatches are refused; an explicit `WPIEMU_EXTENSION` permits them with a warning.
A missing matching extension is `no_extension_for_season` in the Toolchain report.
2026 uses `simulateJava`; 2027 uses `run` (JavaExec) and `simulateExternalJava`,
with Java 25 and Gradle 9.2+. The 2027 native simulation task package is `org.wpilib`.
The alpha install folder is `2027_alpha7`, searched under Windows public WPILib,
Linux XDG data (`~/.local/share` by default), and macOS `~/.wpilib`.

The host requests protocol Stop before cleanup. Cancellation stays active through startup
and escalates to terminating the process tree after its grace period. Shutdown joins the
worker and cleans up the process tree.

## Session, runner and host

[`SimulationSession`](../../core/runtime/include/wpiemu/core/runtime/simulation_session.h)
owns synchronous simulation state; it does no I/O, pacing or threading. Its borrowed
registries and logger must outlive it. [`SessionRunner`](../../core/runtime/include/wpiemu/core/runtime/session_runner.h)
adds command queues, IPC, pacing, snapshots and watchdog handling. The authoritative tick
order and clock-master contract are in [Architecture](../ARCHITECTURE.md).

Run mode refuses configuration errors. Armed discovery may continue with warnings; if the
full model cannot be built, it can fall back to an empty model so the robot process can still
discover devices.

The default fixed step is 5,000 microseconds and must divide the 20,000-microsecond robot
period. A reset clears cached actuation and sensor history but keeps mappings and discovered
devices; its next sensor frame is full. Reconnect also clears discovered devices. Old motor
commands must never actuate the first tick of a new run. See [Extension and IPC](extension-ipc.md)
for identity, acknowledgement and reconnect rules.

The battery model uses supply current from the drivetrain and mechanisms plus a baseline
draw. The next tick's voltage is `clamp(Vnominal - I * R, 0, Vnominal)`, which avoids
solving an electrical feedback loop within one tick. Brownout disables the outgoing Driver
Station state; recovery requires voltage at least 0.5 V above the configured threshold.
See [BatteryModel](../../core/runtime/src/runtime/battery_model.cpp).

## Physical start placement

`startPose`, when present, is a finite field-frame position `(xM, yM, headingRad)`. It changes
the physical spawn pose, not robot odometry or gyro offsets; chassis construction still sets
spawn height. Without it, the host places the robot at the origin if there is no field
description; otherwise it uses `x = min(2 m, 15% of field length)` and the field's mid-width.

`startPositions` is an optional list of named placements. Each entry has a unique, nonempty
`id`, a label, `xM`, `yM`, `headingRad`, and an alliance (`blue` or `red`, default `blue`).
There are no built-in positions, and selecting one never flips it for alliance. Alliance
copying uses the game plugin's field symmetry; see [Game plugins](game-plugin.md).

The selected named position is retained through a same-project robot restart, and cleared
when the project changes or the selected entry is removed. `ISimController::setStartPlacement`
queues a placement override; it does not teleport the robot. The next Reset restores the
override at the built spawn height. Passing no override restores the configured default.
This also applies to a session's initial start placement. Reset and restart are distinct:
reset does not restart robot code or reset its odometry.

## Device feedback and game-fed sensors

[`DeviceHub`](../../core/runtime/src/runtime/device_hub.cpp) resolves mappings on the
simulation thread. Missing or unusable motor outputs are neutral; writes to unresolved slots
are ignored. Unmapped live devices remain in extension-side loopback and discovery never
auto-binds them. Fixing a mapping removes warnings derived from the old state.

Mechanism and drivetrain feedback units, motor orientation and steering slots belong to
[Mechanisms](mechanisms.md). Vendor calibration and current normalization belong to
[Bridges](bridges.md). The optional `robot.imu` reports attitude, body-frame angular rates
and specific force (including +g at rest).

Each configured game-fed sensor maps a game value to one `<id>.sensor` slot. Distance values
are metres, not analogue volts; beam breaks are digital. Sensors sample after the game
update. An unavailable or non-finite source produces the configured idle reading; a named
but unavailable source also reports `sensor_source_unavailable`. See [Game plugins](game-plugin.md)
for the game-value contract.

**Next:** [Mechanisms](mechanisms.md).
