# Runtime and configuration

**Read this when:** changing the persisted robot model, composing a simulation, or tracing
robot feedback and reset placement.
**Prerequisites:** [Architecture](../ARCHITECTURE.md) for ownership and tick order;
[Contributing](../CONTRIBUTING.md) for conventions.

## The robot.json contract

[RobotConfig](../../sdk/include/wpiemu/sdk/config/robot_config.h) defines the model;
[RobotConfigCodec](../../core/runtime/src/config/robot_config_codec.cpp) defines JSON spelling.
The core schema is `1`: an omitted `schemaVersion` defaults to it; another version is
rejected, not migrated. Selected plugin sections also use schema version `1`.

| Root members | Responsibility |
|---|---|
| `name`, `project`, `gamePluginId` | Robot identity, project/toolchain metadata, explicit plugin selection; an empty plugin id selects by `project.wpilibYear`. |
| `chassis`, `drivetrain` | Body dimensions/mass, wheel specifications, drivetrain/tire registry keys and drivetrain mappings; see [Physics](physics.md). |
| `mechanisms` | Instances with stable `id`, template `type`, `properties`, `geometry`, `mount`, structured `attachment` and mappings; see [Mechanisms](mechanisms.md). |
| `robotMappings` | Robot-level slots, including the optional `robot.imu`. |
| `motionLinks` | One-way modeled-axis relationships, not device aliases or attachment frames; see [motion links](mechanisms.md#one-way-motion-links). |
| `zones` | Plugin-owned kinds/properties on robot attachment frames; see [Game plugins](game-plugin.md). |
| `cameras` | Core-owned camera models on attachment frames; vendor output is described in [Bridges](bridges.md). |
| `sensors` | Core-owned game-value-to-device responses, described below. |
| `startPose`, `startPositions` | Default physical start and optional named placements, described below. |
| `battery`, `sim` | Electrical model, fixed step, seed and physics settings. |
| `plugins` | Sections keyed by plugin id, each with `schemaVersion` and opaque object `data`. |

A mapping stores `owner`, `slot`, a declared `device` address and optional `reversed`
(omitted when false). Owners are `robot`, `drivetrain`, a mechanism id or a sensor id.
The declared address is not necessarily a unique physical identity; preserve
[alias validation](bridges.md) rather than treating bus labels as independent hardware.

Absent fields receive model defaults. Unknown structural members are ignored and not
round-tripped; property bags and plugin `data` remain `Value` objects.
Wrong types produce pathed `ParseError`s. Typed integers must be finite, integral and
in range before conversion; geometry numbers must also fit `float`. The persisted
`sim.seed` is signed int64, so negative on-disk values can represent uint64 seeds.
The JSON text codec keeps insertion order and integer/double distinctions without a
second tree dependency. Its numeric stream fallbacks explicitly use
`std::locale::classic()`; see [JSON conversion](../../core/runtime/src/config/json_codec.cpp).

## Decode, default, validate, build

[ConfigPipeline](../../core/runtime/src/config/config_pipeline.cpp) is the common loading path.
`load(Value)` decodes, then calls `process(RobotConfig)`. Processing selects the plugin,
registers its templates through the idempotent `PluginRegistrar`, fills missing defaults
for its section, mechanisms, zone kinds and sensor responses, then validates.
Defaults fill missing property keys without replacing present values or unknown keys.

Decode failures, failed plugin registration and an unsupported selected-plugin section
version return an error. Semantic problems return `LoadedConfig` with issues:
successful loading alone does **not** imply `valid()`. Validation does not mutate the
model. Keep stable issue codes and paths; the authoritative rules are in
[ConfigValidator](../../core/runtime/src/config/config_validator.cpp) and the
[SDK validation keys](../../sdk/include/wpiemu/sdk/config/validation_codes.h).

Required unmapped mechanism slots are errors; missing drivetrain mappings are warnings.
Motion-link followers are exempt from motor requirements. Device-capability checking
uses an optional lookup and skips unknown devices, allowing offline editing.
The default step is 5,000 microseconds; a valid step must be positive and divide the
20,000-microsecond robot period exactly.

`SessionBuilder` expects defaulted, validated input and already-created collaborators;
it is not a replacement for the pipeline. It constructs field, chassis/coupling,
tire/drivetrain, slots/mappings, mechanisms and game state in dependency order.
Missing collaborators, unknown factory keys or construction failures return errors
without exposing a partly built session. With no selected plugin, the app host supplies
a floor and runs core features only.

## Persistence responsibilities

[ProjectConfigRepository](../../core/runtime/include/wpiemu/core/config/config_store.h)
owns disk policy. The app's
[ConfigRepository](../../app/common/config_repository.cpp) adapts it and processes loaded
models through the pipeline; it does not implement another store.

The working copy lives under the platform app-data root at
`projects/<project-key>/robot.json`. The key incorporates the canonical project path,
case-folded on Windows. The shareable export is `<project>/.wpiemu/robot.json`.
`open()` prefers an existing working copy; a corrupt one is reported, not silently
replaced by the export. With no working copy, it loads the export and attempts to cache
it; cache failure warns but does not discard a successfully loaded model.

Writes flush a sibling scratch file and atomically replace one target, so a failed
replacement does not leave partial JSON. Saving the working copy and exporting are
separate operations, not a two-file transaction. Import loads the export, backs up an
existing working copy without overwriting another backup, then replaces it; backup
failure aborts the import. Platform paths and error handling are implemented in
[the store](../../core/runtime/src/config/config_store.cpp).
Edits to wizard basics must preserve the existing mechanisms, mappings and plugin
sections, not replace the whole model. User-facing file management is in the
[User guide](../USER_GUIDE.md).

## Session, runner and host

[SimulationSession](../../core/runtime/include/wpiemu/core/runtime/simulation_session.h)
owns simulation state and advances synchronously: no I/O, threads or wall-clock pacing.
Borrowed registries and the logger must outlive it. The complete tick sequence
has one home in [Architecture](../ARCHITECTURE.md).

[SessionRunner](../../core/runtime/include/wpiemu/core/runtime/session_runner.h)
adds the thread-safe controller queue, link handling, snapshots, watchdog and pacing.
`poll()` is a bounded iteration; `start()` drives it on the sim thread. It preserves
the single-outstanding-tick and session/epoch rules in [Extension and IPC](extension-ipc.md).
Reset clears cached actuation and sensor history while retaining mappings and discovered
devices; the next sensor frame is full. Reconnect additionally clears discovered devices.
Old motor commands must never power the first tick of a new run.

The app's [RobotSession](../../app/host/robot_session.cpp) composes the world, IPC,
runner and launcher. Run refuses validation errors; Armed discovery can warn and attempt
a bare model if full construction fails. Armed operation handles discovery and loopback
without stepping; Begin/End lock-step keep the robot process alive. The host waits for
the extension's program-start lifecycle signal before ticking or starting its tick
watchdog, so a slow robot constructor is not mistaken for a stalled tick.
UI host state and restart policy belong to [App](app.md).

The session's battery model uses supply current from mechanisms and drivetrain plus
baseline draw. `Vnext = clamp(Vnominal - I * R, 0, Vnominal)` becomes the next tick's
available voltage: the one-tick lag avoids solving an electrical feedback loop.
Brownout disables the outgoing Driver Station state; recovery requires voltage at least
0.5 V above the configured brownout threshold. See
[BatteryModel](../../core/runtime/src/runtime/battery_model.cpp).

## Launcher boundary

[RobotLauncher](../../core/runtime/include/wpiemu/core/launcher/robot_launcher.h) supervises unmodified robot code.
Default launch injects a generated Gradle init script; external-direct mode builds/describes
the simulation target, then starts its JVM/native binary. Neither edits sources/build scripts.
`ProjectInspector` reads metadata without executing Gradle; the wizard words its report and
adds nothing of its own. The JDK Gradle runs with is chosen in one order,
[`kJdkSources`](../../core/runtime/include/wpiemu/core/launcher/project_inspector.h): the Settings JDK folder,
the project season's WPILib JDK (all-users or this-user install, or the Settings WPILib folder), `JAVA_HOME`, then
`java` on `PATH`. Both the session launch and the wizard's toolchain check follow it. The host requests protocol Stop
before cleanup; cancellation remains latched through startup and escalates to whole-tree kill
after the grace deadline. Destruction joins the worker and cleans up the process tree.
Robot console lines get a level from [`classifyOutputLine`](../../core/runtime/include/wpiemu/core/launcher/output_level.h)
(`ERROR`/`Exception`/stack frames, `Warning`, `DEBUG`, `TRACE` as whole words; plain stderr defaults to Warn).

## Device feedback and game-fed sensors

[DeviceHub](../../core/runtime/src/runtime/device_hub.cpp) resolves slots on the sim thread.
Missing/unusable motor outputs are neutral and writes to unresolved slots are ignored.
Unmapped live devices stay in extension-side loopback; discovery never auto-binds them.
Warnings are derived from current state, so fixing a mapping removes its warning.
Per-motor mounting orientation is applied only at this boundary; see
[multi-motor actuation](mechanisms.md#motor-commands-and-multiple-controllers).

Drivetrain motor slots receive rotor-side turns and turns/s, including their gearing.
Only drivetrains whose registered slots include a wheel's steer slots (swerve) expose
`<wheel>.steer` and the optional `<wheel>.steerEncoder`; tank and mecanum never do, whatever
`WheelSpec::steered` says. `steerEncoder` reports output-side azimuth
in rotations and rotations/s, with no steer gear ratio. A remote/fused steering encoder
belongs there, not on the steer motor slot. Generic mechanism feedback conversions are
specified in [Mechanisms](mechanisms.md#feedback-units).
Slots carry calibrated model-frame readings; `sensorOffsetRot` is a model offset, not
a second firmware magnet calibration. Vendor calibration and reported-current versus
current-request normalization belong to [Bridges](bridges.md).
The optional `robot.imu` reports attitude, body-frame angular rates and specific force
(+g at rest), not gravity-free linear acceleration.

[SensorConfig](../../sdk/include/wpiemu/sdk/config/sensor.h) has no pose: each sensor maps
a game value to its single `<id>.sensor` slot. Distance responses are metres, never
analog volts; beam-break responses are digital levels. `SensorSystem` samples after
the game update. Missing/non-finite published values produce the configured idle reading
and, for a nonempty source, `sensor_source_unavailable`. Snapshots include source value
and reading.

## Physical start placement

When present, `startPose` requires finite `xM`, `yM`, `headingRad`. The host uses those
field-frame coordinates and yaw; chassis construction still determines spawn height.
Without it, the host chooses a placement inside the described field, or the origin
without a field description. This moves physics, not robot odometry or gyro offsets.

`startPositions` is an optional list of `{id, label, alliance, xM, yM, headingRad}`;
the coordinates are required, ids are nonempty and unique within the list, and alliance
is `blue` or `red` (default blue). An absent list is empty; none are built in.
Selection never implicitly flips a pose. Alliance-copy tooling uses the plugin's
field symmetry, described in [Game plugins](game-plugin.md).

`ISimController::setStartPlacement(optional<RobotStartPose>)` queues an override in order
with Reset. Setting it alone never teleports: the next reset restores override X/Y/yaw
at the built spawn height. `nullopt` restores the built default placement.
`host::Options::startPose` seeds this override for a new session. Reset/restart UI
semantics and selected-position lifetime are documented in [App](app.md).

**Next:** [Mechanisms](mechanisms.md).
