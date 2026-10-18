# Runtime, config and launcher (`core/runtime`)

Everything between the UI and the physics that is not physics or a mechanism: the persisted robot model, the device hub,
the lock-step sim loop and the code that starts the robot project. No Qt, no season knowledge; it depends only on the SDK
(plus, for the optional wiring header, the sibling `core/physics`). Rationale and rejected options: [ADR 0010](decisions/0010-runtime.md).

```
core/runtime/
  include/wpiemu/core/config/     json_codec, robot_config_codec, config_validator, config_pipeline, config_store
  include/wpiemu/core/runtime/    device_hub, sim_clock (+BatteryModel), simulation_session, session_runner,
                                  robot_link, physics_wiring (header-only glue to core/physics)
  include/wpiemu/core/launcher/   process, init_script, robot_launcher, project_inspector
  src/{config,runtime,launcher}/  implementations (process_win.cpp / process_posix.cpp per OS)
tests/runtime/                    doctest suite (fixtures in runtime_fixtures.h, built only on sdk/testing fakes)
```

## 1. Configuration (`robot.json`)

`RobotConfig <-IRobotConfigCodec-> Value <-IValueTextCodec-> JSON text`.

| Piece | What it does |
|---|---|
| `JsonValueCodec` | In-house RFC 8259 parser/writer for `sdk::Value` (no third-party dependency, see ADR). Keeps key order, int vs double, shortest round-trip doubles (`0.7` stays `0.7`), `\uXXXX` + surrogates, BOM, nesting limit 128, `line N, column M` in errors. |
| `RobotConfigCodec` | Current schema only (a `schemaVersion` other than 1 => `VersionMismatch`, no migration), lenient decode (absent => default, unknown fields ignored, wrong type => `ParseError` naming the path), writes and reads the structured `attachment`. |
| `ConfigValidator` | `IConfigValidator`: the SDK contracts (`validateDeviceAliases`, `validateMechanismAttachments`) plus the rules below. Never mutates, never throws. |
| `ConfigPipeline` | Coordinator, ARCHITECTURE 4.4: **decode -> defaults -> validation**. |
| `PluginRegistrar` | Registers a plugin's mechanism templates once per factory (idempotent). Route every `registerMechanisms` call through it. |
| `FileConfigStore` / `ProjectConfigRepository` | Disk persistence (below). |

### JSON layout

```jsonc
{
  "schemaVersion": 1,
  "name": "Robot", "gamePluginId": "",
  "project": { "path": "...", "language": "java", "wpilibYear": 2026, "gradleRioVersion": "2026.2.1" },
  "chassis": { "sizeM": {x,y,z}, "massKg": 50, "comHeightM": 0.2, "comOffsetM": {..}, "groundClearanceM": 0.03,
               "inertiaDiagKgM2": {..}, "material": {friction, restitution, rollingResistance}, "primitives": [ ... ] },
  "drivetrain": { "kind": "swerve", "tireModel": "brush", "loadTransferTauS": 0.02,
                  "wheels": [ { "name": "fl", "kind": "swerve", "positionM": {..}, "radiusM": .., "tire": {..},
                                "driveMotor": {..}, "driveGearRatio": .., "steerMotor": {..}, ... } ],
                  "mappings": [ { "owner": "drivetrain", "slot": "fl.drive", "device": {family,type,id,bus,channel} },
                                { "owner": "drivetrain", "slot": "fl.drive2", "reversed": true, "device": {..} } ] },
  "mechanisms": [ { "id": "intake", "type": "mechanism", "label": "", "parent": "chassis",
                    "attachment": { "parentMechanismId": "chassis", "parentLink": "root" },
                    "mount": { "position": {..}, "rotation": {x,y,z,w} }, "geometry": [ ... ],
                    "properties": { ... }, "mappings": [ ... ] } ],
  "robotMappings": [ { "owner": "robot", "slot": "imu", "device": {..} } ],
  "zones": [ { "id": "pickup", "kind": "game_2026.pickup", "label": "",
               "attachment": { "parentMechanismId": "chassis", "parentLink": "root" },
               "pose": { "position": {..}, "rotation": {x,y,z,w} }, "sizeM": {x,y,z}, "properties": { ... },
               "activation": { "mode": "all",
                               "conditions": [ { "mechanism": "flywheel", "value": "velocity", "op": "above",
                                                 "threshold": 100, "tolerance": 0.1, "absolute": true } ] } } ],
  "battery": { "nominalVoltageV": 12.6, "internalResistanceOhm": 0.015, "brownoutVoltageV": 6.8 },
  "sim": { "stepUs": 5000, "seed": 1, "physics": { "workerCount": 1, "subSteps": 4, "sleeping": true, "gravity": {..} },
           "startPose": { "xM": 2.0, "yM": 4.1, "yawRad": 0.0 } },
  "plugins": { "game_2026": { "schemaVersion": 1, "data": { ... } } }
}
```
Units are SI, key names carry the unit suffix of the C++ field. Enums are strings (`"swerve"`, `"box"`, ...).
Typed numeric fields are checked before conversion: integers (including schema versions) must be finite,
integral and within their destination's signed range; floats and geometry components must be finite and
within the float range. Violations are pathed `ParseError`s, not truncated/wrapped values. `sim.seed` keeps
its signed int64 on-disk representation: negative values encode uint64 seeds with the high bit set.
The generic SDK `Value` conversions remain unchanged. JSON conversion and its stream fallbacks use the
classic locale, independent of the process locale.

Unknown fields in a file are ignored on load and are not written back.

### Start pose (`sim.startPose`)

`SimSettings::startPose` is an `std::optional<StartPose>` (`sdk/config/robot_config.h`: `xM`, `yM`, `yawRad`, world frame,
yaw counter-clockwise about +Z; `toPose3` / `toStartPose` convert to and from a `Pose3`). **Unset** (the key is absent and is
only written when set) keeps today's placement exactly: the chassis builder's own spawn pose, which in the app is the field's
default start spot (`host::startPlacementFor`, inside the plugin's field) and for the reference builder the origin. **Set**, the
robot spawns there and `SimulationSession::reset()` (hence every Reset, Begin and reconnect) returns to it. The session applies it
right after the chassis was built, so every `ChassisBuilder` honours it; x, y and the yaw come from the config, the chassis
keeps the builder's own height and stays upright. `RobotSession` also passes it to the builder as the placement. An explicit pose
at the origin is a real value, not "unset"; a partial object defaults its missing parts to 0; non-finite or wrongly typed
components are pathed `ParseError`s (`sim.startPose.yM`). The editor writes it (numeric fields, dragging the footprint marker),
the run view's "Set start pose from here" too; the wizard keeps it when basics change (`updateRobotConfig` starts from the
existing config).

### Validation codes (`Issue::code`)

Optional root `motionLinks` records describe motion links. For example:
`{"id":"left_front","source":{"owner":"drivetrain","axis":"left1.drive"},
"target":{"owner":"drivetrain","axis":"left0.drive"},"ratio":1}`.
Ratios are target SI units per source SI unit (rad or m).
`motion_link_invalid` reports invalid endpoints/ratios, cycles, duplicate followers, and
conflicting motor mappings. Follower motor slots are not required.
See [ADR 0012](decisions/0012-motion-links.md) for the runtime contract.

SDK codes (`device_alias_conflict`, `attachment_*`, `mechanism_id_reserved`) plus, in `config/config_validator.h`
(`issue_code::`): `schema_version_unsupported`, `chassis_invalid`, `drivetrain_kind_unknown`, `tire_model_unknown`
(needs a `TireModelRegistry`), `drivetrain_no_wheels` (W), `wheel_invalid`, `wheel_duplicate`, `mechanism_id_invalid`,
`mechanism_id_duplicate`, `mechanism_type_unknown`, `property_invalid`, `slot_owner_unknown`, `slot_owner_mismatch`,
`slot_unknown` (W for the `robot` owner), `slot_required_unmapped` (Error for mechanisms, Warning for drivetrain slots),
`slot_duplicate_mapping`, `device_duplicate_mapping`, `slot_device_incompatible` (needs a capability lookup, unknown
devices are skipped), `battery_invalid`, `sim_invalid` (step must divide 20 ms), `plugin_not_found` (W),
`plugin_section_unknown` (W), `plugin_section_version_unsupported`, `plugin_property_invalid`, and the zone codes of
"Zones" below. The SDK's `start_pose_invalid` (`sdk::validation_code::kStartPoseInvalid`, path `sim.startPose`) is an Error for a
non-finite component and a Warning when |x| or |y| is above 1000 m.
Drivetrain slot names follow the convention `"<wheel>.drive"`, `"<wheel>.drive2"`.. (one per `driveMotorCount`, up to 8) / `"<wheel>.steer"` (steer for swerve/steered wheels), robot slot `"imu"`.
A mapping may carry `"reversed": true` (Motor slots only, omitted when false): the controller spins against the mechanism's positive direction, and the hub negates its command and rotor feedback ([ADR 0014](decisions/0014-multi-motor-gearboxes.md)). New codes: `slot_motor_inactive` (W, a mapped motor above the motor count), `slot_reversed_invalid` (W, `reversed` on a non-motor slot).

### Zones (`RobotConfig::zones`)

A zone ([ADR 0016](decisions/0016-zones.md), `sdk/config/zone.h`) is a game-owned place on the robot (a pickup volume,
a muzzle): `kind` and `properties` belong to the plugin that owns the kind (`<plugin id>.<name>`, declared through
`IGameModule::contributeZoneKinds()` as `ZoneKindInfo`), core only handles structure. It hangs on the chassis or a
mechanism link through `attachment` (same shape and link rules as a mechanism) at `pose`; `sizeM` is the box of a
`Volume` kind. `activation` lists conditions on values mechanisms publish (`mechanism`, `value` = a `NamedValue` name,
`op` = `above|below|within`, SI `threshold`, `tolerance` for `within`, `absolute` compares `|v|`) combined with
`mode` `all` (AND) or `any` (OR); no conditions = always active. Evaluate with `evaluateActivation`
(`sdk/game/zone_activation.h`, an unpublished value falls back to the mechanism's `active` flag). `zones` may be
absent on load (= none) and is always written. The pipeline fills `properties` defaults from the kind's schema.

Validator codes: `zone_id_invalid` (empty id), `zone_id_duplicate` (also an id equal to a mechanism id),
`zone_kind_not_owned` (no plugin selected, or the kind lacks the selected plugin's `<id>.` prefix), `zone_kind_unknown`
(W, owned prefix but not declared; properties are not checked), `zone_property_invalid`, `zone_geometry_invalid`
(non-finite pose, or a Volume size <= 0), `zone_condition_invalid` (empty value name, non-finite threshold, negative
tolerance), `zone_condition_mechanism_unknown` (W). Attachment problems reuse `attachment_parent_missing` /
`attachment_link_missing` with the path `zones[i].attachment`.

### Load pipeline

```cpp
RobotConfigCodec codec; ConfigValidator validator; PluginRegistrar registrar(mechanismFactory);
ConfigPipeline pipeline(codec, validator, registrar, drivetrains, plugins);
Result<LoadedConfig> r = pipeline.load(value);     // or pipeline.process(config) for store-loaded configs
// r->config (defaulted), r->issues (check r->valid()), r->pluginId
```
Fatal errors (parse error, a core or plugin-section `schemaVersion` that is not the supported one, a plugin that fails to register) are
`Error`s; everything else is an issue. A missing section of the selected plugin is created with schema defaults.

### Storage (`config_store.h`)

* App data root: Windows `%APPDATA%\WpiEmulator`, macOS `~/Library/Application Support/WpiEmulator`, Linux
  `$XDG_DATA_HOME/wpi-emulator` (fallback `~/.local/share`). Per project: `projects/<folder>-<fnv1a64(canonical path)>/robot.json`
  (case-folded on Windows). Export/import: `<project>/.wpiemu/robot.json`.
* Writes are atomic: temp file in the same directory, flushed (`fsync`/`_commit`), renamed over the target.
* `ProjectConfigRepository::open` = app-data copy, else the project export (cached into app data); `save(..., alsoExport)`;
  `exportToProject`, `importFromProject` (backs the working copy up first; backup failure aborts import
  without modifying it). Failed atomic replacement retains the rename error even after scratch-file cleanup.

## 2. Device hub (`DeviceHub`)

`IDeviceHub` + `IDeviceHubIngress`, **sim thread only**. The IPC thread never touches it: it posts into a `MessageInbox`.
* Resolution ignores the bus (`PhysicalDeviceId`); aliases/conflicts/bus-mismatch follow ADR 0006 exactly like `FakeDeviceHub`.
* Warnings are **derived** from the current state after every change (so they are de-duplicated and disappear by themselves):
  `unmapped_loopback`, `slot_incompatible` (needs `declareSlots`), `slot_missing_device` (after `markDiscoveryComplete`),
  `device_alias_conflict`, `device_aliased`, `device_bus_mismatch`.
* Signals are addressed by announced name: outputs `dutyCycle|pwmSpeed`, `motorVoltage` (else `duty x battery`), `torqueCurrent`,
  `digitalOut|solenoid`; inputs `rotorPosition|position`, `rotorVelocity|velocity`, `rotorAcceleration`, `forward/reverseLimit`,
  `digitalIn|detected`, `analogVoltage`, IMU signals. Frames are delta-compressed (`buildSensorFrame(out, full)`).
* Bindings are keyed by `DeviceKey` and survive `clearDevices()`; `loadMappings(config.allMappings())` / `exportMappings()` persist them;
  `mappingRevision()` changes whenever the loopback set may have changed.
* `resetSignalHistory()` preserves descriptors, discovery and mappings while invalidating all cached
  actuator output and zeroing sensor history. The next sensor frame is forced full: previously-written
  inputs are sent neutral unless overwritten by fresh feedback, including optional inputs no longer
  written after reset. Subsequent frames resume ordinary delta compression.

## 3. Simulation (`SimulationSession`, `SessionRunner`)

`SessionBuilder::build(SessionParts)` takes **already constructed collaborators** (world, registries, plugin registry, optional
`ChassisBuilder`, optional `ICouplingDriver`) and builds, in this order: game field -> chassis -> (coupling bind) -> tire + drivetrain -> slots +
persisted mappings -> mechanisms (config order) -> game state; destruction is the reverse. It owns `SimClock`, `Rng(seed)`, `DeviceHub`,
`BatteryModel` and the `SimContext`.

`SimulationSession::tick(SensorFrame&, full)` is one pass of ARCHITECTURE 4 (steps 2-6): mechanisms -> drivetrain commands from `drivetrain.<wheel>.drive[N]|steer` (`sdk::wheelCommandFromHub`: every drive motor with its own voltage)
slots (rotor feedback written back with the gear ratio) -> `coupling.beforeStep` -> `world.step` -> `coupling.afterStep` -> game state -> game `onTick` -> chassis IMU
(`robot.imu`: yaw/pitch/roll, body rates, proper acceleration = +g at rest) -> battery. The battery uses the total current of the tick
(mechanisms + wheel motors + `baselineCurrentA`) with a one-tick lag: `V = nominal - I R`, brownout with a 0.5 V hysteresis (warning `brownout`).
Tick numbers restart at 0 per epoch; `reset()` restores pose (the start pose, see above), drivetrain, mechanisms, game state, clock, RNG and battery but keeps devices and bindings.
`setRobotPose(Pose3)` is the teleport behind the UI's drag-to-place: it moves the chassis there, zeroes its linear and angular velocity, resets the drivetrain's wheel state and the IMU's velocity history
(no acceleration spike, only a jump in the heading) and leaves mechanisms, game state, clock, RNG and the mechanism coupling alone; a non-finite pose is ignored with a warning.
It first resets hub signal history, so the first tick of every new epoch consumes neutral commands rather
than outputs accepted in the prior run (including repeated Armed End/Begin transitions).

`SessionRunner` (implements `ISimController`) drives a session over a link; one `poll()` = one bounded iteration, `start()` runs it on a thread.
* **Handshake**: `Hello` -> `HelloAck{sessionId, runEpoch, stepUs, negotiated watchdog, policy}` (version below `kMinCompatibleProtocolVersion` is refused),
  then `LoopbackSet` and `DsState`. Reconnect = new session id, new epoch, reset sim, devices cleared.
* **Lock-step**: `LockstepWindow` (one outstanding tick), stale/foreign frames dropped and counted, ack verdicts handled (`Stalled`, `Stopped`, `ProtocolError`).
* **Commands**: Pause/Resume/Step/SetTimeScale are local; Reset bumps the epoch, sends `SimControl{Reset}` and blocks ticking until it is acknowledged;
  Stop is acknowledged (or abandoned after the watchdog) and ends the run. DS state, live `bindSlot/unbindSlot` and `setRobotPose` are queued and applied on the sim thread at the start of the next `poll()`
  (so also while paused or before a robot connected; a robot that connects afterwards starts a new epoch, which resets the pose to the start pose) and every applied command marks the snapshot dirty.
* **Watchdog**: tick outstanding longer than the negotiated timeout => `RobotStalled`, no more frames until Reset/reconnect; late answers are ignored.
* **Pacing**: `timeScale` 1 = real time (no catch-up burst after a stall), 0 = as fast as the robot answers.
* **Brownout**: the DS state sent to the robot is forced to disabled while browned out and restored afterwards.
* **DS state semantics**: `DsState::matchTimeS` is seconds *remaining* in the current period (WPILib convention), `-1` = no timed match
  (consumers integrate sim time themselves); the runner stamps `sessionId`/`runEpoch`. `gameSpecificMessage` is the UI's manual text unless the loaded
  game plugin's `IGameState::gameSpecificMessage()` returns a value (FMS game data, e.g. the 2026 hub order `R`/`B`), which then overrides it; the runner
  re-sends the `DsState` after the tick in which the value changes and after Reset.
* **Snapshots**: published to the `ISnapshotSink` every `snapshotEveryTicks` and on every state change (robot connected, paused, device list...).
* Link layer: `IRobotLink`, `MessageInbox`, `ChannelRobotLink` (any `ITransport` + `IMessageCodec`), `RobotLinkAcceptor` (any `ITransportServer`). Only sdk types are used,
  the app plugs in the ipc module's server/codec.
  `RobotLinkAcceptor::stop()` closes the listener, joins the accept thread, then closes and joins its accepted readers.
  `MessageInbox::close()` permanently rejects new events, wakes waiters, and moves pending connection owners out under
  the mailbox lock before closing/destroying them unlocked. Readers retain private mailbox state, not a borrowed mailbox
  reference, so late disconnect notifications are safely rejected even after the mailbox owner is destroyed. Stop
  producers before disposing of the consumer; never join a reader while holding the mailbox lock.

### Composition root recipe

```cpp
physics::registerPhysicsComponents(drivetrains, tireModels);
PluginRegistrar registrar(mechanismFactory);  registrar.ensureRegistered(*selectedPlugin);
SessionParts p;  p.config = loaded.config;  p.world = physics::createBox3dWorld(p.config.sim.physics);
p.mechanisms = &mechanismFactory; p.drivetrains = &drivetrains; p.tireModels = &tireModels; p.plugins = &plugins; p.logger = &logger;
p.chassisBuilder = runtime::physicsChassisBuilder();                 // runtime/physics_wiring.h
p.coupling = std::make_unique<runtime::PhysicsCouplingDriver>();     // runtime/physics_wiring.h
auto session = SessionBuilder::build(std::move(p));
MessageInbox inbox;  LatestSnapshotSlot slot;
RobotLinkAcceptor acceptor(ipc::makeServer(...), ipc::makeCodec(), inbox, &logger);  acceptor.start();
SessionRunner runner(**session, inbox, slot, logger, options);  runner.start();   // UI: runner.control(...), slot.latest()
```

## 4. Launching the robot (`core/launcher`)

`RobotLauncher` starts the project **without editing it** and supervises it (no orphans: the destructor kills the tree).
Its worker exclusively owns the process through startup, waiting and cleanup. `requestStop(grace)` latches cancellation
without joining; repeated requests preserve the earliest whole-tree kill deadline. The worker observes cancellation
after successful startup and around each bounded 100 ms process wait, first requesting a polite stop, then killing the
tree when the grace deadline expires. `stop(grace)` requests cancellation and joins the worker, so cancellation during
native startup cannot be lost before PID/job publication.

| Mode | Command | Notes |
|---|---|---|
| `GradleInitScript` | `gradlew[.bat] --no-daemon --console=plain [--offline] -I <scratch>/wpiemu-init.gradle simulateJava|simulateNative` | The init script (ADR 0001) is generated by `generateInitScript()`, identical every run, no paths inside. `--no-daemon` keeps the process tree ours. |
| `ExternalDirect` | `gradlew simulateExternal{Java,Native}{Release,Debug}` (build + describe), then `java -Djava.library.path=<libraryDir> -jar build/libs/<fat jar>` or `<launchfile>` | Parses `build/sim/<release|debug>_<java|native>.json`; our `HALSIM_EXTENSIONS` replaces the stock Sim GUI one; library dirs are prepended to `PATH` / `LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH`; macOS adds `-XstartOnFirstThread`. |

Environment contract (`launcher::env_var`): `WPIEMU_EXT`, `WPIEMU_IPC_ENDPOINT` (+ alias `WPIEMU_ENDPOINT`),
`WPIEMU_CLOCK=master|realtime`, `WPIEMU_PROGRAM_NAME`, `JAVA_HOME` (WPILib JDK), `HALSIM_EXTENSIONS` (external mode). The robot PID for emergency kills comes from `Hello`
(`launcher::killProcessTree(pid)`: Windows process walk, POSIX group + pid; refuses to kill the emulator or its ancestors).

`IRobotProcess` / `createSystemProcess()`:
* **Windows**: `CreateProcessW` suspended into a kill-on-close **job object**, then resumed; exactly three inheritable handles (`PROC_THREAD_ATTRIBUTE_HANDLE_LIST`);
  `.bat/.cmd` run through `cmd /d /s /c "..."` and arguments with cmd metacharacters (`" % ^ & | < > !`) are refused; `CTRL_BREAK` is only a best-effort polite stop
  (the clean path is the protocol `Stop`).
* **POSIX**: `fork` into a new process group (SIGTERM / SIGKILL via `killpg`), exec failure reported through a CLOEXEC pipe, everything prepared before `fork`.
* Native startup, PID/handle publication and stop/kill requests are serialized. Calling `kill()` before startup cancels
  a later `start()` instead of allowing an unsupervised child to appear.
* Output: stdout/stderr line-split into an `ILogSink` (CRLF aware, 64 KiB line cap); after the main process exits the readers get 1.5 s, then pipes held open by grandchildren are cancelled.

### `ProjectInspector` and `inspectToolchain` (wizard steps 1-2)

Read-only: `build.gradle[.kts]` (GradleRIO version in Groovy, Kotlin and `classpath` forms -> year; variable version -> `.wpilib` `projectYear` / `settings.gradle` `frcYear`),
language (`.wpilib` `currentLanguage` > build script signals > `src/main/{java,cpp}`), team number, Gradle wrapper files + Gradle version, `vendordeps/*.json`.
`inspectToolchain` produces `ToolchainCheck`s (`project`, `gradlerio`, `game_plugin`, `gradle_wrapper`, `wpilib_install`, `jdk`, `cpp_toolchain`, `vendordeps`, `team`, `extension`) with
`Ok/Info/Warning/Error` and a fix hint; `ready()` = no Error. WPILib is searched in `%PUBLIC%\wpilib\<year>` / `C:\Users\Public\wpilib\<year>` and `~/wpilib/<year>`; the JDK is
the WPILib one, else `JAVA_HOME`, else `java` on `PATH` (Java >= 17 expected). It was verified against `FRC-Shelby-2026OS` (read-only; a test asserts nothing changed).

## 6. Running it for real: `app/host` (`RobotSession`) and `RealRuntimeHost`

The composition root of the product lives in the app, in a Qt-free library (`app/host`, target `wpiemu_app_host`) so the shell and the end-to-end
test share one implementation. `RobotSession::start(registries, config, options)` does, in this order:

1. cheap checks with actionable errors: project folder, **Gradle wrapper present** (`gradlew[.bat]`), HALSim extension file;
2. `ConfigPipeline::process` (defaults, validation): errors refuse a **Run**, a **Discovery** only warns (and falls back to an empty
   model if the session cannot be built from a half-edited one);
3. game plugin by `gamePluginId` / season (`selectPlugin`) + `PluginRegistrar`; **no plugin for the season = core-only mode** (a floor slab replaces the
   plugin field, a log line says so); the robot starts at `sim.startPose` when the config sets one, else inside the plugin's field (`startPlacementFor`), else at the origin;
4. `SessionBuilder::build` over a Box3D world, `physicsChassisBuilder` and `PhysicsCouplingDriver`;
5. IPC: `ConnectionEndpoint::generate()` -> `makeLocalServer` -> `RobotLinkAcceptor` (binary codec) feeding the `MessageInbox`;
6. `SessionRunner` thread (`awaitProgramStart = true`: no tick and no watchdog until the extension's `program_started` lifecycle Log, because a robot
   constructor may take longer than the watchdog; `discoveryOnly` now means the session starts **Armed** and switches to lock-step on `BeginLockstep`, see ADR 0011);
7. `RobotLauncher` (`GradleInitScript` default, `ExternalDirect` = `simulateExternal*` + direct JVM/exe launch; selected with `WPIEMU_LAUNCH_MODE=direct`
   in the app). The JDK is the WPILib one of the project year. `WPIEMU_LIVE_DISCOVERY` is passed explicitly (`0` unless `Options::preStartPolling` /
   `WPIEMU_PRESTART_POLLING=1`): devices are announced when the robot program starts.

`status()` is a pure function of the launcher phase, the runner state and elapsed time; it is what the UI polls:

| Situation | `process` / `link` | Detail |
|---|---|---|
| gradle building / JVM starting | `Starting` / `Connecting` | "Building and starting the robot project..." |
| extension connected | `Running` / `Connected` (`programStarted` once the constructor finished) | |
| robot stops answering (watchdog) | `Running` / `Stalled` | |
| process ends before the extension connected (build failure...) | `FailedToStart` | exit code + last 40 console lines |
| no handshake within `handshakeTimeout` (5 min) | `FailedToStart` / `HandshakeTimeout` | |
| protocol too old (`SessionRunner::refusal()`) | `FailedToStart` / `ProtocolMismatch` | |
| robot dies after connecting | `Crashed` / `Disconnected` | exit code + console tail |
| user Stop | `Stopping` -> `Exited` | protocol `Stop`, `stopGrace`, then the process **tree** is killed |
| app exit / session destroyed | - | the destructor always kills the tree (job object) |

After protocol-stop grace and launcher tree cleanup, `RobotSession` stops the acceptor and joins IPC readers before
stopping the runner and closing the inbox. Pending, unconsumed connections are therefore released while mailbox
shutdown state is still alive, without holding the queue lock during reader joins.

`RealRuntimeHost` (`app/common`, the default `IRuntimeHost` of `AppServices`; `--demo` keeps `DemoRuntimeHost`) owns one `RobotSession` and turns its
thread-safe state into UI signals with a 30 ms poll (`statusChanged`, `snapshotPublished`, batched `logLine`). The extension is located by
`core::launcher::locateHalsimExtension` (`docs/packaging.md`: `WPIEMU_EXTENSION`, `WPIEMU_EXTENSION_DIR`, install layouts, then the build-tree path
CMake bakes in). Starting in the other mode, restarting, or switching project stops the current run synchronously (bounded by `stopGrace`).

`SessionRunner` additions for this: `RunnerOptions::awaitProgramStart`, `programStarted()`, `refusal()`, `robotLoopCount()` (the extension's loop counter).
`planExternalLaunch` accepts GradleRIO's real descriptor shape (a **list** of targets).

## 7. Testing

`tests/runtime/` (140+ cases): JSON edge cases, codec round trips/schema version checks, validator and pipeline, atomic writes and
backups, hub behaviour (aliases, loopback, delta frames), session ordering/determinism/battery/IMU/coupling, runner with a scripted fake extension over the SDK in-memory
transport (handshake, lock-step, pause/step/reset, watchdog, reconnect, stop, brownout, pacing, thread), launch planning, real process supervision
(capture, env, kill tree, destructor, batch files), inspector/toolchain on synthetic projects and the real Shelby project, and (when `core/physics` is in the build) a
Box3D swerve integration test. The same sources compile warning-free with MSVC `/W4 /WX` and g++ `-Wall -Wextra -Wpedantic -Werror` and pass on Windows and Linux (WSL).

Above the runtime: `wpiemu_host_tests` (`app/host/tests`, CTest `host.*`) drives `RobotSession` against synthetic projects whose `gradlew` is a tiny script
(no Gradle wrapper, bad model, build failure, handshake timeout, protocol mismatch through a fake extension client, crash after connect, user stop, core-only
season, destructor kills the tree), `app.ui_tests` covers `RealRuntimeHost` and the window the same way, and the opt-in **end-to-end test** (CTest label `e2e`,
`app/host/tests/e2e_test.cpp`) runs the unmodified FRC-Shelby-2026OS project: discovery (29 devices), a generated swerve + mechanism (arm) model, DS enabled in
teleop, real-time then free-running sim, loop count vs sim time, wheel/chassis/pivot response, stop and process cleanup. See docs/BUILDING.md.
