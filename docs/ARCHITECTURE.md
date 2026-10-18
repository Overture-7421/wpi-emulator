# Architecture

Status: SDK **1.0**, IPC protocol **1**. Until release the SDK stays at version 1 and no compatibility is kept
([ADR 0013](decisions/0013-pre-release-single-sdk-version.md)); the interfaces were first laid out in
[ADR 0005](decisions/0005-sdk-freeze.md) and extended by [ADR 0006](decisions/0006-sdk-1-1-amendments.md) and
[ADR 0011](decisions/0011-single-robot-instance.md). All modules below are implemented;
this document describes the code as built. Background and rationale: ADRs
[0001](decisions/0001-halsim-extension-injection.md) (HALSim injection),
[0002](decisions/0002-phoenix6-interception.md) (Phoenix 6 via SimDevices),
[0003](decisions/0003-box3d-chassis-physics.md) (Box3D chassis physics),
[0004](decisions/0004-modular-boundaries.md) (module boundaries),
[0005](decisions/0005-sdk-freeze.md) (SDK freeze), [0006](decisions/0006-sdk-1-1-amendments.md) (SDK amendments),
[0007](decisions/0007-ipc-transport.md) (IPC transport), [0008](decisions/0008-physics-implementation.md) (physics),
[0009](decisions/0009-mechanisms-impl.md) (mechanisms), [0010](decisions/0010-runtime.md) (runtime / config / launcher).
Full index: [decisions/README.md](decisions/README.md). How to build and contribute:
[BUILDING.md](BUILDING.md), [CONTRIBUTING.md](CONTRIBUTING.md); using the product: [USER_GUIDE.md](USER_GUIDE.md).

### Documentation map

| Topic | Document |
|---|---|
| Runtime, `robot.json`, device hub, session runner, launcher, `RobotSession` | [runtime.md](runtime.md) |
| Generic mechanisms, plants, motor catalog | [mechanisms.md](mechanisms.md) |
| HALSim extension (robot process side) | [halsim-extension.md](halsim-extension.md) |
| HAL bridges / vendor (CTRE) bridges | [bridges-hal.md](bridges-hal.md), [bridges-vendor.md](bridges-vendor.md) |
| 2026 game plugin / writing a new season | [game_2026.md](game_2026.md), [new-year-plugin.md](new-year-plugin.md) |
| App shell, editor, run view | [app/README.md](../app/README.md), [app/editor/README.md](../app/editor/README.md), [app/runview/README.md](../app/runview/README.md) |
| Installers and releases | [packaging.md](packaging.md) |

## 1. Modules and the dependency rule

```
                       +--------------------------------------------+
                       |  wpiemu_sdk  (header-only, std only)       |
                       |  interfaces, data models, registries, fakes|
                       +--------------------------------------------+
                          ^        ^        ^         ^         ^
          +---------------+        |        |         |         +----------------+
          |                +-------+        |         +-------+                  |
   +------------+   +-------------+   +------------+   +--------------+   +--------------+
   | core       |   | ipc         |   | bridges/   |   | plugins/     |   | halsim_ext   |
   | physics,   |   | transports, |   |  hal       |   |  game_<year> |   | (robot proc) |
   | mechanisms,|   | codec       |   |  vendor    |   |  _template   |   | uses sdk+ipc |
   | runtime,   |   +-------------+   +------------+   +--------------+   +--------------+
   | config     |          ^                                   ^
   +------------+          |                                   |
          ^                |                  +----------------+
          |                |                  | plugins/builtin_plugins.cpp
   +------+----------------+--------+         | (aggregation seam, target wpiemu_plugins_builtin)
   | app (Qt): wizard, editor,      |---------+  app links ONLY this, never a wpiemu_plugin_*
   | run view, DS panel, launcher   |
   +--------------------------------+

   bridges/builtin_bridges.cpp (target wpiemu_bridges_builtin) is the same seam for device bridges:
   it links bridges/hal + bridges/vendor, and only halsim_ext links it (enforced as a distinct
   aggregation owner by tools/check_boundaries.py).
```

| Module | May depend on | Notes |
|---|---|---|
| `sdk` | nothing (std only) | no Qt, no Box3D, no HAL/WPILib headers; header-only INTERFACE |
| `core` | sdk (+ Box3D privately) | game-agnostic; knows no season. Sub-folders: `src` (core info), `physics` (Box3D adapter, tire models, drivetrains, ADR 0008), `mechanisms` (generic templates, plants, motor catalog, ADR 0009), `runtime` (config codec/pipeline/store, device hub, `SimulationSession`/`SessionRunner`, launcher, project inspector; ADR 0010). One target, `wpiemu_core` |
| `ipc` | sdk | implements `ITransport` / `IMessageCodec` (named pipe / unix socket + binary codec, ADR 0007) |
| `halsim_ext` | sdk, ipc, bridges aggregator (+ HAL headers) | `halsim_wpiemu`, runs inside the robot process |
| `bridges/hal`, `bridges/vendor` | sdk (+ HAL / vendor headers) | not each other. `hal` = WPILib HAL families; `vendor` = CTRE Phoenix 6 / Phoenix 5 through SimDevices |
| `bridges/` aggregator | sdk + every bridge module | only `wpiemu_bridges_builtin`, linked only by `halsim_ext` |
| `plugins/<name>` | sdk only | no core, app, ipc, bridges, or other plugin. Shipped: `game_2026`; `_template` (compiled, never aggregated) |
| `plugins/` aggregator | sdk + every plugin | only `wpiemu_plugins_builtin` |
| `app` | sdk, core, ipc, aggregator target, Qt | never includes/links a plugin directly. Sub-folders: `common` (models, `AppServices`, `RealRuntimeHost`), `wizard`, `shell`, `editor`, `runview`, `host` (Qt-free `RobotSession`), `resources`, `tests` |

Enforced mechanically by `tools/check_boundaries.py` (CTest `boundaries.check`): include and
`target_link_libraries` rules, explicit implementation-target ownership (unknown first-party targets
are errors), declaration/map completeness, plus a guard that headers do not use identifiers Qt turns into
macros (`signals`, `slots`, `emit`, `foreach`).
Imported `wpiemu_wpilib_hal` / `wpiemu_wpilib_wpiutil` targets are third-party robot-process
backends used by `halsim_ext`, not SDK dependencies or alternative bridge aggregation seams.

## 2. SDK file map (`sdk/include/wpiemu/sdk/`)

| Dir | Headers | Purpose |
|---|---|---|
| `core/` | `interface.h` `result.h` `math.h` `units.h` `ids.h` `value.h` `logger.h` `clock.h` `rng.h` `registry.h` `geometry.h` `severity.h` `sim_context.h` | Basics every module uses; `SimContext` |
| `physics/` | `physics_types.h` `physics_world.h` `tire_model.h` `chassis_spec.h` `drivetrain.h` | `IPhysicsWorld`, `ITireModel`, `IDrivetrain`, `ChassisSpec`/`WheelSpec` |
| `dynamics/` | `motor_model.h` `plant.h` | `MotorSpec`/`IMotorModel`, `IPlant`/`PlantFactory` |
| `devices/` | `device_descriptor.h` `device_identity.h` `signal.h` `signals.h` `slot.h` `frames.h` `discovered_device.h` `device_hub.h` `device_bridge.h` `sim_device_bridge.h` `channel_family_bridge.h` `builtin_bridges.h` | Device model (declared address vs physical identity), hub, bridges, the `registerBuiltinBridges` aggregation seam |
| `mechanisms/` | `mechanism.h` `mechanism_state.h` `physics_coupling.h` `attachment.h` | `IMechanism`, `MechanismFactory`, template metadata, chassis coupling, link attachment |
| `game/` | `game_module.h` `game_state.h` `field.h` `ui_descriptors.h` `zone_kind.h` `zone_activation.h` `plugin_registry.h` `builtin_plugins.h` `null_objects.h` | Year-plugin contract, zone kinds and activation evaluation |
| `config/` | `robot_config.h` `mechanism_attachment.h` `zone.h` `property_schema.h` `plugin_config.h` `value_codec.h` `config_io.h` `validation_codes.h` | Persisted model (incl. zones), schemas, stable issue codes |
| `ipc/` | `messages.h` `transport.h` `lockstep.h` `lifecycle.h` | Protocol structs, `ITransport`, `IMessageCodec`, `MessageChannel`, lock-step state machines, lifecycle signals carried as `Log` messages (e.g. robot program started) |
| `telemetry/` | `snapshot.h` `sim_controller.h` | `ISimSnapshot` view-model, `ISimController` |
| `testing/` | `fake_physics_world.h` `fake_device_hub.h` `fake_game_module.h` `fake_sim_environment.h` `fake_clock_logger.h` `in_memory_transport.h` `fake_mechanism_physics_coupling.h` `fake_mechanism_directory.h` | Header-only fakes for isolated tests |

Conventions: SI units; right-handed **Z-up NWU** (+X toward the opposing wall, +Y left, +Z up;
yaw CCW+); quaternion (x,y,z,w); geometry `float`, signals/time/electrical `double`; sim time is
integer microseconds; unit suffixes on plain scalars (`massKg`, `radiusM`, `timeUs`).

### Error handling (one approach)
`Result<T>` / `Status` (`core/result.h`) for every recoverable failure. **No exception may cross an
SDK interface**; catch at the boundary of third-party code (JSON, Qt, regex) and convert. Contract
violations (bugs) abort via `contractViolation`. Hot per-tick calls return neutral values for
invalid ids instead of errors (documented per interface).

### Ownership
`std::unique_ptr` for owning relationships; registries own factories (or modules for plugins);
`SimContext` and `DrivetrainBuildArgs` hold **non-owning** references whose owners outlive the
borrower (runtime destroys in reverse construction order: game state, mechanisms, drivetrain,
field, then physics world). No singletons or globals: registries and `SimContext` are instances
created by the composition root (app or test).

## 3. Processes, threads, and ownership

The SDK has optional one-way motion links ([ADR 0012](decisions/0012-motion-links.md)).
Linked sessions evaluate advertised wheel/mechanism axes in dependency order before publishing
mechanism state and stepping the world. Followers use prescribed motion without motors or
back-propagated load; independent sessions keep the original tick pipeline.

```
 Emulator process (Qt app)                                 Robot process
 +---------------------------------------------+          +---------------------------------+
 | UI thread: Qt widgets / Quick 3D            |          | robot threads (HAL callbacks)   |
 |   reads ISimSnapshot (immutable)            |          |   only record events            |
 |   writes via ISimController (thread-safe)   |          | clock thread: HALSIM_StepTiming |
 +---------------------^-----------------------+          |   bridges: poll/collect/apply   |
   LatestSnapshotSlot  |  command queue                   | IPC thread: socket I/O          |
 +---------------------+-----------------------+          +----------------^----------------+
 | SIM THREAD (owns everything below)          |  SensorFrame ->           |
 |  IPhysicsWorld, field, drivetrain,          |  <- ActuatorFrame, Announce  (ITransport)
 |  mechanisms, IDeviceHub, IGameState, RNG    |<--------------------------+
 | IPC thread: ITransport -> queue -> sim      |
 +---------------------------------------------+
```

* **Emulator = clock master** (ADR 0001): fixed step (`RobotConfig::sim.stepUs`, default 5 ms = 200 Hz; it must divide the
  20 ms robot period; the Settings dialog offers 100-1000 Hz and the setup wizard writes it into the config), lock-step
  with the robot via `HALSIM_StepTiming`. Sim time is exact integer microseconds from `ISimClock`; nothing reads a wall clock.
* **Sim thread owns** physics, mechanisms, hub, game state, config, RNG. Nothing in `SimContext`
  is thread-safe. The UI never touches them: it reads `shared_ptr<const ISimSnapshot>` and sends
  commands through `ISimController` (enqueued, applied at the start of the next tick).
* **Extension side**: HAL callbacks (robot threads) only record events; the clock thread calls
  `IDeviceBridge::poll/collectOutputs/applyInputs/applyLoopback` while the robot is quiescent
  (between `HALSIM_StepTiming` calls). Socket I/O lives on one IPC thread. Never block in a HAL callback.
* Determinism: single physics worker, fixed dt, `Rng` seeded from `RobotConfig::sim.seed`,
  `IPhysicsWorld::stateHash()` for regression tests.

## 4. Per-tick data flow (emulator <-> extension, ADR 0001 / 0002)

Discovery (once, then incremental): `Hello` / `HelloAck`, then `DeviceAnnounce` per device (any
time, idempotent by `DeviceKey`). The hub binds announced devices to slots from `RobotConfig`;
live devices without a slot are put in `LoopbackSet` (extension echoes output to input, no IPC hop)
and a `HubWarning{"unmapped_loopback"}` is raised.

One tick `N` (sim thread unless noted):

1. **Ingest** the newest `ActuatorFrame` from tick N-1 into `IDeviceHubIngress` (frames whose `runEpoch`
   is stale are dropped, §4.1); apply queued UI commands (`DsState`, `SimControl`, mapping changes).
2. **Mechanisms** in config order: `IMechanism::tick` reads `MotorOutput` per slot from the hub,
   steps its `IPlant`, writes encoder/limit feedback back to the hub (absolute truth).
   Mechanisms also **submit** to `ctx.coupling()` (chassis wrench, anchored constraints, payload mass)
   and may **read** `constraintLoad()` = the load of the previous physics step (§4.2).
3. **Drivetrain** `IDrivetrain::step(WheelCommand[])` -> `ITireModel` -> `applyForce` on the chassis.
   The runtime then composes chassis + payload mass, hands the wrenches to the world and (re)creates constraints.
4. **Physics** `IPhysicsWorld::step(dt)`; the runtime stores the step's constraint reactions.
5. **Game** `IGameState::step` (sees this step's sensor/contact events), then `IGameModule::onTick`.
6. **Egress**: chassis IMU written to the hub; battery model from `IMechanism::batteryCurrentA()` +
   drivetrain currents; `buildSensorFrame` -> `SensorFrame{tick N, stepUs, batteryVoltageV, samples, sessionId, runEpoch}`.
7. **Snapshot** built (`SnapshotBuilder`, mechanisms/game state `publish`) and handed to `ISnapshotSink`.
8. **Extension**, on receipt: bridges `applyInputs` (writes `HAL_SetSimValue`, `HALSIM_Set*`),
   `HALSIM_StepTiming(stepUs)` (robot loop runs), bridges `collectOutputs`,
   `ActuatorFrame{tick N, ackTick N, result, sessionId, runEpoch}` sent back. Exactly one tick is ever outstanding.

Signals are named/typed per device at announce time and addressed by index in frames; canonical
names/units are in `devices/signals.h` (bridges normalise vendor names; `SimDeviceRule::scale`).
Phoenix 6 devices are read/written through HAL SimDevice values (`SimDeviceBridge`, ADR 0002); the
emulator drives the **rotor** (Phoenix firmware sim applies gearing/offsets).

### 4.1 Lock-step robustness
Reference code: `ipc/messages.h` (types, `classifyFrame`, `negotiateWatchdogTimeoutMs`) and
`ipc/lockstep.h` (`LockstepWindow` emulator side, `LockstepFollower` extension side).

* **Window = one outstanding tick** (`kLockstepWindow == 1`). The emulator sends `SensorFrame` N+1 only after the
  `ActuatorFrame` acknowledging N, or after a Reset/reconnect opened a new epoch. No pipelining.
* **Identity.** `sessionId` (one per accepted connection, assigned in `HelloAck`) and `runEpoch` (monotonic, owned by
  the emulator, bumped on every Reset and every reconnect; first value `kFirstRunEpoch` = 1) are carried by
  `SensorFrame`, `ActuatorFrame`, `DsState` and `SimControl`. `0` = unspecified (a peer that does not send it / in-process caller) and is
  accepted. **Receivers drop frames with a stale epoch** (and foreign sessions); `classifyFrame` returns
  `Accept | DropStaleEpoch | DropForeignSession | NewerEpoch`. A newer epoch is adopted by the extension and is a
  protocol error for the emulator. Tick numbering restarts per epoch; never assume monotonic ticks across epochs.
* **Tick ack.** `ActuatorFrame::result` (`Completed | RobotStalled | Stopped | ProtocolError`) says how the tick ended and
  `ackTick` which `SensorFrame::tick` it answers (`ackedTick()` falls back to `tick` for frames that do not send it). `RobotStalled`,
  `Stopped` and `ProtocolError` end lock-step until a Reset/reconnect (`LockstepWindow::stalled()`).
* **Commands.** `SimControl::commandId != 0` is answered by exactly one `SimControlAck{commandId, result, runEpoch, tick}`.
  `Reset` and `Stop` MUST carry a command id. Reset is the first message of the new epoch (its `runEpoch` is already
  bumped; the ack reports the new epoch). Recent command ids are re-acked, never re-applied; older ids
  outside the bounded acknowledgement history are rejected rather than replayed. Foreign-session
  controls are rejected before consulting that history. `Stop` is honoured regardless of epoch
  within its own session; a stale non-Reset command is rejected with `ControlResult::StaleEpoch`.
* **Watchdog.** `Hello::proposedWatchdogMs` and `HelloAck::watchdogTimeoutMs` negotiate one timeout (the larger of the two
  preferences, clamped to 100 ms..60 s, default 2 s) used by both sides. On timeout **or disconnect the extension MUST itself
  unblock the robot**: `HALSIM_ResumeTiming` (`WatchdogPolicy::ResumeRealtime`, default) or terminate (`WatchdogPolicy::Terminate`).
  It never waits for the emulator to do so. The emulator reports `RobotStalled`, stops sending frames and never blocks the
  sim thread past the watchdog. The wall-clock itself is the transport/loop's business (no clock in the SDK state machines).
* Versioning: `kProtocolVersion` is 1 and `kMinCompatibleProtocolVersion` is 1; codecs default absent trailing fields. Message
  `SimControlAck` (type 11) is part of the `proto::Message` variant.
* **Armed capability.** `Hello::supportsArmed` defaults to false.
  Only peers explicitly advertising it receive `BeginLockstep` / `EndLockstep`. An Armed
  (`discoveryOnly`) handshake requires it; other peers can still use normal lock-step.
  Armed keeps the one robot process alive with announcements/loopback and free-running HAL time;
  Begin starts a new epoch at t = 0, End returns to Armed, and Stop ends the connection.
* **Input history.** The optional default-no-op `IDeviceBridge::resetInputHistory()` clears bridge
  transform integration/unwrap history on connection/reconnect, Reset, Begin, and adoption of a
  newer sensor epoch. It runs on the clock thread between steps, preserving firmware/user offsets
  and discovery. Core separately clears device signal history on session reset while retaining
  descriptors/mappings, so prior actuator values cannot leak into the new epoch.

### 4.2 Mechanism <-> physics coupling (`mechanisms/physics_coupling.h`)
`SimContext::coupling()` returns an `IMechanismPhysicsCoupling` (never null; a no-op `NullMechanismPhysicsCoupling` when the
runtime binds none, check `capabilities()`). It offers: `applyWrench` (force + couple on the chassis at a chassis-local point,
this step only), `addConstraint`/`removeConstraint`/`constraintLoad` (prismatic / distance / contact anchor between a chassis-local
point and a field point; invalid handle when unsupported; reaction force/torque on the chassis), and `setPayload`/`clearPayload`
(mass, COM, inertia per owner; the runtime composes them with the chassis, `composeMassProperties`).
**Ordering:** mechanisms submit before the physics step; they read the previous step's loads (`constraintLoad` always reports the
last completed step); submissions made after the step apply to the next one. Wrenches live one step, constraints and payloads
persist until removed and are dropped by the runtime on reset (mechanisms re-register from `reset()`). Fake:
`testing/fake_mechanism_physics_coupling.h`, wired into `FakeSimEnvironment`.

### 4.3 Device identity: declared address vs physical device
`DeviceKey` (family/type/id/bus[/channel]) is the **robot-declared address** (robot.json and robot code). `PhysicalDeviceId`
(`DeviceKey::physical()`, `DeviceDescriptor::physicalId()`) is what the **simulator** can tell apart: no bus (CTRE sim state is
keyed by type+id). The hub resolves a declared address to the discovered device with the same physical id; the bus never takes
part in the match. `DiscoveredDevice` gains `observedBus`, `aliasState` (`None | Aliased | Conflict`) and `aliases`.
* `bus` is a label: **never invented from a static `SimDeviceRule`**. CAN-vendor rules set `busIsStatic = false` (descriptor bus
  stays `kBusUnknown`); the real bus only arrives through an L3 observation: `IBridgeHost::observeBus` ->
  `DeviceAnnounce::observedBus` -> `IDeviceHubIngress::observeBus` (mismatch with a declared alias => warning
  `device_bus_mismatch`).
* **Validator contract:** `IConfigValidator` MUST reject independently mapped aliases of one physical device with an Error whose
  `Issue::code == validation_code::kDeviceAliasConflict` (`validateDeviceAliases(config.allMappings())`).
* **Safe behaviour at runtime** (stale configs): a `Conflict` device is treated as unmapped: it goes into the `LoopbackSet`,
  all its slots read neutral data and ignore writes, `HubWarning{Error, device_alias_conflict}` is raised; mappings are kept so
  fixing the config re-enables it. `Aliased` (several declared addresses, one mapping) is only a `device_aliased` warning.

### 4.4 Config load pipeline
Coordinator order (core/app): **decode** (`IRobotConfigCodec::fromValue`, current schema only: a core or plugin-section
`schemaVersion` other than the supported one is refused, there is no migration) -> **defaults** -> **validation**
(`IConfigValidator`) -> **construction**. **Ownership rule:** a plugin owns `config.plugins[its id]` and mechanisms
for which `ownsMechanismType(id, type)` holds (type prefixed `"<id>."`) incl. their mappings/attachment; nothing else (core
fields, other plugins). `Issue::code` carries stable machine keys (`config/validation_codes.h`).

### 4.5 Mechanism attachment
`MechanismConfig::attachment` (`{parentMechanismId = "chassis", parentLink = "root"}`) is the only attachment form
(`attachTo()` sets it). Every mechanism (and the chassis)
has the stable link **`root`**; templates declare more in `MechanismTemplateInfo::links` (`LinkSpec`, rest pose) and publish
live poses in `MechanismState::links` (`IMechanism::linkPose`). `IMechanismDirectory::linkPoseInChassis(id, link)` returns the
resolved transform (`resolveLinkPoseInChassis`). `mount` is relative to the parent **link** frame. Validation contract
(`validateMechanismAttachments`): `attachment_parent_missing`, `attachment_link_missing`, `attachment_cycle`,
`mechanism_id_reserved`; list parents before children for a same-tick-consistent view.

### 4.6 Zones
`RobotConfig::zones` ([ADR 0016](decisions/0016-zones.md), `config/zone.h`): game-owned places (pickup volumes, muzzles)
attached to the chassis or a mechanism link exactly like a mechanism mount, so they follow that link. Core validates
structure only; `kind` (`<plugin id>.<name>`) and `properties` belong to the plugin, which declares its kinds through
`IGameModule::contributeZoneKinds()` (`game/zone_kind.h`). A zone's `activation` (conditions on mechanism-published
values combined with AND/OR) is evaluated with `evaluateActivation` (`game/zone_activation.h`) by whoever consumes the zone.

### 4.7 Plugin ABI caveat
v1 supports **static built-in plugins only** (one toolchain/CRT/flags). The SDK exposes C++ virtual interfaces and
`std::` types, which are not a stable ABI across separately built libraries. Shared-library plugins would need a versioned,
size-prefixed `extern "C"` POD entry descriptor (sdk major/minor, id/year/version, create/destroy pair) in addition to
`registerPlugin`; until then do not export C++ types across a DLL boundary (see `game/plugin_registry.h`).

## 5. Extension points

| To add... | Implement | Register with | Touches |
|---|---|---|---|
| A season | `IGameModule` (+ `IGameState`, `IField`) | `WPIEMU_REGISTER_GAME_PLUGIN`, one line in `plugins/builtin_plugins.cpp` | `plugins/` only |
| A mechanism template | `IMechanism` (+ `IPlant`) | `MechanismFactory::registerTemplate` (core generic set, or a plugin's `registerMechanisms`) | owner module only |
| A device family / vendor | `IDeviceBridge` (`SimDeviceBridge` rules table, or `ChannelFamilyBridge`) | `DeviceBridgeRegistry::registerBridge` inside the module's `registerXBridges(registry)` | `bridges/<x>` + `bridges/CMakeLists.txt` and one call in `bridges/builtin_bridges.cpp` (the extension does not change) |
| A drivetrain type | `IDrivetrain` | `DrivetrainRegistry::registerDrivetrain` | `core/physics` |
| A tire model | `ITireModel` | `TireModelRegistry::registerModel` | `core/physics` |
| A plant model | `IPlant` | `PlantFactory::registerPlant` | `core/mechanisms` |
| A transport / wire codec | `ITransport` / `IMessageCodec` | composition root | `ipc/` |
| Config fields / HUD / panels | `PropertySchema`, `HudWidgetDescriptor`, `EditorPanelDescriptor` | returned by `IGameModule::contribute*` | the plugin only |
| A game zone kind (intake volume, muzzle) | `ZoneKindInfo` | `IGameModule::contributeZoneKinds()` | the plugin only (kind prefix = plugin id) |

## 6. Checklists

### Add a year plugin
1. `copy plugins/_template plugins/game_<year>`; rename ids, namespaces (`wpiemu::plugins::game_<year>`) and CMake targets (`wpiemu_plugin_game_<year>`). The target links **only** `wpiemu_sdk`.
2. Implement `IGameModule` in `src/game_module.cpp`: `metadata()` (id `game_<year>`, season year, `minSdkMinor` if it needs a newer SDK minor), `registerMechanisms`, `createField(IPhysicsWorld&)`, `createGameState(SimContext&)`, `contributeConfigSchema()` (`schemaVersion` stays 1 until release, ADR 0013), optional `contributeEditorPanels/Hud`, `onTick`. End the file with `WPIEMU_REGISTER_GAME_PLUGIN(YourModule)` inside your namespace and declare `registerPlugin` in your `plugin.h`.
3. Add `add_subdirectory(game_<year>)` and the link line to `plugins/CMakeLists.txt`, and one `registerPlugin(registry)` call in `plugins/builtin_plugins.cpp`. Nothing else changes.
4. Tests under `tests/plugins/` using `sdk/testing/fake_sim_environment.h`; run `python tools/check_boundaries.py`.
5. Never include Qt or another plugin; keep per-session state inside `IGameState`/`IField`, never in the module.

### Add a device bridge (vendor/family)
1. New module under `bridges/<name>/` linking `wpiemu_sdk` (+ the vendor/HAL headers only there).
2. SimDevice-exposed vendor: write a `std::vector<SimDeviceRule>` table (name regex, id/child groups, bindings to canonical signals with `scale`/`offset`, `loopbackTo`) and construct `SimDeviceBridge`. CAN vendors whose SimDevice names carry no bus set `busIsStatic = false` (never fabricate a bus; feed L3 observations via `SimDeviceBridge::observeBus`). Per-index HAL family: derive `ChannelFamilyBridge`. Anything else: implement `IDeviceBridge`.
3. Expose `sdk::Status registerXBridges(DeviceBridgeRegistry&)` (or `void`); add the module to `bridges/CMakeLists.txt` (link into `wpiemu_bridges_builtin`) and one call in `bridges/builtin_bridges.cpp`. The extension is untouched. Capabilities in the `DeviceDescriptor` decide which slot types accept the device (`isCompatible`). Adding a vendor (REV, NavX) to the existing vendor module instead: recipe in [bridges-vendor.md](bridges-vendor.md).
4. Test with `FakeBackend`-style fakes (see `tests/sdk/test_devices.cpp`, `tests/bridges_vendor/fakes.h`) and recorded fixtures.

### Add a mechanism template
1. Implement `IMechanism` (constructor receives `MechanismBuildContext{config, sim}`); declare `MechanismTemplateInfo` (`type`, `slotSpecs`, `PropertySchema`, default primitives, optional attachable `links`; "root" is implicit). Slots are the **only** way to talk to devices (`ctx.devices()`), plants the only way to integrate motion, `ctx.coupling()` the only way to push on the chassis; publish moving links in `state().links`.
2. Register via `MechanismFactory::registerTemplate` (core: generic set; plugin: in `registerMechanisms` with keys prefixed by the plugin id).
3. The editor renders the inspector and slot panel from the metadata; no UI change.
4. Unit-test with `FakeSimEnvironment` + `FakeDeviceHub::autoBind`.

## 7. Directory ownership (how the work was split)

Historical split used while the modules were built in parallel; useful to see which tests and docs belong to which folder.

| Stream | Owns | Depends on (SDK headers) |
|---|---|---|
| IPC | `ipc/` | `ipc/messages.h`, `ipc/transport.h` |
| Extension | `halsim_ext/` | `devices/device_bridge.h`, `ipc/*` |
| Physics | `core/physics/` | `physics/*`, `dynamics/motor_model.h` |
| Mechanisms & runtime | `core/mechanisms/`, `core/runtime/` (hub, sim loop, config codec, battery) | `mechanisms/*`, `devices/device_hub.h`, `dynamics/*`, `config/*` |
| HAL bridges | `bridges/hal/` | `devices/*` |
| Vendor bridges | `bridges/vendor/` | `devices/*` |
| game_2026 | `plugins/game_2026/` | `game/*`, `mechanisms/*` |
| UI | `app/wizard`, `app/editor`, `app/runview`, `app/shell`, `app/common`, `app/host` | `telemetry/*`, `config/*`, `mechanisms/mechanism.h` (metadata) |
| Packaging/CI | `packaging/`, `.github/` | - |
