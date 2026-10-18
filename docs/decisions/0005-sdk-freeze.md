# ADR 0005 - SDK freeze (`wpiemu_sdk` 1.0)

- Status: accepted; the freeze and additive-only rules are superseded until release by [ADR 0013](0013-pre-release-single-sdk-version.md)
- Date: 2026-10-15
- Scope: the public, header-only SDK in `sdk/include/wpiemu/sdk/`, the plugin aggregation seam,
  and the process for changing either.

## Context
About nine streams (IPC, extension, physics, mechanisms/runtime, HAL bridges, vendor bridges,
`game_2026`, UI, packaging) implement against shared interfaces in parallel with disjoint
directories. The interfaces must therefore be stable before they start, and cheap to extend
without breaking anyone.

## Decision

### 1. Frozen API (SDK 1.0)
Everything below is frozen at `kSdkVersionMajor = 1`, `kSdkVersionMinor = 0` (`sdk/version.h`).
Names and semantics are as documented in the headers (the doc comments are normative: threading,
units, lifetime, error handling).

| Area | Frozen items |
|---|---|
| Basics (`core/`) | `Vec3`, `Quat`, `Pose3` (Z-up NWU, float), units/`SimTimeUs`, `StrongId` + `BodyId`/`ShapeId`/`EntityId`/`DeviceId`, `Result<T>`/`Status`/`Error`/`ErrorCode`, `Value`/`ValueObject`, `ILogger`, `ISimClock`, `Rng`, `Registry<>`, `PrimitiveSpec`, `Severity`/`Issue`, `SimContext` + `SimContextServices`, `WPIEMU_INTERFACE` |
| Physics | `IPhysicsWorld` + `physics_types.h` (ADR 0003 shape, layers, events, `PhysicsConfig`), `ITireModel` (`TireParams`/`TireInput`/`WheelDrive`/`TireOutput`), `TireModelRegistry`, `ChassisSpec`/`WheelSpec`/`DrivetrainSpec`, `IDrivetrain` + `WheelCommand`/`WheelState`/`DrivetrainBuildArgs`, `DrivetrainRegistry` |
| Dynamics | `MotorSpec`, `IMotorModel`, `DcMotorModel`, motor catalog, `PlantSpec`/`PlantState`/`PlantInput`, `IPlant`, `PlantFactory` |
| Devices | `DeviceDescriptor`, `DeviceKey`, `Capability`/`CapabilitySet`, `SignalSpec`/`SignalSample`, canonical signal names (`signals.h`), `SlotSpec`/`SlotType`/`SlotAddress`/`SlotMapping`, typed frames (`MotorOutput`, `MotorFeedback`, `EncoderFeedback`, `ImuFeedback`), `IActuatorReader`/`ISensorWriter`/`IDeviceDirectory`/`IDeviceHub`/`IDeviceHubIngress`, `DiscoveredDevice`/`HubWarning`, `IDeviceBridge`/`IBridgeHost`/`DeviceBridgeRegistry`, `SimDeviceBridge` (+ `ISimDeviceBackend`, `SimDeviceRule`, `SimValueBinding`), `ChannelFamilyBridge` |
| Mechanisms | `IMechanism`, `IMechanismDirectory`, `MechanismTemplateInfo`, `MechanismBuildContext`, `MechanismFactory`, `MechanismState` |
| Game | `IGameModule`, `GameMetadata`, `IGameState`, `IField`/`FieldDescription`, `HudWidgetDescriptor`, `EditorPanelDescriptor`, `GamePluginRegistry`, `WPIEMU_REGISTER_GAME_PLUGIN`, `registerBuiltinGamePlugins` |
| Config | `RobotConfig` (+ `ProjectInfo`, `BatteryConfig`, `SimSettings`, `DrivetrainConfig`, `MechanismConfig`, `PluginSection`), `kRobotConfigSchemaVersion`, `PropertySchema`/`PropertySpec`/`PropertyValue`, `IConfigMigrator`, `migrateToVersion`, `PluginConfigSchema`, `IValueTextCodec`, `IRobotConfigCodec`, `IRobotConfigStore`, `IConfigValidator`, `value_codec.h` encodings |
| IPC | `proto::` messages (`Hello`, `HelloAck`, `DeviceAnnounce`, `LoopbackSet`, `SensorFrame`, `ActuatorFrame`, `DsState`, `SimControl`, `Log`, `Goodbye`), `MessageType` ids, `kProtocolVersion` (= 1), `IMessageCodec`, `ITransport`, `ITransportServer`, `ITransportClient`, `MessageChannel` |
| Telemetry | `ISimSnapshot`, `SimSnapshotData`/`SimSnapshot`, `SnapshotBuilder`, `ISnapshotSink`/`ISnapshotSource`/`LatestSnapshotSlot`, `ISimController` |
| Testing | `sdk/testing/*` fakes (not part of the compatibility promise beyond "keep compiling"; they may gain features) |

### 2. Cross-cutting rules (frozen)
* Dependency rule and targets of ADR 0004, plus the **aggregation seam** below.
* Error handling: `Result<T>`/`Status`; no exceptions across SDK interfaces.
* No singletons/globals; ownership via `unique_ptr`; collaborators by constructor injection / `SimContext`.
* Identifiers in SDK headers must not collide with Qt keyword macros (`signals`, `slots`, `emit`,
  `foreach`); enforced by `tools/check_boundaries.py`.
* Wire-protocol rules: additive-only, append fields at the end of structs, never reuse a
  `MessageType`, `typeOf()` relies on the `proto::Message` variant order.

### 3. Plugin aggregation seam (amends ADR 0004)
ADR 0004 announced "a plugins/-owned registration target". It is `wpiemu_plugins_builtin`
(`plugins/CMakeLists.txt`, `plugins/builtin_plugins.cpp`), which links every built-in plugin and
defines `wpiemu::sdk::registerBuiltinGamePlugins()`, declared in `sdk/game/builtin_plugins.h`.
`app` links this one target; nothing else may include or link it. `tools/check_boundaries.py` models
it as owner `plugins` (may see sdk + every plugin; `app` may link it; includes of it are forbidden
everywhere else). `plugins/_template` is not part of the aggregate. Adding a season = new plugin
folder + a link line and a `registerPlugin` call in the aggregator.

### 4. Process to change the SDK
* **Additive changes** (allowed by a normal PR reviewed by the SDK owner; bump `kSdkVersionMinor`):
  new header; new struct field **at the end** with a default value; new virtual in an interface
  **with a default implementation** (non-pure); new enum value **appended** (and consumers must have
  a `default` path); new `Registry` or free function; new canonical signal name; new `MessageType`
  with a new struct appended to the end of the `proto::Message` variant (bump `kProtocolVersion` and
  keep `kMinCompatibleProtocolVersion` unless older peers cannot cope).
* **Not additive** (needs an ADR superseding/amending this one, bump `kSdkVersionMajor`, and a
  migration note): removing or renaming anything; changing a signature, a default, units, frame
  conventions or thread-ownership rules; reordering struct fields or enum values; adding a pure
  virtual to a frozen interface; changing `kRobotConfigSchemaVersion` semantics (needs a core
  `IConfigMigrator` as well).
* Every SDK change must keep `wpiemu_tests` green: the generated header self-containment check
  (`tests/sdk/header_check.cmake`, one TU per header, all linked together for ODR), the SDK contract
  tests in `tests/sdk/`, and `boundaries.check`.
* A stream that needs something the SDK lacks opens an additive request (issue/PR touching only
  `sdk/` + `tests/sdk/`) instead of working around it with a private interface.

## Consequences
* Streams can work in disjoint directories against fakes (`sdk/testing`) without waiting for each other.
* The SDK is deliberately fat in header-only helpers (`SimDeviceBridge`, `ChannelFamilyBridge`,
  `migrateToVersion`, fakes) so bridges and plugins stay small and uniform; this code is part of the
  contract and must keep working with all supported compilers.
* Known open questions (decide additively when needed, see handoff notes): runtime mechanism <->
  game-state event bus; per-wheel suspension/non-flat-ground inputs; multi-robot `SimContext`;
  dynamic (shared-library) plugins; a binary wire codec choice.
