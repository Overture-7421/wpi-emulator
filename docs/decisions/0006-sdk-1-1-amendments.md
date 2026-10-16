# ADR 0006 - SDK 1.1 amendments (additive)

- Status: accepted
- Date: 2026-10-15
- Amends: [ADR 0005](0005-sdk-freeze.md) (SDK freeze). `kSdkVersionMinor` 0 -> 1, `proto::kProtocolVersion` 1 -> 2.
- Scope: `sdk/include/wpiemu/sdk/` and `tests/sdk/` only. Everything below follows ADR 0005 section 4
  ("additive changes"): new headers, new fields appended with defaults, new defaulted virtuals, new
  message type appended to the variant. No frozen name, signature, default, unit or order changed.

## Context
An architecture review of the frozen SDK found gaps that would force streams to invent private
interfaces: the lock-step IPC cannot tell a late frame from a current one after a Reset or reconnect and
has no defined behaviour when the robot stalls; mechanisms cannot push on or load the chassis; the device
model conflates what the robot declares with what the simulator can distinguish (CTRE keys by type+id
only, and the bus is not observable from SimDevice names); plugins cannot migrate anything outside their
own config section; mechanism attachment is a bare string with no links or validation contract.

## Decision

### 1. IPC lock-step robustness (protocol v2) - `ipc/messages.h`, `ipc/lockstep.h`
* Window = **one outstanding tick** (`kLockstepWindow`).
* `sessionId` + `runEpoch` (monotonic; bumped by the emulator on Reset and reconnect) on `SensorFrame`,
  `ActuatorFrame`, `DsState`, `SimControl`. Receivers drop stale epochs / foreign sessions
  (`classifyFrame`); `0` = unspecified is always accepted (v1 peers).
* `ActuatorFrame`: `ackTick`, `runEpoch`, `sessionId`, `TickResult {Completed, RobotStalled, Stopped, ProtocolError}`.
* `SimControl::commandId` + new `SimControlAck` (`MessageType` 11, appended to the variant). Reset/Stop
  are acknowledged commands; retransmissions are re-acked, not re-applied; Stop ignores the epoch.
* Watchdog negotiated in `Hello::proposedWatchdogMs` / `HelloAck::watchdogTimeoutMs`
  (`negotiateWatchdogTimeoutMs`) with `HelloAck::watchdogPolicy`. **On timeout or disconnect the extension
  itself resumes HAL timing (or terminates); it never depends on the emulator to unfreeze the robot.**
* `LockstepWindow` / `LockstepFollower` are the reference state machines for both ends.
* `DeviceAnnounce::observedBus` (see 3). `kMinCompatibleProtocolVersion` stays 1: every new field has a
  default meaning "v1 behaviour".

### 2. Mechanism <-> physics coupling - `mechanisms/physics_coupling.h`
`IMechanismPhysicsCoupling` reached through `SimContext::coupling()` (never null, `NullMechanismPhysicsCoupling`
fallback; `SimContextServices::coupling` appended): chassis wrench at a chassis-local point, anchored
constraints (prismatic / distance / contact anchor to a field point) with reaction-load reporting (invalid
handle when unsupported), payload mass/COM/inertia composed by the runtime (`composeMassProperties`).
Ordering: mechanisms submit before the physics step and read the **previous** step's loads. Fake:
`testing/fake_mechanism_physics_coupling.h` (wired into `FakeSimEnvironment`).

### 3. Device identity - `devices/device_descriptor.h`, `device_identity.h`, `discovered_device.h`
`DeviceKey` stays the robot-declared address. New `PhysicalDeviceId` (no bus) is the simulator identity;
hubs resolve declared addresses by physical id. `DiscoveredDevice` gains `observedBus`, `aliasState`
(`None | Aliased | Conflict`), `aliases`. Bus is a label fed only by an L3 observation (`observeBus` hooks on
`IBridgeHost`, `IDeviceHubIngress`, `SimDeviceBridge`); `SimDeviceRule::busIsStatic = false` stops static rules
from fabricating one. `Issue::code` + `config/validation_codes.h`; the config validator MUST reject
independent mappings of conflicting aliases (`device_alias_conflict`, `validateDeviceAliases`). Safe runtime
behaviour: a conflicted device is looped back and its slots are neutral, with a hub warning.

### 4. Config migration hook - `game/game_module.h`, `config/plugin_config.h`
`IGameModule::migrateRobotConfig(RobotConfig&, int persistedPluginVersion) const` (default no-op). Pipeline:
core migration -> selected plugin migration (section migrators, then this hook) -> defaults -> validation ->
construction. A plugin owns only its section and mechanisms with `ownsMechanismType(id, type)`.
`GameMetadata::minSdkMinor` lets a plugin require 1.1.

### 5. Mechanism attachment - `config/robot_config.h`, `mechanisms/mechanism.h`, `mechanisms/attachment.h`
`MechanismConfig::attachment` (`{parentMechanismId, parentLink = "root"}`) augments the legacy `parent`
(kept; structured form wins; `attachTo()` keeps both in sync). Every mechanism and the chassis expose the
stable link `root`; templates add `LinkSpec`s; `IMechanism::linkPose` and
`IMechanismDirectory::linkPoseInChassis` give resolved transforms; `validateMechanismAttachments` defines the
validation contract (missing parent / link, cycles, reserved id). `MechanismFactory::registerTemplate` rejects
templates that redeclare `root` or duplicate links.

### 6. Documentation
`docs/ARCHITECTURE.md` section 4.x documents all of the above. **Dynamic plugin ABI caveat:** v1 supports
static built-in plugins only; shared-library plugins would need a versioned `extern "C"` POD entry
descriptor in addition to `registerPlugin` (noted in `game/plugin_registry.h`).

## Consequences
* Existing code compiles unchanged: new struct fields are appended with defaults, new virtuals are
  non-pure, the legacy `MechanismConfig::parent` and `DeviceDescriptor::bus` remain.
* Behavioural notes for implementers:
  * `IDeviceDirectory::find(const DeviceKey&)` now matches on physical identity (bus ignored).
  * `SimContext::coupling()` is always valid; check `capabilities()` before relying on a feature.
  * Aggregate initialisation with fewer initialisers still compiles, but `-Wmissing-field-initializers`
    users should prefer assigning fields (`Issue` gained `code`, `SimControl` gained ids).
  * Core `IConfigValidator` / codecs / hub / runtime must adopt the contracts above; the SDK helpers
    (`validateDeviceAliases`, `validateMechanismAttachments`, `composeMassProperties`, `LockstepWindow`,
    `LockstepFollower`, `resolveLinkPoseInChassis`) exist so every stream implements the same rules.
* Still open (unchanged from ADR 0005): runtime mechanism <-> game-state event bus, per-wheel suspension /
  non-flat ground, multi-robot `SimContext`, dynamic plugins, binary wire codec.
