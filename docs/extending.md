# Extending the emulator

Read this when deciding where a new game, device adapter or configurable simulation feature belongs.
Prerequisites: [Architecture](ARCHITECTURE.md), [a working build](BUILDING.md), and
[contribution conventions](CONTRIBUTING.md).

Choose the smallest existing SDK seam. This page is the decision map; the linked guides own
step-by-step implementation and tested excerpts. References own units, lifecycle and failure contracts.
Do not implement a season/vendor feature as a special case in core or UI.

## Add a season plugin

Use an `IGameModule` when the change defines a season's field, rules, entities or metadata.
Copy the [buildable template](../plugins/_template/README.md), keep the module stateless, and put
session state in the returned `IField` and `IGameState`. Plugin config and contributed keys stay
within the plugin's identity.

Register through `plugins/CMakeLists.txt` and `plugins/builtin_plugins.cpp`, not a direct app-to-season
dependency. Bring up year selection, field cleanup, reset and one published value before adding
full rule fidelity. Plugins are statically built with the host toolchain, not independently loaded
binary extensions.

Walkthrough: [bring up a game year](guides/game-years.md).
Contract: [game plugins](reference/game-plugin.md).

## Add a zone kind

Use a plugin-contributed `ZoneKindInfo` for a robot-attached game volume or launch muzzle.
Metadata describes properties, defaults and editor gizmos; game state implements the action.
Resolve attachment links and evaluate SDK activation conditions instead of duplicating mechanism
physics or adding zone-specific widgets.

Walkthrough: [game interactions](guides/game-interactions.md#describe-a-zone-then-implement-its-action).
Contract: [game plugins](reference/game-plugin.md).

## Add a game-fed sensor

Publish a plugin-owned numeric `GameValueInfo` and implement `IGameState::gameValue`.
Configure an existing Distance or BeamBreak sensor response and map its `sensor` slot.
Core supplies device feedback; the plugin does not write bridge signals or parse HUD strings.
A new source is much smaller than a genuinely new sensor kind.

Walkthrough: [game values and sensor responses](guides/game-interactions.md#publish-a-numeric-game-value).
Contract: [runtime sensors](reference/runtime.md).

## Add a sound cue

Use a plugin-contributed `SoundCueInfo` for a sound the game plays (a match horn): the plugin ships the audio file,
raises the cue through a `SoundCueLog` and publishes it in the snapshot. The run view plays it; the plugin includes
no Qt and the app names no season.

Walkthrough: [raise a sound cue](guides/game-interactions.md#raise-a-sound-cue).
Contract: [game plugins](reference/game-plugin.md#sound-cues).

## Add a device bridge

Choose a SimDevice rule for a named backend, `ChannelFamilyBridge` for indexed HAL families, or
`IDeviceBridge` for a genuinely different API. Bridge commands/feedback through canonical signals,
real capabilities and verified units. Preserve retirement, reset, unmapped loopback and physical
identity; an unobserved CAN bus must not be invented.

New modules are added, linked and registered through the bridge aggregator and boundary ownership
tables. The extension never gains a direct dependency on an individual bridge module.

Walkthrough: [device adapters](guides/devices.md).
Contract: [bridges](reference/bridges.md).

## Add a vendor rule set

First inspect the wrapper's actual SimDevices, names, directions and units.
Reuse the existing vendor bridge/decorators and register the rule set in its shared setup.
CTRE-native hooks and integrated-input guards are policies with verified semantics, not generic
defaults for every vendor. A controller adapter is distinct from motor preset data.

Walkthrough: [vendor rule registration](guides/devices.md#register-a-vendor-rule-set).
Contract: [bridges](reference/bridges.md).

## Add a vision vendor

Implement `IVisionBridge` to encode/publish core-produced camera results; keep encoding separate from
the NT backend. Add sources to the vision module, register its factory and retain the aggregator's
real/inert backend selection. Do not move capture scheduling or visibility into the robot process.

Walkthrough: [vision adapters](guides/vision.md).
Contract: [vision publication](reference/bridges.md).

## Add a mechanism template

Configure the generic `mechanism` template first: rotary, arm and linear properties cover many
rollers, pivots and stages. Use a new `IPlant` for different local dynamics and a new `IMechanism`
only for different slots, axes or published structure.

Register generic templates through `registerGenericMechanisms`, plugin-specific ones through
`IGameModule::registerMechanisms`. Metadata supplies forms and slots; moving links, battery draw,
reset and chassis coupling are implementation responsibilities.

Walkthrough: [mechanisms](guides/mechanisms.md).
Contract: [mechanisms](reference/mechanisms.md).

## Add an editor property

Extend the owner's `PropertySchema` with SDK builders; use `ConfigBindings` for typed configuration.
Describe units, defaults, bounds, grouping and visibility rather than adding a custom widget.
Structural SDK fields also need codec, validation and editor bindings updated together.
Route user edits through undoable editor operations, never direct runtime/persistence mutation.

Walkthrough: [editor properties](guides/editor-properties.md).
Contract: [app](reference/app.md).

## Other SDK strategies

| Need | Smallest seam | Walkthrough |
|---|---|---|
| Physical motor constants | Shared SDK `motors::presets()` table; core catalog consumes it | [Motors](guides/motors.md) |
| Shaft torque/current law | `IMotorModel` at an explicit composition point, not a global registry | [Motor injection](guides/motors.md#use-the-real-behavioral-injection-seam) |
| Wheel layout/kinematics | `IDrivetrain` and `DrivetrainRegistry` with shared slot metadata | [Drivetrains](guides/simulation-strategies.md#drivetrains) |
| Contact force law | Stateless `ITireModel` and `TireModelRegistry` | [Tires](guides/simulation-strategies.md#tires) |
| Local load dynamics | `IPlant` and `PlantFactory`, supplied to its consumer | [Plants](guides/simulation-strategies.md#plants) |
| World implementation | `IPhysicsWorld` injected during session composition | [Physics backends](guides/simulation-strategies.md#physics-backends) |
| Local transport or message fields | `ITransport` / `IMessageCodec` and both peer composition roots | [Transport/messages](guides/simulation-strategies.md#transport-and-message-changes) |

Preserve the [physics](reference/physics.md) and [extension/IPC](reference/extension-ipc.md) contracts.
The codec's optional-tail handling is implementation behavior, not a supported cross-version
interoperability promise. Update the current pre-release schema/SDK/protocol and callers together.

## Validate the extension

Exercise construction, normal use, a failure case, reset and teardown through the public seam.
Keep registration failures visible and metadata defaults valid. Each guide links focused tests;
run the applicable build, boundary and documentation checks from [Building](BUILDING.md).
A passing fake, skipped robot integration or offscreen UI suite does not prove real backend or
rendering behavior.

Update the owning reference for changed contracts and the [user guide](USER_GUIDE.md) for observable
behavior. Put task recipes in guides and rationale through the [decision index](decisions/README.md).
For a failure rather than a new feature, start with [maintenance diagnostics](guides/maintenance.md).

Next: [pick a task from the documentation home](README.md).
