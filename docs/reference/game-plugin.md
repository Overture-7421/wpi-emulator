# Game plugin contract

Game plugins own season-specific fields, rules and metadata while core and the app remain
season-agnostic. For the implementation procedure see the
[game-plugin guide](../guides/game-plugins.md). Season behavior and limitations belong in the
owning plugin's README; for the built-in season see
[`seasons/2026/game/README.md`](../../seasons/2026/game/README.md).

## Module, field and state

A season implements [`IGameModule`](../../sdk/include/wpiemu/sdk/game/game_module.h) and depends
only on the SDK: not core, Qt, IPC, bridges, the HAL or another season. WPILib support is declared by the
[season pack](seasons.md) that names the plugin (`game.plugin`), never linked into it. The registry owns module instances;
keep mutable per-session data in the `IField` and `IGameState` they create. Descriptor queries,
including tag layouts and field frames, must be pure and thread-safe.

Field construction supplies owned static colliders and sensor volumes as well as the description
the app draws. Visuals are independent of colliders: a plugin can draw a collider differently or
add a visual with no collider. A field must destroy its physics bodies before its world is
destroyed.

The runtime resets game state before its first tick and on Reset. `IGameState::step()` sees the
current physics events and mechanism readings; `publish()` contributes entities, zone states and
HUD values to immutable snapshots. `onTick()` is an optional post-game hook. The
[runtime reference](runtime.md) defines tick order and session lifecycle.

`WPIEMU_REGISTER_GAME_PLUGIN` defines the namespaced `registerPlugin()` entry point. Built-ins are
registered through [`seasons/builtin_plugins.cpp`](../../seasons/builtin_plugins.cpp), whose plugin must
match the season manifest, and the app links the plugin aggregator, not individual seasons. A season
without a plugin (`"game": null`) runs core-only on the generic
[default field](physics.md#default-field). This is static registration, not a shared
library ABI. Selection honors `RobotConfig::gamePluginId` when present, otherwise
`project.wpilibYear`; an unavailable plugin leaves core features available.

## Configuration and contributed metadata

A plugin owns `RobotConfig::plugins[metadata().id]` and mechanism types prefixed with that ID plus
`.`. `registerMechanisms()` contributes season templates through `MechanismFactory`.
`contributeConfigSchema()` declares the plugin section. The config pipeline supplies defaults and
checks the declared schema version and properties; a mismatched section version is rejected.
Preserve other plugin sections. The [runtime reference](runtime.md) owns persistence and load order.

Declare each typed config member once with
[`ConfigBindings`](../../sdk/include/wpiemu/sdk/config/config_bindings.h): a `PropertySpec` bound
to a member yields the schema, decode and encode. Missing, mistyped or non-finite values use the
member's default; numeric values are clamped to the declared range.

`contributeZoneKinds()` declares `ZoneKindInfo`, and `contributeGameValues()` declares available
numeric sensor sources. Kind and value IDs must begin with the selected plugin ID plus `.`.
Foreign prefixes are validation errors; undeclared IDs with the plugin's prefix produce warnings.
`contributeEditorPanels()` and `contributeHud()` return Qt-free SDK descriptors, never widgets.

`fieldFrame()` supplies field dimensions and alliance symmetry for start-position copying.
`aprilTagLayout()` supplies field-frame tag poses and field dimensions; a nonempty layout requires
finite positive length and width. An empty layout gives simulated cameras no tags to observe. Tags
default to the FRC-wide `kFrcAprilTagSizeM` (6.5 in); set `AprilTag::sizeM` only for a different
tag size. Vision computation and robot-side publishing are not season code; see the
[bridge reference](bridges.md). Descriptor presentation belongs to the [app contract](app.md).

## Zones and game-fed sensors

A zone is game-owned spatial configuration attached to the chassis or a mechanism link; it is not
a mechanism or device. `ZoneShape::Volume` uses full `sizeM` extents. `ZoneShape::Muzzle` uses a
point and local +X direction and ignores size. The plugin resolves the chassis-frame pose from the
attachment-link pose multiplied by the zone's local pose. A plugin may choose a safe fallback for
an unresolved attachment; invalid attachments are also reported by config validation.

SDK `evaluateActivation()` evaluates All/Any conditions against published mechanism values,
converting recognized units to SI. An empty condition list is active. An unknown mechanism makes
that condition false; if the mechanism exists but the named value does not, evaluation falls back
to its `active` flag (inverted for Below).

Generic Distance and BeamBreak sensors belong to core, not the plugin. `SensorConfig::source`
names a declared game value, and `IGameState::gameValue()` returns its live number. Core evaluates
responses after the game step and post-game hook in the same tick, then writes the mapped device
slot. Missing or non-finite values yield idle readings and warnings. This is a value-to-reading
model, not ray casting or conveyor simulation; response details belong to the
[runtime sensor contract](runtime.md).

## Sound cues

A plugin declares its own `SoundCueInfo` records through `contributeSoundCues()` and requests a
cue ID with `SimContext::requestSound(id)`. Plugin cue IDs begin with the plugin ID plus `.`.
Each `asset` names `seasons/<year>/game/assets/sfx/<asset>.wav`; the plugin ships that file and the
plugins aggregation seam embeds it, so a season needs no app or audio dependency. Common cues use
`app/runview/assets/sfx/<asset>.wav` and the SDK's shared IDs.

Requests are one-shot events. They travel through a bounded
[`SoundCueQueue`](../../sdk/include/wpiemu/sdk/game/sound_cue_queue.h) that the host drains, not
through snapshots that the UI may skip. The app owns playback, volume and Driver Station practice
cues; use the shared IDs for lifecycle cues rather than declaring or requesting duplicates.

For a verified season procedure, including how to request cues only on rule boundaries, see
[request a sound cue](../guides/game-plugins.md#request-a-sound-cue). Cue ownership and timing for
the built-in season are documented in its [season README](../../seasons/2026/game/README.md#sound-cues).
