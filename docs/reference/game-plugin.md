# Game plugin reference

**Read this when:** implementing season rules or connecting a robot mechanism to game behavior.
**Prerequisites:** [Architecture](../ARCHITECTURE.md), [runtime](runtime.md), and [mechanisms](mechanisms.md).
The [extension recipes](../extending.md) own all add-a-season, zone-kind, and game-value procedures.

## Module contract and registration

A season implements [`IGameModule`](../../sdk/include/wpiemu/sdk/game/game_module.h).
Its implementation depends only on the SDK: no core, Qt, IPC, bridges, or other seasons.
Keep session state in the returned `IField` / `IGameState`, not the registry-owned module.
Descriptor queries, including tag layouts and field frames, must be pure and thread-safe.
Registration and construction run on the caller's setup thread, including UI field previews.
Live game-state ticking and publication belong to the simulation thread.

`createField()` supplies owned static colliders and sensor volumes before `createGameState()`.
The runtime resets the game state before the first tick and on Reset. `IGameState::step()` sees
fresh physics events and mechanism readings; `publish()` contributes entities, zone state, and
HUD values to snapshots. `onTick()` is an optional post-game hook; see [runtime](runtime.md) for lifecycle details.

`GamePluginRegistry` owns module instances. `WPIEMU_REGISTER_GAME_PLUGIN` defines the plugin's
namespaced `registerPlugin()` entry point; the
[built-in aggregator](../../plugins/builtin_plugins.cpp) exposes `registerBuiltinGamePlugins()`.
The app links `wpiemu_plugins_builtin`, not individual season targets. This is static built-in
registration, not a shared-library plugin ABI.

Selection honors `RobotConfig::gamePluginId` when present, otherwise `project.wpilibYear`.
An unavailable plugin leaves core features available; do not add year checks to core or UI.

## Ownership and descriptors

The plugin owns `RobotConfig::plugins[metadata().id]` and season mechanism types prefixed with
that id plus `.`. `registerMechanisms()` contributes those templates through `MechanismFactory`.

`contributeConfigSchema()` describes the plugin section. The config pipeline supplies defaults
and checks its declared schema version and properties; a mismatched section version is rejected.
Keep other plugins' sections intact. Persistence and loading rules are in [runtime](runtime.md).

`contributeZoneKinds()` declares owned `ZoneKindInfo` metadata, and `contributeGameValues()`
declares available numeric sensor sources. Kind and value ids start with the selected plugin id
plus `.`. A foreign prefix is a validation error; an undeclared id with the owned prefix produces
a warning. Do not silently reinterpret another season's configuration.

`contributeEditorPanels()` and `contributeHud()` return SDK descriptors, never Qt widgets.
`fieldFrame()` supplies field dimensions and alliance symmetry for start-position copying.
`aprilTagLayout()` supplies field-frame tags and the field size (`fieldLengthM`/`fieldWidthM`, finite and positive
whenever the layout has tags); an empty layout gives simulated cameras no tags to observe. The size travels in
`VisionSetup`; the bridges never assume a season's field. Vision computation and robot-side publishing are not
season code; see [bridges](bridges.md). The [app contract](app.md) explains descriptor presentation.

## Robot zones and game-fed sensors

A zone is a game-owned place, not a mechanism or a device. `ZoneShape::Volume` uses full
`sizeM` extents; `ZoneShape::Muzzle` uses a point and local +X direction and ignores size.
The plugin resolves its chassis-frame pose as attachment-link pose multiplied by `local`,
using `SimContext::mechanisms()`. Unresolved attachments fall back to the chassis in the
2026 runtime, but invalid attachments still produce config errors.

[`evaluateActivation()`](../../sdk/include/wpiemu/sdk/game/zone_activation.h) evaluates All/Any
conditions against published mechanism values, converting recognized units to SI. An empty
condition list holds. An unknown mechanism makes that condition false; an existing mechanism
without the named value falls back to its `active` flag (inverted for Below).

Generic Distance and Beam-break sensors belong to core, not the plugin. They have no pose:
`SensorConfig::source` names a declared game value, and `IGameState::gameValue()` returns its
live number. Core evaluates responses after the game step and module hook in the same tick,
then writes the mapped device slot. Missing or non-finite values yield idle readings and warnings.
This is a value-to-reading model, not ray casting or conveyor simulation.

For REBUILT the source contract is:

| Game value | Unit | Meaning |
|---|---|---|
| `game_2026.fuel.count` | count | Fuel held by the robot |
| `game_2026.fuel.capacity` | count | Configured inventory capacity |
| `game_2026.fuel.fraction` | ratio | Count / capacity; zero when capacity is zero |

## REBUILT robot behavior

The [2026 module](../../plugins/game_2026/src/game_module.cpp) registers no mechanism templates
or custom editor panels. Generic mechanisms supply motion; `game_2026.pickup` and
`game_2026.shooter` zones connect that motion to the fuel inventory.

* A pickup absorbs fuel whose center is inside its oriented box while activation holds and
  capacity remains, nearest first. `intakeRateHz` limits collection; zero means unlimited.
* A shooter requires activation, a nonempty inventory, and sufficient positive exit speed.
  Speed is absolute flywheel angular speed × radius × efficiency, or a positive
  `fixedExitSpeedMps`. Hood and turret readings modify aim.
* Attaching the muzzle to a turret link already rotates it: an additional turret reference
  applies another rotation. Missing nonempty aim/speed references warn once per zone and
  mechanism and suppress firing; fixed speed bypasses the otherwise unused flywheel reference.
* `fireRateHz` is total fuel per second, not volleys per second. `parallelBalls` spreads a volley
  sideways; a partially filled inventory can fire a partial volley. Optional seeded dispersion
  and chassis point velocity affect launches.
* `ZoneState::active` reports activation alone, not firing readiness or inventory availability.
  Do not infer a shot from the highlighted muzzle.

Defaults, bounds, and property descriptions are authoritative in
[the zone schemas](../../plugins/game_2026/src/fuel_zones.cpp) and
[plugin config schema](../../plugins/game_2026/src/config.cpp), rather than duplicated tables.

## REBUILT field and rules

The field includes alliance walls, guardrails, hollow hubs, bumps, trenches, tower rungs, depots,
and outpost openings/corrals. Blue geometry is rotated about the field center for red.
Fuel is simulated as rigid spheres; reset stages field fuel and restores the robot inventory
and chute stocks. Body limits bound fuel creation; out-of-play fuel is removed.

Timed play uses `DsState::matchTimeS` as seconds **remaining** in the active period:
AUTO lasts 20 seconds; TELEOP has a 10-second transition, four 25-second shifts, then a
30-second end game. Without a supplied match time, enabled play is free play with both hubs
active and no timed-match HUD. Test mode also has both hubs active.

Both hubs are active in AUTO, transition, and end game. During shifts they alternate.
The first-inactive alliance is latched at TELEOP start: explicit `firstInactiveHub` override,
then FMS `R`/`B`, then AUTO fuel totals, with a seeded tie break. Disabling `enforceHubShifts`
keeps both hubs active while play is running. The latched game message is forwarded to robot
code by the runtime and takes precedence over manual Driver Station text.

Fuel entering a scoring sensor earns one point while scoring is open, including the
three-second grace after deactivation. Fuel outside active/grace windows is consumed without points.
With recycling enabled, consumed hub fuel waits for the processing delay before release into
the neutral zone; a full body pool or refused spawn retains pending releases for retry.
Outpost corral fuel becomes chute stock; the optional timed human player releases that stock.
Snapshots expose fuel entities, zone activation, inventory/counters, and scores/hub status.
The blue-left/red-right scoreboard keeps a stopped timed result inspectable as `FINAL` until
reset, another timed match, or enabled free play.

## Accuracy, sources, and pitfalls

Use [field dimensions](../../plugins/game_2026/include/wpiemu/plugins/game_2026/field_dimensions.h)
for numbers and provenance tags: **GM** is the
[FIRST game manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf),
**AT** is WPILib's welded AprilTag dataset, and **MS** is
[MapleSim's community geometry](https://github.com/Shenzhen-Robotics-Alliance/maple-sim).
Derived and approximate dimensions are marked in source; this is not an official CAD model.
[Field construction](../../plugins/game_2026/src/field_layout.cpp),
[embedded tags](../../plugins/game_2026/src/april_tags.cpp),
[match timing](../../plugins/game_2026/include/wpiemu/plugins/game_2026/match_clock.h), and
[hub rules](../../plugins/game_2026/src/hub_rules.cpp) are the implementation authorities.

The hub omits its funnel, internal ramps, and back net; tower supports/climb scoring, ranking
points, penalties, trench motion, and outpost doors/throwing are not simulated.
Pickup has no jamming or mouth-contact model; shooting has no hopper, indexer, or flywheel
compression model. Inventory belongs to one robot, not a simulated alliance of robots.

Keep fuel collision masks compatible with field sensors in both directions, including the
temporary shot category. `shotGhostS` suppresses robot and other freshly shot fuel collisions,
not field, settled-fuel, or scoring interactions. Sensors need depth to avoid fast fuel skipping
overlap detection; see [physics](physics.md). For attached-zone issues, check link resolution
and mechanism ordering before altering geometry.

## Testing

[Season tests](../../tests/game_2026) cover rules, zones, reset, snapshots, and core-mechanism
integration; real-physics tests check geometry and fuel behavior that fakes cannot establish.
The [new-season integration example](../../tests/plugins/test_new_year_plugin.cpp) exercises
registration, config defaults, and a real core session without season changes in core.
Test commands and conventions: [Building](../BUILDING.md), [Contributing](../CONTRIBUTING.md).

**Next:** [Extend the simulator](../extending.md).
