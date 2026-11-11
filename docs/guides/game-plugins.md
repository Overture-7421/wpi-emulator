# Build a game plugin

Use this guide to add a season, connect robot mechanisms to game behavior, or expose game state to
robot sensors. Start with the [plugin contract](../reference/game-plugin.md); the
[2026 plugin notes](../../seasons/2026/game/README.md) are season-specific and are not a template
for other games. For an end-to-end worked example aimed at a mentor new to this codebase, read the
[Implementing a Game Season](../book/README.md).

## Mental model

A game plugin is a season adapter, not a place to put general simulation behavior. Its stateless
module describes the season and creates two per-session owners: a field that owns field physics,
and game state that owns rules, entities and timers. Core advances generic mechanisms and physics;
the game state consumes their results and publishes game state. The app renders metadata and
snapshots without knowing the season.

For a robot-attached interaction, think of a **zone** as a described frame and shape, and game state
as the rule that gives it behavior. A **game value** is a numeric input to a generic sensor, not a
formatted HUD string. These distinctions keep season rules out of core, UI and device bridges.

## Bring up a season

1. Create the [season pack](../reference/seasons.md) `seasons/<year>/` (copy `seasons/_template`) and set its
   `"game": {"plugin": "game_<year>"}`. Copy [`seasons/_template/game`](../../seasons/_template/game) to `seasons/<year>/game` and follow its
   [identity checklist](../../seasons/_template/game/README.md). Keep the CMake target, namespace,
   include tree, plugin ID and game year consistent; depend on SDK only.
2. Implement a stateless `IGameModule`, an `IField` that destroys its bodies before its world, and
   an `IGameState` that resets all per-session state. Use simulation time and context RNG. Give
   rigid field structure the shared [contact materials](../reference/physics.md#contact-materials)
   so robots slide along it instead of sticking.
3. Return plugin-prefixed metadata and register the module in `seasons/builtin_plugins.cpp` (CMake links it
   from the manifest). The app uses the aggregator, not a direct season dependency.
   [`check_seasons.py`](../../tools/check_seasons.py) and the boundary checker verify the wiring.
4. Start with a selectable year, a resettable state and one observable HUD value. Then add config
   with `ConfigBindings`, field fidelity, rules and entities.
5. Add focused tests for registration, selection, defaults, field cleanup, repeated reset, rule
   boundaries and deterministic behavior. Use a real physics test where contact geometry matters.

For season-accurate Run-view Driver POVs, optionally populate
`FieldDescription::driverStations` with the alliance-wall eye positions. The SDK supplies a generic
evenly spaced fallback when the field does not provide a spot.

The small example in
[`test_new_year_plugin.cpp`](../../tests/plugins/test_new_year_plugin.cpp) is a verified first
integration: it registers a 2998 module beside the built-in plugin, loads it through the real config
pipeline, constructs its field and state, and advances a counter. The test expects the
`game_2998.steps` HUD value and mechanism reading to be `5` after five ticks. This proves plugin
wiring and session integration, not season-rule fidelity.

```powershell
ctest --preset windows-ninja-release -R "new season|new plugin|template plugin"
```

Expected result: the new-season registration/config/session tests and template exclusion test pass.
If a season is absent, check plugin metadata and both aggregator wiring points before adding an
app-side year special case.

## Add a zone interaction

Return a plugin-prefixed `ZoneKindInfo` from `IGameModule::contributeZoneKinds()`. Metadata defines
the editor description, properties, default local pose, color and shape. It does not implement
gameplay.

1. In game state, read the matching `RobotConfig::zones` entries.
2. Resolve the attachment with `IMechanismDirectory::linkPoseInChassis` and compose that pose with
   the zone's local pose. An attachment link is a spatial frame; a motion link prescribes an axis.
3. Evaluate the configured activation with SDK `evaluateActivation`; do not invent a second
   condition language.
4. Implement collection, launch, capacity and rate limits in game state. Handle missing
   attachments and readings safely.
5. Test activation first, then test game behavior at geometric and rate-limit boundaries.

The SDK case `zone activation: All/Any combination, empty list and missing mechanisms` in
[`test_zone.cpp`](../../tests/sdk/test_zone.cpp) verifies that an empty condition list is active,
including with no mechanism directory. Its fixture also exercises `All`, `Any`, and missing
mechanisms. For a complete season implementation, follow
[`fuel_zones.cpp`](../../seasons/2026/game/src/fuel_zones.cpp) and
[`game_state.cpp`](../../seasons/2026/game/src/game_state.cpp): metadata defines pickup/muzzle
zones, while game state resolves poses, activation, and inventory behavior.

```powershell
ctest --preset windows-ninja-release -R "^(zone |zones:|pipeline: zone|validator: zone)"
```

Expected result: activation, zone config/defaults, and validation cases pass. A metadata-only test
does not prove real overlap or contact behavior; use the real physics backend for those claims.

## Support several robots

In a practice match the host's own robot (participant 0) and up to five remote robots share one simulation.
A season that declares zone kinds must make its rules work for all of them, or remote robots are only bodies
on the field. The seam is Qt-free SDK: [`robot_view.h`](../../sdk/include/wpiemu/sdk/game/robot_view.h)
(`IRobotDirectory`, `RobotView`, read through `SimContext::robots()`) and the `onRobotAdded` /
`onRobotRemoved` / `publishRobot` hooks of [`IGameState`](../../sdk/include/wpiemu/sdk/game/game_state.h),
which run on the sim thread between ticks. The runtime and the [runtime reference](../reference/runtime.md)
own the roster; the [game-plugin contract](../reference/game-plugin.md#several-robots) lists the rules. The
refactor of the 2026 season showed which parts are mechanical and which need care, in this order:

1. **Per-robot state.** Put everything one robot owns (fuel meter, shot counter, held pieces, zone runtimes) in
   a struct and keep it in a [`PerRobotState`](../../sdk/include/wpiemu/sdk/game/per_robot_state.h):
   `add` in `onRobotAdded` (skip `RobotView::isHost`), `remove` in `onRobotRemoved`, `forEach` in `step()`
   and `publishRobot`. The host state is always present and lives outside the map, so a session without a
   roster plays exactly as before. `reset()` clears every robot's state but keeps the roster. Loop the existing
   one-robot code over the container; the risk is not the loop but keeping the host bit-identical: the host
   still resolves its pose from `ctx.chassisBody()` and its zones from its mechanisms, never from a view.
2. **Deterministic order.** `forEach` visits the host first, then remote robots by ascending participant id,
   whatever the join order. Spawning, consuming and random draws (`ctx.rng()`) must happen in that order.
   Update every robot's zones first, then fire all shooters, then absorb with all intakes, so one robot does
   not act on another's half-finished tick.
3. **Alliance.** A robot's alliance is `RobotView::alliance()` (from `RobotView::station`); it is not a plugin
   setting. For the host it is the Driver Station's.
4. **Zones.** A remote robot has no mechanisms. `RobotView::zones` are the zone configs of its Join spec and
   bit `i` of `zoneFlags` is zone `i` of that list, counting zones of other plugins. If the plugin owns only
   some kinds, keep the index of each owned zone in the full list (2026 calls it `zoneBits`) instead of
   assuming the two lists line up. A remote zone is active when `RobotView::enabled && zoneFlagActive(i)`
   (the runtime already forces flags to 0 while the robot cannot act); do not evaluate `activation`. Its
   pose is `zonePose(i)` (what its client commanded, clamped to the robot's envelope), falling back to
   `remoteZoneSpecPose(zone)` ([`zone_pose.h`](../../sdk/include/wpiemu/sdk/game/zone_pose.h)). Put that
   rule in one helper next to the zone runtime so pickups and shooters share it.
5. **Remote shooters have no mechanisms.** There is no flywheel to read, so a remote shooter leaves at a fixed
   speed: the zone's `fixedExitSpeedMps`, else a named constant (2026: `kRemoteDefaultExitSpeedMps`, 8 m/s).
   Make it a constant rather than a config key: a new key changes the config schema and the fixed dumps of
   existing tests. Ignore mechanism references silently, and take the robot velocity from its body.
6. **Caps for client-supplied specs.** A remote client chooses its zone properties (rates, volleys, speeds,
   sizes). Clamp them when the plugin builds the runtime for a remote robot, never for the host, and define the
   numbers once as named constants (2026: the `kRemoteMax*` constants of `fuel_zones.h`). The generic clamp in
   [`remote_spec_limits.h`](../../sdk/include/wpiemu/sdk/physics/remote_spec_limits.h) cannot know a
   plugin's properties; rebuild the clamped runtime on `reset()` so a reset keeps the limits.
7. **Per-robot HUD.** `publishRobot` runs for each remote robot right after the runtime added its
   `RobotSample`; use `SnapshotBuilder::setRobotHud` for its inventory and own-alliance values. The host's
   values stay in `publish()` / `setHud`, and `gameValue()` (sensors) stays the host's.
8. **Contacts and sensors by robot.** Every robot's shapes carry
   `makeRobotTag(participantId)` ([`robot_tags.h`](../../sdk/include/wpiemu/sdk/physics/robot_tags.h)), so
   `ContactEvent` and `SensorEvent` name the robot that touched something (`isRobotTag`,
   `robotTagParticipant`). Judge a visitor by its tag, never by the layer it is on, and keep plugin tags off the
   top byte `0xFE`. A plugin that does not care must ignore robot-tagged visitors.
9. **Entity ids.** Entities that remote clients draw (`EntityGroupSample`) should carry `ids`: one per
   position, stable for the life of the entity within a match and unique within the group. A pool that reuses
   slots needs a separate counter that is not reused until `reset()` (2026: `FuelPool::entityId`), or a
   recycled slot looks like the same entity to a client that delta-encodes the group.

Tests, from cheap to real:

- `RecordingGameState` and `FakeRobotDirectory` (the SDK's testing headers) exercise hooks and order without a plugin.
  2026 wraps the directory in a `Roster` fixture with a `publishAll` snapshot assembler
  ([`multirobot_support.h`](../../tests/game_2026/multirobot_support.h)); copy it for a new season.
- [`remoteRobotContractViolation`](../../sdk/include/wpiemu/sdk/testing/multi_robot_contract.h) drives any
  module with remote robots whose zones are all active and fails a plugin that declares zone kinds but never
  reacts, one that reacts without setting `GameMetadata::supportsMultiRobot`, and one whose outcome depends on
  the order in which robots were announced. [`test_multi_robot_contract.cpp`](../../tests/sdk/test_multi_robot_contract.cpp)
  runs it over every plugin of `registerBuiltinGamePlugins` (and the template in
  [`test_template_plugin.cpp`](../../tests/plugins/test_template_plugin.cpp)), so a new season gets the check by
  registering. It cannot see rules that need a field layout; test those with the roster fixture.
- The real-session rig ([`multirobot_session_rig.h`](../../tests/game_2026/multirobot_session_rig.h)) runs a
  `SimulationSession` with remote robots. Swap the plugin id and zone kinds for a new season. Resend each remote
  input within `kInputTimeoutUs` (250 ms of sim time) or the robot brakes and its flags read 0, and teleport
  remote robots onto the staged game pieces instead of driving them there. `SimulationSession` has no
  `gameState()` accessor, so observe through snapshots.

[`seasons/_template/game`](../../seasons/_template/game/src/game_module.cpp) contains a minimal working
example (a marker zone, per-robot state, `publishRobot`, `supportsMultiRobot`); the built-in 2026 season is the
complete one, described in its [season README](../../seasons/2026/game/README.md#several-robots).

## Publish a game-fed sensor value

Declare a plugin-owned `GameValueInfo` with a label, description and unit, then return the live
number from `IGameState::gameValue(id)`. Do not parse HUD text or cache the value in the module.
Configure an existing Distance or BeamBreak sensor to consume that source and map its `sensor`
slot. Core evaluates the response after the game step and writes device feedback; no plugin-to-
bridge dependency is needed.

The SDK's sensor-response case in
[`test_sensors.cpp`](../../tests/runtime/test_sensors.cpp) checks that a Distance sensor maps a
game value below/above its threshold to the configured near/far readings. The test also covers
interpolation, endpoint clamping and active-low BeamBreak responses.

```powershell
ctest --preset windows-ninja-release -R "^(sensors:|pipeline: zone|validator: zone)"
```

Expected result: sensor response and config validation cases pass. A missing or non-finite source
uses idle output and is reported; it is not a numeric zero. See the
[runtime sensor contract](../reference/runtime.md) for response and idle semantics.

## Request a sound cue

Sound requests are one-shot events, so the SDK transfers them through a bounded queue rather than a
snapshot that the UI may skip. The plugin declares its own cues and requests an ID from game state;
the app owns embedding and playback.

1. Return plugin-prefixed `SoundCueInfo` records from `contributeSoundCues()`.
2. Put each declared WAV at `seasons/<year>/game/assets/sfx/<asset>.wav`. The plugins aggregation seam
   embeds it; no app or Qt dependency is needed.
3. Request the cue on a rule boundary, not every tick. Use the common SDK IDs for shared match
   lifecycle cues; the Driver Station owns those requests.
4. Test queue contents and simulation timestamps with `FakeSimEnvironment::sounds`.

The SDK case `sim context: requestSound stamps the sim time, and an unbound sink discards` verifies
the hand-off: one cue carries the current simulation timestamp, and a context without a sink drops
the request safely.

```powershell
ctest --preset windows-ninja-release -R "^sim context: requestSound stamps the sim time"
```

Season timing and cue ownership vary; see the
[2026 season notes](../../seasons/2026/game/README.md#sound-cues) for that implementation, and the
[game-plugin contract](../reference/game-plugin.md#sound-cues) for the shared lifecycle.

## Finish and verify

Keep plugin config under `RobotConfig::plugins[pluginId]`, preserve unrelated sections, and expose
only SDK descriptors to the app. A new generic mechanism belongs in core only when it is truly
season-independent; season templates register through `IGameModule::registerMechanisms`.

Run the smallest relevant test selection above, then the applicable build and boundary/docs checks
from [Building](../BUILDING.md). A passing fake proves the SDK seam, not physical fidelity or
robot-process integration. Update the [game-plugin reference](../reference/game-plugin.md) only
when a shared contract changes.

Related procedures: [mechanisms and motors](mechanisms.md), [editor properties](editor-properties.md),
and [device adapters](devices.md).
