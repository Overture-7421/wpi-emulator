# Build a game plugin

Use this guide to add a season, connect robot mechanisms to game behavior, or expose game state to
robot sensors. Start with the [plugin contract](../reference/game-plugin.md); the
[2026 plugin notes](../../seasons/2026/game/README.md) are season-specific and are not a template
for other games.

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

1. Copy [`seasons/_template/game`](../../seasons/_template/game) to `seasons/<year>/game` and follow its
   [identity checklist](../../seasons/_template/game/README.md). Keep the CMake target, namespace,
   include tree, plugin ID and game year consistent; depend on SDK only.
2. Implement a stateless `IGameModule`, an `IField` that destroys its bodies before its world, and
   an `IGameState` that resets all per-session state. Use simulation time and context RNG.
3. Return plugin-prefixed metadata and register the module through `seasons/CMakeLists.txt`,
   `seasons/builtin_plugins.cpp`, and the test target. The app uses the aggregator, not a direct
   season dependency.
4. Start with a selectable year, a resettable state and one observable HUD value. Then add config
   with `ConfigBindings`, field fidelity, rules and entities.
5. Add focused tests for registration, selection, defaults, field cleanup, repeated reset, rule
   boundaries and deterministic behavior. Use a real physics test where contact geometry matters.

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
