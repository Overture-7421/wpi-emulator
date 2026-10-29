# Weeks 4–5: keep score, then add the extras

**In 30 seconds:** keep rule state in `IGameState`, put tunable scenario values in the plugin config,
and publish numbers through the SDK. The editor and HUD should describe your game, not contain it.

## 1. Make scoring a ledger, not a pile of booleans

Give each alliance a score ledger with separate entries for autonomous ownership, teleop ownership,
vault cubes, parking, climbing, power-ups, and ranking points. Derive the displayed total from those
entries. That makes a replay explainable and stops reset from subtracting one bonus twice.

Match scoring in the worked example includes auto-run, switch and scale ownership, vault cubes,
platform parking, and climbs. Qualification ranking points include Auto Quest, Face the Boss, and
win/tie. The full rule summary and the deliberate one-robot omissions are in the [fact sheet](appendix-power-up.md).
Do not award ranking points in a practice run just because a HUD can display the number.

Vault cubes score when deposited; activating Force, Boost, or Levitate consumes the appropriate
column's cubes. The Force and Boost columns each hold up to three cubes; one, two, or three cubes
select switch, scale, or both. Force grants temporary ownership, Boost doubles ownership scoring
for its target, and Levitate costs three cubes and credits one climb. Model each activation with a
start and end simulation time, a target, and a once-only cost. Use simulation time; never use
`std::chrono` in a game rule.

**Quick win:** add a HUD counter for total vault cubes and verify that one deposit changes it exactly
once. Then test that reset returns it to zero. A pretty counter is useful; a counter that survives
the next match is a witness.

## 2. Add robot interactions through zones

Describe pickup, exchange, vault, platform, and climb interactions as plugin-owned zone kinds. The
metadata supplies labels, property schemas, default local poses, and shapes. `IGameState` resolves
the configured attachment pose, checks activation, and applies the Power UP rule.

Do not add a special HAL device for a game zone. For a robot-visible reading, declare a game value
and let the existing generic Distance or BeamBreak sensor consume it. For inventory, track the
piece in game state and publish a sensor value; keep the robot-facing device bridge generic.

For an attached zone, follow the repository's [zone interaction procedure](../guides/game-plugins.md#add-a-zone-interaction):
describe the kind in the module, resolve its configured mechanism link in game state, then evaluate
the SDK activation conditions before applying pickup or launch rules. A zone descriptor tells the
editor what the interaction is; it does not perform the interaction by itself.

The single-robot scope matters here. A simulated opponent is not present to push a cube into the
portal or claim a scale plate. Make that an explicit scenario setting, with a default that does not
invent opponent actions.

## 3. Let the generic editor do its job

Add plugin config through `contributeConfigSchema` and typed `ConfigBindings`. Use it for scenario
choices such as initial cube layout, opponent behavior, and whether scoring is enabled. Keep the
schema version explicit; the SDK currently rejects an unsupported plugin schema version rather than
running an undocumented migration.

Use `contributeHud` for score, ownership timers, and cube counts. Use `contributeEditorPanels` only
when the generic property inspector is insufficient. Use `contributeSoundCues` for short, declared
assets, then request a cue on a rule transition—not every simulation tick. The app owns playback and
Qt remains outside the plugin.

Qt is the desktop UI framework; Qt Quick 3D is the run-view renderer. The plugin gives the app plain
SDK descriptors and immutable snapshot data. QML turns the field frame into the renderer's scene
frame, so keep saved poses in the SDK's Z-up NWU coordinates. For the config, zone, sensor, and
sound seams in more detail, use the [game-plugin guide](../guides/game-plugins.md).

**Student asks:** “Can we add a Qt label directly?” You can, but then the plugin owns UI lifecycle,
translation, layout, and test burden. A declared HUD value gets you the label without the side quest.

## 4. The climber is an optional bonus level

Power UP's endgame is a fun place to oversell a simulation. The SDK has no general physics-joint
system. A real hanging robot needs a credible hook, rope, load transfer, and support condition; a
single robot also cannot reproduce alliance-assisted climbing.

For a useful first pass, implement a rule-level climb zone: the robot must enter the endgame window
and meet a configured height/contact condition before the plugin awards a climb. Keep the contact
and score logic separate so a later rope-and-hook model can replace the simplified trigger without
rewriting the score ledger. If you do build a mechanism, exercise it with chassis coupling and test
the support and reset cases.

> **Inspection:** a robot touching a rung is not automatically a climb. A score should be tied to a
> defined condition and a match-time boundary, not a visual guess.

## You now have…

A rule ledger, game values for sensors and HUD, editor-facing config, sound cues, and a bounded
climber design. Next, we install the plugin in a local build and decide whether the result behaves
like a usable emulator game.

Next: [Week 6—registration, integration, and inspection](05-test-and-run.md).
