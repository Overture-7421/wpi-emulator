# 2026 REBUILT game plugin

This page documents the season-specific behavior implemented by `game_2026`. It is not a general
game-plugin recipe; use the [game-plugin guide](../../../docs/guides/game-plugins.md) to create another
season, and the [shared contract](../../../docs/reference/game-plugin.md) for SDK ownership and
lifecycle.

## Mental model

The plugin turns generic robot mechanisms and field contacts into REBUILT game behavior. The
mechanism still owns its motor-driven motion; plugin-contributed pickup and shooter zones connect
that motion to fuel inventory. The plugin owns per-session fuel, match rules, scoring and outposts.
Core owns generic mechanisms, sensors and physics, and the app displays published snapshots.

The plugin registers no custom mechanism templates or editor panels. Its config, field dimensions,
zone properties and rule implementation are in the linked source files below; those declarations
are authoritative when a value in this overview becomes stale.

## Robot fuel interactions

- A pickup collects fuel whose center lies inside its oriented box while its activation condition is
  true and the robot meter has room. Candidates are taken nearest the box center first.
  `intakeRateHz` limits collection; `0` is unlimited.
- A shooter fires only while activation is true, inventory is nonempty, its configured references
  are valid, and exit speed is positive and at least `minExitSpeedMps`.
- With a flywheel reference, exit speed is the absolute published angular speed multiplied by
  radius and efficiency. A positive `fixedExitSpeedMps` bypasses the flywheel reference.
- Hood and turret readings affect aim. Attaching the muzzle to a turret link already rotates it;
  do not apply that turret angle again through a second reference.
- Missing named aim or speed references warn once per zone/mechanism and suppress firing. A fixed
  exit speed bypasses an otherwise unused flywheel reference.
- `fireRateHz` is total fuel per second, not volleys per second. `parallelBalls` lays each volley
  side by side; the final volley can be partial when inventory runs out.
- Optional seeded speed/aim jitter and inherited chassis point velocity affect launched fuel.
  `shotGhostS` temporarily suppresses collisions with the robot and other newly shot fuel; it does
  not suppress field, settled-fuel or scoring interactions.
- A published zone's `active` state reports activation only. It does not promise that the shooter
  is ready to fire or has inventory.

See [`fuel_zones.cpp`](src/fuel_zones.cpp) for schemas and activation, speed, aim and rate handling;
[`game_state.cpp`](src/game_state.cpp) implements inventory, pickup, firing and per-tick updates.
The season tests under [`tests/game_2026`](../../../tests/game_2026) cover these rules.

## Match clock, hubs and scoring

`DsState::matchTimeS` is interpreted as seconds remaining in the current period when nonnegative.
The timed timeline is 20 seconds of AUTO, then 140 seconds of TELEOP: a 10-second transition,
four 25-second alliance shifts and a 30-second END GAME. The clock uses simulation steps when no
Driver Station countdown is supplied. Without a timed match, enabled play behaves as free play with
both hubs active and no timed-match HUD; Test mode also has both hubs active.

While the match is running, both hubs are active during AUTO, transition and END GAME. During
shifts, the first-inactive alliance is selected at TELEOP start in this order:
`firstInactiveHub` override, FMS game-specific message (`R` or `B`), AUTO fuel totals, then a
context-RNG tie break. `enforceHubShifts = false` keeps both hubs active while play is running.
Once latched, the selected alliance is published to robot code as the game-specific message and
takes precedence over manual Driver Station text.

Fuel entering a hub sensor is consumed. It scores while that hub is active or within the three
second scoring grace period after deactivation; other fuel is consumed without points. With
`recycleScoredFuel` enabled, scored fuel is queued for release after `hubProcessingDelayS`.
Capacity-limited or refused spawns remain pending for retry. Fuel entering an outpost corral becomes
chute stock; the optional timed human-player setting releases stock while a match is running.

After a timed match stops, the final score remains visible as `FINAL` until reset, another timed
match, or enabled free play. The scoreboard places Blue on the left and Red on the right. The
plugin publishes fuel entities, zones, inventory/counters, scores and hub status through snapshots.

Rules are implemented in [`match_clock.cpp`](src/match_clock.cpp),
[`hub_rules.cpp`](src/hub_rules.cpp), and [`game_state.cpp`](src/game_state.cpp). Config keys,
defaults, ranges and descriptions come from [`config.cpp`](src/config.cpp).

## Field and staged fuel

The field models alliance walls, guardrails, hollow hubs, bumps, trenches, tower rungs, depots and
outpost openings/corrals. Red geometry is the blue geometry rotated about the field center. Wall
and guardrail colliders remain solid for robots and fuel; visuals distinguish opaque structure
from clear plastic so the field can be seen through transparent barriers. Rigid structure uses the
shared `kFieldStructure` [contact material](../../../docs/reference/physics.md#contact-materials)
(slick, low bounce) so a robot driven at an angle into a wall slides along it; bump ramps and fuel
keep their own materials.

Each trench has a 12 in support beside the bump and a 6 in-long, 4 in-high arm across the 50.34 in
opening at the 22.25 in clearance; the rest is open overhead, and the tag panels above the arm are
not colliders.

Alliance-wall visuals model the driver-station base, sponsor band, clear sheet, top rail and shelf;
the tower wall and outpost are opaque. Guardrails are drawn as clear plastic between aluminum
extrusions while their collision geometry remains solid.
The field description also provides six Driver POV eye positions centered behind the station shelves;
the tower separates stations 2 and 3, matching the field layout.

Fuel is simulated as rigid spheres. Reset stages the configured field fuel and restores robot
inventory and chute stocks. A body-count limit bounds creation, and fuel that leaves the configured
play bounds is removed.

This is an intentionally bounded model, not full field CAD or every rule interaction:

- The hub omits its funnel, internal ramps and back net.
- Tower supports and climb scoring, ranking points, penalties, moving trenches, and outpost doors
  and throwing are not simulated.
- Pickup has no jamming or mouth-contact model. Shooting has no hopper, indexer or flywheel
  compression model.
- Inventory belongs to the single simulated robot, not an alliance of robots.
- Bump ramps preserve slope and ridge height but extend to the carpet without an edge step, so
  their footprint is deeper than nominal.

Dimensions and provenance tags are in
[`field_dimensions.h`](include/wpiemu/plugins/game_2026/field_dimensions.h): **GM** means the
[FIRST game manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf),
**AT** is WPILib's welded AprilTag dataset, and **MS** is
[MapleSim community geometry](https://github.com/Shenzhen-Robotics-Alliance/maple-sim).
Derived or approximate dimensions are marked in source; this is not an official CAD model.
Construction is in [`field_layout.cpp`](src/field_layout.cpp). The welded 2026 tag layout is
defined in [`april_tags.cpp`](src/april_tags.cpp) and checked against its source dataset in
[`test_april_tags.cpp`](../../../tests/game_2026/test_april_tags.cpp). Match and hub behavior are in
the files linked above.

## Game-fed sensor values

The plugin exposes these live numeric values to generic Distance and BeamBreak sensors:

| ID | Unit | Meaning |
|---|---|---|
| `game_2026.fuel.count` | count | Fuel currently held by the robot |
| `game_2026.fuel.capacity` | count | Configured robot inventory capacity |
| `game_2026.fuel.fraction` | ratio | Count divided by capacity; zero when capacity is zero |

The IDs and units are declared in [`game_values.cpp`](src/game_values.cpp); live values are
returned by `Game2026State::gameValue()` in [`game_state.cpp`](src/game_state.cpp).

## Sound cues

The plugin declares `game_2026.shift_change` for an alliance shift. It requests that cue only when
a running TELEOP crosses into a shift; the first TELEOP tick, free play, Test mode and re-enabling
inside a phase are silent. The SDK common cues are owned by the Driver Station where noted:

| Event | Cue | Owner |
|---|---|---|
| MATCH start, AUTO ends, TELEOP begins, MATCH ends | `match.start`, `match.auto_end`, `match.teleop_start`, `match.end` | Driver Station |
| MATCH stopped | `match.stopped` | Driver Station, E-Stop of a running Practice match |
| ALLIANCE SHIFT starts (2:10, 1:45, 1:20, 0:55) | `game_2026.shift_change` | Plugin |
| END GAME begins (0:30) | `match.end_game` | Plugin |
| Final score is posted | `match.result` | Plugin, after scoring assessment |
| Practice restart | `field.reset` | Driver Station |

The result cue waits until the three-second scoring assessment closes and is skipped after E-Stop.
The plugin does not request the lifecycle cues owned by the Driver Station. Declaration and
boundary handling are in [`game_sounds.cpp`](src/game_sounds.cpp) and
[`game_state.cpp`](src/game_state.cpp); focused cases are in
[`test_sound_cues.cpp`](../../../tests/game_2026/test_sound_cues.cpp). The generic queue, asset and
playback contract is in the [game-plugin reference](../../../docs/reference/game-plugin.md#sound-cues).

Keep fuel collision masks compatible with field sensors in both directions, including the
temporary shot category. The shot-ghost setting suppresses only robot/new-shot-fuel collisions;
it must not suppress field, settled-fuel or scoring interactions. Fast fuel needs adequate sensor
depth so it cannot skip overlap detection; physics limits are described in the
[physics reference](../../../docs/reference/physics.md).

## Verify this season

```powershell
ctest --preset windows-ninja-release -R "^(initial staging follows|pickup zone absorbs|teleop phase table|hub schedule alternates|fuel dropped into a hub|sound cues:|field layout preserves)"
```

The season tests exercise rules, zones, config, reset, snapshots and integration with generic
mechanisms. Real-physics cases are needed for geometry and fuel movement; fake-only tests do not
establish collision fidelity. Use the [Building guide](../../../docs/BUILDING.md) for presets and the
[game-plugin guide](../../../docs/guides/game-plugins.md) for extension procedures.
