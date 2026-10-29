# Appendix B: pit inspection checklist

## Before the prototype's first match

- [ ] Season manifest pins the intended WPILib release, ABI, Java major, and launch profile.
- [ ] The plugin ID, folder, namespace, target, manifest entry, and registration agree.
- [ ] The game plugin links only the SDK.
- [ ] Field visuals and colliders use the same poses, units, and coordinate frame.
- [ ] Field and game-state destructors release every body they own before the physics world ends.
- [ ] Reset restores game pieces, plate states, inventory, timers, scores and game data.
- [ ] The match clock follows Driver Station mode/time, including Practice durations.
- [ ] HAL game data reaches autonomous initialization before the first autonomous tick.
- [ ] Zone event logic handles entry, exit, repeated overlap, and start-inside cases.
- [ ] Rule tests cover point rates, disabled/expired-time/eStop guards and endgame boundaries.
- [ ] Real-physics tests cover contacts, pyramid stability and preload/reserve conservation.
- [ ] C++ and Java robot projects compile with the selected season's installed toolchains.
- [ ] `check_seasons.py`, `check_boundaries.py`, and `check_docs.py` pass.
- [ ] The Qt app was launched to inspect Quick 3D rendering; an offscreen test is not a substitute.
- [ ] Any unsupported feature or simplification is visible in the guide and the plugin description.

## Before claiming complete Power Up

The provided implementation does **not** pass this second inspection:

- [ ] Official discrete scoring, eligibility, penalties and qualification context are implemented.
- [ ] Human feeds and exchange/vault transfers preserve cube conservation.
- [ ] Force/Boost/Levitate allocation, activation, expiry and reset have tests.
- [ ] Parking and supported climb awards have real-physics and rule-boundary tests.
- [ ] Ranking points are tested against the actual alliance qualification conditions.

## If something breaks

| Symptom | First place to look |
|---|---|
| Game is missing from the selector | Manifest `game` entry, target aggregation, plugin aggregator, plugin metadata ID/year |
| Robot sees an empty game message | Reset timing, `IGameState::gameSpecificMessage`, `DsState` forwarding, selected season's robot API |
| Cube passes through the field | Shape category/mask, static body creation, collider pose, shape lifetime |
| Sensor never fires | Sensor flag, sensor events on both shapes, mask, visitor shape, reset overlap |
| Score differs by match mode | Driver Station mode/time, phase transition, fixed-step accumulator |
| Reset leaves a ghost body | Field/game-state ownership and destructor order |
| HUD is blank | Plugin-prefixed key, declared descriptor, matching published key |
| Plugin cannot include a library | Recheck the boundary; game plugins intentionally depend only on SDK |
| Field looks wrong only in the app | Check Quick 3D rendering interactively; offscreen widget tests do not render the scene |

The [game-plugin guide](../guides/game-plugins.md), [physics reference](../reference/physics.md),
[season reference](../reference/seasons.md), and [building guide](../BUILDING.md) remain the
authoritative repository procedures and contracts.
