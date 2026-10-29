# Appendix A: Power UP field and rule card

Use this as the example's lookup sheet, not as a substitute for the official 2018 Game and Season
Manual or field drawings. The simulator models only the rules it declares; this card separates match
facts from the book's approximations.

## The field at a glance

| Item | Nominal measurement | Simulation treatment |
|---|---:|---|
| Carpet | 54 × 27 ft (16.46 × 8.23 m) | rectangular field frame, rotational symmetry |
| Auto line | 10 ft (3.05 m) from each alliance wall | painted visual plus line/zone used by auto-run rule |
| Power Cube | 13 × 13 × 11 in (33 × 33 × 27 cm), about 1.5 kg | dynamic box; drawn axis-aligned |
| Switch plate | 3 × 4 ft (0.91 × 1.22 m) | level kinematic plate with ownership height states |
| Scale plate | 3 × 4 ft (0.91 × 1.22 m) | level kinematic plate with ownership height states |
| Starting cubes | 30 per alliance, 60 total | staged from the official layout drawing; resettable pool |

Use the official [2018 FRC Game and Season Manual](https://firstfrc.blob.core.windows.net/frc2018/Manual/2018FRCGameSeasonManual.pdf)
and [Power UP-specific field drawings](https://firstfrc.blob.core.windows.net/frc2018/Drawings/FieldDrawings-FIRSTPOWERUPspecific.pdf)
for exact field geometry and rule wording. The values above are nominal, rounded conversions; the
drawing is authoritative for placements.

| Field element | Nominal measurement or placement |
|---|---|
| Switch | Centered across width, 14 ft (4.27 m) from each alliance wall; plates are 9 in (0.23 m) high when level |
| Scale | Center of the field; level plate height 5 ft (1.52 m); outer plate edges 15 ft (4.57 m) apart |
| Platform zone | 11 ft 1.5 in × 9 ft 11.75 in (3.39 × 3.04 m), between switch and scale |
| Vault | Three columns, each with a capacity of three cubes: Force, Levitate, Boost |
| Pre-match pieces | 14 at the two portals, 6 beside the switch, and 10 in the switch-side pyramid per alliance |

## Scoring and game data

- Auto Run: 5 points per robot that crosses the Auto Line during autonomous.
- Switch and scale ownership: 2 points per second in autonomous; 1 point per second in teleop.
- Vault cube: 5 points per cube.
- Platform parking: 5 points; climb: 30 points.
- Auto Quest: one ranking point for the qualification condition in the manual.
- Face the Boss: one ranking point for the qualification endgame condition in the manual.
- Qualification win/tie: two/one ranking points.
- The final 30 seconds of teleop are Endgame.
- Force and Boost last 10 seconds. Each costs one cube for switch, two for scale, or three for both;
  Levitate costs three vault cubes and credits one climb.

The game-specific message has three characters in alliance-view order: near switch, scale, far
switch. Legal assignments are `LLL`, `LRL`, `RLR`, and `RRR`. Both alliances receive the same
message.

Check exact award timing, eligibility, and ranking-point conditions against the linked official
manual before implementing a competition-faithful scoring engine. This book's one-robot simulator
does not model the opponent alliance, human players, fouls, or all qualification context.

## Deliberate approximations

1. Plates move vertically and remain level. The SDK has no physics joints, and the current entity
   renderer has no orientation for instanced entities.
2. Cubes can tumble in physics but render axis-aligned.
3. Opponent actions are scenario inputs, not a second autonomous robot.
4. Climbing is a rule trigger or an explicitly tested simple mechanism, not a general rope solver.
5. The game has no AprilTags; use its 2018 vision targets only where the official drawings specify
   them. Do not invent a tag layout for robot localization.
