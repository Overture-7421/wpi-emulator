# Appendix C. Conformance, traceability and evaluation

## C.1 Conformance classification

Implemented means the stated rule has an executable test. Approximate means
the model substitutes a stated operational condition or geometry. Scripted
means a declared agent supplies the input. Omitted means no such behavior is
claimed. A row can contain more than one classification.

| ID | Feature and classification | Model / chapter | Principal test file |
|---|---|---|---|
| F-01 | Field: approximate | Shared visual/collider boxes, 6 and 16 | <code>test_field.cpp</code> |
| F-02 | Starting arrangement: implemented, approximate centres | Fence rows and stable 6/3/1 piles, 7 | <code>test_cube_layout.cpp</code> |
| F-03 | Cubes: approximate | Rigid boxes, nominal mass and materials, 3 and 7 | <code>test_pool.cpp</code>, <code>test_field.cpp</code> |
| F-04 | Supply and preload: implemented for one robot | Conserved reserve transfer; no escaped-cube recovery, 7 and 16 | <code>test_pool.cpp</code>, <code>test_human_player.cpp</code> |
| F-05 | Assignment: implemented | Four strings, both alliance views, 4 and 11-13 | <code>test_config.cpp</code>, <code>test_plate_scoring.cpp</code>, process match |
| F-06 | Timeline: implemented | Driver Station windows, 9 | <code>test_match_clock.cpp</code>, process match |
| F-07 | Plate mechanics: approximate | Count-driven horizontal heave, 8 and 15 | <code>test_plate_motion.cpp</code> |
| F-08 | Ownership: implemented, approximate observation | Sampled height and 1 s acquisition, 15 | <code>test_rules_ownership.cpp</code>, <code>test_plate_scoring.cpp</code> |
| F-09 | Point values: implemented; accounting policy explicit | Fixed-point integration and floor, not official engine equivalence, 15 | <code>test_rules_ownership.cpp</code> |
| F-10 | Auto-run: approximate bumper geometry | Leading rotated chassis extent, 15 | <code>test_rules_auto.cpp</code>, process match |
| F-11 | Human feeding: scripted | Conserved portal transfer during teleoperation, 16 | <code>test_human_player.cpp</code> |
| F-12 | Exchange: implemented, approximate endpoints | Lower aperture, hold/upper return/vault, 16 | <code>test_exchange.cpp</code>, <code>test_vault_physics.cpp</code> |
| F-13 | Vault: implemented, nonphysical columns | Capacity, permanent custody and 5 points, 17 | <code>test_rules_vault.cpp</code> |
| F-14 | Force: implemented, scripted presses | Global timed slot and opponent queue, 17 | <code>test_rules_power.cpp</code>, <code>test_rules_power_scoring.cpp</code> |
| F-15 | Boost: implemented, scripted presses | Owned targets, latched level and split expiration, 17 | <code>test_rules_power.cpp</code>, <code>test_rules_power_scoring.cpp</code> |
| F-16 | Levitate: implemented | One eligible credit, no double parking or fourth climb, 19 | <code>test_rules_ranking.cpp</code> |
| F-17 | Parking: approximate | Footprint, height and speed proxy at finish, 18-19 | <code>test_climb.cpp</code>, <code>test_rules_ranking.cpp</code> |
| F-18 | Climbing: approximate | Direct anchored support; no transitive robot support, 18 | <code>test_climb.cpp</code>, real-HAL feature case |
| F-19 | Other robots: scripted | Virtual auto-runs/finishes; one physical opponent cube drop, 19 | <code>test_scenario_session.cpp</code>, process scenario |
| F-20 | Ranking points: implemented | Win/tie, Auto Quest, Face the Boss; no playoff RP, 19 | <code>test_rules_ranking.cpp</code> |
| F-21 | Penalties: omitted | Including illegal-contact forced-climb adjudication | No conformance claim |
| F-22 | Historical vision targets: omitted | No retroreflective model; empty AprilTag layout | <code>test_presentation.cpp</code> checks only the empty tag layout |
| F-23 | Sounds: implemented requests, original asset | Edge-triggered cues and generic editor panel, 20 | <code>test_presentation.cpp</code> |
| F-24 | Robot legality: omitted | Chassis and fixed chute are a test fixture | No inspection claim |
| F-25 | Cube orientation rendering: omitted | Physics rotates boxes; entity groups publish centres only | SDK format boundary, 11 and 22 |

## C.2 Requirement traceability

| Requirement | Evidence |
|---|---|
| R1 | Module identity test, season and registry edits |
| R2 | SDK-only CMake dependency and boundary checker |
| R3 | Shared box construction and physical wall test |
| R4 | Pool custody test, physical pickup/release and human transfer tests |
| R5 | Staging counts, symmetry and five-second drift test |
| R6 | Physical plate loading plus pure acquisition tests |
| R7 | Phase boundary tests and disabled periods in real matches |
| R8 | Pipeline selection, module message and robot console assignment |
| R9 | Real-HAL drive, delivery, vault and climber cases |
| R10 | Reset epoch, restored inventory, repeated hashes across worker counts |
| R11 | Snapshot/descriptor/schema comparison and recorded sound requests |
| R12 | C.1 and Chapter 22 |
| R13 | Fresh baseline export, cumulative Markdown extraction and builds |

## C.3 Evaluation record

The reproduction baseline is commit <code>103a8ea</code>. The source is extracted
from the Markdown, not copied from the development prototype. The environment
is Windows x64, MSVC 14.51, Ninja, Release, warnings as errors, and the season's
2027.0.0-alpha-7 HAL with JDK 25.

| Question | Observation from the reproduction |
|---|---|
| RQ1: invariants and integration | 28 selected plugin, physical and session tests passed, including three worked exercises; 38 pure-rule cases passed 428 assertions |
| RQ2: repeatability | All 1,000 per-step physics hashes matched in four runs with worker counts 1, 1, 2 and 4 |
| RQ3: ordinary robot delivery | Both languages delivered physically, completed disabled windows and repeated autonomous after an epoch reset; scores appear in Table C.1 |
| RQ4: mean fixture tick cost | RESULTS_TIMING |

| Additional check | Recorded result |
|---|---|
| Cumulative checkpoints | Chapters 5-22 built and passed their selected tests; Chapter 14's core matches were also reproduced independently |
| Real-HAL vault and climb | Java and C++ each deposited one cube for 5 vault points, froze Force at 7.000 s while disabled, and achieved a supported climb at chassis Z = 0.500 m for 30 endgame points |
| Optional partners/opponent match | RESULTS_SCENARIO |
| Desktop rendering and audio | RESULTS_DESKTOP |
| GCC 14 portability | Not run: the installed Docker CLI could not connect to its local Linux engine |

Table C.1. Complete real-HAL match observations from the core and completed model.
Body counts include disabled pooled cubes.

| Model | Robot | Autonomous score | Final score | Simulated time (s) | Allocated bodies |
|---|---|---|---|---|---|
| Chapter 14 continuous baseline | Java | 15.360 | 150.360 | 156.000 | 83 |
| Chapter 14 continuous baseline | C++ | 15.360 | 150.360 | 156.000 | 83 |
| Completed fixed-point model | Java | 16 | 151 | 156.000 | 87 |
| Completed fixed-point model | C++ | 16 | 151 | 156.000 | 87 |

Figure C.1. Initial custody observed in the published snapshot. Each `#`
represents one cube; a held cube has no enabled collider.

```text
Field    ################################  32
Held     #                                  1
Reserve  ###########################       27
Total                                      60
```

Tick timing excludes UI drawing and robot-process waits. The hash experiment
covers one fixed input history and the pinned backend, not cross-platform
bitwise determinism. The desktop entry records only what was inspected; an
offscreen test is not substituted for rendering.

[Appendix D](appendix-d-glossary.md)
