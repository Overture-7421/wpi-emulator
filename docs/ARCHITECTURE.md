# Architecture

Read this when deciding where a change belongs or who owns simulation state.

WPI Emulator has two processes. The Qt desktop app owns the simulated world. An extension loaded
inside the robot process connects back to the app and translates between WPILib simulation state
and emulator messages. The robot project remains unmodified. For background on WPILib's simulation
hooks, see [How WPILib simulation works](concepts/wpilib-simulation.md). The
[glossary](GLOSSARY.md) defines shared terms.

```text
Desktop app                                      Robot process
UI -> queued commands                                robot code -> WPILib HAL
          |                                                  |
SessionRunner <--------------- local IPC --------> HALSim extension
          |                                                  |
SimulationSession                                  device and vision bridges
mechanisms -> drivetrain -> physics
game plugin -> sensors -> immutable snapshot
```

These arrows show data flow, not permission to depend on another module. A bridge translates
robot-facing device signals. A mechanism models a physical load. Shared SDK types do not merge
those responsibilities.

## Module boundaries

These are the production dependency rules. Tests may combine modules to verify their contracts.

| Module | May depend on | Owns |
|---|---|---|
| `sdk` | C++ standard library | Header-only interfaces, values, registries (game plugins, seasons), protocol types, and fakes |
| `core` | SDK; Box3D privately | Configuration/runtime, mechanisms, plants, and physics adapter |
| `ipc` | SDK | Local transport and message codec |
| `hal_abi` | WPILib headers | Per-HAL-ABI includes, HAL clock and SimDevice constants for robot-process modules |
| `bridges/hal` | SDK, `hal_abi`; HAL headers inside its binding | WPILib HAL device families; per-ABI sources in `abi/<abi>` |
| `bridges/vendor` | SDK, `hal_abi`; backend headers inside the module | Vendor devices exposed through SimDevices |
| `bridges/vision` | SDK, `hal_abi`; ntcore headers inside its binding | Camera-coprocessor encoders and publishing; generated PhotonLib catalog |
| `bridges/` | SDK and bridge modules | Built-in bridge registration |
| `halsim_ext` | SDK, IPC, bridge aggregator, `hal_abi`, WPILib HAL | Robot-process extension; per-ABI platform calls in `src/hal/abi/<abi>` |
| Each `seasons/<year>/game` | SDK only | One game's field, entities, rules, and metadata |
| `seasons/` | SDK and game plugins | Season packs (`season.json`), built-in plugin registration and the generated season catalog |
| `app` | SDK, core, IPC, season and plugin aggregators, Qt | Desktop composition, host, editor, and run view |
| `app/contract` | SDK and Qt | Host interface shared by editor and run view |
| `app/ui` | SDK and Qt | Shared Qt support, including 3D scene helpers |
| `app/editor` | SDK, Qt, `app/ui` | Editor page |
| `app/runview` | SDK, Qt, `app/ui`, `app/contract` | Run page |

Qt stays out of SDK, core, IPC, bridges, and plugins. Box3D and robot-process backend headers do
not leak into SDK headers. The app registers game plugins through
[`seasons/builtin_plugins.cpp`](../seasons/builtin_plugins.cpp) and WPILib season profiles through
the catalog generated from the season manifests; the extension registers adapters
through [`bridges/builtin_bridges.cpp`](../bridges/builtin_bridges.cpp). Neither consumer links
directly to an individual season or bridge. The [boundary checker](../tools/check_boundaries.py)
enforces these ownership rules.

## WPILib seasons

WPILib support is data plus a small number of per-ABI adapters ([Season packs](reference/seasons.md)).
A season pack (`seasons/<year>/season.json`) declares the WPILib pin, verified versions, install
folder, Java major, HAL ABI, launch profile, vendor schemas and an optional game plugin. Core and the
app read the generated `SeasonRegistry` and never name a year. HAL differences follow the ABI, not the
year: each season builds its own HALSim extension, and only robot-process code includes `hal_abi` and
the per-ABI sources. A season therefore needs a game plugin only for game rules; a platform-only season
runs core-only.

## Processes and threads

`AppServices` composes the app. The editor and run view share one `IRuntimeHost`.
`RobotSession` supervises the robot process and composes IPC with the runtime.
`SimulationSession` owns synchronous simulation. `SessionRunner` adds queues, IPC, snapshots,
watchdog handling, and pacing. See the [application contract](reference/app.md) for the UI seam.

| Owner | Responsibility |
|---|---|
| UI thread | Reads immutable `ISimSnapshot` objects and submits commands through `ISimController`; it never reads mutable simulation state. |
| Simulation thread | Owns `SimulationSession`, physics, mechanisms, device hub, game state, config, and random generator. |
| Physics workers | Box3D helpers run only while the simulation thread calls `step()`; they do not access other state and are joined when the world is destroyed. |
| Robot threads | Run the team's code. HAL callbacks record events only; they must not block or perform socket I/O. |
| Extension clock thread | Applies device feedback, advances HAL timing, and collects robot outputs. |
| Extension IPC workers | Move messages to and from queues; they do not mutate the simulated world. |
| Extension watchdog | Recovers independently of bridge locks when the emulator stalls or disappears. |

`SimulationSession` creates no threads and performs no I/O. `SessionRunner` owns the simulation
thread and applies queued UI/control commands there. During lock-step, bridge work happens around
a robot step, not concurrently with it. Free-running discovery is different: robot code can run
while the extension discovers devices.

## One simulation tick

The emulator owns simulation time. Its fixed step defaults to 5 ms and must divide the robot's
20 ms period. Wall-clock pacing controls when a tick is sent; it does not determine simulated time.

For tick N:

1. `SessionRunner` applies queued commands and handles the previous acknowledgment. It sends no new
   tick while another is outstanding.
2. `SimulationSession` advances mechanisms using the latest actuator outputs and mapped slots.
   Motion-linked axes follow dependency order; mechanisms also submit chassis coupling.
3. The drivetrain computes wheel forces. Runtime applies chassis/payload coupling, then advances
   the physics world once.
4. Game state consumes the resulting world events. The plugin's `onTick` hook follows; game-fed
   sensors then translate published values into device feedback.
5. Runtime updates IMU and battery feedback, advances its integer clock, and builds a sensor frame.
   Due camera results join that frame.
6. In the robot process, the extension applies device and vision feedback, calls
   `HALSIM_StepTiming`, then polls bridges and collects robot outputs into an `ActuatorFrame`.
7. The runner accepts only the matching acknowledgment, applies its outputs to the device hub, and
   then may send tick N+1.

The UI receives immutable snapshots; it does not participate in the tick. See
[simulation and physics](concepts/simulation-and-physics.md) for the model and units, and
[Extension and IPC](reference/extension-ipc.md) for the wire contract.

## Reset, reconnect, and failure

`sessionId` identifies a robot connection. `runEpoch` distinguishes runs within it. Both are
carried by frames and controls. A reset starts a new epoch; reconnect creates a new session and
epoch. Tick numbers can restart, so they are not identities on their own. Reject frames for other
sessions and stale epochs.

Control command IDs and acknowledgments make retries safe: a command must not be applied twice.
Reset clears simulation signal history and starts the next sensor frame with a full baseline, but
keeps configured mappings and discovered devices. The extension clears bridge input history and
vision state at lifecycle boundaries. A reset does not relaunch robot code or clear robot-owned
firmware offsets.

Only one tick may be outstanding. If the robot does not answer before the negotiated watchdog
deadline, the runner suspends lock-step and stops sending ticks. The extension has its own
watchdog: it resumes HAL timing or terminates the robot according to policy, without waiting for
the emulator. A late answer cannot complete a tick after the watchdog fires. Reset or reconnect
opens a new tick window. Keep this independent recovery path intact when changing queues, logging,
or shutdown. Details belong to the
[Extension and IPC contract](reference/extension-ipc.md).

## Extension points

Use the [extension seam map](extending.md) to choose an owner before adding simulator behavior.
The [codebase tour](CODEBASE_TOUR.md) traces the main flows through source and tests.

Next: [tour the codebase](CODEBASE_TOUR.md).
