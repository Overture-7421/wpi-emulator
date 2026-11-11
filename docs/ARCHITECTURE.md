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
| `core` | SDK; Box3D privately | Configuration/runtime, mechanisms, plants, physics adapter, and the practice-match server (`core/match`) |
| `ipc` | SDK | Local transport and message codec |
| `net` | SDK, IPC; ENet and Monocypher privately | ENet and relay transports, invite codes and the end-to-end `SecureTransport` |
| `relay` | SDK, IPC, `net` | The relay server, its probe and tests (`wpiemu_relay`) |
| `hal_abi` | WPILib headers | Per-HAL-ABI includes, HAL clock and SimDevice constants for robot-process modules |
| `bridges/hal` | SDK, `hal_abi`; HAL headers inside its binding | WPILib HAL device families; per-ABI sources in `abi/<abi>` |
| `bridges/vendor` | SDK, `hal_abi`; backend headers inside the module | Vendor devices exposed through SimDevices |
| `bridges/vision` | SDK, `hal_abi`; ntcore headers inside its binding | Camera-coprocessor encoders and publishing; generated PhotonLib catalog |
| `bridges/` | SDK and bridge modules | Built-in bridge registration |
| `halsim_ext` | SDK, IPC, bridge aggregator, `hal_abi`, WPILib HAL | Robot-process extension; per-ABI platform calls in `src/hal/abi/<abi>` |
| Each `seasons/<year>/game` | SDK only | One game's field, entities, rules, and metadata |
| `seasons/` | SDK and game plugins | Season packs (`season.json`), built-in plugin registration and the generated season catalog |
| `app` | SDK, core, IPC, `net`, season and plugin aggregators, Qt | Desktop composition, host (including practice-match hosting), editor, and run view |
| `app/contract` | SDK and Qt | Host interface shared by editor and run view |
| `app/ui` | SDK and Qt | Shared Qt support: 3D scene helpers, the key-value settings port and the relay presets |
| `app/editor` | SDK, Qt, `app/ui` | Editor page |
| `app/runview` | SDK, Qt, `app/ui`, `app/contract` | Run page |
| `app/client` | SDK, core, `net`, plugin aggregator, Qt, `app/ui`, `app/contract`, `app/runview` | `wpiemu_client`, the thin practice-match game client: join form, session, drive screen |

Qt stays out of SDK, core, IPC, `net`, `relay`, bridges, and plugins. Box3D and robot-process backend headers do
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

## Who owns the simulated world

**The host desktop app, and only it.** Even in a practice match with remote players there is one
Box3D world, owned by the host's `SimulationSession` on the simulation thread. The server is
authoritative; nobody else holds authoritative state.

| Party | Owns | Does not own |
|---|---|---|
| Host app (simulation thread) | The physics world, the host's real WPILib robot, every remote robot's chassis and wheel controller, game state, sim clock, tick order, match rules (through the host Driver Station), roster and epoch | |
| Game client | Its own input, a disposable prediction world that authoritative state overwrites, interpolation and rendering | Other robots, game pieces, scores, match time |
| Relay | The room table | Any simulation or readable game data (traffic is end-to-end encrypted) |

`MatchHost` (`core/match`) is a game server that only feeds the session scheduled inputs through its
inbox and reads snapshots; it does not own the world. It runs as the runner's tick observer on the
simulation thread, while connection reader threads and per-client sender threads handle I/O, so the
simulation never blocks on a network peer. The world still advances only while the host's robot is in
lock-step. Clients trust the host. See [Match network](reference/match-network.md) for the protocol,
encryption and trust model, and [Runtime](reference/runtime.md#remote-robots-practice-matches) for how
remote robots are simulated.

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
| Match I/O threads | When hosting: one ENet pump thread per transport, a reader thread and a sender thread per client. They move messages and never touch the session or HAL callbacks. `MatchHost` itself runs on the simulation thread through the runner's tick observer. |

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
3. The drivetrain computes wheel forces. In a practice match every remote robot then applies the input
   due on this tick, turns its wheel setpoints into motor voltages and adds its own tire forces
   (in participant order). Runtime applies chassis/payload coupling, then advances the physics world
   once for every robot; each remote robot completes its step after it.
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

Practice-match frames follow the same rule with their own identity: the host assigns a `sessionId` to
each client connection and bumps `runEpoch` on every reset, and a client rejoins after a drop with its
resume token. The robot watchdog is untouched; if it fires, clients see `HostStalled`.

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
