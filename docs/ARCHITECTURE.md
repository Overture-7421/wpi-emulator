# Architecture

Read this when you want to understand where a change belongs or what owns simulation state.
Prerequisites: the [project overview](../README.md); no WPILib internals are assumed.

WPI Emulator runs unmodified robot code in a separate process. The desktop app owns the simulated
world; a HALSim extension inside the robot process exchanges device data with it. Game plugins
supply season-specific worlds and rules without teaching the core or UI about a season.

```text
Desktop app                            Robot process (unmodified WPILib project)
  UI -> queued commands                  robot code -> HAL / vendor simulation
          |                                           |
  SessionRunner <------- local IPC -------> HALSim extension
          |                                           |
  SimulationSession                         built-in device / vision bridges
    mechanisms -> drivetrain -> physics
    game plugin -> sensors -> snapshot
```

The arrows show data flow, not permission to link modules. A motor controller bridge translates
robot-facing signals; a mechanism or drivetrain models the physical load. Their shared SDK types
do not make them the same responsibility.

## Terms used here

| Term | Meaning |
|---|---|
| HAL | WPILib's hardware abstraction layer, used by robot code instead of real hardware during simulation |
| HALSim | Simulation APIs for observing and writing the robot process's HAL state |
| SimDevice | A named tree of simulated device values exposed by WPILib or a vendor library |
| IPC | Inter-process communication: the messages exchanged between the app and robot process |
| SDK | The shared C++ interfaces and value types used by emulator modules |

## Modules and dependencies

This is the dependency rule for first-party production code. An owner may use its own implementation
and the dependencies listed below; tests may combine modules to exercise their contracts.

| Owner | Allowed dependencies | Responsibility |
|---|---|---|
| `sdk` | Standard library only | Header-only interfaces, models, registries, protocol types and testing fakes |
| `core` | SDK; Box3D privately | Game-agnostic runtime/config, mechanisms/plants and physics adapter |
| `ipc` | SDK | Local transport and message codec |
| `bridges/hal` | SDK; HAL headers in its binding | WPILib device families |
| `bridges/vendor` | SDK; backend headers inside the module | Vendor devices exposed through SimDevices |
| `bridges/vision` | SDK; ntcore headers in its binding | Camera-coprocessor encoders and publishing |
| Bridge aggregator (`bridges/`) | SDK and bridge modules | Built-in bridge registration |
| `halsim_ext` | SDK, IPC, bridge aggregator; WPILib HAL | Robot-process extension |
| Each `plugins/<name>` | SDK only | One game's field, entities, rules and metadata |
| Plugin aggregator (`plugins/`) | SDK and game plugins | Built-in plugin registration |
| `app` | SDK, core, IPC, plugin aggregator, Qt | Desktop composition, host, editor and run view |
| `app/contract`, `app/ui`, `app/editor`, `app/runview` | SDK and Qt only (run view also the host contract; both page libraries also `app/ui`) | Host seam, shared Qt support (3D scene, text, small widgets) and the page libraries |

There is no Qt in SDK, core, IPC, bridges or plugins. Box3D and robot-process backend headers never
leak into the SDK. The physics adapter is compiled into `wpiemu_core`, with Box3D linked privately;
public physics contracts remain SDK interfaces.

The only built-in aggregation seams are:

- [plugins/builtin_plugins.cpp](../plugins/builtin_plugins.cpp), target `wpiemu_plugins_builtin`:
  the app uses `registerBuiltinGamePlugins`, never includes or links an individual season.
- [bridges/builtin_bridges.cpp](../bridges/builtin_bridges.cpp), target `wpiemu_bridges_builtin`:
  the extension uses `registerBuiltinBridges` and `registerBuiltinVisionBridges`, never an individual
  bridge module. Bridges do not depend on one another.

[The boundary checker](../tools/check_boundaries.py) checks includes, CMake links/target ownership and that each
plugin and bridge directory is added, linked and registered through its aggregator and each test source is built.
Dependency changes need an explicit contract, not an exception that bypasses an aggregator.
Header conventions belong in [Contributing](CONTRIBUTING.md#coding-conventions); rationale lives in the
[decision index](decisions/README.md).

## Processes and threads

The app's composition root, `AppServices`, supplies one `IRuntimeHost` to both editor and run view.
The real host delegates Qt-free process/session orchestration to `RobotSession`; the demo host
needs no robot process. See the [app contract](reference/app.md) for their lifecycle.

The Gradle launcher injects `halsim_wpiemu` through WPILib's extension mechanism rather than editing
the robot project. Device detection and simulation use one long-lived robot instance: Armed mode
allows discovery with free-running robot time, lock-step runs the simulation, and ending a run
returns to Armed mode. A robot-code restart is a process restart, not a simulation reset.

Ownership is deliberately narrow:

- **UI thread:** reads immutable `ISimSnapshot` objects and sends commands through
  `ISimController`; it never reaches into mutable simulation objects.
- **Sim thread (live session):** owns physics, mechanisms, device hub, game state, configuration and RNG.
  `SimulationSession` is synchronous, deterministic, and creates no threads and does no I/O.
  `SessionRunner` wraps it with command queues, lock-step coordination, pacing and snapshot delivery.
- **Physics workers:** with more than one worker the Box3D world owns helper threads for its lifetime. They work
  only while the sim thread is inside `step()`, never touch other state, and are joined when the world is destroyed
  after the runner stops. The count is a per-machine launch choice; see [physics](reference/physics.md#determinism-and-limits).
- **Robot threads:** HAL callbacks record events only; they must not block or do socket I/O.
- **Extension clock thread:** applies feedback, advances HAL timing and collects outputs.
  During lock-step, bridge operations happen between robot steps while the robot is quiescent.
- **Transport workers:** move messages between processes and queues; they do not mutate the world.
  The extension also has an independent watchdog thread that can recover without a bridge lock.
  Hand-off queues and joinable workers are the SDK's [BlockingQueue](../sdk/include/wpiemu/sdk/core/blocking_queue.h)
  and [StoppableThread](../sdk/include/wpiemu/sdk/core/stoppable_thread.h); overflow and drop policy stays with the owner.

The [runtime contract](reference/runtime.md) describes construction and configuration ownership;
the [extension/IPC contract](reference/extension-ipc.md) describes robot startup and worker lifetimes.

## One simulation tick

The emulator is clock master. Its fixed step defaults to 5 ms and must divide the robot's 20 ms
period. Wall-clock pacing does not define simulation time.

For tick N:

1. The runner applies queued commands and ingests the accepted actuator outputs from tick N-1.
2. Mechanisms read mapped slots, advance their plants and stage sensor feedback. Motion-linked
   axes are evaluated in dependency order; mechanisms submit chassis coupling before physics.
3. The drivetrain computes wheel forces. Runtime composes chassis/payload mass and applies coupling
   wrenches and constraints.
4. The physics world advances once.
5. Game state consumes the resulting contacts/sensor events; the module's `onTick` hook follows.
   Game-fed sensors then translate the game's published values into device feedback.
6. Runtime updates IMU/battery feedback and advances its clock. Due camera results join the outgoing
   `SensorFrame`; immutable snapshots are available for publication to the UI.
7. The extension applies sensors and due vision results, calls `HALSIM_StepTiming`, then returns an
   `ActuatorFrame` acknowledging N with the robot's resulting outputs.

Only **one tick may be outstanding**. N+1 is not sent until N is acknowledged or a reset/reconnect
opens a new epoch. The [mechanism](reference/mechanisms.md), [physics](reference/physics.md) and
[game-plugin](reference/game-plugin.md) contracts define the subsystem details.

## Reset, reconnect and failure

`sessionId` identifies an accepted connection; `runEpoch` distinguishes runs within it. Both travel
with frames and controls. Reject foreign sessions and stale epochs; tick numbers restart with an
epoch and cannot be treated as globally increasing.

Reset and reconnect open a new epoch. Command IDs and acknowledgements prevent retried controls
from being applied twice. Bridge input history and queued vision results are cleared at lifecycle
boundaries without erasing robot-owned firmware offsets. A reset does not reconstruct the robot
program or reset its odometry for it.

The negotiated watchdog bounds a stalled tick. On timeout or disconnect the extension must itself
resume HAL time, or terminate according to policy, without waiting for the emulator. The runner
reports the stall and stops issuing ticks. Preserve this independent recovery path when changing
logging, queues or shutdown. Wire fields, acknowledgements and pause/recovery semantics belong in
the [extension/IPC contract](reference/extension-ipc.md).

## Extension points

Use the [extension decision guide](extending.md) and its linked walkthroughs, not special cases
in unrelated modules:

- [Season plugins](extending.md#add-a-season-plugin), [zones](extending.md#add-a-zone-kind) and
  [game-fed sensors](extending.md#add-a-game-fed-sensor) contribute game content through SDK metadata.
- [Device bridges](extending.md#add-a-device-bridge), [vendor rules](extending.md#add-a-vendor-rule-set)
  and [vision vendors](extending.md#add-a-vision-vendor) adapt robot-process APIs.
- [Mechanism templates](extending.md#add-a-mechanism-template) and
  [editor properties](extending.md#add-an-editor-property) use slots and schemas instead of bespoke UI.
- [Other SDK strategies](extending.md#other-sdk-strategies) cover drivetrains, tire models, plants,
  motor models, physics backends and transport/codecs.

Game plugins are statically built with the application. The C++ SDK is a source contract, not a
stable ABI for independently built shared libraries.

## Vocabulary

- **HALSim:** WPILib's simulation HAL and extension hooks, operating inside the robot process.
- **SimDevice:** a named HAL simulation device with typed values, often exposed by a vendor wrapper.
- **Slot:** a mechanism/runtime endpoint mapped to a discovered device by compatible capabilities.
- **Plant:** a mechanism-local dynamical model turning commands and loads into motion.
- **Bridge:** a robot-process adapter that discovers devices or presents a simulated coprocessor.
- **Game plugin:** the owner of a season's field and game state; its module is stateless across sessions.
- **Zone:** a game-owned volume or muzzle attached to a chassis/mechanism link, not a motorized part.
- **Game value:** a plugin-published number that generic sensors can consume, distinct from HUD text.
- **Attachment link:** a named spatial frame. A **motion link** instead prescribes one axis from another.
- **Lock-step:** coordinated simulation/HAL stepping with one acknowledged tick at a time.

Next: [build and test the project](BUILDING.md).
