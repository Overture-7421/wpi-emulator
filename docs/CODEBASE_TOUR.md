# Codebase tour

Read this when you want to follow a feature through the repository rather than read a subsystem
contract first.

The [glossary](GLOSSARY.md) defines shared terms. For background on the technologies, see
[WPILib simulation](concepts/wpilib-simulation.md) and
[NetworkTables and cameras](concepts/networktables-and-cameras.md),
[Qt in this project](concepts/qt-in-this-project.md), and
[simulation and physics](concepts/simulation-and-physics.md).

## Repository map

| Path | What lives there |
|---|---|
| `sdk/` | Header-only interfaces and shared data types used across module boundaries |
| `core/runtime/` | Configuration, validation, launcher, device hub, session, runner, and sensors |
| `core/mechanisms/` | Mechanism templates, motor drives, and physical plants |
| `core/physics/` | Drivetrains, tire models, chassis, and the Box3D adapter |
| `ipc/` | Local transport and binary message codec |
| `halsim_ext/` | Extension loaded into the robot process; HAL timing, message handling, and bridge lifecycle |
| `bridges/` | HAL, vendor SimDevice, and camera adapters; `builtin_bridges.cpp` aggregates them |
| `plugins/` | Season modules; `builtin_plugins.cpp` aggregates built-in plugins |
| `app/` | Qt composition root, shell, setup wizard, editor, run view, and process host |
| `tests/` | Shared SDK, runtime, mechanism, bridge, plugin, and editor tests |
| `halsim_ext/tests/` | Extension lifecycle, lock-step, bridge, and reconnect tests |
| `samples/` | Unmodified-style WPILib robot projects used for examples and integration checks |
| `cmake/`, `tools/` | Build configuration, repository checks, and test helpers |
| `docs/` | User guide, concepts, architecture, tour, extension guides, and contracts |

The root [`CMakeLists.txt`](../CMakeLists.txt) adds the SDK, core, IPC, extension, bridges, plugins,
app, and tests. [`tests/CMakeLists.txt`](../tests/CMakeLists.txt) builds the main test executable
from test sources, including `halsim_ext/tests/test_*.cpp`. When real WPILib HAL artifacts are
available, [`halsim_ext/CMakeLists.txt`](../halsim_ext/CMakeLists.txt) also defines a separate
HAL integration target. See [Building](BUILDING.md) for focused test commands.

## Open a project and reach a running robot

1. [`app/main.cpp`](../app/main.cpp) creates `AppServices`, the shell controller, pages, and main
   window. After showing the window, startup opens `--project` or restores the last project.
2. [`MainWindow::openProjectPath`](../app/shell/main_window.cpp) calls
   [`ShellController::openProject`](../app/shell/shell_controller.cpp).
   [`ProjectSession::open`](../app/common/project_session.cpp) resolves the project root and asks
   the Gradle project inspector to examine it.
3. If `<project>/.wpiemu/robot.json` exists,
   [`ConfigRepository`](../app/common/config_repository.cpp) loads it through the core
   configuration pipeline. That pipeline supplies defaults and validates the model. If setup is
   missing, the shell opens the wizard instead.
4. The shell binds the model and selected game module to the runtime. The real host is
   [`RealRuntimeHost`](../app/common/real_runtime_host.cpp). Opening a configured project launches
   one robot in **Armed** mode for startup and device discovery; it does not yet advance simulation.
5. [`RobotSession::start`](../app/host/robot_session.cpp) builds the simulation, starts the local
   IPC listener and `SessionRunner`, then launches the robot with `RobotLauncher`. The default
   Gradle launch injects the HALSim extension without editing the robot project. The extension
   connects, handshakes, and reports program readiness. Choosing Run begins lock-step on that same
   robot process.

Follow the shell path in [`app/tests/test_gui_shell.cpp`](../app/tests/test_gui_shell.cpp), runtime
composition in [`app/tests/test_real_host.cpp`](../app/tests/test_real_host.cpp), and opt-in robot
launch in [`app/host/tests/e2e_test.cpp`](../app/host/tests/e2e_test.cpp).

## Follow one simulation tick

`SessionRunner::poll` applies queued commands and inbound messages. It ticks only after the robot
is ready and the prior tick has been acknowledged. `shouldTick` also enforces one outstanding tick.

`SessionRunner::doTick` calls `SimulationSession::tick` before sending the resulting `SensorFrame`.
Inside that synchronous call, mechanisms update first, then drivetrain and chassis coupling;
physics advances once; game state and the plugin hook run; game-fed sensors, IMU, and battery
feedback update; then the simulation clock advances and due camera results are added.

In the robot process, [`SimSession::onSensorFrame`](../halsim_ext/src/sim_session.cpp) checks
session and epoch identity, applies device and vision inputs, and steps HAL timing. Robot code
runs during that HAL step. The extension then polls bridges, collects outputs, and returns an
`ActuatorFrame`. The runner accepts the matching acknowledgment and applies its outputs to the
device hub before another tick can start.

The exact order is in [`SimulationSession::tick`](../core/runtime/src/runtime/simulation_session.cpp)
and [`SessionRunner`](../core/runtime/src/runtime/session_runner.cpp). Focused evidence lives in
[`tests/runtime/test_session.cpp`](../tests/runtime/test_session.cpp),
[`tests/runtime/test_session_runner.cpp`](../tests/runtime/test_session_runner.cpp), and
[`halsim_ext/tests/test_lockstep.cpp`](../halsim_ext/tests/test_lockstep.cpp).
See [Architecture](ARCHITECTURE.md#one-simulation-tick) for ownership and the full system-level sequence.

## Follow an editor click to `robot.json`

For example, change a property in the editor's inspector:

1. [`PropertyFormWidget`](../app/editor/views/property_form.cpp) emits `valueEdited`.
   [`InspectorPanel`](../app/editor/views/inspector_panel.cpp) sends it to
   `EditorDocument::setProperty`.
2. [`EditorDocument::edit`](../app/editor/model/editor_document.cpp) makes an undoable config
   command. The document's change notification reaches `EditorController`.
3. [`EditorController::onDocumentModified`](../app/editor/viewmodel/editor_controller.cpp) submits
   the new model to the backend and schedules autosave. Explicit Save uses the same persistence
   path.
4. [`EditorBackendAdapter`](../app/shell/editor_backend_adapter.cpp) holds the pending model.
   Saving commits it through [`ProjectSession`](../app/common/project_session.cpp) and
   [`ConfigRepository`](../app/common/config_repository.cpp).
5. Core's project config repository writes `<project>/.wpiemu/robot.json`. The runtime uses the
   submitted model immediately; saving makes that model persistent for the next open.

The undoable edits are tested in
[`tests/app_editor/test_document_controller.cc`](../tests/app_editor/test_document_controller.cc)
and [`tests/app_editor/test_config_ops.cc`](../tests/app_editor/test_config_ops.cc).
The app-to-repository save path is covered by
[`app/tests/test_config_session.cpp`](../app/tests/test_config_session.cpp) and
[`tests/runtime/test_config_store.cpp`](../tests/runtime/test_config_store.cpp).
For disk format and failure behavior, see [Runtime and configuration](reference/runtime.md).

## Follow a device command to a mechanism

Assume the editor maps a discovered motor controller to a mechanism's motor slot. Robot code
commands its ordinary WPILib controller; the robot project needs no emulator-specific call.

1. A robot-process bridge reads the controller's output signals. The extension packages those
   samples in an `ActuatorFrame`.
2. `SessionRunner` accepts that frame and calls `DeviceHub::onActuatorFrame`. The hub resolves
   configured device-to-slot mappings; an unmapped output is not silently assigned to a mechanism.
3. During the next `SimulationSession::tick`, the mechanism asks the hub for its motor output.
   [`MechanismAxis::tick`](../core/mechanisms/src/axis_support.cpp) converts it to motor voltage
   and steps the configured plant.
4. The mechanism writes motor and encoder feedback back through the hub. The session includes
   that feedback in the next `SensorFrame`; the extension bridge writes it into the robot's HAL
   simulation state.

The main code paths are [`DeviceHub`](../core/runtime/src/runtime/device_hub.cpp),
[`SimulationSession`](../core/runtime/src/runtime/simulation_session.cpp), and
[`SimSession`](../halsim_ext/src/sim_session.cpp). Focused tests include
[`tests/runtime/test_device_hub.cpp`](../tests/runtime/test_device_hub.cpp),
[`tests/mechanisms/test_mechanism_templates.cpp`](../tests/mechanisms/test_mechanism_templates.cpp),
and [`halsim_ext/tests/test_bridge_set.cpp`](../halsim_ext/tests/test_bridge_set.cpp).
The [device mapping contract](reference/mechanisms.md) and [bridge contract](reference/bridges.md)
define mapping compatibility, signals, and units.

Next: [choose an extension seam](extending.md).
