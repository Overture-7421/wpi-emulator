# Glossary

Short definitions for terms used across the handbook. Follow a link for the project-specific
explanation or contract.

- **Actuator frame** — Robot outputs for a simulation tick; see
  [extension and IPC](reference/extension-ipc.md).
- **Box3D** — The rigid-body physics library behind the core adapter; see
  [simulation and physics](concepts/simulation-and-physics.md).
- **Bridge** — An adapter between robot APIs and emulator models; see
  [bridges](reference/bridges.md).
- **Camera frame** — One simulated camera result; see
  [NetworkTables and cameras](concepts/networktables-and-cameras.md).
- **Camera view** — The Run viewport's 3D viewpoint, distinct from simulated robot cameras and their
  [vision results](concepts/networktables-and-cameras.md).
- **Coprocessor** — An external vision computer, such as Limelight or PhotonVision; see
  [camera concepts](concepts/networktables-and-cameras.md).
- **Driver Station (DS)** — Supplies robot mode and joystick/match inputs; see
  [WPILib simulation](concepts/wpilib-simulation.md).
- **Fixed step** — A simulation update of a configured duration; see
  [simulation and physics](concepts/simulation-and-physics.md).
- **HAL** — WPILib's hardware abstraction layer; see
  [WPILib simulation](concepts/wpilib-simulation.md).
- **HAL ABI** — One generation of WPILib's HAL interface (headers, types, units, device families); a
  season names its ABI and the extension selects per-ABI sources; see [season packs](reference/seasons.md).
- **HALSim** — WPILib's simulation APIs for HAL; see
  [WPILib simulation](concepts/wpilib-simulation.md).
- **HALSim extension** — A library that adds simulator behavior inside the robot process; see
  [WPILib simulation](concepts/wpilib-simulation.md).
- **Launch profile** — A named way a GradleRIO generation launches a robot (Gradle tasks and descriptor);
  see [season packs](reference/seasons.md).
- **NetworkTables (ntcore)** — WPILib's typed, named-value messaging system; see
  [camera concepts](concepts/networktables-and-cameras.md).
- **NetworkTables topic** — A named, typed value that publishers update and subscribers read; see
  [camera concepts](concepts/networktables-and-cameras.md).
- **NWU** — North-West-Up, this project's right-handed Z-up world frame; see
  [simulation and physics](concepts/simulation-and-physics.md).
- **Plant** — A model of mechanism motion under commands and loads; see
  [mechanisms](reference/mechanisms.md).
- **Publisher / subscriber** — A topic writer and reader; see
  [camera concepts](concepts/networktables-and-cameras.md).
- **Qt** — The C++ framework used for the desktop interface; see
  [Qt in this project](concepts/qt-in-this-project.md).
- **Qt Quick / QML** — Qt's declarative UI technology, used for the 3D views; see
  [Qt in this project](concepts/qt-in-this-project.md).
- **Quaternion** — A four-component representation of 3D rotation; see
  [simulation and physics](concepts/simulation-and-physics.md).
- **Rigid body** — A simulated object with motion, mass, and collision geometry; see
  [simulation and physics](concepts/simulation-and-physics.md).
- **Robot process** — The separate process running robot code and its HALSim extension; see
  [WPILib simulation](concepts/wpilib-simulation.md).
- **Season pack** — `seasons/<year>/season.json` plus an optional game plugin: every WPILib fact the
  emulator needs for one year; see [season packs](reference/seasons.md).
- **Season pack** — `seasons/<year>/season.json` plus an optional game plugin: every WPILib fact the
  emulator needs for one year; see [season packs](reference/seasons.md).
- **SI units** — The International System of Units used for core geometry and physics; see
  [simulation and physics](concepts/simulation-and-physics.md).
- **SimDevice** — A named set of simulated HAL values; see
  [WPILib simulation](concepts/wpilib-simulation.md).
- **Simulation time** — The emulator-controlled clock; see
  [simulation and physics](concepts/simulation-and-physics.md).
