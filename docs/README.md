# Documentation

Read this when choosing what to read or where to make a change.

WPI Emulator runs ordinary WPILib robot code in a separate process. The app configures and
simulates the robot; it does not require changes to the robot project.

## Reading path

| If you are… | Read in this order |
|---|---|
| New to the codebase | [WPILib simulation](concepts/wpilib-simulation.md) → [NetworkTables and cameras](concepts/networktables-and-cameras.md) → [simulation and physics](concepts/simulation-and-physics.md) → [Qt in this project](concepts/qt-in-this-project.md) → [architecture](ARCHITECTURE.md) → [codebase tour](CODEBASE_TOUR.md) |
| Building or contributing | [Building](BUILDING.md) → [Contributing](CONTRIBUTING.md) → [first-change tutorial](tutorials/first-change.md) |
| Extending simulator behavior | [Architecture](ARCHITECTURE.md) → [Extending](extending.md) → the owning guide and reference |
| Running a team's robot | [User guide](USER_GUIDE.md) → [robot samples](../samples/swerve_robot/README.md) |
| Diagnosing or maintaining a build | [Maintenance guide](guides/maintenance.md) → the relevant reference |
| Preparing a new WPILib season | [WPILib upgrade guide](guides/wpilib-upgrade.md) → [releasing](releasing.md) |
| Learning to build a season game plugin | [Implementing a Game Season: FIRST POWER UP on the 2027 ABI](book/README.md) |

## Concepts

These pages explain unfamiliar tools and ideas before the architecture and code traces:

- [Glossary](GLOSSARY.md)
- [How WPILib simulation works](concepts/wpilib-simulation.md)
- [NetworkTables and simulated cameras](concepts/networktables-and-cameras.md)
- [Qt in this project](concepts/qt-in-this-project.md)
- [Simulation, physics, and units](concepts/simulation-and-physics.md)

## Project map and contracts

- [Codebase tour](CODEBASE_TOUR.md): repository map and guided traces through real code.
- [Architecture](ARCHITECTURE.md): dependencies, thread ownership, tick order, and recovery rules.
- [Extending](extending.md): the single map from a change to its smallest extension seam.
- [Runtime and configuration](reference/runtime.md)
- [Mechanisms](reference/mechanisms.md)
- [Physics](reference/physics.md)
- [Extension and IPC](reference/extension-ipc.md)
- [Bridges](reference/bridges.md)
- [Game plugins](reference/game-plugin.md)
- [Season packs](reference/seasons.md)
- [Application](reference/app.md)

## Guides and examples

- [Building and testing](BUILDING.md)
- [Contributing](CONTRIBUTING.md)
- [First change](tutorials/first-change.md)
- [Devices and bridges](guides/devices.md)
- [Editor properties](guides/editor-properties.md)
- [Game plugins and interactions](guides/game-plugins.md)
- [Maintenance](guides/maintenance.md)
- [Mechanisms](guides/mechanisms.md)
- [Simulation strategies](guides/simulation-strategies.md)
- [Vision](guides/vision.md)
- [WPILib season upgrade](guides/wpilib-upgrade.md)
- [Season plugin template](../seasons/_template/game/README.md)
- [Robot samples](../samples/swerve_robot/README.md), [vision samples](../samples/vision_robot/README.md),
  and [2027 preview samples](../samples/season_2027/README.md)

## Users, maintainers, and decisions

- [User guide](USER_GUIDE.md)
- [Releasing](releasing.md)
- [Architecture decisions](decisions/README.md)

The guides explain procedures; references define subsystem behavior. Use links rather than
copying a contract into a second page.

Next: [learn how WPILib simulation works](concepts/wpilib-simulation.md).
