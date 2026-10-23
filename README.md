# WPI Emulator

WPI Emulator runs **unmodified WPILib Java and C++ robot projects** in a desktop simulator.
The app owns the simulated world; a HALSim extension runs inside the robot process.
Game plugins own season-specific fields and rules.

## Run a robot

Install and launch with the [user guide](docs/USER_GUIDE.md), or build from source with
[BUILDING](docs/BUILDING.md). `--demo` runs scripted devices and motion instead of robot code.
See the user guide's [limits](docs/USER_GUIDE.md#troubleshooting-and-limits) before relying on a
particular platform, device, season, or rendering feature.

## Work on the project

The [documentation home](docs/README.md) is the table of contents and reading path.
It links to the [codebase tour](docs/CODEBASE_TOUR.md), [architecture](docs/ARCHITECTURE.md),
and [extension seam map](docs/extending.md). Use [Contributing](docs/CONTRIBUTING.md) for
change conventions and [releasing](docs/releasing.md) for packaging.

Next: [choose a documentation path](docs/README.md).
