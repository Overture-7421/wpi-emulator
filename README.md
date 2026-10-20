# WPI Emulator

WPI Emulator is a desktop simulator for **unmodified WPILib Java and C++ robot projects**.
The app owns physics, mechanisms and the 3D field; a HALSim extension connects the robot process.
Season-specific rules live in game plugins, not in the application or simulation core.

Read this when choosing where to start.
Prerequisites: a WPILib project and its desktop simulation toolchain to run robot code;
see the [user guide](docs/USER_GUIDE.md) or [build prerequisites](docs/BUILDING.md).

## What you can do

- Launch through the project's Gradle wrapper without rewriting source, build files or vendordeps.
- Describe swerve, tank or mecanum robots in the setup wizard and undoable 3D editor.
- Model mechanisms, attach cameras and map detected controllers and sensors to physical parts.
- Run the robot with the built-in Driver Station, joysticks, console and live state inspection.
- Simulate the built-in REBUILT season and Limelight/PhotonVision AprilTag observations.

## Quick start

For an installer or portable package, follow [Install and launch](docs/USER_GUIDE.md#install-and-launch),
then open your project folder and finish the wizard. `--demo` substitutes scripted devices and motion
for robot execution.

To build from source, use [BUILDING.md](docs/BUILDING.md): it owns the prerequisites,
platform presets and configure/build/test commands. Package production is in
[releasing.md](docs/releasing.md). See the user guide's [limits](docs/USER_GUIDE.md#limits-and-platforms)
before relying on a device, season or platform.

## Documentation

Users: [USER_GUIDE](docs/USER_GUIDE.md). Maintainers: [releasing](docs/releasing.md).

Developer reading path: [Architecture](docs/ARCHITECTURE.md) →
[Building](docs/BUILDING.md) → [Contributing](docs/CONTRIBUTING.md) →
[Extending](docs/extending.md) → [reference manuals](docs/reference/runtime.md).
The reference set covers runtime, mechanisms, physics, extension IPC, bridges, game plugins and the app.
Rationale belongs in the [decision index](docs/decisions/README.md).

Next: [Architecture](docs/ARCHITECTURE.md), or [the user guide](docs/USER_GUIDE.md) to run a robot.
