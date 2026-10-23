# HALSim injection and vendor SimDevice interception

Read this when changing how the emulator observes an unmodified robot project.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Requiring robot-source hooks or replacing controller APIs would make the emulator a fork of the
team's project. The extension must observe the same HAL state and timing as robot code, while
vendor simulation may interpret control requests using firmware behavior the emulator should not
duplicate.

## Decision
Inject a HALSim extension at launch, use HAL simulation APIs for native devices, and adapt supported
vendor SimDevices at the robot-process boundary.

## Trade-offs
The extension shares the robot process and its native-library constraints. Support therefore
depends on what the loaded HAL and vendor simulation expose; it is not a promise to emulate every
controller API.

Current integration and adapter contracts live in [Extension and IPC](../reference/extension-ipc.md)
and [Bridges](../reference/bridges.md).
