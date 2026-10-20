# HALSim injection and vendor SimDevice interception

Read this when changing how the emulator observes an unmodified robot.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Robot projects must keep their own WPILib and vendor control code. Requiring robot-side
simulation hooks or replacing controller APIs would undermine that promise.
Sensor access and timing control must reach the same HAL instance the robot uses.

## Decision
Inject the HALSim extension through the launcher, not through robot source edits.
A launch-scoped Gradle init script selects the extension because GradleRIO can replace
an inherited extension list; it must not alter the user's global Gradle configuration.

Use HAL simulation APIs for native devices and table-driven SimDevice bindings for
supported vendor devices. Phoenix firmware simulation still interprets control requests:
the emulator supplies physical feedback and reads resolved actuation rather than
reimplementing firmware or assuming Phoenix traffic passes through HAL CAN callbacks.

Keep blocking timing and IPC work off HAL callbacks so device construction cannot
deadlock waiting for simulation progress.
Gradle-run and external-descriptor launch paths serve different needs: direct launch
gives process ownership but cannot reproduce every custom simulation-task setting.

## Consequences
Java and C++ robot source stays unmodified, but support is bounded by the observable
simulation backend. The extension shares the robot's address space: native-library
mismatches or callback mistakes can crash or hang the robot.
Binding a private second HAL/vendor runtime would observe the wrong state.
Robot-owned simulation can compete for sensor writes. Declared CAN bus labels cannot
create physical identities that a vendor simulator cannot distinguish.
Normalization and alias safety belong to the [bridge contract](../reference/bridges.md).

Next: [Extension and IPC reference](../reference/extension-ipc.md).
