# Local IPC with one outstanding tick

Read this when changing timing, transport or reset behavior.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
The emulator computes physical state while the robot process computes its response to sensor
feedback. Pairing each input with one completed robot step makes causality and reset boundaries
understandable; free-running or pipelined exchange would permit stale outputs to cross those
boundaries.

## Decision
Use local IPC and a bounded binary protocol, with the emulator as clock master and one
acknowledged simulation tick at a time.

## Trade-offs
Pausing and stepping are straightforward, but throughput is limited by the round trip. Lock-step
coordinates HAL timing; it cannot make arbitrary robot threads or third-party code deterministic.
The extension must retain its own recovery path if the emulator disappears.

Current transport and lifecycle rules are in [Extension and IPC](../reference/extension-ipc.md).
