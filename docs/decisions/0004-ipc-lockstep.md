# Local IPC with one outstanding tick

Read this when changing timing, transport or reset behavior.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
The emulator and robot are separate processes, but must agree when a step has finished.
Free-running exchanges or pipelining make the relationship between actuator outputs,
sensor inputs and resets harder to reason about.

## Decision
Use per-session local named pipes on Windows and Unix domain sockets elsewhere,
with endpoint access restricted locally. These avoid network-port allocation and
another service; shared memory adds unnecessary synchronization complexity.
Use a bounded binary codec to avoid a heavy runtime dependency inside the robot.

The emulator is clock master. It advances simulation from prior actuator outputs,
sends sensor inputs, then waits for the extension to apply them, advance HAL time
and return actuator outputs. Permit exactly one outstanding tick, with no pipelining.

Session and run-epoch identity separate resets and reconnects from stale traffic.
Control acknowledgements and duplicate suppression keep retries from applying a
reset twice. The [protocol reference](../reference/extension-ipc.md) owns those rules.

The extension independently resumes HAL timing or terminates according to the
watchdog policy on timeout or disconnect. A dead emulator cannot be responsible
for unfreezing the robot it left paused.

## Consequences
Tick causality is explicit and simulation can pause or step without racing the robot.
Throughput is limited by the slowest tick and its round trip; this is not a distributed
simulation or a throughput-oriented pipeline.
The handwritten wire format and its bounds checks are ours to maintain.
Lock-step controls HAL time, not every thread or third-party library's wall clock,
so it does not make arbitrary robot code deterministic.

Next: [Runtime reference](../reference/runtime.md).
