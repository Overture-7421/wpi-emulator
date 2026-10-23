# One long-lived robot and runtime composition

Read this when changing discovery, simulation lifecycle or runtime ownership.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Launching a second robot process for simulation repeats startup and can observe a different device
lifetime. Mixing process control, transport, pacing and mutable simulation state also makes reset
and failure handling harder to isolate.

## Decision
Keep discovery and simulation in one robot process. Separate synchronous simulation state from
the runner that coordinates queues and pacing, and keep process supervision at the launcher edge.
The UI reads snapshots rather than mutable simulation objects.

## Trade-offs
Reset is not equivalent to restarting robot code, and one local robot is not a multi-robot
composition. The split adds interfaces and coordination, but keeps simulation testable without
process or UI dependencies.

Current lifecycle and ownership rules are in [Runtime](../reference/runtime.md) and
[Application](../reference/app.md).
