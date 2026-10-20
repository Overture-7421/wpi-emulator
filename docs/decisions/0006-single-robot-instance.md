# One long-lived robot and runtime composition

Read this when changing discovery, simulation lifecycle or runtime ownership.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Separate discovery and simulation launches repeat expensive startup and observe
different device lifetimes. Mixing deterministic simulation with UI, process control
and transport work also makes resets and failures difficult to isolate.

## Decision
Keep one robot process for a project's discovery and simulation. In Armed mode it
announces devices and runs HAL time freely with unmapped-device loopback.
Beginning and ending simulation switch timing mode without reconstructing the robot;
Restart relaunches the process. Discovery does not imply automatic slot binding.

The application composition root supplies registries and runtime collaborators.
A synchronous, Qt-free session owns mutable simulation state; a separate runner adds
queued commands, IPC and pacing, while the launcher owns process supervision.
The UI consumes immutable snapshots rather than sharing mutable simulation objects.

An explicit configuration coordinator performs decode, defaults, validation and
construction. An in-house JSON codec reuses the SDK's insertion-ordered value tree
for stable diffs, avoiding a second tree representation, conversion layer and build
dependency. The [runtime reference](../reference/runtime.md) owns persistence and lifecycle.

## Consequences
Discovery observes the same devices used during a run and repeated runs avoid startup.
Reset restores simulator state, not arbitrary robot-program memory or constructors;
use Restart when fresh robot initialization is needed.
One local robot is supported, not a multi-robot or distributed composition.
NetworkTables and vendor process globals are not namespaced for concurrent robots.
Owning the JSON codec and platform process supervision incurs maintenance work.

Next: [Application reference](../reference/app.md).
