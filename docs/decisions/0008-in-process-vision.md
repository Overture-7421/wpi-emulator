# In-process vision coprocessors

Read this when changing simulated cameras or robot-visible vision output.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Robot programs consume coprocessor results, not rendered camera images. Publishing from the
robot process reuses its NetworkTables time base and loaded ntcore instead of introducing a
second client stack and clock to reconcile across pause and reset.

## Decision
Compute visibility, capture timing and noise in the emulator, then publish supported vendor
results through bridges in the robot process.

## Trade-offs
This exercises robot estimator integration without requiring source hooks, but does not model
image pipelines, occlusion or photorealistic perception. Results also depend on configured camera
mounts and available ntcore.

Current camera and bridge behavior is in [Bridges](../reference/bridges.md).
