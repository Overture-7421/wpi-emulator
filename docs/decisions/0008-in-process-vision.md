# In-process vision coprocessors

Read this when changing simulated cameras or their robot-visible output.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Robot programs consume coprocessor results, not rendered camera images.
They expect vendor topics, payloads and timestamps in their own time base.
Requiring camera simulation hooks in robot code would violate unmodified-project use.

## Decision
Compute camera visibility, capture timing, latency and seeded noise in the emulator;
the selected game plugin supplies the AprilTag field layout.
Publish Limelight and PhotonVision-style results from the HALSim extension into the
robot's default NetworkTables instance through its already-loaded ntcore.

Use the robot's FPGA time for publication timestamps. An external app-side NT client
would introduce another NT stack and a separate clock to reconcile across pauses.
Do not load a private second ntcore instance or require robot-side camera simulators.

Keep vendor encodings in vision bridges, separate from the visibility model.
Reset queued captures and published results with the simulation so a new run cannot
inherit a delayed result. The [bridge reference](../reference/bridges.md) owns frame,
payload and timing details.

## Consequences
Existing robot camera consumers can run without source edits, but camera names and
mounts must match their configuration; the emulator cannot infer them from robot code.
Packed PhotonVision results require a matching schema; unknown encodings are refused,
and unavailable ntcore leaves camera publication inactive.
The visibility model omits occlusion and image-pipeline effects. Estimated poses use
ground truth plus configured noise, not a photorealistic image solve.
This exercises estimator integration and latency, not real-world perception accuracy
or dashboard video streams.

Next: [User guide](../USER_GUIDE.md).
