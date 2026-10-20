# Explicit start placement and calibrated feedback

Read this when changing chassis placement or robot-visible sensor state.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
A robot odometry reset changes its estimate, not the physical chassis location.
Closed-loop control also fails if a calibrated model value is written as raw sensor
state and firmware applies its calibration a second time.

## Decision
Keep physical start placement under emulator control: a project default or a selected
named field-frame pose is restored on Reset. Selecting a placement alone does not
teleport the chassis. Explicit alliance copies use the plugin's field symmetry;
robot pose estimates and alliance changes do not implicitly reposition the world.

Publish physical feedback in the appropriate model frame: motor slots receive rotor
motion, and external encoders receive output or steering azimuth motion.
The vendor bridge inversely applies observed CANcoder calibration before raw writes,
so firmware reports calibrated model truth rather than a double-applied offset.
Apply per-motor mounting reversal once at the mapping boundary, not again in plants.

Keep reported Phoenix torque current as telemetry. Its applied motor voltage already
reflects firmware control and drives the plant; a measured current is not a request.
The [bridge reference](../reference/bridges.md) defines normalization and calibration.

## Consequences
Unmodified robot code remains responsible for its odometry and firmware zeroing.
Choosing a physical start does not secretly seed the robot's pose estimator.
Correct feedback still depends on configured ratios, mounts and sensor mappings.
Missing calibration warns and retains raw passthrough rather than guessing.
Calibration read-back is a session snapshot; later robot-side calibration changes
require a new session. Accurate model-frame feedback is not a guarantee of real
sensor geometry, noise or hardware fidelity.
Placement selection and reset semantics belong in the
[runtime reference](../reference/runtime.md).

Next: [User guide](../USER_GUIDE.md).
