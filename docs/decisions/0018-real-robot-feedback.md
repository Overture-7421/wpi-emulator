# ADR 0018 - Calibrated robot feedback, explicit start placement and game inventory sensors

- Status: accepted
- Scope: drivetrain azimuth slots, vendor feedback normalization, `RobotConfig::startPose`,
  `SlotType::DistanceSensor`, host and editor

## Context

Running an unmodified swerve robot exposed missing azimuth feedback, firmware magnet-calibration
offsets applied a second time, and a reported current being mistaken for a command. Autonomous
odometry resets also cannot substitute for placing the physical chassis at the path's start.
The robot's CANrange devices need a game-owned source of distance without season logic in core.

## Decision

- Every steered wheel exposes an optional `<wheel>.steerEncoder` PositionSensor slot. Runtime writes
  wheel azimuth and azimuth velocity in rotations and rotations/s. Motor slots still receive
  rotor-side feedback; unmapped external encoders remain optional. The editor preserves sensor
  mappings when motor count changes and removes them when steering is removed.
- Slots carry calibrated model-frame encoder values. The Phoenix vendor bridge inversely applies
  the finite, read-back CANcoder magnet offset and direction before writing raw position/velocity.
  It decodes signed as well as unsigned float bit patterns. Missing/incomplete L2 calibration
  warns and retains raw passthrough, rather than guessing. No SDK firmware-config channel is added.
- Phoenix `torqueCurrent` is reported telemetry, named `torqueCurrentReported`, not the canonical
  torque-current request. Applied `motorVoltage` already resolves firmware control, including FOC.
  Per-mapping Reversed remains the only mechanism/motor-frame conversion (ADR 0014).
- The yaw guard accepts equivalent read-back angles modulo 360 degrees, while outgoing raw yaw
  remains continuous. Encoder/rotor turns remain significant; foreign-writer handling is preserved.
- Optional root `startPose` contains required finite `xM`, `yM`, `headingRad`. Host construction
  uses it, Reset restores it, and editor changes use normal undoable property operations. Absence
  preserves the previous default placement. It neither teleports on robot odometry changes nor
  automatically flips for alliance.
- `DistanceSensor` requires the existing `Capability::Distance`. (The plugin-owned
  `game_2026.fuel_sensor` first introduced here was superseded by ADR 0019.)
- SDK, protocol and configuration versions remain 1 under ADR 0013. No migration or compatibility
  fields are introduced.

## Consequences and limitations

Fused CANcoder steering works without changing robot source or vendor libraries. Explicit
`sensorOffsetRot` still describes an additional model-frame offset, not firmware calibration.
Calibration is an initial config snapshot; subsequent robot-side calibration changes require a
new session. Phoenix closed-loop verification remains paced at 1x wall time.

The fuel sensor approximates inventory, not beam geometry or fuel staging. Since mechanisms tick
before game state, inventory changes reach sensor output one simulation tick later. A real range
or conveyor-flow model can replace this plugin mechanism without changing core.

Regression coverage includes slot/runtime feedback and editor mapping lifecycle, signed config
decoding and inverse calibration, voltage-versus-current telemetry, equivalent yaw read-back,
start-pose round-trip/validation/placement, and inventory thresholds/reset/distance routing.
The existing real-project e2e now maps module encoders, checks commanded-direction movement and
bounded pivot/azimuth error, and keeps Phoenix closed loops at real-time pace.
