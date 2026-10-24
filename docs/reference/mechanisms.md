# Mechanisms

Read this when modeling an actuator or connecting its motion to sensors and geometry.

## Overview

A mechanism turns robot commands into modeled motion and reports that motion through
device slots and named values. It owns its local plant state; device bridges own
robot-facing signals. See [simulation and physics concepts](../concepts/simulation-and-physics.md)
for the shared frames and units, and the [mechanism guide](../guides/mechanisms.md)
for implementation steps.

Mechanism coordinates use +X forward, +Y left and +Z up. Store geometry in SI units;
angular positions use radians, linear positions use meters, and sensor rotations use
rotations.

## Templates, properties and slots

[MechanismTemplateInfo](../../sdk/include/wpiemu/sdk/mechanisms/mechanism.h) describes
an editor palette entry: property schema, device slots, default geometry, attachment
links and optional motion axes. Property keys are persisted API. Labels, groups,
sections, unit hints and visibility affect presentation, not stored representation.
The schema supplies defaults and type/range/enum validation. Unknown keys warn; they
are not silently removed.

Core registers one generic template, `mechanism`. Its `motion` property selects one
degree of freedom: `rotary`, `arm` or `linear`. Rotary and linear use the selected
`axis`; arms always pivot about Y, with zero horizontal and positive position raising
the +X end. A linear stage simulates gravity only on Z. Its moving attachment link is
`mover`. Published state contains `position`, `velocity`, `voltage`, `current` and
limit flags. `active` is true only when speed exceeds 0.01 rad/s or m/s.

Defaults are rotary about Z, one `kraken_x60`, 10:1 motor-to-output gearing and brake
neutral. Equal minimum and maximum positions disable stops; otherwise the start
position must be within them. An arm with zero configured inertia derives
uniform-rod inertia: `I = (1/3) * mass * length²`. The
[axis metadata](../../core/mechanisms/src/axis_support.cpp) owns full defaults and ranges.

`IMechanism` owns plant state, reads commands and writes feedback through slots, and
publishes display/game state as `MechanismState`. Normal mechanism ticks follow
configuration order. Core applies property defaults and validation before creating a
mechanism. An invalid motor key, nonphysical plant parameter or inconsistent stops
logs a reason and fails creation. Plugins use the same SDK contract but own their
namespaced template keys. See [Extending](../extending.md) to choose a seam.

Slots name roles as `{instance id, slot name}`, not HAL channels. A driven generic
mechanism requires `motor`; `motor2` through `motor8` are optional. `motorCount`
activates the first one to eight motor slots. `SlotSpec::countKey` and `ordinal`
describe that relationship to validation and editor code. Optional `encoder` and
`velocitySensor` slots receive output-side sensor truth. `reverseLimit` and
`forwardLimit` derive from hard stops, with configurable margin and pressed polarity.
`reportMotorLimits` also writes limit feedback to the motor controller. Feedback is
ideal and written every tick, not integrated from deltas. Game-fed distance and
beam-break sensors are separate [runtime sensors](runtime.md#device-feedback-and-game-fed-sensors).

## Plants, units and current

[IPlant](../../sdk/include/wpiemu/sdk/dynamics/plant.h) advances mechanism-output
position and velocity: rad/rad/s for angular axes, m/m/s for linear axes.
`PlantInput::externalLoad` is a signed output-side force or torque; the generic
template supplies zero. Plants do not own device addressing, supply saturation or
neutral-mode policy.

Built-in plant models are `dc_motor`, `turret`, `flywheel`, `arm` and `elevator`.
The motor catalog indexes the SDK's single [motor table](../../sdk/include/wpiemu/sdk/dynamics/motor_model.h),
whose rows are WPILib `DCMotor` models; it stores one-motor specifications, and the
plant's motor count aggregates identical motors. `IPlant::motorCurrentA()` reports
signed stator current. Runtime battery accounting uses
`IMechanism::batteryCurrentA()` instead. There is no motor thermal model.

Hard stops clamp position and zero velocity without bounce. Flywheel plants ignore
stops but still integrate position for encoders. Arm/elevator gravity uses 9.8 m/s²
and is independent of rigid-body world gravity.

## Motor commands and multiple controllers

Each motor has its own controller output. A torque-current request resolves to
`V = I * R + omega / Kv` with the motor the plant simulates; otherwise the drive uses
applied voltage, or duty cycle times battery voltage when voltage is absent.
Non-neutral commands are limited to battery voltage and then stator current. An
invalid or zero command is neutral: brake supplies 0 V, while coast supplies back-EMF
voltage for zero current. Which controllers regulate current, and when a reported
current is a request rather than telemetry, is decided by vendor signal normalization
in [Bridges](bridges.md).

Unmapped additional motor slots mirror the primary motor. Mapped motors with valid
outputs contribute their own voltage. The plant receives the mean voltage across
motors; the mechanism sums absolute stator current and each motor's positive
electrical power. For a usable supply:

```text
Ibattery = sum(max(0, Vi * Ii)) / Vbattery
```

At rest, equal opposing voltages cancel torque but both motors still draw current.
Wheel drives also retain each controller's voltage; see [Physics](physics.md).

`SlotMapping::reversed` means physical mounting opposes the modeled positive axis; it
is not another controller inversion. Robot output already includes firmware
inversion. **Only DeviceHub applies mapping reversal:** it negates duty, voltage and
requested torque current when reading commands, and rotor position, velocity and
acceleration when writing feedback. Mechanisms and drivetrains must not invert again.
Limit bits are not swapped. Mirrored, unmapped motors have no independent reversal
flag. A mapping beyond `motorCount` warns as inactive; reversal on a non-motor slot
also warns.

## Feedback units

For angular output position `q`, `qRot = q / (2*pi)`. For linear travel,
`qRot = q / (2*pi*drumRadiusM)`. Rotor feedback is `gearRatio * qRot`, before any
controller-side sensor gearing; the same conversion applies to velocity and
acceleration.

Output-sensor position is
`sign * sensorRatio * qRot + sensorOffsetRot`, where `sign` is -1 when inverted and
+1 otherwise. Velocity uses the same sign and ratio but no offset. These sensor
properties do not change rotor feedback. Firmware calibration belongs to a bridge;
drivetrain azimuth feedback is documented in
[Runtime](runtime.md#device-feedback-and-game-fed-sensors).

## Geometry attachments

An attachment places a mechanism root on its parent's named link:
`childRoot = parentLinkInChassis * child.mount`. A mechanism's `root` is implicit,
stable and identity; templates must not redeclare it. Other declared links publish
their live poses. The chassis offers only `root`.

The [attachment contract](../../sdk/include/wpiemu/sdk/mechanisms/attachment.h)
rejects missing parents or links, cycles and the reserved mechanism ID `chassis`.
Runtime resolution is bounded; a broken chain warns and falls back to chassis root
instead of recursing indefinitely. List parents before children for a consistent
same-pass view of moving attachments. Geometry attachments do not transmit torque,
create rigid-body joints or connect plants.

## One-way motion links

`motionLinks` connect advertised output axes: the generic mechanism exposes `axis`,
and a drivetrain wheel exposes `<wheel>.drive`. JSON endpoints use `owner` and
`axis`. The signed ratio is target SI units per source SI unit, including rad-to-m
conversion:

```text
target = targetStart + ratio * (source - sourceStart)
targetVelocity = ratio * sourceVelocity
```

Each follower has one source; chains and fan-out are allowed. Missing or unsupported
endpoints, zero or non-finite ratios, duplicate targets, cycles, and independently
mapped follower motors are errors (`motion_link_invalid`). Followers skip their own
motor/plant advance and motor draw, but still update limits, sensors, geometry and
game-facing state. They clamp to their own stops without stopping or loading the
source.

The session advances advertised axes once in stable dependency order, then finalizes
mechanism state. Implementations that advertise axes must provide state/advance
callbacks and override `finishMotionTick` without advancing those axes again.
Unlinked models keep their ordinary tick path. This is visible shared motion, not a
power-conserving transmission: it has no reflected inertia/load, compliance or
bidirectional shaft solver. Use [physics coupling](physics.md#chassis-coupling) for
chassis forces, payloads and field anchors.
