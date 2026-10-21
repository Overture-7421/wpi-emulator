# Mechanisms

**Read this when:** modeling an actuator, adding template metadata, or connecting motion
and sensor feedback without coupling a mechanism to a device vendor.
**Prerequisites:** [Runtime](runtime.md) for loading and ownership;
[Contributing](../CONTRIBUTING.md) for units and conventions.

## Template and property contract

[MechanismTemplateInfo](../../sdk/include/wpiemu/sdk/mechanisms/mechanism.h) describes
the palette entry, property schema, device slots, default geometry, attachment links and
optional modeled motion axes. The editor consumes this metadata, not concrete mechanism
classes. Property keys are persisted API; labels, groups, sections, unit hints and
conditional visibility describe presentation, not a different storage representation.
[PropertySchema](../../sdk/include/wpiemu/sdk/config/property_schema.h) supplies defaults
and type/range/enum validation. Unknown property keys warn; they are not silently removed.

Core registers one template, `mechanism`, through `registerGenericMechanisms`.
Its `motion` selects `rotary`, `arm` or `linear`; these are configurations of one degree
of freedom, not separate mechanism types. Rotary/linear use the selected `axis`; arms
always pivot about Y, with zero horizontal and positive position raising the +X end.
Only a Z-axis linear stage enables gravity. The moving attachment link is `mover`.
The template publishes `position`, `velocity`, `voltage`, `current` and limit flags;
`active` means speed exceeds 0.01 rad/s or m/s.

Defaults are rotary about Z, one `kraken_x60`, 10:1 motor/output gearing and brake neutral.
Travel stops are disabled when minimum equals maximum; otherwise start position must
lie within them. An arm with zero configured inertia derives a uniform-rod inertia.
The full property defaults and ranges live in the template's
[axis defaults](../../core/mechanisms/src/axis_support.h) and
[axis metadata](../../core/mechanisms/src/axis_support.cpp), not a second table here.

`IMechanism` owns local plant state, reads commands and writes feedback through slots,
and exposes display/game state through `MechanismState`. Normal ticks follow config
order. The core factory wrapper defaults and checks properties before construction;
invalid motor keys, nonphysical plant parameters or inconsistent stops log a reason
and fail creation. Plugin templates use the same SDK contract; their type keys belong
to the plugin namespace. Extension steps are in [Extending](../extending.md).

## Slots and plant state

Slots address roles as `{instance id, slot name}`, never a HAL channel directly.
A driven generic mechanism requires `motor`; `motor2` through `motor8` are optional.
`motorCount` activates the first one through eight motor slots; `SlotSpec::countKey`
and `ordinal` expose that relationship to validation/editor code.
The optional `encoder` and `velocitySensor` slots receive output-side sensor truth.
`reverseLimit` and `forwardLimit` derive from hard stops, with configurable margin and
pressed polarity. `reportMotorLimits` additionally writes controller limit feedback.
Feedback is ideal and written every tick, rather than integrated from deltas.
Game-fed distance/beam-break sensors are separate [runtime sensors](runtime.md#device-feedback-and-game-fed-sensors).

[IPlant](../../sdk/include/wpiemu/sdk/dynamics/plant.h) advances mechanism-output position
and velocity: rad/rad/s for angular axes, m/m/s for linear axes. `PlantInput::externalLoad`
is an output-side signed force or torque; the generic template passes zero.
Plants do not own device addressing, supply saturation or neutral-mode policy.

The built-ins are `dc_motor`, `turret`, `flywheel`, `arm` and `elevator`. Core owns the
small WPILib-style system equations rather than linking WPILib native math:
see [linear systems](../../core/mechanisms/src/linear_system.cpp) and
[plant integration](../../core/mechanisms/src/plants.cpp). The n-motor gearbox scales a copy of the SDK
`MotorSpec`, so R, Kt and Kv are defined only there.
Linear dynamics use exact matrix-exponential discretization. Nonlinear cases use
rate-bounded RK4 or a stiff exponential-midpoint path; replacing them with one explicit
Euler step is not equivalent. Stops clamp position and zero velocity, with no bounce.
Flywheel plants ignore stops but still integrate position for encoders.
Arm/elevator gravity uses 9.8 m/s² independently of the rigid-body world's gravity.

[MotorCatalog](../../core/mechanisms/src/motor_catalog.cpp) indexes the SDK's one
[motor table](../../sdk/include/wpiemu/sdk/dynamics/motor_model.h) (WPILib `DCMotor` rows, also listed
by the wizard and editor wheel selectors) and stores one-motor specifications;
the plant's motor count aggregates identical motors. `IPlant::motorCurrentA()` is signed
stator current, not supply draw. Runtime battery accounting uses
`IMechanism::batteryCurrentA()` instead. There is no motor thermal model.

## Motor commands and multiple controllers

Each motor keeps its own controller output. For mechanisms,
[MotorDrive](../../core/mechanisms/src/motor_drive.cpp) resolves a torque-current request
as `V = I * R + omega / Kv`, otherwise uses applied voltage (or duty times battery).
Non-neutral commands are saturated to battery voltage and then stator-current limited.
An invalid or zero command is neutral: brake shorts the motor at 0 V; coast supplies
back-EMF voltage for zero current. A reported current is not a new current request;
vendor normalization is described in [Bridges](bridges.md).

Additional active motor slots without valid output mirror the primary motor.
Mapped motors with valid output contribute their own voltage. The mechanism feeds the
mean voltage into its N-motor plant, using the linear DC-motor torque relation, but sums
absolute stator currents and positive electrical power per motor separately:
`Ibattery = sum(max(0, Vi * Ii)) / Vbattery` for a usable supply.
At rest, equal opposing voltages cancel torque while both motors still draw current.
Wheel drives likewise retain per-controller voltages; see [Physics](physics.md).

`SlotMapping::reversed` describes physical mounting against the modeled positive axis,
not a second controller inversion. Robot output already includes firmware inversion.
**Only DeviceHub applies mapping reversal:** it negates duty, voltage and requested
torque current on reads, and rotor position/velocity/acceleration on feedback writes.
Mechanisms and drivetrains must not flip them again. Limit bits are not swapped.
Unmapped mirrored motors have no independent reversal flag. Mappings beyond motor count
warn as inactive; `reversed` on a non-motor slot also warns.

## Feedback units

Let `qRot = q / (2*pi)` for an angular output, or `q / (2*pi*drumRadiusM)` for linear
travel. Rotor feedback is `gearRatio * qRot`, before any controller-side sensor gearing.
The same conversion applies to velocity and acceleration.
The output encoder reports `sign * sensorRatio * qRot + sensorOffsetRot`, with the
offset omitted for velocity. These sensor properties never alter rotor feedback.
See the [sensor models](../../core/mechanisms/include/wpiemu/core/mechanisms/sensor_models.h)
and [feedback writer](../../core/mechanisms/src/axis_support.cpp).
Firmware calibration is a bridge concern; the distinction and drivetrain azimuth
feedback are documented in [Runtime](runtime.md#device-feedback-and-game-fed-sensors).

## Geometry attachments

An attachment places a mechanism root on its parent's named link:
`childRoot = parentLinkInChassis * child.mount`. A mechanism's `root` is implicit,
stable and identity; templates must not redeclare it. Other declared links should be
published at their live poses. The chassis offers only `root`.
The [attachment contract](../../sdk/include/wpiemu/sdk/mechanisms/attachment.h) rejects
missing parents/links, cycles and the reserved mechanism id `chassis`.
Runtime resolution is bounded and returns no pose for a broken chain, with a warning
and chassis-root fallback instead of unbounded recursion.
List parents before children for a consistent same-pass view of moving attachments.
Attaching geometry does not transmit torque, create a rigid-body joint or link plants.

## One-way motion links

`motionLinks` connect advertised output axes: the generic mechanism exposes `axis`,
and a drivetrain wheel exposes `<wheel>.drive`. JSON endpoints use `owner` and `axis`.
The signed ratio is target SI units per source SI unit, including rad-to-m conversions:
`target = targetStart + ratio * (source - sourceStart)`; velocity is ratio-scaled.

Each follower has one source; chains/fan-out are allowed. Missing/unsupported endpoints,
zero/non-finite ratios, duplicate targets, cycles and independently mapped follower
motors are errors (`motion_link_invalid`). Followers skip their own motor/plant advance
and motor draw, but still update limits, sensors, geometry and game-facing state.
They clamp to their own stops without stopping or loading the source.

The session advances advertised axes once in stable dependency order before finalizing
mechanism state. Implementations advertising axes must provide state/advance callbacks
and override `finishMotionTick` without advancing their axes again. Unlinked models keep
their ordinary tick/step path. See [motion data](../../sdk/include/wpiemu/sdk/mechanisms/motion_link.h)
and [validation](../../sdk/include/wpiemu/sdk/mechanisms/motion_config.h).
This models visible shared motion, not a power-conserving transmission: no reflected
inertia/load, compliance or bidirectional shaft solver. For chassis forces, payloads
and field anchors use [physics coupling](physics.md#chassis-coupling), not a motion link.

**Next:** [Physics](physics.md).
