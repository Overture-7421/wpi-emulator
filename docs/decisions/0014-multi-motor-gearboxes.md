# ADR 0014 - Several controllers per gearbox, with a per-motor Reversed flag

- Status: accepted
- Scope: SDK (`SlotMapping`, `SlotSpec`, `WheelCommand`), device hub, mechanisms, drivetrain, editor

## Context

A shooter can have five motors, a tank side three. Robot code creates one controller per motor
(leader plus followers, some inverted or `Opposed`), and the emulator has to simulate what each one
does. Before this change a mechanism axis had one axis-wide `motorInverted` flag, so a mixed set
(two forward, one reversed) could not be expressed, and a drivetrain wheel had one drive slot and no
orientation at all: the usual WPILib pattern of inverting the right side in robot code drove that
side backwards in the simulator.

## Decision

- **Orientation lives on the mapping.** `SlotMapping::reversed` (default false, written to
  `robot.json` only when true, meaningful on Motor slots only) says that the controller spins against
  the mechanism's positive direction. `motorInverted` is removed.
- **The device hub applies it, once.** `DeviceHub::motorOutput()` returns the command in the
  *mechanism frame* (duty, voltage and torque current negated for a reversed slot) and
  `writeMotorFeedback()` negates rotor position, velocity and acceleration for it, which is what the
  controller firmware reports for an inverted device. Mechanisms, the drivetrain and future consumers
  therefore never deal with inversion. Commands always reach the emulator *after* the controller's own
  inversion, so Reversed describes the mounting (a motor that robot code inverted because it sits
  mirrored is Reversed).
- **A gearbox has up to `kMaxMotorsPerAxis` (8) motors.** Mechanisms expose motor slots
  `motor`..`motor8` (intake axes up to 4, `deployMotor2`, ...); a wheel exposes `<wheel>.drive`,
  `<wheel>.drive2`, ... (`driveSlotName`). `SlotSpec::countKey/ordinal` mark a slot as active only while
  the owner's count (mechanism property `motorCount`, `WheelSpec::driveMotorCount`) is at least the
  ordinal. The count stays the single source of truth; mappings above it are dropped by the editor and
  reported by the validator (`slot_motor_inactive`).
- **Each motor keeps its own voltage.** All motors of a gearbox are the same motor model. The
  mechanism plant is fed the mean voltage (torque is linear in the voltage, so this is exact) while
  currents are computed per motor and summed in absolute value; battery draw is the sum of
  `max(0, V*I)`. Two equal motors with opposite signs cancel their torque but draw full current. A wheel
  does the same in `TractionDrivetrain::driveFor` (torque, damping, supply current per motor,
  current limit per motor), driven by `WheelCommand::driveMotorVoltagesV`. Unmapped extra motors mirror
  motor 1 in the same direction (they have no mapping to carry a Reversed flag).
- **Editor.** The Device slots list shows one row per motor with a controller picker and a Reversed
  check box, plus `+ Add motor` / `- Remove motor`. Dropping a motor controller on a part with no free
  motor row appends a motor (one undo step). A wheel that follows another wheel (ADR 0012) has no rows.

## Consequences

- Motor inversion and multi-motor gearboxes are simulated exactly in the mechanism frame, including
  fighting motors and per-motor currents.
- ADR 0012's approximation is unchanged: a follower wheel adds no load to the source's motors, so
  three motors on a tank side move as many wheels as the source's gearbox torque allows, not as a
  shared-shaft solve.
- Reversed is set by hand. Reading `Inverted` / `Opposed` from the controller configuration (the CTRE
  bridge already reads it back) and auto-filling the flag is a follow-up.
- Whether a vendor follower device reports a motor voltage in its simulation state is vendor behaviour;
  when it does not, leave the extra motors unmapped (they mirror motor 1) and set the count.
- `limit switch` feedback on a motor slot is not mirrored for reversed motors.
