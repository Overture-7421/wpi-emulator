# FOC requests reach the plant as torque current

Read this when changing how a vendor motor controller's output becomes a plant command.
Prerequisite: [Bridges](../reference/bridges.md#vendor-rules-and-firmware-feedback).

## Status
Accepted.

## Rationale
The emulator and the vendor's firmware simulation each hold a motor model. For a voltage
request they agree by construction: the firmware reports the voltage it applies, and the
plant drives its own motor with it. For a Phoenix 6 FOC request the firmware regulates
torque current instead. It resolves the request to a current with its motor model and
reports the voltage that model needs at the rotor speed the emulator fed back. That
voltage carries CTRE's back-EMF estimate, not ours. When the two models differ, even by a
motor variant, the plant sees a voltage that keeps rising with speed, and a swerve steer
module ran away to free speed while its controller asked for the opposite sign. The
voltage report is the wrong quantity to carry across the seam in those modes.

## Decision
In the FOC control modes the vendor bridge hands the plant the firmware's torque current
as a canonical `torqueCurrent` command, and the plant resolves it against the motor it
simulates (`V = I * R + omega / Kv`). In every other mode that signal is zero and the
voltage path is unchanged. The active mode is a status signal read through the existing
read-only native seam; without it the voltage path is kept and logged once.

## Trade-offs
A plant resolving current with its own model exposes any mismatch with the firmware's
motor as a torque error rather than an instability, which is the better failure. The
decision needs the control mode, so it depends on the native seam being present; the
bridge degrades to the old behaviour without it. A `Follower` cannot be decided, because
its leader's mode is not a status of the follower, and keeps the voltage path.

See [Mechanisms](../reference/mechanisms.md#motor-commands-and-multiple-controllers) and
[Physics](../reference/physics.md#traction-and-loads) for the two consumers.
