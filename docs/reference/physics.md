# Physics

Read this when changing chassis contact, traction, determinism or mechanism loads.

## Overview

The physics world moves the robot chassis and other rigid bodies. A drivetrain turns
motor commands into tire forces; mechanisms remain separate local models unless they
use the explicit chassis-coupling interface. For shared frames and units, see
[simulation and physics concepts](../concepts/simulation-and-physics.md). Implementation
steps are in [Simulation strategies](../guides/simulation-strategies.md).

## Rigid-body boundary and chassis frame

[IPhysicsWorld](../../sdk/include/wpiemu/sdk/physics/physics_world.h) is the
engine-neutral contract. Core supplies the Box3D adapter; engine types do not enter
SDK headers. The world uses Z-up NWU and SI units; vectors and poses are world-frame
unless named `Local`. SDK quaternions use `(x, y, z, w)`. Only the simulation thread
calls world methods; other threads read snapshots. `step(dtSeconds)` advances exactly
the supplied fixed interval and has no internal accumulator. Forces and torques last
for the next step only, so reapply them each step.

Creation failure returns an invalid ID. Invalid or stale body/shape IDs return neutral
query values and setters ignore them. Destroying a body destroys its shapes. Do not
create or destroy bodies while iterating event spans; copy needed events first.
Explicit mass overrides remain authoritative when shapes change. Mesh shapes are
static-body-only. Sensor events require compatible filters and `sensorEvents` enabled
on both shapes.
`contactEvents()` reports hits, not touch begin/end, and events for destroyed shapes
are discarded.

The chassis is one dynamic rigid body with no suspension. Each wheel has a
zero-friction sphere pad for support; the tire model supplies tangential force at the
ground point. Keep pad friction at zero so the contact solver does not fight the tire
model. Ground rays find a contact point and normal; they are not springs and query
`layer::Field`, so drivable field geometry must use that category.

The chassis body origin is its center of mass (COM); the hull is offset from it.
Wheel X/Y positions are chassis-local. `positionM.z` is hub height above the rest
floor; a value at or below zero means one wheel radius. Spawn height comes from COM
height and wheel geometry, not start X/Y/yaw. Derived box inertia includes the
offset to the COM. Dimensions and defaults are in
[ChassisSpec](../../sdk/include/wpiemu/sdk/physics/chassis_spec.h); built-in wheel
layouts are in [wheel_layout.h](../../sdk/include/wpiemu/sdk/physics/wheel_layout.h).

## Traction and loads

`IDrivetrain` applies forces before the world step; it never steps the world. Built-in
drivetrain kinds are `swerve`, `tank` and `mecanum`. Tank grouping is caller wiring,
not a fixed wheel count. Swerve integrates continuous azimuth motion. Mecanum defaults
to the X layout: FL/RR -45°, FR/RL +45°, unless a nonzero `tractionAngleRad`
overrides it. Roller wheels cap lateral scale at 0.15.

Normal loads are estimated over grounded wheels using a quasi-static minimum-norm
balance:

```text
sum(N)   = m * gNormal
sum(N*x) = -h * m * ax
sum(N*y) = -h * m * ay
```

Here `x` and `y` are grounded wheel coordinates relative to the composed COM; `h` is
its height, and `ax` and `ay` are filtered chassis accelerations.
Negative loads are set to zero, then the remaining loads are renormalized. Fewer than
three grounded wheels or a collinear layout uses equal shares. Chassis acceleration
is low-pass filtered; `loadTransferTauS` defaults to 20 ms. Payload-composed mass and
COM affect the estimate. This approximates grip and load transfer; it is not a
suspension or tipping solver. `TractionOptions::gravityMps2` defaults to
`sdk::kDefaultGravityMps2` (9.81 m/s²), independently of
`PhysicsConfig::gravity`; changing world gravity alone does not retune the estimator.

The default `brush` and alternative `pacejka_lite` models implement
[ITireModel](../../sdk/include/wpiemu/sdk/physics/tire_model.h). Slip compares wheel
rim speed with contact-point speed and uses a finite reference speed near rest. The
fixed-step solve couples wheel inertia, motor back-EMF and chassis mass share so stiff
tire curves remain stable; the same longitudinal force applies wheel reaction torque.
Airborne wheels spin freely without traction. The curves and friction limits are
defined in [tire_models.cpp](../../core/physics/src/tire_models.cpp).

Drive current limits apply per motor. Reported wheel current is nonnegative supply
draw: positive electrical power divided by motor nominal voltage, not stator current
or regenerated power. `WheelCommand` carries voltage only: **0 V brakes** and does
not represent a disconnected or coasting controller. Steering has no tire-scrub
torque or steer-current limit. One-way follower traction is approximate and adds no
reflected drive load; see [motion links](mechanisms.md#one-way-motion-links).

## Chassis coupling

Mechanisms access
[IMechanismPhysicsCoupling](../../sdk/include/wpiemu/sdk/mechanisms/physics_coupling.h)
through `SimContext::coupling()`, not chassis body IDs. Check `capabilities()`:
unsupported operations are no-ops or return invalid handles. The Box3D-backed
implementation supports wrenches, payloads and field-anchored distance, prismatic and
contact-anchor constraints.

A wrench is a chassis-frame force and couple at a chassis-local point. It applies to
the next physics step only, so submit it each tick. Constraints persist until removed,
broken or reset. `constraintLoad()` reports the last completed step's world-frame
reaction on the chassis; new or unknown constraints have no valid load yet. A load
above `maxForceN` breaks the constraint and marks it disengaged. Submissions after a
physics step affect the next step.

Payloads latch per owner until changed, cleared or reset. Mass composition uses the
parallel-axis theorem with diagonal inertia only; the body origin stays fixed while
the combined COM moves inside it. The drivetrain receives the composed mass too.
Reset removes constraints and clears payloads. See
[coupling implementation](../../core/physics/src/chassis_coupling.cpp).

## Determinism and limits

Physics uses fixed inputs and a fixed step; wall-clock pacing is separate. Engine
defaults are one worker and four substeps. The app's worker setting defaults to Auto:
one worker below four logical cores, two for four to seven, and four at eight or more.
The selected count is machine-specific, not stored in the robot model. Results do not
depend on worker count, and `PhysicsStats::workerCount` reports the resolved value.
The default simulation step is 5 ms and must divide the robot's 20 ms period.

With multiple workers, Box3D owns helper threads for the world's lifetime. They are
created with the world, work only inside `step()`, and are joined when the world is
destroyed. Stop the simulation thread before destroying that world. Preserve fixed
inputs, body creation order and physics settings when comparing runs. First-party
physics disables floating-point contraction; do not add fast-math.

`stateHash()` hashes user-body poses and velocities in creation order, folds -0 into
+0 and excludes internal anchor bodies. It is not a serialization of the whole
session. Tests compare repeated runs and worker counts but do not promise bit equality
across operating systems or compilers; tire math still uses platform math libraries.
See [determinism coverage](../../tests/physics/test_physics_determinism.cpp).
