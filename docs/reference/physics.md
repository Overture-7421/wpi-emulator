# Physics

**Read this when:** changing chassis contact, traction, determinism or mechanism-to-chassis
loads without leaking an engine dependency into models.
**Prerequisites:** [Architecture](../ARCHITECTURE.md) for tick ownership;
[Mechanisms](mechanisms.md) for plants and modeled motion axes.

Use [Simulation strategies](../guides/simulation-strategies.md) for drivetrain, tire, and backend
implementation steps; this page defines the physical contract they must preserve.

## Rigid-body boundary

[IPhysicsWorld](../../sdk/include/wpiemu/sdk/physics/physics_world.h) is the engine-neutral
contract; [the Box3D adapter](../../core/physics/src/box3d_world.cpp) contains engine calls.
Keeping that boundary allows physics fakes and isolates upstream API changes.
The pinned dependency is defined in [dependency configuration](../../cmake/WpiEmuDependencies.cmake).
The adapter tolerates stale ids with neutral results, retains explicit mass overrides
when shapes change, and copies mesh input; mesh shapes are static-body-only.
Sensor events require compatible filters and `sensorEvents` on both shapes.
`contactEvents()` exposes hits, not touch begin/end; destroyed-shape events are discarded.

The chassis is one dynamic rigid body with no suspension. One zero-friction pad sphere
per wheel supplies support contact; custom tires supply tangential force at the ground
point. Pad friction must stay zero or the contact solver fights the tire model.
Ground rays find point/normal only; they are not springs and query `layer::Field`,
so drivable field geometry must use that category.

The base chassis origin is its COM; the hull is offset relative to it.
Wheel X/Y are chassis-local, but `positionM.z` is hub height above the rest floor
(`<= 0` means one wheel radius). Spawn height is derived from COM height and wheel
geometry, not from a start position's X/Y/yaw. Derived box inertia includes the offset
to the actual COM. See [chassis construction](../../core/physics/src/chassis_builder.cpp)
and [ChassisSpec](../../sdk/include/wpiemu/sdk/physics/chassis_spec.h) for dimensions/defaults. The default
wheels of the built-in kinds, shared by the wizard, the editor's add-wheel and the demo robots, are in
[wheel_layout.h](../../sdk/include/wpiemu/sdk/physics/wheel_layout.h).

## Traction and load model

`IDrivetrain` applies forces before the world step; it never steps the world itself.
Built-in kinds are `swerve`, `tank` and `mecanum`. Tank grouping is caller wiring,
not a fixed wheel count. Swerve integrates continuous azimuth motor motion.
Mecanum uses the X-layout traction axes (FL/RR -45°, FR/RL +45°), unless overridden
by a nonzero `tractionAngleRad`. Roller wheels cap lateral scale at 0.15.
See [drivetrain implementation](../../core/physics/src/drivetrains.cpp).

Normal loads use a quasi-static minimum-norm balance over **grounded** wheels:
`sum(N) = m*gNormal`, `sum(N*x) = -h*m*ax`, `sum(N*y) = -h*m*ay`.
Negative loads become zero and remaining loads are renormalized; collinear or fewer
than three grounded wheels fall back to equal shares. Chassis acceleration is low-pass
filtered (`loadTransferTauS`, default 20 ms). Payload-composed mass/COM affect this solve.
This avoids contact-impulse spikes at edges becoming enormous traction impulses.
It is an approximate grip/load-transfer model, not a suspension or tipping solver.
The equations and degenerate-layout handling live in
[quasi-static loads](../../core/physics/src/quasi_static_loads.cpp).
`TractionOptions::gravityMps2` defaults to `sdk::kDefaultGravityMps2` (9.81) but is not linked to a
configured `PhysicsConfig::gravity`; changing world gravity alone does not retune the load estimator.

The default `brush` and alternative `pacejka_lite` implement
[ITireModel](../../sdk/include/wpiemu/sdk/physics/tire_model.h). Slip uses wheel rim speed
minus contact-point speed, with a finite reference speed near rest. The semi-implicit
solve couples wheel inertia, motor back-EMF and a chassis mass share so a stiff tire
curve remains stable at the fixed step; the same longitudinal force produces the wheel
reaction torque. Airborne wheels free-spin without traction.
Curve equations and friction limits belong to [tire models](../../core/physics/src/tire_models.cpp).

Drive current limits apply per motor. Reported wheel currents are nonnegative supply
draw, using positive electrical power divided by motor nominal voltage, not stator
current or regeneration. `WheelCommand` contains voltage only: **0 V brakes**, it does
not model a disconnected/coasting controller. Steering has neither tire scrub torque
nor a steer-current limit. One-way follower traction is approximate and adds no reflected
drive load; see [motion links](mechanisms.md#one-way-motion-links).

## Chassis coupling

Mechanisms reach [IMechanismPhysicsCoupling](../../sdk/include/wpiemu/sdk/mechanisms/physics_coupling.h)
through `SimContext::coupling()`, rather than manipulating chassis body ids.
Check `capabilities()`: unsupported operations are no-ops or return invalid handles.
The Box3D-backed implementation supports wrenches/payloads and field-anchored distance,
prismatic and contact-anchor constraints.

Wrenches are chassis-frame force/couple at a chassis-local point, consumed by the next
physics step; resubmit each tick. Constraints persist until removed, broken or reset.
`constraintLoad()` returns the **last completed step's** world-frame reaction on the
chassis; a new/unknown constraint has no valid load yet. A load exceeding `maxForceN`
breaks the constraint and marks it disengaged. Submissions after physics affect the next step.
Payloads latch per owner until changed/cleared/reset. Mass composition uses the
parallel-axis theorem with diagonal inertia only; the body origin stays fixed as the
combined COM moves inside it. The drivetrain receives the composed mass too.
See [coupling implementation](../../core/physics/src/chassis_coupling.cpp).

## Determinism and limits

The engine defaults are one worker and four substeps. Results do not depend on the worker
count, so the app picks it per machine at launch (Settings > Simulation, default Auto:
under four logical cores 1, four to seven 2, eight or more 4) instead of storing it in the
robot model; [resolveWorkerCount](../../core/physics/include/wpiemu/core/physics/box3d_world.h)
holds the rule and `PhysicsStats::workerCount` reports it. With more than one worker Box3D
owns helper threads for the world's lifetime: they are created with the world, sleep between
steps, work only inside `step()` and are joined when the world is destroyed, which must
happen after the sim thread has stopped. Preserve fixed inputs, creation
order and physics settings when comparing runs. Floating-point contraction is disabled
for first-party physics with `/fp:precise` on MSVC and `-ffp-contract=off` elsewhere;
do not add fast-math. The flags live in [physics CMake](../../core/physics/CMakeLists.txt).
`stateHash()` hashes user-body poses and velocities in creation order, folds -0 into +0,
and excludes internal anchor bodies; it is not a serialization of the whole session.
Regressions compare repeated runs and worker counts, but do not guarantee cross-OS or
cross-compiler bit equality: tire math still uses platform math-library functions.
See [determinism coverage](../../tests/physics/test_physics_determinism.cpp).

**Next:** [Extension and IPC](extension-ipc.md).
