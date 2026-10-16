# ADR 0008 - Physics implementation (`core/physics`)

- Status: accepted
- Date: 2026-10-15
- Implements: [ADR 0003](0003-box3d-chassis-physics.md) (Box3D chassis physics), SDK 1.1 coupling ([ADR 0006](0006-sdk-1-1-amendments.md))
- Scope: `core/physics/`, `tests/physics/`. **No SDK change was needed.**

## What exists

| Piece | Header (`wpiemu/core/physics/`) | Role |
|---|---|---|
| Box3D world | `box3d_world.h` | `createBox3dWorld(PhysicsConfig)` / `box3dWorldFactory()`: the only code that sees Box3D (`src/box3d_world.cpp`). Also implements `IPhysicsJointHost` (`joint_host.h`) |
| Tire models | `tire_models.h` | `BrushTireModel` ("brush"), `PacejkaLiteTireModel` ("pacejka_lite"), `registerTireModels()` |
| Normal loads | `quasi_static_loads.h` | `quasiStaticLoads()` (any layout, lifted wheels, collinear fallback) |
| Chassis build | `chassis_builder.h` | `buildChassisBody()`: hull + collidable primitives + one frictionless pad sphere per wheel + mass |
| Drivetrains | `drivetrains.h` | `SwerveDrivetrain`, `TankDrivetrain`, `MecanumDrivetrain` (common `TractionDrivetrain`), `registerDrivetrains()` |
| Coupling | `chassis_coupling.h` | `ChassisPhysicsCoupling : IMechanismPhysicsCoupling` |
| IMU truth | `chassis_imu.h` | `ChassisImuSensor`, `eulerZyx()` |
| Facade | `chassis_simulator.h` | `ChassisSimulator::create/prepareStep/completeStep/step/reset`, `registerPhysicsComponents()` |

CMake: a separate static library `wpiemu_core_physics` (links `wpiemu_sdk` publicly, `box3d` privately, compiled with
`-ffp-contract=off` / `/fp:precise`); `wpiemu_core` links it publicly. `core/CMakeLists.txt` gained one
`add_subdirectory(physics)`. The target `wpiemu_physics_tests` (EXCLUDE_FROM_ALL) builds only `tests/physics/*.cpp` against that library
for a fast loop; the same files are part of `wpiemu_tests`, which is what CTest runs.

The composition root must call `registerPhysicsComponents(drivetrainRegistry, tireModelRegistry)` and pass
`box3dWorldFactory()` where a `PhysicsWorldFactory` is expected. Tick order with the facade:
mechanisms submit to the coupling -> `prepareStep(commands, dt)` -> world step -> `completeStep(dt)`
(`step()` does the last three; if the runtime owns `world.step()` it calls the two halves around it).

## Adapter behaviour worth knowing

- Ids: `BodyId`/`ShapeId` are `b3StoreBodyId/b3StoreShapeId` (index1 is in the top 32 bits, so 0 is never valid). Every getter
  validates through an adapter-side map, so stale ids give neutral values instead of crashing Box3D. User tags live in an
  adapter map keyed by shape id (not `userData`), so they survive on 32-bit builds.
- Mass: `setMassProperties` is an *override*. Shapes added or destroyed afterwards would make Box3D recompute mass from
  density (the doc says so); the adapter re-applies the override after each shape change and stops zero-density shapes from
  touching it. So a plugin can attach an intake sensor to the chassis at any time.
- Events: sensor begin/end carry both tags; events whose shapes were destroyed since (the intake absorbing fuel) are dropped,
  as ADR 0003 prescribed. `contactEvents()` reports Box3D **hit** events (approach speed above the world hit threshold, 1 m/s by default;
  shapes need `contactEvents = true`); begin/end touch events are not reported (the SDK type needs a point/speed that touch events
  do not carry).
- `stateHash()`: FNV-1a over pose (position + quaternion), linear and angular velocity of all user bodies in creation order,
  `-0` folded into `+0`. Internal joint anchor bodies are not part of it and not counted in `stats().bodies`.
- Cylinders are Box3D Y-axis hulls rotated onto the SDK's local Z. Meshes (static only) are copied, baked with the local pose, and
  released with their shape.
- Bit-identical across `workerCount` 1, 2, 4 (test `determinism: the hash does not depend on the physics worker count`,
  Windows/MSVC). Cross-OS/compiler equality is *not* asserted anywhere yet; libm differences (`exp`, `atan`, `sin`) in our tire
  code can break it on other platforms. A CI hash-comparison job is still open.

## Box3D v0.1.0 quirks found and how the adapter handles them

1. `b3Body_SetMassData` does not refresh the cached world-space inverse inertia (`invInertiaWorld`). For a body created without
   density it stays zero, so the chassis would ignore torque for its first step. The adapter re-poses the body once on the
   first override (setup time, no contacts). Later mass changes (payload moving) are correct one step late, which is negligible.
2. `b3Joint_GetConstraintForce` is wrong for **prismatic** joints: it returns `(perp.x, perp.y, axial)` in joint space although the
   slide axis is joint X, and adds the upper-limit impulse with the wrong sign. `jointLoad()` undoes both (remap the
   components, flip the axial sign near the upper limit). Distance and spherical joints report correctly.
3. A mesh shape keeps a reference to its mesh: the adapter owns the mesh and destroys it after the shape.
4. Sensors only report visitors whose mask contains the sensor's category: the robot hull mask therefore includes
   `layer::Sensor` (`kRobotHullMask`) and its hull has `sensorEvents = true`.
5. `workerCount > 1` creates Box3D's own threads (as ADR 0003 noted); default stays 1.

## Chassis conventions

- Body origin = COM (ADR 0003). The hull box centre is at `-comOffsetM` (xy) and `groundClearanceM + sizeM.z/2 - comHeightM` (z).
- `WheelSpec::positionM`: x, y relative to the COM in the chassis frame; **z is the hub height above the floor in the rest pose**
  (`<= 0` means "one wheel radius"). The rest height of the COM is `comHeightM + max(radius - hubZ)`, plus a 2 mm settle margin.
- Derived inertia (when `ChassisSpec::inertiaDiagKgM2` is all zero): solid box of `sizeM` with the parallel-axis shift to the real COM,
  so it differs slightly from the plain box of ADR 0003.
- Wheel pads: sphere of the wheel radius at the hub, friction 0, restitution 0, density 0, category `Robot`, mask
  `Field | Fuel | Robot | Projectile`. Every robot shape carries `ChassisBuildOptions::userTag`.
- Ground rays use `layer::Field` only, so **field geometry must carry the Field category**.

## Drivetrain and motor model

- Layouts: swerve (a wheel is steered if `WheelSpec::steered` or `kind == Swerve`), tank (fixed heading, any wheel count; the left/right
  grouping is just the caller's command wiring), mecanum (traction axis +-45 degrees derived from the wheel position, WPILib X layout:
  FL and RR -45, FR and RL +45; a non-zero `tire.tractionAngleRad` overrides it).
- Roller wheels (`Omni`, `Mecanum`) cap `lateralScale` at 0.15 (the spec may go lower, never higher).
- Per step: acceleration estimate from the velocity change (low-passed with `loadTransferTauS`, gravity subtracted in the chassis frame),
  loads via `quasiStaticLoads` over the **grounded** wheels only (an airborne wheel does not take load), then one `ITireModel::solve` per
  wheel and `applyForce` at the ray hit point. Gravity for that estimate is a constant (9.81, `TractionOptions::gravityMps2`) because
  `IPhysicsWorld` does not expose it.
- Motor: `IMotorModel` (default `DcMotorModel`; inject another through `TractionOptions::motorModels`). The back-EMF slope for the tire
  solver is a finite difference of `torqueNm`, so any monotone model works. Stator current limit (`driveCurrentLimitA`, per motor)
  scales the torque and zeroes the damping. Rotor inertia (5e-5 kg m^2 per motor, reflected through the gearbox) and the steer
  efficiency (0.9) are constants in `TractionOptions`.
- **0 V is brake, not coast**: `WheelCommand` carries only a voltage, so a motor at 0 V is shorted and brakes through its back-EMF
  (plug braking behaves the same). Real controllers default to coast when disabled. If the runtime needs coast it needs a flag
  on `WheelCommand` (additive SDK change); not done here.
- `WheelState::driveMotorCurrentA` / `steerMotorCurrentA` are the **supply-side** (battery) currents `max(0, V * I / Vnominal)` per motor
  group, not stator current: regeneration or a shorted coasting motor must not draw from the battery model (plug braking does).
- Swerve azimuth: motor + gearbox + `steerInertiaKgM2`, continuous (unwrapped) angle, no tire scrub torque on the azimuth, no limit.
- `TractionDrivetrain::setChassisMass` (`IChassisMassAware`) receives the payload-composed mass; loads then use the composed COM and mass.

## Coupling (`ChassisPhysicsCoupling`)

- `capabilities()`: wrench + payload always; constraints only when the world implements `IPhysicsJointHost` (the Box3D adapter does,
  `FakePhysicsWorld` does not, and then `addConstraint` returns an invalid handle).
- `applyWrench` converts to world frame and applies immediately (consumed by the next step, like the world's own forces).
- Payloads: stored per owner, `composeMassProperties` result pushed to the body at once, listener notified only on change.
  The body origin stays where it is; the COM moves inside the body (velocity reported by the world is the COM velocity).
- Constraints: Distance -> rigid distance joint, Prismatic -> prismatic joint with limits relative to the field point, ContactAnchor ->
  spherical joint; each gets an internal static anchor body at the field point. Loads are cached by `onStepCompleted()` (the facade
  calls it) so `constraintLoad()` reports the last completed step; above `maxForceN` the joint is destroyed and `engaged` turns false.
  `reset()` drops all constraints and payloads.

## IMU truth (`ChassisImuSensor`)

Yaw/pitch/roll are intrinsic Z-Y-X as `Quat::fromEuler` / WPILib `Rotation3d`, so **positive pitch = nose down**; climbing a ramp reads
a negative pitch. Rates are in the chassis frame; `accelerationMps2` is the accelerometer's specific force in the chassis frame
(+g up at rest, ~0 in free fall). Mounting offset, zeroing and noise belong to the sensor layer.

## Validation (automated, `tests/physics`; 50 kg, 0.5 m square, COM 0.2 m, mu 1.1, Kraken 6.75:1)

| Scenario | Result | ADR 0003 |
|---|---|---|
| Static settle | COM 0.2 m, 122.6 N per wheel | same |
| 1 V | 2.77 m/s^2 over 0.1 s, free speed 0.396 m/s | 2.8 / 0.39 |
| 12 V unlimited | 9.24 m/s^2, slip ratio 0.41, front/rear 29 / 216 N | 9.24, 32 / 213 |
| 12 V, 60 A | 7.24 m/s^2 | 7.07 |
| Plug braking from 4 m/s | 9.17 m/s^2 | 9.17 |
| Swerve spin in place | 8.485 rad/s | 8.485 |
| Swerve strafe 2 m/s | 2.0 m/s, no yaw | same |
| Tank 4 / 6 wheels straight | 3.0 / 2.0 m/s commanded, reached within 3 % | same |
| Tank turn (+-1 m/s, ideal 4 rad/s) | 1.22 (traction), 3.55 (omni) | 1.64, 3.81 |
| Mecanum | forward 1.90, strafe 1.90, spin 3.00 rad/s | 1.95, 1.95, 3.00 |
| 15 deg ramp | pitch -15.0 deg, total load 473.8 N | 14.4-15.2 deg, 475 N |
| 200 fuel + robot plowing, 1 worker | 0.15 ms mean, 0.5 ms max per tick (Release, whole rig step) | 0.12 ms |

Also covered: adapter id tolerance, filters, ray/sensor/hit events, forces, mass override, meshes, cylinders, kinematic bodies,
`setEnabled`, hash reproducibility and worker independence, tire curves and invariants over an input grid, load solver edge cases,
registry registration, simulator creation errors, coupling wrench/payload/constraints/breakage/reset, IMU conventions.

## Open items

- Cross-platform determinism of the hash (CI job), see above.
- No coast flag in `WheelCommand` (see 0 V above).
- Hit events only (no touch begin/end), 1 m/s default threshold, not configurable through `PhysicsConfig`.
- Tank turn gives somewhat lower yaw than the spike (the 3 s steady value is measured here; the spike's measurement point is unknown).
- Azimuth scrub torque and steer current limits are not modelled.
