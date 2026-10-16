# ADR 0009 – Generic mechanisms, plants and motor models (`core/mechanisms`)

Status: accepted · Scope: `core/mechanisms/` (compiled into `wpiemu_core`), `tests/mechanisms/`, `docs/mechanisms.md`.
No SDK change was needed (SDK 1.1 used as frozen).

## Decisions

1. **Own the WPILib maths instead of linking wpimath.** The plan allowed wpimath for plants; we reproduce the four
   `LinearSystemId` systems (elevator, flywheel, DC motor, single-jointed arm) and the matrix-exponential
   discretization (`discretizeAB`) in ~150 lines. Reasons: `core` stays dependency-light and buildable on all
   three OSes without WPILib natives; the equations are tiny and verified against WPILib's own unit-test
   constants (elevator A = −99.05473 / B = 20.8, flywheel/DC A = −26.87032 / B = 1354.166667 for 2×NEO) and analytic
   solutions. The SDK `IPlant` abstraction means wpimath could still be dropped in behind `PlantFactory` later.
2. **One plant engine, thin subclasses** (`Dc1DofPlant` + DC-motor/arm/elevator gravity hooks) registered under the
   five keys `dc_motor`, `turret`, `flywheel`, `arm`, `elevator`. Linear cases use the exact discretization; nonlinear
   cases (angle-dependent gravity, efficiency < 1) use RK4 with sub-steps bounded by the dynamics rate. Coulomb
   friction is an explicit post-step correction (stiction + sliding) so the integrators stay smooth.
3. **Motor data table = values of WPILib 2026 `DCMotor`** (`MotorCatalog`, a value, not a global). The frozen SDK
   `motors::cim()` has 5330 rpm vs WPILib's 5310; the catalog follows WPILib. Per-motor specs plus `motorCount`
   reproduce WPILib's `numMotors` aggregation (`Kt` unchanged, `R/N`).
4. **Battery/current handling lives in `MotorDrive`, not the plant**: supply saturation (battery sag via
   `IDeviceHub::batteryVoltage()`), torque-current requests, neutral mode (brake = 0 V, coast = back-EMF) and a
   stator-current limit expressed as a voltage limit (`CurrentLimitedMotorModel::limitedVoltage`). The plant stays a
   pure `V → motion` model. Battery draw reported to the runtime is `Σ max(0, V·I)/Vbattery`.
5. **A single "axis" building block** (`MechanismAxis`, private): motors → voltage → `IPlant` → feedback
   (rotor side, output encoders, limit switches). Templates are data-driven flavours of three classes
   (`RollerMechanism`: roller/shooter/indexer; `OneDofMechanism`: generic_1dof/pivot/turret/elevator;
   `IntakeMechanism`: deploy axis + roller), registered through `registerValidated` which applies schema defaults,
   validates, logs and returns null on error. Adding a template = a flavour + one registration line (OCP).
6. **Followers**: one logical axis with `motorCount` motors; extra motor slots optional. Mapped extras contribute their
   own voltage, unmapped ones mirror the primary; the plant sees the mean (currents are linear in V).
7. **Slots/sensors follow ADR 0002**: motor slots receive *rotor-side* feedback (`output × gearRatio`); `encoder` /
   `velocitySensor` slots get output-side values × `sensorRatio`. Limit switches come from the stops; beam breaks are
   **never written** by the templates unless `generic_1dof` is set to the `window` source, so a game plugin owns them
   through `ctx.devices().writeDigital` (plugins depend only on the SDK).
8. **Game-agnostic**: no season concepts. 2026 behaviour (fuel absorb/feed) reads `state().active` and named values
   (`rollerSurfaceSpeed`, `surfaceSpeed`, …) or wraps these templates through the factory.
9. **Gravity constant 9.8** in arm/elevator plants (WPILib sims), independent of `PhysicsConfig::gravity`; documented.
10. **Default damping** of 0.001 N·m·s/rad on roller-type axes so a coasting roller spins down (an ideal DC model with
    a disconnected motor never loses speed).

## Consequences
* New code is part of `wpiemu_core`; sources are added by `core/mechanisms/CMakeLists.txt` (`target_sources`).
* The runtime gets `registerGenericMechanisms(MechanismFactory&, MechanismTemplateServices = {})`; the services are
  shared pointers so the registered callbacks never dangle.
* Known limits / open: no external-load hook from game logic into a running mechanism (use payload mass or the
  constraint/coupling API once the runtime wires it); the stator current limit uses the speed at the start of the
  tick (can overshoot by a few percent at 200 Hz during fast transients); no thermal model; no end-stop restitution
  (WPILib semantics: clamp + zero velocity).
