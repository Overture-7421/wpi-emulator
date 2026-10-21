# Add motor data or a motor model

Read this when a mechanism needs different physical motor constants or different torque behavior.
Prerequisites: [a working build](../BUILDING.md) and the
[mechanism contract](../reference/mechanisms.md).

## Identify which kind of support is missing

- **Motor preset:** one motor's voltage, stall torque/current, free current and free speed.
  This is data, independent of CAN/PWM controllers.
- **Device bridge:** the robot library's simulated controller/sensor values. Follow
  [device adapters](devices.md); a new preset does not make a controller discoverable.
- **Plant:** a load's position/velocity dynamics, gravity and stops. Follow
  [simulation strategies](simulation-strategies.md#plants).
- **Motor model:** the shaft current/torque law, behind `IMotorModel`.
  It is not a season plugin or an additional physical motor preset.

## Add a sourced preset in the shared table

The **SDK**, not core, owns the authoritative built-in data in
[`motor_model.h`](../../sdk/include/wpiemu/sdk/dynamics/motor_model.h):
`motors::presets()` returns `MotorPreset` entries. Core's
[`MotorCatalog::builtin`](../../core/mechanisms/src/motor_catalog.cpp) consumes those entries.
SDK defaults and editor/wizard motor choices also derive from the SDK table.
Do not add a second core/vendor/season table.

1. Obtain constants from a cited manufacturer specification or the matching WPILib `DCMotor`
   factory, distinguishing FOC from non-FOC variants. Record the source next to the table/test.
2. Add a unique stable snake-case key and display label to `motors::presets()`. Use **one motor**
   and SI units; the helper takes RPM and converts to rad/s.
3. Check `motorSpecFaults`: voltage, stall torque/current and free speed must be positive and finite;
   free current must be finite, nonnegative and below stall current.
4. Extend the independent expected-data assertions in
   [`test_linear_system_and_motors.cpp`](../../tests/mechanisms/test_linear_system_and_motors.cpp).
   Do not generate expected values by reading the table being tested.
5. Exercise the shared selector/binding tests, then inspect the editor/wizard choice.
   A persisted wheel stores a `MotorSpec`; mechanisms select a catalog key through `motorModel`.

For orientation, these are exact lines from the existing table, not a proposal for another NEO row:

```cpp
row("neo", "NEO", 12, 2.6, 105, 1.8, 5676),
row("neo_550", "NEO 550", 12, 0.97, 100, 1.4, 11000),
```

The catalog table test checks those sourced operating points, accepts every SDK row, and exercises
each model at stall and free speed. No new CMake registration is needed for a data row in this
header-only table.

## Keep gearing, count and direction separate

Keep motor constants per motor. Configure `motorCount` independently: multiplying the table's stall
torque/current and then setting count would aggregate twice. A gear ratio is motor revolutions per
mechanism revolution; output position and motor-shaft feedback are different quantities.

For multiple mapped motors, test both same-direction and opposed commands: opposed motors can
cancel torque while still drawing battery current. Unmapped followers mirror the primary according
to the existing mechanism behavior. Apply mapping reversal at the hub, not again in a bridge or plant.
Sensor ratio, inversion and offset are separate sensor settings. The exact conversions belong in
the [mechanism](../reference/mechanisms.md) and [bridge](../reference/bridges.md) contracts.

## Use the real behavioral injection seam

`IMotorModel` exposes `spec`, `currentA` and `torqueNm` at the **motor shaft**. It has no global
registry. `MotorCatalog::createModel` currently creates `DcMotorModel`; adding a preset does not
select a new behavioral class.

There are concrete composition points:

- [`TractionOptions::motorModels`](../../core/physics/include/wpiemu/core/physics/drivetrains.h)
  accepts a factory for directly constructed traction drivetrains. To use another model in normal
  registry construction, explicitly supply it in the drivetrain factory; there is no saved
  configuration key that discovers arbitrary C++ models.
- [`CurrentLimitedMotorModel`](../../core/mechanisms/include/wpiemu/core/mechanisms/motor_catalog.h)
  takes an owned `IMotorModel`. This existing excerpt from its focused test demonstrates composition:

```cpp
const MotorCatalog c = MotorCatalog::builtin();
const sdk::MotorSpec neo = *c.spec("neo");
CurrentLimitedMotorModel limited(std::make_unique<sdk::DcMotorModel>(neo), 40.0);
```

This excerpt runs inside
[`test_linear_system_and_motors.cpp`](../../tests/mechanisms/test_linear_system_and_motors.cpp),
with that file's includes and aliases. The following assertions check current/torque saturation and
the effective voltage at speed.

Generic mechanism registration accepts `MechanismTemplateServices` with a catalog and plant factory,
not an arbitrary motor-model factory. A different mechanism law may therefore require an injected
plant or a custom mechanism implementation. Do not claim that a model becomes available simply by
implementing the interface.

## Validate the change

```powershell
ctest --preset windows-ninja-release -R "motor|Motor|gearbox|shared chassis and wheel|ConfigBindings"
```

This runs the two excerpts' surrounding tests, table uniqueness/physical validation, model operating
points, aggregation, reversal, and shared wheel binding coverage. Add a regression for any new
behavioral model's limiting or failure case; table checks alone cannot validate a new torque law.
If selectors omit a row, inspect shared-table consumption rather than introducing a UI-local list.

Next: [build the mechanism around that motor](mechanisms.md).
