# Configure and extend mechanisms

A mechanism is the simulated robot part that turns mapped motor commands into motion and sensor
feedback. Its plant models the load; a motor preset supplies shaft constants; a device bridge
translates robot-process signals. These are separate layers. The
[mechanism reference](../reference/mechanisms.md) owns slots, units and lifecycle contracts.

A game interaction such as collecting a game piece usually belongs in a
[game-plugin zone](game-plugins.md#add-a-zone-interaction), not a new motorized “intake” template.

## Start with the generic template

Core registers one template, type `mechanism`, in
[`mechanism_family.cpp`](../../core/mechanisms/src/mechanism_family.cpp).
Its `motion` property selects rotary, arm or linear behavior. Configure the motor, count, reduction,
inertia or mass, stops and gravity before adding code.

The following exact excerpt from
[`test_mechanism_templates.cpp`](../../tests/mechanisms/test_mechanism_templates.cpp) is inside
`rotary spin-up matches the WPILib LinearSystemId solution and drives the sensors`.
Use that test's `Rig` and `props` helper, not these lines as a complete program:

```cpp
Rig rig;
rig.bind("r", "motor", sdk::SlotType::Motor);
rig.bind("r", "encoder", sdk::SlotType::PositionSensor);
rig.bind("r", "velocitySensor", sdk::SlotType::VelocitySensor);
auto m = rig.make(kType, "r", props({{"axis", "y"}, {"gearRatio", 3.0}, {"momentOfInertiaKgM2", 0.002}, {"viscousFrictionNmS", 0.0}}));
rig.drive("r", "motor", 12.0);
rig.run(*m, 100);
```

Expected result: a 3:1 rotary mechanism spins for 100 fixed steps and publishes position/velocity
matching the analytic solution. The same case checks sensor feedback and the moving link.
The rig registers `registerGenericMechanisms`, constructs a `MechanismConfig`, and calls
`MechanismFactory::create` with `MechanismBuildContext{cfg, env.ctx}`.
For a linear stage or pivot, adapt the tested `elevatorProps` and `pivotProps` helpers in that file.

When mapping a real project, create the mechanism in the editor, select compatible discovered
devices for its slots, and save through the editor. Do not copy fixture device IDs into a user's
configuration. An unmapped motor is neutral, not an excuse to bypass the device hub.

## Decide whether to add a plant or a template

Use an `IPlant` when commands, slots and presentation stay the same but the dynamical model differs.
Use a new `IMechanism` when the part has a different slot topology, several independently driven
axes, or different published structure that generic configuration cannot express.
See [plant registration](simulation-strategies.md#plants) before duplicating command/feedback logic.

## Implement the custom template

1. Implement [`IMechanism`](../../sdk/include/wpiemu/sdk/mechanisms/mechanism.h), taking
   `MechanismBuildContext` so construction uses validated config and injected services.
   The small counter in
   [`test_new_year_plugin.cpp`](../../tests/plugins/test_new_year_plugin.cpp) is a complete
   lifecycle example, not a physical motor model.
2. Build `MechanismTemplateInfo`: stable type, display/category, origin, slot specs, property schema
   and useful default geometry. Plugin-owned types and `origin` use the plugin ID prefix.
3. Read commands and stage feedback through mapped slots in `SimContext`; advance an `IPlant`
   rather than hiding physics in a widget or bridge.
4. Publish `MechanismState` with stable value names/units and any moving attachment links.
   `root` is implicit; do not redeclare it. Advertise additional `links` and `motionAxes` in metadata.
5. Implement reset of local state, sensor history and published state. Implement battery current
   reporting; moving successfully does not prove battery draw is correct.
6. For motion-linked followers, honor the prescribed axis motion rather than independently
   integrating it again. For chassis forces, constraints or payloads, submit through `ctx.coupling()`
   before the world step. Follow the [physics coupling contract](../reference/physics.md).

## Register and build at the correct owner

- Generic registration goes through `registerGenericMechanisms` in
  [`template_support.cpp`](../../core/mechanisms/src/template_support.cpp), declared in
  [`mechanism_templates.h`](../../core/mechanisms/include/wpiemu/core/mechanisms/mechanism_templates.h).
  Its services own shared catalog/plant collaborators for the registered factories.
- Plugin-only templates register from `IGameModule::registerMechanisms` through
  `MechanismFactory::registerTemplate`. See the [game-plugin guide](game-plugins.md#bring-up-a-season).
  No concrete-template switch belongs in the app.
- Add implementation sources to the owning module's CMake list:
  [`core/mechanisms/CMakeLists.txt`](../../core/mechanisms/CMakeLists.txt) or the plugin's file.
  Put tests under `tests/mechanisms` or the plugin's source-mirroring test folder. New plugin
  libraries need explicit links in [`tests/CMakeLists.txt`](../../tests/CMakeLists.txt).

Metadata supplies the editor's menu, property form and slots. Only a genuinely new presentation
behavior needs UI implementation; [editor properties](editor-properties.md) explains the normal path.

## Verify behavior, not just registration

```powershell
ctest --preset windows-ninja-release -R "rotary spin-up|motion variant|unmapped required motor|follower motors|motion links:|coupling:"
```

This includes the excerpt and checks defaults/links, neutral operation, motor followers, dependency
ordering and coupling. Add focused tests using
[`FakeSimEnvironment`](../../sdk/include/wpiemu/sdk/testing/fake_sim_environment.h) and
[`FakeDeviceHub`](../../sdk/include/wpiemu/sdk/testing/fake_device_hub.h) for your template's
command/feedback/reset path. Use real physics for contact geometry.

If geometry does not move, check the published link name and frame before changing QML.
If an arm falls the wrong way, compare the tested arm convention, mass/length and gravity settings;
do not silently invert mapped motors a second time.

## Motor data and behavior

### Choose the missing layer

- A **motor preset** is one motor's voltage, stall torque/current, free current and free speed.
  It is data, independent of CAN/PWM controllers.
- A **device bridge** adapts the robot library's simulated controller/sensor values. Follow
  [device adapters](devices.md); adding a preset does not make a controller discoverable.
- A **plant** models a load's position/velocity dynamics, gravity and stops. See
  [plants](simulation-strategies.md#plants).
- An **IMotorModel** supplies the shaft current/torque law; it is not a season plugin or another
  physical motor preset.

### Add a sourced preset

The SDK owns the single built-in motor table in
[`motor_model.h`](../../sdk/include/wpiemu/sdk/dynamics/motor_model.h). Core's
[`MotorCatalog::builtin`](../../core/mechanisms/src/motor_catalog.cpp), the SDK defaults and editor
and wizard choices consume it. Do not add a second vendor, core or season table.

1. Source constants from a manufacturer specification or the matching WPILib `DCMotor` factory;
   distinguish FOC and non-FOC variants.
2. Add a unique snake-case key, label and one-motor SI values to `motors::presets()`. The helper
   accepts free speed in RPM and converts it to rad/s.
3. Check `motorSpecFaults`: voltage, stall torque/current and free speed must be positive and
   finite; free current must be finite, nonnegative and below stall current.
4. Extend independent expected-data assertions in
   [`test_linear_system_and_motors.cpp`](../../tests/mechanisms/test_linear_system_and_motors.cpp).
   Do not derive expected values from the table under test.
5. Exercise selector/binding tests and inspect the editor/wizard choice. A persisted wheel stores a
   `MotorSpec`; a mechanism selects a catalog key through `motorModel`.

These existing rows illustrate the table format; they are not proposed additions:

```cpp
row("neo", "NEO", 12, 2.6, 105, 1.8, 5676),
row("neo_550", "NEO 550", 12, 0.97, 100, 1.4, 11000),
```

The table tests check sourced operating points, unique/valid entries, and model behavior at stall
and free speed. Expected result for a new row: the SDK catalog and selectors expose it, while the
independent assertions verify its constants. No CMake registration is needed for a row in this
header-only table.

### Keep motor count, gearing and direction distinct

Keep constants per motor. Configure `motorCount` separately: multiplying stall torque/current in
the table and then setting a count aggregates twice. Gear ratio is motor revolutions per mechanism
revolution; output position and motor-shaft feedback are different quantities.

For multiple mapped motors, test same-direction and opposed commands: opposed motors can cancel
torque while still drawing battery current. Unmapped followers mirror the primary according to the
mechanism behavior. Apply mapping reversal at the hub, not again in a bridge or plant. Sensor ratio,
inversion and offset are separate settings; see the [mechanism](../reference/mechanisms.md) and
[bridge](../reference/bridges.md) contracts.

### Inject a different behavioral model

`IMotorModel` exposes `spec`, `currentA` and `torqueNm` at the motor shaft. It has no global
registry. `MotorCatalog::createModel` currently creates `DcMotorModel`; adding a preset does not
select a different behavioral class.

Concrete composition points are explicit:

- [`TractionOptions::motorModels`](../../core/physics/include/wpiemu/core/physics/drivetrains.h)
  accepts a factory for directly constructed traction drivetrains. Normal registry construction
  must supply another model in its drivetrain factory; saved config does not discover arbitrary
  C++ model classes.
- [`CurrentLimitedMotorModel`](../../core/mechanisms/include/wpiemu/core/mechanisms/motor_catalog.h)
  composes an owned `IMotorModel`. This focused test excerpt demonstrates that composition:

```cpp
const MotorCatalog c = MotorCatalog::builtin();
const sdk::MotorSpec neo = *c.spec("neo");
CurrentLimitedMotorModel limited(std::make_unique<sdk::DcMotorModel>(neo), 40.0);
```

It runs in [`test_linear_system_and_motors.cpp`](../../tests/mechanisms/test_linear_system_and_motors.cpp);
the surrounding assertions verify current/torque saturation and effective voltage at speed.
Generic mechanism registration accepts a catalog and plant factory, not an arbitrary motor-model
factory. A different mechanism law may need an injected plant or custom mechanism; implementing an
interface alone does not make it selectable.

```powershell
ctest --preset windows-ninja-release -R "motor|Motor|gearbox|shared chassis and wheel|ConfigBindings"
```

Expected result: catalog/model operating-point and aggregation tests pass, as do reversal and
shared wheel-binding cases. Add a focused failure/limit test for a new behavioral model; table
checks cannot validate a new torque law.
