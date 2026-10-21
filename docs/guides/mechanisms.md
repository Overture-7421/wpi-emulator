# Configure and extend mechanisms

Read this when modeling a roller, arm, elevator, or a part needing custom dynamics.
Prerequisites: [motor choices](motors.md), [a working build](../BUILDING.md), and the
[mechanism contract](../reference/mechanisms.md), which owns slots, units and lifecycle details.

A mechanism is a motor-driven part with mapped command/feedback slots. A plant is its local
position/velocity model. A game interaction such as collecting fuel usually belongs in a
[zone](game-interactions.md), not a new motorized “intake” template.

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
  `MechanismFactory::registerTemplate`. See [the season walkthrough](game-years.md).
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

Next: [attach game interactions to the mechanism](game-interactions.md).
