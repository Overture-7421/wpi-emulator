# Extend a simulation strategy

Each strategy owns one physical or transport responsibility: a drivetrain maps chassis motion to
wheels, a tire model computes contact forces, a plant models one mechanism axis, a physics world
implements contacts, and a transport carries messages. Change the narrowest seam that expresses
the behavior. The [physics](../reference/physics.md) and
[runtime](../reference/runtime.md) references define the shared contracts.

For physical motor data and shaft laws, use [motor data and behavior](mechanisms.md#motor-data-and-behavior).

## Drivetrains

1. Implement [`IDrivetrain`](../../sdk/include/wpiemu/sdk/physics/drivetrain.h).
   `step` computes forces and wheel state before the world step; it must never step the world.
2. Define `DrivetrainInfo::slotSpecs` as a pure function of `DrivetrainSpec`. The validator/editor
   must see exactly the slots that the constructed instance reports. Reuse `tractionSlotSpecs`,
   `isSteered`, and the drive/steer naming helpers where the traction convention fits.
3. Register the factory through `DrivetrainRegistry::registerDrivetrain` in
   [`drivetrains.cpp`](../../core/physics/src/drivetrains.cpp).
   Existing built-in metadata is shared through `builtinDrivetrainInfos`; do not copy its slot table.
4. Update persisted kind selectors/validation/editor choices when introducing a new selectable key,
   then exercise creation through the runtime registry, not just direct construction.
5. Test reset, finite commands, slot parity, steering and force direction. Start from
   [`test_chassis_scenarios.cpp`](../../tests/physics/test_chassis_scenarios.cpp); include real
   physics when claiming motion or traction behavior.

For example, the shipped tank factory constructs `TankDrivetrain` from `DrivetrainBuildArgs`,
while its metadata uses non-steerable traction slots. Changing only the factory would leave the
editor asking for the wrong slots. The registry tests below assert metadata/instance parity.

## Tires

1. Implement stateless [`ITireModel::solve`](../../sdk/include/wpiemu/sdk/physics/tire_model.h).
   Inputs describe contact/load/slip and wheel drive; output supplies forces and next wheel speed.
2. Register through `TireModelRegistry::registerModel` in
   [`tire_models.cpp`](../../core/physics/src/tire_models.cpp).
3. Add the new key to applicable config validation/selectors and test normal load zero, no slip,
   drive/braking signs, saturation and finite results.

The working brush/Pacejka examples in
[`test_tire_model.cpp`](../../tests/physics/test_tire_model.cpp) compare a gentle drive against
excess torque and enforce the friction limit. Match that kind of observable test instead of asserting
only that `solve` returns. The drivetrain owns wheel state; do not hide mutable wheel history in a
shared tire model.

## Plants

An `IPlant` models one degree of freedom in output-side SI units. Implement its step/reset,
limits, current draw and motor-shaft speed, then register through `PlantFactory::registerPlant`.
The built-in registration is
[`registerBuiltinPlants`](../../core/mechanisms/src/plants.cpp).

This exact excerpt from `plant factory registers the five WPILib-style models once` in
[`test_plants.cpp`](../../tests/mechanisms/test_plants.cpp) checks the factory seam:

```cpp
sdk::PlantFactory f;
REQUIRE(registerBuiltinPlants(f).ok());
for (const char* key : {"dc_motor", "turret", "flywheel", "arm", "elevator"}) CHECK(f.contains(key));
CHECK(registerBuiltinPlants(f).error().code == sdk::ErrorCode::AlreadyExists);
```

It uses that file's core-mechanism namespace and SDK alias. Expected result: each built-in model is
registered once and duplicate setup is rejected. The same file constructs plants with `createFor`,
compares step response against analytic solutions, and tests invalid specs, stops and reset.

To consume your model, supply the factory to the constructing mechanism.
`MechanismTemplateServices::plants` is a shared factory injection point in
[`mechanism_templates.h`](../../core/mechanisms/include/wpiemu/core/mechanisms/mechanism_templates.h).
The generic template still chooses model keys from its rotary/arm/linear settings in
[`axis_support.cpp`](../../core/mechanisms/src/axis_support.cpp); injecting a factory does not add
a new arbitrary model selector. A new key needs an explicit consuming template or selection change.

## Physics backends

1. Implement [`IPhysicsWorld`](../../sdk/include/wpiemu/sdk/physics/physics_world.h), keeping backend
   types private. Use the existing [Box3D adapter](../../core/physics/src/box3d_world.cpp) as the
   behavioral example, not as a public dependency of plugins.
2. Inject the owned world through `SessionParts::world` before `SessionBuilder::build`.
   The [new-year integration test](../../tests/plugins/test_new_year_plugin.cpp) does this with a
   fake; production composition lives under [`app/host`](../../app/host).
   There is no universal backend registry to discover a new class automatically.
3. Match handle invalidation, collision filtering, sensors/contacts, forces/rays, cleanup and reset.
   Add real-backend parity/determinism cases alongside
   [`test_box3d_world.cpp`](../../tests/physics/test_box3d_world.cpp) and
   [`test_physics_determinism.cpp`](../../tests/physics/test_physics_determinism.cpp).

Preserve simulation's single world step and frame conventions. A fake world proves orchestration,
not collision fidelity. Backend limitations belong in the physics reference.

## Transport and message changes

1. Implement `ITransport` or `IMessageCodec` behind the SDK interfaces under
   [SDK IPC headers](../../sdk/include/wpiemu/sdk/ipc), then supply it at both peers' composition roots.
   Follow [`ipc`](../../ipc/src) rather than bypassing framing/handshake in a bridge.
2. For a message field, find its struct through
   [`messages.h`](../../sdk/include/wpiemu/sdk/ipc/messages.h) and update its shared `fields`
   function in [`binary_codec.cpp`](../../ipc/src/binary_codec.cpp).
   Preserve current optional-tail guards and extend golden bytes, round-trip and truncation tests.
3. Update callers together under the single pre-release contract policy. Optional-tail decoder
   behavior is not a promise of supported cross-version interoperability.
4. Test failures on both peers, stale sessions/epochs, reset acknowledgements, one-outstanding-tick
   flow and independent watchdog recovery. See the [IPC contract](../reference/extension-ipc.md).

## Build and run focused checks

Core physics/mechanism `.cpp` files are discovered by their owning CMake source globs; reconfigure
after adding files. IPC uses an explicit source list in
[`ipc/CMakeLists.txt`](../../ipc/CMakeLists.txt). Public SDK headers receive standalone include
checks. New backend dependencies remain private and must respect the boundary checker.

```powershell
ctest --preset windows-ninja-release -R "drivetrain registry:|^tire model:|^plant |^box3d world:|^determinism:|^codec:|^framing:|^transport:"
```

This executes the factory excerpt and existing seam/behavior cases. Add tests for the strategy
actually introduced, including its error/reset path; a passing unrelated built-in is only a baseline.
If a selection finds no tests, list CTest cases using [Building](../BUILDING.md) rather than accepting
an empty run.
