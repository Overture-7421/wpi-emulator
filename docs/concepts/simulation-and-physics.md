# Simulation, physics, and units

Read this when you need a mental model for how robot commands become motion in the emulator.

For example, robot code commands a motor. A bridge carries the robot-facing output into a mapped
mechanism; its model advances, and the drivetrain applies tire forces before the physics world
updates the chassis. The runtime owns this order; see the
[architecture tick](../ARCHITECTURE.md#one-simulation-tick).

## Simulation time and repeatability

The emulator advances by [fixed steps](../GLOSSARY.md), not by whatever time a computer's wall
clock happened to take. The default increment is **5 ms**; the robot period is **20 ms**, and
configured increments must divide that period evenly. The constants and validation live in
[`units.h`](../../sdk/include/wpiemu/sdk/core/units.h) and
[`config_validator_robot.cpp`](../../core/runtime/src/config/config_validator_robot.cpp).
The detailed stepping contract belongs to [Architecture](../ARCHITECTURE.md) and
[Physics](../reference/physics.md).

The configured simulation seed initializes the runtime's random generator. Camera pose noise uses
that generator. Fixed inputs and a fixed seed therefore make repeated comparisons useful, but do
not promise bit-for-bit equality across operating systems or compilers; see
[determinism and limits](../reference/physics.md#determinism-and-limits).

## Mechanism models and rigid-body physics

A **[plant](../GLOSSARY.md)** models how a mechanism moves under commands and loads. The emulator
advances those mechanism models alongside a [rigid-body](../GLOSSARY.md) physics world. The
drivetrain and tire model calculate traction; the physics world updates chassis motion and contacts.
This separates robot-facing controller signals from the physical response. The contracts for
motors and mechanisms live in
[Mechanisms](../reference/mechanisms.md); chassis contact and force details live in
[Physics](../reference/physics.md).

The core talks to physics through the engine-neutral `IPhysicsWorld` interface. The
[Box3D](../GLOSSARY.md) adapter implements that interface in `core/physics`; Box3D types stay out
of the SDK and other modules.
This seam lets the core use test fakes and keeps the engine dependency behind one adapter. See
[`physics_world.h`](../../sdk/include/wpiemu/sdk/physics/physics_world.h) and the
[Box3D adapter](../../core/physics/src/box3d_world.cpp).

## Coordinate frames and units

The world and field use a right-handed **[NWU](../GLOSSARY.md)** frame: +X points toward the
opposing alliance wall, +Y points left, and +Z points up. The robot frame uses +X forward, +Y left,
and +Z up. Poses are
expressed in the frame named by their context; a local attachment pose is not a field pose.

The SDK stores geometry in [SI units](../GLOSSARY.md): meters, kilograms, seconds, radians,
newtons, and newton-meters.
Simulation time is an integer number of microseconds. The SDK
[quaternion](../GLOSSARY.md) component order is `(x, y, z, w)`. These conventions are defined in
[`math.h`](../../sdk/include/wpiemu/sdk/core/math.h) and
[`units.h`](../../sdk/include/wpiemu/sdk/core/units.h).

UI display units and vendor or robot APIs can use other conventions. Convert at those boundaries;
do not change core geometry to match a display. The Qt Quick 3D view uses Y-up, so the app performs
that scene conversion in the UI layer rather than changing the simulation frame.
