# Week 1: pour the carpet

**In 30 seconds:** start with the field frame, then build collision geometry and visuals from the
same measurements. In this emulator, +X runs downfield, +Y points left from the blue alliance's
view, and +Z points up. All lengths are metres.

## 1. Draw the field on graph paper first

Power UP's carpet is nominally 54 by 27 feet: about 16.46 by 8.23 metres. Put the blue alliance wall
at X=0 and the red wall at X=16.46. Center the field-width axis at Y=4.115. The auto lines are 10
feet from each alliance wall. The layout is rotationally symmetric, so the same robot start can be
flipped for the other alliance.

Use the measured field drawings for final placement. The fact sheet gives nominal sizes and a
readable top-down map; the official drawings give the offsets that a quick web summary tends to
round away. Don't scale a screenshot and call the result a measurement.

```text
Blue wall                                                 Red wall
X=0     auto line      switch    platform    scale    platform   switch   auto line    X=16.46m
 |---------|-------------[ ]--------[ ]--------[ ]--------[ ]------[ ]---------|
 +Y=left (blue view)                         field width = 8.23m
```

Describe the field's length, width, rotational symmetry, driver-station viewpoints, and render
primitives in `FieldDescription`. Set the plugin's `fieldFrame()` to the same dimensions and
symmetry so the editor can flip robot starts. Create colliders through `IPhysicsWorld`; a visual
box is not a collider, and a collider is not automatically visible. The SDK keeps those jobs
separate on purpose.

## 2. Give the carpet things to hit

Use static bodies for field structures that never move: walls, rails, fences, platforms, and the
ramps. For a box, create a static `BodyDesc`, add a `ShapeDesc` whose geometry is `BoxShape` with
half-extents in metres, then retain the body ID for cleanup. Use the `layer::Field` category and a
mask that includes the robot and game pieces. Use boxes for most geometry; a small number of boxes
are easier to inspect and tune than one giant box that blocks the robot from somewhere mysterious.
Use the same `Pose3` for the visual and its collider.

The `IField` owns the body IDs it creates and destroys them in its destructor. It is destroyed
before the physics world. Keep that RAII rule; a field that leaves orphan bodies behind will make a
reset look like a poltergeist with a tape measure.

Power UP's switch and scale are real see-saws. The SDK does not expose physics joints, and the
renderer does not currently draw an entity's orientation. For this book, represent plates as
**level kinematic platforms** whose height changes between a small number of positions. Keep the
collision and visual heights synchronized. The cubes remain dynamic and respond to gravity and
contact.

> **Mentor's note:** this models ownership and robot access, not the analog tilt of the real plate.
> A kinematic plate is not a broken approximation if the game question is “who owns this side?” It
> is a broken approximation if you claim to be measuring a cube's moment arm.

## 3. Sixty yellow crates, one reset path

Use dynamic box bodies for the approximately 60 pre-match Power Cubes. The real cube is roughly
33 × 33 × 27 cm and about 1.5 kg. Give each body a stable entity tag, add a box with the cube
half-extents, and set mass properties after its shape is added. Keep a stable ID for each entity so
the game state can update snapshots and scoring without guessing which yellow square it found.

Create the initial layout in `reset`, not in a constructor that may run once while a match is reset
many times. Reset must restore poses and velocities, clear inventory and scoring timers, reset
power-ups, and seed any randomized staging through the supplied simulation context. Do not use
wall-clock time or a second random generator.

The renderer's entity-group path draws axis-aligned instances. Cubes will not visually tumble even
though their physics bodies can rotate. Say so in the presentation notes; don't silently replace the
physical shape with a static billboard that cannot be collected.

## 4. Sensors are volumes, not opinions

Represent exchange, portal, platform, and pickup regions as sensor shapes. Set `sensor` and
`sensorEvents` on the sensor, and enable `sensorEvents` on the visitor shape too. Give the sensor a
plugin-owned category and choose masks deliberately. The field should report “a robot entered the
exchange” or “a cube is in the vault”; the game state decides what those facts mean.

The physics event spans are valid only until the next step or destroy. Copy the IDs or tags you need
before creating or destroying bodies. Avoid changing the world while walking its event spans.
Check entry **and** exit boundaries, including a body that starts inside a volume after reset.

## You now have…

A field frame with a known origin, static geometry that collides, and a plan for dynamic cubes and
sensor zones. Next, we give the pieces jobs and give the match a clock.

Next: [Week 2 and 3—cubes, plates, phases, and game data](03-rules-and-robot.md).
