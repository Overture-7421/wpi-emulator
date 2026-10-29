# Week 5 workshop: when the roller turns, let go

**In 30 seconds:** the robot does not call the plugin. It drives a normal PWM motor.
The generic mechanism publishes its velocity; a configured zone uses that velocity as activation.
Continue the source from [match and snapshots](04a-match-and-snapshots.md).

## 1. Find the interaction in the robot's frame

Ignore zones that belong to another plugin. A chassis-mounted zone starts with an identity
attachment; a mechanism-mounted zone resolves its named link through the SDK directory.
The product of chassis pose, attachment pose and zone-local pose gives the world pose.
Missing attachments disable this interaction and produce a warning, not a silent fallback.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    void interact(s::SimContext& ctx) {
        if (!ctx.chassisBody().valid()) return;
        const auto chassis = world_.pose(ctx.chassisBody());
        for (const auto& zone : ctx.config().zones) {
            if (zone.kind != kPickup && zone.kind != kRelease) continue;
            s::Pose3 attachment;
            if (!zone.attachment.onChassis()) {
                const auto pose = ctx.mechanisms() == nullptr ? std::optional<s::Pose3>{} :
                    ctx.mechanisms()->linkPoseInChassis(zone.attachment.parentMechanismId, zone.attachment.parentLink);
                if (!pose) {
                    warnMissing(ctx, zone.attachment.parentMechanismId);
                    continue;
                }
                attachment = *pose;
            }
```

Activation is the SDK's existing condition language: for our fixture, roller velocity above
1 rad/s. An empty condition list is always active, which is useful in a focused physics test
but is not how the match robot's release is wired. Resolve missing readings before using the pose.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
            std::vector<std::string> missing;
            const bool active = s::evaluateActivation(zone.activation, ctx.mechanisms(), &missing);
            for (const auto& id : missing) warnMissing(ctx, id);
            if (!active) continue;
            const auto pose = chassis * attachment * zone.local;
```

## 2. Transfer a held cube back into physics

Release reuses the held body's ID. Move it to the muzzle, inherit chassis point velocity,
add a small forward release speed, clear its spin, and enable its collider. Only then clear
inventory. The visible cube will fall under gravity and may or may not settle on a plate;
release itself awards no score.

This is a fixed-height chute approximation. The later fixture places it above the switch fence;
it is not a simulated elevator or a geometric model of the gripper.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
            if (zone.kind == kRelease && inventory_ != 0) {
                const s::BodyId cube = cubes_[held_];
                world_.setPose(cube, pose);
                world_.setLinearVelocity(cube, world_.pointVelocity(ctx.chassisBody(), pose.position) +
                                              pose.transformVector({0.2f, 0.f, 0.f}));
                world_.setAngularVelocity(cube, {});
                world_.setEnabled(cube, true);
                active_[held_] = true;
                inventory_ = 0;
                ++released_;
```

Pickup transforms candidate centers into the zone's local frame and checks its half-extents.
Only active field pieces can be picked up: disabled human-player reserves are excluded.
It takes the first matching cube in stable pool order, not the nearest cube. This explicit
one-cube rule prevents picking up the whole pyramid in one tick.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
            } else if (zone.kind == kPickup && inventory_ == 0) {
                const auto toZone = pose.inverse();
                for (std::size_t i = 0; i < cubes_.size(); ++i) {
                    if (!active_[i]) continue;
                    const auto p = toZone.transformPoint(world_.pose(cubes_[i]).position);
                    const auto h = zone.sizeM * 0.5f;
                    if (std::abs(p.x) > h.x || std::abs(p.y) > h.y || std::abs(p.z) > h.z) continue;
                    held_ = i;
                    world_.setEnabled(cubes_[i], false);
                    active_[i] = false;
                    inventory_ = 1;
                    break;
                }
            }
        }
    }
```

## 3. Count settled pieces, not flying yellow shapes

Each plate queries its current sensor overlaps, including bodies that began inside the volume
or have gone to sleep. Filter by our cube tags, then require a cube center over the plate,
above its surface and moving slowly. This is a practical ownership proxy, not a contact-force
or moment-arm calculation. A supported multi-layer pile can count; the fixture tests one cube.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    void updatePlates(double dt) {
        for (Plate& plate : plates_) {
            overlaps_.clear();
            world_.shapesOverlapping(plate.sensor, overlaps_);
            plate.cubes = 0;
            const auto center = world_.pose(plate.body).position;
            for (const s::ShapeId shape : overlaps_) {
                const auto tag = world_.shapeTag(shape);
                if (tag < kCubeTag || tag >= kCubeTag + cubes_.size()) continue;
                const auto p = world_.pose(world_.shapeBody(shape)).position;
                if (std::abs(p.x - center.x) <= kPlateHalfM.x &&
                    std::abs(p.y - center.y) <= kPlateHalfM.y &&
                    p.z >= center.z + kPlateHalfM.z + kCubeHalfM.z - 0.05f &&
                    s::length(world_.linearVelocity(world_.shapeBody(shape))) < 0.4f)
                    ++plate.cubes;
            }
        }
```

After **all** plate counts are known, move each plate toward its target at at most 2 cm/s.
`i ^ 1` is the partner plate: 0 pairs with 1, 2 with 3, 4 with 5.
Using a kinematic velocity rather than teleporting it lets supported cubes ride along.
We keep plates level and move them 6 cm up or down; there is no see-saw torque or angular tilt.
The fixed-step host always supplies a positive `dt`.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        for (std::size_t i = 0; i < plates_.size(); ++i) {
            Plate& plate = plates_[i];
            const int delta = plate.cubes - plates_[i ^ 1].cubes;
            const float offset = delta > 0 ? -0.06f : delta < 0 ? 0.06f : 0.f;
            const auto position = world_.pose(plate.body).position;
            const float target = plate.levelCenter.z + offset;
            const float dz = std::clamp(target - position.z, -0.02f * static_cast<float>(dt),
                                       0.02f * static_cast<float>(dt));
            world_.setLinearVelocity(plate.body, {0.f, 0.f, dz / static_cast<float>(dt)});
        }
    }
```

## 4. Keep ownership separate from the picture

Refresh the two display lists from physics after interactions. Neither snapshots nor these lists
own bodies. `active_`, `reserve_` and the held index form the inventory bookkeeping; reset is the
only place we recreate that initial partition.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    void refreshPositions() {
        positions_.clear();
        reservePositions_.clear();
        for (std::size_t i = 0; i < cubes_.size(); ++i) {
            if (active_[i]) positions_.push_back(world_.pose(cubes_[i]).position);
            if (reserve_[i]) reservePositions_.push_back(world_.pose(cubes_[i]).position);
        }
    }
```

These are the match's persistent members. Notice what is **not** here: a Qt object, HAL controller,
wall clock, network client or static mutable match state. `bodies_` destroys all cubes and plates
when the state dies. The world reference remains valid until after state destruction.
Close the class, but leave the anonymous namespace open for the module.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    s::IPhysicsWorld& world_;
    Bodies bodies_;
    std::array<s::BodyId, kCubeCount> cubes_{};
    std::array<bool, kCubeCount> active_{};
    std::array<bool, kCubeCount> reserve_{};
    std::array<Plate, 6> plates_{};
    std::vector<s::ShapeId> overlaps_;
    std::vector<s::Vec3> positions_;
    std::vector<s::Vec3> reservePositions_;
    std::vector<std::string> missingWarnings_;
    std::string assignment_;
    bool preload_;
    std::size_t held_ = 29;
    int inventory_ = 0;
    int released_ = 0;
    std::array<double, 2> scores_{};
    std::array<bool, 2> autoRuns_{};
    double autoSeconds_ = 0.0;
    double teleopSeconds_ = 0.0;
    double endgameSeconds_ = 0.0;
    double remainingS_ = -1.0;
    std::string phase_ = "Disabled";
};
```

**Inspection:** the physical tests separately check a missing activation reference, pickup without
a preload, scoring while disabled and body cleanup. That is more useful than asserting that
the string "release" appears somewhere in a config.

**You now have:** all `State` methods and members. The remaining piece tells the emulator how to
construct the field and state and how to label their controls.

Next: [register the module and reach the first build checkpoint](04c-module-and-registration.md).
