# Weeks 2-3 workshop: sixty identities, one reset

**In 30 seconds:** we allocate bodies once, then reset their state instead of destroying and
recreating a match. Stable cube identities make inventory, sensors and cleanup much easier.
Continue appending to the source file begun in [the field workshop](02a-field-implementation.md).

## 1. Name the six plates

There are **six** plates: two on each of two switches, plus two on the scale. Each plate records
its rigid body, its sensor shape, its level position and its current cube count. These belong
to game state because they move with ownership, not to the static field.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
struct Plate {
    s::BodyId body;
    s::ShapeId sensor;
    s::Vec3 levelCenter;
    int cubes = 0;
};

class State final : public s::IGameState {
public:
    State(s::SimContext& ctx, std::string assignment, bool preload)
        : world_(ctx.physics()), bodies_(world_), assignment_(std::move(assignment)), preload_(preload) {}
```

The state borrows the world from `SimContext`, owns its bodies through `Bodies`, and stores the
chosen assignment. First create sixty dynamic cube bodies with consecutive tags. Allocation
happens once; reset will decide which bodies are active, held, or in human-player reserve.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    s::Status build() {
        for (std::size_t i = 0; i < cubes_.size(); ++i) {
            auto body = bodies_.box({{0.f, 0.f, 0.5f}, {}}, kCubeHalfM, s::BodyType::Dynamic,
                                     s::layer::Fuel, kCubeTag + i, {0.5f, 0.0f, 0.0f});
            if (!body) return body.error();
            cubes_[i] = body.value();
        }
```

Plate indices 0/1 are the near switch, 2/3 the far switch, and 4/5 the scale.
Within each pair, the even index is high field Y. The level height describes the plate **surface**;
subtracting its half-thickness gives the body's center. Use kinematic bodies: the game chooses
their velocity, while the physics engine resolves their contact with dynamic cubes.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        for (std::size_t i = 0; i < plates_.size(); ++i) {
            Plate& plate = plates_[i];
            const std::size_t pair = i / 2;
            const float x = pair == 0 ? kSwitchXM : pair == 1 ? kLengthM - kSwitchXM : kLengthM / 2.f;
            const float offset = pair == 2 ? 1.6764f : kPlateOffsetM;
            plate.levelCenter = {x, kWidthM / 2.f + (i % 2 == 0 ? offset : -offset),
                                 (pair == 2 ? 1.524f : 0.2286f) - kPlateHalfM.z};
            auto body = bodies_.box({plate.levelCenter, {}}, kPlateHalfM, s::BodyType::Kinematic, s::layer::Field);
            if (!body) return body.error();
            plate.body = body.value();
```

Add a non-solid box just above each plate. It only detects the generic game-piece layer.
Unlike the plate collider, the sensor must not push anything. The tall detection volume finds
candidate pieces; a later geometric and velocity check rejects pieces still falling through it.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
            s::ShapeDesc sensor;
            sensor.geometry = s::BoxShape{{kPlateHalfM.x, kPlateHalfM.y, 0.50f}};
            sensor.local.position = {0.f, 0.f, 0.525f};
            sensor.sensor = true;
            sensor.sensorEvents = true;
            sensor.category = s::layer::Sensor;
            sensor.mask = s::layer::Fuel;
            plate.sensor = world_.addShape(plate.body, sensor);
            if (!plate.sensor.valid())
                return s::makeError(s::ErrorCode::Internal, "Power UP plate sensor creation failed");
        }
        return s::okStatus();
    }

    std::string_view pluginId() const override { return kId; }
```

## 2. Reset the bookkeeping before the bodies

Reset is a complete new match, not just a score reset. Choose the held cube from the current
alliance's portal reserve. Clear release counts, elapsed phase counters and warn-once state.
The two arrays of thirty positions represent Blue's supply first and Red's supply second.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    void reset(s::SimContext& ctx) override {
        scores_ = {};
        autoRuns_ = {};
        inventory_ = preload_ ? 1 : 0;
        held_ = s::proto::allianceOf(ctx.driverStation().station) == s::Alliance::Blue ? 29 : 59;
        released_ = 0;
        autoSeconds_ = teleopSeconds_ = endgameSeconds_ = 0.0;
        missingWarnings_.clear();
        phase_ = "Disabled";
        remainingS_ = -1.0;
        const auto initialPositions = allianceCubePositions();
```

For the second alliance, rotate X and Y around the field center; do not merely negate X.
The first sixteen pieces per alliance are on-field. The other fourteen stay disabled in the
world, including the optional held preload. A reserved cube is displayed off-field; a held one
is removed from the reserve display. All velocities are cleared, not just positions.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        for (std::size_t i = 0; i < cubes_.size(); ++i) {
            const bool far = i >= 30;
            const std::size_t n = i % 30;
            s::Vec3 position = initialPositions[n];
            if (far) position = {kLengthM - position.x, kWidthM - position.y, position.z};
            reserve_[i] = n >= 16 && !(preload_ && i == held_);
            active_[i] = n < 16;
            world_.setEnabled(cubes_[i], active_[i]);
            world_.setPose(cubes_[i], {position, {}});
            world_.setLinearVelocity(cubes_[i], {});
            world_.setAngularVelocity(cubes_[i], {});
        }
```

Restore all six plate poses and stop their kinematic velocities. Then refresh the display lists.
This keeps a plate from continuing downward after the next match's reset. We have not assigned
ownership yet: zero cubes means level and unowned.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        for (Plate& plate : plates_) {
            world_.setPose(plate.body, {plate.levelCenter, {}});
            world_.setLinearVelocity(plate.body, {});
            plate.cubes = 0;
        }
        refreshPositions();
    }
```

**Student asks:** "Why not delete and respawn all the cubes?" You could, but stale IDs and
unmatched overlap events would become your problem. A fixed pool gives every cube a stable
identity, bounds allocation, and makes a leaked-body test meaningful.

The physics proof later checks 32 starting field pieces, 28 reserve pieces without a preload,
and 27 reserve pieces plus one held cube with a preload. It also lets both pyramids settle for
five simulated seconds. Those are different tests: a correct count does not prove a stable pile.

**You now have:** body allocation and the entire reset path. The `State` class is intentionally
still open; next we add its public tick and snapshot methods.

Next: [set the referee's scope](04-scoring-and-features.md), then write ticks and snapshots.
