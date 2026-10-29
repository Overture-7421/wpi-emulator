# Week 4 workshop: referee one tick at a time

**In 30 seconds:** a tick reads the Driver Station, applies robot interactions, updates plate
ownership and accumulates score. The picture gets a copy of the result, never the live world.
Continue the open `State` class from [reset and pieces](03a-reset-and-pieces.md).

## 1. Ask the Driver Station which period is running

The host provides `dt` in seconds from its fixed simulation step. Disabled, emergency stop and
zero remaining match time all stop scoring and interactions. A remaining time of -1 means an
untimed run, not "match expired." Endgame is the last thirty seconds of a timed teleop period.
There is no second wall-clock timer hidden in the plugin.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    void step(s::SimContext& ctx, double dt) override {
        const auto& ds = ctx.driverStation();
        remainingS_ = ds.matchTimeS;
        const bool running = ds.enabled && !ds.eStop && ds.matchTimeS != 0.0;
        const bool autonomous = ds.mode == s::proto::RobotMode::Autonomous;
        const bool teleop = ds.mode == s::proto::RobotMode::Teleop;
        const bool endgame = teleop && ds.matchTimeS >= 0.0 && ds.matchTimeS <= 30.0;
        phase_ = !running ? "Disabled" : autonomous ? "Autonomous" : endgame ? "Endgame" : teleop ? "Teleop" : "Test";
        if (running) {
            if (autonomous) autoSeconds_ += dt;
            if (teleop) teleopSeconds_ += dt;
            if (endgame) endgameSeconds_ += dt;
            interact(ctx);
        }
```

Count physical cubes even when disabled so the field picture can settle, but award points only
in enabled autonomous or teleop. A switch is useful to its own alliance only; the neutral scale
can score for either side. An equal number of cubes is unowned.

**Approximation:** this first referee integrates ownership at 2 points/s in autonomous and
1 point/s in teleop. It does not implement the official discrete acquisition/whole-second award
rules. Fractional scores are therefore expected; do not use this as a competition scoreboard.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        updatePlates(dt);
        if (running && (autonomous || teleop)) {
            const double rate = autonomous ? 2.0 : 1.0;
            for (std::size_t pair = 0; pair < 3; ++pair) {
                const int a = plates_[pair * 2].cubes;
                const int b = plates_[pair * 2 + 1].cubes;
                if (a == b) continue;
                const int side = a > b ? 0 : 1;
                const int owner = ownerOf(pair, side);
                if ((pair == 0 && owner != 0) || (pair == 1 && owner != 1)) continue;
                scores_[static_cast<std::size_t>(owner)] += dt * rate;
            }
```

The auto-run check is deliberately conservative: the complete configured chassis length must
cross the line. It is a simple axis-aligned bounding-box check, not the manual's bumper test
for every robot orientation. This straight-driving fixture stops against the switch fence and
does not earn an auto-run bonus. The boolean prevents awarding the bonus every tick.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
            if (autonomous && ctx.chassisBody().valid()) {
                const auto alliance = s::proto::allianceOf(ds.station);
                const std::size_t side = alliance == s::Alliance::Blue ? 0 : 1;
                const float x = world_.pose(ctx.chassisBody()).position.x;
                const float halfLength = ctx.config().chassis.sizeM.x * 0.5f;
                const bool crossed = side == 0 ? x - halfLength > 3.048f : x + halfLength < kLengthM - 3.048f;
                if (crossed && !autoRuns_[side]) {
                    scores_[side] += 5.0;
                    autoRuns_[side] = true;
                }
            }
        }
        refreshPositions();
    }
```

## 2. Publish what the driver needs to see

HUD keys are plugin-prefixed. The snapshot owns copied values, so the UI can safely render them
on another thread. Extra phase counters are included to make a full-match test explain itself:
we can assert fifteen autonomous seconds instead of guessing from a console line.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    void publish(s::SnapshotBuilder& out) const override {
        out.setHud("game_2027.blue_score", scores_[0]);
        out.setHud("game_2027.red_score", scores_[1]);
        out.setHud("game_2027.phase", phase_);
        out.setHud("game_2027.remaining_s", remainingS_);
        out.setHud("game_2027.game_data", assignment_);
        out.setHud("game_2027.inventory", inventory_);
        out.setHud("game_2027.released", released_);
        out.setHud("game_2027.portal_blue", static_cast<int>(std::count(reserve_.begin(), reserve_.begin() + 30, true)));
        out.setHud("game_2027.portal_red", static_cast<int>(std::count(reserve_.begin() + 30, reserve_.end(), true)));
        out.setHud("game_2027.auto_seconds", autoSeconds_);
        out.setHud("game_2027.teleop_seconds", teleopSeconds_);
        out.setHud("game_2027.endgame_seconds", endgameSeconds_);
        for (std::size_t i = 0; i < plates_.size(); ++i)
            out.setHud("game_2027.plate_" + std::to_string(i), plates_[i].cubes);
```

Field cubes and portal supply use separate entity groups, with the same yellow box dimensions.
These groups are pictures, not additional physics bodies. Held cubes are absent from both;
the inventory HUD reports the held cube instead.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        s::EntityGroupSample cubes;
        cubes.group = "game_2027.cubes";
        cubes.shape = "box";
        cubes.halfExtentsM = kCubeHalfM;
        cubes.color = "#ffda19";
        cubes.positions = positions_;
        out.addEntityGroup(std::move(cubes));
        s::EntityGroupSample supply;
        supply.group = "game_2027.portal_supply";
        supply.shape = "box";
        supply.halfExtentsM = kCubeHalfM;
        supply.color = "#ffda19";
        supply.positions = reservePositions_;
        out.addEntityGroup(std::move(supply));
```

Moving plates are published from their **current physics poses** rather than duplicated in the
static field description. Their color belongs to an alliance; the count on that plate decides
whether that alliance owns the balance.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        for (std::size_t i = 0; i < plates_.size(); ++i) {
            s::EntityGroupSample plate;
            plate.group = "game_2027.plate_" + std::to_string(i);
            plate.shape = "box";
            plate.halfExtentsM = kPlateHalfM;
            plate.color = ownerOf(i / 2, static_cast<int>(i % 2)) == 0 ? "#2457c5" : "#c52c30";
            plate.positions = {world_.pose(plates_[i].body).position};
            out.addEntityGroup(std::move(plate));
        }
    }
```

The runtime asks for game data before autonomous starts and overlays this message on its requested
Driver Station state. A numeric game value is a separate seam: a generic BeamBreak can consume
inventory without a new HAL bridge. This fixture publishes that value but does not fit a sensor.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    std::optional<std::string> gameSpecificMessage() const override { return assignment_; }

    std::optional<double> gameValue(std::string_view id) const override {
        if (id == "game_2027.inventory") return inventory_;
        return std::nullopt;
    }
```

## 3. Translate station-relative left into field coordinates

The shared message is own switch, scale, far switch. For Blue, left is high Y. Red views the
field rotated 180 degrees; the legal messages have matching first and third letters, so the
same field-side color mapping works for both station views. The real-physics tests exercise
both alliances with all four assignments to catch a plausible-looking mirror error.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
private:
    int ownerOf(std::size_t pair, int side) const {
        // A shared FMS string describes alliance-relative left. The far switch is rotated 180 degrees.
        const char assignment = assignment_[pair == 2 ? 1 : pair == 0 ? 0 : 2];
        const int blueSide = (assignment == 'L' ? 0 : 1);
        return side == blueSide ? 0 : 1;
    }
```

An unresolved mechanism/link is a configuration problem, not an instruction to release a cube
at the origin. Log it once per identifier per match. We will use this helper when evaluating
zone attachments and activation in the next workshop.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    void warnMissing(s::SimContext& ctx, const std::string& id) {
        if (std::find(missingWarnings_.begin(), missingWarnings_.end(), id) != missingWarnings_.end()) return;
        missingWarnings_.push_back(id);
        ctx.logger().warn("game_2027", "Power UP zone has an unresolved attachment or mechanism: " + id);
    }
```

**You now have:** the public game-state methods and their observable keys. The private helpers
are still to come; do not invent stubs for them.

Next: [make a PWM-controlled mechanism release a real cube](04b-zone-interactions.md).
