# 8. Plates and ownership

## 8.1 Objectives

After this chapter the reader can distinguish cube counts, plate motion and
alliance ownership, and verify a moving collider with real physics. Chapter 7
is the prerequisite. Plate files and tests address R6.

## 8.2 Problem statement

A sensor overlap alone does not establish that a cube rests on a plate.
Moreover, the side carrying more cubes is not an alliance identifier.
The design must preserve these distinctions before scoring can use them.

## 8.3 Background and definitions

**Definition 8.1 (resting cube).** In this model, a candidate cube rests on a
plate when its centre lies over the footprint, is at least the support height
minus 0.05 m, and its linear speed is below 0.4 m/s. These are operational
thresholds, classified in F-08.

The sensor supplies candidates. The footprint rejects cubes overlapping only
an edge; the height test rejects bodies below the surface. The velocity test
rejects fast transients, but does not measure weight or contact force.

## 8.4 Design

A general hinge would couple plate angle to physical loads, but the SDK does not
expose arbitrary-body revolute joints. A static plate with a counter would
provide no moving surface for the robot to encounter. The chosen design uses
kinematic plates driven by cube counts.

**Invariant 8.1 (shared plate pose).** Physics and publication read the same
plate body's pose. There is no independent animation that can diverge from
the collider.

## 8.5 Implementation

`Plate` stores a body, a sensor and the centre used at reset. `Plates` owns
all six plates and their sensors. Sensor shapes are attached above the solid
surface and accept only the generic game-piece layer.

Listing 8.1. One plate's physical state. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\plate.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\plate.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/constants.h"

namespace wpiemu::plugins::game_2027 {
struct Plate {
    s::BodyId body;
    s::ShapeId sensor;
    s::Vec3 levelCenter;
    int cubes = 0;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 8.2. Plate collection and rule interfaces. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\plates.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\plates.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/body_set.h"
#include "wpiemu/plugins/game_2027/plate.h"

namespace wpiemu::plugins::game_2027 {

int plateOwner(std::string_view assignment, std::size_t pair, int side);
int heavierSide(int left, int right);

class Plates {
public:
    explicit Plates(s::IPhysicsWorld& world) : world_(world), bodies_(world) {}
    s::Status build();
    void reset();
    void update(double dt);
    const std::array<Plate, 6>& values() const { return plates_; }
    s::Vec3 center(std::size_t i) const { return world_.pose(plates_[i].body).position; }

private:
    s::IPhysicsWorld& world_;
    Bodies bodies_;
    std::array<Plate, 6> plates_{};
    std::vector<s::ShapeId> overlaps_;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 8.3. Plate and sensor construction. <code>seasons\2027\game\src\plate_build.cpp</code>.

<!-- book-file: seasons\2027\game\src\plate_build.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/plates.h"

namespace wpiemu::plugins::game_2027 {

s::Status Plates::build() {
    for (std::size_t i = 0; i < plates_.size(); ++i) {
        Plate& plate = plates_[i];
        const std::size_t pair = i / 2;
        const float x = pair == 0 ? kSwitchXM : pair == 1 ? kLengthM - kSwitchXM : kLengthM / 2.f;
        const float offset = pair == 2 ? 1.6764f : kPlateOffsetM;
        plate.levelCenter = {x, kWidthM / 2.f + (i % 2 == 0 ? offset : -offset),
                             (pair == 2 ? 1.524f : 0.2286f) - kPlateHalfM.z};
        auto body = bodies_.box({plate.levelCenter, {}}, kPlateHalfM, s::BodyType::Kinematic,
                                s::layer::Field);
        if (!body) return body.error();
        plate.body = body.value();
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

void Plates::reset() {
    for (Plate& plate : plates_) {
        world_.setPose(plate.body, {plate.levelCenter, {}});
        world_.setLinearVelocity(plate.body, {});
        plate.cubes = 0;
    }
}

}  // namespace wpiemu::plugins::game_2027
```

`heavierSide` returns -1 for a tie, independently of assignment.
`plateOwner` maps a side to an alliance. Pair indices 0 and 1 are the two
switches; pair 2 is the scale. The assignment's middle letter therefore maps
pair 2, not pair 1.

Listing 8.4. Side selection and alliance mapping. <code>seasons\2027\game\src\plate_rules.cpp</code>.

<!-- book-file: seasons\2027\game\src\plate_rules.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/plates.h"

namespace wpiemu::plugins::game_2027 {

int plateOwner(std::string_view assignment, std::size_t pair, int side) {
    const auto index = pair == 2 ? 1 : pair == 0 ? 0 : 2;
    const auto blueSide = assignment[index] == 'L' ? 0 : 1;
    return side == blueSide ? 0 : 1;
}

int heavierSide(int left, int right) {
    return left == right ? -1 : left > right ? 0 : 1;
}

}  // namespace wpiemu::plugins::game_2027
```

`Plates::update` counts candidates before setting velocities. The target offset
is -0.06 m for the heavier side, +0.06 m for the other side and zero for a tie.
At 0.02 m/s a level plate takes three seconds to reach an extreme. The
clamp prevents overshoot when the target is nearer than one tick's travel.

Listing 8.5. Candidate filtering and rate-limited motion. <code>seasons\2027\game\src\plates.cpp</code>.

<!-- book-file: seasons\2027\game\src\plates.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/plates.h"

namespace wpiemu::plugins::game_2027 {

void Plates::update(double dt) {
    for (Plate& plate : plates_) {
        overlaps_.clear();
        world_.shapesOverlapping(plate.sensor, overlaps_);
        plate.cubes = 0;
        const auto center = world_.pose(plate.body).position;
        for (const s::ShapeId shape : overlaps_) {
            const auto tag = world_.shapeTag(shape);
            if (tag < kCubeTag || tag >= kCubeTag + kCubeCount) continue;
            const auto p = world_.pose(world_.shapeBody(shape)).position;
            if (std::abs(p.x - center.x) <= kPlateHalfM.x &&
                std::abs(p.y - center.y) <= kPlateHalfM.y &&
                p.z >= center.z + kPlateHalfM.z + kCubeHalfM.z - 0.05f &&
                s::length(world_.linearVelocity(world_.shapeBody(shape))) < 0.4f)
                ++plate.cubes;
        }
    }

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

}  // namespace wpiemu::plugins::game_2027
```

Listing 8.6. Library sources through Chapter 8. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp
    src/field_layout.cpp
    src/cube_layout.cpp
    src/cube_pool.cpp
    src/plate_build.cpp
    src/plate_rules.cpp
    src/plates.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 8.6 Verification

The test drops a real cube rather than assigning a count. After settling, the
loaded plate must have moved down by the prescribed amount; reset must restore
its centre and count. This tests Definition 8.1 and Invariant 8.1 together.

Listing 8.7. Physical loading and reset. <code>tests\game_2027\test_plate_motion.cpp</code>.

<!-- book-file: tests\game_2027\test_plate_motion.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2027/field_layout.h"
#include "wpiemu/plugins/game_2027/plates.h"

namespace pu = wpiemu::plugins::game_2027;
namespace s = wpiemu::sdk;

TEST_CASE("Power UP: a physical resting cube lowers a plate then reset levels it") {
    auto world = wpiemu::core::physics::createBox3dWorld();
    pu::Field field(*world);
    REQUIRE(field.build());
    pu::Plates plates(*world);
    REQUIRE(plates.build());
    plates.reset();
    pu::Bodies bodies(*world);
    const auto initial = plates.center(0);
    const auto cube = bodies.box({initial + s::Vec3{0.f, 0.f, 0.5f}, {}}, pu::kCubeHalfM,
                                 s::BodyType::Dynamic, s::layer::Fuel, pu::kCubeTag);
    REQUIRE(cube);
    for (int i = 0; i < 1200; ++i) {
        world->step(0.005f);
        plates.update(0.005);
    }
    CHECK(plates.values()[0].cubes == 1);
    CHECK(plates.center(0).z == doctest::Approx(initial.z - 0.06f).epsilon(0.01));
    CHECK(plates.center(1).z > initial.z);
    plates.reset();
    CHECK(plates.center(0) == initial);
    CHECK(plates.values()[0].cubes == 0);
}
```

## 8.7 Reproduction

Extract through Chapter 8, rebuild and run:

```powershell
ctest --test-dir build\book -R "Power UP: a physical resting" --no-tests=error --output-on-failure
```

## 8.8 Discussion

Rows F-07 and F-08 separate plate mechanics from ownership observation.
Chapter 15 adds the manual's one-second acquisition rule. A count inequality
alone is not yet a complete scoring condition.

## 8.9 Summary

Plate motion now follows observed cubes and remains visible through the same
body pose. Side selection and alliance mapping are separate pure decisions.
The next chapter determines when the game may award points.

## 8.10 Exercises

1. Calculate the maximum travel during a 5 ms tick.
2. Add a test that removes the loaded cube and waits for both plates to level.
3. Identify a cube trajectory that passes the sensor but fails Definition 8.1.

[Chapter 9](09-match-time.md)
