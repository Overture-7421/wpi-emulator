# 7. Game pieces

## 7.1 Objectives

After this chapter the reader can construct the initial cube layout, represent
custody without reallocating bodies, and test conservation. Chapter 6 is the
prerequisite. Layout and pool files address R4 and R5.

## 7.2 Problem statement

The initial field is not a uniform grid of cubes. Each alliance has a fence row,
a ten-cube pyramid and off-field portal reserves [1, section 4.1.2, Figure 4-2].
A preload must be deducted from an existing reserve rather than added to it.

## 7.3 Background and definitions

The layout function returns 30 positions for one allocation: six fence cubes,
ten pyramid cubes and fourteen reserves. The opposing allocation is obtained
by the half-turn from Chapter 4. Off-field reserve positions are for display;
those bodies remain disabled.

The pyramid contains triangular layers of six, three and one cube. The layer
counts sum to ten. The arrangement of centres and the shape dimensions together
determine physical support; a layer-count formula alone says nothing about
stability under contact resolution.

## 7.4 Design

Creating and destroying a cube for every intake would make identity and teardown
harder to audit. A fixed pool instead allocates 60 bodies once and enables only
field cubes. Its location array is the sole custody record.

**Invariant 7.1 (pool identity).** Each pool index names one body for the lifetime
of the game state. Transitions change location and pose but never the number of
allocated bodies. Invariant 4.1 then reduces to one location per index.

## 7.5 Implementation

`allianceCubePositions` computes row spacing and the three pyramid tiers.
The pitch equals a cube's full horizontal extent. The tier offsets stagger
upper cubes over the lower layer; the physical stability test, rather than a
claimed theorem about arbitrary stacks, is the acceptance criterion.

Listing 7.1. Pure starting-layout interface. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\cube_layout.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\cube_layout.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/constants.h"

namespace wpiemu::plugins::game_2027 {
std::array<s::Vec3, 30> allianceCubePositions();

}  // namespace wpiemu::plugins::game_2027
```

Listing 7.2. Fence, pyramid and portal positions. <code>seasons\2027\game\src\cube_layout.cpp</code>.

<!-- book-file: seasons\2027\game\src\cube_layout.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/cube_layout.h"

namespace wpiemu::plugins::game_2027 {
std::array<s::Vec3, 30> allianceCubePositions() {
    std::array<s::Vec3, 30> result{};
    const float groundZ = kCubeHalfM.z + 0.002f;
    for (std::size_t i = 0; i < 6; ++i)
        result[i] = {kSwitchXM + 0.7112f + 0.025f + kCubeHalfM.x + 0.02f,
                     kWidthM / 2.f - 1.625f + 0.65f * static_cast<float>(i), groundZ};

    // Manual 4.1.2 / Figure 4-2: a 6/3/1 pyramid, with its broad face toward the
    // switch.
    const float pitch = 2.f * kCubeHalfM.x;
    const float frontX = kSwitchXM - 0.7112f - 0.025f - kCubeHalfM.x - 0.02f;
    std::size_t next = 6;
    for (int tier = 0; tier < 3; ++tier) {
        const int rows = 3 - tier;
        for (int row = 0; row < rows; ++row) {
            const int count = rows - row;
            for (int column = 0; column < count; ++column)
                result[next++] = {
                    frontX - pitch * (static_cast<float>(row) + 0.5f * static_cast<float>(tier)),
                    kWidthM / 2.f +
                        pitch * (static_cast<float>(column) - 0.5f * static_cast<float>(count - 1)),
                    groundZ + 2.f * kCubeHalfM.z * static_cast<float>(tier)};
        }
    }

    // Human-player supply is outside the opposite alliance wall, not loose on the
    // carpet.
    for (std::size_t portal = 0; portal < 2; ++portal)
        for (std::size_t i = 0; i < 7; ++i) {
            const float y = 0.25f + 0.35f * static_cast<float>(i % 4);
            result[16 + portal * 7 + i] = {kLengthM + 0.35f + 0.35f * static_cast<float>(i / 4),
                                           portal == 0 ? y : kWidthM - y, groundZ};
        }
    return result;
}

}  // namespace wpiemu::plugins::game_2027
```

`CubePool` keeps body identifiers separate from location states. `held_` is an
optional index: an empty optional means the robot has no cube. The lowest
eligible index wins pickup, giving a stable tie-breaking rule.

Listing 7.3. Pool state and custody operations. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\cube_pool.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\cube_pool.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/body_set.h"
#include "wpiemu/plugins/game_2027/cube_layout.h"

namespace wpiemu::plugins::game_2027 {

enum class CubeLocation { Field, Held, Reserve, Exchange, Vault, Removed };

class CubePool {
public:
    explicit CubePool(s::IPhysicsWorld& world) : world_(world), bodies_(world) {}
    s::Status build();
    void reset(bool preload, std::size_t alliance);
    std::size_t count(CubeLocation location, std::size_t alliance) const;
    std::vector<s::Vec3> positions(CubeLocation location) const;
    bool pickup(const s::Pose3& zone, const s::Vec3& half);
    bool release(const s::Pose3& pose, const s::Vec3& velocity);
    bool feed(std::size_t alliance, const s::Pose3& pose);
    bool exchange(std::size_t alliance, const s::Vec3& center, const s::Vec3& half);
    bool deposit(std::size_t alliance);
    int inventory() const { return held_ ? 1 : 0; }
    int released() const { return released_; }

private:
    bool contains(std::size_t i, const s::Pose3& pose, const s::Vec3& half) const;
    void move(std::size_t i, CubeLocation location);
    s::IPhysicsWorld& world_;
    Bodies bodies_;
    std::array<s::BodyId, kCubeCount> cubes_{};
    std::array<CubeLocation, kCubeCount> locations_{};
    std::array<std::size_t, kCubeCount> custodians_{};
    std::optional<std::size_t> held_;
    int released_ = 0;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 7.4. Allocation, reset and custody transitions. <code>seasons\2027\game\src\cube_pool.cpp</code>.

<!-- book-file: seasons\2027\game\src\cube_pool.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/cube_pool.h"

namespace wpiemu::plugins::game_2027 {

s::Status CubePool::build() {
    for (std::size_t i = 0; i < cubes_.size(); ++i) {
        auto body = bodies_.box({{0.f, 0.f, 0.5f}, {}}, kCubeHalfM, s::BodyType::Dynamic,
                                s::layer::Fuel, kCubeTag + i, {0.5f, 0.f, 0.f});
        if (!body) return body.error();
        cubes_[i] = body.value();
    }
    return s::okStatus();
}

void CubePool::reset(bool preload, std::size_t alliance) {
    held_.reset();
    released_ = 0;
    const auto layout = allianceCubePositions();
    for (std::size_t i = 0; i < cubes_.size(); ++i) {
        const auto n = i % 30;
        auto p = layout[n];
        if (i >= 30) p = {kLengthM - p.x, kWidthM - p.y, p.z};
        custodians_[i] = i / 30;
        move(i, n < 16 ? CubeLocation::Field : CubeLocation::Reserve);
        world_.setPose(cubes_[i], {p, {}});
        world_.setLinearVelocity(cubes_[i], {});
        world_.setAngularVelocity(cubes_[i], {});
    }
    if (preload) {
        held_ = alliance * 30 + 29;
        move(*held_, CubeLocation::Held);
    }
}

void CubePool::move(std::size_t i, CubeLocation location) {
    locations_[i] = location;
    world_.setEnabled(cubes_[i], location == CubeLocation::Field);
}

std::size_t CubePool::count(CubeLocation location, std::size_t alliance) const {
    std::size_t count = 0;
    for (std::size_t i = 0; i < cubes_.size(); ++i)
        if (locations_[i] == location && custodians_[i] == alliance) ++count;
    return count;
}

std::vector<s::Vec3> CubePool::positions(CubeLocation location) const {
    std::vector<s::Vec3> result;
    for (std::size_t i = 0; i < cubes_.size(); ++i)
        if (locations_[i] == location) result.push_back(world_.pose(cubes_[i]).position);
    return result;
}

bool CubePool::contains(std::size_t i, const s::Pose3& pose, const s::Vec3& half) const {
    const auto p = pose.inverse().transformPoint(world_.pose(cubes_[i]).position);
    return std::abs(p.x) <= half.x && std::abs(p.y) <= half.y && std::abs(p.z) <= half.z;
}

bool CubePool::pickup(const s::Pose3& pose, const s::Vec3& half) {
    if (held_) return false;
    for (std::size_t i = 0; i < cubes_.size(); ++i) {
        if (locations_[i] != CubeLocation::Field || !contains(i, pose, half)) continue;
        held_ = i;
        move(i, CubeLocation::Held);
        return true;
    }
    return false;
}

bool CubePool::release(const s::Pose3& pose, const s::Vec3& velocity) {
    if (!held_) return false;
    const auto i = *held_;
    move(i, CubeLocation::Field);
    world_.setPose(cubes_[i], pose);
    world_.setLinearVelocity(cubes_[i], velocity);
    world_.setAngularVelocity(cubes_[i], {});
    held_.reset();
    ++released_;
    return true;
}

bool CubePool::feed(std::size_t alliance, const s::Pose3& pose) {
    for (std::size_t i = 0; i < cubes_.size(); ++i) {
        if (custodians_[i] != alliance || locations_[i] != CubeLocation::Reserve) continue;
        move(i, CubeLocation::Field);
        world_.setPose(cubes_[i], pose);
        world_.setLinearVelocity(cubes_[i], {});
        world_.setAngularVelocity(cubes_[i], {});
        return true;
    }
    return false;
}

bool CubePool::exchange(std::size_t alliance, const s::Vec3& center, const s::Vec3& half) {
    for (std::size_t i = 0; i < cubes_.size(); ++i) {
        if (locations_[i] != CubeLocation::Field || !contains(i, {center, {}}, half)) continue;
        custodians_[i] = alliance;
        move(i, CubeLocation::Exchange);
        return true;
    }
    return false;
}

bool CubePool::deposit(std::size_t alliance) {
    for (std::size_t i = 0; i < cubes_.size(); ++i) {
        if (custodians_[i] != alliance || locations_[i] != CubeLocation::Exchange) continue;
        move(i, CubeLocation::Vault);
        return true;
    }
    return false;
}

}  // namespace wpiemu::plugins::game_2027
```

Release supplies a world pose and velocity. The pool does not decide how the
robot computes them. Exchange, feed and deposit are custody primitives;
their eligibility and timing rules are developed in Chapters 16 and 17.

Listing 7.5. Library sources through Chapter 7. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp
    src/field_layout.cpp
    src/cube_layout.cpp
    src/cube_pool.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 7.6 Verification

The custody test counts every location, including disabled bodies, before and
after a release, pickup, exchange and reset. The stable-layout integration test
in Chapter 12 advances the actual field for five seconds and bounds each cube's
displacement by 0.04 m. That tolerance is a model acceptance criterion, not a
measurement from the manual.

Listing 7.6. Conservation through custody transfers. <code>tests\game_2027\test_pool.cpp</code>.

<!-- book-file: tests\game_2027\test_pool.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2027/cube_pool.h"
namespace pu = wpiemu::plugins::game_2027;
namespace s = wpiemu::sdk;

TEST_CASE("Power UP: pool conserves cubes across all custody transfers") {
    auto world = wpiemu::core::physics::createBox3dWorld();
    pu::CubePool pool(*world);
    REQUIRE(pool.build());
    pool.reset(true, 0);
    auto total = [&] {
        std::size_t sum = 0;
        for (std::size_t a = 0; a < 2; ++a)
            for (auto state :
                 {pu::CubeLocation::Field, pu::CubeLocation::Held, pu::CubeLocation::Reserve,
                  pu::CubeLocation::Exchange, pu::CubeLocation::Vault, pu::CubeLocation::Removed})
                sum += pool.count(state, a);
        return sum;
    };
    CHECK(total() == 60);
    CHECK(pool.inventory() == 1);
    const s::Pose3 pose{{2.f, 2.f, 0.5f}, {}};
    REQUIRE(pool.release(pose, {}));
    CHECK_FALSE(pool.release(pose, {}));
    REQUIRE(pool.pickup(pose, {0.2f, 0.2f, 0.2f}));
    REQUIRE(pool.release(pose, {}));
    REQUIRE(pool.exchange(1, pose.position, {0.2f, 0.2f, 0.2f}));
    REQUIRE(pool.deposit(1));
    CHECK(pool.count(pu::CubeLocation::Vault, 1) == 1);
    REQUIRE(pool.feed(0, pose));
    CHECK(total() == 60);
    pool.reset(false, 0);
    CHECK(total() == 60);
    CHECK(pool.positions(pu::CubeLocation::Field).size() == 32);
    CHECK(pool.positions(pu::CubeLocation::Reserve).size() == 28);
}
```

## 7.7 Reproduction

Extract through Chapter 7, rebuild and run:

```powershell
ctest --test-dir build\book -R "Power UP: pool conserves" --no-tests=error --output-on-failure
```

## 7.8 Discussion

Rows F-02 through F-04 identify the staging and preload interpretation. The
layout is a pure calculation, while stability depends on materials, solver
settings and the field. This is why both kinds of tests are necessary.

## 7.9 Summary

Cubes now have stable identities and explicit custody. Reset reuses all bodies
and clears their velocities. Game actions cannot create a cube merely by
incrementing a counter.

## 7.10 Exercises

1. Count the cubes in a four-layer triangular pyramid.
2. Extend the conservation test with a second reset and compare all positions.
3. Explain why off-field reserves must not be eligible for ordinary pickup.

[Chapter 8](08-plates.md)
