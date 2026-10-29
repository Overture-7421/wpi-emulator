# 6. Field geometry

## 6.1 Objectives

After this chapter the reader can express a field in the NWU frame, construct
matching visuals and colliders, and manage body lifetime. Chapter 5 is the
prerequisite. The field files and real-physics test address R2 and R3.

## 6.2 Problem statement

A visible wall does not stop a robot unless it also has a collider. A collider
without a matching visual produces an obstacle the driver cannot interpret.
The field must also release its bodies before the physics world is destroyed.

## 6.3 Background and definitions

**Definition 6.1 (field description).** A field description is data for drawing
the static scene: dimensions, named visuals and driver-station viewpoints.
It does not itself create physics shapes.

The drawing dimensions and collision dimensions therefore meet in a common
`Box` record. Its pose is the centre of the box; `sizeM` contains full extents.
Chapter 3 explains why the physics shape receives half that vector.

## 6.4 Design

Separate hand-written visual and collider loops permit inconsistent dimensions.
A scene imported from a mesh could avoid some duplicated geometry, but would
require asset preparation unrelated to the SDK lesson. The selected design
lists boxes once and builds both representations in one loop.

**Invariant 6.1 (body ownership).** Each body created by a field belongs to its
owning body set. Destruction releases each body exactly once. Copying that set
is prohibited because copying identifiers does not copy physics objects.

## 6.5 Implementation

The dimension headers distinguish manual measurements from the simplified
collision model. Cube dimensions appear here because the field test drops a
cube; Chapter 7 develops its game-specific layout and custody.

Listing 6.1. Field dimensions and their provenance. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\field_dimensions.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\field_dimensions.h -->
```cpp
#pragma once

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::plugins::game_2027 {

inline constexpr float kLengthM = 16.4592f;      // [GM 3.2] 54 ft nominal.
inline constexpr float kWidthM = 8.2296f;        // [GM 3.2] 27 ft nominal.
inline constexpr float kSwitchXM = 4.2672f;      // [approx] switch centre, nominal drawing layout.
inline constexpr float kPlateOffsetM = 1.3716f;  // [approx] switch plate lateral centre.
inline constexpr sdk::Vec3 kPlateHalfM{0.6096f, 0.4572f, 0.025f};  // [approx] collision box.
inline constexpr float kAutoLineM = 3.048f;  // [GM 3.2] 10 ft from alliance wall.

}  // namespace wpiemu::plugins::game_2027
```

Listing 6.2. Nominal cube dimensions. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\cube_dimensions.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\cube_dimensions.h -->
```cpp
#pragma once

#include <cstddef>
#include <cstdint>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::plugins::game_2027 {

inline constexpr sdk::Vec3 kCubeHalfM{0.1651f, 0.1651f, 0.1397f};  // [GM 3.8] 13 x 13 x 11 in.
inline constexpr float kCubeMassKg = 1.5f;                         // [GM 3.8] approximate mass.
inline constexpr std::size_t kCubeCount = 60;        // [GM 4.1.2] 30 allocated per alliance.
inline constexpr std::uint64_t kCubeTag = 0x100000;  // [derived] local physics tag range.

}  // namespace wpiemu::plugins::game_2027
```

Listing 6.3. Shared SDK names and plugin identifiers. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\constants.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\constants.h -->
```cpp
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/plugins/game_2027/cube_dimensions.h"
#include "wpiemu/plugins/game_2027/field_dimensions.h"
#include "wpiemu/sdk/config/property_builders.h"
#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/game/zone_activation.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/physics/materials.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::plugins::game_2027 {
namespace s = sdk;
inline constexpr const char* kId = "game_2027";
inline constexpr const char* kPickup = "game_2027.pickup";
inline constexpr const char* kRelease = "game_2027.release";

}  // namespace wpiemu::plugins::game_2027
```

`Bodies::box` checks both creation operations. If shape creation fails, the
body remains owned and is released during unwinding of the failed factory.
`Box` carries presentation data without owning a body.

Listing 6.4. Noncopyable ownership of physics bodies. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\body_set.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\body_set.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/constants.h"

namespace wpiemu::plugins::game_2027 {
class Bodies {
public:
    explicit Bodies(s::IPhysicsWorld& world) : world_(world) {}
    Bodies(const Bodies&) = delete;
    Bodies& operator=(const Bodies&) = delete;
    ~Bodies() {
        for (const s::BodyId body : ids_) world_.destroyBody(body);
    }

    s::Result<s::BodyId> box(const s::Pose3& pose, s::Vec3 half, s::BodyType type,
                             std::uint64_t category, std::uint64_t tag = 0,
                             s::Material material = s::materials::kFieldStructure) {
        s::BodyDesc desc;
        desc.pose = pose;
        desc.type = type;
        desc.userTag = tag;
        const s::BodyId body = world_.createBody(desc);
        if (!body.valid())
            return s::makeError(s::ErrorCode::Internal, "Power UP body creation failed");
        ids_.push_back(body);
        s::ShapeDesc shape;
        shape.geometry = s::BoxShape{half};
        shape.category = category;
        shape.mask = s::layer::All;
        shape.material = material;
        shape.density =
            type == s::BodyType::Dynamic ? kCubeMassKg / (8.f * half.x * half.y * half.z) : 0.f;

        shape.userTag = tag;
        shape.sensorEvents = true;
        if (!world_.addShape(body, shape).valid())
            return s::makeError(s::ErrorCode::Internal, "Power UP shape creation failed");
        return body;
    }

private:
    s::IPhysicsWorld& world_;
    std::vector<s::BodyId> ids_;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 6.5. Shared collider and visual geometry. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\box.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\box.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/constants.h"

namespace wpiemu::plugins::game_2027 {
struct Box {
    std::string name;
    s::Pose3 pose;
    s::Vec3 sizeM;
    std::string color;
};

}  // namespace wpiemu::plugins::game_2027
```

The `Field` object retains the body set and immutable description. Its build
loop uses each box's pose unchanged for the collider and visual. The carpet
top is at Z=0 because its centre is 0.10 m below the surface.

Listing 6.6. Field ownership and public interface. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\field_layout.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\field_layout.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/body_set.h"
#include "wpiemu/plugins/game_2027/box.h"

namespace wpiemu::plugins::game_2027 {
class Field final : public s::IField {
public:
    explicit Field(s::IPhysicsWorld& world);

    s::Status build();

    const s::FieldDescription& description() const override;

private:
    Bodies bodies_;
    s::FieldDescription description_;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 6.7. Static field construction. <code>seasons\2027\game\src\field_layout.cpp</code>.

<!-- book-file: seasons\2027\game\src\field_layout.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/field_layout.h"

namespace wpiemu::plugins::game_2027 {
Field::Field(s::IPhysicsWorld& world) : bodies_(world) {}

s::Status Field::build() {
    description_.name = "FIRST POWER UP (2027 ABI case study)";
    description_.lengthM = kLengthM;
    description_.widthM = kWidthM;
    description_.driverStations = s::genericDriverStations(kLengthM, kWidthM);
    std::vector<Box> boxes{
        {"carpet",
         {{kLengthM / 2.f, kWidthM / 2.f, -0.10f}, {}},
         {kLengthM, kWidthM, 0.20f},
         "#555962"},
        {"blue_wall", {{-0.05f, kWidthM / 2.f, 0.60f}, {}}, {0.10f, kWidthM, 1.20f}, "#2457c5"},
        {"red_wall",
         {{kLengthM + 0.05f, kWidthM / 2.f, 0.60f}, {}},
         {0.10f, kWidthM, 1.20f},
         "#c52c30"},
        {"left_rail", {{kLengthM / 2.f, -0.05f, 0.25f}, {}}, {kLengthM, 0.10f, 0.50f}, "#9199a0"},
        {"right_rail",
         {{kLengthM / 2.f, kWidthM + 0.05f, 0.25f}, {}},
         {kLengthM, 0.10f, 0.50f},
         "#9199a0"},
    };

    for (const float x : {kSwitchXM, kLengthM - kSwitchXM}) {
        for (const float dx : {-0.7112f, 0.7112f})
            boxes.push_back({"switch_fence",
                             {{x + dx, kWidthM / 2.f, 0.24f}, {}},
                             {0.05f, 3.90f, 0.48f},
                             "#a0a9b0"});
        for (const float dy : {-1.95f, 1.95f})
            boxes.push_back({"switch_end",
                             {{x, kWidthM / 2.f + dy, 0.24f}, {}},
                             {1.47f, 0.05f, 0.48f},
                             "#a0a9b0"});
    }
    boxes.push_back({"scale_tower",
                     {{kLengthM / 2.f, kWidthM / 2.f, 1.05f}, {}},
                     {0.30f, 0.30f, 2.10f},
                     "#a0a9b0"});
    for (const float x : {6.90f, kLengthM - 6.90f})
        boxes.push_back(
            {"platform", {{x, kWidthM / 2.f, 0.045f}, {}}, {1.05f, 2.64f, 0.09f}, "#9b9b9b"});

    for (const Box& box : boxes) {
        auto body = bodies_.box(
            box.pose, box.sizeM * 0.5f, s::BodyType::Static, s::layer::Field, 0,
            box.name == "carpet" ? s::materials::kCarpet : s::materials::kFieldStructure);
        if (!body) return body.error();
        s::FieldVisual visual;
        visual.name = box.name;
        visual.pose = box.pose;
        visual.sizeM = box.sizeM;
        visual.color = box.color;
        description_.visuals.push_back(std::move(visual));
    }

    for (const float x : {3.048f, kLengthM - 3.048f}) {
        s::FieldVisual line;
        line.name = "auto_line";
        line.pose.position = {x, kWidthM / 2.f, 0.002f};
        line.sizeM = {0.05f, kWidthM, 0.004f};
        line.color = "#ffffff";
        description_.visuals.push_back(std::move(line));
    }
    for (const float x : {-0.6f, kLengthM + 0.6f})
        for (const float y : {0.775f, kWidthM - 0.775f}) {
            s::FieldVisual portal;
            portal.name = "off_field_portal_supply";
            portal.pose.position = {x, y, -0.015f};
            portal.sizeM = {1.f, 1.5f, 0.03f};
            portal.color = x < 0.f ? "#c52c30" : "#2457c5";
            description_.visuals.push_back(std::move(portal));
        }
    return s::okStatus();
}

const s::FieldDescription& Field::description() const {
    return description_;
}

}  // namespace wpiemu::plugins::game_2027
```

The module now creates `Field`. Its game state remains empty, preserving the
distinction between field construction and game rules. CMake adds only the
new field translation unit.

Listing 6.8. Field factory, revision 2. <code>seasons\2027\game\src\game_module.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
#include "wpiemu/sdk/game/game_module.h"

#include "wpiemu/plugins/game_2027/field_layout.h"
#include "wpiemu/plugins/game_2027/plugin.h"
#include "wpiemu/sdk/game/null_objects.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::plugins::game_2027 {
namespace s = sdk;

class Module final : public s::IGameModule {
public:
    s::GameMetadata metadata() const override {
        s::GameMetadata result;
        result.id = "game_2027";
        result.displayName = "FIRST POWER UP (2018 rules, 2027 ABI)";
        result.year = 2027;
        return result;
    }

    s::Status registerMechanisms(s::MechanismFactory&) override { return s::okStatus(); }

    s::Result<std::unique_ptr<s::IField>> createField(s::IPhysicsWorld& world) override {
        auto field = std::make_unique<Field>(world);
        const auto status = field->build();
        if (!status) return status.error();
        return std::unique_ptr<s::IField>(std::move(field));
    }

    s::Result<std::unique_ptr<s::IGameState>> createGameState(s::SimContext&) override {
        return std::unique_ptr<s::IGameState>(std::make_unique<s::NullGameState>("game_2027"));
    }

    s::PluginConfigSchema contributeConfigSchema() const override { return {}; }
};

WPIEMU_REGISTER_GAME_PLUGIN(Module)

}  // namespace wpiemu::plugins::game_2027
```

Listing 6.9. Library sources through Chapter 6. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp
    src/field_layout.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 6.6 Verification

The test gives a dynamic cube a velocity toward the alliance wall. Gravity and
contact must leave it above the carpet and inside the wall. This checks physical
behavior rather than comparing the model's dimensions with themselves.
The final body count tests Invariant 6.1.

Listing 6.10. Drop, wall contact and teardown. <code>tests\game_2027\test_field.cpp</code>.

<!-- book-file: tests\game_2027\test_field.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2027/field_layout.h"

namespace pu = wpiemu::plugins::game_2027;
namespace s = wpiemu::sdk;

TEST_CASE("Power UP: field stops a dropped cube and a cube moving into a wall") {
    auto world = wpiemu::core::physics::createBox3dWorld();
    REQUIRE(world);
    {
        pu::Field field(*world);
        REQUIRE(field.build());
        pu::Bodies bodies(*world);
        const auto cube =
            bodies.box({{1.f, 1.f, 1.f}, {}}, pu::kCubeHalfM, s::BodyType::Dynamic, s::layer::Fuel);
        REQUIRE(cube);
        world->setLinearVelocity(cube.value(), {-2.f, 0.f, 0.f});
        for (int i = 0; i < 1000; ++i) world->step(0.005f);
        const auto p = world->pose(cube.value()).position;
        CHECK(p.x >= pu::kCubeHalfM.x - 0.01f);
        CHECK(p.z == doctest::Approx(pu::kCubeHalfM.z).epsilon(0.05));
        CHECK(field.description().lengthM == pu::kLengthM);
    }
    CHECK(world->stats().bodies == 0);
}
```

## 6.7 Reproduction

Extract through Chapter 6 and rebuild the test target. Run:

```powershell
ctest --test-dir build\book -R "Power UP: field stops" --no-tests=error --output-on-failure
```

## 6.8 Discussion

Row F-01 records the geometry's fidelity. Rotational symmetry is an invariant
of the layout, not a reason to exchange X and Y or negate only one coordinate.
The same field-frame convention is used by robot start poses and zones.

## 6.9 Summary

Visuals and colliders now share one source of poses and dimensions. Body
ownership has an explicit lifetime and teardown test. No rule state depends
on Qt or on a graphics object.

## 6.10 Exercises

1. Derive the centre of a carpet box whose top remains at Z=0 after doubling
   its thickness.
2. Add a test of the opposite alliance wall without changing production geometry.
3. Identify which field dimensions require drawings rather than the scoring rules.

[Chapter 7](07-cubes.md)
