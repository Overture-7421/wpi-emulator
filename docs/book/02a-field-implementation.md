# Week 1 workshop: give the carpet an owner

**In 30 seconds:** this is the first actual plugin source, not pseudocode. We will build a small,
usable single-robot Power Up simulation, explaining each piece before adding it. It is a working
vertical slice, **not the full game manual**: vaults, power-ups, parking, climbing and ranking points
are still extension work.

## 1. Put the files in their pit

Work in a fresh source export as described in [Replay the book](appendix-replay.md). All destination
paths in this workshop are relative to that export, never your main checkout. There is no dependency
on the author's private scratch tree.

Create these two small files first. The CMake target is the thing the compiler builds; its public
include directory lets the aggregator see the registration declaration. Its only linked dependency
is the SDK. Neither Qt nor WPILib belongs in this target.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC src/game_module.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

The header exposes just one operation. The forward declaration avoids exposing the module's
implementation to whoever registers it. The namespace, folder, target and manifest will all say
the same thing: `game_2027`.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\plugin.h -->
```cpp
#pragma once

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::sdk {
class GamePluginRegistry;
}

namespace wpiemu::plugins::game_2027 {
sdk::Status registerPlugin(sdk::GamePluginRegistry& registry);
}
```

Now create <code>seasons\2027\game\src\game_module.cpp</code>. The marked C++ blocks from this
page through [module registration](04c-module-and-registration.md) concatenate, in reading order,
into **one complete file**. Append each block once; do not close a class or namespace early.
Keeping the translation unit together makes ownership visible without sending you through a dozen
headers. Individual fragments are not standalone programs; the first build checkpoint comes after
the module is complete. The [replay helper](appendix-replay.md) can assemble the exact same blocks.

Start with the includes, private namespace and dimensions. `s` is simply a short name for the SDK.
Our field origin is the Blue wall's right-hand corner: +X toward Red, +Y left as Blue sees it,
and +Z up. A box takes **half** its full width, length and height.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/plugin.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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
namespace {

namespace s = sdk;
constexpr float kLengthM = 16.4592f;
constexpr float kWidthM = 8.2296f;
constexpr float kSwitchXM = 4.2672f;
constexpr float kPlateOffsetM = 1.3716f;
constexpr s::Vec3 kCubeHalfM{0.1651f, 0.1651f, 0.1397f};
constexpr s::Vec3 kPlateHalfM{0.6096f, 0.4572f, 0.025f};
constexpr std::uint64_t kCubeTag = 0x100000;
constexpr std::size_t kCubeCount = 60;
constexpr const char* kId = "game_2027";
constexpr const char* kPickup = "game_2027.pickup";
constexpr const char* kRelease = "game_2027.release";
```

## 2. Lay out a match, not a warehouse

Author one alliance's thirty starting positions; rotate them at reset for the other alliance.
The first six are a row on the switch's scale-facing fence. The small clearance keeps a cube from
starting inside the fence. These are nominal coordinates, not field-construction dimensions.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
std::array<s::Vec3, 30> allianceCubePositions() {
    std::array<s::Vec3, 30> result{};
    const float groundZ = kCubeHalfM.z + 0.002f;
    for (std::size_t i = 0; i < 6; ++i)
        result[i] = {kSwitchXM + 0.7112f + 0.025f + kCubeHalfM.x + 0.02f,
                     kWidthM / 2.f - 1.625f + 0.65f * static_cast<float>(i), groundZ};
```

Next is the ten-cube pyramid on the alliance-facing side. For the bottom tier, row lengths are
3, 2 and 1 (six cubes); the next tier has 2 and 1 (three); the top has one. Each higher tier is
shifted half a cube toward the narrower end so it has support below. The broad face is toward the
switch. Do not scatter the ten pieces across midfield and call the count close enough.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    // Manual 4.1.2 / Figure 4-2: a 6/3/1 pyramid, with its broad face toward the switch.
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
                    kWidthM / 2.f + pitch * (static_cast<float>(column) - 0.5f * static_cast<float>(count - 1)),
                    groundZ + 2.f * kCubeHalfM.z * static_cast<float>(tier)};
        }
    }
```

The remaining fourteen positions are **off-field** portal supply at the opposite alliance wall.
They will be drawn but disabled in physics until a human feed exists. This lesson does not
implement that feed. Preload accounting will take a cube from this supply, not manufacture one.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    // Human-player supply is outside the opposite alliance wall, not loose on the carpet.
    for (std::size_t portal = 0; portal < 2; ++portal)
        for (std::size_t i = 0; i < 7; ++i) {
            const float y = 0.25f + 0.35f * static_cast<float>(i % 4);
            result[16 + portal * 7 + i] = {
                kLengthM + 0.35f + 0.35f * static_cast<float>(i / 4),
                portal == 0 ? y : kWidthM - y, groundZ};
        }
    return result;
}
```

Our two scenario controls are a legal FMS assignment and whether the robot starts with one cube.
Schema defaults are editor-friendly data; validation still protects construction. This example
uses the SDK value accessors for two fields, rather than introducing a binding class.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
bool validAssignment(const std::string& value) {
    return value == "LLL" || value == "LRL" || value == "RLR" || value == "RRR";
}

s::PluginConfigSchema configSchema() {
    s::PluginConfigSchema schema;
    schema.schemaVersion = 1;
    schema.properties.properties = {
        s::props::choice("assignment", "Field assignment", "Scenario", "LRL",
                         {{"LLL", "LLL"}, {"LRL", "LRL"}, {"RLR", "RLR"}, {"RRR", "RRR"}}),
        s::props::flag("preload", "One preloaded cube", "Scenario", true),
    };
    return schema;
}
```

## 3. Own every body you create

A collider is not automatically a picture, and a picture is not automatically a collider.
`Box` records what both will need. `Bodies` is a small RAII owner: when it dies, it destroys every
body it created. This matters when building the fifth wall fails after the first four succeeded.
The borrowed world must outlive the owner; the runtime guarantees that lifetime.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
struct Box {
    std::string name;
    s::Pose3 pose;
    s::Vec3 sizeM;
    std::string color;
};

class Bodies {
public:
    explicit Bodies(s::IPhysicsWorld& world) : world_(world) {}
    ~Bodies() {
        for (const s::BodyId body : ids_) world_.destroyBody(body);
    }
```

Create the body, immediately record ownership, then add its shape. An invalid ID becomes an
explicit `Result` error. For a cube, density is mass divided by volume, with volume computed from
three half-extents. For static or kinematic bodies the shape density stays zero.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    s::Result<s::BodyId> box(const s::Pose3& pose, s::Vec3 half, s::BodyType type,
                            std::uint64_t category, std::uint64_t tag = 0,
                            s::Material material = s::materials::kFieldStructure) {
        s::BodyDesc desc;
        desc.pose = pose;
        desc.type = type;
        desc.userTag = tag;
        const s::BodyId body = world_.createBody(desc);
        if (!body.valid()) return s::makeError(s::ErrorCode::Internal, "Power UP body creation failed");
        ids_.push_back(body);
        s::ShapeDesc shape;
        shape.geometry = s::BoxShape{half};
        shape.category = category;
        shape.mask = s::layer::All;
        shape.material = material;
        shape.density = type == s::BodyType::Dynamic ? 1.5f / (8.f * half.x * half.y * half.z) : 0.f;
```

Tags tell a plate sensor whether it found one of our cubes. Sensor events must be enabled on both
the sensor and visiting shapes. Destroying a body destroys its shapes, so one ownership list is
enough. We do not catch an engine failure and pretend an invisible wall was built.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
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
```

## 4. Pour the carpet and build the fence

`Field` owns things that stay put. Its description is read-only presentation data for the app.
The carpet's top is Z=0; the solid extends downward. A plugin replaces the fallback field,
so omitting this floor would let everything fall forever. The wall boxes sit outside the legal
rectangle, leaving the full interior available.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
class Field final : public s::IField {
public:
    explicit Field(s::IPhysicsWorld& world) : bodies_(world) {}

    s::Status build() {
        description_.name = "FIRST POWER UP (temporary 2027 prototype)";
        description_.lengthM = kLengthM;
        description_.widthM = kWidthM;
        description_.driverStations = s::genericDriverStations(kLengthM, kWidthM);
        std::vector<Box> boxes{
            {"carpet", {{kLengthM / 2.f, kWidthM / 2.f, -0.10f}, {}}, {kLengthM, kWidthM, 0.20f}, "#555962"},
            {"blue_wall", {{-0.05f, kWidthM / 2.f, 0.60f}, {}}, {0.10f, kWidthM, 1.20f}, "#2457c5"},
            {"red_wall", {{kLengthM + 0.05f, kWidthM / 2.f, 0.60f}, {}}, {0.10f, kWidthM, 1.20f}, "#c52c30"},
            {"left_rail", {{kLengthM / 2.f, -0.05f, 0.25f}, {}}, {kLengthM, 0.10f, 0.50f}, "#9199a0"},
            {"right_rail", {{kLengthM / 2.f, kWidthM + 0.05f, 0.25f}, {}}, {kLengthM, 0.10f, 0.50f}, "#9199a0"},
        };
```

Each switch gets two long fence faces and two end faces. The scale gets a simplified central tower
and platforms. There are no detailed ramps or climb rungs in this first playable slice. We do not
pretend that a decorative tower makes the climber implemented.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        for (const float x : {kSwitchXM, kLengthM - kSwitchXM}) {
            for (const float dx : {-0.7112f, 0.7112f})
                boxes.push_back({"switch_fence", {{x + dx, kWidthM / 2.f, 0.24f}, {}},
                                 {0.05f, 3.90f, 0.48f}, "#a0a9b0"});
            for (const float dy : {-1.95f, 1.95f})
                boxes.push_back({"switch_end", {{x, kWidthM / 2.f + dy, 0.24f}, {}},
                                 {1.47f, 0.05f, 0.48f}, "#a0a9b0"});
        }
        boxes.push_back({"scale_tower", {{kLengthM / 2.f, kWidthM / 2.f, 1.05f}, {}},
                         {0.30f, 0.30f, 2.10f}, "#a0a9b0"});
        for (const float x : {6.90f, kLengthM - 6.90f})
            boxes.push_back({"platform", {{x, kWidthM / 2.f, 0.045f}, {}},
                             {1.05f, 2.64f, 0.09f}, "#9b9b9b"});
```

One loop builds both the collider and visual from the same pose and dimensions. The carpet uses
the SDK carpet material; rigid structure uses its shared slicker material. This avoids a robot
sticking to a fence because its friction was accidentally copied from the floor.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        for (const Box& box : boxes) {
            auto body = bodies_.box(box.pose, box.sizeM * 0.5f, s::BodyType::Static, s::layer::Field, 0,
                                    box.name == "carpet" ? s::materials::kCarpet : s::materials::kFieldStructure);
            if (!body) return body.error();
            s::FieldVisual visual;
            visual.name = box.name;
            visual.pose = box.pose;
            visual.sizeM = box.sizeM;
            visual.color = box.color;
            description_.visuals.push_back(std::move(visual));
        }
```

The auto lines and off-field portal pads are visuals only. Giving painted tape a collider would
make it a speed bump. The portal pads explain where the reserve is drawn, but they do not let
robots pass through the alliance wall. Finally, expose the description and close the field class.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
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

    const s::FieldDescription& description() const override { return description_; }

private:
    Bodies bodies_;
    s::FieldDescription description_;
};
```

**Inspection:** the shape is built before we publish its visual. If construction fails, the
factory will return that error and the RAII owner cleans up. An editor screenshot alone cannot
prove this; later we destroy the field and assert that no bodies remain.

**You now have:** the complete static field implementation and a deterministic starting-position
function. Keep the source file open.

Next: [understand the piece states and clock](03-rules-and-robot.md), then write the reset workshop.
