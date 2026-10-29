# 11. Configuration and observability

## 11.1 Objectives

After this chapter the reader can declare typed plugin settings, describe a
HUD without Qt, and distinguish a snapshot from mutable simulation state.
Chapter 10 is the prerequisite. Configuration and descriptor files address R8
and R11.

## 11.2 Problem statement

The assignment and preload must be configurable without adding a custom editor.
Scores and cube positions must reach the run view without giving it access to
the physics world. Both tasks are handled by SDK data descriptions.

## 11.3 Background and definitions

**Definition 11.1 (descriptor).** A descriptor states what a generic application
component should display or edit. It is a standard C++ value, not a widget.
Qt is the application's UI framework; the plugin neither links it nor owns its
event loop.

Configuration follows decode, default, validate and construct. The core pipeline
validates the plugin schema before creating a session. The factory also rejects
an invalid assignment when called directly, which makes the SDK boundary safe
for tests and isolated previews.

## 11.4 Design

A custom dialog would couple the plugin to UI code. A second hand-written
decoder would duplicate schema keys. `ConfigBindings` instead binds property
specifications to typed members; the same table defines the schema and decoder.
The generic editor renders those properties.

**Invariant 11.1 (publication keys).** Every HUD and game-value descriptor names
a key that the game state publishes or returns. Descriptor IDs have the
plugin's namespace prefix.

## 11.5 Implementation

`GameConfig` gives missing properties defaults. The binding table names each
key once. The assignment remains a string because it is also the message sent
to the robot; `validAssignment` checks the four legal values.

Listing 11.1. Typed plugin configuration. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\config.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\config.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/constants.h"

namespace wpiemu::plugins::game_2027 {
struct GameConfig {
    std::string assignment = "LRL";
    bool preload = true;
};

bool validAssignment(const std::string& value);
s::PluginConfigSchema configSchema();
GameConfig parseConfig(const s::Value& data);

}  // namespace wpiemu::plugins::game_2027
```

Listing 11.2. Schema and decoding from shared bindings. <code>seasons\2027\game\src\config.cpp</code>.

<!-- book-file: seasons\2027\game\src\config.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/config.h"

#include "wpiemu/sdk/config/config_bindings.h"

namespace wpiemu::plugins::game_2027 {
namespace {

s::ConfigBindings<GameConfig> bindings() {
    s::ConfigBindings<GameConfig> b;
    b.add(s::props::choice("assignment", "Field assignment", "Scenario", "LRL",
                           {{"LLL", "LLL"}, {"LRL", "LRL"}, {"RLR", "RLR"}, {"RRR", "RRR"}}),
          &GameConfig::assignment);
    b.add(s::props::flag("preload", "One preloaded cube", "Scenario", true), &GameConfig::preload);
    return b;
}

}  // namespace

bool validAssignment(const std::string& value) {
    return value == "LLL" || value == "LRL" || value == "RLR" || value == "RRR";
}

s::PluginConfigSchema configSchema() {
    s::PluginConfigSchema schema;
    schema.schemaVersion = 1;
    schema.properties = bindings().schema();
    return schema;
}

GameConfig parseConfig(const s::Value& data) {
    return bindings().decode(data);
}

}  // namespace wpiemu::plugins::game_2027
```

The module is now declared separately from its implementation. This permits
metadata, game values and UI descriptors to occupy complete, short files.
The factory still returns an empty game state until Chapter 12.

Listing 11.3. The module's complete declaration. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_module.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\game_module.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/config.h"
#include "wpiemu/plugins/game_2027/field_layout.h"
#include "wpiemu/plugins/game_2027/plugin.h"

namespace wpiemu::plugins::game_2027 {
class Module final : public s::IGameModule {
public:
    s::GameMetadata metadata() const override;

    s::Status registerMechanisms(s::MechanismFactory&) override;

    s::Result<std::unique_ptr<s::IField>> createField(s::IPhysicsWorld& world) override;

    s::Result<std::unique_ptr<s::IGameState>> createGameState(s::SimContext& ctx) override;

    s::PluginConfigSchema contributeConfigSchema() const override;

    s::FieldFrame fieldFrame() const override;

    std::vector<s::ZoneKindInfo> contributeZoneKinds() const override;

    std::vector<s::GameValueInfo> contributeGameValues() const override;

    std::vector<s::HudWidgetDescriptor> contributeHud() const override;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 11.4. Descriptor-capable factories, revision 3. <code>seasons\2027\game\src\game_module.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

#include "wpiemu/sdk/game/null_objects.h"

namespace wpiemu::plugins::game_2027 {
s::Status Module::registerMechanisms(s::MechanismFactory&) {
    return s::okStatus();
}

s::Result<std::unique_ptr<s::IField>> Module::createField(s::IPhysicsWorld& world) {
    auto field = std::make_unique<Field>(world);
    const auto status = field->build();
    if (!status) return status.error();
    return std::unique_ptr<s::IField>(std::move(field));
}

s::Result<std::unique_ptr<s::IGameState>> Module::createGameState(s::SimContext& ctx) {
    (void)ctx;
    return std::unique_ptr<s::IGameState>(std::make_unique<s::NullGameState>(kId));
}

s::PluginConfigSchema Module::contributeConfigSchema() const {
    return configSchema();
}

}  // namespace wpiemu::plugins::game_2027

namespace wpiemu::plugins::game_2027 {
WPIEMU_REGISTER_GAME_PLUGIN(Module)

}  // namespace wpiemu::plugins::game_2027
```

Listing 11.5. Identity and field frame. <code>seasons\2027\game\src\plugin_info.cpp</code>.

<!-- book-file: seasons\2027\game\src\plugin_info.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

namespace wpiemu::plugins::game_2027 {
s::GameMetadata Module::metadata() const {
    s::GameMetadata result;
    result.id = kId;
    result.displayName = "FIRST POWER UP (2018 rules, 2027 ABI)";
    result.year = 2027;
    result.description =
        "Temporary single-robot prototype: physical cubes and "
        "plates, ownership scoring. "
        "Not a complete manual implementation: no vault, "
        "power-ups or assisted climbs.";
    return result;
}

s::FieldFrame Module::fieldFrame() const {
    return {kLengthM, kWidthM, s::FieldSymmetry::Rotational};
}

}  // namespace wpiemu::plugins::game_2027
```

`contributeHud` describes a scoreboard and counters. A `valueKey` is a lookup
into the snapshot, not a pointer to a score variable. The held-cube game value
is also available as a source for core's generic sensors.

Listing 11.6. HUD and zone descriptors. <code>seasons\2027\game\src\game_ui.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_ui.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

namespace wpiemu::plugins::game_2027 {
std::vector<s::HudWidgetDescriptor> Module::contributeHud() const {
    s::HudWidgetDescriptor scores;
    scores.id = "game_2027.scoreboard";
    scores.title = "POWER UP";
    scores.kind = s::HudWidgetKind::Scoreboard;
    scores.anchor = s::HudAnchor::TopCenter;
    scores.valueKey = "game_2027.phase";
    scores.sides = {{"BLUE", "game_2027.blue_score", "", "#2457c5"},
                    {"RED", "game_2027.red_score", "", "#c52c30"}};

    s::HudWidgetDescriptor inventory;
    inventory.id = "game_2027.inventory";
    inventory.title = "Held cube";
    inventory.valueKey = inventory.id;
    s::HudWidgetDescriptor assignment;
    assignment.id = "game_2027.game_data";
    assignment.title = "FMS";
    assignment.kind = s::HudWidgetKind::Text;
    assignment.valueKey = assignment.id;

    s::HudWidgetDescriptor blueSupply;
    blueSupply.id = "game_2027.portal_blue";
    blueSupply.title = "Blue portal supply";
    blueSupply.valueKey = blueSupply.id;
    s::HudWidgetDescriptor redSupply;
    redSupply.id = "game_2027.portal_red";
    redSupply.title = "Red portal supply";
    redSupply.valueKey = redSupply.id;
    return {scores, inventory, assignment, blueSupply, redSupply};
}

std::vector<s::ZoneKindInfo> Module::contributeZoneKinds() const {
    s::ZoneKindInfo pickup;
    pickup.kind = kPickup;
    pickup.displayName = "Cube pickup";
    pickup.shape = s::ZoneShape::Volume;
    pickup.defaultLocal.position = {0.5f, 0.f, 0.f};
    pickup.defaultSizeM = {0.5f, 0.6f, 0.5f};
    s::ZoneKindInfo release;
    release.kind = kRelease;
    release.displayName = "Cube release";
    release.shape = s::ZoneShape::Muzzle;
    release.defaultLocal.position = {0.75f, 0.f, 0.75f};
    return {pickup, release};
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 11.7. The declared inventory sensor source. <code>seasons\2027\game\src\game_values.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_values.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

namespace wpiemu::plugins::game_2027 {
std::vector<s::GameValueInfo> Module::contributeGameValues() const {
    return {{"game_2027.inventory", "Held cube", "One when the robot holds a cube",
             s::PropertyUnit::Count}};
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 11.8. Library sources through Chapter 11. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp
    src/field_layout.cpp
    src/cube_layout.cpp
    src/cube_pool.cpp
    src/plate_build.cpp
    src/plate_rules.cpp
    src/plates.cpp
    src/match_clock.cpp
    src/cube_zones.cpp
    src/config.cpp
    src/plugin_info.cpp
    src/game_ui.cpp
    src/game_values.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 11.6 Verification

The core pipeline checks declared property types and enum options. The direct
factory rejection test in Chapter 12 additionally checks that invalid assignment
construction leaks no bodies. The assembled snapshot supplies the independent
key set for testing Invariant 11.1.

## 11.7 Reproduction

Extract through Chapter 11 and build the plugin and tests. Run the module
identity case and the repository boundary checker. Snapshot and direct-factory
tests become executable at the Chapter 12 assembly checkpoint.

## 11.8 Discussion

F-25 describes the entity-group rendering boundary. Position samples contain
no orientations; a physically rotated cube therefore appears axis-aligned.
The distinction follows from the snapshot type, not a coordinate conversion
inside the plugin.

## 11.9 Summary

Configuration and presentation use SDK values rather than application objects.
The module remains stateless. The next chapter supplies the live values and
ties their keys to these descriptors.

## 11.10 Exercises

1. Trace an assignment property from schema to the robot's game-specific message.
2. Add an assertion that every scoreboard side has a published numeric key.
3. Explain why an entity-group sample cannot display a cube's angular velocity.

[Chapter 12](12-assembly.md)
