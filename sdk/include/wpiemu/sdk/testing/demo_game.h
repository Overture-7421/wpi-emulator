#pragma once

// The one neutral demo game behind the in-process fakes (the run view's FakeRunBackend and the editor's
// FakeEditorBackend), so both describe the same plugin id, zone kinds, game values, field, tags and HUD.
// It is not a season: season data belongs to a game plugin. Every id carries the plugin prefix, like a real
// plugin's (ownsPluginId).

#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/config/property_builders.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/game_value.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/vision/apriltag.h"

namespace wpiemu::sdk::testing::demo_game {

inline constexpr const char* kPluginId = "demo_game";
inline constexpr const char* kDisplayName = "Demo game";

inline constexpr const char* kPickupZone = "demo_game.pickup";
inline constexpr const char* kShooterZone = "demo_game.shooter";

inline constexpr const char* kHopperPanel = "demo_game.hopper";
inline constexpr const char* kAimingPanel = "demo_game.aiming";

// Game values; the fuel ones double as HUD keys.
inline constexpr const char* kFuelCount = "demo_game.fuel.count";
inline constexpr const char* kFuelCapacity = "demo_game.fuel.capacity";
inline constexpr const char* kFuelFraction = "demo_game.fuel.fraction";

// HUD-only keys.
inline constexpr const char* kHubActive = "demo_game.hub.active";
inline constexpr const char* kHubBlueStatus = "demo_game.hub.blue.status";
inline constexpr const char* kHubRedStatus = "demo_game.hub.red.status";
inline constexpr const char* kScoreBlue = "demo_game.score.blue";
inline constexpr const char* kScoreRed = "demo_game.score.red";
inline constexpr const char* kMatchTime = "demo_game.match_time";
inline constexpr const char* kPhase = "demo_game.phase";
inline constexpr const char* kMessage = "demo_game.message";

inline constexpr float kFieldLengthM = 16.46f;
inline constexpr float kFieldWidthM = 8.23f;
inline constexpr float kHubXM[2] = {4.6f, 11.86f};   // blue, red
inline constexpr float kHubYM = 4.115f;

inline FieldFrame fieldFrame() { return {kFieldLengthM, kFieldWidthM, FieldSymmetry::Rotational}; }

inline PluginConfigSchema configSchema() {
    PluginConfigSchema schema;
    schema.schemaVersion = 1;
    schema.properties.properties = {
        props::integer("preloadFuel", "Preloaded fuel", "Robot", 8, 0, 100, "Fuel the robot starts a match with."),
        props::real("fuelDiameterM", "Fuel diameter", "Fuel", 0.15, 0.05, 0.4, PropertyUnit::Meters, {}, 0.005)};
    return schema;
}

inline std::vector<ZoneKindInfo> zoneKinds() {
    using props::integer;
    using props::real;

    ZoneKindInfo pickup;
    pickup.kind = kPickupZone;
    pickup.displayName = "Pickup";
    pickup.description = "Absorbs field fuel inside its volume while it is active.";
    pickup.shape = ZoneShape::Volume;
    pickup.properties.properties = {integer("capacity", "Capacity (fuel)", "Pickup", 30, 1, 100),
                                    real("absorbRateHz", "Absorb rate", "Pickup", 10.0, 0.5, 60.0, PropertyUnit::Hertz, {}, 0.5)};
    pickup.defaultSizeM = {0.4f, 0.6f, 0.25f};
    pickup.defaultLocal.position = {0.3f, 0.f, 0.f};
    pickup.color = "#22c55e";

    ZoneKindInfo shooter;
    shooter.kind = kShooterZone;
    shooter.displayName = "Shooter";
    shooter.description = "Launches fuel from its muzzle along +X while it is active.";
    shooter.shape = ZoneShape::Muzzle;
    shooter.properties.properties = {real("fireRateHz", "Fire rate", "Shooter", 8.0, 0.5, 30.0, PropertyUnit::Hertz, {}, 0.5),
                                     real("exitSpeedMps", "Exit speed", "Shooter", 10.0, 1.0, 30.0, PropertyUnit::MetersPerSecond, {}, 0.5),
                                     real("spreadRad", "Spread", "Shooter", 0.05, 0.0, 0.5, PropertyUnit::Radians, {}, 0.01)};
    shooter.defaultLocal.position = {0.1f, 0.f, 0.f};
    shooter.color = "#f43f5e";
    return {pickup, shooter};
}

inline std::vector<GameValueInfo> gameValues() {
    return {{kFuelCount, "Fuel in robot", "Fuel currently held by the robot.", PropertyUnit::Count},
            {kFuelCapacity, "Fuel capacity", "How much fuel the robot can hold.", PropertyUnit::Count},
            {kFuelFraction, "Fuel fill", "Held fuel divided by capacity (0 to 1).", PropertyUnit::Ratio}};
}

// A plugin-config panel with a viewport overlay, and a mechanism-scope panel for `aimedMechanismType` (the editor's
// sample shooter): only mechanisms of that type get it.
inline std::vector<EditorPanelDescriptor> editorPanels(const std::string& aimedMechanismType) {
    Box3 hopper;
    hopper.pose.position = {-0.05f, 0.f, 0.f};
    hopper.sizeM = {0.4f, 0.5f, 0.3f};

    EditorPanelDescriptor hopperPanel;
    hopperPanel.id = kHopperPanel;
    hopperPanel.title = "Fuel hopper";
    hopperPanel.scope = PanelScope::PluginConfig;
    hopperPanel.properties.properties = {
        props::integer("capacity", "Fuel capacity", "Hopper", 40, 1, 200, "The fuel meter shows this as 100%."),
        props::box("hopperVolume", "Hopper volume", "Hopper", hopper, "Where stored fuel is drawn (chassis frame).")};
    hopperPanel.overlays.push_back({"hopperVolume", "Hopper", "#44cc88", true});
    hopperPanel.order = 1;

    EditorPanelDescriptor aiming;
    aiming.id = kAimingPanel;
    aiming.title = "Aiming";
    aiming.scope = PanelScope::Mechanism;
    aiming.mechanismTypes = {aimedMechanismType};
    aiming.properties.properties = {props::real("hoodAngleRad", "Hood angle", "Aiming", 0.7, 0.0, 1.5, PropertyUnit::Radians, {}, 0.01)};
    aiming.order = 2;
    return {hopperPanel, aiming};
}

inline FieldDescription field() {
    constexpr float length = kFieldLengthM;
    constexpr float width = kFieldWidthM;
    FieldDescription f;
    f.name = "Demo field";
    f.lengthM = length;
    f.widthM = width;
    auto add = [&](const char* name, FieldVisual::Kind kind, Vec3 pos, Vec3 size, const char* color) {
        FieldVisual v;
        v.name = name;
        v.kind = kind;
        v.pose.position = pos;
        v.sizeM = size;
        v.color = color;
        f.visuals.push_back(std::move(v));
    };
    using K = FieldVisual::Kind;
    add("zone.blue", K::Box, {1.5f, width / 2, 0.001f}, {3.0f, width, 0.002f}, "#1e3a8a");
    add("zone.red", K::Box, {length - 1.5f, width / 2, 0.001f}, {3.0f, width, 0.002f}, "#7f1d1d");
    add("line.center", K::Box, {length / 2, width / 2, 0.002f}, {0.06f, width, 0.002f}, "#e2e8f0");
    add("wall.south", K::Box, {length / 2, -0.05f, 0.3f}, {length + 0.2f, 0.1f, 0.6f}, "#475569");
    add("wall.north", K::Box, {length / 2, width + 0.05f, 0.3f}, {length + 0.2f, 0.1f, 0.6f}, "#475569");
    add("wall.blue", K::Box, {-0.05f, width / 2, 0.45f}, {0.1f, width, 0.9f}, "#334155");
    add("wall.red", K::Box, {length + 0.05f, width / 2, 0.45f}, {0.1f, width, 0.9f}, "#334155");
    add("hub.blue", K::Cylinder, {kHubXM[0], kHubYM, 0.9f}, {1.2f, 1.2f, 1.8f}, "#2563eb");
    add("hub.red", K::Cylinder, {kHubXM[1], kHubYM, 0.9f}, {1.2f, 1.2f, 1.8f}, "#dc2626");
    add("bump.a", K::Box, {length / 2 - 2.2f, 1.6f, 0.08f}, {1.4f, 1.4f, 0.16f}, "#64748b");
    add("bump.b", K::Box, {length / 2 + 2.2f, width - 1.6f, 0.08f}, {1.4f, 1.4f, 0.16f}, "#64748b");
    return f;
}

// Eight tags on the field's walls.
inline AprilTagLayout aprilTags() {
    constexpr float length = kFieldLengthM;
    constexpr float width = kFieldWidthM;
    AprilTagLayout layout;
    layout.name = "demo";
    layout.fieldLengthM = length;
    layout.fieldWidthM = width;
    auto add = [&](int id, Vec3 pos, float yaw) {
        AprilTag t;
        t.id = id;
        t.pose = {pos, Quat::fromYaw(yaw)};
        layout.tags.push_back(t);
    };
    // Blue wall (+X out of the face points into the field), red wall, then the two long walls.
    add(1, {0.f, 2.6f, 1.2f}, 0.f);
    add(2, {0.f, kHubYM, 1.2f}, 0.f);
    add(3, {0.f, 5.6f, 1.2f}, 0.f);
    add(4, {length, 2.6f, 1.2f}, kPiF);
    add(5, {length, kHubYM, 1.2f}, kPiF);
    add(6, {length, 5.6f, 1.2f}, kPiF);
    add(7, {5.0f, 0.f, 1.0f}, kPiF * 0.5f);
    add(8, {11.5f, width, 1.0f}, -kPiF * 0.5f);
    return layout;
}

inline std::vector<HudWidgetDescriptor> hud() {
    auto widget = [](const char* id, const char* title, HudWidgetKind kind, const char* key, HudAnchor anchor, int order) {
        HudWidgetDescriptor d;
        d.id = id;
        d.title = title;
        d.kind = kind;
        d.valueKey = key;
        d.anchor = anchor;
        d.order = order;
        return d;
    };
    auto fuel = widget("demo_game.fuel", "Fuel", HudWidgetKind::Meter, kFuelCount, HudAnchor::BottomCenter, 0);
    fuel.maxKey = kFuelCapacity;
    fuel.color = "#facc15";
    auto timer = widget("demo_game.match", "Match", HudWidgetKind::Timer, kMatchTime, HudAnchor::TopCenter, 1);
    auto hub = widget("demo_game.hub", "Hub", HudWidgetKind::Badge, kHubActive, HudAnchor::TopCenter, 2);
    auto scoreboard = widget("demo_game.scoreboard", "Scoreboard", HudWidgetKind::Scoreboard, kPhase, HudAnchor::TopCenter, 0);
    scoreboard.sides = {{"BLUE", kScoreBlue, kHubBlueStatus, "#1a73e8"}, {"RED", kScoreRed, kHubRedStatus, "#d93025"}};
    auto message = widget("demo_game.message", "Game data", HudWidgetKind::Text, kMessage, HudAnchor::TopRight, 1);
    return {fuel, scoreboard, timer, hub, message};
}

}  // namespace wpiemu::sdk::testing::demo_game
