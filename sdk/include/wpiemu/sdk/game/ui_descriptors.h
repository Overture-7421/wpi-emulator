#pragma once

// Qt-free descriptors of what a game plugin wants shown. The UI renders them generically; a
// plugin NEVER includes Qt. Values come from ISimSnapshot::hud() under `valueKey`.

#include <string>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/value.h"

namespace wpiemu::sdk {

// ---- HUD ---------------------------------------------------------------------------------------
enum class HudWidgetKind {
    Counter,      // integer/number with label
    Meter,        // filled bar value/max (fuel meter): max from `maxKey` or `maxValue`
    Gauge,
    Text,         // string value
    Badge,        // boolean / enum state pill (hub active / inactive)
    Timer,        // seconds formatted mm:ss
};

enum class HudAnchor { TopLeft, TopCenter, TopRight, BottomLeft, BottomCenter, BottomRight, Center };

struct HudWidgetDescriptor {
    std::string id;           // unique, namespaced: "game_2026.fuel_meter"
    std::string title;
    HudWidgetKind kind = HudWidgetKind::Counter;
    std::string valueKey;     // HudValue::key
    std::string maxKey;       // optional HUD key for the dynamic maximum (Meter/Gauge)
    double minValue = 0.0;
    double maxValue = 1.0;    // used when maxKey is empty
    std::string unit;
    std::string color;        // "#rrggbb" or empty for theme default
    HudAnchor anchor = HudAnchor::TopLeft;
    int order = 0;            // within an anchor, ascending
};

// ---- editor panels ------------------------------------------------------------------------------
// A volume/point drawn in the 3D editor viewport and bound to a property of the panel.
struct ViewportOverlayDescriptor {
    std::string propertyKey;     // key in the panel's schema; Box3 / Vec3 / Pose3
    std::string label;
    std::string color = "#4488ff";
    bool editable = true;        // gizmo-editable in the viewport
};

enum class PanelScope {
    PluginConfig,   // edits the plugin's section of robot.json (RobotConfig::plugins[pluginId])
    Mechanism,      // adds a section to the inspector of mechanisms whose type is in `mechanismTypes`
};

struct EditorPanelDescriptor {
    std::string id;                      // "game_2026.intake_zone"
    std::string title;
    PanelScope scope = PanelScope::PluginConfig;
    std::vector<std::string> mechanismTypes;   // scope == Mechanism
    PropertySchema properties;           // keys relative to the target (plugin section / mechanism properties)
    std::vector<ViewportOverlayDescriptor> overlays;
    int order = 0;
};

}  // namespace wpiemu::sdk
