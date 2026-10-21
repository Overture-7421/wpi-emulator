#pragma once

// Turns the robot model into a flat list of primitives for the 3D viewport (Z-up world, floor at
// z = 0). Pure data: the viewport model/QML only draw it, picking maps an item back to a selection.

#include <string>
#include <vector>

#include <QString>

#include "editor_backend.h"
#include "model/scene_theme.h"
#include "model/selection.h"
#include "ui/scene_support.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::editor {

enum class ItemRole { Hull, Wheel, Shape, Overlay, Link, Marker, Com };

struct SceneItem {
    std::string id;                 // unique and stable across rebuilds (diffing, QML delegates)
    SelectionRef ref;               // what a click selects (None = not pickable)
    SelectionRef group;             // entity the item belongs to (highlighted together, drill-down)
    sdk::PrimitiveKind shape = sdk::PrimitiveKind::Box;
    ItemRole role = ItemRole::Shape;
    sdk::Pose3 world;               // centre of the primitive; cylinders have their axis on local Z
    sdk::Vec3 size{0.1f, 0.1f, 0.1f};  // box: extents; cylinder: diameter, diameter, height
    QColor color = ui::defaultColor();
    float opacity = 1.f;
    bool pickable = false;
};

struct SceneContent {
    std::vector<SceneItem> items;
    const SceneItem* find(const std::string& id) const {
        for (const auto& i : items)
            if (i.id == id) return &i;
        return nullptr;
    }
};

SceneContent buildScene(const sdk::RobotConfig& config, const EditorCatalog& catalog, const SceneTheme& theme);

}  // namespace wpiemu::app::editor
