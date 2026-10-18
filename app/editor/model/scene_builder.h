#pragma once

// Turns the robot model into a flat list of primitives for the 3D viewport (Z-up world, floor at
// z = 0). Pure data: the viewport model/QML only draw it, picking maps an item back to a selection.

#include <string>
#include <vector>

#include <QString>

#include "editor_backend.h"
#include "model/scene_theme.h"
#include "model/selection.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::editor {

enum class ItemShape { Box, Cylinder };
// Frustum = the edges and the far plane of a camera's view pyramid (not outlined or framed with the robot, picked only when nothing
// solid is under the cursor); Tag = a thin AprilTag plate (never picked).
enum class ItemRole { Hull, Wheel, Shape, Overlay, Link, Marker, Com, Frustum, Tag };

// Ids of the start pose marker's items: the footprint moves the pose, the nose (tip and shaft ids start with this) turns it.
inline constexpr const char* kStartPoseFootprintId = "startpose";
inline constexpr const char* kStartPoseNoseId = "startpose:nose";

struct SceneItem {
    std::string id;                 // unique and stable across rebuilds (diffing, QML delegates)
    SelectionRef ref;               // what a click selects (None = not pickable)
    SelectionRef group;             // entity the item belongs to (highlighted together, drill-down)
    ItemShape shape = ItemShape::Box;
    ItemRole role = ItemRole::Shape;
    sdk::Pose3 world;               // centre of the primitive; cylinders have their axis on local Z
    sdk::Vec3 size{0.1f, 0.1f, 0.1f};  // box: extents; cylinder: diameter, diameter, height
    QColor color{"#808080"};
    float opacity = 1.f;
    bool pickable = false;
    QString text;                   // Tag: the AprilTag id the viewport writes next to the plate
};

struct SceneContent {
    std::vector<SceneItem> items;
    const SceneItem* find(const std::string& id) const {
        for (const auto& i : items)
            if (i.id == id) return &i;
        return nullptr;
    }
};

// What else the scene shows besides the robot.
struct SceneOptions {
    bool showTags = false;      // the active AprilTag layout's tags, as the robot sees the field from its start pose
    std::string cameraId;       // the selected camera (may be empty): the tags it would see are tinted

    bool operator==(const SceneOptions&) const = default;
};

SceneContent buildScene(const sdk::RobotConfig& config, const EditorCatalog& catalog, const SceneTheme& theme, const SceneOptions& options = {});

// How far a camera's frustum is drawn: 10 % of the layout's field length (clamped to 1 .. 3 m), 1.5 m without a layout.
float cameraFrustumDepthM(const sdk::RobotConfig& config, const EditorCatalog& catalog);

// A geometric preview of what `camera` (its frame in the display frame) detects, by the rules of the vision engine without the line of
// sight: all four corners of the tag are in front of the lens and inside the image, the tag faces the camera within the maximum view
// angle and its shortest projected edge is at least the minimum number of pixels. `tagSizeM` is the edge length to project. Occlusion by
// the field is not considered: the run view shows the real detections.
bool cameraSeesTag(const sdk::CameraConfig& camera, const sdk::Pose3& cameraWorld, const sdk::Pose3& tagWorld, double tagSizeM);

// Rotation taking +X to `direction` (used for link lines).
sdk::Quat rotationFromX(const sdk::Vec3& direction);

}  // namespace wpiemu::app::editor
