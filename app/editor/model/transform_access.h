#pragma once

// Reading and writing the transform (pose + size) of whatever is selected, in the frame it is stored
// in. The viewport gizmos, the numeric transform bar and the scene builder all go through this, so a
// new selectable kind only needs to be taught here.

#include <optional>

#include "editor_backend.h"
#include "model/selection.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::editor {

enum class SizeKind { None, Box, Cylinder, Wheel };

struct EditableTransform {
    sdk::Pose3 local;                 // relative to the parent frame
    sdk::Vec3 size;                   // extents along the item's own axes (cylinder: d, d, h; wheel: d, width, d)
    SizeKind sizeKind = SizeKind::None;
    bool canMove = true;
    bool canRotate = true;
    bool canScale = false;
};

struct TransformTarget {
    EditableTransform transform;
    sdk::Pose3 parentWorld;           // frame `local` lives in; world pose = parentWorld * local
    sdk::Pose3 world() const { return parentWorld * transform.local; }
};

std::optional<TransformTarget> readTransform(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel);

// Writes `t` back (position/rotation/size as far as the target allows). false when not editable.
bool writeTransform(sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel, const EditableTransform& t);

}  // namespace wpiemu::app::editor
