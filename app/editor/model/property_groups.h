#pragma once

// What the inspector shows for a selection, as plain data: a list of property groups, each a
// PropertySchema + the current values. Core data (chassis, wheels, ...) and template/plugin data
// are described the same way, so ONE generic form renders and edits everything and every edit takes
// the same undo path (applyProperty).

#include <QString>

#include <string>
#include <vector>

#include "editor_backend.h"
#include "model/selection.h"
#include "model/units.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::editor {

struct PropertyGroup {
    std::string id;                 // "chassis", "mech.placement", "mech.props", "panel:<panelId>", ...
    QString title;
    QString hint;                   // one-line help under the title
    sdk::PropertySchema schema;
    sdk::Value values;              // object keyed by PropertySpec::key, defaults applied
    bool collapsed = false;         // start collapsed
    sdk::PropertySection section = sdk::PropertySection::Configure;   // which inspector card shows it
    std::string axis;               // Drive groups: the motion axis ("axis", "deploy", "roller", "" = wheel drive)
};

// Ids of the groups the inspector places in the Drive card: "mech.drive:<axis>" (mechanisms) and "wheel.drive".
inline bool isDriveGroup(const std::string& id) { return id.rfind("mech.drive:", 0) == 0 || id == "wheel.drive"; }

// The selection the inspector actually edits: a slot edits its owner, an overlay its owner/plugin.
SelectionRef formTargetOf(const sdk::RobotConfig& c, const SelectionRef& sel);

std::vector<PropertyGroup> propertyGroupsFor(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel);

// Applies one edit made in group `groupId`. Returns false when nothing could be applied.
bool applyProperty(sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel, const std::string& groupId,
                   const std::string& key, const sdk::Value& value);

// What kind of mechanism this is: the label of its `motion` choice (Rotary / Arm / Linear) for a template that has one,
// otherwise the template's display name.
QString mechanismKindLabel(const sdk::MechanismConfig& m, const sdk::MechanismTemplateInfo& info);

// The camera's robot-to-camera transform in the WPILib robot frame (see ops::cameraRobotPose) as one line in the
// display units; `tooltip` (optional) receives the same pose in metres / radians, ready to copy into robot code.
QString robotToCameraText(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam, UnitSystem units,
                          QString* tooltip = nullptr);

// Short human description of the selection ("Intake (roller)", "Wheel fl", ...).
QString describeSelection(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel);

}  // namespace wpiemu::app::editor
