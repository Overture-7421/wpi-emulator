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
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::editor {

struct PropertyGroup {
    std::string id;                 // "chassis", "mech.placement", "mech.props", "panel:<panelId>", ...
    QString title;
    QString hint;                   // one-line help under the title
    sdk::PropertySchema schema;
    sdk::Value values;              // object keyed by PropertySpec::key, defaults applied
    bool collapsed = false;         // start collapsed
};

// The selection the inspector actually edits: a slot edits its owner, an overlay its owner/plugin.
SelectionRef formTargetOf(const sdk::RobotConfig& c, const SelectionRef& sel);

std::vector<PropertyGroup> propertyGroupsFor(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel);

// Applies one edit made in group `groupId`. Returns false when nothing could be applied.
bool applyProperty(sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel, const std::string& groupId,
                   const std::string& key, const sdk::Value& value);

// Short human description of the selection ("Intake (roller)", "Wheel fl", ...).
QString describeSelection(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel);

}  // namespace wpiemu::app::editor
