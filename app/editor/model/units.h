#pragma once

// Display units for the inspector: the SDK's unit table (sdk/config/unit_display.h; persisted values are ALWAYS SI)
// plus the Qt suffix helper.

#include <QString>

#include "wpiemu/sdk/config/unit_display.h"

namespace wpiemu::app::editor {

using sdk::fromDisplay;
using sdk::toDisplay;
using sdk::UnitDisplay;
using sdk::unitDisplay;
using sdk::UnitSystem;

inline QString unitSuffix(sdk::PropertyUnit unit, UnitSystem system) { return QString::fromUtf8(unitDisplay(unit, system).suffix); }

}  // namespace wpiemu::app::editor
