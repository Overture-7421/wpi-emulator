#pragma once

// Helpers shared by the config_ops_<domain>.cpp files (internal to app/editor/model; the public vocabulary is config_ops.h).

#include <string>
#include <string_view>

#include "wpiemu/sdk/config/mechanism_attachment.h"
#include "wpiemu/sdk/devices/slot.h"

namespace wpiemu::app::editor::ops::detail {

// The owner strings that address the chassis, the drivetrain and the robot slots: no entity id may take them.
inline bool reservedOwner(std::string_view id) {
    return id == sdk::kChassisParent || id == sdk::slot_owner::kDrivetrain || id == sdk::slot_owner::kRobot;
}

inline bool onChassis(const std::string& parent) { return parent.empty() || parent == sdk::kChassisParent; }

}  // namespace wpiemu::app::editor::ops::detail
