#pragma once

// Helpers shared by the config_ops_<domain>.cpp files (internal to app/editor/model; the public vocabulary is config_ops.h).

#include <string>
#include <string_view>
#include <utility>

#include "model/config_ops.h"
#include "wpiemu/sdk/config/mechanism_attachment.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/devices/slot.h"

namespace wpiemu::app::editor::ops::detail {

// The owner strings that address the chassis, the drivetrain and the robot slots: no entity id may take them.
inline bool reservedOwner(std::string_view id) { return sdk::isReservedEntityId(id); }

inline bool onChassis(const std::string& parent) { return parent.empty() || parent == sdk::kChassisParent; }

// First string in {base, base + sep + "2", base + sep + "3", ...} for which `taken` is false.
template <class Pred>
std::string uniqueSuffixed(std::string base, const char* fallback, const char* sep, Pred taken) {
    if (base.empty()) base = fallback;
    if (!taken(base)) return base;
    for (int n = 2;; ++n) {
        std::string candidate = base + sep + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}

// Unique id across all entity collections (mechanisms, zones, cameras, sensors) and reserved owner names.
inline std::string uniqueEntityId(const sdk::RobotConfig& c, std::string base, const char* fallback) {
    return uniqueSuffixed(std::move(base), fallback, "_",
                          [&](const std::string& id) { return reservedOwner(id) || c.hasEntityId(id); });
}

// World pose of the frame an attachment's local pose is expressed in (chassis frame when on the chassis).
inline sdk::Pose3 attachmentParentWorld(const sdk::RobotConfig& c, const EditorCatalog& catalog,
                                        const sdk::MechanismAttachment& att) {
    const sdk::Pose3 chassis = chassisFrameWorld(c.chassis);
    if (att.onChassis()) return chassis;
    if (const auto p = linkInChassis(c, catalog, att.parentMechanismId, att.parentLink)) return chassis * *p;
    return chassis;
}

// Falls back to the chassis root when `attachment` names an unknown mechanism or link.
inline sdk::MechanismAttachment sanitizeAttachment(const sdk::RobotConfig& c, const EditorCatalog& catalog,
                                                   const sdk::MechanismAttachment& attachment) {
    if (attachment.onChassis()) return {};
    const sdk::MechanismConfig* parent = c.findMechanism(attachment.parentMechanismId);
    const sdk::MechanismTemplateInfo* tpl = parent != nullptr ? catalog.findTemplate(parent->type) : nullptr;
    const std::string link = attachment.parentLink.empty() ? std::string(sdk::kRootLink) : attachment.parentLink;
    if (parent == nullptr || (tpl != nullptr && !sdk::hasLink(*tpl, link))) return {};
    return {attachment.parentMechanismId, link};
}

// Re-attaches an item (zone or camera) to `newParent` / `newLink`, optionally keeping its world pose.
inline bool reattachItem(const sdk::RobotConfig& c, const EditorCatalog& catalog, sdk::MechanismAttachment& att,
                         sdk::Pose3& local, const std::string& newParent, const std::string& newLink,
                         bool keepWorldPose) {
    const std::string link = newLink.empty() ? std::string(sdk::kRootLink) : newLink;
    if (onChassis(newParent)) {
        if (link != sdk::kRootLink) return false;
    } else {
        const sdk::MechanismConfig* parent = c.findMechanism(newParent);
        if (parent == nullptr) return false;
        if (const auto* info = catalog.findTemplate(parent->type); info != nullptr && !sdk::hasLink(*info, link))
            return false;
    }
    const sdk::Pose3 before = attachmentParentWorld(c, catalog, att) * local;
    att.parentMechanismId = onChassis(newParent) ? std::string(sdk::kChassisParent) : newParent;
    att.parentLink = link;
    if (keepWorldPose) local = attachmentParentWorld(c, catalog, att).inverse() * before;
    return true;
}

}  // namespace wpiemu::app::editor::ops::detail
