#include <algorithm>

#include "model/config_ops.h"
#include "model/config_ops_internal.h"

namespace wpiemu::app::editor::ops {

using detail::onChassis;
using detail::reservedOwner;
using sdk::MechanismConfig;
using sdk::Pose3;
using sdk::RobotConfig;

// ---- zones ------------------------------------------------------------------------------------
std::string uniqueZoneId(const RobotConfig& c, std::string base) {
    return detail::uniqueEntityId(c, std::move(base), "zone");
}

std::string zoneDisplayName(const sdk::ZoneConfig& z, const EditorCatalog& catalog) {
    if (!z.label.empty()) return z.label;
    if (const auto* k = catalog.findZoneKind(z.kind)) return k->displayName;
    return z.id;
}

Pose3 zoneParentWorld(const RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneConfig& z) {
    return detail::attachmentParentWorld(c, catalog, z.attachment);
}

Pose3 zoneWorld(const RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneConfig& z) {
    return zoneParentWorld(c, catalog, z) * z.local;
}

sdk::ZoneConfig makeZone(const RobotConfig& c, const sdk::ZoneKindInfo& info, const sdk::MechanismAttachment& attachment) {
    sdk::ZoneConfig z;
    z.id = uniqueZoneId(c, baseIdForType(info.kind));
    z.kind = info.kind;
    z.label = info.displayName;
    for (const auto& other : c.zones)
        if (other.kind == info.kind && other.label == z.label) { z.label = info.displayName + " " + std::to_string(c.zones.size() + 1); break; }
    z.attachment.parentMechanismId = attachment.onChassis() ? std::string(sdk::kChassisParent) : attachment.parentMechanismId;
    z.attachment.parentLink = attachment.parentLink.empty() ? std::string(sdk::kRootLink) : attachment.parentLink;
    z.local = info.defaultLocal;
    z.sizeM = info.defaultSizeM;
    z.properties = sdk::defaultsOf(info.properties);
    return z;
}

std::string addZone(RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneKindInfo& info, const sdk::MechanismAttachment& attachment) {
    sdk::ZoneConfig z = makeZone(c, info, detail::sanitizeAttachment(c, catalog, attachment));
    std::string id = z.id;
    c.zones.push_back(std::move(z));
    return id;
}

bool removeZone(RobotConfig& c, std::string_view id) { return sdk::detail::eraseById(c.zones, id); }

std::string duplicateZone(RobotConfig& c, std::string_view id) {
    const sdk::ZoneConfig* original = c.findZone(id);
    if (original == nullptr) return {};
    sdk::ZoneConfig copy = *original;
    copy.id = uniqueZoneId(c, copy.id);
    copy.label = (copy.label.empty() ? original->id : copy.label) + " (copy)";
    copy.local.position.y += 0.1f;
    std::string newId = copy.id;
    c.zones.push_back(std::move(copy));
    return newId;
}

bool reattachZone(RobotConfig& c, const EditorCatalog& catalog, std::string_view id, const std::string& newParent,
                  const std::string& newLink, bool keepWorldPose) {
    sdk::ZoneConfig* z = c.findZone(id);
    return z != nullptr && detail::reattachItem(c, catalog, z->attachment, z->local, newParent, newLink, keepWorldPose);
}

int addZoneCondition(RobotConfig& c, std::string_view zoneId) {
    sdk::ZoneConfig* z = c.findZone(zoneId);
    if (z == nullptr) return -1;
    sdk::ZoneCondition cond;
    if (!c.mechanisms.empty()) cond.mechanismId = c.mechanisms.front().id;
    z->activation.conditions.push_back(std::move(cond));
    return static_cast<int>(z->activation.conditions.size()) - 1;
}

bool removeZoneCondition(RobotConfig& c, std::string_view zoneId, int index) {
    sdk::ZoneConfig* z = c.findZone(zoneId);
    if (z == nullptr || index < 0 || index >= static_cast<int>(z->activation.conditions.size())) return false;
    z->activation.conditions.erase(z->activation.conditions.begin() + index);
    return true;
}

bool setZoneCondition(RobotConfig& c, std::string_view zoneId, int index, const sdk::ZoneCondition& condition) {
    sdk::ZoneConfig* z = c.findZone(zoneId);
    if (z == nullptr || index < 0 || index >= static_cast<int>(z->activation.conditions.size())) return false;
    sdk::ZoneCondition& slot = z->activation.conditions[static_cast<std::size_t>(index)];
    if (slot == condition) return false;
    slot = condition;
    return true;
}

bool setZoneActivationMode(RobotConfig& c, std::string_view zoneId, sdk::ActivationMode mode) {
    sdk::ZoneConfig* z = c.findZone(zoneId);
    if (z == nullptr || z->activation.mode == mode) return false;
    z->activation.mode = mode;
    return true;
}

}  // namespace wpiemu::app::editor::ops
