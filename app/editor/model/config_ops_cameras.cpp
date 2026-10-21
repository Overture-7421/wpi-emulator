#include <algorithm>

#include "model/config_ops.h"
#include "model/config_ops_internal.h"

namespace wpiemu::app::editor::ops {

using detail::onChassis;
using detail::reservedOwner;
using sdk::MechanismConfig;
using sdk::Pose3;
using sdk::RobotConfig;
using sdk::Vec3;

namespace {

const char* cameraKindName(sdk::CameraKind kind) { return kind == sdk::CameraKind::PhotonVision ? "PhotonVision" : "Limelight"; }

const char* cameraBaseId(sdk::CameraKind kind) { return kind == sdk::CameraKind::PhotonVision ? "photon" : "limelight"; }

bool nameTaken(const RobotConfig& c, sdk::CameraKind kind, const std::string& name, std::string_view exceptId) {
    return std::any_of(c.cameras.begin(), c.cameras.end(), [&](const sdk::CameraConfig& other) {
        return other.kind == kind && other.name == name && other.id != exceptId;
    });
}

}  // namespace

// ---- cameras ----------------------------------------------------------------------------------
std::string uniqueCameraId(const RobotConfig& c, std::string base) {
    return detail::uniqueEntityId(c, std::move(base), "camera");
}

std::string cameraDisplayName(const sdk::CameraConfig& cam) {
    if (!cam.label.empty()) return cam.label;
    return cameraKindName(cam.kind) + std::string(" (") + cam.id + ")";
}

Pose3 cameraParentWorld(const RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    return detail::attachmentParentWorld(c, catalog, cam.attachment);
}

Pose3 cameraWorld(const RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    return cameraParentWorld(c, catalog, cam) * cam.local;
}

Pose3 cameraRobotPose(const RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    return cameraWorld(c, catalog, cam);
}

std::string uniqueCameraName(const RobotConfig& c, sdk::CameraKind kind, std::string base, std::string_view exceptId) {
    if (!nameTaken(c, kind, base, exceptId)) return base;
    return detail::uniqueSuffixed(base.empty() ? std::string("cam") : std::move(base), "cam", "",
                                  [&](const std::string& candidate) { return nameTaken(c, kind, candidate, exceptId); });
}

sdk::CameraConfig makeCamera(const RobotConfig& c, sdk::CameraKind kind, const sdk::MechanismAttachment& attachment) {
    sdk::CameraConfig cam;
    cam.kind = kind;
    cam.id = uniqueCameraId(c, cameraBaseId(kind));
    cam.label = cameraKindName(kind);
    int sameKind = 0;
    for (const auto& other : c.cameras) sameKind += other.kind == kind ? 1 : 0;
    if (sameKind > 0) cam.label += " " + std::to_string(sameKind + 1);
    cam.name = uniqueCameraName(c, kind, kind == sdk::CameraKind::PhotonVision ? "photon" : "");
    cam.attachment.parentMechanismId = attachment.onChassis() ? std::string(sdk::kChassisParent) : attachment.parentMechanismId;
    cam.attachment.parentLink = attachment.parentLink.empty() ? std::string(sdk::kRootLink) : attachment.parentLink;
    if (cam.attachment.onChassis()) {
        // Just behind the front face, high on the hull, looking forward (chassis frame = origin at the COM).
        const sdk::ChassisSpec& s = c.chassis;
        const Vec3 hullTopFront{s.sizeM.x * 0.5f - 0.05f, 0.f, s.groundClearanceM + s.sizeM.z * 0.8f};
        cam.local.position = hullTopFront - Vec3{s.comOffsetM.x, s.comOffsetM.y, s.comHeightM};
    }
    return cam;
}

std::string addCamera(RobotConfig& c, const EditorCatalog& catalog, sdk::CameraKind kind, const sdk::MechanismAttachment& attachment) {
    sdk::CameraConfig cam = makeCamera(c, kind, detail::sanitizeAttachment(c, catalog, attachment));
    std::string id = cam.id;
    c.cameras.push_back(std::move(cam));
    return id;
}

bool removeCamera(RobotConfig& c, std::string_view id) { return sdk::detail::eraseById(c.cameras, id); }

std::string duplicateCamera(RobotConfig& c, std::string_view id) {
    const sdk::CameraConfig* original = c.findCamera(id);
    if (original == nullptr) return {};
    sdk::CameraConfig copy = *original;
    copy.id = uniqueCameraId(c, copy.id);
    copy.label = (copy.label.empty() ? original->id : copy.label) + " (copy)";
    copy.name = uniqueCameraName(c, copy.kind, copy.name);
    copy.local.position.y += 0.1f;
    std::string newId = copy.id;
    c.cameras.push_back(std::move(copy));
    return newId;
}

bool reattachCamera(RobotConfig& c, const EditorCatalog& catalog, std::string_view id, const std::string& newParent,
                    const std::string& newLink, bool keepWorldPose) {
    sdk::CameraConfig* cam = c.findCamera(id);
    return cam != nullptr && detail::reattachItem(c, catalog, cam->attachment, cam->local, newParent, newLink, keepWorldPose);
}

}  // namespace wpiemu::app::editor::ops
