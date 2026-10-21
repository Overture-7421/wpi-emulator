#include "model/config_ops.h"
#include "model/config_ops_internal.h"

namespace wpiemu::app::editor::ops {

using sdk::MechanismConfig;
using sdk::Pose3;
using sdk::RobotConfig;
using sdk::Vec3;

// ---- frames -----------------------------------------------------------------------------------
Pose3 chassisFrameWorld(const sdk::ChassisSpec& s) {
    return Pose3{Vec3{s.comOffsetM.x, s.comOffsetM.y, s.comHeightM}, sdk::Quat{}};
}

Pose3 hullWorld(const sdk::ChassisSpec& s) {
    return Pose3{Vec3{0.f, 0.f, s.groundClearanceM + s.sizeM.z * 0.5f}, sdk::Quat{}};
}

Pose3 wheelWorld(const sdk::ChassisSpec& chassis, const sdk::WheelSpec& wheel) {
    return Pose3{Vec3{chassis.comOffsetM.x + wheel.positionM.x, chassis.comOffsetM.y + wheel.positionM.y, wheel.positionM.z},
                 sdk::Quat{}};
}

sdk::LinkPoseSource restLinkPoses(const RobotConfig& c, const EditorCatalog& catalog) {
    return [&c, &catalog](std::string_view id, std::string_view link) -> std::optional<Pose3> {
        if (link == sdk::kRootLink) return Pose3{};
        const MechanismConfig* m = c.findMechanism(id);
        if (m == nullptr) return std::nullopt;
        const sdk::MechanismTemplateInfo* info = catalog.findTemplate(m->type);
        if (info == nullptr) return std::nullopt;
        for (const auto& l : info->links)
            if (l.name == link) return l.local;
        return std::nullopt;
    };
}

std::optional<Pose3> linkInChassis(const RobotConfig& c, const EditorCatalog& catalog, std::string_view id,
                                   std::string_view link) {
    return sdk::resolveLinkPoseInChassis(c, id, link, restLinkPoses(c, catalog));
}

Pose3 mechanismRootWorld(const RobotConfig& c, const EditorCatalog& catalog, std::string_view id) {
    const Pose3 chassis = chassisFrameWorld(c.chassis);
    if (const auto p = linkInChassis(c, catalog, id)) return chassis * *p;
    if (const MechanismConfig* m = c.findMechanism(id)) return chassis * m->mount;
    return chassis;
}

Pose3 mechanismParentWorld(const RobotConfig& c, const EditorCatalog& catalog, const MechanismConfig& m) {
    return detail::attachmentParentWorld(c, catalog, m.attachment);
}

}  // namespace wpiemu::app::editor::ops
