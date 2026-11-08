#pragma once

// Chassis-frame pose of a zone at REST (no live mechanism state), for robots whose mechanisms are not instantiated, such
// as the remote robots of a practice match (see game/robot_view.h). Pure functions of the zone and, for a zone hung on a
// mechanism link, of the robot config and a rest link source; nothing here reads a running session.

#include <optional>

#include "wpiemu/sdk/config/zone.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

namespace wpiemu::sdk {

// A zone attached to the chassis: pose_in_chassis = zone.local. nullopt when the zone hangs on a mechanism link, which
// cannot be resolved without a config (use the overload below).
inline std::optional<Pose3> restPoseOfZone(const ZoneConfig& zone) {
    if (!zone.attachment.onChassis()) return std::nullopt;
    return zone.local;
}

// General form: pose_in_chassis(zone) = pose_in_chassis(attachment parent link) * zone.local, with the parent link
// resolved by resolveLinkPoseInChassis (mechanisms/attachment.h) over `config` and `linkPoseOf` (link pose relative to
// its mechanism's root, e.g. the template's LinkSpec::local). nullopt for a broken chain, as that function documents.
inline std::optional<Pose3> restPoseOfZone(const RobotConfig& config, const ZoneConfig& zone,
                                           const LinkPoseSource& linkPoseOf) {
    if (zone.attachment.onChassis()) return zone.local;
    const std::optional<Pose3> parent =
        resolveLinkPoseInChassis(config, zone.attachment.parentMechanismId, zone.attachment.parentLink, linkPoseOf);
    if (!parent) return std::nullopt;
    return *parent * zone.local;
}

}  // namespace wpiemu::sdk
