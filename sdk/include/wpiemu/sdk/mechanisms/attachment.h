#pragma once

// Mechanism attachment contract (ADR 0006).
//
// A mechanism's mount frame hangs on a named LINK of its parent: another mechanism or the chassis.
//   pose_in_chassis(m.root) = pose_in_chassis(parent.link) * m.mount
//   pose_in_chassis(m.link) = pose_in_chassis(m.root) * m.linkPose(link)
// Every mechanism has the stable link "root" (kRootLink, identity in its own frame); the chassis has
// only "root" (the chassis frame). Other links come from MechanismTemplateInfo::links and move with the
// mechanism's current state (IMechanism::linkPose).
//
// Validation contract (IConfigValidator MUST call validateMechanismAttachments and reject on Errors):
//   attachment_parent_missing  parent mechanism id does not exist
//   attachment_link_missing    parent has no such link (chassis: anything but "root"; empty link)
//   attachment_cycle           the parent chain returns to the mechanism (self-parenting included)
//   mechanism_id_reserved      a mechanism uses the id "chassis"
// Runtime safety net for configs that slipped through: resolveLinkPoseInChassis returns nullopt for a
// broken chain; the runtime then mounts the mechanism on the chassis root and raises a warning. It never
// recurses without bound.

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/config/validation_codes.h"
#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::sdk {

// Pose of `link` relative to mechanism `mechanismId`'s ROOT frame, or nullopt when unknown. Runtimes
// implement it over live mechanisms (IMechanism::linkPose, falling back to the template's LinkSpec::local).
using LinkPoseSource = std::function<std::optional<Pose3>(std::string_view mechanismId, std::string_view link)>;

// Pose of `link` of `mechanismId` in the chassis frame, composed along the attachment chain.
// nullopt: unknown mechanism/link, missing parent, bad parent link, or a cycle.
inline std::optional<Pose3> resolveLinkPoseInChassis(const RobotConfig& config, std::string_view mechanismId,
                                                     std::string_view link, const LinkPoseSource& linkPoseOf) {
    std::vector<const MechanismConfig*> chain;  // target first, then its parent, ... up to the chassis
    const MechanismConfig* cur = config.findMechanism(mechanismId);
    while (cur != nullptr) {
        for (const MechanismConfig* seen : chain)
            if (seen == cur) return std::nullopt;  // cycle
        chain.push_back(cur);
        const MechanismAttachment att = cur->attachment;
        if (att.onChassis()) {
            if (att.parentLink != kRootLink) return std::nullopt;
            break;
        }
        cur = config.findMechanism(att.parentMechanismId);
        if (cur == nullptr) return std::nullopt;  // missing parent
    }
    if (chain.empty()) return std::nullopt;

    Pose3 rootInChassis = chain.back()->mount;
    for (std::size_t i = chain.size() - 1; i-- > 0;) {
        const MechanismConfig& parent = *chain[i + 1];
        const MechanismConfig& child = *chain[i];
        const std::optional<Pose3> parentLink = linkPoseOf(parent.id, child.attachment.parentLink);
        if (!parentLink) return std::nullopt;
        rootInChassis = rootInChassis * *parentLink * child.mount;
    }
    const std::optional<Pose3> own = linkPoseOf(chain.front()->id, link);
    if (!own) return std::nullopt;
    return rootInChassis * *own;
}

inline std::vector<Issue> validateMechanismAttachments(const RobotConfig& config, const MechanismFactory& factory) {
    std::vector<Issue> out;
    for (std::size_t i = 0; i < config.mechanisms.size(); ++i) {
        const MechanismConfig& m = config.mechanisms[i];
        const std::string base = "mechanisms[" + std::to_string(i) + "]";
        if (m.id == kChassisParent)
            out.push_back({Severity::Error, base + ".id", "'chassis' is reserved and cannot be a mechanism id",
                           validation_code::kMechanismIdReserved});

        const MechanismAttachment att = m.attachment;
        const std::string path = base + ".attachment";
        if (att.parentLink.empty()) {
            out.push_back({Severity::Error, path, "parent link must not be empty (use 'root')",
                           validation_code::kAttachmentLinkMissing});
        } else if (att.onChassis()) {
            if (att.parentLink != kRootLink)
                out.push_back({Severity::Error, path, "the chassis only exposes the link 'root', not '" + att.parentLink + "'",
                               validation_code::kAttachmentLinkMissing});
            continue;
        }
        if (att.onChassis()) continue;

        const MechanismConfig* parent = config.findMechanism(att.parentMechanismId);
        if (parent == nullptr) {
            out.push_back({Severity::Error, path, "parent mechanism '" + att.parentMechanismId + "' does not exist",
                           validation_code::kAttachmentParentMissing});
            continue;
        }
        if (const MechanismTemplateInfo* info = factory.templateInfo(parent->type);
            info != nullptr && !att.parentLink.empty() && !hasLink(*info, att.parentLink))
            out.push_back({Severity::Error, path,
                           "mechanism '" + parent->id + "' (" + parent->type + ") has no link '" + att.parentLink + "'",
                           validation_code::kAttachmentLinkMissing});

        // Cycle: walking up from `m` returns to `m`. Only members of the cycle are reported.
        const MechanismConfig* walk = parent;
        for (std::size_t steps = 0; walk != nullptr && steps <= config.mechanisms.size(); ++steps) {
            if (walk == &m) {
                out.push_back({Severity::Error, path, "attachment cycle through '" + m.id + "'",
                               validation_code::kAttachmentCycle});
                break;
            }
            const MechanismAttachment next = walk->attachment;
            walk = next.onChassis() ? nullptr : config.findMechanism(next.parentMechanismId);
        }
    }
    return out;
}

}  // namespace wpiemu::sdk
