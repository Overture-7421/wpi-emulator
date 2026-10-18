#pragma once

#include <algorithm>
#include <map>
#include <set>

#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::sdk {

using MotionTemplateLookup = std::function<const MechanismTemplateInfo*(std::string_view)>;

inline std::vector<MotionAxisInfo> motionAxisInfos(const RobotConfig& config,
                                                   const MotionTemplateLookup& lookup) {
    std::vector<MotionAxisInfo> out;
    for (const auto& wheel : config.drivetrain.spec.wheels) {
        const SlotAddress address{slot_owner::kDrivetrain, driveSlotName(wheel.name, 0)};
        MotionAxisInfo item{address, PlantKind::Angular, {}};
        for (int i = 0; i < std::clamp(wheel.driveMotorCount, 1, kMaxMotorsPerAxis); ++i)
            item.motorSlots.push_back({slot_owner::kDrivetrain, driveSlotName(wheel.name, i)});
        out.push_back(std::move(item));
    }
    for (const auto& mechanism : config.mechanisms) {
        const auto* info = lookup(mechanism.type);
        if (info == nullptr) continue;
        for (const auto& axis : info->motionAxes) {
            if (!axis.enabledProperty.empty()) {
                const auto* enabled = mechanism.properties.find(axis.enabledProperty);
                if (enabled != nullptr && !enabled->asBool()) continue;
            }
            MotionAxisInfo item{{mechanism.id, axis.name}, axis.kind, {}};
            if (!axis.motionProperty.empty()) {
                const auto* motion = mechanism.properties.find(axis.motionProperty);
                if (motion != nullptr && motion->asString() == "linear")
                    item.kind = PlantKind::Linear;
            }
            for (const auto& motor : axis.motorSlots)
                item.motorSlots.push_back({mechanism.id, motor});
            out.push_back(std::move(item));
        }
    }
    return out;
}

inline const MotionLink* motionLinkTo(const RobotConfig& config, const SlotAddress& target) {
    for (const auto& link : config.motionLinks)
        if (link.target == target) return &link;
    return nullptr;
}

inline bool motionFollowerMotor(const RobotConfig& config, const std::vector<MotionAxisInfo>& axes,
                                const SlotAddress& slot) {
    for (const auto& axis : axes)
        if (motionLinkTo(config, axis.address) != nullptr &&
            std::find(axis.motorSlots.begin(), axis.motorSlots.end(), slot) !=
                axis.motorSlots.end())
            return true;
    return false;
}

inline std::vector<Issue> motionLinkIssues(const RobotConfig& config,
                                           const std::vector<MotionAxisInfo>& axes) {
    std::vector<Issue> out;
    std::set<std::string> ids;
    std::map<SlotAddress, SlotAddress> sources;
    const auto known = [&](const SlotAddress& address) {
        return std::any_of(axes.begin(), axes.end(),
                           [&](const auto& axis) { return axis.address == address; });
    };
    const auto error = [&](const std::string& path, const std::string& message) {
        out.push_back({Severity::Error, path, message, "motion_link_invalid"});
    };
    for (const auto& link : config.motionLinks) {
        const std::string path = "motionLinks." + link.id;
        if (link.id.empty() || !ids.insert(link.id).second)
            error(path, "Motion link IDs must be non-empty and unique.");
        if (!known(link.source) || !known(link.target))
            error(path, "Select existing, supported source and target axes.");
        if (!std::isfinite(link.ratio) || link.ratio == 0.0)
            error(path, "The motion ratio must be finite and non-zero.");
        if (!sources.emplace(link.target, link.source).second)
            error(path, "An axis can follow only one source.");
    }
    for (const auto& [target, source] : sources) {
        std::set<SlotAddress> visited{target};
        SlotAddress next = source;
        for (;;) {
            if (!visited.insert(next).second) {
                error("motionLinks", "Motion links must not contain cycles or self-links.");
                break;
            }
            const auto found = sources.find(next);
            if (found == sources.end()) break;
            next = found->second;
        }
    }
    for (const auto& mapping : config.allMappings())
        if (motionFollowerMotor(config, axes, mapping.slot))
            error(mapping.slot.toString(),
                  "This axis follows another axis. Remove its motor mapping or its motion link.");
    return out;
}

// Upgrade only the ordinary tank layout; never discard a real controller mapping.
inline Result<std::vector<MotionLink>> tankMotionLinks(const RobotConfig& config) {
    std::vector<MotionLink> links;
    for (const int side : {1, -1}) {
        std::vector<const WheelSpec*> wheels;
        const WheelSpec* source = nullptr;
        for (const auto& wheel : config.drivetrain.spec.wheels) {
            if (!std::isfinite(wheel.positionM.y) || std::abs(wheel.positionM.y) < 1e-6f)
                return makeError(ErrorCode::InvalidArgument,
                                 "Tank motion links need wheels clearly on the left or right.");
            if ((wheel.positionM.y > 0.f ? 1 : -1) != side) continue;
            wheels.push_back(&wheel);
            for (const auto& mapping : config.drivetrain.mappings) {
                bool ownMotor = false;
                for (int i = 0; i < kMaxMotorsPerAxis && !ownMotor; ++i)
                    ownMotor = mapping.slot == SlotAddress{slot_owner::kDrivetrain,
                                                           driveSlotName(wheel.name, i)};
                if (ownMotor) {
                    if (source != nullptr && source != &wheel)
                        return makeError(
                            ErrorCode::InvalidArgument,
                            "Several wheels on one tank side have motor mappings. Choose a source "
                            "and remove conflicting mappings before linking.");
                    source = &wheel;
                }
            }
        }
        if (wheels.empty())
            return makeError(ErrorCode::InvalidArgument,
                             "Tank motion links need wheels on both sides.");
        if (source == nullptr) source = wheels.front();
        for (const auto* wheel : wheels)
            if (wheel != source)
                links.push_back({wheel->name + "_drive",
                                 {slot_owner::kDrivetrain, source->name + ".drive"},
                                 {slot_owner::kDrivetrain, wheel->name + ".drive"},
                                 1.0});
    }
    return links;
}

}  // namespace wpiemu::sdk
