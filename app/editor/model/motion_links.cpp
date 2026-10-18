#include "model/motion_links.h"

#include <algorithm>
#include <cmath>

#include "model/config_ops.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::editor {

namespace {

constexpr std::string_view kDriveSuffix = ".drive";

bool sameOwner(const SelectionRef& a, const SelectionRef& b) { return a.kind == b.kind && a.owner == b.owner && a.sub == b.sub; }

double unitFactor(sdk::PlantKind kind) { return kind == sdk::PlantKind::Angular ? sdk::kTwoPi : 1.0; }

}  // namespace

MotionIndex::MotionIndex(const sdk::RobotConfig& config, const EditorCatalog& catalog)
    : config_(config),
      axes_(sdk::motionAxisInfos(config, [&](std::string_view type) { return catalog.findTemplate(type); })) {}

const sdk::MotionAxisInfo* MotionIndex::axis(const sdk::SlotAddress& address) const {
    for (const auto& a : axes_)
        if (a.address == address) return &a;
    return nullptr;
}

const sdk::MotionLink* MotionIndex::incoming(const sdk::SlotAddress& axis) const { return sdk::motionLinkTo(config_, axis); }

SelectionRef MotionIndex::ownerOf(const sdk::SlotAddress& axis) {
    if (axis.owner == sdk::slot_owner::kDrivetrain) {
        std::string name = axis.slot;
        if (name.size() > kDriveSuffix.size() && name.compare(name.size() - kDriveSuffix.size(), kDriveSuffix.size(), kDriveSuffix) == 0)
            name.resize(name.size() - kDriveSuffix.size());
        return SelectionRef::wheel(std::move(name));
    }
    return SelectionRef::mechanism(axis.owner);
}

std::vector<sdk::MotionAxisInfo> MotionIndex::axesOf(const SelectionRef& owner) const {
    std::vector<sdk::MotionAxisInfo> out;
    for (const auto& a : axes_)
        if (sameOwner(ownerOf(a.address), owner)) out.push_back(a);
    return out;
}

bool MotionIndex::isFollowerMotor(const sdk::SlotAddress& slot) const { return sdk::motionFollowerMotor(config_, axes_, slot); }

OwnerMotion MotionIndex::ownerMotion(const SelectionRef& owner) const {
    OwnerMotion out;
    out.owner = owner;
    out.axes = axesOf(owner);
    bool anyMotor = false;
    bool allFollow = true;
    for (const auto& a : out.axes) {
        const sdk::MotionLink* link = incoming(a.address);
        if (link != nullptr) {
            out.incoming.push_back(link);
            const SelectionRef source = ownerOf(link->source);
            if (!out.primarySource && !sameOwner(source, owner)) out.primarySource = source;
        }
        if (!a.motorSlots.empty()) {
            anyMotor = true;
            if (link == nullptr) allFollow = false;
        }
    }
    out.followsAll = anyMotor && allFollow;
    for (const auto& link : config_.motionLinks) {
        const SelectionRef from = ownerOf(link.source);
        const SelectionRef to = ownerOf(link.target);
        if (!sameOwner(from, owner) || sameOwner(to, owner)) continue;
        if (std::none_of(out.drives.begin(), out.drives.end(), [&](const SelectionRef& d) { return sameOwner(d, to); }))
            out.drives.push_back(to);
    }
    return out;
}

std::string applyDrivenBy(sdk::RobotConfig& config, const EditorCatalog& catalog, const sdk::SlotAddress& target,
                          const sdk::SlotAddress& source, double displayRatio) {
    const MotionIndex before(config, catalog);
    const sdk::MotionAxisInfo* to = before.axis(target);
    const sdk::MotionAxisInfo* from = before.axis(source);
    if (to == nullptr || from == nullptr) return "Select existing, supported source and follower axes.";
    if (!std::isfinite(displayRatio) || displayRatio == 0.0) return "The motion ratio must be finite and non-zero.";
    if (target == source) return "An axis cannot follow itself.";

    sdk::RobotConfig candidate = config;
    sdk::MotionLink link;
    if (const sdk::MotionLink* existing = before.incoming(target)) link.id = existing->id;
    for (int n = 1; link.id.empty(); ++n) {
        const std::string id = "link" + std::to_string(n);
        if (std::none_of(candidate.motionLinks.begin(), candidate.motionLinks.end(), [&](const auto& l) { return l.id == id; })) link.id = id;
    }
    link.source = source;
    link.target = target;
    link.ratio = ratioToSi(displayRatio, from->kind, to->kind);
    std::erase_if(candidate.motionLinks, [&](const auto& l) { return l.id == link.id || l.target == target; });
    candidate.motionLinks.push_back(link);
    for (const auto& motor : to->motorSlots) ops::clearMapping(candidate, motor);

    // Only problems this edit introduces block it; unrelated existing ones stay for the validation list.
    const auto known = sdk::motionLinkIssues(config, before.axes());
    const auto after = sdk::motionLinkIssues(candidate, MotionIndex(candidate, catalog).axes());
    for (const auto& issue : after) {
        const bool existed = std::any_of(known.begin(), known.end(), [&](const sdk::Issue& k) { return k.path == issue.path && k.message == issue.message; });
        if (!existed) return issue.message;
    }
    config = std::move(candidate);
    return {};
}

void removeDrivenBy(sdk::RobotConfig& config, const sdk::SlotAddress& target) {
    std::erase_if(config.motionLinks, [&](const auto& l) { return l.target == target; });
}

std::string axisLabel(const sdk::RobotConfig& config, const EditorCatalog& catalog, const sdk::SlotAddress& axis) {
    if (axis.owner == sdk::slot_owner::kDrivetrain) return MotionIndex::ownerOf(axis).sub;
    const sdk::MechanismConfig* m = config.findMechanism(axis.owner);
    if (m == nullptr) return axis.toString();
    const std::string name = ops::displayNameFor(*m, catalog);
    const auto* info = catalog.findTemplate(m->type);
    return info != nullptr && info->motionAxes.size() > 1 ? name + " " + axis.slot : name;
}

double ratioFromSi(double siRatio, sdk::PlantKind source, sdk::PlantKind target) {
    return siRatio * unitFactor(source) / unitFactor(target);
}

double ratioToSi(double displayRatio, sdk::PlantKind source, sdk::PlantKind target) {
    return displayRatio * unitFactor(target) / unitFactor(source);
}

std::string ratioUnits(sdk::PlantKind source, sdk::PlantKind target) {
    const auto unit = [](sdk::PlantKind k) { return k == sdk::PlantKind::Angular ? "rev" : "m"; };
    return std::string(unit(target)) + " / " + unit(source);
}

}  // namespace wpiemu::app::editor
