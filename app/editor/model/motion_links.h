#pragma once

// Editor-side view of the robot's motion links (ADR 0012): which axes follow which, and what that means for
// the wheels and mechanisms the project tree and inspector show. Everything is keyed by axis address, so
// wheels (`<wheel>.drive`) and mechanism axes (`axis`, `deploy`, `roller`) are treated identically.

#include <optional>
#include <string>
#include <vector>

#include "editor_backend.h"
#include "model/selection.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

namespace wpiemu::app::editor {

// What a wheel or a mechanism contributes to the motion graph.
struct OwnerMotion {
    SelectionRef owner;                                // wheel(name) or mechanism(id)
    std::vector<sdk::MotionAxisInfo> axes;             // its axes (advertised by the SDK model)
    std::vector<const sdk::MotionLink*> incoming;      // links that target one of its axes
    // Every axis that has a motor slot follows another axis: the owner takes no motor of its own.
    bool followsAll = false;
    // Source owner of its first incoming link that comes from another owner (where the tree nests it).
    std::optional<SelectionRef> primarySource;
    std::vector<SelectionRef> drives;                  // other owners with an axis that follows one of its axes
};

class MotionIndex {
public:
    MotionIndex(const sdk::RobotConfig& config, const EditorCatalog& catalog);

    const std::vector<sdk::MotionAxisInfo>& axes() const { return axes_; }
    const sdk::MotionAxisInfo* axis(const sdk::SlotAddress& address) const;
    // The single link that drives `axis`, if any.
    const sdk::MotionLink* incoming(const sdk::SlotAddress& axis) const;
    OwnerMotion ownerMotion(const SelectionRef& owner) const;
    // True when `slot` is the motor slot of a following axis.
    bool isFollowerMotor(const sdk::SlotAddress& slot) const;
    // Owner (wheel or mechanism) an axis belongs to.
    static SelectionRef ownerOf(const sdk::SlotAddress& axis);
    // Axes advertised by one owner, in model order.
    std::vector<sdk::MotionAxisInfo> axesOf(const SelectionRef& owner) const;

private:
    const sdk::RobotConfig& config_;
    std::vector<sdk::MotionAxisInfo> axes_;
};

// Pure edits (undo is handled by the document). Both mark the links as explicitly configured.
// Makes `target` follow `source`; the ratio is the number shown in the UI (see ratioToSi). Replaces the
// target's existing incoming link and drops the target's own motor mappings (a follower takes no motor). Returns an
// empty string on success, otherwise why it was refused (the config is then unchanged).
std::string applyDrivenBy(sdk::RobotConfig& config, const EditorCatalog& catalog, const sdk::SlotAddress& target,
                          const sdk::SlotAddress& source, double displayRatio);
// The target takes its own motor again.
void removeDrivenBy(sdk::RobotConfig& config, const sdk::SlotAddress& target);

// Human label of an axis: "fl", "intake roller", "Elevator".
std::string axisLabel(const sdk::RobotConfig& config, const EditorCatalog& catalog, const sdk::SlotAddress& axis);
// Display ratio <-> SI ratio. The UI presents rev/rev, m/m, m/rev or rev/m; storage is rad/m.
double ratioFromSi(double siRatio, sdk::PlantKind source, sdk::PlantKind target);
double ratioToSi(double displayRatio, sdk::PlantKind source, sdk::PlantKind target);
std::string ratioUnits(sdk::PlantKind source, sdk::PlantKind target);

}  // namespace wpiemu::app::editor
