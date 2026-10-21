#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/drivetrain.h"

namespace wpiemu::core::runtime {

// Resolved one-way motion-link dependency graph and topological evaluation order for a session's axes.
struct MotionLinkGraph {
    std::vector<sdk::MotionAxisBinding> axes;
    std::vector<sdk::PlantState> starts;
    std::vector<std::optional<std::size_t>> sources;
    std::vector<double> ratios;
    std::vector<std::size_t> order;
    std::vector<bool> mechanismHasAxes;
    bool drivetrainHasAxes = false;

    // Validates `config.motionLinks` against `axes`, resolves link source indices and ratios, and
    // returns a topological evaluation order for the axes.
    static sdk::Result<MotionLinkGraph> resolveAndSort(
        const sdk::RobotConfig& config, std::span<const sdk::MotionAxisInfo> axes);

    // Collects motion axes from the drivetrain and mechanisms, captures initial plant states, and
    // resolves/topologically sorts their motion links when `config.motionLinks` is non-empty.
    static sdk::Result<MotionLinkGraph> build(
        const sdk::RobotConfig& config, sdk::IDrivetrain* drivetrain,
        std::span<const std::unique_ptr<sdk::IMechanism>> mechanisms);

    // True when `slot` is a motor slot on a follower axis (driven by a motion link).
    static bool isDrivenByMotionLink(std::span<const sdk::MotionAxisInfo> axes,
                                     std::span<const std::optional<std::size_t>> sources,
                                     const sdk::SlotAddress& slot);
    static bool isDrivenByMotionLink(std::span<const sdk::MotionAxisBinding> axes,
                                     std::span<const std::optional<std::size_t>> sources,
                                     const sdk::SlotAddress& slot);
};

}  // namespace wpiemu::core::runtime
