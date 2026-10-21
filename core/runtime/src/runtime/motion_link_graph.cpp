#include "wpiemu/core/runtime/motion_link_graph.h"

#include <algorithm>

#include "wpiemu/sdk/mechanisms/motion_config.h"

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

Result<MotionLinkGraph> MotionLinkGraph::resolveAndSort(
    const RobotConfig& config, std::span<const MotionAxisInfo> axes) {
    const std::vector<MotionAxisInfo> infos(axes.begin(), axes.end());
    const auto issues = sdk::motionLinkIssues(config, infos);
    if (!issues.empty()) return makeError(ErrorCode::InvalidArgument, issues.front().message);

    MotionLinkGraph graph;
    graph.sources.resize(infos.size());
    graph.ratios.assign(infos.size(), 1.0);
    for (std::size_t i = 0; i < infos.size(); ++i) {
        if (const auto* link = sdk::motionLinkTo(config, infos[i].address)) {
            for (std::size_t j = 0; j < infos.size(); ++j)
                if (infos[j].address == link->source) graph.sources[i] = j;
            graph.ratios[i] = link->ratio;
        }
    }

    std::vector<bool> ordered(infos.size(), false);
    while (graph.order.size() < infos.size()) {
        const auto previousSize = graph.order.size();
        for (std::size_t i = 0; i < infos.size(); ++i)
            if (!ordered[i] && (!graph.sources[i] || ordered[*graph.sources[i]])) {
                graph.order.push_back(i);
                ordered[i] = true;
            }
        if (graph.order.size() == previousSize)
            return makeError(ErrorCode::InvalidArgument,
                             "Motion links contain a dependency cycle.");
    }
    return graph;
}

Result<MotionLinkGraph> MotionLinkGraph::build(
    const RobotConfig& config, IDrivetrain* drivetrain,
    std::span<const std::unique_ptr<IMechanism>> mechanisms) {
    if (config.motionLinks.empty()) return MotionLinkGraph{};

    std::vector<MotionAxisBinding> axes;
    bool drivetrainHasAxes = false;
    if (drivetrain != nullptr) {
        axes = drivetrain->motionAxes();
        drivetrainHasAxes = !axes.empty();
    }
    std::vector<bool> mechanismHasAxes;
    mechanismHasAxes.reserve(mechanisms.size());
    for (const auto& mechanism : mechanisms) {
        auto mechAxes = mechanism->motionAxes();
        mechanismHasAxes.push_back(!mechAxes.empty());
        for (auto& axis : mechAxes) axes.push_back(std::move(axis));
    }

    std::vector<MotionAxisInfo> infos;
    std::vector<PlantState> starts;
    infos.reserve(axes.size());
    starts.reserve(axes.size());
    for (const auto& axis : axes) {
        infos.push_back(axis.info);
        starts.push_back(axis.state());
    }

    Result<MotionLinkGraph> resolved = resolveAndSort(config, infos);
    if (!resolved) return resolved.error();

    MotionLinkGraph graph = std::move(resolved).value();
    graph.axes = std::move(axes);
    graph.starts = std::move(starts);
    graph.mechanismHasAxes = std::move(mechanismHasAxes);
    graph.drivetrainHasAxes = drivetrainHasAxes;
    return graph;
}

bool MotionLinkGraph::isDrivenByMotionLink(std::span<const MotionAxisInfo> axes,
                                           std::span<const std::optional<std::size_t>> sources,
                                           const SlotAddress& slot) {
    for (std::size_t i = 0; i < axes.size() && i < sources.size(); ++i) {
        const auto& motors = axes[i].motorSlots;
        if (sources[i] && std::find(motors.begin(), motors.end(), slot) != motors.end())
            return true;
    }
    return false;
}

bool MotionLinkGraph::isDrivenByMotionLink(std::span<const MotionAxisBinding> axes,
                                           std::span<const std::optional<std::size_t>> sources,
                                           const SlotAddress& slot) {
    for (std::size_t i = 0; i < axes.size() && i < sources.size(); ++i) {
        const auto& motors = axes[i].info.motorSlots;
        if (sources[i] && std::find(motors.begin(), motors.end(), slot) != motors.end())
            return true;
    }
    return false;
}

}  // namespace wpiemu::core::runtime
