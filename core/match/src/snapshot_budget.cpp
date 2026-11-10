#include "wpiemu/core/match/snapshot_budget.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::core::match {

SnapshotFit fitSnapshot(sdk::match::MatchSnapshot& snapshot, const EntityPlan& plan, std::size_t budget, const SnapshotSizer& sizeOf,
                        EntitySizeEstimate& estimate) {
    SnapshotFit fit;
    const auto withChanges = [&](std::size_t n) {
        snapshot.entityDeltas = EntityDeltaEncoder::deltas(plan, n);
        return sizeOf(snapshot);
    };

    std::size_t base = withChanges(0);
    if (base > budget) {
        fit.coreOverBudget = true;
        for (sdk::match::RobotWireState& r : snapshot.robots) r.hud.clear();
        fit.hudDropped = true;
        base = withChanges(0);
        if (base > budget) {
            snapshot.gameHud.clear();
            base = withChanges(0);
        }
    }

    const std::size_t total = plan.changes.size();
    std::size_t fitCount = 0;
    std::size_t fitSize = base;
    if (total > 0 && base < budget) {
        // Estimate how many changes fit, then correct with a few measurements (the encoded size of a change varies with
        // its values), keeping the best prefix that is known to fit.
        const double est = std::max(1.0, estimate.bytesPerChange);
        std::size_t hi = total + 1;
        auto n = std::min(total, static_cast<std::size_t>(static_cast<double>(budget - base) / est));
        for (int iteration = 0; iteration < 4; ++iteration) {
            n = std::min(n, hi - 1);
            if (n <= fitCount) break;
            const std::size_t size = withChanges(n);
            if (size <= budget) {
                fitCount = n;
                fitSize = size;
                if (n == total) break;
                const auto more = static_cast<std::size_t>(static_cast<double>(budget - size) / est);
                if (more == 0) break;
                n += more;
            } else {
                hi = n;
                const auto less = static_cast<std::size_t>(std::ceil(static_cast<double>(size - budget) / est));
                n = n > less ? n - less : 0;
            }
        }
        if (fitCount > 0)
            estimate.bytesPerChange = 0.5 * estimate.bytesPerChange + 0.5 * (static_cast<double>(fitSize - base) / static_cast<double>(fitCount));
    }
    snapshot.entityDeltas = EntityDeltaEncoder::deltas(plan, fitCount);
    fit.entityChanges = fitCount;
    fit.encodedBytes = fitSize;
    return fit;
}

}  // namespace wpiemu::core::match
