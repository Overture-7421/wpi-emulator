#include "sim_stats.h"

#include <cmath>

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::runview {

QString formatSimTime(double seconds) {
    const double s = seconds < 0.0 ? 0.0 : seconds;
    const int minutes = static_cast<int>(s / 60.0);
    return QStringLiteral("%1:%2").arg(minutes, 2, 10, QLatin1Char('0')).arg(s - minutes * 60.0, 4, 'f', 1, QLatin1Char('0'));
}

void SimStats::reset() {
    values_ = {};
    windowWallMs_ = -1;
    windowTick_ = 0;
    windowSimS_ = 0.0;
}

bool SimStats::update(const sdk::ISimSnapshot* snapshot, std::int64_t wallMs) {
    SimStatsValues next = values_;
    if (snapshot == nullptr) {
        next = {};
        windowWallMs_ = -1;
    } else {
        next.hasData = true;
        next.simTimeS = sdk::usToSeconds(snapshot->simTimeUs());
        next.batteryV = snapshot->batteryVoltageV();
        next.physicsStepMs = snapshot->physicsStats().lastStepMs;
        next.bodies = snapshot->physicsStats().bodies;
        next.contacts = snapshot->physicsStats().contacts;
        next.paused = snapshot->paused();
        next.robotConnected = snapshot->robotConnected();
        int n = 0;
        for (const auto& g : snapshot->entityGroups()) n += static_cast<int>(g.positions.size());
        next.entities = n;

        const std::uint64_t tick = snapshot->tick();
        if (windowWallMs_ < 0 || tick < windowTick_) {
            windowWallMs_ = wallMs;
            windowTick_ = tick;
            windowSimS_ = next.simTimeS;
        } else if (wallMs - windowWallMs_ >= 400) {
            const double wallS = static_cast<double>(wallMs - windowWallMs_) / 1000.0;
            next.tickRateHz = static_cast<double>(tick - windowTick_) / wallS;
            next.realtimeFactor = (next.simTimeS - windowSimS_) / wallS;
            windowWallMs_ = wallMs;
            windowTick_ = tick;
            windowSimS_ = next.simTimeS;
        }
    }
    if (next == values_) return false;
    values_ = next;
    return true;
}

}  // namespace wpiemu::app::runview
