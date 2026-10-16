#pragma once

#include <QString>

#include <cstdint>

#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::runview {

// Rolling live statistics derived from consecutive snapshots. Wall time is injected so tests are exact.
struct SimStatsValues {
    double simTimeS = 0.0;
    double tickRateHz = 0.0;       // sim ticks per wall second
    double realtimeFactor = 0.0;   // sim seconds per wall second
    double batteryV = 0.0;
    double physicsStepMs = 0.0;
    int bodies = 0;
    int contacts = 0;
    int entities = 0;              // instanced entities (fuel ...)
    bool paused = false;
    bool robotConnected = false;
    bool hasData = false;

    bool operator==(const SimStatsValues&) const = default;
};

class SimStats {
public:
    // Returns true when the published values changed.
    bool update(const sdk::ISimSnapshot* snapshot, std::int64_t wallMs);
    const SimStatsValues& values() const { return values_; }
    void reset();

private:
    SimStatsValues values_;
    std::uint64_t windowTick_ = 0;
    double windowSimS_ = 0.0;
    std::int64_t windowWallMs_ = -1;
};

QString formatSimTime(double seconds);  // mm:ss.t

}  // namespace wpiemu::app::runview
