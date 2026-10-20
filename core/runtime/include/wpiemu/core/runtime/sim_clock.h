#pragma once

#include <cstdint>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/clock.h"

namespace wpiemu::core::runtime {

// The sim clock: integer microseconds, advanced only by the sim loop (never reads a wall clock).
class SimClock final : public sdk::ISimClock {
public:
    explicit SimClock(sdk::SimTimeUs stepUs = sdk::kDefaultStepUs) : stepUs_(stepUs > 0 ? stepUs : sdk::kDefaultStepUs) {}

    sdk::SimTimeUs simTimeUs() const override { return timeUs_; }
    std::uint64_t tickIndex() const override { return tick_; }
    sdk::SimTimeUs fixedStepUs() const override { return stepUs_; }

    void advance() {
        timeUs_ += stepUs_;
        ++tick_;
    }
    void reset() {
        timeUs_ = 0;
        tick_ = 0;
    }

private:
    sdk::SimTimeUs stepUs_;
    sdk::SimTimeUs timeUs_ = 0;
    std::uint64_t tick_ = 0;
};

// Battery / PDP model: terminal voltage = nominal - I * R, from the TOTAL current drawn last tick (the
// algebraic loop motors <-> voltage is broken by a one-tick lag). Brownout trips below brownoutVoltageV and
// clears only after the voltage recovers by `kRecoveryMarginV` (hysteresis; no chatter).
class BatteryModel {
public:
    static constexpr double kRecoveryMarginV = 0.5;

    explicit BatteryModel(const sdk::BatteryConfig& config = {}) : config_(config), voltageV_(config.nominalVoltageV) {}

    // Returns the new terminal voltage. Negative currents (regeneration) are treated as zero draw.
    double update(double totalCurrentA);
    void reset();

    double voltageV() const { return voltageV_; }
    double currentA() const { return currentA_; }
    bool brownedOut() const { return brownedOut_; }
    std::uint64_t brownoutCount() const { return brownoutCount_; }
    const sdk::BatteryConfig& config() const { return config_; }

private:
    sdk::BatteryConfig config_;
    double voltageV_;
    double currentA_ = 0.0;
    bool brownedOut_ = false;
    std::uint64_t brownoutCount_ = 0;
};

}  // namespace wpiemu::core::runtime
