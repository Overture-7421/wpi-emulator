#include "wpiemu/core/runtime/sim_clock.h"

#include <algorithm>

namespace wpiemu::core::runtime {

double BatteryModel::update(double totalCurrentA) {
    currentA_ = std::max(0.0, totalCurrentA);
    voltageV_ = std::clamp(config_.nominalVoltageV - currentA_ * config_.internalResistanceOhm, 0.0, config_.nominalVoltageV);
    if (!brownedOut_ && voltageV_ < config_.brownoutVoltageV) {
        brownedOut_ = true;
        ++brownoutCount_;
    } else if (brownedOut_ && voltageV_ >= config_.brownoutVoltageV + kRecoveryMarginV) {
        brownedOut_ = false;
    }
    return voltageV_;
}

void BatteryModel::reset() {
    voltageV_ = config_.nominalVoltageV;
    currentA_ = 0.0;
    brownedOut_ = false;
}

}  // namespace wpiemu::core::runtime
