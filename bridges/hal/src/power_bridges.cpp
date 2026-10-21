#include "power_bridges.h"

#include <string>

#include "bridge_support.h"
#include "wpiemu/bridges/hal/hal_units.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/signals.h"

namespace wpiemu::bridges::hal {

using namespace detail;
using sdk::Capability;
using sdk::SignalDirection;
using sdk::SignalType;

// ---- PDP / PDH ---------------------------------------------------------------------------------

namespace {
constexpr std::size_t kPdVoltage = 0;
constexpr std::size_t kPdTemperature = 1;
constexpr std::size_t kPdFirstCurrent = 2;
}  // namespace

int PowerDistributionBridge::channelCount() const { return hal_->moduleCount(); }
bool PowerDistributionBridge::isInitialized(int module) const { return hal_->initialized(module); }

sdk::DeviceDescriptor PowerDistributionBridge::describe(int module) const {
    return makeDescriptor("power_distribution", module, -1, Capability::SupplyVoltage,
                          "HALSIM PowerDistribution[" + std::to_string(module) + "]",
                          "Power distribution " + std::to_string(module));
}

std::vector<sdk::SignalSpec> PowerDistributionBridge::signalsOf(int) const {
    std::vector<sdk::SignalSpec> specs{
        sig(sdk::signal::kSupplyVoltage, SignalDirection::Input, SignalType::Double, "V"),
        sig("temperature", SignalDirection::Input, SignalType::Double, "degC"),
    };
    for (int ch = 0; ch < hal_->channelCount(); ++ch)
        specs.push_back(sig(("channelCurrent" + std::to_string(ch)).c_str(), SignalDirection::Input, SignalType::Double, "A"));
    return specs;
}

void PowerDistributionBridge::writeSignal(int module, std::size_t signalIndex, double value) {
    if (signalIndex == kPdVoltage) {
        hal_->setVoltage(module, value);
    } else if (signalIndex == kPdTemperature) {
        hal_->setTemperature(module, value);
    } else if (signalIndex >= kPdFirstCurrent) {
        const int ch = static_cast<int>(signalIndex - kPdFirstCurrent);
        if (ch < hal_->channelCount()) hal_->setCurrent(module, ch, value);
    }
}

// ---- roboRIO -----------------------------------------------------------------------------------

namespace {
enum RioSignal : std::size_t { kRioSupplyVoltage, kRioVInCurrent, kRioBrownout };
}

sdk::DeviceDescriptor RoboRioBridge::describe(int) const {
    return makeDescriptor("roborio", 0, -1, Capability::SupplyVoltage, "HALSIM RoboRIO", "roboRIO");
}

std::vector<sdk::SignalSpec> RoboRioBridge::signalsOf(int) const {
    return {
        sig(sdk::signal::kSupplyVoltage, SignalDirection::Input, SignalType::Double, "V"),
        sig("vInCurrent", SignalDirection::Input, SignalType::Double, "A"),
        sig("brownoutVoltage", SignalDirection::Output, SignalType::Double, "V"),
    };
}

double RoboRioBridge::readSignal(int, std::size_t signalIndex) const {
    return signalIndex == kRioBrownout ? hal_->brownoutVoltage() : 0.0;
}

void RoboRioBridge::writeSignal(int, std::size_t signalIndex, double value) {
    if (signalIndex == kRioSupplyVoltage) hal_->setVInVoltage(value);
    else if (signalIndex == kRioVInCurrent) hal_->setVInCurrent(value);
}

// ---- accelerometer -----------------------------------------------------------------------------

bool AccelerometerBridge::isInitialized(int) const { return hal_->active(); }

sdk::DeviceDescriptor AccelerometerBridge::describe(int) const {
    return makeDescriptor("accelerometer", 0, -1, Capability::Imu, "HALSIM Accelerometer[0]", "roboRIO accelerometer");
}

std::vector<sdk::SignalSpec> AccelerometerBridge::signalsOf(int) const {
    return {
        sig(sdk::signal::kAccelX, SignalDirection::Input, SignalType::Double, "m/s^2"),
        sig(sdk::signal::kAccelY, SignalDirection::Input, SignalType::Double, "m/s^2"),
        sig(sdk::signal::kAccelZ, SignalDirection::Input, SignalType::Double, "m/s^2"),
    };
}

void AccelerometerBridge::writeSignal(int, std::size_t signalIndex, double value) {
    if (signalIndex >= accelMps2_.size()) return;
    accelMps2_[signalIndex] = value;
    hal_->setAcceleration(accelMps2_[0] / sdk::kStandardGravityMps2, accelMps2_[1] / sdk::kStandardGravityMps2,
                          accelMps2_[2] / sdk::kStandardGravityMps2);
}

}  // namespace wpiemu::bridges::hal
