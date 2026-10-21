#pragma once

#include <array>
#include <memory>
#include <utility>
#include <vector>

#include "hal_family_bridge.h"
#include "wpiemu/bridges/hal/hal_api.h"

namespace wpiemu::bridges::hal {

// PDP / PDH: the sim writes bus voltage, temperature and per-channel current.
class PowerDistributionBridge final : public HalFamilyBridge<IPowerDistributionHal> {
public:
    explicit PowerDistributionBridge(std::shared_ptr<IPowerDistributionHal> hal)
        : HalFamilyBridge("hal.power_distribution", std::move(hal)) {}

protected:
    int channelCount() const override;
    bool isInitialized(int module) const override;
    sdk::DeviceDescriptor describe(int module) const override;
    std::vector<sdk::SignalSpec> signalsOf(int module) const override;
    void writeSignal(int module, std::size_t signalIndex, double value) override;
};

// The roboRIO itself (always present): battery voltage / current write-back and brownout threshold.
class RoboRioBridge final : public HalFamilyBridge<IRoboRioHal> {
public:
    explicit RoboRioBridge(std::shared_ptr<IRoboRioHal> hal) : HalFamilyBridge("hal.roborio", std::move(hal)) {}

protected:
    int channelCount() const override { return 1; }
    bool isInitialized(int) const override { return true; }
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;
};

// Built-in roboRIO accelerometer; live once the robot activates it. Canonical m/s^2, HAL in g.
class AccelerometerBridge final : public HalFamilyBridge<IAccelerometerHal> {
public:
    explicit AccelerometerBridge(std::shared_ptr<IAccelerometerHal> hal)
        : HalFamilyBridge("hal.accelerometer", std::move(hal)) {}

protected:
    int channelCount() const override { return 1; }
    bool isInitialized(int) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    std::array<double, 3> accelMps2_{0.0, 0.0, 0.0};
};

}  // namespace wpiemu::bridges::hal
