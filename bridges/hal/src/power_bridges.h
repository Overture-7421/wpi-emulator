#pragma once

#include <array>
#include <memory>
#include <vector>

#include "wpiemu/bridges/hal/hal_api.h"
#include "wpiemu/sdk/devices/channel_family_bridge.h"

namespace wpiemu::bridges::hal {

// PDP / PDH: the sim writes bus voltage, temperature and per-channel current.
class PowerDistributionBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit PowerDistributionBridge(std::shared_ptr<IPowerDistributionHal> hal);

protected:
    int channelCount() const override;
    bool isInitialized(int module) const override;
    sdk::DeviceDescriptor describe(int module) const override;
    std::vector<sdk::SignalSpec> signalsOf(int module) const override;
    double readSignal(int, std::size_t) const override { return 0.0; }
    void writeSignal(int module, std::size_t signalIndex, double value) override;

private:
    std::shared_ptr<IPowerDistributionHal> hal_;
};

// The roboRIO itself (always present): battery voltage / current write-back and brownout threshold.
class RoboRioBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit RoboRioBridge(std::shared_ptr<IRoboRioHal> hal);

protected:
    int channelCount() const override { return 1; }
    bool isInitialized(int) const override { return true; }
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    std::shared_ptr<IRoboRioHal> hal_;
};

// Built-in roboRIO accelerometer; live once the robot activates it. Canonical m/s^2, HAL in g.
class AccelerometerBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit AccelerometerBridge(std::shared_ptr<IAccelerometerHal> hal);

protected:
    int channelCount() const override { return 1; }
    bool isInitialized(int) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int, std::size_t) const override { return 0.0; }
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    std::shared_ptr<IAccelerometerHal> hal_;
    std::array<double, 3> accelMps2_{0.0, 0.0, 0.0};
};

}  // namespace wpiemu::bridges::hal
