#pragma once

#include <memory>

#include "wpiemu/bridges/hal/hal_api.h"
#include "wpiemu/sdk/devices/channel_family_bridge.h"

namespace wpiemu::bridges::hal {

// PWM headers. The controller model behind a PWM channel is unknown to the HAL, so every channel is
// reported as a generic "pwm" motor-capable output (speed already normalised to [-1, 1] by the HAL).
class PwmBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit PwmBridge(std::shared_ptr<IPwmHal> hal);

protected:
    int channelCount() const override;
    bool isInitialized(int channel) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int, std::size_t, double) override {}

private:
    std::shared_ptr<IPwmHal> hal_;
};

// DIO. Direction is fixed at init by WPILib (DigitalInput vs DigitalOutput): inputs are driven by the
// sim (limit switch / beam break), outputs are observed.
class DioBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit DioBridge(std::shared_ptr<IDioHal> hal);

protected:
    int channelCount() const override;
    bool isInitialized(int channel) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    std::shared_ptr<IDioHal> hal_;
};

// Relay header: forward and reverse are separate devices (DeviceKey.channel 0 = forward, 1 = reverse).
class RelayBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit RelayBridge(std::shared_ptr<IRelayHal> hal);

protected:
    int channelCount() const override;
    bool isInitialized(int channel) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int, std::size_t, double) override {}

private:
    std::shared_ptr<IRelayHal> hal_;
};

}  // namespace wpiemu::bridges::hal
