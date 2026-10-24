#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "hal_family_bridge.h"
#include "wpiemu/bridges/hal/hal_api.h"

namespace wpiemu::bridges::hal {

// A PWM channel is motor-capable only when HAL exposes a normalized controller command.
class PwmBridge final : public IndexedHalBridge<IPwmHal> {
public:
    explicit PwmBridge(std::shared_ptr<IPwmHal> hal)
        : IndexedHalBridge("hal.pwm", std::move(hal)),
          speedAvailable_(static_cast<std::size_t>(hal_->channelCount())), positionAvailable_(hal_->hasPosition()) {}

protected:
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;

private:
    mutable std::vector<bool> speedAvailable_;
    bool positionAvailable_;
};

// DIO. Direction is fixed at init by WPILib (DigitalInput vs DigitalOutput): inputs are driven by the
// sim (limit switch / beam break), outputs are observed.
class DioBridge final : public IndexedHalBridge<IDioHal> {
public:
    explicit DioBridge(std::shared_ptr<IDioHal> hal) : IndexedHalBridge("hal.dio", std::move(hal)) {}

protected:
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;
};

// Relay header: forward and reverse are separate devices (DeviceKey.channel 0 = forward, 1 = reverse).
class RelayBridge final : public HalFamilyBridge<IRelayHal> {
public:
    explicit RelayBridge(std::shared_ptr<IRelayHal> hal) : HalFamilyBridge("hal.relay", std::move(hal)) {}

protected:
    int channelCount() const override;
    bool isInitialized(int channel) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
};

}  // namespace wpiemu::bridges::hal
