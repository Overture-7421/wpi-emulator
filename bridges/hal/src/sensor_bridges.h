#pragma once

#include <memory>
#include <vector>

#include "wpiemu/bridges/hal/hal_api.h"
#include "wpiemu/sdk/devices/channel_family_bridge.h"

namespace wpiemu::bridges::hal {

// Analog input: the sim drives the pin voltage (potentiometers, pressure sensors, ...).
class AnalogInBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit AnalogInBridge(std::shared_ptr<IAnalogInHal> hal);

protected:
    int channelCount() const override;
    bool isInitialized(int channel) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int, std::size_t) const override { return 0.0; }
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    std::shared_ptr<IAnalogInHal> hal_;
};

// Analog output (observe only).
class AnalogOutBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit AnalogOutBridge(std::shared_ptr<IAnalogOutHal> hal);

protected:
    int channelCount() const override;
    bool isInitialized(int channel) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int, std::size_t, double) override {}

private:
    std::shared_ptr<IAnalogOutHal> hal_;
};

// HAL quadrature encoder. Canonical position/velocity are converted with
// HalBridgeOptions::encoderCountsPerRevolution; the robot's reverseDirection flag is honoured
// because the sim HAL itself ignores it (a reversed encoder reads negative on real hardware).
class EncoderBridge final : public sdk::ChannelFamilyBridge {
public:
    EncoderBridge(std::shared_ptr<IEncoderHal> hal, double countsPerRevolution);

protected:
    int channelCount() const override;
    bool isInitialized(int channel) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    std::shared_ptr<IEncoderHal> hal_;
    double countsPerRevolution_;
};

// DutyCycle input / DutyCycleEncoder (absolute position as a duty ratio).
class DutyCycleBridge final : public sdk::ChannelFamilyBridge {
public:
    DutyCycleBridge(std::shared_ptr<IDutyCycleHal> hal, int defaultFrequencyHz);

    sdk::Status start(sdk::IBridgeHost& host) override;
    void poll() override;
    void stop() noexcept override;

protected:
    int channelCount() const override;
    bool isInitialized(int channel) const override;
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    std::shared_ptr<IDutyCycleHal> hal_;
    int defaultFrequencyHz_;
    std::vector<bool> seen_;       // channel was live on the previous poll
    std::vector<bool> defaulted_;  // we wrote the default frequency (restored in stop())
};

}  // namespace wpiemu::bridges::hal
