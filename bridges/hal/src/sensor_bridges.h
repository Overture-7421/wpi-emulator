#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "hal_family_bridge.h"
#include "wpiemu/bridges/hal/hal_api.h"

namespace wpiemu::bridges::hal {

// Analog input: the sim drives the pin voltage (potentiometers, pressure sensors, ...).
class AnalogInBridge final : public IndexedHalBridge<IAnalogInHal> {
public:
    explicit AnalogInBridge(std::shared_ptr<IAnalogInHal> hal) : IndexedHalBridge("hal.analog_in", std::move(hal)) {}

protected:
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;
};

// Analog output (observe only).
class AnalogOutBridge final : public IndexedHalBridge<IAnalogOutHal> {
public:
    explicit AnalogOutBridge(std::shared_ptr<IAnalogOutHal> hal) : IndexedHalBridge("hal.analog_out", std::move(hal)) {}

protected:
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
};

// HAL quadrature encoder. Canonical position/velocity are converted with
// HalBridgeOptions::encoderCountsPerRevolution; the robot's reverseDirection flag is honoured
// because the sim HAL itself ignores it (a reversed encoder reads negative on real hardware).
class EncoderBridge final : public IndexedHalBridge<IEncoderHal> {
public:
    EncoderBridge(std::shared_ptr<IEncoderHal> hal, double countsPerRevolution)
        : IndexedHalBridge("hal.encoder", std::move(hal)), countsPerRevolution_(countsPerRevolution) {}

protected:
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    double countsPerRevolution_;
};

// DutyCycle input / DutyCycleEncoder (absolute position as a duty ratio).
class DutyCycleBridge final : public IndexedHalBridge<IDutyCycleHal> {
public:
    DutyCycleBridge(std::shared_ptr<IDutyCycleHal> hal, int defaultFrequencyHz)
        : IndexedHalBridge("hal.duty_cycle", std::move(hal)), defaultFrequencyHz_(defaultFrequencyHz) {}

    sdk::Status start(sdk::IBridgeHost& host) override;
    void poll() override;
    void stop() noexcept override;

protected:
    sdk::DeviceDescriptor describe(int channel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int channel) const override;
    double readSignal(int channel, std::size_t signalIndex) const override;
    void writeSignal(int channel, std::size_t signalIndex, double value) override;

private:
    int defaultFrequencyHz_;
    std::vector<bool> seen_;       // channel was live on the previous poll
    std::vector<bool> defaulted_;  // we wrote the default frequency (restored in stop())
};

}  // namespace wpiemu::bridges::hal
