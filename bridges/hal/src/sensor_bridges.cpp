#include "sensor_bridges.h"

#include <string>

#include "bridge_support.h"
#include "wpiemu/bridges/hal/hal_units.h"
#include "wpiemu/sdk/devices/signals.h"

namespace wpiemu::bridges::hal {

using namespace detail;
using sdk::Capability;
using sdk::SignalDirection;
using sdk::SignalType;

// ---- AnalogIn ----------------------------------------------------------------------------------

AnalogInBridge::AnalogInBridge(std::shared_ptr<IAnalogInHal> hal)
    : ChannelFamilyBridge("hal.analog_in"), hal_(std::move(hal)) {}

int AnalogInBridge::channelCount() const { return hal_->channelCount(); }
bool AnalogInBridge::isInitialized(int channel) const { return hal_->initialized(channel); }

sdk::DeviceDescriptor AnalogInBridge::describe(int channel) const {
    return makeDescriptor("analog_in", channel, -1, Capability::AnalogInput,
                          "HALSIM AnalogIn[" + std::to_string(channel) + "]", "Analog in " + std::to_string(channel));
}

std::vector<sdk::SignalSpec> AnalogInBridge::signalsOf(int) const {
    return {sig(sdk::signal::kAnalogVoltage, SignalDirection::Input, SignalType::Double, "V")};
}

void AnalogInBridge::writeSignal(int channel, std::size_t, double value) { hal_->setVoltage(channel, value); }

// ---- AnalogOut ---------------------------------------------------------------------------------

AnalogOutBridge::AnalogOutBridge(std::shared_ptr<IAnalogOutHal> hal)
    : ChannelFamilyBridge("hal.analog_out"), hal_(std::move(hal)) {}

int AnalogOutBridge::channelCount() const { return hal_->channelCount(); }
bool AnalogOutBridge::isInitialized(int channel) const { return hal_->initialized(channel); }

sdk::DeviceDescriptor AnalogOutBridge::describe(int channel) const {
    return makeDescriptor("analog_out", channel, -1, sdk::CapabilitySet{},
                          "HALSIM AnalogOut[" + std::to_string(channel) + "]", "Analog out " + std::to_string(channel));
}

std::vector<sdk::SignalSpec> AnalogOutBridge::signalsOf(int) const {
    return {sig(sdk::signal::kAnalogVoltage, SignalDirection::Output, SignalType::Double, "V")};
}

double AnalogOutBridge::readSignal(int channel, std::size_t) const { return hal_->voltage(channel); }

// ---- Encoder -----------------------------------------------------------------------------------

namespace {
enum EncoderSignal : std::size_t { kEncPosition, kEncVelocity, kEncCount, kEncChannelA, kEncChannelB, kEncDistancePerPulse, kEncReversed };
}

EncoderBridge::EncoderBridge(std::shared_ptr<IEncoderHal> hal, double countsPerRevolution)
    : ChannelFamilyBridge("hal.encoder"), hal_(std::move(hal)), countsPerRevolution_(countsPerRevolution) {}

int EncoderBridge::channelCount() const { return hal_->channelCount(); }
bool EncoderBridge::isInitialized(int channel) const { return hal_->initialized(channel); }

sdk::DeviceDescriptor EncoderBridge::describe(int channel) const {
    const std::string pair =
        "DIO " + std::to_string(hal_->digitalChannelA(channel)) + "/" + std::to_string(hal_->digitalChannelB(channel));
    return makeDescriptor("encoder", channel, -1, Capability::PositionSensor | Capability::VelocitySensor,
                          "HALSIM Encoder[" + std::to_string(channel) + "]",
                          "Encoder " + std::to_string(channel) + " (" + pair + ")");
}

std::vector<sdk::SignalSpec> EncoderBridge::signalsOf(int) const {
    return {
        sig(sdk::signal::kPosition, SignalDirection::Input, SignalType::Double, "rot"),
        sig(sdk::signal::kVelocity, SignalDirection::Input, SignalType::Double, "rps"),
        sig("count", SignalDirection::Input, SignalType::Int),
        sig("digitalChannelA", SignalDirection::Output, SignalType::Int),
        sig("digitalChannelB", SignalDirection::Output, SignalType::Int),
        sig("distancePerPulse", SignalDirection::Output, SignalType::Double),
        sig("reverseDirection", SignalDirection::Output, SignalType::Bool),
    };
}

double EncoderBridge::readSignal(int channel, std::size_t signalIndex) const {
    switch (signalIndex) {
        case kEncChannelA: return hal_->digitalChannelA(channel);
        case kEncChannelB: return hal_->digitalChannelB(channel);
        case kEncDistancePerPulse: return hal_->distancePerPulse(channel);
        case kEncReversed: return fromBool(hal_->reverseDirection(channel));
        default: return 0.0;
    }
}

void EncoderBridge::writeSignal(int channel, std::size_t signalIndex, double value) {
    const double sign = hal_->reverseDirection(channel) ? -1.0 : 1.0;
    switch (signalIndex) {
        case kEncPosition: hal_->setCount(channel, rotationsToCounts(sign * value, countsPerRevolution_)); break;
        case kEncVelocity: {
            const double signedRps = sign * value;
            hal_->setPeriod(channel, velocityToPeriod(signedRps, countsPerRevolution_));
            if (signedRps != 0.0) hal_->setDirection(channel, signedRps > 0.0);
            break;
        }
        case kEncCount: hal_->setCount(channel, rotationsToCounts(value, 1.0)); break;
        default: break;
    }
}

// ---- DutyCycle ---------------------------------------------------------------------------------

namespace {
enum DutyCycleSignal : std::size_t { kDcPosition, kDcFrequency, kDcDigitalChannel };
}

DutyCycleBridge::DutyCycleBridge(std::shared_ptr<IDutyCycleHal> hal, int defaultFrequencyHz)
    : ChannelFamilyBridge("hal.duty_cycle"), hal_(std::move(hal)), defaultFrequencyHz_(defaultFrequencyHz) {}

sdk::Status DutyCycleBridge::start(sdk::IBridgeHost& host) {
    const auto n = static_cast<std::size_t>(channelCount());
    seen_.assign(n, false);
    defaulted_.assign(n, false);
    return ChannelFamilyBridge::start(host);
}

// A freshly created DutyCycle reports 0 Hz, which DutyCycleEncoder treats as "disconnected".
void DutyCycleBridge::poll() {
    for (std::size_t i = 0; i < seen_.size(); ++i) {
        const int ch = static_cast<int>(i);
        const bool live = isInitialized(ch);
        if (live && !seen_[i] && hal_->frequency(ch) == 0 && defaultFrequencyHz_ > 0) {
            hal_->setFrequency(ch, defaultFrequencyHz_);
            defaulted_[i] = true;
        } else if (!live) {
            defaulted_[i] = false;
        }
        seen_[i] = live;
    }
    ChannelFamilyBridge::poll();
}

void DutyCycleBridge::stop() noexcept {
    for (std::size_t i = 0; i < defaulted_.size(); ++i)
        if (defaulted_[i]) hal_->setFrequency(static_cast<int>(i), 0);
    seen_.clear();
    defaulted_.clear();
    ChannelFamilyBridge::stop();
}

int DutyCycleBridge::channelCount() const { return hal_->channelCount(); }
bool DutyCycleBridge::isInitialized(int channel) const { return hal_->initialized(channel); }

sdk::DeviceDescriptor DutyCycleBridge::describe(int channel) const {
    return makeDescriptor("duty_cycle", channel, -1, Capability::PositionSensor,
                          "HALSIM DutyCycle[" + std::to_string(channel) + "]",
                          "Duty cycle " + std::to_string(channel) + " (DIO " +
                              std::to_string(hal_->digitalChannel(channel)) + ")");
}

std::vector<sdk::SignalSpec> DutyCycleBridge::signalsOf(int) const {
    return {
        sig(sdk::signal::kPosition, SignalDirection::Input, SignalType::Double, "rot"),
        sig("frequency", SignalDirection::Input, SignalType::Int, "Hz"),
        sig("digitalChannel", SignalDirection::Output, SignalType::Int),
    };
}

double DutyCycleBridge::readSignal(int channel, std::size_t signalIndex) const {
    return signalIndex == kDcDigitalChannel ? static_cast<double>(hal_->digitalChannel(channel)) : 0.0;
}

void DutyCycleBridge::writeSignal(int channel, std::size_t signalIndex, double value) {
    if (signalIndex == kDcPosition) {
        hal_->setOutput(channel, rotationsToDutyRatio(value));
    } else if (signalIndex == kDcFrequency) {
        hal_->setFrequency(channel, static_cast<int>(std::lround(value)));
        if (channel >= 0 && static_cast<std::size_t>(channel) < defaulted_.size()) defaulted_[static_cast<std::size_t>(channel)] = false;
    }
}

}  // namespace wpiemu::bridges::hal
