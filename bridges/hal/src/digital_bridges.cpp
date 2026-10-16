#include "digital_bridges.h"

#include <string>

#include "bridge_support.h"
#include "wpiemu/bridges/hal/hal_units.h"
#include "wpiemu/sdk/devices/signals.h"

namespace wpiemu::bridges::hal {

using namespace detail;
using sdk::Capability;
using sdk::SignalDirection;
using sdk::SignalType;

// ---- PWM ---------------------------------------------------------------------------------------

PwmBridge::PwmBridge(std::shared_ptr<IPwmHal> hal) : ChannelFamilyBridge("hal.pwm"), hal_(std::move(hal)) {}

int PwmBridge::channelCount() const { return hal_->channelCount(); }
bool PwmBridge::isInitialized(int channel) const { return hal_->initialized(channel); }

sdk::DeviceDescriptor PwmBridge::describe(int channel) const {
    return makeDescriptor("pwm", channel, -1, Capability::MotorOutput | Capability::PwmOutput,
                          "HALSIM PWM[" + std::to_string(channel) + "]", "PWM " + std::to_string(channel));
}

std::vector<sdk::SignalSpec> PwmBridge::signalsOf(int) const {
    return {
        sig(sdk::signal::kDutyCycle, SignalDirection::Output, SignalType::Double),
        sig(sdk::signal::kPwmSpeed, SignalDirection::Output, SignalType::Double),
        sig("pwmPosition", SignalDirection::Output, SignalType::Double),
        sig("pulseMicroseconds", SignalDirection::Output, SignalType::Int, "us"),
    };
}

double PwmBridge::readSignal(int channel, std::size_t signalIndex) const {
    switch (signalIndex) {
        case 0:
        case 1: return normalizeSpeed(hal_->speed(channel));
        case 2: return hal_->position(channel);
        case 3: return static_cast<double>(hal_->pulseMicroseconds(channel));
        default: return 0.0;
    }
}

// ---- DIO ---------------------------------------------------------------------------------------

DioBridge::DioBridge(std::shared_ptr<IDioHal> hal) : ChannelFamilyBridge("hal.dio"), hal_(std::move(hal)) {}

int DioBridge::channelCount() const { return hal_->channelCount(); }
bool DioBridge::isInitialized(int channel) const { return hal_->initialized(channel); }

sdk::DeviceDescriptor DioBridge::describe(int channel) const {
    const bool input = hal_->isInput(channel);
    return makeDescriptor("dio", channel, -1, input ? Capability::DigitalInput : Capability::DigitalOutput,
                          "HALSIM DIO[" + std::to_string(channel) + "]",
                          std::string(input ? "DIO in " : "DIO out ") + std::to_string(channel));
}

std::vector<sdk::SignalSpec> DioBridge::signalsOf(int channel) const {
    if (hal_->isInput(channel))
        return {sig(sdk::signal::kDigitalIn, SignalDirection::Input, SignalType::Bool)};
    return {sig(sdk::signal::kDigitalOut, SignalDirection::Output, SignalType::Bool)};
}

double DioBridge::readSignal(int channel, std::size_t) const { return fromBool(hal_->value(channel)); }

void DioBridge::writeSignal(int channel, std::size_t, double value) {
    // Never fight the robot for a pin it configured as an output.
    if (hal_->isInput(channel)) hal_->setValue(channel, toBool(value));
}

// ---- Relay -------------------------------------------------------------------------------------

RelayBridge::RelayBridge(std::shared_ptr<IRelayHal> hal) : ChannelFamilyBridge("hal.relay"), hal_(std::move(hal)) {}

int RelayBridge::channelCount() const { return hal_->headerCount() * 2; }

bool RelayBridge::isInitialized(int channel) const {
    const int header = channel / 2;
    return (channel % 2 == 0) ? hal_->forwardInitialized(header) : hal_->reverseInitialized(header);
}

sdk::DeviceDescriptor RelayBridge::describe(int channel) const {
    const int header = channel / 2;
    const int dir = channel % 2;
    return makeDescriptor("relay", header, dir, Capability::DigitalOutput,
                          "HALSIM Relay[" + std::to_string(header) + "]." + (dir == 0 ? "forward" : "reverse"),
                          "Relay " + std::to_string(header) + (dir == 0 ? " forward" : " reverse"));
}

std::vector<sdk::SignalSpec> RelayBridge::signalsOf(int) const {
    return {sig(sdk::signal::kDigitalOut, SignalDirection::Output, SignalType::Bool)};
}

double RelayBridge::readSignal(int channel, std::size_t) const {
    const int header = channel / 2;
    return fromBool((channel % 2 == 0) ? hal_->forward(header) : hal_->reverse(header));
}

}  // namespace wpiemu::bridges::hal
