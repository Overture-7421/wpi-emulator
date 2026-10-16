#include "pneumatics_bridges.h"

#include "bridge_support.h"
#include "wpiemu/sdk/devices/signals.h"

namespace wpiemu::bridges::hal {

using namespace detail;
using sdk::Capability;
using sdk::SignalDirection;
using sdk::SignalType;

namespace {
enum ModuleSignal : std::size_t { kCompressorOn, kClosedLoop, kPressureSwitch, kCompressorCurrent };
}

// ---- module ------------------------------------------------------------------------------------

PneumaticsModuleBridge::PneumaticsModuleBridge(std::shared_ptr<IPneumaticsHal> hal, PneumaticsKind kind)
    : ChannelFamilyBridge(kind.bridgeName), hal_(std::move(hal)), kind_(std::move(kind)) {}

int PneumaticsModuleBridge::channelCount() const { return hal_->moduleCount(); }
bool PneumaticsModuleBridge::isInitialized(int module) const { return hal_->moduleInitialized(module); }

sdk::DeviceDescriptor PneumaticsModuleBridge::describe(int module) const {
    return makeDescriptor(kind_.moduleType, module, -1, sdk::CapabilitySet{},
                          "HALSIM " + kind_.label + "[" + std::to_string(module) + "]",
                          kind_.label + " " + std::to_string(module), kind_.vendor);
}

std::vector<sdk::SignalSpec> PneumaticsModuleBridge::signalsOf(int) const {
    return {
        sig("compressorOn", SignalDirection::Output, SignalType::Bool),
        sig("compressorClosedLoop", SignalDirection::Output, SignalType::Bool),
        sig("pressureSwitch", SignalDirection::Input, SignalType::Bool),
        sig("compressorCurrent", SignalDirection::Input, SignalType::Double, "A"),
    };
}

double PneumaticsModuleBridge::readSignal(int module, std::size_t signalIndex) const {
    switch (signalIndex) {
        case kCompressorOn: return fromBool(hal_->compressorOn(module));
        case kClosedLoop: return fromBool(hal_->closedLoopEnabled(module));
        default: return 0.0;
    }
}

void PneumaticsModuleBridge::writeSignal(int module, std::size_t signalIndex, double value) {
    if (signalIndex == kPressureSwitch) hal_->setPressureSwitch(module, toBool(value));
    else if (signalIndex == kCompressorCurrent) hal_->setCompressorCurrent(module, value);
}

// ---- solenoids ---------------------------------------------------------------------------------

SolenoidBridge::SolenoidBridge(std::shared_ptr<IPneumaticsHal> hal, PneumaticsKind kind, bool announceAll)
    : ChannelFamilyBridge(kind.bridgeName + ".solenoid"),
      hal_(std::move(hal)),
      kind_(std::move(kind)),
      announceAll_(announceAll) {}

int SolenoidBridge::channelsPerModule() const { return hal_->solenoidChannelCount(); }

int SolenoidBridge::channelCount() const { return hal_->moduleCount() * channelsPerModule(); }

sdk::Status SolenoidBridge::start(sdk::IBridgeHost& host) {
    latched_.assign(static_cast<std::size_t>(channelCount()), false);
    return ChannelFamilyBridge::start(host);
}

void SolenoidBridge::poll() {
    const int perModule = channelsPerModule();
    for (std::size_t flat = 0; flat < latched_.size() && perModule > 0; ++flat) {
        const int module = static_cast<int>(flat) / perModule;
        const int ch = static_cast<int>(flat) % perModule;
        if (!hal_->moduleInitialized(module)) latched_[flat] = false;
        else if (hal_->solenoidOutput(module, ch)) latched_[flat] = true;
    }
    ChannelFamilyBridge::poll();
}

bool SolenoidBridge::isInitialized(int flatChannel) const {
    const int perModule = channelsPerModule();
    if (perModule <= 0 || !hal_->moduleInitialized(flatChannel / perModule)) return false;
    return announceAll_ || latched_[static_cast<std::size_t>(flatChannel)];
}

sdk::DeviceDescriptor SolenoidBridge::describe(int flatChannel) const {
    const int module = flatChannel / channelsPerModule();
    const int ch = flatChannel % channelsPerModule();
    return makeDescriptor(kind_.solenoidType, module, ch, Capability::Solenoid | Capability::DigitalOutput,
                          "HALSIM " + kind_.label + "[" + std::to_string(module) + "].solenoid[" + std::to_string(ch) + "]",
                          kind_.label + " " + std::to_string(module) + " solenoid " + std::to_string(ch), kind_.vendor);
}

std::vector<sdk::SignalSpec> SolenoidBridge::signalsOf(int) const {
    return {sig(sdk::signal::kSolenoid, SignalDirection::Output, SignalType::Bool)};
}

double SolenoidBridge::readSignal(int flatChannel, std::size_t) const {
    const int perModule = channelsPerModule();
    return fromBool(hal_->solenoidOutput(flatChannel / perModule, flatChannel % perModule));
}

}  // namespace wpiemu::bridges::hal
