#pragma once

#include <memory>
#include <string>
#include <vector>

#include "wpiemu/bridges/hal/hal_api.h"
#include "wpiemu/sdk/devices/channel_family_bridge.h"

namespace wpiemu::bridges::hal {

// Naming for one pneumatics technology (the HAL data shape of CTRE PCM and REV PH is identical).
struct PneumaticsKind {
    std::string bridgeName;      // "hal.ctre_pcm"
    std::string moduleType;      // "ctre_pcm"
    std::string solenoidType;    // "ctre_pcm_solenoid"
    std::string vendor;          // "ctre"
    std::string label;           // "PCM"

    static PneumaticsKind ctrePcm() { return {"hal.ctre_pcm", "ctre_pcm", "ctre_pcm_solenoid", "ctre", "PCM"}; }
    static PneumaticsKind revPh() { return {"hal.rev_ph", "rev_ph", "rev_ph_solenoid", "rev", "PH"}; }
};

// One device per live module: compressor state (observed) and pressure switch / compressor current
// (driven by the sim). The module id is the CAN id the robot passed to the PCM/PH constructor.
class PneumaticsModuleBridge final : public sdk::ChannelFamilyBridge {
public:
    PneumaticsModuleBridge(std::shared_ptr<IPneumaticsHal> hal, PneumaticsKind kind);

protected:
    int channelCount() const override;
    bool isInitialized(int module) const override;
    sdk::DeviceDescriptor describe(int module) const override;
    std::vector<sdk::SignalSpec> signalsOf(int module) const override;
    double readSignal(int module, std::size_t signalIndex) const override;
    void writeSignal(int module, std::size_t signalIndex, double value) override;

private:
    std::shared_ptr<IPneumaticsHal> hal_;
    PneumaticsKind kind_;
};

// One device per solenoid channel (DeviceKey.id = module, channel = solenoid). The sim HAL has no
// per-channel "initialised" flag: a channel is announced the first time it is commanded ON (latched
// until the module is freed), or immediately for every channel with announceAll.
class SolenoidBridge final : public sdk::ChannelFamilyBridge {
public:
    SolenoidBridge(std::shared_ptr<IPneumaticsHal> hal, PneumaticsKind kind, bool announceAll);

    sdk::Status start(sdk::IBridgeHost& host) override;
    void poll() override;

protected:
    int channelCount() const override;
    bool isInitialized(int flatChannel) const override;
    sdk::DeviceDescriptor describe(int flatChannel) const override;
    std::vector<sdk::SignalSpec> signalsOf(int flatChannel) const override;
    double readSignal(int flatChannel, std::size_t signalIndex) const override;
    void writeSignal(int, std::size_t, double) override {}

private:
    int channelsPerModule() const;

    std::shared_ptr<IPneumaticsHal> hal_;
    PneumaticsKind kind_;
    bool announceAll_;
    std::vector<bool> latched_;
};

}  // namespace wpiemu::bridges::hal
