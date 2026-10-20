#pragma once

// Helper base for HAL families that are addressed by a small integer channel/index (PWM, DIO,
// AnalogIn, Relay, Encoder, PCM/PH solenoids...). The derived class only answers "is channel N
// initialised?", describes it, and reads/writes its signals through HALSIM_* calls; the base
// handles live-flag diffing (announce on init, retire on free), DeviceId bookkeeping and sample
// (de)multiplexing. (docs/reference/extension-ipc.md: poll the per-index Initialized flag; HAL callbacks only set a dirty flag.)

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::sdk {

class ChannelFamilyBridge : public IDeviceBridge {
public:
    explicit ChannelFamilyBridge(std::string name) : name_(std::move(name)) {}

    std::string_view name() const override { return name_; }

    Status start(IBridgeHost& host) override {
        host_ = &host;
        slots_.assign(static_cast<std::size_t>(channelCount()), Slot{});
        return okStatus();
    }

    void poll() override {
        if (host_ == nullptr) return;
        for (int ch = 0; ch < channelCount(); ++ch) {
            Slot& s = slots_[static_cast<std::size_t>(ch)];
            const bool init = isInitialized(ch);
            if (init && !s.live) {
                s.signalSpecs = signalsOf(ch);
                s.id = host_->announce(describe(ch), s.signalSpecs);
                s.live = true;
            } else if (!init && s.live) {
                host_->retire(s.id);
                s.live = false;
            }
        }
    }

    void collectOutputs(std::vector<SignalSample>& out) override {
        for (std::size_t ch = 0; ch < slots_.size(); ++ch) {
            const Slot& s = slots_[ch];
            if (!s.live) continue;
            for (std::size_t i = 0; i < s.signalSpecs.size(); ++i)
                if (s.signalSpecs[i].direction != SignalDirection::Input)
                    out.push_back({s.id, static_cast<std::uint16_t>(i), readSignal(static_cast<int>(ch), i)});
        }
    }

    void applyInputs(std::span<const SignalSample> samples) override {
        for (const auto& sample : samples) {
            const int ch = channelOf(sample.device);
            if (ch < 0) continue;
            const Slot& s = slots_[static_cast<std::size_t>(ch)];
            if (sample.signal < s.signalSpecs.size() && s.signalSpecs[sample.signal].direction != SignalDirection::Output)
                writeSignal(ch, sample.signal, sample.value);
        }
    }

    void applyLoopback(DeviceId id) override {
        const int ch = channelOf(id);
        if (ch >= 0) loopback(ch);
    }

    bool owns(DeviceId id) const override { return channelOf(id) >= 0; }

    void stop() noexcept override {
        host_ = nullptr;
        slots_.clear();
    }

protected:
    virtual int channelCount() const = 0;                      // HAL_GetNum*()
    virtual bool isInitialized(int channel) const = 0;
    virtual DeviceDescriptor describe(int channel) const = 0;  // family, type, vendor, id/channel, capabilities
    virtual std::vector<SignalSpec> signalsOf(int channel) const = 0;
    virtual double readSignal(int channel, std::size_t signalIndex) const = 0;
    virtual void writeSignal(int channel, std::size_t signalIndex, double value) = 0;
    // Default: nothing to echo (output-only devices). DIO-style bidirectional signals already echo.
    virtual void loopback(int /*channel*/) {}

private:
    struct Slot {
        DeviceId id;
        bool live = false;
        std::vector<SignalSpec> signalSpecs;
    };
    int channelOf(DeviceId id) const {
        for (std::size_t i = 0; i < slots_.size(); ++i)
            if (slots_[i].live && slots_[i].id == id) return static_cast<int>(i);
        return -1;
    }

    std::string name_;
    IBridgeHost* host_ = nullptr;
    std::vector<Slot> slots_;
};

}  // namespace wpiemu::sdk
