#include "led_bridge.h"

#include <string>

#include "bridge_support.h"
#include "wpiemu/sdk/devices/signals.h"

namespace wpiemu::bridges::hal {

using namespace detail;
using sdk::Capability;
using sdk::SignalDirection;
using sdk::SignalType;

namespace {
enum LedSignal : std::size_t { kRunning, kLength, kOutputPort, kAvgR, kAvgG, kAvgB, kFirstRgb };
}

sdk::Status AddressableLedBridge::start(sdk::IBridgeHost& host) {
    summaries_.assign(static_cast<std::size_t>(channelCount()), Summary{});
    hasOutputPort_ = hal_->hasOutputPort();
    hasRunning_ = hal_->hasRunning();
    return ChannelFamilyBridge::start(host);
}

void AddressableLedBridge::collectOutputs(std::vector<sdk::SignalSample>& out) {
    for (std::size_t i = 0; i < summaries_.size(); ++i) {
        const int index = static_cast<int>(i);
        if (!isInitialized(index)) continue;
        scratch_.clear();
        hal_->readPixels(index, scratch_);
        Summary s;
        if (!scratch_.empty()) {
            double r = 0.0, g = 0.0, b = 0.0;
            for (const LedPixel& p : scratch_) {
                r += p.r;
                g += p.g;
                b += p.b;
            }
            const double n = static_cast<double>(scratch_.size());
            s = Summary{r / n, g / n, b / n,
                        static_cast<double>((static_cast<unsigned>(scratch_[0].r) << 16) |
                                            (static_cast<unsigned>(scratch_[0].g) << 8) | scratch_[0].b)};
        }
        summaries_[i] = s;
    }
    ChannelFamilyBridge::collectOutputs(out);
}

sdk::DeviceDescriptor AddressableLedBridge::describe(int index) const {
    return makeDescriptor("addressable_led", index, -1, Capability::Led, "HALSIM AddressableLED[" + std::to_string(index) + "]",
                          "Addressable LED " + std::to_string(index));
}

std::vector<sdk::SignalSpec> AddressableLedBridge::signalsOf(int) const {
    std::vector<sdk::SignalSpec> result{
        sig("running", SignalDirection::Output, SignalType::Bool),
        sig("length", SignalDirection::Output, SignalType::Int, "leds"),
        sig("outputPort", SignalDirection::Output, SignalType::Int),
        sig("avgRed", SignalDirection::Output, SignalType::Double, "0-255"),
        sig("avgGreen", SignalDirection::Output, SignalType::Double, "0-255"),
        sig("avgBlue", SignalDirection::Output, SignalType::Double, "0-255"),
        sig("firstPixelRgb", SignalDirection::Output, SignalType::Int, "0xRRGGBB"),
    };
    if (!hasOutputPort_) result.erase(result.begin() + kOutputPort);
    if (!hasRunning_) result.erase(result.begin() + kRunning);
    return result;
}

double AddressableLedBridge::readSignal(int index, std::size_t signalIndex) const {
    const Summary& s = summaries_[static_cast<std::size_t>(index)];
    if (!hasRunning_) ++signalIndex;
    if (!hasOutputPort_ && signalIndex >= kOutputPort) ++signalIndex;
    switch (signalIndex) {
        case kRunning: return fromBool(hal_->running(index));
        case kLength: return hal_->length(index);
        case kOutputPort: return hal_->outputPort(index);
        case kAvgR: return s.avgR;
        case kAvgG: return s.avgG;
        case kAvgB: return s.avgB;
        case kFirstRgb: return s.firstRgb;
        default: return 0.0;
    }
}

}  // namespace wpiemu::bridges::hal
