#pragma once

#include <memory>
#include <vector>

#include "wpiemu/bridges/hal/hal_api.h"
#include "wpiemu/sdk/devices/channel_family_bridge.h"

namespace wpiemu::bridges::hal {

// Addressable LED strip (observe only). Per-pixel data is summarised (average colour + first pixel)
// so a 1000-pixel strip does not become 3000 signals.
class AddressableLedBridge final : public sdk::ChannelFamilyBridge {
public:
    explicit AddressableLedBridge(std::shared_ptr<IAddressableLedHal> hal);

    sdk::Status start(sdk::IBridgeHost& host) override;
    void collectOutputs(std::vector<sdk::SignalSample>& out) override;

protected:
    int channelCount() const override;
    bool isInitialized(int index) const override;
    sdk::DeviceDescriptor describe(int index) const override;
    std::vector<sdk::SignalSpec> signalsOf(int index) const override;
    double readSignal(int index, std::size_t signalIndex) const override;
    void writeSignal(int, std::size_t, double) override {}

private:
    struct Summary {
        double avgR = 0.0;
        double avgG = 0.0;
        double avgB = 0.0;
        double firstRgb = 0.0;
    };

    std::shared_ptr<IAddressableLedHal> hal_;
    std::vector<Summary> summaries_;
    std::vector<LedPixel> scratch_;
};

}  // namespace wpiemu::bridges::hal
