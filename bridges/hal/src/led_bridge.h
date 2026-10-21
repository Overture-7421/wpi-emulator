#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "hal_family_bridge.h"
#include "wpiemu/bridges/hal/hal_api.h"

namespace wpiemu::bridges::hal {

// Addressable LED strip (observe only). Per-pixel data is summarised (average colour + first pixel)
// so a 1000-pixel strip does not become 3000 signals.
class AddressableLedBridge final : public IndexedHalBridge<IAddressableLedHal> {
public:
    explicit AddressableLedBridge(std::shared_ptr<IAddressableLedHal> hal)
        : IndexedHalBridge("hal.addressable_led", std::move(hal)) {}

    sdk::Status start(sdk::IBridgeHost& host) override;
    void collectOutputs(std::vector<sdk::SignalSample>& out) override;

protected:
    sdk::DeviceDescriptor describe(int index) const override;
    std::vector<sdk::SignalSpec> signalsOf(int index) const override;
    double readSignal(int index, std::size_t signalIndex) const override;

private:
    struct Summary {
        double avgR = 0.0;
        double avgG = 0.0;
        double avgB = 0.0;
        double firstRgb = 0.0;
    };

    std::vector<Summary> summaries_;
    std::vector<LedPixel> scratch_;
};

}  // namespace wpiemu::bridges::hal
