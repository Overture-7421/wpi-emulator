#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "wpiemu/sdk/devices/channel_family_bridge.h"

namespace wpiemu::bridges::hal {

// A HAL family over one HAL seam. Most families only observe the robot or only drive it, so reads default to 0 and
// writes to nothing; a family overrides the direction it has.
template <class Hal>
class HalFamilyBridge : public sdk::ChannelFamilyBridge {
protected:
    HalFamilyBridge(std::string name, std::shared_ptr<Hal> hal)
        : ChannelFamilyBridge(std::move(name)), hal_(std::move(hal)) {}

    double readSignal(int, std::size_t) const override { return 0.0; }
    void writeSignal(int, std::size_t, double) override {}

    std::shared_ptr<Hal> hal_;
};

// The common HALSIM shape: `channelCount()` channels, each with an `initialized(channel)` flag.
template <class Hal>
class IndexedHalBridge : public HalFamilyBridge<Hal> {
protected:
    using HalFamilyBridge<Hal>::HalFamilyBridge;

    int channelCount() const override { return this->hal_->channelCount(); }
    bool isInitialized(int channel) const override { return this->hal_->initialized(channel); }
};

}  // namespace wpiemu::bridges::hal
