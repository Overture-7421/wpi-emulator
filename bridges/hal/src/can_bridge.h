#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include "wpiemu/bridges/hal/can_ids.h"
#include "wpiemu/bridges/hal/hal_api.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::bridges::hal {

// Observe-only view of frames the robot sends through the HAL CAN API (HAL_CAN_SendMessage / CANAPI /
// WPILib CAN class). One device per (manufacturer, device type, device id). Vendor libraries that use
// SimDevices (Phoenix 6) never show up here. HALSIM 2026.2.1 callbacks carry no bus id, so every device
// is on "rio". Frames are recorded on robot threads and announced from poll() on the clock thread.
class CanFrameBridge final : public sdk::IDeviceBridge, private ICanFrameListener {
public:
    explicit CanFrameBridge(std::shared_ptr<ICanHal> hal);
    ~CanFrameBridge() override;

    std::string_view name() const override { return "hal.can"; }
    sdk::Status start(sdk::IBridgeHost& host) override;
    void poll() override;
    void collectOutputs(std::vector<sdk::SignalSample>& out) override;
    void applyInputs(std::span<const sdk::SignalSample>) override {}
    void applyLoopback(sdk::DeviceId) override {}
    bool owns(sdk::DeviceId id) const override;
    void stop() noexcept override;

private:
    using Key = std::tuple<int, int, int>;  // manufacturer, device type, device id
    struct State {
        sdk::DeviceId id;
        bool announced = false;
        std::uint64_t frameCount = 0;
        int lastApi = 0;
        int lastPeriodMs = 0;
        std::uint32_t lastLow = 0;
        std::uint32_t lastHigh = 0;
    };

    void onFrame(const CanFrame& frame) override;

    static constexpr std::size_t kMaxDevices = 512;

    std::shared_ptr<ICanHal> hal_;
    sdk::IBridgeHost* host_ = nullptr;
    std::unique_ptr<ICanSubscription> subscription_;
    mutable std::mutex mutex_;
    std::map<Key, State> devices_;
    std::vector<Key> pending_;
};

}  // namespace wpiemu::bridges::hal
