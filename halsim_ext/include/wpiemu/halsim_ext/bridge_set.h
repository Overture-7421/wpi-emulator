#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/halsim_ext/device_table.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::halsim {

// IBridgeHost handed to ONE bridge: stamps every announce with the bridge's index so the table can
// route samples back to it.
class BridgeHost final : public sdk::IBridgeHost {
public:
    BridgeHost(std::size_t bridgeIndex, DeviceTable& table, sdk::ILogger& logger,
               const std::atomic<bool>& optionalHooks)
        : index_(bridgeIndex), table_(&table), logger_(&logger), optionalHooks_(&optionalHooks) {}

    sdk::DeviceId announce(const sdk::DeviceDescriptor& descriptor, std::vector<sdk::SignalSpec> signalSpecs) override {
        return table_->announce(index_, descriptor, std::move(signalSpecs));
    }
    void retire(sdk::DeviceId id) override { table_->retire(id); }
    sdk::ILogger& logger() override { return *logger_; }
    bool enableOptionalHooks() const override { return optionalHooks_->load(std::memory_order_relaxed); }
    void observeBus(sdk::DeviceId id, std::string_view busLabel) override { table_->observeBus(id, busLabel); }

private:
    std::size_t index_;
    DeviceTable* table_;
    sdk::ILogger* logger_;
    const std::atomic<bool>* optionalHooks_;
};

// Owns the live bridges and fans the per-tick calls out to them. All methods except `start` run on
// the clock thread (`start` runs once on the thread that initialises the extension).
class BridgeSet {
public:
    BridgeSet(std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges, DeviceTable& table, sdk::ILogger& logger);
    ~BridgeSet();
    BridgeSet(const BridgeSet&) = delete;
    BridgeSet& operator=(const BridgeSet&) = delete;

    // Starts every bridge; a bridge whose start() fails is logged and dropped, never fatal.
    void start(const std::atomic<bool>& optionalHooks);
    void poll();
    void resetInputHistory();
    // Routes each sample to the bridge that announced the device; unknown/retired devices are skipped.
    void applyInputs(std::span<const sdk::SignalSample> samples);
    void collectOutputs(std::vector<sdk::SignalSample>& out);
    void applyLoopback(std::span<const sdk::DeviceId> ids);
    // Idempotent; stops bridges in reverse start order.
    void stop() noexcept;

    std::size_t activeCount() const;

private:
    struct Slot {
        std::unique_ptr<sdk::IDeviceBridge> bridge;
        std::unique_ptr<BridgeHost> host;
        bool active = false;
    };

    DeviceTable* table_;
    sdk::ILogger* logger_;
    std::vector<Slot> slots_;
    std::vector<std::vector<sdk::SignalSample>> routed_;  // per-bridge scratch, reused each tick
};

}  // namespace wpiemu::halsim
