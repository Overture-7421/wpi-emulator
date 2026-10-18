#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/halsim_ext/device_table.h"
#include "wpiemu/halsim_ext/hal_ports.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::halsim {

// The vision setup of the CURRENT connection (HelloAck::vision) and the number of HelloAcks adopted
// so far, shared by every BridgeHost. `adopt` runs between two sessions (the handshake thread; the
// clock thread when it reconnects) and the bridges read on the clock thread, so the content needs
// no lock; the revision is atomic so that any thread may look at it.
class VisionSetupSource {
public:
    // Replaces the setup and bumps the revision, also when the new setup equals the old one.
    void adopt(const sdk::VisionSetup& setup) {
        setup_ = setup;
        revision_.fetch_add(1, std::memory_order_release);
    }
    const sdk::VisionSetup& setup() const { return setup_; }
    std::uint64_t revision() const { return revision_.load(std::memory_order_acquire); }

private:
    sdk::VisionSetup setup_;
    std::atomic<std::uint64_t> revision_{0};
};

// IBridgeHost handed to ONE bridge: stamps every announce with the bridge's index so the table can
// route samples back to it.
class BridgeHost final : public sdk::IBridgeHost {
public:
    BridgeHost(std::size_t bridgeIndex, DeviceTable& table, sdk::ILogger& logger,
               const std::atomic<bool>& optionalHooks, const VisionSetupSource& vision,
               const IHalClock& clock)
        : index_(bridgeIndex),
          table_(&table),
          logger_(&logger),
          optionalHooks_(&optionalHooks),
          vision_(&vision),
          clock_(&clock) {}

    sdk::DeviceId announce(const sdk::DeviceDescriptor& descriptor, std::vector<sdk::SignalSpec> signalSpecs) override {
        return table_->announce(index_, descriptor, std::move(signalSpecs));
    }
    void retire(sdk::DeviceId id) override { table_->retire(id); }
    sdk::ILogger& logger() override { return *logger_; }
    bool enableOptionalHooks() const override { return optionalHooks_->load(std::memory_order_relaxed); }
    void observeBus(sdk::DeviceId id, std::string_view busLabel) override { table_->observeBus(id, busLabel); }
    const sdk::VisionSetup& visionSetup() const override { return vision_->setup(); }
    std::uint64_t visionSetupRevision() const override { return vision_->revision(); }
    // The condition SimSession waits for before it polls at program start: the HAL's own flag, an
    // atomic read that is safe from any thread.
    bool programStarted() const override { return clock_->programStarted(); }

private:
    std::size_t index_;
    DeviceTable* table_;
    sdk::ILogger* logger_;
    const std::atomic<bool>* optionalHooks_;
    const VisionSetupSource* vision_;
    const IHalClock* clock_;
};

// Owns the live bridges and fans the per-tick calls out to them. All methods except `start` and
// `adoptVisionSetup` run on the clock thread (`start` runs once on the thread that initialises the
// extension). `clock` must outlive the set: the hosts answer IBridgeHost::programStarted() from it.
class BridgeSet {
public:
    BridgeSet(std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges, DeviceTable& table,
              sdk::ILogger& logger, const IHalClock& clock);
    ~BridgeSet();
    BridgeSet(const BridgeSet&) = delete;
    BridgeSet& operator=(const BridgeSet&) = delete;

    // Starts every bridge; a bridge whose start() fails is logged and dropped, never fatal.
    void start(const std::atomic<bool>& optionalHooks);
    // Adopts the vision setup of an accepted HelloAck (what IBridgeHost::visionSetup() returns from
    // now on) and bumps the revision. Never while a bridge call runs: the runtime calls it right
    // after the handshake, before the first frame of that connection.
    void adoptVisionSetup(const sdk::VisionSetup& setup) { vision_.adopt(setup); }
    void poll();
    void resetInputHistory();
    // Routes each sample to the bridge that announced the device; unknown/retired devices are skipped.
    void applyInputs(std::span<const sdk::SignalSample> samples);
    // Camera names are not devices, so there is no routing: every active bridge sees all frames and
    // keeps its own cameras. No frames, no call.
    void applyVision(std::span<const sdk::VisionFrame> frames);
    void collectOutputs(std::vector<sdk::SignalSample>& out);
    // Every active bridge appends (never clears) what the robot did with its cameras.
    void collectVision(std::vector<sdk::CameraRobotState>& out, bool full);
    void applyLoopback(std::span<const sdk::DeviceId> ids);
    // Any thread. From now on every fan-out above is a no-op: a call that is already running cannot
    // be interrupted, but no new one starts. The runtime closes the set first thing when the HAL
    // shutdown hook stops the extension, so that a clock thread which is stuck and gets detached
    // cannot drive the bridges (and through them ntcore, which may be torn down right after) once
    // the hook returned.
    void close() noexcept { closed_.store(true, std::memory_order_release); }
    bool closed() const { return closed_.load(std::memory_order_acquire); }
    // Idempotent; closes the set, then stops bridges in reverse start order.
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
    const IHalClock* clock_;
    VisionSetupSource vision_;
    std::atomic<bool> closed_{false};
    std::vector<Slot> slots_;
    std::vector<std::vector<sdk::SignalSample>> routed_;  // per-bridge scratch, reused each tick
};

}  // namespace wpiemu::halsim
