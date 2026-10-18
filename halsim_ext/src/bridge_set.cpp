#include "wpiemu/halsim_ext/bridge_set.h"

#include <optional>
#include <string>
#include <utility>

namespace wpiemu::halsim {

namespace {
constexpr const char* kLog = "extension.bridges";
}

BridgeSet::BridgeSet(std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges, DeviceTable& table,
                     sdk::ILogger& logger, const IHalClock& clock)
    : table_(&table), logger_(&logger), clock_(&clock) {
    slots_.reserve(bridges.size());
    for (auto& b : bridges) {
        Slot slot;
        slot.bridge = std::move(b);
        slots_.push_back(std::move(slot));
    }
    routed_.resize(slots_.size());
}

BridgeSet::~BridgeSet() { stop(); }

void BridgeSet::start(const std::atomic<bool>& optionalHooks) {
    for (std::size_t i = 0; i < slots_.size(); ++i) {
        Slot& slot = slots_[i];
        if (!slot.bridge) continue;
        slot.host =
            std::make_unique<BridgeHost>(i, *table_, *logger_, optionalHooks, vision_, *clock_);
        const sdk::Status status = slot.bridge->start(*slot.host);
        if (!status) {
            logger_->error(kLog, "bridge '" + std::string(slot.bridge->name()) + "' failed to start: " +
                                     status.error().message + " (skipped)");
            slot.bridge->stop();
            continue;
        }
        slot.active = true;
    }
}

void BridgeSet::poll() {
    if (closed()) return;
    for (Slot& slot : slots_)
        if (slot.active) slot.bridge->poll();
}

void BridgeSet::resetInputHistory() {
    if (closed()) return;
    for (Slot& slot : slots_)
        if (slot.active) slot.bridge->resetInputHistory();
}

void BridgeSet::applyInputs(std::span<const sdk::SignalSample> samples) {
    if (samples.empty() || closed()) return;
    for (auto& v : routed_) v.clear();
    for (const sdk::SignalSample& s : samples) {
        const std::optional<std::size_t> owner = table_->ownerOf(s.device);
        if (owner && *owner < routed_.size() && slots_[*owner].active) routed_[*owner].push_back(s);
    }
    for (std::size_t i = 0; i < slots_.size(); ++i)
        if (!routed_[i].empty()) slots_[i].bridge->applyInputs(routed_[i]);
}

void BridgeSet::collectOutputs(std::vector<sdk::SignalSample>& out) {
    if (closed()) return;
    for (Slot& slot : slots_)
        if (slot.active) slot.bridge->collectOutputs(out);
}

void BridgeSet::applyVision(std::span<const sdk::VisionFrame> frames) {
    if (frames.empty() || closed()) return;
    for (Slot& slot : slots_)
        if (slot.active) slot.bridge->applyVision(frames);
}

void BridgeSet::collectVision(std::vector<sdk::CameraRobotState>& out, bool full) {
    if (closed()) return;
    for (Slot& slot : slots_)
        if (slot.active) slot.bridge->collectVision(out, full);
}

void BridgeSet::applyLoopback(std::span<const sdk::DeviceId> ids) {
    if (closed()) return;
    for (const sdk::DeviceId id : ids) {
        const std::optional<std::size_t> owner = table_->ownerOf(id);
        if (owner && *owner < slots_.size() && slots_[*owner].active) slots_[*owner].bridge->applyLoopback(id);
    }
}

void BridgeSet::stop() noexcept {
    close();
    for (auto it = slots_.rbegin(); it != slots_.rend(); ++it) {
        if (it->bridge) it->bridge->stop();
        it->active = false;
    }
}

std::size_t BridgeSet::activeCount() const {
    std::size_t n = 0;
    for (const Slot& slot : slots_)
        if (slot.active) ++n;
    return n;
}

}  // namespace wpiemu::halsim
