#include "wpiemu/halsim_ext/device_table.h"

#include <utility>

namespace wpiemu::halsim {

void DeviceTable::setSink(AnnounceSink sink) {
    std::lock_guard lock(mutex_);
    sink_ = std::move(sink);
}

void DeviceTable::emitLocked(const Entry& entry) {
    if (sink_) sink_(sdk::proto::DeviceAnnounce(entry.announce));
}

sdk::DeviceId DeviceTable::announce(std::size_t bridge, const sdk::DeviceDescriptor& descriptor,
                                    std::vector<sdk::SignalSpec> signalSpecs) {
    std::lock_guard lock(mutex_);
    const sdk::DeviceKey key = descriptor.key();
    const auto found = byKey_.find(key);
    if (found == byKey_.end()) {
        Entry entry;
        entry.bridge = bridge;
        entry.announce.id = sdk::DeviceId(static_cast<std::uint32_t>(entries_.size() + 1));
        entry.announce.descriptor = descriptor;
        entry.announce.signalSpecs = std::move(signalSpecs);
        entries_.push_back(std::move(entry));
        byKey_.emplace(key, entries_.size() - 1);
        emitLocked(entries_.back());
        return entries_.back().announce.id;
    }

    Entry& entry = entries_[found->second];
    const bool changed = entry.announce.retired || !(entry.announce.descriptor == descriptor) ||
                         entry.announce.signalSpecs != signalSpecs;
    entry.announce.retired = false;
    entry.announce.descriptor = descriptor;
    entry.announce.signalSpecs = std::move(signalSpecs);
    // The first claimant keeps ownership: a second bridge announcing the same key is a bridge bug.
    if (changed) emitLocked(entry);
    return entry.announce.id;
}

void DeviceTable::retire(sdk::DeviceId id) {
    std::lock_guard lock(mutex_);
    if (!id.valid() || id.value > entries_.size()) return;
    Entry& entry = entries_[id.value - 1];
    if (entry.announce.retired) return;
    entry.announce.retired = true;
    emitLocked(entry);
}

void DeviceTable::observeBus(sdk::DeviceId id, std::string_view bus) {
    std::lock_guard lock(mutex_);
    if (!id.valid() || id.value > entries_.size()) return;
    Entry& entry = entries_[id.value - 1];
    if (entry.announce.observedBus == bus) return;
    entry.announce.observedBus = std::string(bus);
    emitLocked(entry);
}

std::optional<std::size_t> DeviceTable::ownerOf(sdk::DeviceId id) const {
    std::lock_guard lock(mutex_);
    if (!id.valid() || id.value > entries_.size()) return std::nullopt;
    const Entry& entry = entries_[id.value - 1];
    if (entry.announce.retired) return std::nullopt;
    return entry.bridge;
}

std::vector<sdk::proto::DeviceAnnounce> DeviceTable::snapshot() const {
    std::lock_guard lock(mutex_);
    std::vector<sdk::proto::DeviceAnnounce> out;
    out.reserve(entries_.size());
    for (const Entry& entry : entries_)
        if (!entry.announce.retired) out.push_back(entry.announce);
    return out;
}

std::size_t DeviceTable::liveCount() const {
    std::lock_guard lock(mutex_);
    std::size_t n = 0;
    for (const Entry& entry : entries_)
        if (!entry.announce.retired) ++n;
    return n;
}

}  // namespace wpiemu::halsim
