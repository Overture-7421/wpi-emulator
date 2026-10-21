#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/devices/signal.h"
#include "wpiemu/sdk/ipc/session_messages.h"

namespace wpiemu::halsim {

// Session-unique DeviceIds and the device -> owning bridge routing table. Announces are idempotent
// by DeviceDescriptor::key() (a repeat with identical content sends nothing, a changed descriptor
// or an un-retire re-sends). Thread-safe: bridges may announce from any thread, but per the SDK they
// do it from poll() on the clock thread.
class DeviceTable {
public:
    using AnnounceSink = std::function<void(sdk::proto::DeviceAnnounce&&)>;

    // Where announcements go (the outbound IPC queue). Empty detaches; devices announced while
    // detached are re-sent by snapshot() after the next connection.
    void setSink(AnnounceSink sink);

    sdk::DeviceId announce(std::size_t bridge, const sdk::DeviceDescriptor& descriptor,
                           std::vector<sdk::SignalSpec> signalSpecs);
    void retire(sdk::DeviceId id);
    void observeBus(sdk::DeviceId id, std::string_view bus);

    // Owning bridge of a live device; nullopt for unknown or retired devices.
    std::optional<std::size_t> ownerOf(sdk::DeviceId id) const;
    // Every live device as a DeviceAnnounce (used to re-announce after a reconnect).
    std::vector<sdk::proto::DeviceAnnounce> snapshot() const;
    std::size_t liveCount() const;

private:
    struct Entry {
        sdk::proto::DeviceAnnounce announce;
        std::size_t bridge = 0;
    };

    void emitLocked(const Entry& entry);

    mutable std::mutex mutex_;
    std::vector<Entry> entries_;                 // index = DeviceId::value - 1
    std::map<sdk::DeviceKey, std::size_t> byKey_;
    AnnounceSink sink_;
};

}  // namespace wpiemu::halsim
