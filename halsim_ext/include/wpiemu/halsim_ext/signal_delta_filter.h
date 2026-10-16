#pragma once

#include <bit>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "wpiemu/sdk/devices/signal.h"

namespace wpiemu::halsim {

// Delta-compresses the per-tick output snapshot (IDeviceBridge::collectOutputs delivers every
// signal every tick): `filter` keeps only the samples that changed since the previous frame. A
// "full" frame (first one, after reset(), or when `forceFull`) passes everything through.
class SignalDeltaFilter {
public:
    // Replaces `samples` with the samples to transmit; returns true when the result is a FULL frame.
    bool filter(std::vector<sdk::SignalSample>& samples, bool forceFull) {
        const bool full = forceFull || !primed_;
        std::size_t kept = 0;
        for (const sdk::SignalSample& s : samples) {
            const std::uint64_t key = keyOf(s);
            const std::uint64_t bits = std::bit_cast<std::uint64_t>(s.value);
            auto [it, inserted] = last_.try_emplace(key, bits);
            const bool changed = inserted || it->second != bits;
            it->second = bits;
            if (full || changed) samples[kept++] = s;
        }
        samples.resize(kept);
        primed_ = true;
        return full;
    }

    // Next frame is a full one (reset, reconnect, a lost frame).
    void reset() {
        last_.clear();
        primed_ = false;
    }

    // Forget a retired device so its signals are re-sent if it comes back.
    void forget(sdk::DeviceId id) {
        for (auto it = last_.begin(); it != last_.end();) {
            if (static_cast<std::uint32_t>(it->first >> 16) == id.value) it = last_.erase(it);
            else ++it;
        }
    }

private:
    static std::uint64_t keyOf(const sdk::SignalSample& s) {
        return (static_cast<std::uint64_t>(s.device.value) << 16) | s.signal;
    }

    std::unordered_map<std::uint64_t, std::uint64_t> last_;
    bool primed_ = false;
};

}  // namespace wpiemu::halsim
