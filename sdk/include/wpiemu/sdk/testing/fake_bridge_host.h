#pragma once

// Recording IBridgeHost for bridge tests: what was announced, retired and observed, plus lookups by device and signal.

#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"

namespace wpiemu::sdk::testing {

class FakeBridgeHost : public IBridgeHost {
public:
    struct Announced {
        DeviceDescriptor descriptor;
        std::vector<SignalSpec> signalSpecs;
    };

    DeviceId announce(const DeviceDescriptor& d, std::vector<SignalSpec> specs) override {
        for (auto& [id, a] : announced) {
            if (a.descriptor.key() == d.key()) {
                a.descriptor = d;
                a.signalSpecs = std::move(specs);
                ++reannounces;
                return id;
            }
        }
        const DeviceId id{next++};
        announced[id] = {d, std::move(specs)};
        return id;
    }
    void retire(DeviceId id) override {
        retired.push_back(id);
        if (eraseOnRetire) announced.erase(id);
    }
    ILogger& logger() override { return log; }
    bool enableOptionalHooks() const override { return hooksEnabled; }
    void observeBus(DeviceId id, std::string_view bus) override { observed.emplace_back(id, std::string(bus)); }

    // The announced device of this type and id, or an invalid DeviceId.
    DeviceId idOf(std::string_view type, int deviceId) const {
        for (const auto& [id, a] : announced)
            if (a.descriptor.type == type && a.descriptor.id == deviceId) return id;
        return DeviceId{};
    }
    std::uint16_t signalIndex(DeviceId id, const std::string& name) const {
        std::uint16_t idx = 0xFFFF;
        const auto it = announced.find(id);
        if (it != announced.end()) findSignal(it->second.signalSpecs, name, &idx);
        return idx;
    }
    const SignalSpec* signalSpec(DeviceId id, const std::string& name) const {
        const auto it = announced.find(id);
        return it == announced.end() ? nullptr : findSignal(it->second.signalSpecs, name);
    }

    std::map<DeviceId, Announced> announced;
    std::vector<DeviceId> retired;
    std::vector<std::pair<DeviceId, std::string>> observed;
    int reannounces = 0;
    std::uint32_t next = 1;
    bool hooksEnabled = false;   // what enableOptionalHooks() answers
    bool eraseOnRetire = false;  // true: a retired device also leaves `announced`
    CapturingLogger log;
};

}  // namespace wpiemu::sdk::testing
