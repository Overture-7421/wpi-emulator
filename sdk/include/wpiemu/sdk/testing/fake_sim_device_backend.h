#pragma once

// In-memory ISimDeviceBackend for bridge tests: a HAL SimDevice tree with named values, a write log and test-side
// pokes. Thread-safe, so live-poll tests can create devices while the bridge polls.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/devices/sim_device_bridge.h"

namespace wpiemu::sdk::testing {

struct FakeValueSpec {
    std::string name;
    SignalDirection direction = SignalDirection::Input;
    SignalType type = SignalType::Double;
    double initial = 0.0;
};

class FakeSimDeviceBackend final : public ISimDeviceBackend {
public:
    std::uint64_t generation() const override { return generation_.load(); }
    void listDevices(std::vector<SimDeviceInfo>& out) const override {
        std::lock_guard lock(mutex_);
        ++listDeviceCalls;
        for (const auto& d : devices_) out.push_back(d);
    }
    void listValues(SimDeviceHandle device, std::vector<SimValueInfo>& out) const override {
        std::lock_guard lock(mutex_);
        const auto it = values_.find(device);
        if (it == values_.end()) return;
        for (const auto& v : it->second) out.push_back(v);
    }
    double getValue(SimValueHandle v) const override {
        std::lock_guard lock(mutex_);
        const auto it = data_.find(v);
        return it == data_.end() ? 0.0 : it->second;
    }
    void setValue(SimValueHandle v, double d) override {
        {
            std::lock_guard lock(mutex_);
            writes.emplace_back(v, d);
            data_[v] = d;
        }
        if (onWrite) onWrite(v, d);
    }

    SimDeviceHandle addDevice(const std::string& name, const std::vector<FakeValueSpec>& values) {
        std::lock_guard lock(mutex_);
        const SimDeviceHandle h{nextHandle_++};
        devices_.push_back({h, name});
        auto& list = values_[h];
        for (const auto& spec : values) {
            const SimValueHandle vh{nextHandle_++};
            list.push_back({vh, spec.name, spec.direction, spec.type});
            data_[vh] = spec.initial;
            byName_[{h, spec.name}] = vh;
        }
        ++generation_;
        return h;
    }
    // A value created after its device (the half-initialised window of a live poll). Bumps the generation like the
    // HAL binding's value-created callback does.
    SimValueHandle addValue(SimDeviceHandle device, const FakeValueSpec& spec) {
        std::lock_guard lock(mutex_);
        const SimValueHandle vh{nextHandle_++};
        values_[device].push_back({vh, spec.name, spec.direction, spec.type});
        data_[vh] = spec.initial;
        byName_[{device, spec.name}] = vh;
        ++generation_;
        return vh;
    }
    void removeDevice(SimDeviceHandle h) {
        std::lock_guard lock(mutex_);
        std::erase_if(devices_, [&](const SimDeviceInfo& d) { return d.handle == h; });
        values_.erase(h);
        ++generation_;
    }

    SimValueHandle handle(SimDeviceHandle h, const std::string& valueName) const {
        std::lock_guard lock(mutex_);
        return byName_.at({h, valueName});
    }
    double get(SimDeviceHandle h, const std::string& valueName) const { return getValue(handle(h, valueName)); }
    // Test-side poke: changes a value without being logged as a bridge write (simulates the vendor mirror or a
    // robot-side simulation writing it).
    void poke(SimDeviceHandle h, const std::string& valueName, double v) {
        const SimValueHandle vh = handle(h, valueName);
        std::lock_guard lock(mutex_);
        data_[vh] = v;
    }
    std::size_t writesTo(SimDeviceHandle h, const std::string& valueName) const {
        const SimValueHandle vh = handle(h, valueName);
        std::lock_guard lock(mutex_);
        std::size_t n = 0;
        for (const auto& w : writes) n += w.first == vh ? 1u : 0u;
        return n;
    }

    std::vector<std::pair<SimValueHandle, double>> writes;
    std::function<void(SimValueHandle, double)> onWrite;
    mutable int listDeviceCalls = 0;

private:
    mutable std::mutex mutex_;
    std::atomic<std::uint64_t> generation_{0};
    std::uint64_t nextHandle_ = 1;
    std::vector<SimDeviceInfo> devices_;
    std::map<SimDeviceHandle, std::vector<SimValueInfo>> values_;
    std::map<std::pair<SimDeviceHandle, std::string>, SimValueHandle> byName_;
    std::map<SimValueHandle, double> data_;
};

}  // namespace wpiemu::sdk::testing
