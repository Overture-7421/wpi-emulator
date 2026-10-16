#pragma once

// Test doubles for the vendor bridge: an in-memory HAL SimDevice tree (names, directions and value names
// copied from the Phoenix 6 26.3.0 wrapper classes) and a recording bridge host.

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/devices/sim_device_bridge.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"

namespace wpiemu::vendor_tests {

using namespace wpiemu::sdk;

struct FakeValueSpec {
    std::string name;
    SignalDirection direction = SignalDirection::Input;
    SignalType type = SignalType::Double;
    double initial = 0.0;
};

class FakeSimBackend final : public ISimDeviceBackend {
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

class FakeHost final : public IBridgeHost {
public:
    struct Announced {
        DeviceDescriptor descriptor;
        std::vector<SignalSpec> signalSpecs;
    };

    DeviceId announce(const DeviceDescriptor& d, std::vector<SignalSpec> signalSpecs) override {
        for (auto& [id, a] : announced) {
            if (a.descriptor.key() == d.key()) {
                a.descriptor = d;
                a.signalSpecs = std::move(signalSpecs);
                ++reannounces;
                return id;
            }
        }
        const DeviceId id{next++};
        announced[id] = {d, std::move(signalSpecs)};
        return id;
    }
    void retire(DeviceId id) override {
        retired.push_back(id);
        announced.erase(id);
    }
    ILogger& logger() override { return log; }
    bool enableOptionalHooks() const override { return hooksEnabled; }
    void observeBus(DeviceId id, std::string_view bus) override { observed.emplace_back(id, std::string(bus)); }

    // Lookup helpers.
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
    bool hooksEnabled = false;
    sdk::testing::CapturingLogger log;
};

// ---- Fixtures: SimDevice trees exactly as the Phoenix 6 26.3.0 wrapper classes create them ---------------------

inline constexpr auto kIn = SignalDirection::Input;
inline constexpr auto kOut = SignalDirection::Output;
inline constexpr auto kBidir = SignalDirection::Bidirectional;

struct TalonFxTree {
    SimDeviceHandle motor, rotor, fwd, rev;
};

inline TalonFxTree addTalonFx(FakeSimBackend& b, int id, const char* model = "Talon FX") {
    const std::string base = std::string(model) + " (v6)[" + std::to_string(id) + "]";
    TalonFxTree t;
    t.motor = b.addDevice("CANMotor:" + base,
                          {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                           {"dutyCycle", kOut},
                           {"motorVoltage", kOut},
                           {"torqueCurrent", kOut},
                           {"supplyCurrent", kOut}});
    t.rotor = b.addDevice("CANEncoder:" + base + "/Rotor Sensor",
                          {{"position", kOut}, {"rawPositionInput", kIn}, {"velocity", kIn}, {"acceleration", kIn}});
    t.fwd = b.addDevice("CANDIO:" + base + "/Fwd Limit",
                        {{"init", kOut, SignalType::Bool, 1}, {"input", kOut, SignalType::Bool, 1}, {"value", kBidir, SignalType::Bool}});
    t.rev = b.addDevice("CANDIO:" + base + "/Rev Limit",
                        {{"init", kOut, SignalType::Bool, 1}, {"input", kOut, SignalType::Bool, 1}, {"value", kBidir, SignalType::Bool}});
    return t;
}

inline SimDeviceHandle addCanCoder(FakeSimBackend& b, int id) {
    return b.addDevice("CANEncoder:CANcoder (v6)[" + std::to_string(id) + "]",
                       {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                        {"position", kOut},
                        {"rawPositionInput", kIn},
                        {"velocity", kIn}});
}

inline SimDeviceHandle addPigeon2(FakeSimBackend& b, int id) {
    return b.addDevice("CANGyro:Pigeon 2 (v6)[" + std::to_string(id) + "]",
                       {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                        {"yaw", kOut},
                        {"rawYawInput", kIn},
                        {"pitch", kIn},
                        {"roll", kIn},
                        {"angularVelX", kIn},
                        {"angularVelY", kIn},
                        {"angularVelZ", kIn}});
}

inline SimDeviceHandle addCanDle(FakeSimBackend& b, int id) {
    return b.addDevice("CAN:CANdle (v6)[" + std::to_string(id) + "]",
                       {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                        {"fiveVRail", kIn, SignalType::Double, 5.0},
                        {"outputCurrent", kIn},
                        {"temperature", kIn, SignalType::Double, 25.0},
                        {"vbatModulation", kOut}});
}

inline SimDeviceHandle addCanRange(FakeSimBackend& b, int id) {
    return b.addDevice("CAN:CANrange (v6)[" + std::to_string(id) + "]",
                       {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                        {"distance", kIn},
                        {"isDetected", kOut, SignalType::Bool}});
}

}  // namespace wpiemu::vendor_tests
