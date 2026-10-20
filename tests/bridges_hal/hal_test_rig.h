#pragma once

// Test rig for the HAL bridges: registers every bridge against a FakeHalApi, starts them against a
// recording host, and mimics the extension's routing (samples are only delivered to the owner).

#include <doctest/doctest.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/bridges/hal/can_ids.h"
#include "wpiemu/bridges/hal/hal_bridges.h"
#include "wpiemu/bridges/hal/testing/fake_hal_api.h"
#include "wpiemu/sdk/devices/signals.h"
#include "wpiemu/sdk/testing/fake_bridge_host.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"

namespace hal_test {

namespace sdk = wpiemu::sdk;
namespace hal = wpiemu::bridges::hal;

using RecordingHost = sdk::testing::FakeBridgeHost;

class Rig {
public:
    explicit Rig(hal::HalBridgeOptions options = {}) : api(std::make_shared<hal::testing::FakeHalApi>()) {
        REQUIRE(hal::registerHalBridges(registry, api, options).ok());
        auto all = registry.createAll();
        REQUIRE(all.ok());
        bridges = std::move(all).value();
        for (auto& b : bridges) REQUIRE(b->start(host).ok());
    }
    ~Rig() {
        for (auto& b : bridges) b->stop();
    }

    void poll() {
        for (auto& b : bridges) b->poll();
    }

    std::vector<sdk::SignalSample> outputs() {
        std::vector<sdk::SignalSample> out;
        for (auto& b : bridges) b->collectOutputs(out);
        return out;
    }

    // Routes each sample to the bridge that owns the device, like the extension does.
    void apply(const std::vector<sdk::SignalSample>& samples) {
        for (const auto& s : samples)
            for (auto& b : bridges)
                if (b->owns(s.device)) b->applyInputs(std::span<const sdk::SignalSample>(&s, 1));
    }

    sdk::IDeviceBridge& bridge(const std::string& name) {
        for (auto& b : bridges)
            if (b->name() == name) return *b;
        FAIL("no bridge " << name);
        return *bridges.front();
    }

    std::optional<sdk::DeviceId> find(const std::string& type, int id, int channel = -1) const {
        for (const auto& [did, a] : host.announced)
            if (a.descriptor.type == type && a.descriptor.id == id && a.descriptor.channel == channel) return did;
        return std::nullopt;
    }

    const RecordingHost::Announced& info(sdk::DeviceId id) const { return host.announced.at(id); }

    std::size_t countType(const std::string& type) const {
        std::size_t n = 0;
        for (const auto& [did, a] : host.announced) n += a.descriptor.type == type ? 1u : 0u;
        return n;
    }

    // Value of a named signal of a device in the current outputs, if it is an output signal.
    std::optional<double> output(sdk::DeviceId id, const std::string& signal) {
        std::uint16_t idx = 0;
        if (sdk::findSignal(info(id).signalSpecs, signal, &idx) == nullptr) return std::nullopt;
        for (const auto& s : outputs())
            if (s.device == id && s.signal == idx) return s.value;
        return std::nullopt;
    }

    // Delivers a value to a named signal; returns false if the signal does not exist.
    bool write(sdk::DeviceId id, const std::string& signal, double value) {
        std::uint16_t idx = 0;
        if (sdk::findSignal(info(id).signalSpecs, signal, &idx) == nullptr) return false;
        apply({{id, idx, value}});
        return true;
    }

    std::shared_ptr<hal::testing::FakeHalApi> api;
    sdk::DeviceBridgeRegistry registry;
    RecordingHost host;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
};

}  // namespace hal_test
