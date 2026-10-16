#pragma once

// Late-value decorator over an ISimDeviceBackend.
//
// The SDK SimDeviceBridge lists the values of a SimDevice exactly once, when it first sees the device. Vendor
// wrappers create a SimDevice first and its values a moment later (on the robot thread), so a poll that runs
// WHILE the robot constructor is still creating devices (live discovery, free-running modes) can observe a
// half-initialised device, and the values created after that listing would be lost for good.
//
// This decorator makes polling safe at any moment without timing heuristics:
//  * a device is reported as soon as it exists (nothing waits for the robot);
//  * values that appear later are reported as an additional pseudo-device with the SAME name and a synthetic
//    handle that only lists the new values. The bridge's rule matches it exactly like the original (same key),
//    folds it into the same DeviceDescriptor and appends the new signals (append-only indices), so the device is
//    simply re-announced with more signals;
//  * synthetic handles live as long as their base device and are dropped with it.
// The inner backend must bump generation() whenever a device or a value is created/freed (the HAL binding
// registers value-created callbacks for that), otherwise nothing is rescanned.

#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <vector>

#include "wpiemu/sdk/devices/sim_device_bridge.h"

namespace wpiemu::bridges::vendor {

class IncrementalSimDeviceBackend final : public sdk::ISimDeviceBackend {
public:
    explicit IncrementalSimDeviceBackend(sdk::ISimDeviceBackend& inner) : inner_(&inner) {}

    std::uint64_t generation() const override { return inner_->generation(); }
    void listDevices(std::vector<sdk::SimDeviceInfo>& out) const override;
    void listValues(sdk::SimDeviceHandle device, std::vector<sdk::SimValueInfo>& out) const override;
    double getValue(sdk::SimValueHandle value) const override { return inner_->getValue(value); }
    void setValue(sdk::SimValueHandle value, double v) override { inner_->setValue(value, v); }

    // Number of synthetic late-value devices currently exposed (diagnostics/tests).
    std::size_t lateValueDeviceCount() const;

private:
    struct Base {
        std::set<sdk::SimValueHandle> reported;  // values handed to the consumer (via listValues or a synthetic device)
        bool listed = false;                     // the consumer has listed this device's values at least once
        std::vector<sdk::SimDeviceHandle> synthetics;
    };
    struct Synthetic {
        sdk::SimDeviceHandle base;
        std::string name;
        std::vector<sdk::SimValueInfo> values;
    };

    static constexpr std::uint64_t kSyntheticBit = std::uint64_t{1} << 62;

    sdk::ISimDeviceBackend* inner_;
    mutable std::mutex mutex_;
    mutable std::map<sdk::SimDeviceHandle, Base> bases_;
    mutable std::map<sdk::SimDeviceHandle, Synthetic> synthetics_;
    mutable std::uint64_t nextSynthetic_ = 1;
};

}  // namespace wpiemu::bridges::vendor
