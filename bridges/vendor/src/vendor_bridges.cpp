#include "wpiemu/bridges/vendor/vendor_bridges.h"

#include <mutex>

#include "wpiemu/bridges/vendor/hal_sim_device_backend.h"

namespace wpiemu::bridges::vendor {

std::string_view bridgeFamily() { return "vendor"; }

namespace {

// Shared by the factories of one registration: resolves the backend once (lazily, because the HAL binding must
// attach to a wpiHal that is already loaded) and hands every bridge the same instance.
class SharedSetup {
public:
    explicit SharedSetup(VendorBridgeOptions options) : options_(std::move(options)) {
        if (options_.readCtreConfig && !options_.nativeProvider) {
            options_.nativeProvider = [] { return makeCtreSimNative(); };
        }
        if (!options_.readCtreConfig) options_.nativeProvider = nullptr;
    }

    std::shared_ptr<sdk::ISimDeviceBackend> backend(bool& available) {
        std::lock_guard lock(mutex_);
        if (!resolved_) {
            resolved_ = true;
            if (options_.backend) {
                backend_ = options_.backend;
                available_ = true;
            } else if (auto hal = makeHalSimDeviceBackend()) {
                backend_ = std::shared_ptr<sdk::ISimDeviceBackend>(std::move(hal));
                available_ = true;
            } else {
                backend_ = std::make_shared<NullSimDeviceBackend>();
                available_ = false;
            }
        }
        available = available_;
        return backend_;
    }

    std::unique_ptr<sdk::IDeviceBridge> makePhoenix6() {
        VendorSimBridgeOptions o;
        o.guard = ctre::phoenix6GuardOptions();
        o.timeSync = options_.timeSync;
        o.nativeProvider = options_.nativeProvider;
        o.onConfig = options_.onConfig;
        o.hooks = options_.hooks;
        auto b = backend(o.backendAvailable);
        return std::make_unique<VendorSimDeviceBridge>("ctre.phoenix6", ctre::phoenix6Rules(), std::move(b), std::move(o));
    }

    std::unique_ptr<sdk::IDeviceBridge> makePhoenix5() {
        VendorSimBridgeOptions o;  // Phoenix 5: no config API, no integrated-input guard (not characterised)
        o.timeSync = options_.timeSync;
        auto b = backend(o.backendAvailable);
        return std::make_unique<VendorSimDeviceBridge>("ctre.phoenix5", ctre::phoenix5Rules(options_.phoenix5),
                                                       std::move(b), std::move(o));
    }

    const VendorBridgeOptions& options() const { return options_; }

private:
    VendorBridgeOptions options_;
    std::mutex mutex_;
    bool resolved_ = false;
    bool available_ = false;
    std::shared_ptr<sdk::ISimDeviceBackend> backend_;
};

}  // namespace

sdk::Status registerVendorBridgesChecked(sdk::DeviceBridgeRegistry& registry, VendorBridgeOptions options) {
    auto setup = std::make_shared<SharedSetup>(std::move(options));
    if (setup->options().enablePhoenix6) {
        sdk::Status st = registry.registerBridge(
            "ctre.phoenix6",
            sdk::BridgeInfo{ctre::kFamilyPhoenix6, "CTRE Phoenix 6 SimDevices (TalonFX/FXS, CANcoder, Pigeon 2, CANdle, CANrange, CANdi)",
                            ctre::kVendor},
            [setup] { return setup->makePhoenix6(); });
        if (!st) return st;
    }
    if (setup->options().enablePhoenix5) {
        sdk::Status st = registry.registerBridge(
            "ctre.phoenix5",
            sdk::BridgeInfo{ctre::kFamilyPhoenix5, "CTRE Phoenix 5 SimDevices (Talon SRX, Victor SPX, Pigeon IMU)", ctre::kVendor},
            [setup] { return setup->makePhoenix5(); });
        if (!st) return st;
    }
    return sdk::okStatus();
}

void registerVendorBridges(sdk::DeviceBridgeRegistry& registry) { registerVendorBridges(registry, VendorBridgeOptions{}); }

void registerVendorBridges(sdk::DeviceBridgeRegistry& registry, VendorBridgeOptions options) {
    // Registration failures are programmer errors (duplicate names); the checked variant reports them.
    (void)registerVendorBridgesChecked(registry, std::move(options));
}

}  // namespace wpiemu::bridges::vendor
