#pragma once

// Entry point of the vendor bridge module (docs/reference/bridges.md): registers one IDeviceBridge per vendor family into the
// extension's DeviceBridgeRegistry:
//   "ctre.phoenix6"  Talon FX/FXS (Kraken, Falcon), CANcoder, Pigeon 2, CANdle, CANrange, CANdi
//   "ctre.phoenix5"  Talon SRX, Victor SPX, legacy Pigeon IMU (discover-only)
// Adding a vendor (REV, NavX, ...) = a rule table + one more registration here; see docs/reference/bridges.md.

#include <functional>
#include <memory>
#include <string_view>

#include "wpiemu/bridges/vendor/ctre_native.h"
#include "wpiemu/bridges/vendor/ctre_rules.h"
#include "wpiemu/bridges/vendor/time_sync.h"
#include "wpiemu/bridges/vendor/vendor_sim_bridge.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/devices/sim_device_bridge.h"

namespace wpiemu::bridges::vendor {

std::string_view bridgeFamily();

struct VendorBridgeOptions {
    // SimDevice access seam. Null: the real HAL binding (attached lazily to the loaded wpiHal) when it was
    // compiled in, otherwise an inert backend (the bridge then reports "no backend" once and finds nothing).
    std::shared_ptr<sdk::ISimDeviceBackend> backend;

    bool enablePhoenix6 = true;
    bool enablePhoenix5 = true;
    ctre::Phoenix5Options phoenix5;

    TimeSyncPolicy timeSync = TimeSyncPolicy::Advise;

    // L2 supplement. Read-only config read-back is ON by default (the module is attached lazily, never loaded;
    // absent symbols are tolerated). Nothing else of L2 is used by the shipped bridges.
    bool readCtreConfig = true;
    CtreNativeProvider nativeProvider;   // empty + readCtreConfig: attach to the loaded CTRE_PhoenixTools_Sim
    ConfigReadbackSink onConfig;         // receives each device's config (UI auto-fill: ratios, invert, remote sensor)

    // L3 extension seam, OFF unless a provider is supplied AND the session enables optional hooks.
    std::shared_ptr<ICtreHookProvider> hooks;
};

// Registers the default bridges (real HAL binding when available, config read-back on).
void registerVendorBridges(sdk::DeviceBridgeRegistry& registry);
void registerVendorBridges(sdk::DeviceBridgeRegistry& registry, VendorBridgeOptions options);

// Same, reporting duplicate registrations instead of ignoring them.
sdk::Status registerVendorBridgesChecked(sdk::DeviceBridgeRegistry& registry, VendorBridgeOptions options = {});

}  // namespace wpiemu::bridges::vendor
