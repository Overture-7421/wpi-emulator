#pragma once

// Device bridges run INSIDE THE ROBOT PROCESS (loaded by the halsim_wpiemu extension). One
// bridge = one device family (HAL PWM, HAL DIO, Phoenix 6 SimDevices, ...). A bridge:
//   1. discovers devices and announces them (IBridgeHost::announce) with a DeviceDescriptor and
//      its signal list,
//   2. reads robot OUTPUT signals each tick (collectOutputs),
//   3. writes sim INPUT signals each tick (applyInputs),
//   4. performs loopback for unmapped devices without an IPC hop (applyLoopback).
// Bridges depend only on the SDK (+ the HAL / vendor headers they wrap, which the SDK never sees).
//
// Threading: callbacks from HAL arrive on ROBOT threads inside HAL init calls, so a bridge MUST
// NOT block, call HALSIM_StepTiming, or do I/O there (ADR 0001). It records the event and
// announces it later from poll(). `poll`, `collectOutputs`, `applyInputs`, `applyLoopback` are
// called on the extension's single CLOCK THREAD while the robot is quiescent (between
// HALSIM_StepTiming calls), so they need no extra locking against the robot loop.
//
// Error handling: no exceptions; failures are logged via IBridgeHost::logger() and the affected
// device is skipped.

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/registry.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/devices/signal.h"

namespace wpiemu::sdk {

// Implemented by the extension; what a bridge can ask of it.
class IBridgeHost {
    WPIEMU_INTERFACE(IBridgeHost)
public:
    // Announces (or updates, idempotent by descriptor.key()) a device; returns its session-unique id.
    virtual DeviceId announce(const DeviceDescriptor& descriptor, std::vector<SignalSpec> signalSpecs) = 0;
    // The robot freed the device.
    virtual void retire(DeviceId id) = 0;
    virtual ILogger& logger() = 0;
    // Optional per-session switches from HelloAck (e.g. CTRE L3 hooks).
    virtual bool enableOptionalHooks() const = 0;
    // SDK 1.1. L3 observation: tells the host which bus label a previously announced device was seen
    // on (e.g. the CTRE constructor hook saw `new TalonFX(4, "canivore")`). The host forwards it to the
    // emulator (re-announce with DeviceAnnounce::observedBus). Never call it from static tables. Default: ignored.
    virtual void observeBus(DeviceId /*id*/, std::string_view /*busLabel*/) {}
};

class IDeviceBridge {
    WPIEMU_INTERFACE(IDeviceBridge)
public:
    virtual std::string_view name() const = 0;  // unique: "hal.pwm", "ctre.simdevice"

    // Register HAL/vendor callbacks. Called once, before the robot's user code runs.
    virtual Status start(IBridgeHost& host) = 0;
    // Announce new/retired devices recorded since the last call. Cheap when idle.
    virtual void poll() = 0;

    // Robot -> sim: append every Output/Bidirectional signal sample (full snapshot; the extension
    // delta-compresses against the previous tick).
    virtual void collectOutputs(std::vector<SignalSample>& out) = 0;
    // Sim -> robot: write Input/Bidirectional signals. Samples for devices this bridge does not
    // own are NOT passed to it.
    virtual void applyInputs(std::span<const SignalSample> samples) = 0;
    // Unmapped device: copy its outputs to its inputs (e.g. PWM speed -> nothing, DIO out -> in,
    // motor duty -> rotor velocity). Must be a no-op if it has nothing to echo.
    virtual void applyLoopback(DeviceId id) = 0;
    // Does this bridge own this device id (announced by it)?
    virtual bool owns(DeviceId id) const = 0;

    // Idempotent; restores any changed robot-visible state and unregisters callbacks.
    virtual void stop() noexcept = 0;
};

struct BridgeInfo {
    std::string family;       // "hal", "phoenix6", ...
    std::string description;
    std::string vendor;
};

// Registry of bridge factories, owned by the extension's composition root. Each bridge module
// exposes ONE free function `void registerXxxBridges(DeviceBridgeRegistry&)`; adding a vendor =
// add a module + one call in the extension (no changes to existing bridges).
class DeviceBridgeRegistry : public Registry<BridgeInfo, IDeviceBridge> {
public:
    Status registerBridge(std::string name, BridgeInfo info, Factory factory) {
        return add(std::move(name), std::move(info), std::move(factory));
    }

    // Instantiates every registered bridge in registration order.
    Result<std::vector<std::unique_ptr<IDeviceBridge>>> createAll() const {
        std::vector<std::unique_ptr<IDeviceBridge>> out;
        for (const std::string& key : keys()) {
            Result<std::unique_ptr<IDeviceBridge>> b = create(key);
            if (!b) return b.error();
            out.push_back(std::move(b).value());
        }
        return out;
    }
};

}  // namespace wpiemu::sdk
