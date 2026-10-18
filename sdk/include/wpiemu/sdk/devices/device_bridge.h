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
// announces it later from poll(). `poll`, `collectOutputs`, `applyInputs`, `applyLoopback` (and the
// vision hooks `applyVision` / `collectVision`) are called on the extension's single CLOCK THREAD
// while the robot is quiescent (between HALSIM_StepTiming calls), so they need no extra locking
// against the robot loop.
//
// Error handling: no exceptions; failures are logged via IBridgeHost::logger() and the affected
// device is skipped.

#include <cstdint>
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
#include "wpiemu/sdk/vision/vision_types.h"

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
    // L3 observation: tells the host which bus label a previously announced device was seen
    // on (e.g. the CTRE constructor hook saw `new TalonFX(4, "canivore")`). The host forwards it to the
    // emulator (re-announce with DeviceAnnounce::observedBus). Never call it from static tables. Default: ignored.
    virtual void observeBus(DeviceId /*id*/, std::string_view /*busLabel*/) {}
    // Vision cameras the emulator configured for this connection (HelloAck::vision). Only the vision
    // bridge reads it; it stays valid until visionSetupRevision() changes. Default: none.
    virtual const VisionSetup& visionSetup() const {
        static const VisionSetup kNone;
        return kNone;
    }
    // Incremented by the host every time a HelloAck is adopted (first connection and every
    // reconnect), before the first SensorFrame of that connection is applied. A bridge that cached
    // visionSetup() polls it from poll() and rebuilds its cameras when it changes. Default: 0.
    virtual std::uint64_t visionSetupRevision() const { return 0; }
    // True once the robot program signalled that its constructor finished (HALSIM_GetProgramStarted).
    // A bridge that creates robot-visible state of its own (the vision bridge's NetworkTables topics)
    // waits for it, so everything the robot constructor created is already visible to it. With
    // WPIEMU_LIVE_DISCOVERY=1 bridges are polled earlier than that. Default: true.
    virtual bool programStarted() const { return true; }
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

    // New connection or simulation epoch: discard input integration/unwrap history.
    // Clock thread, between steps, before the new epoch's inputs. Preserve discovered devices and
    // robot-requested firmware offsets/configuration. Bridges without history need do nothing.
    virtual void resetInputHistory() {}

    // Emulator -> robot: the processed camera frames due this tick (SensorFrame::vision, already
    // filtered to this epoch). Clock thread, robot quiescent, after applyInputs and BEFORE
    // HALSIM_StepTiming. Only the vision bridge implements it; the default ignores the frames.
    virtual void applyVision(std::span<const VisionFrame> /*frames*/) {}
    // Robot -> emulator: appends what the robot did with each camera (informational): every camera
    // when `full` (the first frame of an epoch or connection), otherwise only the entries that
    // changed since the last call. Clock thread, after the step. The default appends nothing.
    virtual void collectVision(std::vector<CameraRobotState>& /*out*/, bool /*full*/) {}
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
