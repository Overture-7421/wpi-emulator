#pragma once

// The vendor SimDevice bridge: the SDK's table-driven SimDeviceBridge (rules in ctre_rules.h) wrapped with the
// vendor policies documented in docs/reference/bridges.md:
//  * input write policy (GuardedSimDeviceBackend): one continuous write per tick for integrated raw inputs,
//    read-back verification, re-assert after the vendor mirror, foreign-writer detection;
//  * "emulator owns device" switch per device (robot-side simulations keep control when disabled);
//  * wall-clock advice for the CTRE firmware sim (ITimeSyncAdvisor);
//  * optional read-only CTRE config read-back (L2) and the L3 observation sink (bus labels).
// It is an IDeviceBridge; the composition root creates it through registerVendorBridges().

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "wpiemu/bridges/vendor/ctre_native.h"
#include "wpiemu/bridges/vendor/guarded_backend.h"
#include "wpiemu/bridges/vendor/incremental_backend.h"
#include "wpiemu/bridges/vendor/time_sync.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/devices/sim_device_bridge.h"

namespace wpiemu::bridges::vendor {

// Backend with no devices: the bridge stays inert (used when no HAL binding is compiled in / available).
class NullSimDeviceBackend final : public sdk::ISimDeviceBackend {
public:
    std::uint64_t generation() const override { return 0; }
    void listDevices(std::vector<sdk::SimDeviceInfo>&) const override {}
    void listValues(sdk::SimDeviceHandle, std::vector<sdk::SimValueInfo>&) const override {}
    double getValue(sdk::SimValueHandle) const override { return 0.0; }
    void setValue(sdk::SimValueHandle, double) override {}
};

// Optional hook for the extension: call reassertAfterVendorMirror() from a HAL SimPeriodicBefore callback that
// was registered AFTER the vendor's own (callbacks run in registration order), i.e. right after the vendor
// mirror ran and BEFORE the robot code reads its sensors. Restores guarded inputs the mirror disturbed.
class IVendorMirrorSync {
    WPIEMU_INTERFACE(IVendorMirrorSync)
public:
    virtual std::size_t reassertAfterVendorMirror() = 0;
};

using CtreNativeProvider = std::function<std::shared_ptr<ICtreSimNative>()>;
using ConfigReadbackSink =
    std::function<void(sdk::DeviceId, const sdk::DeviceDescriptor&, const CtreConfigSnapshot&)>;

struct VendorSimBridgeOptions {
    GuardOptions guard;
    TimeSyncPolicy timeSync = TimeSyncPolicy::Advise;
    bool backendAvailable = true;  // false: logged once; the bridge then sees no devices

    // L2 (read-only): evaluated lazily because the CTRE module is only loaded once the robot created devices.
    CtreNativeProvider nativeProvider;  // empty: no config read-back
    std::string configBus = "rio";      // the sim is bus-blind, any label works
    int configReadAttempts = 3;
    ConfigReadbackSink onConfig;

    // L3 extension seam (no implementation ships here); installed only if the session enabled optional hooks.
    std::shared_ptr<ICtreHookProvider> hooks;
};

class VendorSimDeviceBridge final : public sdk::IDeviceBridge,
                                    public IVendorMirrorSync,
                                    public ITimeSyncAdvisor,
                                    public ICtreObservationSink {
public:
    VendorSimDeviceBridge(std::string name, std::vector<sdk::SimDeviceRule> rules,
                          std::shared_ptr<sdk::ISimDeviceBackend> backend, VendorSimBridgeOptions options = {});
    ~VendorSimDeviceBridge() override;

    // IDeviceBridge
    std::string_view name() const override { return inner_.name(); }
    sdk::Status start(sdk::IBridgeHost& host) override;
    void poll() override;
    void collectOutputs(std::vector<sdk::SignalSample>& out) override;
    void applyInputs(std::span<const sdk::SignalSample> samples) override;
    void applyLoopback(sdk::DeviceId id) override;
    bool owns(sdk::DeviceId id) const override { return inner_.owns(id); }
    void stop() noexcept override;

    // IVendorMirrorSync
    std::size_t reassertAfterVendorMirror() override { return guarded_.reassertAfterMirror(); }

    // ITimeSyncAdvisor
    TimeSyncAdvice timeSyncAdvice() const override;

    // ICtreObservationSink (any thread): queued and applied in poll().
    void onDeviceObserved(const CtreDeviceObservation& observation) override;

    // "Emulator owns the device" switch. Disabled devices are neither written nor looped back (a robot-side
    // simulation keeps control); outputs are still read so the emulator can display them.
    void setInputWritesEnabled(sdk::DeviceId id, bool enabled);
    bool inputWritesEnabled(sdk::DeviceId id) const { return disabled_.count(id) == 0; }

    // Forget input write history / angle unwrapping (emulator Reset or new run epoch).
    void resetInputHistory() override { guarded_.resetHistory(); }

    // CTRE config read-back (L2). It calls into CTRE's native sim, which is NOT safe to enter while robot code is
    // creating/configuring devices on other threads (a JVM access violation was observed). It therefore NEVER runs
    // from poll() (which the extension may call while the robot constructor is still running); it runs from
    // collectOutputs() (documented to be called only while the robot is quiescent) and from this method, which a
    // host that knows the robot is quiescent may call explicitly. At most `maxDevices` devices per call.
    // Returns the number of configs obtained.
    std::size_t readPendingConfigs(std::size_t maxDevices = 1);

    std::vector<InputDisturbance> inputDisturbances() const { return guarded_.disturbances(); }
    const CtreConfigSnapshot* configOf(sdk::DeviceId id) const;
    std::string observedBus(sdk::DeviceId id) const { return inner_.observedBus(id); }
    std::size_t liveDeviceCount() const { return announced_.size(); }

private:
    class RecordingHost;
    struct Announced {
        sdk::DeviceDescriptor descriptor;
        std::optional<std::uint32_t> positionSignal;
        std::optional<std::uint32_t> velocitySignal;
        // Phoenix 6 motors: the firmware's torque current (vendor telemetry) and the canonical torqueCurrent this
        // bridge synthesizes from it in the FOC control modes (see appendTorqueCommands).
        std::optional<std::uint32_t> torqueReportedSignal;
        std::optional<std::uint32_t> torqueCommandSignal;
        int controlMode = -1;  // last ControlModeValue read, -1 = never
        bool controlModeUnavailable = false;
        int configAttempts = 0;
        bool configDone = false;
    };

    void drainObservations();
    bool ensureNative();
    void appendTorqueCommands(std::vector<sdk::SignalSample>& out, std::size_t first);
    bool readOneConfig();

    std::shared_ptr<sdk::ISimDeviceBackend> backend_;
    VendorSimBridgeOptions options_;
    IncrementalSimDeviceBackend incremental_;
    GuardedSimDeviceBackend guarded_;
    sdk::SimDeviceBridge inner_;
    std::unique_ptr<RecordingHost> recording_;
    sdk::IBridgeHost* host_ = nullptr;

    std::map<sdk::DeviceId, Announced> announced_;
    std::map<sdk::DeviceId, CtreConfigSnapshot> configs_;
    std::set<sdk::DeviceId> disabled_;
    std::vector<sdk::SignalSample> filtered_;
    std::shared_ptr<ICtreSimNative> native_;
    int nativeRetryIn_ = 0;
    bool hooksInstalled_ = false;

    mutable std::mutex observationMutex_;
    std::vector<CtreDeviceObservation> observations_;
};

}  // namespace wpiemu::bridges::vendor
