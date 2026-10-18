#pragma once

// Emulator-side device hub: the RUNTIME mapping between discovered devices (announced by the
// extension) and model slots (mechanisms, drivetrain, robot IMU), plus the typed data plane.
//
// Ownership/threading: implemented by core; owned by the sim runtime; all methods are called on
// the SIM THREAD only (IPC thread output is queued to it). References/pointers returned by
// query methods are valid until the next hub mutation (announce/bind/unbind/clear).
//
// Unmapped behaviour (Decision "discover-then-map"): a live device with no slot mapping is
//  * put in the LoopbackSet sent to the extension (output echoed to input there),
//  * reported once as a HubWarning{code="unmapped_loopback"}.
// A slot with no device returns neutral data (MotorOutput{valid=false}, false, 0) and ignores writes.

#include <cstddef>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/devices/discovered_device.h"
#include "wpiemu/sdk/devices/frames.h"
#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::sdk {

// Robot -> model reads.
class IActuatorReader {
    WPIEMU_INTERFACE(IActuatorReader)
public:
    virtual MotorOutput motorOutput(const SlotAddress& slot) const = 0;
    // Solenoid / DigitalOutput slots. false when unmapped.
    virtual bool digitalOutput(const SlotAddress& slot) const = 0;
};

// Model -> robot writes (absolute physical truth every tick, ADR 0002; never deltas).
class ISensorWriter {
    WPIEMU_INTERFACE(ISensorWriter)
public:
    virtual void writeMotorFeedback(const SlotAddress& slot, const MotorFeedback& feedback) = 0;
    virtual void writeEncoder(const SlotAddress& slot, const EncoderFeedback& feedback) = 0;
    virtual void writeDigital(const SlotAddress& slot, bool value) = 0;   // limit / beam break
    virtual void writeAnalog(const SlotAddress& slot, double volts) = 0;
    virtual void writeImu(const SlotAddress& slot, const ImuFeedback& feedback) = 0;
};

// Discovery + mapping.
class IDeviceDirectory {
    WPIEMU_INTERFACE(IDeviceDirectory)
public:
    virtual std::span<const DiscoveredDevice> discovered() const = 0;
    virtual const DiscoveredDevice* find(DeviceId id) const = 0;
    // Resolves a robot-DECLARED address to the discovered device with the same PhysicalDeviceId; the
    // bus label does not take part in the match (device_identity.h). Several declared addresses can
    // therefore resolve to one device (see DiscoveredDevice::aliasState).
    virtual const DiscoveredDevice* find(const DeviceKey& key) const = 0;
    // Exact physical-identity lookup. The default scans discovered(); cores may override.
    virtual const DiscoveredDevice* findPhysical(const PhysicalDeviceId& physical) const {
        for (const DiscoveredDevice& d : discovered())
            if (d.physicalId() == physical) return &d;
        return nullptr;
    }

    // Binds a slot to a device key. The device need not be discovered yet (saved configs bind
    // before the robot starts); compatibility is checked when it appears (HubWarning).
    // AlreadyExists if the device (same DeviceKey) is bound to another slot (one device : one slot).
    // Binding two DIFFERENT aliases of one physical device to two slots is accepted here but puts the
    // device in AliasState::Conflict (disabled + loopback + HubWarning); the config validator rejects it.
    virtual Status bind(const SlotAddress& slot, const DeviceKey& device) = 0;
    virtual void unbind(const SlotAddress& slot) = 0;
    virtual std::optional<DeviceKey> boundDevice(const SlotAddress& slot) const = 0;
    virtual std::optional<DeviceId> resolve(const SlotAddress& slot) const = 0;
    virtual bool isMapped(DeviceId id) const = 0;

    // Accumulated, de-duplicated warnings (cleared by clear()).
    virtual std::span<const HubWarning> warnings() const = 0;
};

// What mechanisms / drivetrain / game code see.
class IDeviceHub : public IActuatorReader, public ISensorWriter, public IDeviceDirectory {
public:
    // Battery voltage currently supplied to all devices (V).
    virtual double batteryVoltage() const = 0;
};

// What the sim runtime (core) feeds from the IPC channel. Separate from IDeviceHub (ISP): models
// must never call these.
class IDeviceHubIngress {
    WPIEMU_INTERFACE(IDeviceHubIngress)
public:
    virtual void onAnnounce(const proto::DeviceAnnounce& announce) = 0;
    // Latest robot outputs for tick N. Partial frames (full == false) update only listed signals.
    virtual void onActuatorFrame(const proto::ActuatorFrame& frame) = 0;
    virtual void setBatteryVoltage(double volts) = 0;
    // Fills samples with every Input signal written since the last call (all of them if `full`),
    // and sets batteryVoltageV. tick/simTime/stepUs are set by the caller.
    virtual void buildSensorFrame(proto::SensorFrame& out, bool full) = 0;
    // Devices that must loopback in the extension (live && !mapped). Includes devices in AliasState::Conflict.
    virtual proto::LoopbackSet loopbackSet() const = 0;
    // Session ended / robot disconnected: forget devices, keep bindings.
    virtual void clearDevices() = 0;
    // A runtime observation (L3 hook) of the bus a device lives on. Updates
    // DiscoveredDevice::observedBus; raises HubWarning{kDeviceBusMismatch} when it contradicts a declared
    // address. Also fed by onAnnounce when DeviceAnnounce::observedBus is non-empty. Default: ignored.
    virtual void observeBus(DeviceId /*device*/, std::string_view /*busLabel*/) {}
};

}  // namespace wpiemu::sdk
