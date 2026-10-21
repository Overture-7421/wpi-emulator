#pragma once

#include <string>
#include <vector>

#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/devices/signal.h"

namespace wpiemu::sdk {

// What one device is, on both sides of the wire: the extension announces it (proto::DeviceAnnounce adds the
// `retired` flag) and the emulator-side hub keeps it (DiscoveredDevice adds the mapping state).
struct DeviceRecord {
    DeviceId id;
    DeviceDescriptor descriptor;
    std::vector<SignalSpec> signalSpecs;  // index == SignalSample::signal
    // CAN bus label OBSERVED at runtime (CTRE L3 hook); empty = not observed. Never derived from a static
    // SimDeviceRule. A re-announce with a new value updates the hub's device.
    std::string observedBus;
};

}  // namespace wpiemu::sdk
