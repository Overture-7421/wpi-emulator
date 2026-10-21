#pragma once

#include <string>
#include <vector>

#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/devices/device_identity.h"
#include "wpiemu/sdk/devices/device_record.h"

namespace wpiemu::sdk {

// A device as known to the emulator-side hub: announced by the extension, optionally mapped to a slot.
struct DiscoveredDevice : DeviceRecord {
    bool live = true;                 // false after the robot freed/destroyed it
    bool mapped = false;              // bound to a slot (else: loopback + warning); false while in Conflict
    // ---- device identity (device_identity.h) ----
    AliasState aliasState = AliasState::None;
    std::vector<DeviceKey> aliases;   // distinct declared addresses resolving to this device (empty/1 => None)

    PhysicalDeviceId physicalId() const { return descriptor.physicalId(); }
};

struct HubWarning {
    Severity severity = Severity::Warning;
    DeviceId device;                 // invalid when not device-specific
    // stable machine key: "unmapped_loopback", "slot_incompatible", "slot_missing_device" and, since 1.1,
    // validation_code::kDeviceAliasConflict / kDeviceAliased / kDeviceBusMismatch
    std::string code;
    std::string message;             // human readable
};

}  // namespace wpiemu::sdk
