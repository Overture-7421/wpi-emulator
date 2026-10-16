#pragma once

#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/devices/signal.h"

namespace wpiemu::bridges::hal::detail {

inline sdk::DeviceDescriptor makeDescriptor(std::string type, int id, int channel, sdk::CapabilitySet capabilities,
                                            std::string nativeName, std::string displayName,
                                            std::string vendor = "wpilib") {
    sdk::DeviceDescriptor d;
    d.family = "hal";
    d.type = std::move(type);
    d.vendor = std::move(vendor);
    d.id = id;
    d.bus = "rio";
    d.channel = channel;
    d.capabilities = capabilities;
    d.nativeName = std::move(nativeName);
    d.displayName = std::move(displayName);
    return d;
}

inline sdk::SignalSpec sig(const char* name, sdk::SignalDirection dir, sdk::SignalType type, const char* unit = "") {
    return sdk::SignalSpec{name, dir, type, unit};
}

inline double fromBool(bool b) { return b ? 1.0 : 0.0; }
inline bool toBool(double v) { return v >= 0.5; }

}  // namespace wpiemu::bridges::hal::detail
