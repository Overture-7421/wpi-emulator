#pragma once

// Device identity model (docs/ARCHITECTURE.md).
//
//  * DeviceKey            robot-DECLARED address: family/type/id/bus[/channel]. What robot.json and the
//                         robot program say. The bus is a label the user/robot asserted.
//  * PhysicalDeviceId     what the SIMULATOR can tell apart: family/type/id[/channel], NO bus. CTRE
//                         sim state is keyed by (type, id) only.
//  * aliases              several declared addresses (differing only in bus) that resolve to ONE
//                         physical device. Harmless when a single slot uses them (Aliased), unsafe
//                         when they are mapped independently (Conflict).
//  * observedBus          bus label seen at runtime by an L3 hook (IBridgeHost::observeBus ->
//                         DeviceAnnounce::observedBus -> IDeviceHubIngress::observeBus). Empty until
//                         observed. NEVER fabricated from a static SimDeviceRule.
//
// Resolution contract (IDeviceDirectory::find(const DeviceKey&) and slot binding): a declared
// address resolves to the discovered device with the same PhysicalDeviceId. The bus does not take
// part in the match.
//
// Conflict contract:
//  * Config validator (IConfigValidator): MUST reject independent mappings of conflicting aliases
//    with an Error Issue whose `code` is validation_code::kDeviceAliasConflict. Use
//    validateDeviceAliases(config.allMappings()) to get exactly that.
//  * Hub (runtime, for saved/stale configs that slipped through): a device in AliasState::Conflict is
//    treated as UNMAPPED for safety: it goes into the LoopbackSet (extension echoes, no IPC hop), every
//    slot bound to it reads neutral data and ignores writes, and a HubWarning{Error,
//    kDeviceAliasConflict} is raised once. Other devices are unaffected; the session keeps running.
//    Mappings are not deleted: fixing the config re-enables the device.
//  * AliasState::Aliased is a Warning (kDeviceAliased) only: traffic flows normally.

#include <algorithm>
#include <cstdint>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/validation_codes.h"
#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/devices/slot.h"

namespace wpiemu::sdk {

enum class AliasState : std::uint8_t {
    None = 0,      // at most one declared address
    Aliased = 1,   // several declared addresses, at most one slot mapping: benign
    Conflict = 2,  // several declared addresses mapped independently: device disabled (loopback)
};

// Pure classification: `distinctDeclared` = number of distinct robot-declared addresses resolving to
// the device (from announces, L3 observation and config mappings); `independentMappings` = how many
// of them are bound to a slot.
constexpr AliasState classifyAliasState(std::size_t distinctDeclared, std::size_t independentMappings) {
    if (distinctDeclared < 2) return AliasState::None;
    return independentMappings >= 2 ? AliasState::Conflict : AliasState::Aliased;
}

// Mappings that address one physical device through different declared addresses.
struct AliasConflict {
    PhysicalDeviceId physical;
    std::vector<SlotMapping> mappings;  // >= 2 entries, pairwise distinct DeviceKey
};

inline std::vector<AliasConflict> findAliasConflicts(std::span<const SlotMapping> mappings) {
    std::map<PhysicalDeviceId, std::vector<SlotMapping>> byPhysical;
    for (const SlotMapping& m : mappings) byPhysical[m.device.physical()].push_back(m);

    std::vector<AliasConflict> out;
    for (auto& [physical, group] : byPhysical) {
        std::vector<DeviceKey> keys;
        for (const SlotMapping& m : group)
            if (std::find(keys.begin(), keys.end(), m.device) == keys.end()) keys.push_back(m.device);
        if (keys.size() >= 2) out.push_back({physical, std::move(group)});
    }
    return out;
}

// One Error per conflict (validation_code::kDeviceAliasConflict), pathed at the first mapping's slot.
inline std::vector<Issue> validateDeviceAliases(std::span<const SlotMapping> mappings) {
    std::vector<Issue> out;
    for (const AliasConflict& c : findAliasConflicts(mappings)) {
        std::string message = "device " + c.physical.toString() + " is mapped through different addresses:";
        for (const SlotMapping& m : c.mappings) message += " " + m.slot.toString() + " -> " + m.device.toString() + ";";
        message += " they are one physical simulator device (the bus is not part of its identity)";
        out.push_back({Severity::Error, c.mappings.front().slot.toString(), std::move(message),
                       validation_code::kDeviceAliasConflict});
    }
    return out;
}

}  // namespace wpiemu::sdk
