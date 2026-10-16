#pragma once

// Slot / device status and the editor's own (local) validation. The SDK validator (via the backend)
// reports model errors; these checks add the discovery-aware ones the user needs while mapping:
// unfilled required slots, mapped devices that were never seen, unmapped live devices (loopback).

#include <QString>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "editor_backend.h"
#include "model/device_directory.h"
#include "model/selection.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/devices/device_identity.h"

namespace wpiemu::app::editor {

enum class SlotState {
    Unmapped,       // required slot without a device
    OptionalEmpty,  // optional slot without a device (neutral data)
    Mapped,         // device mapped (and seen, or no discovery data yet)
    NotDetected,    // mapped, but discovery ran and the device was not seen / is gone
    Incompatible,   // mapped device lacks the capabilities of the slot type
    AliasConflict,  // the mapped physical device is also mapped through another address
};

struct SlotInfo {
    sdk::SlotAddress address;
    sdk::SlotSpec spec;
    std::optional<sdk::DeviceKey> mapped;
    SlotState state = SlotState::OptionalEmpty;
    QString detail;               // human text for tooltips
};

const char* slotTypeName(sdk::SlotType type);
QString slotTypeLabel(sdk::SlotType type);

// Slots of one owner (mechanism id, "drivetrain", "robot"), in spec order, with their state.
std::vector<SlotInfo> slotInfos(const sdk::RobotConfig& c, const EditorCatalog& catalog, const DeviceDirectory& devices,
                                std::string_view owner);

// First compatible slot a device can take: unfilled required slots first, then unfilled optional ones.
// `slotPrefix` restricts the search (drivetrain wheels: "fl.").
std::optional<sdk::SlotAddress> bestSlotFor(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner,
                                            sdk::CapabilitySet caps, std::string_view slotPrefix = {});

struct DeviceStatus {
    std::vector<sdk::SlotAddress> usedBy;          // slots the physical device is mapped to
    sdk::AliasState alias = sdk::AliasState::None;
    bool mapped() const { return !usedBy.empty(); }
};
DeviceStatus deviceStatus(const sdk::RobotConfig& c, const sdk::DiscoveredDevice& device);

// Checks that need the device list. Stable codes (see validation_codes.h for the SDK ones):
//   slot_unfilled (Warning), slot_incompatible (Error), device_not_detected (Info),
//   unmapped_loopback (Warning, path "device:<key>"), device_multi_mapped (Warning)
std::vector<sdk::Issue> localIssues(const sdk::RobotConfig& c, const EditorCatalog& catalog, const DeviceDirectory& devices);

// Order-preserving union; entries with the same (code, path, severity) collapse.
std::vector<sdk::Issue> mergeIssues(const std::vector<std::vector<sdk::Issue>>& lists);

struct IssueLocation {
    SelectionRef selection;
    std::string fieldKey;     // property key to focus in the inspector, when known
};
// Maps an issue path ("mechanisms[2].properties.gearRatio", "intake.motor", "chassis.size", ...) to what to select.
IssueLocation locateIssue(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::Issue& issue);

}  // namespace wpiemu::app::editor
