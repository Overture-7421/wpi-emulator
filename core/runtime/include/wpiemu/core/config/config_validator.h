#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "wpiemu/sdk/config/config_io.h"
#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/tire_model.h"

namespace wpiemu::core::config {

// Stable machine keys of the core validator (Issue::code), next to sdk::validation_code.
namespace issue_code {
inline constexpr const char* kSchemaVersion = "schema_version_unsupported";
inline constexpr const char* kChassisInvalid = "chassis_invalid";
inline constexpr const char* kDrivetrainKindUnknown = "drivetrain_kind_unknown";
inline constexpr const char* kTireModelUnknown = "tire_model_unknown";
inline constexpr const char* kDrivetrainNoWheels = "drivetrain_no_wheels";
inline constexpr const char* kWheelInvalid = "wheel_invalid";
inline constexpr const char* kWheelDuplicate = "wheel_duplicate";
inline constexpr const char* kTireInvalid = "tire_invalid";
inline constexpr const char* kMotorInvalid = "motor_invalid";
inline constexpr const char* kMechanismIdInvalid = "mechanism_id_invalid";
inline constexpr const char* kMechanismIdDuplicate = "mechanism_id_duplicate";
inline constexpr const char* kMechanismTypeUnknown = "mechanism_type_unknown";
inline constexpr const char* kPropertyInvalid = "property_invalid";
inline constexpr const char* kSlotOwnerUnknown = "slot_owner_unknown";
inline constexpr const char* kSlotOwnerMismatch = "slot_owner_mismatch";
inline constexpr const char* kSlotUnknown = "slot_unknown";
inline constexpr const char* kSlotMotorInactive = "slot_motor_inactive";
inline constexpr const char* kSlotReversedInvalid = "slot_reversed_invalid";
inline constexpr const char* kSlotRequiredUnmapped = "slot_required_unmapped";
inline constexpr const char* kSlotDuplicateMapping = "slot_duplicate_mapping";
inline constexpr const char* kDeviceDuplicateMapping = "device_duplicate_mapping";
inline constexpr const char* kSlotDeviceIncompatible = "slot_device_incompatible";
inline constexpr const char* kBatteryInvalid = "battery_invalid";
inline constexpr const char* kSimInvalid = "sim_invalid";
inline constexpr const char* kPluginNotFound = "plugin_not_found";
inline constexpr const char* kPluginUnknownSection = "plugin_section_unknown";
inline constexpr const char* kPluginSectionVersion = "plugin_section_version_unsupported";
inline constexpr const char* kPluginPropertyInvalid = "plugin_property_invalid";
// zones (sdk/config/zone.h); attachment problems reuse sdk::validation_code::kAttachment*
inline constexpr const char* kZoneIdInvalid = "zone_id_invalid";                // empty id
inline constexpr const char* kZoneIdDuplicate = "zone_id_duplicate";            // duplicate, or equal to a mechanism id
inline constexpr const char* kZoneKindNotOwned = "zone_kind_not_owned";         // no plugin, or kind lacks the plugin id prefix (Error)
inline constexpr const char* kZoneKindUnknown = "zone_kind_unknown";           // owned prefix but not declared by the plugin (Warning)
inline constexpr const char* kZonePropertyInvalid = "zone_property_invalid";
inline constexpr const char* kZoneGeometryInvalid = "zone_geometry_invalid";    // non-finite pose, size <= 0 on a Volume
inline constexpr const char* kZoneConditionInvalid = "zone_condition_invalid";  // empty value name, non-finite threshold
inline constexpr const char* kZoneConditionMechanismUnknown = "zone_condition_mechanism_unknown";  // Warning
}  // namespace issue_code

// Slot names of the drivetrain owner (sdk::slot_owner::kDrivetrain): "<wheel>.drive" (+ "<wheel>.drive2"..
// up to driveMotorCount), and "<wheel>.steer" for swerve / steered wheels. Matches IDrivetrain::slotSpecs() by convention (the validator has no world to
// instantiate a drivetrain in).
std::vector<sdk::SlotSpec> drivetrainSlotSpecs(const sdk::DrivetrainSpec& spec);
// robot-level slots: "imu".
std::vector<sdk::SlotSpec> robotSlotSpecs();

// The plugin a config selects: gamePluginId when set, else the registered plugin for project.wpilibYear;
// nullptr = core features only ("unsupported year").
sdk::IGameModule* selectPlugin(const sdk::RobotConfig& config, const sdk::GamePluginRegistry& plugins);

// Whole-model validation (docs/runtime.md "Validation"). Pure: never mutates, never throws.
class ConfigValidator final : public sdk::IConfigValidator {
public:
    // Optional device knowledge: capabilities of a robot-declared device (from discovery or a bridge catalog).
    // Returns nullopt when the device is not known (the slot/device compatibility check is then skipped).
    using DeviceCapabilityLookup = std::function<std::optional<sdk::CapabilitySet>(const sdk::DeviceKey&)>;

    ConfigValidator() = default;
    ConfigValidator(const sdk::TireModelRegistry* tireModels, DeviceCapabilityLookup capabilities)
        : tireModels_(tireModels), capabilities_(std::move(capabilities)) {}

    std::vector<sdk::Issue> validate(const sdk::RobotConfig& config, const sdk::MechanismFactory& mechanisms,
                                     const sdk::DrivetrainRegistry& drivetrains,
                                     const sdk::GamePluginRegistry& plugins) const override;

    static bool hasErrors(const std::vector<sdk::Issue>& issues);

private:
    const sdk::TireModelRegistry* tireModels_ = nullptr;
    DeviceCapabilityLookup capabilities_;
};

}  // namespace wpiemu::core::config
