#pragma once

// The per-section steps of ConfigValidator::validate, shared by its translation units (internal to core/runtime/src/config):
// config_validator.cpp sequences them and checks the plugin-facing sections, config_validator_robot.cpp checks the physical
// model (chassis, drivetrain, sim, battery) and config_validator_slots.cpp the mechanisms and their slot mappings. Each step
// appends to `out`; the order validate() calls them in is the order of the reported issues.

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/sdk/mechanisms/motion_link.h"

namespace wpiemu::core::config::detail {

// The slot specs of every slot owner: mechanism ids, "drivetrain", "robot" and sensor ids.
using SlotSpecsByOwner = std::map<std::string, std::vector<sdk::SlotSpec>>;

inline void add(std::vector<sdk::Issue>& out, sdk::Severity sev, std::string path, std::string message, const char* code) {
    out.push_back({sev, std::move(path), std::move(message), code});
}

// config_validator_robot.cpp
void validateDrivetrain(const sdk::RobotConfig& c, const sdk::DrivetrainRegistry& drivetrains, const sdk::TireModelRegistry* tires,
                        std::vector<sdk::Issue>& out);
void validateSimSettings(const sdk::RobotConfig& c, std::vector<sdk::Issue>& out);

// config_validator_slots.cpp
// Ids, templates and properties of the mechanisms; records the slot specs of each known template in `slotSpecsByOwner`.
void validateMechanisms(const sdk::RobotConfig& config, const sdk::MechanismFactory& mechanisms, SlotSpecsByOwner& slotSpecsByOwner,
                        std::vector<sdk::Issue>& out);
// The slot specs of the owners that are not mechanism templates: the drivetrain, the robot and the sensors.
void addNonMechanismSlotSpecs(const sdk::RobotConfig& config, const sdk::DrivetrainRegistry& drivetrains,
                              SlotSpecsByOwner& slotSpecsByOwner);
void validateSlotMappings(const sdk::RobotConfig& config, const sdk::MechanismFactory& mechanisms,
                          const SlotSpecsByOwner& slotSpecsByOwner, const ConfigValidator::DeviceCapabilityLookup& capabilities,
                          std::vector<sdk::Issue>& out);
void validateMappingUniqueness(const std::vector<sdk::SlotMapping>& all, std::vector<sdk::Issue>& out);
void validateRequiredSlots(const sdk::RobotConfig& config, const std::vector<sdk::MotionAxisInfo>& motionAxes,
                           const SlotSpecsByOwner& slotSpecsByOwner, std::vector<sdk::Issue>& out);

}  // namespace wpiemu::core::config::detail
