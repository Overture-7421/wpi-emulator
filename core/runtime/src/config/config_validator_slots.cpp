#include <map>
#include <set>
#include <string>

#include "config_validator_steps.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/config/sensor.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

namespace wpiemu::core::config::detail {

using namespace wpiemu::sdk;

namespace {

const SlotSpec* findSpec(const std::vector<SlotSpec>& specs, const std::string& name) {
    for (const SlotSpec& s : specs)
        if (s.name == name) return &s;
    return nullptr;
}

// Checks the mappings of each owner against the slot specs of that owner.
class MappingChecker {
public:
    MappingChecker(const RobotConfig& config, const MechanismFactory& mechanisms, const SlotSpecsByOwner& slotSpecsByOwner,
                   const ConfigValidator::DeviceCapabilityLookup& capabilities, std::vector<Issue>& out)
        : config_(config), mechanisms_(mechanisms), slotSpecsByOwner_(slotSpecsByOwner), capabilities_(capabilities), out_(out) {}

    void checkGroup(const std::vector<SlotMapping>& group, const std::string& expectedOwner, const std::string& path) const {
        for (std::size_t i = 0; i < group.size(); ++i) checkMapping(group[i], expectedOwner, path + "[" + std::to_string(i) + "]");
    }

private:
    void checkMapping(const SlotMapping& map, const std::string& expectedOwner, const std::string& p) const {
        if (map.slot.owner != expectedOwner) {
            add(out_, Severity::Error, p, "slot owner '" + map.slot.owner + "' does not match '" + expectedOwner + "'",
                issue_code::kSlotOwnerMismatch);
            return;
        }
        const auto specs = slotSpecsByOwner_.find(map.slot.owner);
        if (specs == slotSpecsByOwner_.end()) return;  // unknown template: already reported
        const SlotSpec* spec = findSpec(specs->second, map.slot.slot);
        const MechanismConfig* owner = nullptr;
        for (const MechanismConfig& m : config_.mechanisms)
            if (m.id == map.slot.owner) owner = &m;
        if (spec == nullptr) {
            const Severity sev = map.slot.owner == slot_owner::kRobot ? Severity::Warning : Severity::Error;
            add(out_, sev, p, "unknown slot '" + map.slot.toString() + "'", issue_code::kSlotUnknown);
            return;
        }
        if (map.reversed && spec->type != SlotType::Motor)
            add(out_, Severity::Warning, p, "only motor slots can be reversed ('" + map.slot.toString() + "')",
                issue_code::kSlotReversedInvalid);
        if (!spec->countKey.empty() && owner != nullptr) {
            std::int64_t active = spec->ordinal;
            if (const Value* count = owner->properties.find(spec->countKey))
                active = count->asInt(active);
            else if (const MechanismTemplateInfo* info = mechanisms_.templateInfo(owner->type))
                if (const PropertySpec* countSpec = info->properties.find(spec->countKey)) active = countSpec->defaultValue.asInt(active);
            if (active < spec->ordinal)
                add(out_, Severity::Warning, p,
                    "motor " + std::to_string(spec->ordinal) + " of '" + map.slot.owner + "' is not active (motor count is " +
                        std::to_string(active) + ")",
                    issue_code::kSlotMotorInactive);
        }
        if (capabilities_) {
            if (const std::optional<CapabilitySet> caps = capabilities_(map.device);
                caps && !caps->containsAll(requiredCapabilities(spec->type)))
                add(out_, Severity::Error, p,
                    "device " + map.device.toString() + " cannot serve slot '" + map.slot.toString() + "'",
                    issue_code::kSlotDeviceIncompatible);
        }
    }

    const RobotConfig& config_;
    const MechanismFactory& mechanisms_;
    const SlotSpecsByOwner& slotSpecsByOwner_;
    const ConfigValidator::DeviceCapabilityLookup& capabilities_;
    std::vector<Issue>& out_;
};

}  // namespace

void validateMechanisms(const RobotConfig& config, const MechanismFactory& mechanisms, SlotSpecsByOwner& slotSpecsByOwner,
                        std::vector<Issue>& out) {
    std::set<std::string> ids;
    for (std::size_t i = 0; i < config.mechanisms.size(); ++i) {
        const MechanismConfig& m = config.mechanisms[i];
        const std::string base = "mechanisms[" + std::to_string(i) + "]";
        if (m.id.empty()) {
            add(out, Severity::Error, base + ".id", "mechanism id must not be empty", issue_code::kMechanismIdInvalid);
        } else if (m.id == slot_owner::kDrivetrain || m.id == slot_owner::kRobot) {
            add(out, Severity::Error, base + ".id", "'" + m.id + "' is a reserved slot owner", issue_code::kMechanismIdInvalid);
        } else if (m.id.find('.') != std::string::npos) {
            add(out, Severity::Error, base + ".id", "mechanism id must not contain '.'", issue_code::kMechanismIdInvalid);
        } else if (!ids.insert(m.id).second) {
            add(out, Severity::Error, base + ".id", "duplicate mechanism id '" + m.id + "'", issue_code::kMechanismIdDuplicate);
        }

        const MechanismTemplateInfo* info = mechanisms.templateInfo(m.type);
        if (info == nullptr) {
            add(out, Severity::Error, base + ".type", "unknown mechanism type '" + m.type + "' (not registered; is its plugin selected?)",
                issue_code::kMechanismTypeUnknown);
            continue;
        }
        if (slotSpecsByOwner.find(m.id) == slotSpecsByOwner.end()) slotSpecsByOwner[m.id] = info->slotSpecs;
        for (Issue& issue : validateProperties(info->properties, m.properties, base + ".properties")) {
            if (issue.code.empty()) issue.code = issue_code::kPropertyInvalid;
            out.push_back(std::move(issue));
        }
    }
}

void addNonMechanismSlotSpecs(const RobotConfig& config, const DrivetrainRegistry& drivetrains, SlotSpecsByOwner& slotSpecsByOwner) {
    slotSpecsByOwner[slot_owner::kDrivetrain] = drivetrainSlotSpecs(drivetrains.info(config.drivetrain.spec.kind), config.drivetrain.spec);
    slotSpecsByOwner[slot_owner::kRobot] = robotSlotSpecs();
    for (const SensorConfig& s : config.sensors)
        if (!s.id.empty() && slotSpecsByOwner.find(s.id) == slotSpecsByOwner.end()) slotSpecsByOwner[s.id] = {sensorSlotSpec(s.kind)};
}

void validateSlotMappings(const RobotConfig& config, const MechanismFactory& mechanisms, const SlotSpecsByOwner& slotSpecsByOwner,
                          const ConfigValidator::DeviceCapabilityLookup& capabilities, std::vector<Issue>& out) {
    const MappingChecker checker(config, mechanisms, slotSpecsByOwner, capabilities, out);
    checker.checkGroup(config.robotMappings, slot_owner::kRobot, "robotMappings");
    checker.checkGroup(config.drivetrain.mappings, slot_owner::kDrivetrain, "drivetrain.mappings");
    for (std::size_t i = 0; i < config.mechanisms.size(); ++i)
        checker.checkGroup(config.mechanisms[i].mappings, config.mechanisms[i].id, "mechanisms[" + std::to_string(i) + "].mappings");
    for (std::size_t i = 0; i < config.sensors.size(); ++i)
        checker.checkGroup(config.sensors[i].mappings, config.sensors[i].id, "sensors[" + std::to_string(i) + "].mappings");
}

void validateMappingUniqueness(const std::vector<SlotMapping>& all, std::vector<Issue>& out) {
    std::map<SlotAddress, int> slotUses;
    std::map<DeviceKey, std::vector<SlotAddress>> deviceUses;
    for (const SlotMapping& m : all) {
        ++slotUses[m.slot];
        deviceUses[m.device].push_back(m.slot);
    }
    for (const auto& [slot, n] : slotUses)
        if (n > 1)
            add(out, Severity::Error, slot.toString(), "slot '" + slot.toString() + "' is mapped " + std::to_string(n) + " times",
                issue_code::kSlotDuplicateMapping);
    for (const auto& [key, slots] : deviceUses) {
        std::set<SlotAddress> distinct(slots.begin(), slots.end());
        if (distinct.size() > 1)
            add(out, Severity::Error, slots.front().toString(),
                "device " + key.toString() + " is mapped to " + std::to_string(distinct.size()) + " different slots (one device : one slot)",
                issue_code::kDeviceDuplicateMapping);
    }
}

void validateRequiredSlots(const RobotConfig& config, const std::vector<MotionAxisInfo>& motionAxes,
                           const SlotSpecsByOwner& slotSpecsByOwner, std::vector<Issue>& out) {
    auto requireSlots = [&](const std::string& owner, const std::vector<SlotMapping>& group, const std::string& path, Severity sev,
                            bool honourRequiredFlag) {
        const auto specs = slotSpecsByOwner.find(owner);
        if (specs == slotSpecsByOwner.end()) return;
        for (const SlotSpec& spec : specs->second) {
            if (motionFollowerMotor(config, motionAxes, {owner, spec.name})) continue;
            if (honourRequiredFlag && !spec.required) continue;
            bool mapped = false;
            for (const SlotMapping& m : group)
                if (m.slot.slot == spec.name) mapped = true;
            if (!mapped)
                add(out, sev, path, "slot '" + owner + "." + spec.name + "' (" + spec.label + ") is not mapped to a device",
                    issue_code::kSlotRequiredUnmapped);
        }
    };
    for (std::size_t i = 0; i < config.mechanisms.size(); ++i)
        requireSlots(config.mechanisms[i].id, config.mechanisms[i].mappings, "mechanisms[" + std::to_string(i) + "].mappings", Severity::Error, true);
    requireSlots(slot_owner::kDrivetrain, config.drivetrain.mappings, "drivetrain.mappings", Severity::Warning, false);
}

}  // namespace wpiemu::core::config::detail
