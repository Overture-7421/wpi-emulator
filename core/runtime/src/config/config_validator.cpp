#include "wpiemu/core/config/config_validator.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>

#include "wpiemu/sdk/devices/device_identity.h"
#include "wpiemu/sdk/mechanisms/attachment.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

namespace wpiemu::core::config {

using namespace wpiemu::sdk;

namespace {

bool finitePositive(float v) { return std::isfinite(v) && v > 0.f; }
bool finiteNonNegative(float v) { return std::isfinite(v) && v >= 0.f; }
bool finite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

void add(std::vector<Issue>& out, Severity sev, std::string path, std::string message, const char* code) {
    out.push_back({sev, std::move(path), std::move(message), code});
}

void validateChassis(const RobotConfig& c, std::vector<Issue>& out) {
    const ChassisSpec& ch = c.chassis;
    const auto bad = [&](const std::string& field, const std::string& why) {
        add(out, Severity::Error, "chassis." + field, why, issue_code::kChassisInvalid);
    };
    if (!finite(ch.sizeM) || ch.sizeM.x <= 0.f || ch.sizeM.y <= 0.f || ch.sizeM.z <= 0.f) bad("sizeM", "chassis size must be positive and finite");
    if (!finitePositive(ch.massKg)) bad("massKg", "chassis mass must be positive");
    if (!finitePositive(ch.comHeightM)) bad("comHeightM", "COM height must be positive");
    if (!finiteNonNegative(ch.groundClearanceM)) bad("groundClearanceM", "ground clearance must be >= 0");
    if (!finite(ch.comOffsetM)) bad("comOffsetM", "COM offset must be finite");
    if (!finite(ch.inertiaDiagKgM2) || ch.inertiaDiagKgM2.x < 0.f || ch.inertiaDiagKgM2.y < 0.f || ch.inertiaDiagKgM2.z < 0.f)
        bad("inertiaDiagKgM2", "inertia must be >= 0 (0 = derive from the box)");
    if (!finiteNonNegative(ch.material.friction) || !finiteNonNegative(ch.material.restitution))
        bad("material", "friction and restitution must be >= 0");
}

void validateTire(const TireParams& t, const std::string& base, std::vector<Issue>& out) {
    const auto positive = [&](float v, const char* field, const char* why) {
        if (!finitePositive(v)) add(out, Severity::Error, base + "." + field, why, issue_code::kTireInvalid);
    };
    positive(t.peakSlipLong, "peakSlipLong", "peak longitudinal slip must be positive and finite");
    positive(t.peakSlipLat, "peakSlipLat", "peak lateral slip must be positive and finite");
    positive(t.vRefMps, "vRefMps", "reference creep velocity must be positive and finite");
    positive(t.lateralScale, "lateralScale", "lateral scale must be positive and finite");
    if (!std::isfinite(t.slideRatio)) add(out, Severity::Error, base + ".slideRatio", "slide ratio must be finite", issue_code::kTireInvalid);
    if (!std::isfinite(t.tractionAngleRad))
        add(out, Severity::Error, base + ".tractionAngleRad", "traction angle must be finite", issue_code::kTireInvalid);
}

void validateMotor(const MotorSpec& m, const std::string& base, std::vector<Issue>& out) {
    const auto positive = [&](double v, const char* field, const char* why) {
        if (!(std::isfinite(v) && v > 0.0)) add(out, Severity::Error, base + "." + field, why, issue_code::kMotorInvalid);
    };
    positive(m.nominalVoltageV, "nominalVoltageV", "nominal voltage must be positive and finite");
    positive(m.stallTorqueNm, "stallTorqueNm", "stall torque must be positive and finite");
    positive(m.stallCurrentA, "stallCurrentA", "stall current must be positive and finite");
    positive(m.freeSpeedRadPerSec, "freeSpeedRadPerSec", "free speed must be positive and finite");
    if (!(std::isfinite(m.freeCurrentA) && m.freeCurrentA >= 0.0))
        add(out, Severity::Error, base + ".freeCurrentA", "free current must be >= 0 and finite", issue_code::kMotorInvalid);
    else if (std::isfinite(m.stallCurrentA) && m.stallCurrentA > 0.0 && m.freeCurrentA >= m.stallCurrentA)
        add(out, Severity::Error, base + ".freeCurrentA", "free current must be below the stall current", issue_code::kMotorInvalid);
}

void validateDrivetrain(const RobotConfig& c, const DrivetrainRegistry& drivetrains, const TireModelRegistry* tires,
                        std::vector<Issue>& out) {
    const DrivetrainSpec& spec = c.drivetrain.spec;
    if (!drivetrains.contains(spec.kind))
        add(out, Severity::Error, "drivetrain.kind", "unknown drivetrain kind '" + spec.kind + "'", issue_code::kDrivetrainKindUnknown);
    if (tires != nullptr && !tires->contains(spec.tireModel))
        add(out, Severity::Error, "drivetrain.tireModel", "unknown tire model '" + spec.tireModel + "'", issue_code::kTireModelUnknown);
    if (spec.wheels.empty())
        add(out, Severity::Warning, "drivetrain.wheels", "the drivetrain has no wheels", issue_code::kDrivetrainNoWheels);

    std::set<std::string> names;
    for (std::size_t i = 0; i < spec.wheels.size(); ++i) {
        const WheelSpec& w = spec.wheels[i];
        const std::string base = "drivetrain.wheels[" + std::to_string(i) + "]";
        if (w.name.empty() || w.name.find('.') != std::string::npos)
            add(out, Severity::Error, base + ".name", "wheel name must be non-empty and must not contain '.'", issue_code::kWheelInvalid);
        else if (!names.insert(w.name).second)
            add(out, Severity::Error, base + ".name", "duplicate wheel name '" + w.name + "'", issue_code::kWheelDuplicate);
        if (!finitePositive(w.radiusM)) add(out, Severity::Error, base + ".radiusM", "wheel radius must be positive", issue_code::kWheelInvalid);
        if (!finite(w.positionM)) add(out, Severity::Error, base + ".positionM", "wheel position must be finite", issue_code::kWheelInvalid);
        if (w.driveMotorCount < 1 || w.driveMotorCount > kMaxMotorsPerAxis)
            add(out, Severity::Error, base + ".driveMotorCount",
                "drive motors per wheel must be in [1, " + std::to_string(kMaxMotorsPerAxis) + "]", issue_code::kWheelInvalid);
        if (!finitePositive(w.driveGearRatio)) add(out, Severity::Error, base + ".driveGearRatio", "gear ratio must be positive", issue_code::kWheelInvalid);
        if (!finitePositive(w.tire.mu)) add(out, Severity::Error, base + ".tire.mu", "friction coefficient must be positive", issue_code::kWheelInvalid);
        validateTire(w.tire, base + ".tire", out);
        validateMotor(w.driveMotor, base + ".driveMotor", out);
        if (w.steered || w.kind == WheelKind::Swerve) {
            if (!finitePositive(w.steerGearRatio))
                add(out, Severity::Error, base + ".steerGearRatio", "steer gear ratio must be positive", issue_code::kWheelInvalid);
            validateMotor(w.steerMotor, base + ".steerMotor", out);
        }
    }
}

void validateSimSettings(const RobotConfig& c, std::vector<Issue>& out) {
    constexpr std::int64_t kRobotPeriodUs = 20000;
    if (c.sim.stepUs <= 0 || kRobotPeriodUs % c.sim.stepUs != 0)
        add(out, Severity::Error, "sim.stepUs", "step must be positive and divide the 20 ms robot period evenly", issue_code::kSimInvalid);
    if (c.sim.physics.subSteps < 1) add(out, Severity::Error, "sim.physics.subSteps", "at least 1 sub-step", issue_code::kSimInvalid);
    if (c.sim.physics.workerCount < 1) add(out, Severity::Error, "sim.physics.workerCount", "at least 1 worker", issue_code::kSimInvalid);
    const BatteryConfig& b = c.battery;
    if (!(b.nominalVoltageV > 0.0) || !std::isfinite(b.nominalVoltageV))
        add(out, Severity::Error, "battery.nominalVoltageV", "nominal voltage must be positive", issue_code::kBatteryInvalid);
    if (!(b.internalResistanceOhm >= 0.0) || !std::isfinite(b.internalResistanceOhm))
        add(out, Severity::Error, "battery.internalResistanceOhm", "resistance must be >= 0", issue_code::kBatteryInvalid);
    if (!(b.brownoutVoltageV >= 0.0) || b.brownoutVoltageV >= b.nominalVoltageV)
        add(out, Severity::Error, "battery.brownoutVoltageV", "brownout voltage must be in [0, nominal)", issue_code::kBatteryInvalid);
}

const SlotSpec* findSpec(const std::vector<SlotSpec>& specs, const std::string& name) {
    for (const SlotSpec& s : specs)
        if (s.name == name) return &s;
    return nullptr;
}

}  // namespace

std::vector<SlotSpec> drivetrainSlotSpecs(const DrivetrainSpec& spec) {
    std::vector<SlotSpec> out;
    for (const WheelSpec& w : spec.wheels) {
        for (int i = 0; i < std::clamp(w.driveMotorCount, 1, kMaxMotorsPerAxis); ++i)
            out.push_back({driveSlotName(w.name, i), w.name + " drive motor" + (i > 0 ? " " + std::to_string(i + 1) : std::string()),
                           SlotType::Motor, false, {}});
        if (w.steered || w.kind == WheelKind::Swerve)
            out.push_back({w.name + ".steer", w.name + " steer motor", SlotType::Motor, false, {}});
    }
    return out;
}

std::vector<SlotSpec> robotSlotSpecs() { return {{"imu", "IMU", SlotType::Imu, false, "robot gyro"}}; }

IGameModule* selectPlugin(const RobotConfig& config, const GamePluginRegistry& plugins) {
    if (!config.gamePluginId.empty()) return plugins.find(config.gamePluginId);
    if (config.project.wpilibYear > 0) return plugins.findByYear(config.project.wpilibYear);
    return nullptr;
}

bool ConfigValidator::hasErrors(const std::vector<Issue>& issues) {
    for (const Issue& i : issues)
        if (i.severity == Severity::Error) return true;
    return false;
}

std::vector<Issue> ConfigValidator::validate(const RobotConfig& config, const MechanismFactory& mechanisms,
                                             const DrivetrainRegistry& drivetrains, const GamePluginRegistry& plugins) const {
    std::vector<Issue> out;

    if (config.schemaVersion != kRobotConfigSchemaVersion)
        add(out, Severity::Error, "schemaVersion",
            "schema version " + std::to_string(config.schemaVersion) + " is not the supported " + std::to_string(kRobotConfigSchemaVersion),
            issue_code::kSchemaVersion);

    validateChassis(config, out);
    validateDrivetrain(config, drivetrains, tireModels_, out);
    validateSimSettings(config, out);
    const auto motionAxes = motionAxisInfos(
        config, [&](std::string_view type) { return mechanisms.templateInfo(type); });
    for (auto& issue : motionLinkIssues(config, motionAxes)) out.push_back(std::move(issue));

    // ---- mechanisms: ids, templates, properties ----
    std::map<std::string, std::vector<SlotSpec>> slotSpecsByOwner;
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
    slotSpecsByOwner[slot_owner::kDrivetrain] = drivetrainSlotSpecs(config.drivetrain.spec);
    slotSpecsByOwner[slot_owner::kRobot] = robotSlotSpecs();

    // ---- attachments and aliases ----
    for (Issue& issue : validateMechanismAttachments(config, mechanisms)) out.push_back(std::move(issue));
    const std::vector<SlotMapping> all = config.allMappings();
    for (Issue& issue : validateDeviceAliases(all)) out.push_back(std::move(issue));

    // ---- mappings ----
    auto checkGroup = [&](const std::vector<SlotMapping>& group, const std::string& expectedOwner, const std::string& path) {
        for (std::size_t i = 0; i < group.size(); ++i) {
            const SlotMapping& map = group[i];
            const std::string p = path + "[" + std::to_string(i) + "]";
            if (map.slot.owner != expectedOwner) {
                add(out, Severity::Error, p, "slot owner '" + map.slot.owner + "' does not match '" + expectedOwner + "'",
                    issue_code::kSlotOwnerMismatch);
                continue;
            }
            const auto specs = slotSpecsByOwner.find(map.slot.owner);
            if (specs == slotSpecsByOwner.end()) continue;  // unknown template: already reported
            const SlotSpec* spec = findSpec(specs->second, map.slot.slot);
            const MechanismConfig* owner = nullptr;
            for (const MechanismConfig& m : config.mechanisms)
                if (m.id == map.slot.owner) owner = &m;
            if (spec == nullptr) {
                const Severity sev = map.slot.owner == slot_owner::kRobot ? Severity::Warning : Severity::Error;
                add(out, sev, p, "unknown slot '" + map.slot.toString() + "'", issue_code::kSlotUnknown);
                continue;
            }
            if (map.reversed && spec->type != SlotType::Motor)
                add(out, Severity::Warning, p, "only motor slots can be reversed ('" + map.slot.toString() + "')",
                    issue_code::kSlotReversedInvalid);
            if (!spec->countKey.empty() && owner != nullptr) {
                std::int64_t active = spec->ordinal;
                if (const Value* count = owner->properties.find(spec->countKey))
                    active = count->asInt(active);
                else if (const MechanismTemplateInfo* info = mechanisms.templateInfo(owner->type))
                    if (const PropertySpec* countSpec = info->properties.find(spec->countKey)) active = countSpec->defaultValue.asInt(active);
                if (active < spec->ordinal)
                    add(out, Severity::Warning, p,
                        "motor " + std::to_string(spec->ordinal) + " of '" + map.slot.owner + "' is not active (motor count is " +
                            std::to_string(active) + ")",
                        issue_code::kSlotMotorInactive);
            }
            if (capabilities_) {
                if (const std::optional<CapabilitySet> caps = capabilities_(map.device);
                    caps && !caps->containsAll(requiredCapabilities(spec->type)))
                    add(out, Severity::Error, p,
                        "device " + map.device.toString() + " cannot serve slot '" + map.slot.toString() + "'",
                        issue_code::kSlotDeviceIncompatible);
            }
        }
    };
    checkGroup(config.robotMappings, slot_owner::kRobot, "robotMappings");
    checkGroup(config.drivetrain.mappings, slot_owner::kDrivetrain, "drivetrain.mappings");
    for (std::size_t i = 0; i < config.mechanisms.size(); ++i)
        checkGroup(config.mechanisms[i].mappings, config.mechanisms[i].id, "mechanisms[" + std::to_string(i) + "].mappings");

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

    // ---- required slots ----
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

    // ---- plugins ----
    IGameModule* selected = selectPlugin(config, plugins);
    if (!config.gamePluginId.empty() && selected == nullptr)
        add(out, Severity::Warning, "gamePluginId", "game plugin '" + config.gamePluginId + "' is not available (core features only)",
            issue_code::kPluginNotFound);
    for (const auto& [id, section] : config.plugins) {
        const std::string path = "plugins." + id;
        IGameModule* module = plugins.find(id);
        if (module == nullptr) {
            add(out, Severity::Warning, path, "section of unknown plugin '" + id + "' is kept but ignored", issue_code::kPluginUnknownSection);
            continue;
        }
        if (selected != module) continue;  // another registered plugin's section: not ours to judge
        const PluginConfigSchema schema = module->contributeConfigSchema();
        if (section.schemaVersion != schema.schemaVersion)
            add(out, Severity::Error, path + ".schemaVersion",
                "section version " + std::to_string(section.schemaVersion) + " is not the supported " + std::to_string(schema.schemaVersion),
                issue_code::kPluginSectionVersion);
        for (Issue& issue : validateProperties(schema.properties, section.data, path + ".data")) {
            if (issue.code.empty()) issue.code = issue_code::kPluginPropertyInvalid;
            out.push_back(std::move(issue));
        }
    }
    return out;
}

}  // namespace wpiemu::core::config
