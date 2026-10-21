#include "wpiemu/core/config/config_validator.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>

#include "config_validator_steps.h"
#include "wpiemu/sdk/config/config_checks.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/devices/device_identity.h"
#include "wpiemu/sdk/mechanisms/attachment.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

namespace wpiemu::core::config {

using namespace wpiemu::sdk;
using namespace detail;

namespace {

void validateSchemaVersion(const RobotConfig& config, std::vector<Issue>& out) {
    if (config.schemaVersion != kRobotConfigSchemaVersion)
        add(out, Severity::Error, "schemaVersion",
            "schema version " + std::to_string(config.schemaVersion) + " is not the supported " + std::to_string(kRobotConfigSchemaVersion),
            issue_code::kSchemaVersion);
}

void validateStartPose(const RobotConfig& config, std::vector<Issue>& out) {
    if (config.startPose &&
        (!std::isfinite(config.startPose->xM) || !std::isfinite(config.startPose->yM) ||
         !std::isfinite(config.startPose->headingRad)))
        add(out, Severity::Error, "startPose", "start pose must have finite x, y and heading",
            issue_code::kStartPoseInvalid);
}

// What the shared zone / camera / sensor checks (sdk/config/config_checks.h) ask of the registries and the selected plugin.
class ValidationCatalog final : public IConfigCatalog {
public:
    ValidationCatalog(const MechanismFactory& mechanisms, const IGameModule* plugin)
        : mechanisms_(mechanisms),
          pluginId_(plugin != nullptr ? plugin->metadata().id : std::string()),
          zoneKinds_(plugin != nullptr ? plugin->contributeZoneKinds() : std::vector<ZoneKindInfo>{}),
          gameValues_(plugin != nullptr ? plugin->contributeGameValues() : std::vector<GameValueInfo>{}) {}

    const MechanismTemplateInfo* mechanismTemplate(std::string_view type) const override { return mechanisms_.templateInfo(type); }
    std::string_view gamePluginId() const override { return pluginId_; }
    const ZoneKindInfo* zoneKind(std::string_view kind) const override {
        const auto it = std::find_if(zoneKinds_.begin(), zoneKinds_.end(), [&](const ZoneKindInfo& k) { return k.kind == kind; });
        return it == zoneKinds_.end() ? nullptr : &*it;
    }
    const GameValueInfo* gameValue(std::string_view id) const override {
        const auto it = std::find_if(gameValues_.begin(), gameValues_.end(), [&](const GameValueInfo& v) { return v.id == id; });
        return it == gameValues_.end() ? nullptr : &*it;
    }

private:
    const MechanismFactory& mechanisms_;
    std::string pluginId_;
    std::vector<ZoneKindInfo> zoneKinds_;
    std::vector<GameValueInfo> gameValues_;
};

void validatePluginSections(const RobotConfig& config, const GamePluginRegistry& plugins, IGameModule* selected, std::vector<Issue>& out) {
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
}

// The zone, camera, sensor and start-position rules shared with the editor's fake backend (sdk/config/config_checks.h).
void validateAgainstCatalog(const RobotConfig& config, const MechanismFactory& mechanisms, IGameModule* selected, std::vector<Issue>& out) {
    const ValidationCatalog catalog(mechanisms, selected);
    validateZones(config, catalog, out);
    validateCameras(config, catalog, out);
    validateSensors(config, catalog, out);
    validateStartPositions(config, out);
}

}  // namespace

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

    validateSchemaVersion(config, out);
    validateChassis(config, out);
    validateStartPose(config, out);
    validateDrivetrain(config, drivetrains, tireModels_, out);
    validateSimSettings(config, out);
    const auto motionAxes = motionAxisInfos(
        config, [&](std::string_view type) { return mechanisms.templateInfo(type); });
    for (auto& issue : motionLinkIssues(config, motionAxes)) out.push_back(std::move(issue));

    SlotSpecsByOwner slotSpecsByOwner;
    validateMechanisms(config, mechanisms, slotSpecsByOwner, out);
    addNonMechanismSlotSpecs(config, drivetrains, slotSpecsByOwner);

    for (Issue& issue : validateMechanismAttachments(config, mechanisms)) out.push_back(std::move(issue));
    const std::vector<SlotMapping> all = config.allMappings();
    for (Issue& issue : validateDeviceAliases(all)) out.push_back(std::move(issue));

    validateSlotMappings(config, mechanisms, slotSpecsByOwner, capabilities_, out);
    validateMappingUniqueness(all, out);
    validateRequiredSlots(config, motionAxes, slotSpecsByOwner, out);

    IGameModule* selected = selectPlugin(config, plugins);
    validatePluginSections(config, plugins, selected, out);
    validateAgainstCatalog(config, mechanisms, selected, out);
    return out;
}

}  // namespace wpiemu::core::config
