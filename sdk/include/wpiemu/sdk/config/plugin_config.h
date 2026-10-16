#pragma once

// Plugin-owned config sections with versioned migrations.
//
// robot.json stores each plugin's data under a namespaced key and per-section version:
//   "plugins": { "game_2026": { "schemaVersion": 2, "data": { ... } } }
// Only the owning plugin interprets `data`. When a section is older than the plugin's current
// version, the chain of the plugin's IConfigMigrators is applied in order (v1->v2, v2->v3, ...).
// The core robot schema uses the same mechanism (see RobotConfig::schemaVersion).

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/value.h"

namespace wpiemu::sdk {

// One upgrade step: transforms a config `Value` from `fromVersion()` to `toVersion()` in place.
// toVersion() MUST equal fromVersion() + 1 (steps are small and composable).
class IConfigMigrator {
    WPIEMU_INTERFACE(IConfigMigrator)
public:
    virtual int fromVersion() const = 0;
    virtual int toVersion() const = 0;
    virtual Status migrate(Value& data) const = 0;
};

// Applies migrators until `data` reaches `targetVersion`. `version` is updated in place.
// Errors: VersionMismatch if the data is NEWER than target (downgrade unsupported) or a step is missing.
inline Status migrateToVersion(Value& data, int& version, int targetVersion,
                               const std::vector<std::unique_ptr<IConfigMigrator>>& migrators) {
    if (version > targetVersion)
        return makeError(ErrorCode::VersionMismatch,
                         "config version " + std::to_string(version) + " is newer than supported " +
                             std::to_string(targetVersion));
    while (version < targetVersion) {
        const IConfigMigrator* step = nullptr;
        for (const auto& m : migrators)
            if (m->fromVersion() == version) { step = m.get(); break; }
        if (step == nullptr)
            return makeError(ErrorCode::VersionMismatch,
                             "no migrator from config version " + std::to_string(version));
        if (step->toVersion() != version + 1)
            return makeError(ErrorCode::InvalidState, "migrator must advance exactly one version");
        if (Status s = step->migrate(data); !s) return s;
        version = step->toVersion();
    }
    return okStatus();
}

// What a plugin contributes to the config model.
struct PluginConfigSchema {
    int schemaVersion = 1;       // current version of this plugin's section
    PropertySchema properties;   // shown by the generic inspector under the plugin's name
    // Optional: grouped display order of property groups (empty = order of first appearance).
    std::vector<std::string> groupOrder;
};

// ---- ownership (SDK 1.1, ADR 0006) --------------------------------------------------------------
// Plugins may register mechanism templates only under their own id prefix ("<pluginId>.<name>", see
// IGameModule::registerMechanisms). A plugin OWNS:
//   * RobotConfig::plugins[<pluginId>] (its section), and
//   * every MechanismConfig whose `type` satisfies ownsMechanismType(<pluginId>, type), including that
//     mechanism's properties, geometry, mount, attachment and slot mappings.
// It owns NOTHING else: IGameModule::migrateRobotConfig must not touch core fields (chassis, drivetrain,
// battery, sim, project, robotMappings), other plugins' sections, or core/other-plugin mechanisms.
inline bool ownsMechanismType(std::string_view pluginId, std::string_view mechanismType) {
    return !pluginId.empty() && mechanismType.size() > pluginId.size() + 1 &&
           mechanismType.substr(0, pluginId.size()) == pluginId && mechanismType[pluginId.size()] == '.';
}

}  // namespace wpiemu::sdk
