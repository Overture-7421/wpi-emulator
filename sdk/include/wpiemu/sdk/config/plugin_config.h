#pragma once

// Plugin-owned config sections.
//
// robot.json stores each plugin's data under a namespaced key and per-section version:
//   "plugins": { "game_2026": { "schemaVersion": 1, "data": { ... } } }
// Only the owning plugin interprets `data`. The SDK stays at version 1 until release (docs/CONTRIBUTING.md): a section
// whose version differs from the plugin's `schemaVersion` is rejected, there is no migration.

#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/value.h"

namespace wpiemu::sdk {

// What a plugin contributes to the config model.
struct PluginConfigSchema {
    int schemaVersion = 1;       // version of this plugin's section; any other persisted version is rejected
    PropertySchema properties;   // shown by the generic inspector under the plugin's name
    // Optional: grouped display order of property groups (empty = order of first appearance).
    std::vector<std::string> groupOrder;
};

// ---- ownership ----------------------------------------------------------------------------------
// Plugins may register mechanism templates only under their own id prefix ("<pluginId>.<name>", see
// IGameModule::registerMechanisms). A plugin OWNS:
//   * RobotConfig::plugins[<pluginId>] (its section), and
//   * every MechanismConfig whose `type` satisfies ownsPluginId(<pluginId>, type), including that
//     mechanism's properties, geometry, mount, attachment and slot mappings.
// It owns NOTHING else: core fields (chassis, drivetrain, battery, sim, project, robotMappings), other
// plugins' sections and core/other-plugin mechanisms are not the plugin's to change.
// The same "<pluginId>.<name>" rule covers the zone kinds (ZoneKindInfo::kind) and game values
// (GameValueInfo::id) a plugin declares.
inline bool ownsPluginId(std::string_view pluginId, std::string_view id) {
    return !pluginId.empty() && id.size() > pluginId.size() + 1 && id.substr(0, pluginId.size()) == pluginId &&
           id[pluginId.size()] == '.';
}

}  // namespace wpiemu::sdk
