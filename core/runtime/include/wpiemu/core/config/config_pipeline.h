#pragma once

#include <set>
#include <string>
#include <vector>

#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::core::config {

// Registers a plugin's mechanism templates exactly once per MechanismFactory (idempotent), so the config
// pipeline, the session builder and the editor can all ask without double-registration errors.
// The composition root must route every `IGameModule::registerMechanisms` call through here.
class PluginRegistrar {
public:
    explicit PluginRegistrar(sdk::MechanismFactory& factory) : factory_(factory) {}
    sdk::Status ensureRegistered(sdk::IGameModule& module);
    bool isRegistered(const std::string& pluginId) const { return registered_.count(pluginId) != 0; }
    const sdk::MechanismFactory& factory() const { return factory_; }

private:
    sdk::MechanismFactory& factory_;
    std::set<std::string> registered_;
};

struct LoadedConfig {
    sdk::RobotConfig config;
    std::vector<sdk::Issue> issues;
    std::string pluginId;              // the selected plugin ("" = core features only)

    bool valid() const { return !ConfigValidator::hasErrors(issues); }
};

// The load pipeline of ARCHITECTURE section 4.4:
//   decode (current schema only) -> defaults -> validation.
// Construction of mechanisms/field/game state is the SessionBuilder's job and only happens for valid configs.
//  * A fatal problem (parse error, a core or plugin-section schemaVersion that is not the supported one,
//    a plugin that fails to register) is an Error; everything else is reported as issues next to a config.
//  * Selected = RobotConfig::gamePluginId, else the plugin registered for project.wpilibYear, else none.
class ConfigPipeline {
public:
    ConfigPipeline(const RobotConfigCodec& codec, const sdk::IConfigValidator& validator, PluginRegistrar& registrar,
                   const sdk::DrivetrainRegistry& drivetrains, const sdk::GamePluginRegistry& plugins)
        : codec_(codec), validator_(validator), registrar_(registrar), drivetrains_(drivetrains), plugins_(plugins) {}

    sdk::Result<LoadedConfig> load(const sdk::Value& stored) const;
    // For configs already decoded by a store.
    sdk::Result<LoadedConfig> process(sdk::RobotConfig config) const;

private:
    const RobotConfigCodec& codec_;
    const sdk::IConfigValidator& validator_;
    PluginRegistrar& registrar_;
    const sdk::DrivetrainRegistry& drivetrains_;
    const sdk::GamePluginRegistry& plugins_;
};

}  // namespace wpiemu::core::config
