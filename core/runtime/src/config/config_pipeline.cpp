#include "wpiemu/core/config/config_pipeline.h"

namespace wpiemu::core::config {

using namespace wpiemu::sdk;

Status PluginRegistrar::ensureRegistered(IGameModule& module) {
    const std::string id = module.metadata().id;
    if (registered_.count(id) != 0) return okStatus();
    if (Status s = module.registerMechanisms(factory_); !s)
        return makeError(s.error().code, "plugin '" + id + "' failed to register its mechanisms: " + s.error().message);
    registered_.insert(id);
    return okStatus();
}

Result<LoadedConfig> ConfigPipeline::load(const Value& stored) const {
    Result<RobotConfig> decoded = codec_.fromValue(stored);
    if (!decoded) return decoded.error();
    return process(std::move(decoded).value());
}

Result<LoadedConfig> ConfigPipeline::process(RobotConfig config) const {
    LoadedConfig out;

    IGameModule* plugin = selectPlugin(config, plugins_);
    PluginConfigSchema schema;
    if (plugin != nullptr) {
        out.pluginId = plugin->metadata().id;
        if (Status s = registrar_.ensureRegistered(*plugin); !s) return s.error();
        schema = plugin->contributeConfigSchema();

        if (const auto found = config.plugins.find(out.pluginId); found != config.plugins.end() && found->second.schemaVersion != schema.schemaVersion)
            return makeError(ErrorCode::VersionMismatch, "plugin '" + out.pluginId + "' section version " + std::to_string(found->second.schemaVersion) +
                                                             " is not supported (expected " + std::to_string(schema.schemaVersion) + ")");
        // defaults: the plugin section always exists for the selected plugin
        PluginSection& section = config.pluginSection(out.pluginId);
        section.schemaVersion = schema.schemaVersion;
        applyDefaults(schema.properties, section.data);
    }

    for (MechanismConfig& m : config.mechanisms)
        if (const MechanismTemplateInfo* info = registrar_.factory().templateInfo(m.type)) applyDefaults(info->properties, m.properties);

    if (plugin != nullptr && !config.zones.empty()) {
        const std::vector<ZoneKindInfo> kinds = plugin->contributeZoneKinds();
        for (ZoneConfig& z : config.zones)
            for (const ZoneKindInfo& k : kinds)
                if (k.kind == z.kind) applyDefaults(k.properties, z.properties);
    }

    auto issues = validator_.validate(config, registrar_.factory(), drivetrains_, plugins_);
    out.issues.insert(out.issues.end(), issues.begin(), issues.end());
    out.config = std::move(config);
    return out;
}

}  // namespace wpiemu::core::config
