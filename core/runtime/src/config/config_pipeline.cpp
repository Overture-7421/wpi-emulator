#include "wpiemu/core/config/config_pipeline.h"

#include <optional>

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

namespace {

// The part of the model a plugin must not change: everything except its section and its own mechanisms.
Value protectedProjection(const RobotConfigCodec& codec, const RobotConfig& c, const std::string& pluginId) {
    Value v = codec.toValue(c);
    if (Value* plugins = v.object().find("plugins"); plugins != nullptr && plugins->isObject()) plugins->object().erase(pluginId);
    Value kept = Value::makeArray();
    if (const Value* mechs = v.find("mechanisms"); mechs != nullptr && mechs->isArray() && mechs->array().size() == c.mechanisms.size()) {
        for (std::size_t i = 0; i < c.mechanisms.size(); ++i)
            if (!ownsMechanismType(pluginId, c.mechanisms[i].type)) kept.push_back(mechs->array()[i]);
    }
    v["mechanisms"] = std::move(kept);
    return v;
}

std::optional<std::string> ownershipViolation(const RobotConfigCodec& codec, const RobotConfig& before, const RobotConfig& after,
                                              const std::string& pluginId) {
    const Value a = protectedProjection(codec, before, pluginId);
    const Value b = protectedProjection(codec, after, pluginId);
    if (a == b) return std::nullopt;
    for (std::size_t i = 0; i < a.object().size(); ++i) {
        const Value* other = b.find(a.object().keyAt(i));
        if (other == nullptr || !(*other == a.object().valueAt(i))) return a.object().keyAt(i);
    }
    return std::string("<added field>");
}

}  // namespace

Result<LoadedConfig> ConfigPipeline::load(const Value& stored) const {
    DecodeInfo info;
    Result<RobotConfig> decoded = codec_.fromValue(stored, &info);
    if (!decoded) return decoded.error();
    return process(std::move(decoded).value(), info);
}

Result<LoadedConfig> ConfigPipeline::process(RobotConfig config, const DecodeInfo& core) const {
    LoadedConfig out;
    out.core = core;

    IGameModule* plugin = selectPlugin(config, plugins_);
    PluginConfigSchema schema;
    if (plugin != nullptr) {
        out.pluginId = plugin->metadata().id;
        if (Status s = registrar_.ensureRegistered(*plugin); !s) return s.error();
        schema = plugin->contributeConfigSchema();

        if (const auto found = config.plugins.find(out.pluginId); found != config.plugins.end()) {
            PluginSection* section = &found->second;
            const int persisted = section->schemaVersion;
            out.persistedPluginVersion = persisted;
            if (persisted > schema.schemaVersion)
                return makeError(ErrorCode::VersionMismatch, "plugin '" + out.pluginId + "' section version " + std::to_string(persisted) +
                                                                 " is newer than supported " + std::to_string(schema.schemaVersion));
            if (persisted < schema.schemaVersion) {
                const RobotConfig before = config;
                int version = persisted;
                const auto migrators = plugin->configMigrators();
                if (Status s = migrateToVersion(section->data, version, schema.schemaVersion, migrators); !s)
                    return makeError(s.error().code, "plugin '" + out.pluginId + "' section migration failed: " + s.error().message);
                section->schemaVersion = schema.schemaVersion;
                if (Status s = plugin->migrateRobotConfig(config, persisted); !s)
                    return makeError(s.error().code, "plugin '" + out.pluginId + "' config migration failed: " + s.error().message);
                if (const auto violated = ownershipViolation(codec_, before, config, out.pluginId))
                    return makeError(ErrorCode::InvalidState, "plugin '" + out.pluginId + "' migration modified data it does not own (" +
                                                                  *violated + ")");
                out.pluginMigrated = true;
            }
        }
        // defaults: the plugin section always exists for the selected plugin
        PluginSection& section = config.pluginSection(out.pluginId);
        if (out.persistedPluginVersion == 0) section.schemaVersion = schema.schemaVersion;
        applyDefaults(schema.properties, section.data);
    }

    for (MechanismConfig& m : config.mechanisms)
        if (const MechanismTemplateInfo* info = registrar_.factory().templateInfo(m.type)) applyDefaults(info->properties, m.properties);

    out.issues = validator_.validate(config, registrar_.factory(), drivetrains_, plugins_);
    out.config = std::move(config);
    return out;
}

}  // namespace wpiemu::core::config
