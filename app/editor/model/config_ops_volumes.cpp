#include <algorithm>

#include "model/config_ops.h"

namespace wpiemu::app::editor::ops {

using sdk::MechanismConfig;
using sdk::RobotConfig;

namespace {

const char* const kVolumeColors[] = {"#4488ff", "#ff9944", "#44cc88", "#cc55bb"};

void addVolume(std::vector<VolumeRef>& out, const sdk::PropertySchema& schema,
               const std::vector<sdk::ViewportOverlayDescriptor>& overlays, std::string_view owner) {
    auto exists = [&](const std::string& key) {
        return std::any_of(out.begin(), out.end(), [&](const VolumeRef& v) { return v.key == key; });
    };
    for (const auto& ov : overlays) {
        const sdk::PropertySpec* spec = schema.find(ov.propertyKey);
        if (spec == nullptr || spec->type != sdk::PropertyType::Box3 || exists(ov.propertyKey)) continue;
        out.push_back({std::string(owner), ov.propertyKey, ov.label.empty() ? spec->label : ov.label, ov.color,
                       ov.editable, spec->defaultValue});
    }
    for (const auto& spec : schema.properties) {
        if (spec.type != sdk::PropertyType::Box3 || exists(spec.key)) continue;
        out.push_back({std::string(owner), spec.key, spec.label, kVolumeColors[out.size() % 4], true, spec.defaultValue});
    }
}

}  // namespace

// ---- plugin-defined volumes -------------------------------------------------------------------
std::vector<VolumeRef> volumesFor(const RobotConfig& c, const EditorCatalog& catalog, std::string_view mechanismId) {
    std::vector<VolumeRef> out;
    if (mechanismId.empty()) {
        if (!catalog.hasPlugin()) return out;
        for (const auto& panel : catalog.pluginPanels)
            if (panel.scope == sdk::PanelScope::PluginConfig) addVolume(out, panel.properties, panel.overlays, "");
        addVolume(out, catalog.pluginSchema.properties, {}, "");
        return out;
    }
    const MechanismConfig* m = c.findMechanism(mechanismId);
    if (m == nullptr) return out;
    for (const auto& panel : catalog.pluginPanels) {
        if (panel.scope != sdk::PanelScope::Mechanism) continue;
        if (std::find(panel.mechanismTypes.begin(), panel.mechanismTypes.end(), m->type) == panel.mechanismTypes.end()) continue;
        addVolume(out, panel.properties, panel.overlays, mechanismId);
    }
    if (const auto* info = catalog.findTemplate(m->type)) addVolume(out, info->properties, {}, mechanismId);
    return out;
}

sdk::Value* volumeStore(RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    if (owner.empty()) {
        if (!catalog.hasPlugin()) return nullptr;
        return &c.pluginSection(catalog.pluginId).data;
    }
    if (MechanismConfig* m = c.findMechanism(owner)) return &m->properties;
    return nullptr;
}

const sdk::Value* volumeStore(const RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    if (owner.empty()) {
        if (!catalog.hasPlugin()) return nullptr;
        const sdk::PluginSection* s = c.findPlugin(catalog.pluginId);
        return s != nullptr ? &s->data : nullptr;
    }
    if (const MechanismConfig* m = c.findMechanism(owner)) return &m->properties;
    return nullptr;
}

}  // namespace wpiemu::app::editor::ops
