#include "model/property_groups.h"

#include <algorithm>

#include "model/config_ops.h"
#include "model/spec_values.h"
#include "model/tr.h"
#include "wpiemu/sdk/config/value_codec.h"

namespace wpiemu::app::editor {

using sdk::PropertySpec;
using sdk::PropertyType;
using sdk::PropertyUnit;
using sdk::Value;

namespace {

PropertySpec textSpec(std::string key, std::string label, std::string group, Value def, std::string description = {}) {
    PropertySpec s;
    s.key = std::move(key);
    s.label = std::move(label);
    s.type = PropertyType::String;
    s.defaultValue = std::move(def);
    s.group = std::move(group);
    s.description = std::move(description);
    return s;
}

PropertyGroup groupOf(std::string id, QString title, sdk::PropertySchema schema, Value values, bool collapsed = false) {
    PropertyGroup g;
    g.id = std::move(id);
    g.title = std::move(title);
    g.schema = std::move(schema);
    g.values = std::move(values);
    g.collapsed = collapsed;
    return g;
}

Value withDefaults(const sdk::PropertySchema& schema, const Value& stored) {
    Value out = Value::makeObject();
    for (const auto& p : schema.properties) {
        const Value* v = stored.find(p.key);
        out[p.key] = v != nullptr ? *v : p.defaultValue;
    }
    return out;
}

std::vector<const sdk::EditorPanelDescriptor*> panelsFor(const EditorCatalog& catalog, sdk::PanelScope scope, const std::string& type) {
    std::vector<const sdk::EditorPanelDescriptor*> out;
    for (const auto& p : catalog.pluginPanels) {
        if (p.scope != scope) continue;
        if (scope == sdk::PanelScope::Mechanism &&
            std::find(p.mechanismTypes.begin(), p.mechanismTypes.end(), type) == p.mechanismTypes.end())
            continue;
        out.push_back(&p);
    }
    std::stable_sort(out.begin(), out.end(), [](auto* a, auto* b) { return a->order < b->order; });
    return out;
}

Value placementValues(const sdk::MechanismConfig& m) {
    Value o = Value::makeObject();
    const sdk::MechanismAttachment a = m.resolvedAttachment();
    o["parent"] = a.onChassis() ? std::string(sdk::kChassisParent) : a.parentMechanismId;
    o["parentLink"] = a.parentLink;
    o["mount"] = sdk::toValue(m.mount);
    return o;
}

sdk::PropertySchema placementSchema(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::MechanismConfig& m) {
    sdk::PropertySchema s;
    PropertySpec parent;
    parent.key = "parent";
    parent.label = ETR("Attached to").toStdString();
    parent.description = ETR("Mechanisms move with the part they are attached to. The placement is kept when you change it.").toStdString();
    parent.type = PropertyType::Enum;
    parent.group = ETR("Placement").toStdString();
    parent.defaultValue = std::string(sdk::kChassisParent);
    parent.options.push_back({sdk::kChassisParent, ETR("Chassis").toStdString()});
    const auto below = ops::descendantsOf(c, m.id);
    for (const auto& other : c.mechanisms) {
        if (other.id == m.id || std::find(below.begin(), below.end(), other.id) != below.end()) continue;
        parent.options.push_back({other.id, ops::displayNameFor(other, catalog)});
    }
    s.properties.push_back(parent);

    PropertySpec link;
    link.key = "parentLink";
    link.label = ETR("Attachment link").toStdString();
    link.description = ETR("Named frame of the parent this mechanism hangs on (e.g. an elevator carriage).").toStdString();
    link.type = PropertyType::Enum;
    link.group = parent.group;
    link.defaultValue = std::string(sdk::kRootLink);
    link.options.push_back({sdk::kRootLink, ETR("Origin").toStdString()});
    const sdk::MechanismAttachment att = m.resolvedAttachment();
    if (!att.onChassis())
        if (const sdk::MechanismConfig* pm = c.findMechanism(att.parentMechanismId))
            if (const auto* info = catalog.findTemplate(pm->type))
                for (const auto& l : info->links) link.options.push_back({l.name, l.label.empty() ? l.name : l.label});
    s.properties.push_back(link);

    PropertySpec mount;
    mount.key = "mount";
    mount.label = ETR("Mount pose").toStdString();
    mount.description = ETR("Relative to the attachment link: x forward, y left, z up.").toStdString();
    mount.type = PropertyType::Pose3;
    mount.unit = PropertyUnit::Meters;
    mount.group = parent.group;
    mount.defaultValue = sdk::toValue(sdk::Pose3{});
    s.properties.push_back(mount);
    return s;
}

sdk::PropertySchema generalSchema() {
    sdk::PropertySchema s;
    s.properties.push_back(textSpec("label", ETR("Name").toStdString(), ETR("General").toStdString(), "", ETR("Shown in the tree and the viewport.").toStdString()));
    PropertySpec id = textSpec("id", ETR("Instance id").toStdString(), ETR("General").toStdString(), "",
                               ETR("Stable id used in robot.json and slot names. Renaming updates mappings and children.").toStdString());
    id.advanced = true;
    s.properties.push_back(id);
    return s;
}

}  // namespace

SelectionRef formTargetOf(const sdk::RobotConfig& c, const SelectionRef& sel) {
    switch (sel.kind) {
        case SelKind::Slot:
            if (sel.owner == sdk::slot_owner::kRobot) return SelectionRef::robot();
            if (sel.owner == sdk::slot_owner::kDrivetrain) {
                const auto dot = sel.sub.rfind('.');
                if (dot != std::string::npos && ops::findWheel(c, sel.sub.substr(0, dot)) != nullptr)
                    return SelectionRef::wheel(sel.sub.substr(0, dot));
                return SelectionRef::drivetrain();
            }
            return SelectionRef::mechanism(sel.owner);
        case SelKind::Overlay:
            return sel.owner.empty() ? SelectionRef::plugin() : SelectionRef::mechanism(sel.owner);
        case SelKind::ChassisCom: return SelectionRef::chassis();
        default: return sel;
    }
}

std::vector<PropertyGroup> propertyGroupsFor(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& selIn) {
    const SelectionRef sel = formTargetOf(c, selIn);
    std::vector<PropertyGroup> out;
    switch (sel.kind) {
        case SelKind::Robot:
            out.push_back(groupOf("robot", ETR("Robot"), spec::robotSchema(), spec::robotValues(c)));
            break;
        case SelKind::Chassis:
            out.push_back(groupOf("chassis", ETR("Chassis"), spec::chassisSchema(), spec::chassisValues(c.chassis)));
            break;
        case SelKind::Drivetrain:
            out.push_back(groupOf("drivetrain", ETR("Drivetrain"), spec::drivetrainSchema(catalog), spec::drivetrainValues(c.drivetrain.spec)));
            break;
        case SelKind::Wheel:
            if (const sdk::WheelSpec* w = ops::findWheel(c, sel.sub))
                out.push_back(groupOf("wheel", ETR("Wheel"), spec::wheelSchema(*w), spec::wheelValues(*w)));
            break;
        case SelKind::Primitive:
            if (const auto* list = ops::primitivesOf(c, sel.owner); list != nullptr && sel.index >= 0 && sel.index < static_cast<int>(list->size()))
                out.push_back(groupOf("primitive", ETR("Shape"), spec::primitiveSchema(), spec::primitiveValues((*list)[static_cast<std::size_t>(sel.index)])));
            break;
        case SelKind::Mechanism: {
            const sdk::MechanismConfig* m = c.findMechanism(sel.owner);
            if (m == nullptr) break;
            Value general = Value::makeObject();
            general["label"] = m->label;
            general["id"] = m->id;
            out.push_back(groupOf("mech.general", ETR("Mechanism"), generalSchema(), general));
            out.push_back(groupOf("mech.placement", ETR("Placement"), placementSchema(c, catalog, *m), placementValues(*m)));
            if (const auto* info = catalog.findTemplate(m->type)) {
                if (!info->properties.properties.empty())
                    out.push_back(groupOf("mech.props", QString::fromStdString(info->displayName), info->properties, withDefaults(info->properties, m->properties)));
            }
            for (const auto* panel : panelsFor(catalog, sdk::PanelScope::Mechanism, m->type))
                out.push_back(groupOf("panel:" + panel->id, QString::fromStdString(panel->title), panel->properties, withDefaults(panel->properties, m->properties)));
            break;
        }
        case SelKind::Plugin: {
            if (!catalog.hasPlugin()) break;
            const sdk::PluginSection* section = c.findPlugin(catalog.pluginId);
            const Value stored = section != nullptr ? section->data : Value::makeObject();
            if (!catalog.pluginSchema.properties.properties.empty())
                out.push_back(groupOf("plugin", QString::fromStdString(catalog.pluginDisplayName), catalog.pluginSchema.properties, withDefaults(catalog.pluginSchema.properties, stored)));
            for (const auto* panel : panelsFor(catalog, sdk::PanelScope::PluginConfig, {}))
                out.push_back(groupOf("panel:" + panel->id, QString::fromStdString(panel->title), panel->properties, withDefaults(panel->properties, stored)));
            break;
        }
        default: break;
    }
    return out;
}

bool applyProperty(sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& selIn, const std::string& groupId,
                   const std::string& key, const Value& value) {
    const SelectionRef sel = formTargetOf(c, selIn);
    if (groupId == "robot") { spec::applyRobot(c, key, value); return true; }
    if (groupId == "chassis") { spec::applyChassis(c, catalog, key, value); return true; }
    if (groupId == "drivetrain") { spec::applyDrivetrain(c, key, value); return true; }
    if (groupId == "wheel") { spec::applyWheel(c, sel.sub, key, value); return true; }
    if (groupId == "primitive") {
        auto* list = ops::primitivesOf(c, sel.owner);
        if (list == nullptr || sel.index < 0 || sel.index >= static_cast<int>(list->size())) return false;
        spec::applyPrimitive((*list)[static_cast<std::size_t>(sel.index)], key, value);
        return true;
    }
    if (sel.kind == SelKind::Mechanism) {
        sdk::MechanismConfig* m = nullptr;
        for (auto& cand : c.mechanisms)
            if (cand.id == sel.owner) m = &cand;
        if (m == nullptr) return false;
        if (groupId == "mech.general") {
            if (key == "label") m->label = value.asString(m->label);
            else if (key == "id") return ops::renameMechanism(c, m->id, value.asString(m->id));
            return true;
        }
        if (groupId == "mech.placement") {
            if (key == "mount") { sdk::fromValue(value, m->mount); return true; }
            const sdk::MechanismAttachment att = m->resolvedAttachment();
            if (key == "parent") {
                const std::string parent = value.asString(sdk::kChassisParent);
                return ops::reparentMechanism(c, catalog, m->id, parent, sdk::kRootLink);
            }
            if (key == "parentLink") return ops::reparentMechanism(c, catalog, m->id, att.parentMechanismId, value.asString(sdk::kRootLink));
            return false;
        }
        if (groupId == "mech.props" || groupId.rfind("panel:", 0) == 0) {
            if (!m->properties.isObject()) m->properties = Value::makeObject();
            m->properties[key] = value;
            return true;
        }
    }
    if (sel.kind == SelKind::Plugin && (groupId == "plugin" || groupId.rfind("panel:", 0) == 0)) {
        if (!catalog.hasPlugin()) return false;
        Value& data = c.pluginSection(catalog.pluginId).data;
        if (!data.isObject()) data = Value::makeObject();
        data[key] = value;
        return true;
    }
    return false;
}

QString describeSelection(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel) {
    switch (sel.kind) {
        case SelKind::Robot: return QString::fromStdString(c.name);
        case SelKind::Chassis: return ETR("Chassis");
        case SelKind::ChassisCom: return ETR("Centre of mass");
        case SelKind::Drivetrain: return ETR("Drivetrain");
        case SelKind::Wheel: return ETR("Wheel %1").arg(QString::fromStdString(sel.sub));
        case SelKind::Primitive: {
            const auto* list = ops::primitivesOf(c, sel.owner);
            if (list != nullptr && sel.index >= 0 && sel.index < static_cast<int>(list->size()))
                return QString::fromStdString((*list)[static_cast<std::size_t>(sel.index)].name);
            return ETR("Shape");
        }
        case SelKind::Mechanism:
            if (const auto* m = c.findMechanism(sel.owner)) return QString::fromStdString(ops::displayNameFor(*m, catalog));
            return QString::fromStdString(sel.owner);
        case SelKind::Slot: return QString::fromStdString(ops::slotDisplayName(c, catalog, {sel.owner, sel.sub}));
        case SelKind::Overlay: return QString::fromStdString(sel.sub);
        case SelKind::Plugin: return QString::fromStdString(catalog.pluginDisplayName);
        case SelKind::Device: return QString::fromStdString(sel.owner);
        case SelKind::None: break;
    }
    return {};
}

}  // namespace wpiemu::app::editor
