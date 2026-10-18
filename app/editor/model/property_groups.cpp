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

PropertyGroup groupOf(std::string id, QString title, sdk::PropertySchema schema, Value values, bool collapsed = false) {
    PropertyGroup g;
    g.id = std::move(id);
    g.title = std::move(title);
    g.schema = std::move(schema);
    // Only this group's own keys: groups cut from one schema share the value object.
    if (values.isObject()) {
        Value own = Value::makeObject();
        for (const auto& p : g.schema.properties)
            if (const Value* v = values.find(p.key)) own[p.key] = *v;
        values = std::move(own);
    }
    g.values = std::move(values);
    g.collapsed = collapsed;
    return g;
}

template <class Pred>
sdk::PropertySchema subset(const sdk::PropertySchema& schema, Pred keep) {
    sdk::PropertySchema out;
    for (const auto& p : schema.properties)
        if (keep(p)) out.properties.push_back(p);
    return out;
}

// The "what is it" property (wheel kind, motion type) leads the Configure card.
bool isTypeKey(const std::string& key) {
    return key == "kind" || key == "motion" || (key.size() > 6 && key.compare(key.size() - 6, 6, "Motion") == 0);
}

void typeFirst(sdk::PropertySchema& schema) {
    std::stable_partition(schema.properties.begin(), schema.properties.end(), [](const PropertySpec& p) { return isTypeKey(p.key); });
}

QString axisTitle(const std::string& axis) {
    if (axis == "axis" || axis.empty()) return {};
    QString t = QString::fromStdString(axis);
    t[0] = t[0].toUpper();
    return t;
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
    const sdk::MechanismAttachment a = m.attachment;
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
    const sdk::MechanismAttachment att = m.attachment;
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
            if (const sdk::WheelSpec* w = ops::findWheel(c, sel.sub)) {
                const sdk::PropertySchema full = spec::wheelSchema(*w);
                const Value values = spec::wheelValues(*w);
                using S = sdk::PropertySection;
                auto in = [](S s) { return [s](const PropertySpec& p) { return p.key != "name" && p.section == s; }; };
                PropertyGroup drive = groupOf("wheel.drive", ETR("Drive"), subset(full, in(S::Drive)), values);
                drive.section = S::Drive;
                out.push_back(std::move(drive));
                sdk::PropertySchema configure = subset(full, in(S::Configure));
                typeFirst(configure);
                out.push_back(groupOf("wheel", ETR("Wheel"), std::move(configure), values));
                PropertyGroup place = groupOf("wheel.placement", ETR("Placement"), subset(full, in(S::Placement)), values);
                place.section = S::Placement;
                out.push_back(std::move(place));
            }
            break;
        case SelKind::Primitive:
            if (const auto* list = ops::primitivesOf(c, sel.owner); list != nullptr && sel.index >= 0 && sel.index < static_cast<int>(list->size()))
                out.push_back(groupOf("primitive", ETR("Shape"), spec::primitiveSchema(), spec::primitiveValues((*list)[static_cast<std::size_t>(sel.index)])));
            break;
        case SelKind::Mechanism: {
            const sdk::MechanismConfig* m = c.findMechanism(sel.owner);
            if (m == nullptr) break;
            if (const auto* info = catalog.findTemplate(m->type)) {
                using S = sdk::PropertySection;
                const Value values = withDefaults(info->properties, m->properties);
                // One Drive group per motion axis; drive properties without an axis go to the first one.
                std::vector<std::string> axes;
                for (const auto& p : info->properties.properties)
                    if (p.section == S::Drive && !p.axis.empty() && std::find(axes.begin(), axes.end(), p.axis) == axes.end()) axes.push_back(p.axis);
                for (const auto& p : info->properties.properties)
                    if (p.section == S::Drive && p.axis.empty()) {
                        if (axes.empty()) axes.push_back({});
                        break;
                    }
                for (std::size_t i = 0; i < axes.size(); ++i) {
                    const std::string axis = axes[i];
                    auto schema = subset(info->properties, [&](const PropertySpec& p) {
                        return p.section == S::Drive && (p.axis == axis || (p.axis.empty() && i == 0));
                    });
                    PropertyGroup g = groupOf("mech.drive:" + axis, axisTitle(axis), std::move(schema), values);
                    g.section = S::Drive;
                    g.axis = axis;
                    out.push_back(std::move(g));
                }
                sdk::PropertySchema configure = subset(info->properties, [](const PropertySpec& p) { return p.section != S::Drive; });
                typeFirst(configure);
                if (!configure.properties.empty())
                    out.push_back(groupOf("mech.props", QString::fromStdString(info->displayName), std::move(configure), values));
            }
            out.push_back(groupOf("mech.placement", ETR("Placement"), placementSchema(c, catalog, *m), placementValues(*m)));
            out.back().section = sdk::PropertySection::Placement;
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
    if (groupId == "wheel" || groupId == "wheel.drive" || groupId == "wheel.placement") { spec::applyWheel(c, sel.sub, key, value); return true; }
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
            return key == "label" ? (m->label = value.asString(m->label), true) : false;
        }
        if (groupId == "mech.placement") {
            if (key == "mount") { sdk::fromValue(value, m->mount); return true; }
            const sdk::MechanismAttachment att = m->attachment;
            if (key == "parent") {
                const std::string parent = value.asString(sdk::kChassisParent);
                return ops::reparentMechanism(c, catalog, m->id, parent, sdk::kRootLink);
            }
            if (key == "parentLink") return ops::reparentMechanism(c, catalog, m->id, att.parentMechanismId, value.asString(sdk::kRootLink));
            return false;
        }
        if (groupId == "mech.props" || isDriveGroup(groupId) || groupId.rfind("panel:", 0) == 0) {
            if (!m->properties.isObject()) m->properties = Value::makeObject();
            m->properties[key] = value;
            ops::pruneInactiveMappings(c, catalog, m->id);
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

QString mechanismKindLabel(const sdk::MechanismConfig& m, const sdk::MechanismTemplateInfo& info) {
    for (const auto& p : info.properties.properties) {
        if (p.key != "motion" || p.type != sdk::PropertyType::Enum) continue;
        const Value* stored = m.properties.isObject() ? m.properties.find(p.key) : nullptr;
        const std::string value = stored != nullptr ? stored->asString() : p.defaultValue.asString();
        for (const auto& o : p.options)
            if (o.value == value) return QString::fromStdString(o.label);
    }
    return QString::fromStdString(info.displayName);
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
