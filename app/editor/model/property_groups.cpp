#include "model/property_groups.h"

#include <algorithm>

#include "model/config_ops.h"
#include "model/spec_values.h"
#include "model/tr.h"
#include "model/units.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"

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

// The "parent" and "parentLink" values every Placement group shows.
void putAttachmentValues(Value& o, const sdk::MechanismAttachment& att) {
    o["parent"] = att.onChassis() ? std::string(sdk::kChassisParent) : att.parentMechanismId;
    o["parentLink"] = att.parentLink;
}

// What differs between the Placement groups of mechanisms, zones and cameras.
struct PlacementOptions {
    const sdk::MechanismConfig* self = nullptr;   // a mechanism cannot hang on itself or on a part that hangs on it
    const char* poseKey = "local";                // mechanisms keep their pose under "mount"
    QString poseLabel;                            // empty = "Pose"
};

// The Placement properties mechanisms, zones and cameras share: what the object hangs on, which frame of it, and the pose
// there. Returns the property group they were put in.
std::string addPlacementProperties(sdk::PropertySchema& s, const sdk::RobotConfig& c, const EditorCatalog& catalog,
                                   const sdk::MechanismAttachment& att, const QString& attachedToHint, const QString& linkHint,
                                   const QString& poseHint, const PlacementOptions& options = {}) {
    PropertySpec parent;
    parent.key = "parent";
    parent.label = ETR("Attached to").toStdString();
    parent.description = attachedToHint.toStdString();
    parent.type = PropertyType::Enum;
    parent.group = ETR("Placement").toStdString();
    parent.defaultValue = std::string(sdk::kChassisParent);
    parent.options.push_back({sdk::kChassisParent, ETR("Chassis").toStdString()});
    const std::vector<std::string> below = options.self != nullptr ? ops::descendantsOf(c, options.self->id) : std::vector<std::string>{};
    for (const auto& m : c.mechanisms) {
        if (options.self != nullptr && (m.id == options.self->id || std::find(below.begin(), below.end(), m.id) != below.end())) continue;
        parent.options.push_back({m.id, ops::displayNameFor(m, catalog)});
    }
    if (!att.onChassis() && c.findMechanism(att.parentMechanismId) == nullptr)
        parent.options.push_back({att.parentMechanismId, att.parentMechanismId + " " + ETR("(missing)").toStdString()});
    s.properties.push_back(parent);

    PropertySpec link;
    link.key = "parentLink";
    link.label = ETR("Attachment link").toStdString();
    link.description = linkHint.toStdString();
    link.type = PropertyType::Enum;
    link.group = parent.group;
    link.defaultValue = std::string(sdk::kRootLink);
    link.options.push_back({sdk::kRootLink, ETR("Origin").toStdString()});
    if (!att.onChassis())
        if (const sdk::MechanismConfig* pm = c.findMechanism(att.parentMechanismId))
            if (const auto* info = catalog.findTemplate(pm->type))
                for (const auto& l : info->links) link.options.push_back({l.name, l.label.empty() ? l.name : l.label});
    s.properties.push_back(link);

    PropertySpec pose;
    pose.key = options.poseKey;
    pose.label = (options.poseLabel.isEmpty() ? ETR("Pose") : options.poseLabel).toStdString();
    pose.description = poseHint.toStdString();
    pose.type = PropertyType::Pose3;
    pose.unit = PropertyUnit::Meters;
    pose.group = parent.group;
    pose.defaultValue = sdk::toValue(sdk::Pose3{});
    s.properties.push_back(pose);
    return parent.group;
}

Value placementValues(const sdk::MechanismConfig& m) {
    Value o = Value::makeObject();
    putAttachmentValues(o, m.attachment);
    o["mount"] = sdk::toValue(m.mount);
    return o;
}

sdk::PropertySchema placementSchema(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::MechanismConfig& m) {
    sdk::PropertySchema s;
    addPlacementProperties(
        s, c, catalog, m.attachment, ETR("Mechanisms move with the part they are attached to. The placement is kept when you change it."),
        ETR("Named frame of the parent this mechanism hangs on (e.g. an elevator carriage)."),
        ETR("Relative to the attachment link: x forward, y left, z up."), {&m, "mount", ETR("Mount pose")});
    return s;
}

sdk::PropertySchema zoneGeneralSchema() {
    sdk::PropertySchema s;
    PropertySpec label;
    label.key = "label";
    label.label = ETR("Label").toStdString();
    label.description = ETR("Name shown in the project tree and the viewport.").toStdString();
    label.type = PropertyType::String;
    label.defaultValue = std::string();
    s.properties.push_back(label);
    return s;
}

sdk::PropertySchema startPositionGeneralSchema() {
    sdk::PropertySchema s = zoneGeneralSchema();
    s.properties[0].description = ETR("Name shown in the project tree and in the run view's start position menu.").toStdString();
    PropertySpec alliance;
    alliance.key = "alliance";
    alliance.label = ETR("Alliance").toStdString();
    alliance.description = ETR("Choosing this position in the run view also switches the Driver Station to this alliance.").toStdString();
    alliance.type = PropertyType::Enum;
    alliance.defaultValue = std::string(sdk::allianceKey(sdk::Alliance::Blue));
    alliance.options.push_back({std::string(sdk::allianceKey(sdk::Alliance::Blue)), ETR("Blue").toStdString()});
    alliance.options.push_back({std::string(sdk::allianceKey(sdk::Alliance::Red)), ETR("Red").toStdString()});
    s.properties.push_back(std::move(alliance));
    return s;
}

sdk::PropertySchema startPositionPlacementSchema() {
    sdk::PropertySchema s;
    const std::string group = ETR("Placement").toStdString();
    const auto number = [&](const char* key, QString label, PropertyUnit unit, QString description) {
        PropertySpec p;
        p.key = key;
        p.label = label.toStdString();
        p.description = description.toStdString();
        p.type = PropertyType::Double;
        p.unit = unit;
        p.group = group;
        p.defaultValue = 0.0;
        return p;
    };
    s.properties.push_back(number("xM", ETR("X"), PropertyUnit::Meters, ETR("Field X: from the blue alliance wall toward the red one.")));
    s.properties.push_back(number("yM", ETR("Y"), PropertyUnit::Meters, ETR("Field Y: to the left when facing the red wall.")));
    s.properties.push_back(number("headingRad", ETR("Heading"), PropertyUnit::Radians, ETR("Counter-clockwise from +X (toward the red wall).")));
    return s;
}

bool applyStartPosition(sdk::StartPosition& p, const std::string& groupId, const std::string& key, const Value& value) {
    if (groupId == "startpos.general") {
        if (key == "label") { p.label = value.asString(p.label); return true; }
        if (key == "alliance") {
            p.alliance = value.asString() == sdk::allianceKey(sdk::Alliance::Red) ? sdk::Alliance::Red : sdk::Alliance::Blue;
            return true;
        }
        return false;
    }
    if (groupId == "startpos.placement") {
        const auto num = [&](float fallback) { return value.isNumber() ? static_cast<float>(value.asDouble()) : fallback; };
        if (key == "xM") p.pose.xM = num(p.pose.xM);
        else if (key == "yM") p.pose.yM = num(p.pose.yM);
        else if (key == "headingRad") p.pose.headingRad = num(p.pose.headingRad);
        else return false;
        return true;
    }
    return false;
}

sdk::PropertySchema zonePlacementSchema(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneConfig& z, bool volume) {
    sdk::PropertySchema s;
    const std::string group = addPlacementProperties(
        s, c, catalog, z.attachment, ETR("A zone moves with the part it is attached to. The placement is kept when you change it."),
        ETR("Named frame of the parent the zone hangs on (e.g. a turret's moving part)."),
        ETR("Relative to the attachment link: x forward, y left, z up."));

    if (volume) {
        PropertySpec size;
        size.key = "sizeM";
        size.label = ETR("Size").toStdString();
        size.description = ETR("Full extents of the volume along its own axes.").toStdString();
        size.type = PropertyType::Vec3;
        size.unit = PropertyUnit::Meters;
        size.group = group;
        size.defaultValue = sdk::toValue(sdk::Vec3{0.3f, 0.3f, 0.3f});
        s.properties.push_back(size);
    }
    return s;
}

sdk::PropertySchema cameraGeneralSchema() {
    sdk::PropertySchema s;
    PropertySpec label;
    label.key = "label";
    label.label = ETR("Label").toStdString();
    label.description = ETR("Name shown in the project tree and the viewport.").toStdString();
    label.type = PropertyType::String;
    label.defaultValue = std::string();
    s.properties.push_back(label);

    PropertySpec kind;
    kind.key = "kind";
    kind.label = ETR("Type").toStdString();
    kind.description = ETR("Which coprocessor the camera behaves like: it decides the NetworkTables data that is published.").toStdString();
    kind.type = PropertyType::Enum;
    kind.defaultValue = std::string(sdk::cameraKindKey(sdk::CameraKind::Limelight));
    kind.options.push_back({sdk::cameraKindKey(sdk::CameraKind::Limelight), "Limelight"});
    kind.options.push_back({sdk::cameraKindKey(sdk::CameraKind::PhotonVision), "PhotonVision"});
    s.properties.push_back(kind);

    PropertySpec name;
    name.key = "name";
    name.label = ETR("Camera name").toStdString();
    name.description = ETR("The name the robot code uses. Unique among cameras of the same type.").toStdString();
    name.type = PropertyType::String;
    name.defaultValue = std::string();
    s.properties.push_back(name);
    return s;
}

PropertySpec cameraNumber(const char* key, const QString& label, const QString& description, double def, double min, double max,
                          double step, PropertyUnit unit = PropertyUnit::None, PropertyType type = PropertyType::Double) {
    PropertySpec p;
    p.key = key;
    p.label = label.toStdString();
    p.description = description.toStdString();
    p.type = type;
    p.unit = unit;
    p.defaultValue = type == PropertyType::Int ? Value(static_cast<std::int64_t>(def)) : Value(def);
    p.min = min;
    p.max = max;
    p.step = step;
    return p;
}

sdk::PropertySchema cameraOpticsSchema() {
    sdk::PropertySchema s;
    using T = PropertyType;
    using U = PropertyUnit;
    s.properties.push_back(cameraNumber("hFovDeg", ETR("Horizontal FOV (\xC2\xB0)"), ETR("Horizontal field of view in degrees."), 75.0, 1.0, 179.0, 1.0));
    s.properties.push_back(cameraNumber("vFovDeg", ETR("Vertical FOV (\xC2\xB0)"), ETR("Vertical field of view in degrees."), 50.0, 1.0, 179.0, 1.0));
    s.properties.push_back(cameraNumber("maxRangeM", ETR("Max range"), ETR("Tags farther than this are not seen."), 6.0, 0.1, 100.0, 0.5, U::Meters));
    s.properties.push_back(cameraNumber("maxIncidenceDeg", ETR("Max incidence (\xC2\xB0)"),
                                        ETR("Tags viewed more obliquely than this angle to their normal are not seen."), 75.0, 1.0, 90.0, 1.0));
    s.properties.push_back(cameraNumber("fps", ETR("Frame rate (fps)"), ETR("Capture rate of the camera."), 30.0, 1.0, 240.0, 1.0));
    s.properties.push_back(cameraNumber("latencyMs", ETR("Latency (ms)"), ETR("Delay between capture and the results reaching the robot."), 40.0, 0.0, 1000.0, 1.0));
    s.properties.push_back(cameraNumber("widthPx", ETR("Resolution width (px)"), ETR("Image width used for corners and area."), 1280.0, 16.0, 8192.0, 16.0, U::Count, T::Int));
    s.properties.push_back(cameraNumber("heightPx", ETR("Resolution height (px)"), ETR("Image height used for corners and area."), 800.0, 16.0, 8192.0, 16.0, U::Count, T::Int));
    s.properties.push_back(cameraNumber("positionNoiseM", ETR("Position noise"), ETR("1-sigma noise on the estimated robot position (x, y)."), 0.0, 0.0, 5.0, 0.01, U::Meters));
    s.properties.push_back(cameraNumber("yawNoiseDeg", ETR("Yaw noise (\xC2\xB0)"), ETR("1-sigma noise on the estimated robot yaw in degrees."), 0.0, 0.0, 90.0, 0.1));
    return s;
}

sdk::PropertySchema cameraPlacementSchema(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    sdk::PropertySchema s;
    addPlacementProperties(
        s, c, catalog, cam.attachment, ETR("A camera moves with the part it is attached to. The placement is kept when you change it."),
        ETR("Named frame of the parent the camera hangs on (e.g. a turret's moving part)."),
        ETR("Relative to the attachment link: x forward (optical axis), y left, z up."));
    return s;
}

Value cameraGeneralValues(const sdk::CameraConfig& cam) {
    Value o = Value::makeObject();
    o["label"] = cam.label;
    o["kind"] = std::string(sdk::cameraKindKey(cam.kind));
    o["name"] = cam.name;
    return o;
}

Value cameraOpticsValues(const sdk::CameraConfig& cam) {
    Value o = Value::makeObject();
    o["hFovDeg"] = cam.hFovDeg;
    o["vFovDeg"] = cam.vFovDeg;
    o["maxRangeM"] = cam.maxRangeM;
    o["maxIncidenceDeg"] = cam.maxIncidenceDeg;
    o["fps"] = cam.fps;
    o["latencyMs"] = cam.latencyMs;
    o["widthPx"] = static_cast<std::int64_t>(cam.widthPx);
    o["heightPx"] = static_cast<std::int64_t>(cam.heightPx);
    o["positionNoiseM"] = cam.positionNoiseM;
    o["yawNoiseDeg"] = cam.yawNoiseDeg;
    return o;
}

Value cameraPlacementValues(const sdk::CameraConfig& cam) {
    Value o = Value::makeObject();
    putAttachmentValues(o, cam.attachment);
    o["local"] = sdk::toValue(cam.local);
    return o;
}

bool applyCamera(sdk::RobotConfig& c, const EditorCatalog& catalog, sdk::CameraConfig& cam, const std::string& groupId,
                 const std::string& key, const Value& value) {
    if (groupId == "camera.general") {
        if (key == "label") { cam.label = value.asString(cam.label); return true; }
        if (key == "name") { cam.name = value.asString(cam.name); return true; }
        if (key == "kind") {
            const sdk::CameraKind kind = value.asString() == sdk::cameraKindKey(sdk::CameraKind::PhotonVision)
                                             ? sdk::CameraKind::PhotonVision : sdk::CameraKind::Limelight;
            if (kind == cam.kind) return false;
            // Keep the default name of each type meaningful: Limelight's is empty, PhotonVision's "photon".
            const std::string defaultFrom = cam.kind == sdk::CameraKind::PhotonVision ? "photon" : "";
            cam.kind = kind;
            if (cam.name == defaultFrom) cam.name = kind == sdk::CameraKind::PhotonVision ? "photon" : "";
            cam.name = ops::uniqueCameraName(c, kind, cam.name, cam.id);
            return true;
        }
        return false;
    }
    if (groupId == "camera.optics") {
        if (key == "hFovDeg") { cam.hFovDeg = value.asDouble(cam.hFovDeg); return true; }
        if (key == "vFovDeg") { cam.vFovDeg = value.asDouble(cam.vFovDeg); return true; }
        if (key == "maxRangeM") { cam.maxRangeM = value.asDouble(cam.maxRangeM); return true; }
        if (key == "maxIncidenceDeg") { cam.maxIncidenceDeg = value.asDouble(cam.maxIncidenceDeg); return true; }
        if (key == "fps") { cam.fps = value.asDouble(cam.fps); return true; }
        if (key == "latencyMs") { cam.latencyMs = value.asDouble(cam.latencyMs); return true; }
        if (key == "widthPx") { cam.widthPx = static_cast<int>(value.asInt(cam.widthPx)); return true; }
        if (key == "heightPx") { cam.heightPx = static_cast<int>(value.asInt(cam.heightPx)); return true; }
        if (key == "positionNoiseM") { cam.positionNoiseM = value.asDouble(cam.positionNoiseM); return true; }
        if (key == "yawNoiseDeg") { cam.yawNoiseDeg = value.asDouble(cam.yawNoiseDeg); return true; }
        return false;
    }
    if (groupId == "camera.placement") {
        if (key == "local") return sdk::fromValue(value, cam.local);
        if (key == "parent") return ops::reattachCamera(c, catalog, cam.id, value.asString(sdk::kChassisParent), sdk::kRootLink);
        if (key == "parentLink")
            return ops::reattachCamera(c, catalog, cam.id, cam.attachment.parentMechanismId, value.asString(sdk::kRootLink));
    }
    return false;
}

bool applySensor(sdk::SensorConfig& s, const std::string& groupId, const std::string& key, const Value& value) {
    if (groupId == "sensor.general") {
        if (key == "label") { s.label = value.asString(s.label); return true; }
        return false;
    }
    if (groupId == "sensor.source") {
        if (key != "source") return false;
        s.source = value.asString();
        return true;
    }
    if (groupId == "sensor.props") {
        if (!s.properties.isObject()) s.properties = Value::makeObject();
        s.properties[key] = value;
        return true;
    }
    return false;
}

sdk::PropertySchema sensorSourceSchema(const EditorCatalog& catalog, const sdk::SensorConfig& s) {
    sdk::PropertySchema schema;
    PropertySpec source;
    source.key = "source";
    source.label = ETR("Game value").toStdString();
    source.description = ETR("The value the game publishes that drives this sensor.").toStdString();
    source.type = PropertyType::Enum;
    source.defaultValue = std::string();
    source.options.push_back({"", ETR("None").toStdString()});
    for (const auto& v : catalog.gameValues) {
        QString label = QString::fromStdString(v.label.empty() ? v.id : v.label);
        const QString unit = unitSuffix(v.unit, UnitSystem::Metric).trimmed();
        if (!unit.isEmpty()) label += QStringLiteral(" (%1)").arg(unit);
        source.options.push_back({v.id, label.toStdString()});
    }
    if (!s.source.empty() && catalog.findGameValue(s.source) == nullptr)
        source.options.push_back({s.source, ETR("(unavailable) %1").arg(QString::fromStdString(s.source)).toStdString()});
    schema.properties.push_back(std::move(source));
    return schema;
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
            if (c.findSensor(sel.owner) != nullptr) return SelectionRef::sensor(sel.owner);
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
        case SelKind::Zone: {
            const sdk::ZoneConfig* z = c.findZone(sel.owner);
            if (z == nullptr) break;
            const sdk::ZoneKindInfo* info = catalog.findZoneKind(z->kind);
            Value general = Value::makeObject();
            general["label"] = z->label;
            out.push_back(groupOf("zone.general", ETR("General"), zoneGeneralSchema(), std::move(general)));
            const bool volume = info == nullptr || info->shape == sdk::ZoneShape::Volume;
            Value placement = Value::makeObject();
            putAttachmentValues(placement, z->attachment);
            placement["local"] = sdk::toValue(z->local);
            placement["sizeM"] = sdk::toValue(z->sizeM);
            out.push_back(groupOf("zone.placement", ETR("Placement"), zonePlacementSchema(c, catalog, *z, volume), std::move(placement)));
            out.back().section = sdk::PropertySection::Placement;
            if (info != nullptr && !info->properties.properties.empty())
                out.push_back(groupOf("zone.props", QString::fromStdString(info->displayName), info->properties, withDefaults(info->properties, z->properties)));
            break;
        }
        case SelKind::Camera: {
            const sdk::CameraConfig* cam = c.findCamera(sel.owner);
            if (cam == nullptr) break;
            PropertyGroup general = groupOf("camera.general", ETR("General"), cameraGeneralSchema(), cameraGeneralValues(*cam));
            general.hint = ETR("Limelight: empty = 'limelight', otherwise 'limelight-<name>'. PhotonVision: the PhotonCamera name.");
            out.push_back(std::move(general));
            out.push_back(groupOf("camera.optics", ETR("Optics"), cameraOpticsSchema(), cameraOpticsValues(*cam)));
            out.push_back(groupOf("camera.placement", ETR("Placement"), cameraPlacementSchema(c, catalog, *cam), cameraPlacementValues(*cam)));
            out.back().section = sdk::PropertySection::Placement;
            break;
        }
        case SelKind::Sensor: {
            const sdk::SensorConfig* s = c.findSensor(sel.owner);
            if (s == nullptr) break;
            Value general = Value::makeObject();
            general["label"] = s->label;
            out.push_back(groupOf("sensor.general", ETR("General"), zoneGeneralSchema(), std::move(general)));
            Value source = Value::makeObject();
            source["source"] = s->source;
            PropertyGroup src = groupOf("sensor.source", ETR("Source"), sensorSourceSchema(catalog, *s), std::move(source));
            if (const sdk::GameValueInfo* v = catalog.findGameValue(s->source)) {
                src.hint = QString::fromStdString(v->description);
            } else if (!s->source.empty()) {
                src.hint = ETR("The selected game plugin does not publish this value. The sensor reads as idle.");
            } else {
                src.hint = catalog.gameValues.empty() ? ETR("The selected game plugin publishes no values.")
                                                      : ETR("Choose the game value that drives this sensor.");
            }
            out.push_back(std::move(src));
            const sdk::PropertySchema response = sdk::sensorResponseSchema(s->kind);
            if (!response.properties.empty())
                out.push_back(groupOf("sensor.props", ETR("Response"), response, withDefaults(response, s->properties)));
            break;
        }
        case SelKind::StartPosition: {
            const sdk::StartPosition* p = c.findStartPosition(sel.owner);
            if (p == nullptr) break;
            Value general = Value::makeObject();
            general["label"] = p->label;
            general["alliance"] = std::string(sdk::allianceKey(p->alliance));
            out.push_back(groupOf("startpos.general", ETR("General"), startPositionGeneralSchema(), std::move(general)));
            Value placement = Value::makeObject();
            placement["xM"] = static_cast<double>(p->pose.xM);
            placement["yM"] = static_cast<double>(p->pose.yM);
            placement["headingRad"] = static_cast<double>(p->pose.headingRad);
            out.push_back(groupOf("startpos.placement", ETR("Placement"), startPositionPlacementSchema(), std::move(placement)));
            out.back().section = sdk::PropertySection::Placement;
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
    if (sel.kind == SelKind::Zone) {
        sdk::ZoneConfig* z = c.findZone(sel.owner);
        if (z == nullptr) return false;
        if (groupId == "zone.general") return key == "label" ? (z->label = value.asString(z->label), true) : false;
        if (groupId == "zone.placement") {
            if (key == "local") return sdk::fromValue(value, z->local);
            if (key == "sizeM") return sdk::fromValue(value, z->sizeM);
            if (key == "parent") return ops::reattachZone(c, catalog, z->id, value.asString(sdk::kChassisParent), sdk::kRootLink);
            if (key == "parentLink") return ops::reattachZone(c, catalog, z->id, z->attachment.parentMechanismId, value.asString(sdk::kRootLink));
            return false;
        }
        if (groupId == "zone.props") {
            if (!z->properties.isObject()) z->properties = Value::makeObject();
            z->properties[key] = value;
            return true;
        }
        return false;
    }
    if (sel.kind == SelKind::Camera) {
        sdk::CameraConfig* cam = c.findCamera(sel.owner);
        return cam != nullptr && applyCamera(c, catalog, *cam, groupId, key, value);
    }
    if (sel.kind == SelKind::Sensor) {
        sdk::SensorConfig* s = ops::findSensor(c, sel.owner);
        return s != nullptr && applySensor(*s, groupId, key, value);
    }
    if (sel.kind == SelKind::StartPosition) {
        sdk::StartPosition* p = ops::findStartPosition(c, sel.owner);
        return p != nullptr && applyStartPosition(*p, groupId, key, value);
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

QString robotToCameraText(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam, UnitSystem units,
                          QString* tooltip) {
    const sdk::Pose3 p = ops::cameraRobotPose(c, catalog, cam);
    const sdk::Euler e = p.rotation.toEuler();
    const UnitDisplay ud = unitDisplay(PropertyUnit::Meters, units);
    auto length = [&](float m) { return QString::number(static_cast<double>(m) * ud.factor, 'f', ud.decimals) + QString::fromUtf8(ud.suffix); };
    auto degrees = [](float rad) { return QString::number(static_cast<double>(sdk::radToDegF(rad)), 'f', 1) + QStringLiteral("\xC2\xB0"); };
    auto num = [](float v) { return QString::number(static_cast<double>(v), 'f', 4); };
    if (tooltip != nullptr)
        *tooltip = ETR("WPILib robot frame: origin on the floor below the hull centre, +X forward, +Y left, yaw counter-clockwise. "
                       "For a moving mechanism this is its rest pose.\nMetres / radians: x %1, y %2, z %3 / roll %4, pitch %5, yaw %6")
                       .arg(num(p.position.x), num(p.position.y), num(p.position.z), num(e.rollRad), num(e.pitchRad), num(e.yawRad));
    return ETR("x %1, y %2, z %3  |  roll %4, pitch %5, yaw %6")
        .arg(length(p.position.x), length(p.position.y), length(p.position.z), degrees(e.rollRad), degrees(e.pitchRad), degrees(e.yawRad));
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
        case SelKind::Zone:
            if (const auto* z = c.findZone(sel.owner)) return QString::fromStdString(ops::zoneDisplayName(*z, catalog));
            return QString::fromStdString(sel.owner);
        case SelKind::Camera:
            if (const auto* cam = c.findCamera(sel.owner)) return QString::fromStdString(ops::cameraDisplayName(*cam));
            return QString::fromStdString(sel.owner);
        case SelKind::Sensor:
            if (const auto* s = c.findSensor(sel.owner)) return QString::fromStdString(ops::sensorDisplayName(*s));
            return QString::fromStdString(sel.owner);
        case SelKind::StartPosition:
            if (const auto* p = c.findStartPosition(sel.owner)) return QString::fromStdString(ops::startPositionDisplayName(*p));
            return QString::fromStdString(sel.owner);
        case SelKind::Device: return QString::fromStdString(sel.owner);
        case SelKind::None: break;
    }
    return {};
}

}  // namespace wpiemu::app::editor
