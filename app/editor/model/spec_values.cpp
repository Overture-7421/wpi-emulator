#include "model/spec_values.h"

#include <algorithm>

#include "model/config_ops.h"
#include "wpiemu/sdk/config/value_codec.h"

namespace wpiemu::app::editor::spec {

using sdk::PropertySchema;
using sdk::PropertySpec;
using sdk::PropertyType;
using sdk::PropertyUnit;
using sdk::Value;

namespace {

PropertySpec makeSpec(std::string key, std::string label, PropertyType type, PropertyUnit unit, Value def,
                      std::string group, std::string description = {}) {
    PropertySpec s;
    s.key = std::move(key);
    s.label = std::move(label);
    s.type = type;
    s.unit = unit;
    s.defaultValue = std::move(def);
    s.group = std::move(group);
    s.description = std::move(description);
    return s;
}

PropertySpec& withRange(PropertySpec& s, double min, double max, double step = 0.0) {
    s.min = min;
    s.max = max;
    s.step = step;
    return s;
}

double num(const Value& v, double fallback) { return v.isNumber() ? v.asDouble() : fallback; }
float numF(const Value& v, float fallback) { return static_cast<float>(num(v, static_cast<double>(fallback))); }

sdk::Vec3 vec(const Value& v, const sdk::Vec3& fallback) {
    sdk::Vec3 out = fallback;
    sdk::fromValue(v, out);
    return out;
}

const char* wheelKindKey(sdk::WheelKind k) {
    switch (k) {
        case sdk::WheelKind::Standard: return "standard";
        case sdk::WheelKind::Omni: return "omni";
        case sdk::WheelKind::Mecanum: return "mecanum";
        case sdk::WheelKind::Swerve: return "swerve";
    }
    return "standard";
}

sdk::WheelKind wheelKindFrom(const std::string& s, sdk::WheelKind fallback) {
    if (s == "standard") return sdk::WheelKind::Standard;
    if (s == "omni") return sdk::WheelKind::Omni;
    if (s == "mecanum") return sdk::WheelKind::Mecanum;
    if (s == "swerve") return sdk::WheelKind::Swerve;
    return fallback;
}

void motorOptions(PropertySpec& spec, const sdk::MotorSpec& current) {
    bool found = false;
    for (const auto& m : motorPresets()) {
        spec.options.push_back({m.name, m.name});
        found = found || m.name == current.name;
    }
    if (!found) spec.options.push_back({current.name, current.name + " (custom)"});
}

}  // namespace

std::vector<sdk::MotorSpec> motorPresets() {
    return {sdk::motors::krakenX60(), sdk::motors::falcon500(), sdk::motors::neo(), sdk::motors::neo550(),
            sdk::motors::vortex(),    sdk::motors::cim(),       sdk::motors::miniCim()};
}

sdk::MotorSpec motorByName(const std::string& name, const sdk::MotorSpec& fallback) {
    for (const auto& m : motorPresets())
        if (m.name == name) return m;
    return fallback;
}

// ---- robot ------------------------------------------------------------------------------------
PropertySchema robotSchema() {
    PropertySchema s;
    s.properties.push_back(makeSpec("name", "Robot name", PropertyType::String, PropertyUnit::None, "Robot", "General"));
    auto volts = makeSpec("batteryNominalV", "Battery voltage", PropertyType::Double, PropertyUnit::Volts, 12.6, "Battery",
                          "Fully charged open-circuit voltage.");
    s.properties.push_back(withRange(volts, 6.0, 14.0, 0.1));
    auto res = makeSpec("batteryResistanceMilliOhm", "Internal resistance (m\xCE\xA9)", PropertyType::Double, PropertyUnit::None, 15.0,
                        "Battery", "Higher resistance = deeper voltage sag under load.");
    s.properties.push_back(withRange(res, 1.0, 100.0, 1.0));
    auto brown = makeSpec("brownoutV", "Brownout voltage", PropertyType::Double, PropertyUnit::Volts, 6.8, "Battery");
    s.properties.push_back(withRange(brown, 4.0, 10.0, 0.1));
    return s;
}

Value robotValues(const sdk::RobotConfig& c) {
    Value o = Value::makeObject();
    o["name"] = c.name;
    o["batteryNominalV"] = c.battery.nominalVoltageV;
    o["batteryResistanceMilliOhm"] = c.battery.internalResistanceOhm * 1000.0;
    o["brownoutV"] = c.battery.brownoutVoltageV;
    return o;
}

void applyRobot(sdk::RobotConfig& c, const std::string& key, const Value& v) {
    if (key == "name") c.name = v.asString(c.name);
    else if (key == "batteryNominalV") c.battery.nominalVoltageV = num(v, c.battery.nominalVoltageV);
    else if (key == "batteryResistanceMilliOhm") c.battery.internalResistanceOhm = num(v, 15.0) / 1000.0;
    else if (key == "brownoutV") c.battery.brownoutVoltageV = num(v, c.battery.brownoutVoltageV);
}

// ---- chassis ----------------------------------------------------------------------------------
PropertySchema chassisSchema() {
    PropertySchema s;
    const sdk::ChassisSpec d;
    auto size = makeSpec("size", "Frame size (L \xC3\x97 W \xC3\x97 H)", PropertyType::Vec3, PropertyUnit::Meters, sdk::toValue(d.sizeM),
                         "Frame", "Bounding box of the frame perimeter: x forward, y left, z up.");
    s.properties.push_back(size);
    auto clr = makeSpec("groundClearance", "Ground clearance", PropertyType::Double, PropertyUnit::Meters, d.groundClearanceM, "Frame",
                        "Bottom of the frame above the floor.");
    s.properties.push_back(withRange(clr, 0.0, 0.5, 0.005));
    auto mass = makeSpec("massKg", "Mass (with battery & bumpers)", PropertyType::Double, PropertyUnit::Kilograms, d.massKg, "Mass");
    mass.required = true;
    s.properties.push_back(withRange(mass, 5.0, 200.0, 0.5));
    auto comx = makeSpec("comOffsetX", "COM forward offset", PropertyType::Double, PropertyUnit::Meters, 0.0, "Mass",
                         "Centre of mass relative to the hull centre. Moving it keeps the robot looking the same.");
    s.properties.push_back(withRange(comx, -1.0, 1.0, 0.005));
    auto comy = makeSpec("comOffsetY", "COM left offset", PropertyType::Double, PropertyUnit::Meters, 0.0, "Mass");
    s.properties.push_back(withRange(comy, -1.0, 1.0, 0.005));
    auto comz = makeSpec("comHeight", "COM height", PropertyType::Double, PropertyUnit::Meters, d.comHeightM, "Mass",
                         "Above the floor; drives load transfer under acceleration.");
    s.properties.push_back(withRange(comz, 0.02, 1.5, 0.005));
    auto mu = makeSpec("friction", "Hull friction", PropertyType::Double, PropertyUnit::None, d.material.friction, "Contact",
                       "Friction coefficient of bumpers/frame against walls and fuel.");
    s.properties.push_back(withRange(mu, 0.0, 2.0, 0.05));
    auto rest = makeSpec("restitution", "Hull bounciness", PropertyType::Double, PropertyUnit::None, d.material.restitution, "Contact");
    s.properties.push_back(withRange(rest, 0.0, 1.0, 0.05));
    auto inertia = makeSpec("inertia", "Inertia override (0 = from box)", PropertyType::Vec3, PropertyUnit::KgM2,
                            sdk::toValue(sdk::Vec3{}), "Contact");
    s.properties.push_back(inertia);
    return s;
}

Value chassisValues(const sdk::ChassisSpec& s) {
    Value o = Value::makeObject();
    o["size"] = sdk::toValue(s.sizeM);
    o["groundClearance"] = s.groundClearanceM;
    o["massKg"] = s.massKg;
    o["comOffsetX"] = s.comOffsetM.x;
    o["comOffsetY"] = s.comOffsetM.y;
    o["comHeight"] = s.comHeightM;
    o["friction"] = s.material.friction;
    o["restitution"] = s.material.restitution;
    o["inertia"] = sdk::toValue(s.inertiaDiagKgM2);
    return o;
}

void applyChassis(sdk::RobotConfig& c, const EditorCatalog& catalog, const std::string& key, const Value& v) {
    sdk::ChassisSpec& s = c.chassis;
    if (key == "size") s.sizeM = vec(v, s.sizeM);
    else if (key == "groundClearance") s.groundClearanceM = numF(v, s.groundClearanceM);
    else if (key == "massKg") s.massKg = numF(v, s.massKg);
    else if (key == "comOffsetX") ops::setCom(c, numF(v, s.comOffsetM.x), s.comOffsetM.y, s.comHeightM, true, &catalog);
    else if (key == "comOffsetY") ops::setCom(c, s.comOffsetM.x, numF(v, s.comOffsetM.y), s.comHeightM, true, &catalog);
    else if (key == "comHeight") ops::setCom(c, s.comOffsetM.x, s.comOffsetM.y, numF(v, s.comHeightM), true, &catalog);
    else if (key == "friction") s.material.friction = numF(v, s.material.friction);
    else if (key == "restitution") s.material.restitution = numF(v, s.material.restitution);
    else if (key == "inertia") s.inertiaDiagKgM2 = vec(v, s.inertiaDiagKgM2);
}

// ---- drivetrain -------------------------------------------------------------------------------
PropertySchema drivetrainSchema(const EditorCatalog& catalog) {
    PropertySchema s;
    auto kind = makeSpec("kind", "Drivetrain type", PropertyType::Enum, PropertyUnit::None, sdk::drivetrain_kind::kSwerve, "Drivetrain",
                         "Changing the type updates every wheel (swerve modules steer, tank wheels do not).");
    for (const auto& d : catalog.drivetrains) kind.options.push_back({d.kind, d.info.displayName.empty() ? d.kind : d.info.displayName});
    if (kind.options.empty()) {
        kind.options.push_back({sdk::drivetrain_kind::kSwerve, "Swerve"});
        kind.options.push_back({sdk::drivetrain_kind::kTank, "Tank"});
        kind.options.push_back({sdk::drivetrain_kind::kMecanum, "Mecanum"});
    }
    s.properties.push_back(kind);
    auto tire = makeSpec("tireModel", "Tire model", PropertyType::String, PropertyUnit::None, "brush", "Traction");
    s.properties.push_back(tire);
    auto tau = makeSpec("loadTransferTauS", "Load-transfer filter", PropertyType::Double, PropertyUnit::Seconds, 0.02, "Traction",
                        "Low-pass time constant on chassis acceleration for quasi-static load transfer.");
    s.properties.push_back(withRange(tau, 0.0, 0.5, 0.005));
    return s;
}

Value drivetrainValues(const sdk::DrivetrainSpec& s) {
    Value o = Value::makeObject();
    o["kind"] = s.kind;
    o["tireModel"] = s.tireModel;
    o["loadTransferTauS"] = s.loadTransferTauS;
    return o;
}

void applyDrivetrain(sdk::RobotConfig& c, const std::string& key, const Value& v) {
    if (key == "kind") ops::setDrivetrainKind(c, v.asString(c.drivetrain.spec.kind));
    else if (key == "tireModel") c.drivetrain.spec.tireModel = v.asString(c.drivetrain.spec.tireModel);
    else if (key == "loadTransferTauS") c.drivetrain.spec.loadTransferTauS = numF(v, c.drivetrain.spec.loadTransferTauS);
}

// ---- wheel ------------------------------------------------------------------------------------
PropertySchema wheelSchema(const sdk::WheelSpec& wheel) {
    PropertySchema s;
    const sdk::WheelSpec d;
    s.properties.push_back(makeSpec("name", "Name", PropertyType::String, PropertyUnit::None, "wheel", "Wheel",
                                    "Unique within the drivetrain; used in the slot names (<name>.drive)."));
    auto kind = makeSpec("kind", "Type", PropertyType::Enum, PropertyUnit::None, "standard", "Wheel");
    kind.options = {{"standard", "Standard"}, {"omni", "Omni"}, {"mecanum", "Mecanum"}, {"swerve", "Swerve module"}};
    s.properties.push_back(kind);
    s.properties.push_back(makeSpec("position", "Hub position (z = height above floor)", PropertyType::Vec3, PropertyUnit::Meters,
                                    sdk::toValue(sdk::Vec3{}), "Wheel"));
    auto radius = makeSpec("radius", "Radius", PropertyType::Double, PropertyUnit::Meters, d.radiusM, "Wheel");
    s.properties.push_back(withRange(radius, 0.01, 0.2, 0.001));
    auto width = makeSpec("width", "Width", PropertyType::Double, PropertyUnit::Meters, d.widthM, "Wheel");
    s.properties.push_back(withRange(width, 0.005, 0.2, 0.001));
    auto mu = makeSpec("mu", "Friction coefficient", PropertyType::Double, PropertyUnit::None, d.tire.mu, "Traction",
                       "Peak tire/carpet friction (typical FRC: 0.8 \xE2\x80\x93 1.4).");
    s.properties.push_back(withRange(mu, 0.1, 3.0, 0.05));
    auto lat = makeSpec("lateralScale", "Lateral grip scale", PropertyType::Double, PropertyUnit::None, d.tire.lateralScale, "Traction",
                        "1 = full grip sideways; omni/mecanum rollers are ~0.12\xE2\x80\x93" "0.15.");
    s.properties.push_back(withRange(lat, 0.0, 1.0, 0.01));

    auto drive = makeSpec("driveMotor", "Motor model", PropertyType::Enum, PropertyUnit::None, d.driveMotor.name, "Drive");
    motorOptions(drive, wheel.driveMotor);
    s.properties.push_back(drive);
    auto count = makeSpec("driveMotorCount", "Motor count", PropertyType::Int, PropertyUnit::Count, 1, "Drive");
    s.properties.push_back(withRange(count, 1, sdk::kMaxMotorsPerAxis, 1));
    auto ratio = makeSpec("driveGearRatio", "Gear ratio (motor : output)", PropertyType::Double, PropertyUnit::Ratio, d.driveGearRatio, "Drive");
    s.properties.push_back(withRange(ratio, 1.0, 40.0, 0.05));
    auto eff = makeSpec("driveEfficiency", "Efficiency", PropertyType::Double, PropertyUnit::None, d.driveEfficiency, "Drive");
    s.properties.push_back(withRange(eff, 0.3, 1.0, 0.01));
    auto lim = makeSpec("driveCurrentLimit", "Current limit (0 = none)", PropertyType::Double, PropertyUnit::Amps, d.driveCurrentLimitA, "Drive");
    s.properties.push_back(withRange(lim, 0.0, 300.0, 5.0));

    // A swerve module is a steered wheel: its steering belongs to what the wheel is, not to how it is driven.
    auto steer = makeSpec("steerMotor", "Steer motor model", PropertyType::Enum, PropertyUnit::None, d.steerMotor.name, "Steering");
    motorOptions(steer, wheel.steerMotor);
    steer.visibleWhenKey = "kind";
    steer.visibleWhenEquals = std::string("swerve");
    s.properties.push_back(steer);
    auto sratio = makeSpec("steerGearRatio", "Steer gear ratio", PropertyType::Double, PropertyUnit::Ratio, d.steerGearRatio, "Steering");
    sratio.visibleWhenKey = "kind";
    sratio.visibleWhenEquals = std::string("swerve");
    s.properties.push_back(withRange(sratio, 1.0, 100.0, 0.1));
    for (auto& p : s.properties) {
        if (p.group == "Drive") p.section = sdk::PropertySection::Drive;
        else if (p.key == "position") {
            p.section = sdk::PropertySection::Placement;
            p.group = "Placement";
        }
    }
    return s;
}

Value wheelValues(const sdk::WheelSpec& w) {
    Value o = Value::makeObject();
    o["name"] = w.name;
    o["kind"] = wheelKindKey(w.kind);
    o["position"] = sdk::toValue(w.positionM);
    o["radius"] = w.radiusM;
    o["width"] = w.widthM;
    o["mu"] = w.tire.mu;
    o["lateralScale"] = w.tire.lateralScale;
    o["driveMotor"] = w.driveMotor.name;
    o["driveMotorCount"] = w.driveMotorCount;
    o["driveGearRatio"] = w.driveGearRatio;
    o["driveEfficiency"] = w.driveEfficiency;
    o["driveCurrentLimit"] = w.driveCurrentLimitA;
    o["steerMotor"] = w.steerMotor.name;
    o["steerGearRatio"] = w.steerGearRatio;
    return o;
}

void applyWheel(sdk::RobotConfig& c, const std::string& wheelName, const std::string& key, const Value& v) {
    if (key == "name") {
        ops::renameWheel(c, wheelName, v.asString(wheelName));
        return;
    }
    sdk::WheelSpec* w = ops::findWheel(c, wheelName);
    if (w == nullptr) return;
    if (key == "kind") {
        w->kind = wheelKindFrom(v.asString(), w->kind);
        w->steered = w->kind == sdk::WheelKind::Swerve;
        if (!w->steered) {
            auto& maps = c.drivetrain.mappings;
            maps.erase(std::remove_if(maps.begin(), maps.end(), [&](const sdk::SlotMapping& m) { return m.slot.slot == w->name + ".steer"; }), maps.end());
        }
    }
    else if (key == "position") w->positionM = vec(v, w->positionM);
    else if (key == "radius") w->radiusM = numF(v, w->radiusM);
    else if (key == "width") w->widthM = numF(v, w->widthM);
    else if (key == "mu") w->tire.mu = numF(v, w->tire.mu);
    else if (key == "lateralScale") w->tire.lateralScale = numF(v, w->tire.lateralScale);
    else if (key == "driveMotor") w->driveMotor = motorByName(v.asString(), w->driveMotor);
    else if (key == "driveMotorCount") {
        w->driveMotorCount = std::clamp(static_cast<int>(v.asInt(w->driveMotorCount)), 1, sdk::kMaxMotorsPerAxis);
        auto& maps = c.drivetrain.mappings;
        maps.erase(std::remove_if(maps.begin(), maps.end(),
                                  [&](const sdk::SlotMapping& m) {
                                      for (int i = 0; i < w->driveMotorCount; ++i)
                                          if (m.slot.slot == sdk::driveSlotName(w->name, i)) return false;
                                      return ops::wheelOfSlot(m.slot.slot) == w->name && m.slot.slot != w->name + ".steer";
                                  }),
                   maps.end());
    }
    else if (key == "driveGearRatio") w->driveGearRatio = numF(v, w->driveGearRatio);
    else if (key == "driveEfficiency") w->driveEfficiency = numF(v, w->driveEfficiency);
    else if (key == "driveCurrentLimit") w->driveCurrentLimitA = numF(v, w->driveCurrentLimitA);
    else if (key == "steerMotor") w->steerMotor = motorByName(v.asString(), w->steerMotor);
    else if (key == "steerGearRatio") w->steerGearRatio = numF(v, w->steerGearRatio);
}

// ---- primitive --------------------------------------------------------------------------------
PropertySchema primitiveSchema() {
    PropertySchema s;
    const sdk::PrimitiveSpec d;
    s.properties.push_back(makeSpec("name", "Name", PropertyType::String, PropertyUnit::None, "box1", "Shape"));
    auto kind = makeSpec("kind", "Shape", PropertyType::Enum, PropertyUnit::None, "box", "Shape");
    kind.options = {{"box", "Box"}, {"cylinder", "Cylinder"}};
    s.properties.push_back(kind);
    s.properties.push_back(makeSpec("pose", "Pose", PropertyType::Pose3, PropertyUnit::Meters, sdk::toValue(sdk::Pose3{}), "Shape"));
    auto size = makeSpec("size", "Size (x \xC3\x97 y \xC3\x97 z)", PropertyType::Vec3, PropertyUnit::Meters, sdk::toValue(d.sizeM), "Shape");
    size.visibleWhenKey = "kind";
    size.visibleWhenEquals = "box";
    s.properties.push_back(size);
    auto radius = makeSpec("radius", "Radius", PropertyType::Double, PropertyUnit::Meters, d.radiusM, "Shape");
    radius.visibleWhenKey = "kind";
    radius.visibleWhenEquals = "cylinder";
    s.properties.push_back(withRange(radius, 0.001, 2.0, 0.005));
    auto height = makeSpec("height", "Height (along local Z)", PropertyType::Double, PropertyUnit::Meters, d.heightM, "Shape");
    height.visibleWhenKey = "kind";
    height.visibleWhenEquals = "cylinder";
    s.properties.push_back(withRange(height, 0.001, 2.0, 0.005));
    s.properties.push_back(makeSpec("color", "Colour", PropertyType::Color, PropertyUnit::None, "#808080", "Appearance"));
    s.properties.push_back(makeSpec("collides", "Collides with the world", PropertyType::Bool, PropertyUnit::None, false, "Appearance",
                                    "Adds a collision shape to the owning body (bumpers, frame rails)."));
    auto mass = makeSpec("massKg", "Mass (informational)", PropertyType::Double, PropertyUnit::Kilograms, 0.0, "Appearance",
                         "Shown in the mass budget; physics uses the owner's total mass.");
    s.properties.push_back(withRange(mass, 0.0, 100.0, 0.1));
    return s;
}

Value primitiveValues(const sdk::PrimitiveSpec& p) {
    Value o = Value::makeObject();
    o["name"] = p.name;
    o["kind"] = p.kind == sdk::PrimitiveKind::Box ? "box" : "cylinder";
    o["pose"] = sdk::toValue(p.local);
    o["size"] = sdk::toValue(p.sizeM);
    o["radius"] = p.radiusM;
    o["height"] = p.heightM;
    o["color"] = p.color;
    o["collides"] = p.collides;
    o["massKg"] = p.massKg;
    return o;
}

void applyPrimitive(sdk::PrimitiveSpec& p, const std::string& key, const Value& v) {
    if (key == "name") p.name = v.asString(p.name);
    else if (key == "kind") p.kind = v.asString() == "cylinder" ? sdk::PrimitiveKind::Cylinder : sdk::PrimitiveKind::Box;
    else if (key == "pose") sdk::fromValue(v, p.local);
    else if (key == "size") p.sizeM = vec(v, p.sizeM);
    else if (key == "radius") p.radiusM = numF(v, p.radiusM);
    else if (key == "height") p.heightM = numF(v, p.heightM);
    else if (key == "color") p.color = v.asString(p.color);
    else if (key == "collides") p.collides = v.asBool(p.collides);
    else if (key == "massKg") p.massKg = numF(v, p.massKg);
}

}  // namespace wpiemu::app::editor::spec
