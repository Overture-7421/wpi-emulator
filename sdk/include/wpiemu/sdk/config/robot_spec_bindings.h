#pragma once

// Declarative PropertySchema + Value bindings for the core robot sections (robot, chassis, drivetrain,
// wheels, primitives, cameras and start positions). Each property is defined once here using
// sdk::ConfigBindings<T>, so the editor's schemas, value extraction and single-key apply all come from
// one table instead of parallel hand-written functions.

#include <algorithm>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/config_bindings.h"
#include "wpiemu/sdk/config/property_builders.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/physics/drivetrain.h"

namespace wpiemu::sdk {

// The motor preset whose `spec.name` is `name`, or `fallback` when custom/unknown.
inline MotorSpec motorByName(std::string_view name, const MotorSpec& fallback) {
    const MotorPreset* m = motors::findPresetByName(name);
    return m != nullptr ? m->spec : fallback;
}

namespace spec_bindings_detail {

inline void addMotorOptions(PropertySpec& spec, const MotorSpec& current) {
    for (const MotorPreset& m : motors::presets()) spec.options.push_back({m.spec.name, m.label});
    if (motors::findPresetByName(current.name) == nullptr)
        spec.options.push_back({current.name, current.name + " (custom)"});
}

inline float numF(const Value& v, float fallback) {
    return static_cast<float>(v.asDouble(static_cast<double>(fallback)));
}

}  // namespace spec_bindings_detail

// ---- robot (identity, battery, default start pose) --------------------------------------------
inline ConfigBindings<RobotConfig> robotEditorBindings() {
    using props::makeSpec;
    using props::withRange;
    using spec_bindings_detail::numF;
    ConfigBindings<RobotConfig> b;
    const RobotConfig defaults;
    const RobotStartPose start;

    b.add(makeSpec("name", "Robot name", PropertyType::String, PropertyUnit::None, defaults.name, "General"),
          &RobotConfig::name);
    auto volts = makeSpec("batteryNominalV", "Battery voltage", PropertyType::Double, PropertyUnit::Volts,
                          defaults.battery.nominalVoltageV, "Battery", "Fully charged open-circuit voltage.");
    b.add(withRange(volts, 6.0, 14.0, 0.1), &RobotConfig::battery, &BatteryConfig::nominalVoltageV);

    auto res = makeSpec("batteryResistanceMilliOhm", "Internal resistance (m\xCE\xA9)", PropertyType::Double,
                        PropertyUnit::None, defaults.battery.internalResistanceOhm * 1000.0, "Battery",
                        "Higher resistance = deeper voltage sag under load.");
    b.custom(
        withRange(res, 1.0, 100.0, 1.0),
        [](RobotConfig& c, const Value& v) {
            c.battery.internalResistanceOhm = v.asDouble(c.battery.internalResistanceOhm * 1000.0) / 1000.0;
        },
        [](const RobotConfig& c) { return Value(c.battery.internalResistanceOhm * 1000.0); });

    auto brown = makeSpec("brownoutV", "Brownout voltage", PropertyType::Double, PropertyUnit::Volts,
                          defaults.battery.brownoutVoltageV, "Battery");
    b.add(withRange(brown, 4.0, 10.0, 0.1), &RobotConfig::battery, &BatteryConfig::brownoutVoltageV);

    b.custom(
        makeSpec("customStartPose", "Use custom start pose", PropertyType::Bool, PropertyUnit::None, false,
                 "Start pose",
                 "Field coordinates restored on start and reset; robot odometry must use the same pose."),
        [](RobotConfig& c, const Value& v) {
            if (v.asBool())
                c.startPose = c.startPose.value_or(RobotStartPose{});
            else
                c.startPose.reset();
        },
        [](const RobotConfig& c) { return Value(c.startPose.has_value()); });

    b.custom(
        makeSpec("startX", "Start X", PropertyType::Double, PropertyUnit::Meters, start.xM, "Start pose"),
        [](RobotConfig& c, const Value& v) {
            auto s = c.startPose.value_or(RobotStartPose{});
            s.xM = numF(v, s.xM);
            c.startPose = s;
        },
        [](const RobotConfig& c) { return Value(c.startPose.value_or(RobotStartPose{}).xM); });

    b.custom(
        makeSpec("startY", "Start Y", PropertyType::Double, PropertyUnit::Meters, start.yM, "Start pose"),
        [](RobotConfig& c, const Value& v) {
            auto s = c.startPose.value_or(RobotStartPose{});
            s.yM = numF(v, s.yM);
            c.startPose = s;
        },
        [](const RobotConfig& c) { return Value(c.startPose.value_or(RobotStartPose{}).yM); });

    b.custom(
        makeSpec("startHeading", "Start heading", PropertyType::Double, PropertyUnit::Radians, start.headingRad,
                 "Start pose"),
        [](RobotConfig& c, const Value& v) {
            auto s = c.startPose.value_or(RobotStartPose{});
            s.headingRad = numF(v, s.headingRad);
            c.startPose = s;
        },
        [](const RobotConfig& c) { return Value(c.startPose.value_or(RobotStartPose{}).headingRad); });

    return b;
}

// ---- chassis (hull, mass, COM, material) ------------------------------------------------------
inline ConfigBindings<ChassisSpec> chassisEditorBindings() {
    using props::makeSpec;
    using props::withRange;
    ConfigBindings<ChassisSpec> b;
    const ChassisSpec d;

    b.add(makeSpec("size", "Frame size (L \xC3\x97 W \xC3\x97 H)", PropertyType::Vec3, PropertyUnit::Meters,
                   toValue(d.sizeM), "Frame", "Bounding box of the frame perimeter: x forward, y left, z up."),
          &ChassisSpec::sizeM);
    auto clr = makeSpec("groundClearance", "Ground clearance", PropertyType::Double, PropertyUnit::Meters,
                        d.groundClearanceM, "Frame", "Bottom of the frame above the floor.");
    b.add(withRange(clr, 0.0, 0.5, 0.005), &ChassisSpec::groundClearanceM);

    auto mass = makeSpec("massKg", "Mass (with battery & bumpers)", PropertyType::Double, PropertyUnit::Kilograms,
                         d.massKg, "Mass");
    mass.required = true;
    b.add(withRange(mass, 5.0, 200.0, 0.5), &ChassisSpec::massKg);

    auto comx = makeSpec("comOffsetX", "COM forward offset", PropertyType::Double, PropertyUnit::Meters, 0.0, "Mass",
                         "Centre of mass relative to the hull centre. Moving it keeps the robot looking the same.");
    b.add(withRange(comx, -1.0, 1.0, 0.005), &ChassisSpec::comOffsetM, &Vec3::x);

    auto comy = makeSpec("comOffsetY", "COM left offset", PropertyType::Double, PropertyUnit::Meters, 0.0, "Mass");
    b.add(withRange(comy, -1.0, 1.0, 0.005), &ChassisSpec::comOffsetM, &Vec3::y);

    auto comz = makeSpec("comHeight", "COM height", PropertyType::Double, PropertyUnit::Meters, d.comHeightM, "Mass",
                         "Above the floor; drives load transfer under acceleration.");
    b.add(withRange(comz, 0.02, 1.5, 0.005), &ChassisSpec::comHeightM);

    auto mu = makeSpec("friction", "Hull friction", PropertyType::Double, PropertyUnit::None, d.material.friction,
                       "Contact", "Friction coefficient of bumpers/frame against walls and fuel.");
    b.add(withRange(mu, 0.0, 2.0, 0.05), &ChassisSpec::material, &Material::friction);

    auto rest = makeSpec("restitution", "Hull bounciness", PropertyType::Double, PropertyUnit::None,
                         d.material.restitution, "Contact");
    b.add(withRange(rest, 0.0, 1.0, 0.05), &ChassisSpec::material, &Material::restitution);

    b.add(makeSpec("inertia", "Inertia override (0 = from box)", PropertyType::Vec3, PropertyUnit::KgM2,
                   toValue(Vec3{}), "Contact"),
          &ChassisSpec::inertiaDiagKgM2);
    return b;
}

// Maps a persisted/validator chassis field name ("sizeM", "comHeightM", ...) to its editor property key.
inline std::string chassisEditorFieldKey(std::string_view field) {
    if (field == "sizeM") return "size";
    if (field == "groundClearanceM") return "groundClearance";
    if (field == "comHeightM") return "comHeight";
    if (field == "comOffsetM") return "comOffsetX";
    if (field == "inertiaDiagKgM2") return "inertia";
    if (field == "material") return "friction";
    return std::string(field);
}

// ---- drivetrain -------------------------------------------------------------------------------
inline ConfigBindings<DrivetrainSpec> drivetrainEditorBindings(std::vector<EnumOption> kindOptions = {}) {
    using props::makeSpec;
    using props::withRange;
    ConfigBindings<DrivetrainSpec> b;
    const DrivetrainSpec defaults;

    auto kind = makeSpec("kind", "Drivetrain type", PropertyType::Enum, PropertyUnit::None, defaults.kind, "Drivetrain",
                         "Changing the type updates every wheel (swerve modules steer, tank wheels do not).");
    if (kindOptions.empty()) {
        for (const DrivetrainEntry& d : builtinDrivetrainInfos())
            kindOptions.push_back({d.kind, d.info.displayName.empty() ? d.kind : d.info.displayName});
    }
    kind.options = std::move(kindOptions);
    b.add(std::move(kind), &DrivetrainSpec::kind);

    b.add(makeSpec("tireModel", "Tire model", PropertyType::String, PropertyUnit::None, defaults.tireModel, "Traction"),
          &DrivetrainSpec::tireModel);

    auto tau = makeSpec("loadTransferTauS", "Load-transfer filter", PropertyType::Double, PropertyUnit::Seconds,
                        defaults.loadTransferTauS, "Traction",
                        "Low-pass time constant on chassis acceleration for quasi-static load transfer.");
    b.add(withRange(tau, 0.0, 0.5, 0.005), &DrivetrainSpec::loadTransferTauS);
    return b;
}

// ---- wheel ------------------------------------------------------------------------------------
inline ConfigBindings<WheelSpec> wheelEditorBindings(const WheelSpec& wheel = {}) {
    using props::makeSpec;
    using props::withRange;
    using spec_bindings_detail::addMotorOptions;
    ConfigBindings<WheelSpec> b;
    const WheelSpec d;

    b.add(makeSpec("name", "Name", PropertyType::String, PropertyUnit::None, "wheel", "Wheel",
                   "Unique within the drivetrain; used in the slot names (<name>.drive)."),
          &WheelSpec::name);

    auto kind = makeSpec("kind", "Type", PropertyType::Enum, PropertyUnit::None, wheelKindName(WheelKind::Standard),
                         "Wheel");
    kind.options = {{wheelKindName(WheelKind::Standard), "Standard"},
                    {wheelKindName(WheelKind::Omni), "Omni"},
                    {wheelKindName(WheelKind::Mecanum), "Mecanum"},
                    {wheelKindName(WheelKind::Swerve), "Swerve module"}};
    b.choice(std::move(kind), &WheelSpec::kind,
             {WheelKind::Standard, WheelKind::Omni, WheelKind::Mecanum, WheelKind::Swerve});

    auto pos = makeSpec("position", "Hub position (z = height above floor)", PropertyType::Vec3, PropertyUnit::Meters,
                        toValue(Vec3{}), "Placement");
    pos.section = PropertySection::Placement;
    b.add(std::move(pos), &WheelSpec::positionM);

    auto radius = makeSpec("radius", "Radius", PropertyType::Double, PropertyUnit::Meters, d.radiusM, "Wheel");
    b.add(withRange(radius, 0.01, 0.2, 0.001), &WheelSpec::radiusM);

    auto width = makeSpec("width", "Width", PropertyType::Double, PropertyUnit::Meters, d.widthM, "Wheel");
    b.add(withRange(width, 0.005, 0.2, 0.001), &WheelSpec::widthM);

    auto mu = makeSpec("mu", "Friction coefficient", PropertyType::Double, PropertyUnit::None, d.tire.mu, "Traction",
                       "Peak tire/carpet friction (typical FRC: 0.8 \xE2\x80\x93 1.4).");
    b.add(withRange(mu, 0.1, 3.0, 0.05), &WheelSpec::tire, &TireParams::mu);

    auto lat = makeSpec("lateralScale", "Lateral grip scale", PropertyType::Double, PropertyUnit::None,
                        d.tire.lateralScale, "Traction",
                        "1 = full grip sideways; omni/mecanum rollers are ~0.12\xE2\x80\x93"
                        "0.15.");
    b.add(withRange(lat, 0.0, 1.0, 0.01), &WheelSpec::tire, &TireParams::lateralScale);

    auto drive = makeSpec("driveMotor", "Motor model", PropertyType::Enum, PropertyUnit::None, d.driveMotor.name,
                          "Drive");
    drive.section = PropertySection::Drive;
    addMotorOptions(drive, wheel.driveMotor);
    b.custom(
        std::move(drive),
        [](WheelSpec& w, const Value& v) { w.driveMotor = motorByName(v.asString(), w.driveMotor); },
        [](const WheelSpec& w) { return Value(w.driveMotor.name); });

    auto count = makeSpec("driveMotorCount", "Motor count", PropertyType::Int, PropertyUnit::Count, 1, "Drive");
    count.section = PropertySection::Drive;
    b.add(withRange(count, 1, kMaxMotorsPerAxis, 1), &WheelSpec::driveMotorCount);

    auto ratio = makeSpec("driveGearRatio", "Gear ratio (motor : output)", PropertyType::Double, PropertyUnit::Ratio,
                          d.driveGearRatio, "Drive");
    ratio.section = PropertySection::Drive;
    b.add(withRange(ratio, 1.0, 40.0, 0.05), &WheelSpec::driveGearRatio);

    auto eff = makeSpec("driveEfficiency", "Efficiency", PropertyType::Double, PropertyUnit::None, d.driveEfficiency,
                        "Drive");
    eff.section = PropertySection::Drive;
    b.add(withRange(eff, 0.3, 1.0, 0.01), &WheelSpec::driveEfficiency);

    auto lim = makeSpec("driveCurrentLimit", "Current limit (0 = none)", PropertyType::Double, PropertyUnit::Amps,
                        d.driveCurrentLimitA, "Drive");
    lim.section = PropertySection::Drive;
    b.add(withRange(lim, 0.0, 300.0, 5.0), &WheelSpec::driveCurrentLimitA);

    auto steer = makeSpec("steerMotor", "Steer motor model", PropertyType::Enum, PropertyUnit::None, d.steerMotor.name,
                          "Steering");
    addMotorOptions(steer, wheel.steerMotor);
    steer.visibleWhenKey = "kind";
    steer.visibleWhenEquals = std::string("swerve");
    b.custom(
        std::move(steer),
        [](WheelSpec& w, const Value& v) { w.steerMotor = motorByName(v.asString(), w.steerMotor); },
        [](const WheelSpec& w) { return Value(w.steerMotor.name); });

    auto sratio = makeSpec("steerGearRatio", "Steer gear ratio", PropertyType::Double, PropertyUnit::Ratio,
                           d.steerGearRatio, "Steering");
    sratio.visibleWhenKey = "kind";
    sratio.visibleWhenEquals = std::string("swerve");
    b.add(withRange(sratio, 1.0, 100.0, 0.1), &WheelSpec::steerGearRatio);

    return b;
}

// Maps a persisted/validator wheel sub-path ("radiusM", "tire.mu", "driveMotor.stallTorqueNm", ...) to its editor key.
inline std::string wheelEditorFieldKey(std::string_view rest) {
    if (rest == "positionM") return "position";
    if (rest == "radiusM") return "radius";
    if (rest == "widthM") return "width";
    if (rest == "driveCurrentLimitA") return "driveCurrentLimit";
    if (rest == "tire.mu") return "mu";
    if (rest == "tire.lateralScale") return "lateralScale";
    if (rest.rfind("driveMotor.", 0) == 0) return "driveMotor";
    if (rest.rfind("steerMotor.", 0) == 0) return "steerMotor";
    if (rest.rfind("tire.", 0) == 0) return "mu";
    return std::string(rest);
}

// ---- primitive (box / cylinder) ---------------------------------------------------------------
inline ConfigBindings<PrimitiveSpec> primitiveEditorBindings() {
    using props::makeSpec;
    using props::withRange;
    ConfigBindings<PrimitiveSpec> b;
    const PrimitiveSpec d;

    b.add(makeSpec("name", "Name", PropertyType::String, PropertyUnit::None, "box1", "Shape"), &PrimitiveSpec::name);

    auto kind = makeSpec("kind", "Shape", PropertyType::Enum, PropertyUnit::None, "box", "Shape");
    kind.options = {{"box", "Box"}, {"cylinder", "Cylinder"}};
    b.choice(std::move(kind), &PrimitiveSpec::kind, {PrimitiveKind::Box, PrimitiveKind::Cylinder});

    b.add(makeSpec("pose", "Pose", PropertyType::Pose3, PropertyUnit::Meters, toValue(Pose3{}), "Shape"),
          &PrimitiveSpec::local);

    auto size = makeSpec("size", "Size (x \xC3\x97 y \xC3\x97 z)", PropertyType::Vec3, PropertyUnit::Meters,
                         toValue(d.sizeM), "Shape");
    size.visibleWhenKey = "kind";
    size.visibleWhenEquals = "box";
    b.add(std::move(size), &PrimitiveSpec::sizeM);

    auto radius = makeSpec("radius", "Radius", PropertyType::Double, PropertyUnit::Meters, d.radiusM, "Shape");
    radius.visibleWhenKey = "kind";
    radius.visibleWhenEquals = "cylinder";
    b.add(withRange(radius, 0.001, 2.0, 0.005), &PrimitiveSpec::radiusM);

    auto height = makeSpec("height", "Height (along local Z)", PropertyType::Double, PropertyUnit::Meters, d.heightM,
                           "Shape");
    height.visibleWhenKey = "kind";
    height.visibleWhenEquals = "cylinder";
    b.add(withRange(height, 0.001, 2.0, 0.005), &PrimitiveSpec::heightM);

    b.add(makeSpec("color", "Colour", PropertyType::Color, PropertyUnit::None, kDefaultPrimitiveColor, "Appearance"),
          &PrimitiveSpec::color);
    b.add(makeSpec("collides", "Collides with the world", PropertyType::Bool, PropertyUnit::None, false, "Appearance",
                   "Adds a collision shape to the owning body (bumpers, frame rails)."),
          &PrimitiveSpec::collides);

    auto mass = makeSpec("massKg", "Mass (informational)", PropertyType::Double, PropertyUnit::Kilograms, 0.0,
                         "Appearance", "Shown in the mass budget; physics uses the owner's total mass.");
    b.add(withRange(mass, 0.0, 100.0, 0.1), &PrimitiveSpec::massKg);

    return b;
}

}  // namespace wpiemu::sdk
