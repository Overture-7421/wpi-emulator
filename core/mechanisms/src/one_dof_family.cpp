#include <cmath>

#include "template_support.h"

namespace wpiemu::core::mechanisms::detail {

namespace {

constexpr const char* kMotionGroup = "Motion";
constexpr const char* kBeamGroup = "Beam break";
constexpr double kMovingSpeed = 0.01;   // rad/s or m/s above which the mechanism reports `active`

struct BeamBreakWindow {
    bool enabled = false;
    WindowBeamBreakModel model;
};

struct OneDofFlavor {
    const char* type;
    const char* displayName;
    const char* category;
    const char* description;
    const char* linkName;
    AxisOptions axis;
    AxisName moveAxis;
    bool axisSelectable;
    double angleSign;       // rotation direction about moveAxis for a positive angle
    bool beamBreakWindow;   // generic 1-DOF only
    sdk::PrimitiveSpec geometry;
};

class OneDofMechanism final : public AxisMechanismBase {
public:
    OneDofMechanism(const sdk::MechanismConfig& config, std::unique_ptr<MechanismAxis> axis, std::string linkName,
                    AxisName moveAxis, double angleSign, BeamBreakWindow window)
        : AxisMechanismBase(config), link_(std::move(linkName)), axisName_(moveAxis), sign_(angleSign), window_(window) {
        addAxis(std::move(axis));
        publishNow();
    }

protected:
    // Writes the window beam break (when the template is configured for it) after the axis update.
    void onTick(sdk::SimContext& ctx, double /*dt*/) override {
        if (!window_.enabled) return;
        ctx.devices().writeDigital({id(), "beamBreak"}, window_.model.reading(axis(0).position()));
    }

    // `active`: moving faster than 0.01 rad/s | m/s. Values: position [rad|m], velocity, voltage [V],
    // current [A], atLowerLimit / atUpperLimit (0|1).
    void publish(sdk::MechanismState& out) const override {
        const MechanismAxis& a = axis(0);
        const bool linear = a.settings().motion == Motion::Linear;
        const char* posUnit = linear ? "m" : "rad";
        const char* velUnit = linear ? "m/s" : "rad/s";
        out.active = std::abs(a.velocity()) > kMovingSpeed;
        const double p = a.position();
        out.links.push_back({link_, linear ? translationAlong(axisName_, p)
                                           : rotationAbout(axisName_, sign_ * (a.settings().limited ? p : wrapAngle(p)))});
        out.values.push_back(namedValue("position", p, posUnit));
        out.values.push_back(namedValue("velocity", a.velocity(), velUnit));
        out.values.push_back(namedValue("voltage", a.appliedVoltageV(), "V"));
        out.values.push_back(namedValue("current", a.motorCurrentA(), "A"));
        out.values.push_back(namedValue("atLowerLimit", a.atLowerLimit() ? 1.0 : 0.0, ""));
        out.values.push_back(namedValue("atUpperLimit", a.atUpperLimit() ? 1.0 : 0.0, ""));
    }

private:
    std::string link_;
    AxisName axisName_;
    double sign_;
    BeamBreakWindow window_;
};

AxisOptions limitedAxis(Motion motion, const char* rotaryModel) {
    AxisOptions o;
    o.motion = motion;
    o.rotaryModel = rotaryModel;
    o.limits = true;
    o.limitSwitchSlots = true;
    o.motorModel = "kraken_x60";
    o.brake = true;
    return o;
}

sdk::PrimitiveSpec primitive(const char* name, sdk::PrimitiveKind kind, sdk::Vec3 size, sdk::Vec3 at, const char* color) {
    sdk::PrimitiveSpec p;
    p.name = name;
    p.kind = kind;
    p.sizeM = size;
    p.radiusM = size.x * 0.5f;
    p.heightM = size.z;
    p.local = sdk::Pose3{at, sdk::Quat{}};
    p.color = color;
    return p;
}

OneDofFlavor genericFlavor() {
    AxisOptions o = limitedAxis(Motion::Rotary, plant_model::kDcMotor);
    o.selectableMotion = true;
    o.gearRatio = 10.0;
    o.inertiaKgM2 = 0.01;
    return {mechanism_type::kGeneric1Dof, "Generic 1-DOF", "Generic",
            "One motor-driven degree of freedom: rotary, arm (gravity) or linear (elevator), with optional stops, "
            "limit switches and a position-window beam break.",
            "mover", o, AxisName::Z, true, 1.0, true,
            primitive("body", sdk::PrimitiveKind::Box, {0.10f, 0.10f, 0.10f}, {0.f, 0.f, 0.f}, "#9b59b6")};
}

OneDofFlavor pivotFlavor() {
    AxisOptions o = limitedAxis(Motion::Arm, plant_model::kDcMotor);
    o.gearRatio = 60.0;
    o.inertiaKgM2 = 0.0;   // uniform rod
    o.massKg = 2.0;
    o.lengthM = 0.3;
    o.gravity = true;
    o.minPosition = 0.0;
    o.maxPosition = 1.5;
    return {mechanism_type::kPivot, "Hood / pivot", "Arm",
            "A pivoting arm, hood or wrist. Angle in radians from horizontal, positive raising the +X end.", "arm", o,
            AxisName::Y, false, -1.0, false,
            primitive("arm", sdk::PrimitiveKind::Box, {0.30f, 0.04f, 0.04f}, {0.15f, 0.f, 0.f}, "#f0ad4e")};
}

OneDofFlavor turretFlavor() {
    AxisOptions o = limitedAxis(Motion::Rotary, plant_model::kTurret);
    o.gearRatio = 40.0;
    o.inertiaKgM2 = 0.05;
    o.minPosition = -3.3;
    o.maxPosition = 3.3;
    return {mechanism_type::kTurret, "Turret", "Turret",
            "A yaw turret about the mechanism Z axis. Set both stops equal for a continuous turret.", "turret", o,
            AxisName::Z, false, 1.0, false,
            primitive("turret", sdk::PrimitiveKind::Cylinder, {0.20f, 0.20f, 0.05f}, {0.f, 0.f, 0.f}, "#5bc0de")};
}

OneDofFlavor elevatorFlavor() {
    AxisOptions o = limitedAxis(Motion::Linear, plant_model::kDcMotor);
    o.motorCount = 2;
    o.gearRatio = 8.0;
    o.massKg = 6.0;
    o.drumRadiusM = 0.0225;
    o.gravity = true;
    o.minPosition = 0.0;
    o.maxPosition = 0.8;
    return {mechanism_type::kElevator, "Elevator / climber", "Elevator",
            "A vertical (or any-axis) linear stage / winch with gravity and hard stops.", "carriage", o, AxisName::Z,
            true, 1.0, false,
            primitive("carriage", sdk::PrimitiveKind::Box, {0.10f, 0.10f, 0.10f}, {0.f, 0.f, 0.f}, "#e67e22")};
}

sdk::MechanismTemplateInfo infoFor(const OneDofFlavor& f, const MotorCatalog& motors) {
    sdk::MechanismTemplateInfo info;
    info.type = f.type;
    info.displayName = f.displayName;
    info.category = f.category;
    info.description = f.description;
    info.slotSpecs = axisSlots(f.axis);
    info.properties.properties = axisProperties(f.axis, motors);
    if (f.axisSelectable)
        info.properties.properties.push_back(
            axisEnumProp("axis", f.axis.motion == Motion::Linear ? "Travel axis" : "Motion axis", f.moveAxis, kMotionGroup));
    if (f.beamBreakWindow) {
        info.slotSpecs.push_back({"beamBreak", "Beam break", sdk::SlotType::BeamBreak, false,
                                  "Driven from the position window (source 'window') or by game logic ('external')."});
        auto& p = info.properties.properties;
        p.push_back(enumProp("beamBreakSource", "Beam break source",
                             "'window': blocked while the position is inside the window; 'external': the game logic writes the slot.",
                             "external", {{"external", "External (game logic)"}, {"window", "Position window"}}, kBeamGroup));
        p.push_back(doubleProp("beamBreakMin", "Window minimum", "Lower edge of the blocking window (rad or m).",
                               sdk::PropertyUnit::None, 0.0, kBeamGroup, -1e4, 1e4));
        p.push_back(doubleProp("beamBreakMax", "Window maximum", "Upper edge of the blocking window (rad or m).",
                               sdk::PropertyUnit::None, 0.0, kBeamGroup, -1e4, 1e4));
        p.push_back(boolProp("beamBreakBlockedValue", "Reads true when blocked", "Off for normally-closed wiring.", true,
                             kBeamGroup, true));
    }
    info.defaultGeometry.push_back(f.geometry);
    info.links.push_back({f.linkName, std::string("Moving ") + f.linkName, sdk::Pose3{},
                          "Pose of the moving part at the start position (rest pose shown for position 0)."});
    return info;
}

sdk::Status registerFlavor(sdk::MechanismFactory& factory, const TemplateDeps& deps, const OneDofFlavor& flavor) {
    return registerValidated(
        factory, infoFor(flavor, *deps.motors),
        [flavor, deps](const sdk::MechanismConfig& config,
                       const sdk::Value& props) -> sdk::Result<std::unique_ptr<sdk::IMechanism>> {
            auto axis = buildAxis(config.id, flavor.axis, props, deps);
            if (!axis.ok()) return axis.error();
            AxisName moveAxis = flavor.moveAxis;
            if (flavor.axisSelectable && !parseAxis(getString(props, "axis", axisKey(flavor.moveAxis)), moveAxis))
                return sdk::makeError(sdk::ErrorCode::InvalidArgument, "property 'axis' has unknown value");
            BeamBreakWindow window;
            if (flavor.beamBreakWindow && getString(props, "beamBreakSource", "external") == "window") {
                window.enabled = true;
                window.model.minPosition = getDouble(props, "beamBreakMin", 0.0);
                window.model.maxPosition = getDouble(props, "beamBreakMax", 0.0);
                window.model.blockedValue = getBool(props, "beamBreakBlockedValue", true);
                if (window.model.maxPosition < window.model.minPosition)
                    return sdk::makeError(sdk::ErrorCode::InvalidArgument,
                                          "property 'beamBreakMax' must not be below 'beamBreakMin'");
            }
            return std::unique_ptr<sdk::IMechanism>(std::make_unique<OneDofMechanism>(
                config, std::move(axis).value(), flavor.linkName, moveAxis, flavor.angleSign, window));
        });
}

}  // namespace

sdk::Status registerOneDofFamily(sdk::MechanismFactory& factory, const TemplateDeps& deps) {
    for (const OneDofFlavor& f : {genericFlavor(), pivotFlavor(), turretFlavor(), elevatorFlavor()})
        if (const sdk::Status s = registerFlavor(factory, deps, f); !s.ok()) return s;
    return sdk::okStatus();
}

}  // namespace wpiemu::core::mechanisms::detail
