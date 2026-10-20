#include <cmath>

#include "template_support.h"

namespace wpiemu::core::mechanisms::detail {

namespace {

constexpr const char* kMotionGroup = "Motion";
constexpr double kMovingSpeed = 0.01;   // rad/s or m/s above which the mechanism reports `active`

// The only core mechanism: one motor-driven degree of freedom. Rotary (flywheel, roller, belt, turret),
// Arm (hood, pivot, wrist) and Linear (elevator, climber, slide) differ only in properties, not in code.
class Mechanism final : public AxisMechanismBase {
public:
    Mechanism(const sdk::MechanismConfig& config, std::unique_ptr<MechanismAxis> axis, AxisName moveAxis, double angleSign)
        : AxisMechanismBase(config), axisName_(moveAxis), sign_(angleSign) {
        addAxis(std::move(axis));
        publishNow();
    }

protected:
    // `active`: moving faster than 0.01 rad/s | m/s. Values: position [rad|m], velocity, voltage [V],
    // current [A], atLowerLimit / atUpperLimit (0|1).
    void publish(sdk::MechanismState& out) const override {
        const MechanismAxis& a = axis(0);
        const bool linear = a.settings().motion == Motion::Linear;
        const char* posUnit = linear ? "m" : "rad";
        const char* velUnit = linear ? "m/s" : "rad/s";
        out.active = std::abs(a.velocity()) > kMovingSpeed;
        const double p = a.position();
        out.links.push_back({kMoverLink, linear ? translationAlong(axisName_, p)
                                                : rotationAbout(axisName_, sign_ * (a.settings().limited ? p : sdk::wrapAngleRad(p)))});
        out.values.push_back(namedValue("position", p, posUnit));
        out.values.push_back(namedValue("velocity", a.velocity(), velUnit));
        out.values.push_back(namedValue("voltage", a.appliedVoltageV(), "V"));
        out.values.push_back(namedValue("current", a.motorCurrentA(), "A"));
        out.values.push_back(namedValue("atLowerLimit", a.atLowerLimit() ? 1.0 : 0.0, ""));
        out.values.push_back(namedValue("atUpperLimit", a.atUpperLimit() ? 1.0 : 0.0, ""));
    }

private:
    static constexpr const char* kMoverLink = "mover";
    AxisName axisName_;
    double sign_;
};

AxisOptions mechanismAxis() {
    AxisOptions o;
    o.motion = Motion::Rotary;
    o.selectableMotion = true;
    o.rotaryModel = plant_model::kDcMotor;
    o.limits = true;
    o.limitSwitchSlots = true;
    o.motorModel = "kraken_x60";
    o.brake = true;
    o.gearRatio = 10.0;
    o.inertiaKgM2 = 0.01;
    o.massKg = 2.0;
    o.lengthM = 0.3;
    o.gravity = true;                 // acts on Arm and Linear only
    o.viscousFrictionNmS = 0.001;     // bearing drag, so that a coasting rotary mechanism spins down
    return o;
}

sdk::PrimitiveSpec bodyPrimitive() {
    sdk::PrimitiveSpec p;
    p.name = "mover";   // every shape of a mechanism turns / slides with its moving link
    p.kind = sdk::PrimitiveKind::Box;
    p.local.position = {0.10f, 0.f, 0.f};   // off the axis, so a rotation is visible
    p.sizeM = {0.20f, 0.05f, 0.05f};
    p.color = "#9b59b6";
    return p;
}

sdk::MechanismTemplateInfo makeInfo(const MotorCatalog& motors) {
    const AxisOptions axis = mechanismAxis();
    sdk::MechanismTemplateInfo info;
    info.type = mechanism_type::kMechanism;
    info.displayName = "Mechanism";
    info.category = "Mechanism";
    info.description =
        "One motor-driven degree of freedom. Motion Rotary: flywheel, roller, belt or turret (stops optional). "
        "Arm: hood, pivot or wrist with gravity. Linear: elevator, climber or slide with gravity.";
    info.slotSpecs = axisSlots(axis);
    info.motionAxes = {axisMotionSpec(axis)};
    info.properties.properties = axisProperties(axis, motors);
    info.properties.properties.push_back(
        axisEnumProp("axis", "Axis (Rotary and Linear)", AxisName::Z, kMotionGroup));
    info.defaultGeometry.push_back(bodyPrimitive());
    info.links.push_back({"mover", "Moving part", sdk::Pose3{},
                          "Pose of the moving part at the start position (rest pose shown for position 0). "
                          "An Arm pivots about Y, 0 = horizontal, positive raising the +X end."});
    return info;
}

}  // namespace

sdk::Status registerMechanism(sdk::MechanismFactory& factory, const TemplateDeps& deps) {
    return registerValidated(
        factory, makeInfo(*deps.motors),
        [deps](const sdk::MechanismConfig& config,
               const sdk::Value& props) -> sdk::Result<std::unique_ptr<sdk::IMechanism>> {
            AxisOptions options = mechanismAxis();
            AxisName moveAxis = AxisName::Z;
            if (!parseAxis(getString(props, "axis", "z"), moveAxis))
                return sdk::makeError(sdk::ErrorCode::InvalidArgument, "property 'axis' has unknown value");
            // The arm convention fixes its pivot (Y, positive raises the +X end); a linear stage only works
            // against gravity along Z.
            const bool arm = getString(props, "motion", "rotary") == "arm";
            if (arm) moveAxis = AxisName::Y;
            options.armGravityEnabled = true;
            options.linearGravityEnabled = moveAxis == AxisName::Z;
            auto axis = buildAxis(config.id, options, props, deps);
            if (!axis.ok()) return axis.error();
            return std::unique_ptr<sdk::IMechanism>(
                std::make_unique<Mechanism>(config, std::move(axis).value(), moveAxis, arm ? -1.0 : 1.0));
        });
}

}  // namespace wpiemu::core::mechanisms::detail
