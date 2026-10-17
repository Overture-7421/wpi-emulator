#include <cmath>

#include "template_support.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::core::mechanisms::detail {

namespace {

constexpr const char* kIntakeGroup = "Intake";
constexpr const char* kDeployPrefix = "deploy";
constexpr const char* kRollerPrefix = "roller";

struct IntakeParams {
    bool deployEnabled = true;
    bool deployedAtMax = true;
    double deployedTolerance = 0.05;
    AxisName deployAxis = AxisName::Y;
    AxisName rollerSpin = AxisName::Y;
    sdk::Vec3 rollerOffset{0.3f, 0.f, 0.f};
    double rollerRadiusM = 0.03;
    double activeSurfaceSpeedMps = 0.5;
};

class IntakeMechanism final : public AxisMechanismBase {
public:
    IntakeMechanism(const sdk::MechanismConfig& config, std::unique_ptr<MechanismAxis> deploy,
                    std::unique_ptr<MechanismAxis> roller, const IntakeParams& params)
        : AxisMechanismBase(config), params_(params), rollerIndex_(deploy ? 1u : 0u) {
        if (deploy) addAxis(std::move(deploy));
        addAxis(std::move(roller));
        publishNow();
    }

protected:
    // `active`: roller surface speed >= threshold AND the deploy axis is at its deployed position (within
    // the tolerance; always true without a deploy axis). Links: "deploy" (the moving frame) and
    // "roller" (child of deploy at rollerOffset, spinning). Values: deployPosition, deployVelocity,
    // deployed (0|1), rollerVelocity [rad/s], rollerSurfaceSpeed [m/s], voltage/current of both axes.
    void publish(sdk::MechanismState& out) const override {
        const MechanismAxis& roller = axis(rollerIndex_);
        const double surface = roller.velocity() * params_.rollerRadiusM;

        sdk::Pose3 deployPose;
        bool deployed = true;
        if (rollerIndex_ == 1u) {
            const MechanismAxis& deploy = axis(0);
            const double p = deploy.position();
            const AxisSettings& s = deploy.settings();
            deployPose = s.motion == Motion::Linear ? translationAlong(params_.deployAxis, p)
                                                    : rotationAbout(params_.deployAxis, s.limited ? p : wrapAngle(p));
            const double target = params_.deployedAtMax ? s.plant.maxPosition : s.plant.minPosition;
            deployed = std::abs(p - target) <= params_.deployedTolerance;
            out.links.push_back({"deploy", deployPose});
            out.values.push_back(namedValue("deployPosition", p, s.motion == Motion::Linear ? "m" : "rad"));
            out.values.push_back(namedValue("deployVelocity", deploy.velocity(), s.motion == Motion::Linear ? "m/s" : "rad/s"));
            out.values.push_back(namedValue("deployVoltage", deploy.appliedVoltageV(), "V"));
            out.values.push_back(namedValue("deployCurrent", deploy.motorCurrentA(), "A"));
        }
        const sdk::Pose3 rollerPose = deployPose * sdk::Pose3{params_.rollerOffset, sdk::Quat{}} *
                                      rotationAbout(params_.rollerSpin, wrapAngle(roller.position()));
        out.links.push_back({"roller", rollerPose});
        out.active = std::abs(surface) >= params_.activeSurfaceSpeedMps && deployed;
        out.values.push_back(namedValue("deployed", deployed ? 1.0 : 0.0, ""));
        out.values.push_back(namedValue("rollerVelocity", roller.velocity(), "rad/s"));
        out.values.push_back(namedValue("rollerSurfaceSpeed", surface, "m/s"));
        out.values.push_back(namedValue("rollerVoltage", roller.appliedVoltageV(), "V"));
        out.values.push_back(namedValue("rollerCurrent", roller.motorCurrentA(), "A"));
    }

private:
    IntakeParams params_;
    std::size_t rollerIndex_;
};

AxisOptions deployOptions() {
    AxisOptions o;
    o.prefix = kDeployPrefix;
    o.title = "Deploy ";
    o.motion = Motion::Arm;
    o.selectableMotion = true;
    o.rotaryModel = plant_model::kDcMotor;
    o.limits = true;
    o.limitSwitchSlots = true;
    o.motorRequired = false;
    o.maxMotors = 2;
    o.gearRatio = 30.0;
    o.inertiaKgM2 = 0.0;
    o.massKg = 3.0;
    o.lengthM = 0.35;
    o.brake = true;
    o.minPosition = 0.0;
    o.maxPosition = 1.6;
    return o;
}

AxisOptions rollerOptions() {
    AxisOptions o;
    o.prefix = kRollerPrefix;
    o.title = "Roller ";
    o.motion = Motion::Rotary;
    o.rotaryModel = plant_model::kFlywheel;
    o.maxMotors = 2;
    o.gearRatio = 3.0;
    o.inertiaKgM2 = 0.002;
    o.viscousFrictionNmS = 0.001;
    return o;
}

sdk::MechanismTemplateInfo makeInfo(const MotorCatalog& motors) {
    sdk::MechanismTemplateInfo info;
    info.type = mechanism_type::kIntake;
    info.displayName = "Intake";
    info.category = "Intake";
    info.description =
        "A deploy axis (pivoting or sliding) plus a roller. Reports 'active' while the roller spins above the "
        "threshold and the intake is deployed; game logic uses that to absorb game pieces.";
    info.slotSpecs = axisSlots(deployOptions());
    for (sdk::SlotSpec& s : axisSlots(rollerOptions())) info.slotSpecs.push_back(std::move(s));

    auto& p = info.properties.properties;
    p = axisProperties(deployOptions(), motors);
    for (sdk::PropertySpec& s : axisProperties(rollerOptions(), motors)) p.push_back(std::move(s));
    p.push_back(boolProp("deployEnabled", "Has deploy axis", "Off: a fixed intake that is always deployed.", true,
                         kIntakeGroup));
    p.push_back(axisEnumProp("deployAxis", "Deploy axis",
                             AxisName::Y, kIntakeGroup));
    p.push_back(boolProp("deployedAtMax", "Deployed at maximum position",
                         "Off: the minimum position is the deployed one.", true, kIntakeGroup));
    p.push_back(doubleProp("deployedTolerance", "Deployed tolerance",
                           "Distance (rad or m) from the deployed position that still counts as deployed.",
                           sdk::PropertyUnit::None, 0.05, kIntakeGroup, 0.0, 10.0));
    sdk::PropertySpec offset;
    offset.key = "rollerOffset";
    offset.label = "Roller position";
    offset.description = "Roller origin in the deploy frame (metres).";
    offset.type = sdk::PropertyType::Vec3;
    offset.unit = sdk::PropertyUnit::Meters;
    offset.defaultValue = sdk::toValue(sdk::Vec3{0.3f, 0.f, 0.f});
    offset.group = kIntakeGroup;
    p.push_back(std::move(offset));
    p.push_back(doubleProp("rollerRadiusM", "Roller radius", "Surface speed = angular velocity * radius.",
                           sdk::PropertyUnit::Meters, 0.03, kIntakeGroup, 0.001, 1.0));
    p.push_back(axisEnumProp("rollerSpinAxis", "Roller spin axis", AxisName::Y, kIntakeGroup));
    p.push_back(doubleProp("activeSurfaceSpeedMps", "Active above surface speed",
                           "Roller surface speed that makes the (deployed) intake 'active'.",
                           sdk::PropertyUnit::MetersPerSecond, 0.5, kIntakeGroup, 0.0, 100.0));

    sdk::PrimitiveSpec frame;
    frame.name = "deploy";
    frame.kind = sdk::PrimitiveKind::Box;
    frame.sizeM = {0.30f, 0.50f, 0.03f};
    frame.local = sdk::Pose3{{0.15f, 0.f, 0.f}, sdk::Quat{}};
    frame.color = "#7f8c8d";
    info.defaultGeometry.push_back(frame);
    sdk::PrimitiveSpec roller;
    roller.name = "roller";
    roller.kind = sdk::PrimitiveKind::Cylinder;
    roller.radiusM = 0.03f;
    roller.heightM = 0.45f;
    roller.local = sdk::Pose3{{0.3f, 0.f, 0.f}, sdk::Quat::fromAxisAngle({1.f, 0.f, 0.f}, static_cast<float>(sdk::kPi / 2))};
    roller.color = "#4a90d9";
    info.defaultGeometry.push_back(roller);

    info.links.push_back({"deploy", "Deploy frame", sdk::Pose3{}, "Moves with the deploy axis."});
    info.links.push_back({"roller", "Roller", sdk::Pose3{sdk::Vec3{0.3f, 0.f, 0.f}, sdk::Quat{}},
                          "Spinning roller, child of the deploy frame."});
    return info;
}

}  // namespace

sdk::Status registerIntake(sdk::MechanismFactory& factory, const TemplateDeps& deps) {
    return registerValidated(
        factory, makeInfo(*deps.motors),
        [deps](const sdk::MechanismConfig& config,
               const sdk::Value& props) -> sdk::Result<std::unique_ptr<sdk::IMechanism>> {
            IntakeParams params;
            params.deployEnabled = getBool(props, "deployEnabled", true);
            params.deployedAtMax = getBool(props, "deployedAtMax", true);
            params.deployedTolerance = getDouble(props, "deployedTolerance", params.deployedTolerance);
            params.rollerRadiusM = getDouble(props, "rollerRadiusM", params.rollerRadiusM);
            params.activeSurfaceSpeedMps = getDouble(props, "activeSurfaceSpeedMps", params.activeSurfaceSpeedMps);
            if (!parseAxis(getString(props, "deployAxis", "y"), params.deployAxis))
                return sdk::makeError(sdk::ErrorCode::InvalidArgument, "property 'deployAxis' has unknown value");
            if (!parseAxis(getString(props, "rollerSpinAxis", "y"), params.rollerSpin))
                return sdk::makeError(sdk::ErrorCode::InvalidArgument, "property 'rollerSpinAxis' has unknown value");
            if (const sdk::Value* v = props.find("rollerOffset"))
                if (!sdk::fromValue(*v, params.rollerOffset))
                    return sdk::makeError(sdk::ErrorCode::InvalidArgument, "property 'rollerOffset' is not a Vec3");

            std::unique_ptr<MechanismAxis> deploy;
            if (params.deployEnabled) {
                AxisOptions options = deployOptions();
                // The arm COM is along +X. Positive Y rotation lowers it; X/Z rotations
                // leave its height unchanged. Only Z translation works against gravity.
                options.gravityAngleOffsetRad = sdk::kPi;
                options.armGravityEnabled = params.deployAxis == AxisName::Y;
                options.linearGravityEnabled = params.deployAxis == AxisName::Z;
                auto built = buildAxis(config.id, options, props, deps);
                if (!built.ok()) return built.error();
                deploy = std::move(built).value();
            }
            auto roller = buildAxis(config.id, rollerOptions(), props, deps);
            if (!roller.ok()) return roller.error();
            return std::unique_ptr<sdk::IMechanism>(
                std::make_unique<IntakeMechanism>(config, std::move(deploy), std::move(roller).value(), params));
        });
}

}  // namespace wpiemu::core::mechanisms::detail
