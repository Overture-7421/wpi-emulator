#include <cmath>

#include "template_support.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::core::mechanisms::detail {

namespace {

constexpr const char* kRollerGroup = "Roller";

struct RollerFlavor {
    const char* type;
    const char* displayName;
    const char* category;
    const char* description;
    const char* linkName;
    AxisOptions axis;
    double radiusM;
    double activeSurfaceSpeedMps;
    AxisName spin;
    bool beamBreakSlots;
    sdk::PrimitiveKind primitive;
    sdk::Vec3 sizeM;
    std::string color;
};

class RollerMechanism final : public AxisMechanismBase {
public:
    RollerMechanism(const sdk::MechanismConfig& config, std::unique_ptr<MechanismAxis> axis, std::string linkName,
                    double radiusM, double activeSurfaceSpeedMps, AxisName spin)
        : AxisMechanismBase(config), link_(std::move(linkName)), radiusM_(radiusM),
          threshold_(activeSurfaceSpeedMps), spin_(spin) {
        addAxis(std::move(axis));
        publishNow();
    }

protected:
    // `active`: |surface speed| >= threshold. Values: position [rad], velocity [rad/s],
    // surfaceSpeed [m/s] (= velocity * radius), voltage [V], current [A].
    void publish(sdk::MechanismState& out) const override {
        const MechanismAxis& a = axis(0);
        const double surface = a.velocity() * radiusM_;
        out.active = std::abs(surface) >= threshold_;
        out.links.push_back({link_, rotationAbout(spin_, wrapAngle(a.position()))});
        out.values.push_back(namedValue("position", a.position(), "rad"));
        out.values.push_back(namedValue("velocity", a.velocity(), "rad/s"));
        out.values.push_back(namedValue("surfaceSpeed", surface, "m/s"));
        out.values.push_back(namedValue("voltage", a.appliedVoltageV(), "V"));
        out.values.push_back(namedValue("current", a.motorCurrentA(), "A"));
    }

private:
    std::string link_;
    double radiusM_;
    double threshold_;
    AxisName spin_;
};

AxisOptions rotaryAxis(const char* motor, double gear, double inertia, bool brake) {
    AxisOptions o;
    o.motion = Motion::Rotary;
    o.rotaryModel = plant_model::kFlywheel;
    o.motorModel = motor;
    o.gearRatio = gear;
    o.inertiaKgM2 = inertia;
    o.brake = brake;
    o.viscousFrictionNmS = 0.001;   // bearings / belt drag so that a coasting roller spins down
    return o;
}

RollerFlavor rollerFlavor() {
    return {mechanism_type::kRoller, "Generic roller", "Generic",
            "A motor-driven roller or wheel (rotary, no stops). Publishes its surface speed.", "roller",
            rotaryAxis("kraken_x60", 3.0, 0.002, false), 0.03, 0.5, AxisName::Y, false, sdk::PrimitiveKind::Cylinder,
            {0.04f, 0.04f, 0.30f}, "#4a90d9"};
}

RollerFlavor shooterFlavor() {
    return {mechanism_type::kShooter, "Shooter / flywheel", "Shooter",
            "A flywheel driven directly or through a ratio. Publishes the surface speed used for projectile exit speed.",
            "flywheel", rotaryAxis("kraken_x60", 1.0, 0.004, false), 0.05, 1.0, AxisName::Y, false,
            sdk::PrimitiveKind::Cylinder, {0.10f, 0.10f, 0.05f}, "#d9534f"};
}

RollerFlavor indexerFlavor() {
    return {mechanism_type::kIndexer, "Indexer / conveyor", "Indexer",
            "A belt or roller stack that moves game pieces; optional entry/exit beam breaks are driven by game logic.",
            "belt", rotaryAxis("neo", 5.0, 0.003, false), 0.025, 0.2, AxisName::Y, true, sdk::PrimitiveKind::Box,
            {0.40f, 0.08f, 0.04f}, "#5cb85c"};
}

sdk::MechanismTemplateInfo infoFor(const RollerFlavor& f, const MotorCatalog& motors) {
    sdk::MechanismTemplateInfo info;
    info.type = f.type;
    info.displayName = f.displayName;
    info.category = f.category;
    info.description = f.description;
    info.slotSpecs = axisSlots(f.axis);
    info.motionAxes = {axisMotionSpec(f.axis)};
    if (f.beamBreakSlots) {
        info.slotSpecs.push_back({"entryBeamBreak", "Entry beam break", sdk::SlotType::BeamBreak, false,
                                  "Written by the game logic (piece present at the entry); the template never writes it."});
        info.slotSpecs.push_back({"exitBeamBreak", "Exit beam break", sdk::SlotType::BeamBreak, false,
                                  "Written by the game logic (piece present at the exit); the template never writes it."});
    }
    info.properties.properties = axisProperties(f.axis, motors);
    info.properties.properties.push_back(doubleProp("radiusM", "Roller radius", "Surface speed = angular velocity * radius.",
                                                    sdk::PropertyUnit::Meters, f.radiusM, kRollerGroup, 0.001, 1.0));
    info.properties.properties.push_back(axisEnumProp("spinAxis", "Spin axis", f.spin, kRollerGroup));
    info.properties.properties.push_back(
        doubleProp("activeSurfaceSpeedMps", "Active above surface speed",
                   "The mechanism reports 'active' while |surface speed| is at least this.",
                   sdk::PropertyUnit::MetersPerSecond, f.activeSurfaceSpeedMps, kRollerGroup, 0.0, 100.0));

    sdk::PrimitiveSpec geometry;
    geometry.name = f.linkName;
    geometry.kind = f.primitive;
    geometry.sizeM = f.sizeM;
    geometry.radiusM = f.sizeM.x * 0.5f;
    geometry.heightM = f.sizeM.z;
    geometry.color = f.color;
    if (f.primitive == sdk::PrimitiveKind::Cylinder)
        geometry.local = sdk::Pose3{sdk::Vec3{}, sdk::Quat::fromAxisAngle({1.f, 0.f, 0.f}, static_cast<float>(sdk::kPi / 2))};
    info.defaultGeometry.push_back(geometry);

    info.links.push_back({f.linkName, std::string("Rotating ") + f.linkName, sdk::Pose3{},
                          "Spins about the spin axis by the integrated position."});
    return info;
}

sdk::Status registerFlavor(sdk::MechanismFactory& factory, const TemplateDeps& deps, const RollerFlavor& flavor) {
    sdk::MechanismTemplateInfo info = infoFor(flavor, *deps.motors);
    return registerValidated(
        factory, std::move(info),
        [flavor, deps](const sdk::MechanismConfig& config,
                       const sdk::Value& props) -> sdk::Result<std::unique_ptr<sdk::IMechanism>> {
            auto axis = buildAxis(config.id, flavor.axis, props, deps);
            if (!axis.ok()) return axis.error();
            AxisName spin = flavor.spin;
            if (!parseAxis(getString(props, "spinAxis", axisKey(flavor.spin)), spin))
                return sdk::makeError(sdk::ErrorCode::InvalidArgument, "property 'spinAxis' has unknown value");
            return std::unique_ptr<sdk::IMechanism>(std::make_unique<RollerMechanism>(
                config, std::move(axis).value(), flavor.linkName, getDouble(props, "radiusM", flavor.radiusM),
                getDouble(props, "activeSurfaceSpeedMps", flavor.activeSurfaceSpeedMps), spin));
        });
}

}  // namespace

sdk::Status registerRollerFamily(sdk::MechanismFactory& factory, const TemplateDeps& deps) {
    for (const RollerFlavor& f : {rollerFlavor(), shooterFlavor(), indexerFlavor()})
        if (const sdk::Status s = registerFlavor(factory, deps, f); !s.ok()) return s;
    return sdk::okStatus();
}

}  // namespace wpiemu::core::mechanisms::detail
