#include "wpiemu/plugins/game_2026/fuel_shooter.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/plugins/game_2026/mechanism_io.h"
#include "wpiemu/plugins/game_2026/property_helpers.h"
#include "wpiemu/sdk/core/sim_context.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr const char* kGroupMuzzle = "Muzzle";
constexpr const char* kGroupFlywheel = "Flywheel";
constexpr const char* kGroupAim = "Aim";
constexpr const char* kGroupFeed = "Feed";
constexpr const char* kGroupDispersion = "Dispersion";
constexpr double kMaxElevationRad = 1.5707;

sdk::PropertySchema shooterSchema() {
    using sdk::PropertyUnit;
    sdk::PropertySchema s;
    auto& p = s.properties;
    const ShooterParams d;
    p.push_back(props::vec3("muzzleOffset", "Muzzle offset", kGroupMuzzle, d.muzzleOffset,
                            "Where fuel leaves the shooter, in the mechanism frame (+X = shooting direction at 0 yaw)."));

    p.push_back(props::text("flywheelMechanism", "Flywheel mechanism", kGroupFlywheel, "",
                            "Id of the flywheel mechanism whose angular speed sets the exit speed."));
    p.push_back(props::text("flywheelValue", "Flywheel value", kGroupFlywheel, d.flywheelValue,
                            "Published angular-speed value name of that mechanism."));
    p.push_back(props::real("flywheelRadiusM", "Flywheel radius", kGroupFlywheel, d.flywheelRadiusM, 0.005, 0.5,
                            PropertyUnit::Meters, "Wheel radius: surface speed = angular speed x radius."));
    p.push_back(props::real("efficiency", "Efficiency", kGroupFlywheel, d.efficiency, 0.05, 1.5, PropertyUnit::Ratio,
                            "Exit velocity = flywheel surface speed x efficiency."));
    p.push_back(props::real("minExitSpeedMps", "Minimum exit speed", kGroupFlywheel, d.minExitSpeedMps, 0.0, 100.0,
                            PropertyUnit::MetersPerSecond, "Below this exit speed the shooter does not fire."));
    p.push_back(props::real("fixedExitSpeedMps", "Fixed exit speed", kGroupFlywheel, 0.0, 0.0, 100.0,
                            PropertyUnit::MetersPerSecond,
                            "If > 0, fuel always leaves at this speed (no flywheel mechanism needed)."));

    p.push_back(props::real("hoodAngleRad", "Hood angle", kGroupAim, d.hoodAngleRad, -1.5, 1.5, PropertyUnit::Radians,
                            "Launch elevation above horizontal; added to the hood mechanism's value when one is named."));
    p.push_back(props::text("hoodMechanism", "Hood mechanism", kGroupAim, "", "Optional hood/pivot mechanism."));
    p.push_back(props::text("hoodValue", "Hood value", kGroupAim, d.hoodValue, "Published angle value name."));
    p.push_back(props::text("turretMechanism", "Turret mechanism", kGroupAim, "",
                            "Optional turret mechanism (leave empty if this shooter is attached to the turret link)."));
    p.push_back(props::text("turretValue", "Turret value", kGroupAim, d.turretValue, "Published angle value name."));
    p.push_back(props::real("turretYawOffsetRad", "Yaw offset", kGroupAim, 0.0, -7.0, 7.0, PropertyUnit::Radians,
                            "Fixed yaw added to the turret value."));
    p.push_back(props::real("hoodGain", "Hood gain", kGroupAim, d.hoodGain, -10.0, 10.0, PropertyUnit::Ratio,
                            "Radians of elevation per radian of the hood value (-1 inverts)."));
    p.push_back(props::real("turretGain", "Turret gain", kGroupAim, d.turretGain, -10.0, 10.0, PropertyUnit::Ratio,
                            "Radians of yaw per radian of the turret value."));

    p.push_back(props::text("feedMechanism", "Feed mechanism", kGroupFeed, "",
                            "Indexer/kicker/roller that feeds the flywheel. Empty: fire whenever the flywheel is up "
                            "to speed."));
    p.push_back(props::text("feedValue", "Feed value", kGroupFeed, d.feedValue,
                            "Published value name of the feed mechanism."));
    p.push_back(props::real("feedThreshold", "Feed threshold", kGroupFeed, d.feedThreshold, 0.0, 1e6,
                            PropertyUnit::RadiansPerSecond, "|value| (SI) above which fuel is being fed."));
    p.push_back(props::real("fireRateHz", "Fire rate", kGroupFeed, d.fireRateHz, 0.1, 200.0, PropertyUnit::Hertz,
                            "Fuel launched per second while feeding."));

    p.push_back(props::real("exitSpeedJitter", "Exit speed jitter", kGroupDispersion, 0.0, 0.0, 0.5, PropertyUnit::Ratio,
                            "Uniform +- fraction applied to each shot's exit speed (seeded RNG)."));
    p.push_back(props::real("aimJitterRad", "Aim jitter", kGroupDispersion, 0.0, 0.0, 0.5, PropertyUnit::Radians,
                            "Uniform +- angle applied to yaw and pitch of each shot."));
    p.push_back(props::flag("inheritRobotVelocity", "Inherit robot velocity", kGroupDispersion, true,
                            "Add the muzzle's velocity (robot motion) to the fuel's launch velocity."));
    return s;
}

}  // namespace

ShooterParams parseShooterParams(const sdk::Value& properties) {
    const sdk::PropertySchema schema = shooterSchema();
    sdk::Value v = properties;
    sdk::applyDefaults(schema, v);
    ShooterParams p;
    if (!sdk::fromValue(v.get("muzzleOffset"), p.muzzleOffset))
        sdk::fromValue(schema.find("muzzleOffset")->defaultValue, p.muzzleOffset);
    auto text = [&](const char* key, const std::string& fallback) { return v.get(key).asString(fallback); };
    auto num = [&](const char* key, double fallback) {
        const double x = v.get(key).asDouble(fallback);
        return std::isfinite(x) ? x : fallback;
    };
    p.flywheelMechanism = text("flywheelMechanism", "");
    p.flywheelValue = text("flywheelValue", p.flywheelValue);
    p.flywheelRadiusM = std::max(1e-3, num("flywheelRadiusM", p.flywheelRadiusM));
    p.efficiency = std::max(0.0, num("efficiency", p.efficiency));
    p.fixedExitSpeedMps = std::max(0.0, num("fixedExitSpeedMps", 0.0));
    p.minExitSpeedMps = std::max(0.0, num("minExitSpeedMps", p.minExitSpeedMps));
    p.hoodMechanism = text("hoodMechanism", "");
    p.hoodValue = text("hoodValue", p.hoodValue);
    p.hoodGain = num("hoodGain", p.hoodGain);
    p.hoodAngleRad = num("hoodAngleRad", p.hoodAngleRad);
    p.turretMechanism = text("turretMechanism", "");
    p.turretValue = text("turretValue", p.turretValue);
    p.turretGain = num("turretGain", p.turretGain);
    p.turretYawOffsetRad = num("turretYawOffsetRad", 0.0);
    p.feedMechanism = text("feedMechanism", "");
    p.feedValue = text("feedValue", p.feedValue);
    p.feedThreshold = std::max(0.0, num("feedThreshold", p.feedThreshold));
    p.fireRateHz = std::max(0.1, num("fireRateHz", p.fireRateHz));
    p.exitSpeedJitter = std::clamp(num("exitSpeedJitter", 0.0), 0.0, 0.5);
    p.aimJitterRad = std::clamp(num("aimJitterRad", 0.0), 0.0, 0.5);
    p.inheritRobotVelocity = v.get("inheritRobotVelocity", true).asBool(true);
    return p;
}

sdk::MechanismTemplateInfo fuelShooterTemplate() {
    sdk::MechanismTemplateInfo info;
    info.type = kShooterType;
    info.displayName = "Fuel Shooter";
    info.category = "Shooter";
    info.origin = "game_2026";
    info.description =
        "REBUILT fuel launcher. Reads the flywheel speed, feed mechanism and optional hood/turret of your own "
        "mechanisms; launches real fuel bodies from the muzzle while feeding and the fuel meter is not empty.";
    info.properties = shooterSchema();
    sdk::LinkSpec muzzle;
    muzzle.name = "muzzle";
    muzzle.label = "Muzzle";
    muzzle.description = "Exit point; its +X axis is the current launch direction.";
    info.links.push_back(std::move(muzzle));
    sdk::PrimitiveSpec barrel;
    barrel.name = "shooter_barrel";
    barrel.kind = sdk::PrimitiveKind::Cylinder;
    barrel.local = sdk::Pose3{{0.25f, 0.f, 0.55f}, sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, 1.5707963f)};
    barrel.radiusM = 0.08f;
    barrel.heightM = 0.30f;
    barrel.color = "#d94f30";
    info.defaultGeometry.push_back(std::move(barrel));
    return info;
}

FuelShooterMechanism::FuelShooterMechanism(const sdk::MechanismBuildContext& ctx)
    : id_(ctx.config.id), params_(parseShooterParams(ctx.config.properties)), mountFallback_(ctx.config.mount),
      rootInChassis_(ctx.config.mount) {}

void FuelShooterMechanism::warnMissing(sdk::SimContext& ctx, const std::string& mechanism) {
    if (std::find(warned_.begin(), warned_.end(), mechanism) != warned_.end()) return;
    warned_.push_back(mechanism);
    ctx.logger().warn("plugin.game_2026", "fuel shooter '" + id_ + "' references unknown mechanism '" + mechanism + "'");
}

void FuelShooterMechanism::tick(sdk::SimContext& ctx, double dt) {
    rootInChassis_ = mountFallback_;
    sdk::IMechanismDirectory* dir = ctx.mechanisms();
    if (dir != nullptr)
        if (const auto p = dir->linkPoseInChassis(id_, sdk::kRootLink)) rootInChassis_ = *p;

    bool referencesValid = true;
    auto read = [&](const std::string& mechanism, const std::string& value) {
        MechanismReading r = readMechanism(dir, mechanism, value);
        if (!mechanism.empty() && !r.found) {
            referencesValid = false;
            warnMissing(ctx, mechanism);
        }
        return r;
    };

    if (params_.fixedExitSpeedMps > 0.0) {
        exitSpeedMps_ = params_.fixedExitSpeedMps;
    } else if (!params_.flywheelMechanism.empty()) {
        const MechanismReading fly = read(params_.flywheelMechanism, params_.flywheelValue);
        exitSpeedMps_ = fly.hasValue ? std::fabs(fly.value) * params_.flywheelRadiusM * params_.efficiency : 0.0;
    } else {
        exitSpeedMps_ = 0.0;
    }

    elevationRad_ = params_.hoodAngleRad;
    if (!params_.hoodMechanism.empty()) {
        const MechanismReading hood = read(params_.hoodMechanism, params_.hoodValue);
        if (hood.hasValue) elevationRad_ += params_.hoodGain * hood.value;
    }
    elevationRad_ = std::clamp(elevationRad_, -kMaxElevationRad, kMaxElevationRad);

    yawRad_ = params_.turretYawOffsetRad;
    if (!params_.turretMechanism.empty()) {
        const MechanismReading turret = read(params_.turretMechanism, params_.turretValue);
        if (turret.hasValue) yawRad_ += params_.turretGain * turret.value;
    }

    bool fed = true;
    if (!params_.feedMechanism.empty())
        fed = exceeds(read(params_.feedMechanism, params_.feedValue), params_.feedThreshold);
    feeding_ =
        referencesValid && fed && exitSpeedMps_ >= params_.minExitSpeedMps && exitSpeedMps_ > 0.0;

    // Token bucket: primed while idle so the first shot leaves as soon as feeding starts.
    if (!feeding_) budget_ = 1.0;
    else budget_ = std::min(2.0, budget_ + params_.fireRateHz * dt);
}

int FuelShooterMechanism::shotBudget() const { return feeding_ ? static_cast<int>(std::floor(budget_)) : 0; }

void FuelShooterMechanism::consumeShots(int count) { budget_ = std::max(0.0, budget_ - static_cast<double>(count)); }

ShotSolution FuelShooterMechanism::solution() const {
    ShotSolution s;
    const double ce = std::cos(elevationRad_);
    const sdk::Vec3 local{static_cast<float>(ce * std::cos(yawRad_)), static_cast<float>(ce * std::sin(yawRad_)),
                          static_cast<float>(std::sin(elevationRad_))};
    s.directionInChassis = sdk::normalized(rootInChassis_.rotation.rotate(local));
    s.muzzleInChassis = rootInChassis_.transformPoint(params_.muzzleOffset);
    s.exitSpeedMps = exitSpeedMps_;
    s.speedJitter = params_.exitSpeedJitter;
    s.aimJitterRad = params_.aimJitterRad;
    s.inheritRobotVelocity = params_.inheritRobotVelocity;
    return s;
}

void FuelShooterMechanism::reset() {
    feeding_ = false;
    budget_ = 1.0;
    exitSpeedMps_ = 0.0;
    rootInChassis_ = mountFallback_;
    warned_.clear();
}

sdk::MechanismState FuelShooterMechanism::state() const {
    sdk::MechanismState s;
    s.id = id_;
    s.type = std::string(kShooterType);
    s.active = feeding_;
    s.values.push_back({"feeding", feeding_ ? 1.0 : 0.0, ""});
    s.values.push_back({"exitSpeed", exitSpeedMps_, "m/s"});
    s.values.push_back({"elevation", elevationRad_, "rad"});
    s.values.push_back({"yaw", yawRad_, "rad"});
    sdk::LinkPose muzzle;
    muzzle.name = "muzzle";
    muzzle.pose = sdk::Pose3{params_.muzzleOffset, sdk::Quat::fromEuler(0.f, static_cast<float>(-elevationRad_),
                                                                         static_cast<float>(yawRad_))};
    s.links.push_back(std::move(muzzle));
    return s;
}

std::unique_ptr<sdk::IMechanism> createFuelShooter(const sdk::MechanismBuildContext& ctx) {
    return std::make_unique<FuelShooterMechanism>(ctx);
}

}  // namespace wpiemu::plugins::game_2026
