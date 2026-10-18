#include "wpiemu/plugins/game_2026/fuel_zones.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "wpiemu/plugins/game_2026/property_helpers.h"
#include "wpiemu/sdk/game/zone_activation.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr const char* kGroupLimits = "Limits";
constexpr const char* kGroupFlywheel = "Flywheel";
constexpr const char* kGroupAim = "Aim";
constexpr const char* kGroupFire = "Fire";
constexpr const char* kGroupDispersion = "Dispersion";
constexpr double kMaxElevationRad = 1.5707;
constexpr float kDefaultPickupLocalX = 0.45f;
constexpr float kDefaultPickupLocalZ = 0.15f;

sdk::PropertySchema pickupSchema() {
    sdk::PropertySchema s;
    const PickupParams d;
    s.properties.push_back(props::real("intakeRateHz", "Intake rate", kGroupLimits, d.intakeRateHz, 0.0, 1000.0,
                                       sdk::PropertyUnit::Hertz,
                                       "Maximum fuel absorbed per second by this pickup (0 = unlimited)."));
    return s;
}

sdk::PropertySchema shooterSchema() {
    using sdk::PropertyUnit;
    sdk::PropertySchema s;
    auto& p = s.properties;
    const ShooterParams d;

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
    p.push_back(props::real("hoodGain", "Hood gain", kGroupAim, d.hoodGain, -10.0, 10.0, PropertyUnit::Ratio,
                            "Radians of elevation per radian of the hood value (-1 inverts)."));
    p.push_back(props::text("turretMechanism", "Turret mechanism", kGroupAim, "",
                            "Optional turret mechanism (leave empty if the zone is attached to the turret link)."));
    p.push_back(props::text("turretValue", "Turret value", kGroupAim, d.turretValue, "Published angle value name."));
    p.push_back(props::real("turretGain", "Turret gain", kGroupAim, d.turretGain, -10.0, 10.0, PropertyUnit::Ratio,
                            "Radians of yaw per radian of the turret value."));
    p.push_back(props::real("turretYawOffsetRad", "Yaw offset", kGroupAim, 0.0, -7.0, 7.0, PropertyUnit::Radians,
                            "Fixed yaw added to the turret value."));

    p.push_back(props::real("fireRateHz", "Fire rate", kGroupFire, d.fireRateHz, 0.1, 200.0, PropertyUnit::Hertz,
                            "Fuel launched per second while the shooter is active."));

    p.push_back(props::real("exitSpeedJitter", "Exit speed jitter", kGroupDispersion, 0.0, 0.0, 0.5, PropertyUnit::Ratio,
                            "Uniform +- fraction applied to each shot's exit speed (seeded RNG)."));
    p.push_back(props::real("aimJitterRad", "Aim jitter", kGroupDispersion, 0.0, 0.0, 0.5, PropertyUnit::Radians,
                            "Uniform +- angle applied to yaw and pitch of each shot."));
    p.push_back(props::flag("inheritRobotVelocity", "Inherit robot velocity", kGroupDispersion, true,
                            "Add the muzzle's velocity (robot motion) to the fuel's launch velocity."));
    return s;
}

double finiteOr(const sdk::Value& v, const char* key, double fallback) {
    const double x = v.get(key).asDouble(fallback);
    return std::isfinite(x) ? x : fallback;
}

}  // namespace

PickupParams parsePickupParams(const sdk::Value& properties) {
    sdk::Value v = properties;
    sdk::applyDefaults(pickupSchema(), v);
    PickupParams p;
    p.intakeRateHz = std::max(0.0, finiteOr(v, "intakeRateHz", p.intakeRateHz));
    return p;
}

ShooterParams parseShooterParams(const sdk::Value& properties) {
    sdk::Value v = properties;
    sdk::applyDefaults(shooterSchema(), v);
    ShooterParams p;
    auto text = [&](const char* key, const std::string& fallback) { return v.get(key).asString(fallback); };
    auto num = [&](const char* key, double fallback) { return finiteOr(v, key, fallback); };
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
    p.fireRateHz = std::max(0.1, num("fireRateHz", p.fireRateHz));
    p.exitSpeedJitter = std::clamp(num("exitSpeedJitter", 0.0), 0.0, 0.5);
    p.aimJitterRad = std::clamp(num("aimJitterRad", 0.0), 0.0, 0.5);
    p.inheritRobotVelocity = v.get("inheritRobotVelocity", true).asBool(true);
    return p;
}

sdk::ZoneKindInfo pickupZoneKind() {
    sdk::ZoneKindInfo k;
    k.kind = kPickupKind;
    k.displayName = "Fuel pickup";
    k.description =
        "REBUILT fuel pickup volume. While its activation holds (e.g. the intake roller is spinning and/or the "
        "intake arm is deployed) fuel whose centre is inside the volume is absorbed into the robot's fuel meter, "
        "nearest the zone centre first, until the meter is full.";
    k.shape = sdk::ZoneShape::Volume;
    k.properties = pickupSchema();
    k.defaultSizeM = {0.30f, 0.60f, 0.30f};
    k.defaultLocal = sdk::Pose3{{kDefaultPickupLocalX, 0.f, kDefaultPickupLocalZ}, sdk::Quat{}};
    k.color = "#44cc66";
    return k;
}

sdk::ZoneKindInfo shooterZoneKind() {
    sdk::ZoneKindInfo k;
    k.kind = kShooterKind;
    k.displayName = "Fuel shooter";
    k.description =
        "REBUILT fuel muzzle. Fuel leaves the zone origin along its +X axis (rotated by the hood and turret) at "
        "flywheel surface speed x efficiency while its activation holds (e.g. the indexer is running), the flywheel "
        "is up to speed and the fuel meter is not empty. Attach it to a turret link to aim with the turret.";
    k.shape = sdk::ZoneShape::Muzzle;
    k.properties = shooterSchema();
    k.defaultLocal = sdk::Pose3{{0.25f, 0.f, 0.55f}, sdk::Quat{}};
    k.color = "#ff7043";
    return k;
}

std::vector<sdk::ZoneKindInfo> zoneKinds() { return {pickupZoneKind(), shooterZoneKind()}; }

void FuelZoneRuntime::warnMissing(sdk::SimContext& ctx, const std::string& mechanism) {
    if (std::find(warned_.begin(), warned_.end(), mechanism) != warned_.end()) return;
    warned_.push_back(mechanism);
    ctx.logger().warn("plugin.game_2026", "zone '" + config_.id + "' references unknown mechanism '" + mechanism + "'");
}

PickupZoneRuntime::PickupZoneRuntime(sdk::ZoneConfig config)
    : FuelZoneRuntime(std::move(config)), params_(parsePickupParams(config_.properties)) {}

void PickupZoneRuntime::update(sdk::SimContext&, const sdk::Pose3& poseInChassis, bool activationHeld, double dt) {
    poseInChassis_ = poseInChassis;
    active_ = activationHeld;
    // Token bucket: primed while idle so the first fuel is taken the moment the zone becomes active.
    if (!active_) budget_ = 1.0;
    else budget_ = std::min(2.0, budget_ + params_.intakeRateHz * dt);
}

int PickupZoneRuntime::pickupBudget() const {
    if (!active_) return 0;
    if (params_.intakeRateHz <= 0.0) return std::numeric_limits<int>::max();
    return static_cast<int>(std::floor(budget_));
}

void PickupZoneRuntime::consumePickups(int count) {
    if (params_.intakeRateHz > 0.0) budget_ = std::max(0.0, budget_ - static_cast<double>(count));
}

sdk::Box3 PickupZoneRuntime::zoneInChassis() const {
    sdk::Box3 z;
    z.pose = poseInChassis_;
    z.sizeM = config_.sizeM;
    return z;
}

ShooterZoneRuntime::ShooterZoneRuntime(sdk::ZoneConfig config)
    : FuelZoneRuntime(std::move(config)), params_(parseShooterParams(config_.properties)) {}

void ShooterZoneRuntime::update(sdk::SimContext& ctx, const sdk::Pose3& poseInChassis, bool activationHeld, double dt) {
    poseInChassis_ = poseInChassis;
    active_ = activationHeld;
    sdk::IMechanismDirectory* dir = ctx.mechanisms();

    bool referencesValid = true;
    auto read = [&](const std::string& mechanism, const std::string& value) {
        const sdk::MechanismReading r = sdk::readMechanismValue(dir, mechanism, value);
        if (!mechanism.empty() && !r.found) {
            referencesValid = false;
            warnMissing(ctx, mechanism);
        }
        return r;
    };

    if (params_.fixedExitSpeedMps > 0.0) {
        exitSpeedMps_ = params_.fixedExitSpeedMps;
    } else if (!params_.flywheelMechanism.empty()) {
        const sdk::MechanismReading fly = read(params_.flywheelMechanism, params_.flywheelValue);
        exitSpeedMps_ = fly.hasValue ? std::fabs(fly.value) * params_.flywheelRadiusM * params_.efficiency : 0.0;
    } else {
        exitSpeedMps_ = 0.0;
    }

    elevationRad_ = params_.hoodAngleRad;
    if (!params_.hoodMechanism.empty()) {
        const sdk::MechanismReading hood = read(params_.hoodMechanism, params_.hoodValue);
        if (hood.hasValue) elevationRad_ += params_.hoodGain * hood.value;
    }
    elevationRad_ = std::clamp(elevationRad_, -kMaxElevationRad, kMaxElevationRad);

    yawRad_ = params_.turretYawOffsetRad;
    if (!params_.turretMechanism.empty()) {
        const sdk::MechanismReading turret = read(params_.turretMechanism, params_.turretValue);
        if (turret.hasValue) yawRad_ += params_.turretGain * turret.value;
    }

    feeding_ = referencesValid && activationHeld && exitSpeedMps_ >= params_.minExitSpeedMps && exitSpeedMps_ > 0.0;

    // Token bucket: primed while idle so the first shot leaves as soon as the shooter starts firing.
    if (!feeding_) budget_ = 1.0;
    else budget_ = std::min(2.0, budget_ + params_.fireRateHz * dt);
}

int ShooterZoneRuntime::shotBudget() const { return feeding_ ? static_cast<int>(std::floor(budget_)) : 0; }

void ShooterZoneRuntime::consumeShots(int count) { budget_ = std::max(0.0, budget_ - static_cast<double>(count)); }

ShotSolution ShooterZoneRuntime::solution() const {
    ShotSolution s;
    const double ce = std::cos(elevationRad_);
    const sdk::Vec3 local{static_cast<float>(ce * std::cos(yawRad_)), static_cast<float>(ce * std::sin(yawRad_)),
                          static_cast<float>(std::sin(elevationRad_))};
    s.directionInChassis = sdk::normalized(poseInChassis_.rotation.rotate(local));
    s.muzzleInChassis = poseInChassis_.position;
    s.exitSpeedMps = exitSpeedMps_;
    s.speedJitter = params_.exitSpeedJitter;
    s.aimJitterRad = params_.aimJitterRad;
    s.inheritRobotVelocity = params_.inheritRobotVelocity;
    return s;
}

}  // namespace wpiemu::plugins::game_2026
