#pragma once

// The two zone kinds this plugin owns (RobotConfig::zones, see plugins/game_2026/README.md) and their
// per-session runtimes:
//   "game_2026.pickup"  a volume on the robot that absorbs fuel into the meter while its activation holds
//   "game_2026.shooter" a muzzle that launches fuel bodies while its activation holds and the flywheel is up to speed
// "When" is the zone's ZoneActivation (conditions on other mechanisms' published values, evaluated by the game
// state with sdk::evaluateActivation); "where" is its resolved pose in the chassis frame. The runtimes only add
// what is specific to fuel: the pickup rate limit, the shooter's exit speed / aim from its flywheel, hood and
// turret references, and the shot budget. The game state owns the fuel and the meter and acts through the ports.

#include <string>
#include <utility>
#include <vector>

#include "wpiemu/plugins/game_2026/fuel_ports.h"
#include "wpiemu/sdk/config/zone.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/zone_kind.h"

namespace wpiemu::plugins::game_2026 {

inline constexpr const char* kPickupKind = "game_2026.pickup";
inline constexpr const char* kShooterKind = "game_2026.shooter";

struct PickupParams {
    double intakeRateHz = 15.0;  // fuel per second; 0 = unlimited
};

struct ShooterParams {
    std::string flywheelMechanism;
    std::string flywheelValue = sdk::value_name::kVelocity;
    double flywheelRadiusM = 0.0508;
    double efficiency = 0.6;
    double fixedExitSpeedMps = 0.0;  // > 0: ignore the flywheel and use this speed
    double minExitSpeedMps = 2.0;
    std::string hoodMechanism;
    std::string hoodValue = sdk::value_name::kPosition;
    double hoodGain = 1.0;
    double hoodAngleRad = 1.0471975512;  // elevation above the muzzle X axis (offset when a hood is named)
    std::string turretMechanism;
    std::string turretValue = sdk::value_name::kPosition;
    double turretGain = 1.0;
    double turretYawOffsetRad = 0.0;
    double fireRateHz = 10.0;
    int parallelBalls = 1;
    double exitSpeedJitter = 0.0;
    double aimJitterRad = 0.0;
    bool inheritRobotVelocity = true;
};

// Defaults are applied and out-of-range or non-finite values are clamped / replaced, so a hand-edited
// properties object never produces an unusable runtime.
PickupParams parsePickupParams(const sdk::Value& properties);
ShooterParams parseShooterParams(const sdk::Value& properties);

sdk::ZoneKindInfo pickupZoneKind();
sdk::ZoneKindInfo shooterZoneKind();
std::vector<sdk::ZoneKindInfo> zoneKinds();

// Per-session state of one owned zone.
class FuelZoneRuntime {
public:
    explicit FuelZoneRuntime(sdk::ZoneConfig config) : config_(std::move(config)) {}
    virtual ~FuelZoneRuntime() = default;

    const sdk::ZoneConfig& config() const { return config_; }
    const std::string& id() const { return config_.id; }
    // The zone's activation currently holds (what the snapshot publishes).
    bool active() const { return active_; }

    // One warning per unknown mechanism id for the lifetime of the zone.
    void warnMissing(sdk::SimContext& ctx, const std::string& mechanism);
    // `poseInChassis` = resolved attachment pose * ZoneConfig::local; `activationHeld` from evaluateActivation.
    virtual void update(sdk::SimContext& ctx, const sdk::Pose3& poseInChassis, bool activationHeld, double dtSeconds) = 0;

protected:
    sdk::ZoneConfig config_;
    sdk::Pose3 poseInChassis_;
    bool active_ = false;

private:
    std::vector<std::string> warned_;
};

class PickupZoneRuntime final : public FuelZoneRuntime, public IFuelIntakePort {
public:
    explicit PickupZoneRuntime(sdk::ZoneConfig config);

    void update(sdk::SimContext& ctx, const sdk::Pose3& poseInChassis, bool activationHeld, double dtSeconds) override;

    // IFuelIntakePort
    bool pickupActive() const override { return active_; }
    int pickupBudget() const override;
    void consumePickups(int count) override;
    sdk::Box3 zoneInChassis() const override;

    const PickupParams& params() const { return params_; }

private:
    PickupParams params_;
    double budget_ = 1.0;
};

class ShooterZoneRuntime final : public FuelZoneRuntime, public IFuelShooterPort {
public:
    explicit ShooterZoneRuntime(sdk::ZoneConfig config);

    void update(sdk::SimContext& ctx, const sdk::Pose3& poseInChassis, bool activationHeld, double dtSeconds) override;

    // IFuelShooterPort
    bool feeding() const override { return feeding_; }
    int shotBudget() const override;
    void consumeShots(int count) override;
    ShotSolution solution() const override;

    const ShooterParams& params() const { return params_; }
    double exitSpeedMps() const { return exitSpeedMps_; }
    double elevationRad() const { return elevationRad_; }
    double yawRad() const { return yawRad_; }

private:
    ShooterParams params_;
    double exitSpeedMps_ = 0.0;
    double elevationRad_ = 0.0;
    double yawRad_ = 0.0;
    bool feeding_ = false;
    double budget_ = 1.0;
};

}  // namespace wpiemu::plugins::game_2026
