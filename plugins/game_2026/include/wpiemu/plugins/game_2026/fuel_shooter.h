#pragma once

// "Fuel Shooter" (game_2026.fuel_shooter): binds a muzzle on the robot to the robot's REAL shooter
// mechanisms. Like the intake it has no slots; it reads
//   * the flywheel's angular speed  -> exit speed = omega * flywheelRadius * efficiency,
//   * the feed mechanism (indexer/kicker) -> "feeding" when its speed exceeds a threshold,
//   * optional hood / turret mechanisms -> launch elevation and yaw
// and publishes everything as named values. The game state spawns real fuel bodies from the muzzle
// while feeding() holds, at fireRateHz, as long as the fuel meter is not empty. Attach this mechanism
// to a turret LINK instead of (or in addition to) naming a turret mechanism if the muzzle turns with it.

#include <memory>
#include <string>
#include <vector>

#include "wpiemu/plugins/game_2026/fuel_ports.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::plugins::game_2026 {

struct ShooterParams {
    sdk::Vec3 muzzleOffset{0.25f, 0.f, 0.55f};  // mechanism root frame
    std::string flywheelMechanism;
    std::string flywheelValue = "velocity";
    double flywheelRadiusM = 0.0508;
    double efficiency = 0.6;
    double fixedExitSpeedMps = 0.0;             // > 0: ignore the flywheel and use this speed
    double minExitSpeedMps = 2.0;
    std::string hoodMechanism;
    std::string hoodValue = "position";
    double hoodGain = 1.0;
    double hoodAngleRad = 1.0471975512;         // elevation above the muzzle X axis (offset when a hood is named)
    std::string turretMechanism;
    std::string turretValue = "position";
    double turretGain = 1.0;
    double turretYawOffsetRad = 0.0;
    std::string feedMechanism;                  // empty = fire whenever the flywheel is up to speed
    std::string feedValue = "velocity";
    double feedThreshold = 5.0;                 // |value| in SI
    double fireRateHz = 10.0;
    double exitSpeedJitter = 0.0;
    double aimJitterRad = 0.0;
    bool inheritRobotVelocity = true;
};

ShooterParams parseShooterParams(const sdk::Value& properties);
sdk::MechanismTemplateInfo fuelShooterTemplate();

class FuelShooterMechanism final : public sdk::IMechanism, public IFuelShooterPort {
public:
    explicit FuelShooterMechanism(const sdk::MechanismBuildContext& ctx);

    // IMechanism
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return kShooterType; }
    void tick(sdk::SimContext& ctx, double dtSeconds) override;
    void reset() override;
    sdk::MechanismState state() const override;

    // IFuelShooterPort
    bool feeding() const override { return feeding_; }
    int shotBudget() const override;
    void consumeShots(int count) override;
    ShotSolution solution() const override;

    const ShooterParams& params() const { return params_; }

private:
    void warnMissing(sdk::SimContext& ctx, const std::string& mechanism);

    std::string id_;
    ShooterParams params_;
    sdk::Pose3 mountFallback_;
    sdk::Pose3 rootInChassis_;
    double exitSpeedMps_ = 0.0;
    double elevationRad_ = 0.0;
    double yawRad_ = 0.0;
    bool feeding_ = false;
    double budget_ = 1.0;
    std::vector<std::string> warned_;
};

std::unique_ptr<sdk::IMechanism> createFuelShooter(const sdk::MechanismBuildContext& ctx);

}  // namespace wpiemu::plugins::game_2026
