#pragma once

// "Fuel Intake" (game_2026.fuel_intake): binds a pickup volume on the robot to the robot's REAL
// intake mechanisms. It has no slots of its own: "is the intake active?" is derived from other
// mechanisms (a roller's speed, a deploy arm's position) read through the mechanism directory, so
// whatever generic roller/arm/elevator templates the user built drive the game logic. The game state
// absorbs fuel inside `pickupZone` while pickupActive() holds and the meter is not full.

#include <memory>
#include <string>
#include <vector>

#include "wpiemu/plugins/game_2026/fuel_ports.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::plugins::game_2026 {

enum class IntakeActivation { Roller, Deployed, RollerAndDeployed, RollerOrDeployed, Always };

struct IntakeParams {
    sdk::Box3 pickupZone;                   // in the mechanism root frame
    IntakeActivation activation = IntakeActivation::Roller;
    std::string rollerMechanism;            // empty = the roller condition is always satisfied
    std::string rollerValue = "velocity";
    double rollerThreshold = 20.0;          // |value| in SI (rad/s for a roller)
    std::string deployMechanism;            // empty = the deploy condition is always satisfied
    std::string deployValue = "position";
    double deployedPosition = 0.0;          // SI
    double deployedTolerance = 0.1;
    double intakeRateHz = 15.0;             // fuel per second per intake; 0 = unlimited
};

IntakeParams parseIntakeParams(const sdk::Value& properties);
sdk::MechanismTemplateInfo fuelIntakeTemplate();

class FuelIntakeMechanism final : public sdk::IMechanism, public IFuelIntakePort {
public:
    explicit FuelIntakeMechanism(const sdk::MechanismBuildContext& ctx);

    // IMechanism
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return kIntakeType; }
    void tick(sdk::SimContext& ctx, double dtSeconds) override;
    void reset() override;
    sdk::MechanismState state() const override;

    // IFuelIntakePort
    bool pickupActive() const override { return active_; }
    int pickupBudget() const override;
    void consumePickups(int count) override;
    sdk::Box3 zoneInChassis() const override;

    const IntakeParams& params() const { return params_; }

private:
    bool rollerSatisfied(sdk::SimContext& ctx);
    bool deploySatisfied(sdk::SimContext& ctx);
    void warnMissing(sdk::SimContext& ctx, const std::string& mechanism);

    std::string id_;
    IntakeParams params_;
    sdk::Pose3 mountFallback_;
    sdk::Pose3 rootInChassis_;
    bool active_ = false;
    double budget_ = 1.0;
    std::vector<std::string> warned_;
};

std::unique_ptr<sdk::IMechanism> createFuelIntake(const sdk::MechanismBuildContext& ctx);

}  // namespace wpiemu::plugins::game_2026
