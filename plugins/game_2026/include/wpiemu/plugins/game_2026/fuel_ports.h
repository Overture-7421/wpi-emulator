#pragma once

// Plugin-private contract between the 2026 zone runtimes (fuel_zones.h: pickup / shooter) and the game
// state: the zones decide WHEN (active / feeding) and WHERE (volume / muzzle) from the robot's
// mechanisms; the game state owns the fuel and the meter and acts on it.

#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::plugins::game_2026 {

class IFuelIntakePort {
public:
    virtual ~IFuelIntakePort() = default;
    // Roller/deploy condition currently satisfied.
    virtual bool pickupActive() const = 0;
    // Fuel this intake may absorb right now (rate limited); consumePickups() spends it.
    virtual int pickupBudget() const = 0;
    virtual void consumePickups(int count) = 0;
    // Pickup volume in the CHASSIS frame (pose + full extents).
    virtual sdk::Box3 zoneInChassis() const = 0;
};

struct ShotSolution {
    sdk::Vec3 muzzleInChassis;
    sdk::Vec3 directionInChassis;  // unit
    double exitSpeedMps = 0.0;
    double speedJitter = 0.0;      // +- fraction of exitSpeedMps
    double aimJitterRad = 0.0;     // +- per axis
    bool inheritRobotVelocity = true;
};

class IFuelShooterPort {
public:
    virtual ~IFuelShooterPort() = default;
    // Feed condition satisfied and the flywheel is fast enough to launch.
    virtual bool feeding() const = 0;
    virtual int shotBudget() const = 0;
    virtual void consumeShots(int count) = 0;
    virtual ShotSolution solution() const = 0;
};

}  // namespace wpiemu::plugins::game_2026
