#pragma once

// The game client's disposable physics world: the static field of the game plus the player's OWN robot, built exactly like the
// host builds a remote robot (physics::ChassisSimulator through runtime::PhysicsRemoteChassis, driven by the same
// physics::WheelSetpointController), so the same wheel setpoints move it the same way. The other robots appear as KINEMATIC
// boxes that the client positions every tick (dead-reckoned from their newest snapshot), so the own robot pushes against them
// instead of driving through; the host's own resolution of the contact later corrects what this world gets wrong. Fuel is not
// in this world.
//
// Threading: sim-thread style, one caller at a time (the MatchClient's update thread).

#include <cstdint>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/core/runtime/remote_robot.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/config/start_position.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/robot_appearance.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/physics_world.h"
#include "wpiemu/sdk/physics/tire_model.h"

namespace wpiemu::core::match {

// The part of the own robot's state a snapshot can reproduce and the prediction restores: the body and the wheels' spin and
// steer.
struct OwnBodyState {
    sdk::Pose3 pose;
    sdk::Vec3 linearVelocity;
    sdk::Vec3 angularVelocity;
    std::vector<sdk::WheelKinematics> wheels;
};

struct PredictionWorldArgs {
    const sdk::RobotConfig& spec;                      // the ACCEPTED spec (acceptRemoteSpec), copied
    const sdk::GamePluginRegistry* plugins = nullptr;  // the field comes from the game module named by spec.gamePluginId
    const sdk::DrivetrainRegistry& drivetrains;
    const sdk::TireModelRegistry& tireModels;
    sdk::PhysicsWorldFactory worldFactory;             // empty: the Box3D world
    sdk::RobotStartPose placement;                     // where the host placed the robot (JoinAccept::startPose)
    std::uint64_t ownTag = 0;                          // stamped on the own robot's shapes
    int settleTicks = 60;                              // zero-input ticks that let the robot come to rest on the floor
};

class PredictionWorld {
public:
    static sdk::Result<std::unique_ptr<PredictionWorld>> create(const PredictionWorldArgs& args);
    ~PredictionWorld();
    PredictionWorld(const PredictionWorld&) = delete;
    PredictionWorld& operator=(const PredictionWorld&) = delete;

    std::size_t wheelCount() const;
    const sdk::FieldDescription& field() const;
    sdk::IPhysicsWorld& world() { return *world_; }

    OwnBodyState capture() const;
    // Puts the body and the wheels back (wheel counts must match). The velocities are set last, so the drivetrain re-bases on
    // them.
    sdk::Status restore(const OwnBodyState& state);

    // The setpoints the controller follows until the next call (one per wheel, spec order); brake() holds the steer angles
    // and commands zero speed.
    void setSetpoints(std::span<const sdk::WheelSetpoint> setpoints);
    void brake();
    // One fixed step: controller voltages (0 V when !enabled), tire forces, the world step, completion.
    void step(bool enabled, float dtSeconds);

    // The other robots. `look` sizes the kinematic hull box; the pose is the body origin (centre of mass) at the start of the
    // coming step and the velocities carry it over the step.
    void setRemote(sdk::match::ParticipantId id, const sdk::match::RobotAppearance& look, const sdk::Pose3& pose,
                   const sdk::Vec3& linearVelocity, float yawRateRadPerSec);
    void removeRemote(sdk::match::ParticipantId id);
    void clearRemotes();
    std::size_t remoteCount() const { return remotes_.size(); }

    std::uint64_t stateHash() const;

private:
    struct RemoteBody {
        sdk::BodyId body;
        sdk::match::RobotAppearance look;
    };

    PredictionWorld() = default;
    void destroyRemote(RemoteBody& body);

    std::unique_ptr<sdk::IPhysicsWorld> world_;
    std::unique_ptr<sdk::IField> field_;
    std::unique_ptr<runtime::IRemoteChassis> chassis_;
    std::map<sdk::match::ParticipantId, RemoteBody> remotes_;
};

}  // namespace wpiemu::core::match
