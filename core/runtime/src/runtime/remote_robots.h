#pragma once

// The remote robots of one SimulationSession and the IRobotDirectory the game sees: the host robot first, then the
// remote robots ordered by participant id (ids are assigned in join order by the match host, so this is join order).
// Internal to core/runtime: SimulationSession owns exactly one and is the only caller. SIM THREAD only.

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/core/runtime/remote_robot.h"
#include "wpiemu/sdk/game/game_state.h"
#include "wpiemu/sdk/game/robot_view.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/tire_model.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::core::runtime {

class RemoteRobots final : public sdk::IRobotDirectory {
public:
    // The host's own robot as the directory describes it; refreshed by the session before every tick and roster change.
    struct Host {
        std::string name;
        sdk::BodyId body;
        sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
        bool enabled = false;
    };

    // Non-owning: the world, registries and factory outlive this object.
    RemoteRobots(sdk::IPhysicsWorld& world, RemoteChassisFactory factory, const sdk::DrivetrainRegistry* drivetrains,
                 const sdk::TireModelRegistry* tireModels);
    ~RemoteRobots() override;
    RemoteRobots(const RemoteRobots&) = delete;
    RemoteRobots& operator=(const RemoteRobots&) = delete;

    // ---- roster -----------------------------------------------------------------------------------------------
    // Everything add() checks, without side effects.
    sdk::Status validate(const RemoteRobotRequest& request) const;
    // `placement` is where the robot starts and where reset() puts it; `nowUs` starts its input timeout.
    sdk::Status add(const RemoteRobotRequest& request, const sdk::RobotStartPose& placement, sdk::SimTimeUs nowUs);
    bool remove(sdk::match::ParticipantId id);
    std::vector<sdk::match::ParticipantId> ids() const;
    std::optional<RemoteRobotInfo> info(sdk::match::ParticipantId id) const;
    // The footprints of the robots present, for placing the next one clear of them.
    std::vector<StartDisc> footprints() const;

    // ---- inputs -----------------------------------------------------------------------------------------------
    // `tickNow` is the tick that will run next (the arrival tick); `stepUs` bounds how far ahead an input may target.
    sdk::Result<RemoteInputOutcome> submit(sdk::match::ParticipantId id, const sdk::match::MatchInput& input,
                                           std::uint64_t tickNow, sdk::SimTimeUs stepUs);

    // ---- tick -------------------------------------------------------------------------------------------------
    // Applies the inputs due on `tick`, then drives every robot's controller and drivetrain (tire forces) for the
    // coming world step. `nowUs` is the sim time at the start of the tick, `enabled` the shared enable gate.
    void beforeStep(std::uint64_t tick, sdk::SimTimeUs nowUs, bool enabled, float dt);
    void afterStep(float dt);
    // Every robot back to its placement at rest; roster kept, queues and counters cleared.
    void reset();

    // ---- directory and snapshot ---------------------------------------------------------------------------------
    void setHost(Host host);
    // The host robot's per-tick state (Driver Station alliance station and the shared enable gate).
    void setHostState(sdk::proto::AllianceStation station, bool enabled);
    std::span<const sdk::RobotView> robots() const override { return views_; }
    // One RobotSample per remote robot (pose, velocities, wheels, zones); the per-robot HUD comes from publishHud.
    void addSamples(sdk::SnapshotBuilder& out) const;
    void publishHud(const sdk::IGameState& state, sdk::SnapshotBuilder& out) const;

private:
    struct Queued {
        std::uint32_t seq = 0;
        std::uint64_t targetTick = 0;
        std::uint32_t zoneFlags = 0;
        std::vector<sdk::WheelSetpoint> setpoints;
    };
    struct Robot {
        sdk::match::ParticipantId id = sdk::match::kNoParticipant;
        std::string name;
        sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
        sdk::RobotConfig spec;
        sdk::RobotStartPose placement;
        float footprintRadiusM = 0.f;
        std::uint32_t zoneMask = 0;
        std::vector<float> maxWheelSpeed;
        std::unique_ptr<IRemoteChassis> chassis;   // destroyed before the rest (it removes its body from the world)
        std::vector<Queued> queue;                 // ordered by sequence (serial-number order), oldest first
        RemoteInputStats stats;
        bool anyReceived = false;
        std::uint32_t newestReceivedSeq = 0;
        std::uint32_t zoneFlags = 0;               // of the input applied last, masked
        std::uint32_t activeZoneFlags = 0;         // what the game sees
        sdk::SimTimeUs lastInputUs = 0;
        bool enabled = false;
    };

    Robot* lookup(sdk::match::ParticipantId id);
    const Robot* lookup(sdk::match::ParticipantId id) const;
    void rebuildViews();
    void refreshViews();
    static sdk::RobotView viewOf(const Robot& robot);

    sdk::IPhysicsWorld& world_;
    RemoteChassisFactory factory_;
    const sdk::DrivetrainRegistry* drivetrains_;
    const sdk::TireModelRegistry* tireModels_;
    Host host_;
    std::vector<std::unique_ptr<Robot>> robots_;   // ordered by participant id
    std::vector<sdk::RobotView> views_;            // host first
};

}  // namespace wpiemu::core::runtime
