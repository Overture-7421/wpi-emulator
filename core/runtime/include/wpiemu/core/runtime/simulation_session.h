#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/core/runtime/device_hub.h"
#include "wpiemu/core/runtime/sim_clock.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/rng.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/game_state.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/physics_world.h"
#include "wpiemu/sdk/physics/tire_model.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::core::runtime {

// ---- seams the composition root can replace -----------------------------------------------------------------

// The runtime-side half of IMechanismPhysicsCoupling (ARCHITECTURE 4.2): `service()` is what mechanisms see via
// SimContext::coupling(); the runtime calls beforeStep/afterStep around IPhysicsWorld::step. The physics
// module owns the real implementation; tests adapt sdk::testing::FakeMechanismPhysicsCoupling.
class ICouplingDriver {
    WPIEMU_INTERFACE(ICouplingDriver)
public:
    // Called once by the session right after the chassis body exists (before any mechanism is built).
    virtual void bind(sdk::IPhysicsWorld& world, sdk::BodyId chassis, const sdk::MassProperties& baseMass) = 0;
    // Called once after the drivetrain was created (null when the config has no wheels): lets an implementation keep it
    // informed about the composed chassis mass.
    virtual void attachDrivetrain(sdk::IDrivetrain* /*drivetrain*/) {}
    // What mechanisms see through SimContext::coupling(); valid from bind() on, for the lifetime of the driver.
    virtual sdk::IMechanismPhysicsCoupling& service() = 0;
    // Compose payloads into the chassis mass (base = chassis alone), apply submitted wrenches, (re)create constraints.
    virtual void beforeStep(sdk::IPhysicsWorld& world, sdk::BodyId chassis, const sdk::MassProperties& baseMass) = 0;
    // The step completed: constraint loads become readable, one-step wrenches are dropped.
    virtual void afterStep() = 0;
    // Session reset: drop every constraint and payload.
    virtual void reset() = 0;
};

struct ChassisInfo {
    sdk::BodyId body;
    sdk::MassProperties baseMass;   // chassis alone, before payload composition
    sdk::Pose3 spawnPose;           // where reset() puts the chassis
};
using ChassisBuilder = std::function<sdk::Result<ChassisInfo>(sdk::IPhysicsWorld&, const sdk::ChassisSpec&, const sdk::DrivetrainSpec&)>;

// Reference builder used when none is injected (ADR 0003 chassis model): one dynamic body whose origin is the COM, a
// hull box, collidable primitives, one frictionless pad sphere per wheel, explicit mass properties.
// WheelSpec::positionM.z is the hub height above the floor (<= 0 means one radius).
sdk::Result<ChassisInfo> buildReferenceChassis(sdk::IPhysicsWorld& world, const sdk::ChassisSpec& chassis, const sdk::DrivetrainSpec& drivetrain);

struct SessionOptions {
    double baselineCurrentA = 2.0;            // roboRIO + radio + PDH idle draw
};

// Everything the builder needs, already constructed by the composition root. Non-owning pointers must outlive
// the session; `world`, `coupling` are moved in.
struct SessionParts {
    sdk::RobotConfig config;                          // defaulted, validated
    std::unique_ptr<sdk::IPhysicsWorld> world;        // required
    const sdk::MechanismFactory* mechanisms = nullptr;      // required (plugin templates already registered)
    const sdk::DrivetrainRegistry* drivetrains = nullptr;   // required when the config has wheels
    const sdk::TireModelRegistry* tireModels = nullptr;     // required when the config has wheels
    const sdk::GamePluginRegistry* plugins = nullptr;       // null => core features only
    sdk::ILogger* logger = nullptr;                   // null => NullLogger
    ChassisBuilder chassisBuilder;                    // empty => buildReferenceChassis
    std::unique_ptr<ICouplingDriver> coupling;        // empty => mechanisms see the SDK null coupling
    SessionOptions options;
};

// One simulation run: owns the world, chassis, drivetrain, mechanisms, device hub, game state and runs the
// per-tick pipeline of ARCHITECTURE section 4. Pure and synchronous: no threads, no I/O, no wall clock, so it is
// deterministic and unit-testable; SessionRunner adds lock-step I/O around it.
// Destruction order (reverse of construction): game state, mechanisms, drivetrain, field, world.
// Threading: sim thread only.
class SimulationSession {
public:
    ~SimulationSession();
    SimulationSession(const SimulationSession&) = delete;
    SimulationSession& operator=(const SimulationSession&) = delete;

    // One tick: mechanisms -> drivetrain -> physics -> game -> egress (IMU, battery) -> SensorFrame.
    // The caller has already ingested the previous ActuatorFrame into hub() (ingress) and set the DS state.
    // Fills tick (0-based within the epoch), simTimeUs (after the step), stepUs, battery, samples; the runner stamps identity.
    void tick(sdk::proto::SensorFrame& out, bool fullFrame);
    // Back to t = 0: chassis pose/velocity, drivetrain, mechanisms, game state, clock, RNG, battery. Bindings and
    // discovered devices stay, but cached actuation/feedback is neutralized. Starts a new epoch of tick
    // numbering; the next sensor frame is full even if the caller requests a delta.
    void reset();

    void setDriverStation(const sdk::proto::DsState& ds) { ds_ = ds; }
    const sdk::proto::DsState& driverStation() const { return ds_; }
    // FMS game data the loaded game plugin currently wants the robot to see (nullopt: none). Overlaid on
    // DsState::gameSpecificMessage by SessionRunner.
    std::optional<std::string> gameSpecificMessage() const { return gameState_ ? gameState_->gameSpecificMessage() : std::nullopt; }

    DeviceHub& hub() { return hub_; }
    const DeviceHub& hub() const { return hub_; }
    const sdk::ISimClock& clock() const { return clock_; }
    const BatteryModel& battery() const { return battery_; }
    const sdk::RobotConfig& config() const { return config_; }
    sdk::Status bindSlot(const sdk::SlotAddress& slot, const sdk::DeviceKey& device);
    sdk::IPhysicsWorld& world() { return *world_; }
    sdk::BodyId chassisBody() const { return chassis_.body; }
    sdk::IDrivetrain* drivetrain() { return drivetrain_.get(); }
    sdk::IMechanismDirectory& mechanisms();
    sdk::IGameModule* gameModule() const { return module_; }
    // Warnings the session raises itself (brownout, unresolved attachments), next to hub().warnings().
    const std::vector<sdk::HubWarning>& sessionWarnings() const { return sessionWarnings_; }

    // Immutable view for the UI (what the runner hands to the snapshot sink).
    std::shared_ptr<const sdk::ISimSnapshot> buildSnapshot(bool paused, bool robotConnected,
                                                           std::span<const sdk::HubWarning> extraWarnings = {}) const;

private:
    friend class SessionBuilder;
    class Directory;
    SimulationSession(SessionParts& parts, sdk::SimTimeUs stepUs);

    sdk::Status build(SessionParts& parts);
    void stepDrivetrain(double dt, double& currentA);
    void writeImu(double dt);
    void updateBrownoutWarning();

    sdk::RobotConfig config_;
    SessionOptions options_;
    sdk::NullLogger nullLogger_;
    sdk::ILogger* logger_;
    SimClock clock_;
    sdk::Rng rng_;
    DeviceHub hub_;
    BatteryModel battery_;
    sdk::proto::DsState ds_;
    std::vector<sdk::HubWarning> sessionWarnings_;

    // construction order == reverse destruction order
    std::unique_ptr<sdk::IPhysicsWorld> world_;
    std::unique_ptr<ICouplingDriver> coupling_;
    sdk::IGameModule* module_ = nullptr;
    std::unique_ptr<sdk::IField> field_;
    ChassisInfo chassis_;
    std::unique_ptr<sdk::ITireModel> tire_;
    std::unique_ptr<sdk::IDrivetrain> drivetrain_;
    std::vector<std::unique_ptr<sdk::IMechanism>> mechanismInstances_;
    std::unique_ptr<Directory> directory_;
    std::unique_ptr<sdk::SimContext> ctx_;
    std::unique_ptr<sdk::IGameState> gameState_;

    std::vector<sdk::WheelCommand> wheelCommands_;
    sdk::Vec3 lastVelocity_;
    bool brownoutWarned_ = false;
    const sdk::MechanismFactory* factory_ = nullptr;
    std::vector<sdk::MotionAxisBinding> motionAxes_;
    std::vector<sdk::PlantState> motionStarts_;
    std::vector<std::optional<std::size_t>> motionSources_;
    std::vector<double> motionRatios_;
    std::vector<std::size_t> motionOrder_;
    std::vector<bool> motionMechanisms_;
    bool motionDrivetrain_ = false;
};

class SessionBuilder {
public:
    // Builds the world content (field, chassis, drivetrain, mechanisms, game state), binds the persisted slot
    // mappings and returns a session ready to tick. Errors: missing collaborators, unknown drivetrain/tire
    // model/mechanism type, plugin failures (all reported by value; nothing is left half built).
    static sdk::Result<std::unique_ptr<SimulationSession>> build(SessionParts parts);
};

}  // namespace wpiemu::core::runtime
