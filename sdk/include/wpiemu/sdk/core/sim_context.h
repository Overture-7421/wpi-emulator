#pragma once

// The one object passed everywhere instead of globals/singletons. Created by the sim runtime
// (core) per session and handed to mechanisms, drivetrains' callers, game state and game module hooks.
//
// SimContext does NOT own anything: it is a bundle of references to services owned by the
// runtime. It is valid for one session; do not store it beyond the objects it was given to
// (those objects are destroyed before the services). Not thread-safe: SIM THREAD only.

#include <cstdint>

#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/rng.h"
#include "wpiemu/sdk/mechanisms/physics_coupling.h"

namespace wpiemu::sdk {

namespace proto {
struct DsState;
}

class IPhysicsWorld;
class IDeviceHub;
class IGameState;
class IMechanismDirectory;
struct RobotConfig;

struct SimContextServices {
    IPhysicsWorld& physics;
    const ISimClock& clock;
    IDeviceHub& devices;
    ILogger& logger;
    const RobotConfig& config;
    Rng& rng;
    const proto::DsState& driverStation;  // latest DS state (enabled/mode/alliance/match time)
    BodyId chassisBody;                   // the robot's rigid body in `physics` (invalid before it is built)
    IMechanismDirectory* mechanisms = nullptr;  // null until mechanisms are built
    IMechanismPhysicsCoupling* coupling = nullptr;  // SDK 1.1; null => SimContext::coupling() is a null object
};

class SimContext {
public:
    explicit SimContext(const SimContextServices& services) : s_(services) {}

    SimContext(const SimContext&) = delete;
    SimContext& operator=(const SimContext&) = delete;

    IPhysicsWorld& physics() { return s_.physics; }
    const ISimClock& clock() const { return s_.clock; }
    IDeviceHub& devices() { return s_.devices; }
    ILogger& logger() { return s_.logger; }
    // Immutable during a session; plugin sections via config().findPlugin(id).
    const RobotConfig& config() const { return s_.config; }
    Rng& rng() { return s_.rng; }
    const proto::DsState& driverStation() const { return s_.driverStation; }
    // Robot pose = physics().pose(chassisBody()).
    BodyId chassisBody() const { return s_.chassisBody; }

    // Null until the runtime has built them (e.g. during IGameModule::createGameState).
    IGameState* gameState() { return gameState_; }
    IMechanismDirectory* mechanisms() { return s_.mechanisms; }
    // SDK 1.1. Never null: a no-op null object (capabilities() all false) when the runtime did not bind
    // one. Mechanisms submit before the physics step and read prior-step loads (see physics_coupling.h).
    IMechanismPhysicsCoupling& coupling() { return s_.coupling != nullptr ? *s_.coupling : nullCoupling_; }

    // Runtime only: bind once the game state exists / mechanisms are built.
    void bindGameState(IGameState* state) { gameState_ = state; }
    void bindMechanisms(IMechanismDirectory* directory) { s_.mechanisms = directory; }
    void bindCoupling(IMechanismPhysicsCoupling* coupling) { s_.coupling = coupling; }
    void bindChassisBody(BodyId body) { s_.chassisBody = body; }

private:
    SimContextServices s_;
    IGameState* gameState_ = nullptr;
    NullMechanismPhysicsCoupling nullCoupling_;
};

}  // namespace wpiemu::sdk
