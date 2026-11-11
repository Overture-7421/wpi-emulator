#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::sdk {

class SimContext;
class SnapshotBuilder;
struct RobotView;

// Game entities and rules (fuel, hubs, scoring, active/inactive state...) for the loaded game
// plugin. Created by IGameModule::createGameState AFTER the field exists, owned by the sim
// runtime, destroyed before the physics world.
//
// Threading: sim thread only. The UI sees results through publish() -> ISimSnapshot.
// Mechanisms provided by the same plugin may reach their game state with a plugin-private
// downcast of SimContext::gameState() (the plugin owns both sides of that contract).
class IGameState {
    WPIEMU_INTERFACE(IGameState)
public:
    virtual std::string_view pluginId() const = 0;
    // Restore the initial layout (match start). Called on Reset and before the first tick.
    virtual void reset(SimContext& ctx) = 0;
    // Called once per sim tick AFTER IPhysicsWorld::step (sensor/contact events are fresh).
    virtual void step(SimContext& ctx, double dtSeconds) = 0;
    // Contribute entities (EntityGroupSample) and HUD values (setHud) to the snapshot.
    virtual void publish(SnapshotBuilder& out) const = 0;
    // FMS "game data" the real field sends to the robot (DriverStation.getGameSpecificMessage()), if the game has
    // produced one yet; nullopt = nothing to send. The runtime forwards it in DsState::gameSpecificMessage and
    // re-reads it after every tick and Reset. Additive: defaults to "none".
    virtual std::optional<std::string> gameSpecificMessage() const { return std::nullopt; }
    // Live value of a game value declared by IGameModule::contributeGameValues(); nullopt = not published.
    // Read by core's sensors every tick after step().
    virtual std::optional<double> gameValue(std::string_view /*id*/) const { return std::nullopt; }

    // ---- several robots (practice matches; all additive, default no-ops) ------------------------------------------
    // Threading and call order. All three run on the SIM THREAD, never concurrently with step(). Robots are added and
    // removed BETWEEN ticks, so a robot never appears or disappears inside step() or publish():
    //   - onRobotAdded: once for every robot that joins, after it exists in ctx.robots() (the host robot too, if the
    //     session lists it, after reset() on the initial roster); a robot already present when the game state is
    //     created is announced right after createGameState and before the first reset().
    //   - onRobotRemoved: once when a robot leaves, AFTER it was dropped from ctx.robots() (it is gone; only its id is
    //     left). The game state releases its per-robot state, e.g. returns held items to the field.
    //   - step() still runs exactly once per tick, after IPhysicsWorld::step, and reads ctx.robots() for the current
    //     roster, body poses, zone flags and enabled state.
    //   - publishRobot runs from the runtime's publish pass for every REMOTE robot (RobotView::isHost == false) of
    //     ctx.robots(), right after publish(), once the runtime has added that robot's RobotSample (pose, wheels, zones).
    //     Use it for per-robot HUD values and inventory with SnapshotBuilder::setRobotHud (telemetry/snapshot.h). The
    //     host robot's own values go through publish()/setHud as before.
    // reset() applies to the whole game: per-robot state is cleared but robots stay on the roster.
    virtual void onRobotAdded(SimContext& /*ctx*/, const RobotView& /*robot*/) {}
    virtual void onRobotRemoved(SimContext& /*ctx*/, match::ParticipantId /*participant*/) {}
    virtual void publishRobot(SnapshotBuilder& /*out*/, const RobotView& /*robot*/) const {}
};

}  // namespace wpiemu::sdk
