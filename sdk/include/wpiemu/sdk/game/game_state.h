#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "wpiemu/sdk/core/interface.h"

namespace wpiemu::sdk {

class SimContext;
class SnapshotBuilder;

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
};

}  // namespace wpiemu::sdk
