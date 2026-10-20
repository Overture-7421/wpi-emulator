#pragma once

// Game 2026 rules and entities: fuel bodies, the robot's fuel meter, hub scoring with the active /
// inactive schedule, hub recycling, outposts (corral intake, timed human-player chute) and the HUD
// values. Owns everything per session; the module that creates it stays stateless.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/plugins/game_2026/fuel_pool.h"
#include "wpiemu/plugins/game_2026/fuel_ports.h"
#include "wpiemu/plugins/game_2026/fuel_zones.h"
#include "wpiemu/plugins/game_2026/hub_rules.h"
#include "wpiemu/plugins/game_2026/match_clock.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/game_state.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::plugins::game_2026 {

namespace hud {
inline constexpr const char* kFuelCount = "game_2026.fuel.count";
inline constexpr const char* kFuelCapacity = "game_2026.fuel.capacity";
inline constexpr const char* kFuelOnField = "game_2026.fuel.on_field";
inline constexpr const char* kShots = "game_2026.fuel.shots";
inline constexpr const char* kScoreRed = "game_2026.score.red";
inline constexpr const char* kScoreBlue = "game_2026.score.blue";
inline constexpr const char* kHubRedActive = "game_2026.hub.red.active";
inline constexpr const char* kHubBlueActive = "game_2026.hub.blue.active";
inline constexpr const char* kHubOwnActive = "game_2026.hub.own.active";
// "Active 0:25" (time left in the current active window) or "Inactive"; drives the hub badges.
inline constexpr const char* kHubRedStatus = "game_2026.hub.red.status";
inline constexpr const char* kHubBlueStatus = "game_2026.hub.blue.status";
inline constexpr const char* kFirstInactive = "game_2026.hub.first_inactive";
inline constexpr const char* kMatchPhase = "game_2026.match.phase";
inline constexpr const char* kMatchTime = "game_2026.match.period_remaining";
inline constexpr const char* kPhaseTime = "game_2026.match.phase_remaining";
inline constexpr const char* kOutpostRed = "game_2026.outpost.red.stock";
inline constexpr const char* kOutpostBlue = "game_2026.outpost.blue.stock";
}  // namespace hud

class Game2026State final : public sdk::IGameState {
public:
    // `ctx.physics()` must outlive the state (the runtime destroys game state before the world).
    Game2026State(sdk::SimContext& ctx, GameConfig config, FieldLayout layout);

    std::string_view pluginId() const override;
    void reset(sdk::SimContext& ctx) override;
    void step(sdk::SimContext& ctx, double dtSeconds) override;
    void publish(sdk::SnapshotBuilder& out) const override;
    // 'R'/'B' = the alliance whose hub goes inactive first (FMS game data), available once TELEOP has started.
    std::optional<std::string> gameSpecificMessage() const override;
    // The values declared by gameValueInfos() (game_values.h); nullopt for any other id.
    std::optional<double> gameValue(std::string_view id) const override;
    // ---- read-only view for HUD, tests and tools ----------------------------------------------
    const GameConfig& config() const { return config_; }
    int fuelInRobot() const { return meter_; }
    int fuelOnField() const { return static_cast<int>(pool_.count()); }
    int shotsFired() const { return shots_; }
    int fuelLost() const { return lost_; }
    int outpostStock(Alliance a) const { return outposts_[index(a)].stock; }
    const AllianceScore& score(Alliance a) const { return hub_.score(a); }
    bool hubActive(Alliance a) const { return hub_.active(a); }
    const MatchStatus& match() const { return status_; }
    const HubRules& hubRules() const { return hub_; }
    // Owned zones in config order (pickups and shooters), for tests and tools.
    const std::vector<std::unique_ptr<FuelZoneRuntime>>& zones() const { return zones_; }
    FuelPool& pool() { return pool_; }
    const FuelPool& pool() const { return pool_; }

private:
    struct PendingRelease {
        Alliance hub;
        double timeLeftS;
    };
    struct OutpostState {
        int stock = 0;
        double timerS = 0.0;
    };
    struct SensorHit {
        EntityKind kind;
        Alliance alliance;
        std::uint32_t fuel;
    };

    void handleSensorEvents(sdk::SimContext& ctx);
    void despawnOutOfPlay(sdk::SimContext& ctx);
    void buildZones(sdk::SimContext& ctx);
    void stepZones(sdk::SimContext& ctx, double dt);
    void stepShooters(sdk::SimContext& ctx);
    void stepIntakes(sdk::SimContext& ctx);
    void stepRecycling(sdk::SimContext& ctx, double dt);
    void stepHumanPlayers(sdk::SimContext& ctx, double dt);
    void refreshPositions();
    sdk::Pose3 chassisPose(sdk::SimContext& ctx) const;

    GameConfig config_;
    FieldLayout layout_;
    FuelPool pool_;
    MatchClock clock_;
    MatchStatus status_;
    HubRules hub_;
    Alliance ownAlliance_ = Alliance::Red;
    bool matchTimed_ = false;   // the DS runs a timed match (Practice); otherwise free play without game timing
    int meter_ = 0;
    int shots_ = 0;
    int lost_ = 0;
    std::vector<PendingRelease> pending_;
    std::array<OutpostState, 2> outposts_{};

    // Runtimes of the zones this plugin owns (RobotConfig::zones); intakes_/shooters_ view into them.
    std::vector<std::unique_ptr<FuelZoneRuntime>> zones_;
    std::vector<IFuelIntakePort*> intakes_;
    std::vector<IFuelShooterPort*> shooters_;
    std::vector<SensorHit> hits_;
    std::vector<std::uint32_t> scratchIds_;
    std::vector<sdk::Vec3> fuelPositions_;
};

}  // namespace wpiemu::plugins::game_2026
