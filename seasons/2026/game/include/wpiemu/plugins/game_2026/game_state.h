#pragma once

// Game 2026 rules and entities: fuel bodies, every robot's fuel meter, hub scoring with the active /
// inactive schedule, hub recycling, outposts (corral intake, timed human-player chute) and the HUD
// values. Owns everything per session; the module that creates it stays stateless.
//
// Several robots (practice matches): the host robot (participant 0) and each remote robot announced through
// onRobotAdded have their own RobotState (fuel meter, shot counter, pickup and shooter zone runtimes) and act on the one
// shared FuelPool, host first, then ascending participant id. Scores, outposts and the match clock are shared.

#include <array>
#include <cstdint>
#include <map>
#include <memory>
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
#include "wpiemu/sdk/game/robot_view.h"
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
    // Always the HOST robot's values (sensors are the host robot's).
    std::optional<double> gameValue(std::string_view id) const override;
    // A remote robot gets a fresh state (meter preloaded, zone runtimes from its own zones); the host is always present.
    void onRobotAdded(sdk::SimContext& ctx, const sdk::RobotView& robot) override;
    void onRobotRemoved(sdk::SimContext& ctx, sdk::ParticipantId participant) override;
    // A remote robot's fuel count, capacity, shots and own-hub status.
    void publishRobot(sdk::SnapshotBuilder& out, const sdk::RobotView& robot) const override;

    // What one robot owns. `zones` are in the robot's zone order restricted to this plugin's kinds; `zoneBits[i]` is
    // the index of zones[i] in the robot's full zone list (its flag bit for a remote robot).
    struct RobotState {
        sdk::ParticipantId id = sdk::match::kHostParticipantId;
        bool remote = false;
        std::vector<sdk::ZoneConfig> zoneConfigs;  // remote robots: copied from the Join spec, rebuilt on reset()
        std::vector<std::unique_ptr<FuelZoneRuntime>> zones;
        std::vector<std::size_t> zoneBits;
        std::vector<IFuelIntakePort*> intakes;
        std::vector<IFuelShooterPort*> shooters;
        int meter = 0;
        int shots = 0;
    };

    // ---- read-only view for HUD, tests and tools ----------------------------------------------
    const GameConfig& config() const { return config_; }
    // The no-argument accessors are the HOST robot's.
    int fuelInRobot() const { return host_.meter; }
    int fuelOnField() const { return static_cast<int>(pool_.count()); }
    int shotsFired() const { return host_.shots; }
    // Fuel that left the play bounds (not attributable to a robot).
    int fuelLost() const { return lost_; }
    // nullptr when no such robot is known (the host, id 0, always is).
    const RobotState* robot(sdk::ParticipantId id) const;
    // Every robot, host first, then ascending participant id: the order in which they act each tick.
    std::vector<sdk::ParticipantId> robotIds() const;
    int outpostStock(Alliance a) const { return outposts_[index(a)].stock; }
    const AllianceScore& score(Alliance a) const { return hub_.score(a); }
    bool hubActive(Alliance a) const { return hub_.active(a); }
    const MatchStatus& match() const { return status_; }
    const HubRules& hubRules() const { return hub_; }
    // The host robot's owned zones in config order (pickups and shooters), for tests and tools.
    const std::vector<std::unique_ptr<FuelZoneRuntime>>& zones() const { return host_.zones; }
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
    // Where a robot is this tick. `view` is null for a remote robot that is not on the directory (it then does nothing).
    struct RobotFrame {
        RobotState* state = nullptr;
        const sdk::RobotView* view = nullptr;
        sdk::BodyId body;
        sdk::Pose3 pose;
        bool present = false;
    };

    void buildHostZones(sdk::SimContext& ctx);
    static void buildRuntimes(RobotState& robot, const std::vector<sdk::ZoneConfig>& configs);
    void resetRobot(RobotState& robot);
    void refreshOrder();
    void collectFrames(sdk::SimContext& ctx);
    void stepZones(sdk::SimContext& ctx, double dt);
    void stepShooters(sdk::SimContext& ctx, const RobotFrame& frame);
    void stepIntakes(const RobotFrame& frame);
    void stepRecycling(sdk::SimContext& ctx, double dt);
    void stepHumanPlayers(sdk::SimContext& ctx, double dt);
    void refreshPositions();
    // Table 5-4 cues owned by the season (seasons/2026/game/README.md).
    void requestSoundCues(sdk::SimContext& ctx, const MatchStatus& previous, bool resultEdge, bool eStop);

    GameConfig config_;
    FieldLayout layout_;
    FuelPool pool_;
    MatchClock clock_;
    MatchStatus status_;
    HubRules hub_;
    Alliance ownAlliance_ = Alliance::Red;
    bool matchTimed_ = false;   // the DS runs a timed match (Practice); otherwise free play without game timing
    bool matchResult_ = false;  // a timed match just stopped: its final score stays on the HUD
    bool resultCuePending_ = false;  // the result chime waits for the scoring window to close
    int lost_ = 0;
    std::vector<PendingRelease> pending_;
    std::array<OutpostState, 2> outposts_{};

    // The host robot (zones from RobotConfig::zones, evaluated against the mechanisms) and the remote robots (zones from
    // their Join spec, activated by the client's zone flags). `order_` = host, then the remote robots by id.
    RobotState host_;
    std::map<sdk::ParticipantId, RobotState> remotes_;
    std::vector<RobotState*> order_;
    std::vector<RobotFrame> frames_;
    std::vector<SensorHit> hits_;
    std::vector<std::uint32_t> scratchIds_;
    std::vector<sdk::Vec3> fuelPositions_;
    std::vector<std::uint32_t> fuelIds_;   // parallel to fuelPositions_: FuelPool::entityId, the telemetry entity id
};

}  // namespace wpiemu::plugins::game_2026
