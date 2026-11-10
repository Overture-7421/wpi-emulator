#pragma once

// The game-client side of a practice match: everything between a transport to the host and a render-facing picture, Qt-free and
// without threads of its own. The app (or a test) calls update() often (>= 120 Hz; it reads the network, advances the
// prediction and rebuilds the view) and binds its UI to view().
//
// What it does
//   join         encodes the robot description with the existing RobotConfigCodec, runs the SAME intake the host runs
//                (acceptRemoteSpec: decode, defaults, validation, fairness clamps) to learn the accepted spec and which fields
//                the host will clamp (view.specClamps), sends Join, handles JoinAccept / JoinReject / Roster, keeps the resume
//                token so the link can be re-established (resume(), or automatically through services.reconnect).
//   clock        pings the host and fits the host's sim time against the client's clock (ClockSync); host ticks are the unit of
//                every timestamp on the wire.
//   input        the player's command (wheel setpoints, zone flags, optional zone poses) becomes MatchInputs stamped with the
//                host tick they apply on, sent 60 Hz (sooner when the command changes) with the last 3 repeated in every packet.
//   own robot    predicted in a local physics world (OwnPredictor) that runs ahead of the host by the input lead, reconciled
//                with every snapshot (rewind, replay, smooth correction).
//   others       interpolated in the past by an adaptive delay, extrapolated for a bounded time (RobotTrack); fuel and other
//                entities through the client entity memory and the shared dead-reckoning model.
//   state        Connecting / Lobby / Live / Paused / HostStalled / Disconnected with reasons, and diagnostics.
//
// Time is the injected IWallClock only. Threading: update(), connect(), resume() and leave() from one thread; the command
// setters and view() / status() may be called from any thread.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/match/client_view.h"
#include "wpiemu/core/match/clock_sync.h"
#include "wpiemu/core/match/hud_keys.h"
#include "wpiemu/core/match/own_predictor.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/physics_world.h"
#include "wpiemu/sdk/physics/tire_model.h"

namespace wpiemu::core::match {

struct MatchClientConfig {
    std::string displayName = "Player";
    int wpilibYear = 0;                                  // 0: the season of the game module named by the robot's gamePluginId

    // Wall-clock timeouts (the injected IWallClock).
    std::chrono::milliseconds joinTimeout{5000};         // no JoinAccept this long after the Join
    std::chrono::milliseconds hostSilenceTimeout{3000};  // joined and nothing from the host this long: the link is dead
    std::chrono::milliseconds stallInferenceTimeout{750};  // "Running" snapshots whose sim time does not advance this long
    std::chrono::milliseconds startupPingInterval{120};
    std::chrono::milliseconds pingInterval{500};
    std::size_t startupPings = 12;                       // sent at the shorter interval after joining and after every restart

    // Reconnecting (only with services.reconnect).
    std::uint32_t maxReconnectAttempts = 8;
    std::chrono::milliseconds reconnectInitialBackoff{250};
    std::chrono::milliseconds reconnectMaxBackoff{4000};

    // Prediction and timing.
    ClockSyncConfig clock;
    OwnPredictorConfig prediction;
    bool predictRemoteContacts = true;                   // the other robots are kinematic bodies in the prediction world
    int minLeadTicks = 3;                                // bounds of the input lead (host ticks the client runs ahead)
    int maxLeadTicks = 120;
    int leadMarginTicks = 3;                             // the margin the lead aims for beyond latency and jitter
    int leadMarginLowTicks = 2;                          // the host reports an input arriving this early or less: raise the lead
    int leadMarginHighTicks = 10;                        // ... this early or more for a while: lower it
    std::size_t leadWindow = 24;                         // snapshots the margin is judged over
    int maxCatchUpTicks = 80;                            // simulated per update at most; beyond it the prediction restarts
    std::int64_t minInputSpacingTicks = 2;               // a changed command is sent no sooner than this after the last input
    std::int64_t maxInterpolationDelayUs = sdk::match::kMaxInterpolationDelayUs;
    std::int64_t maxExtrapolationUs = sdk::match::kMaxExtrapolationUs;
};

struct MatchClientServices {
    std::shared_ptr<const sdk::proto::IMessageCodec> codec;   // required
    std::shared_ptr<sdk::IWallClock> clock;                   // required
    sdk::ILogger* logger = nullptr;
    // The real config pipeline (decode, defaults, validation) that turns the robot description into the accepted spec;
    // required by connect().
    const config::ConfigPipeline* specPipeline = nullptr;
    // What the prediction world is built from. Without the registries (or when the drivetrain cannot be built) the client
    // still plays: the own robot is then drawn from snapshots like a remote one (view.own.predicted == false).
    const sdk::GamePluginRegistry* plugins = nullptr;         // the field comes from the module of the robot's gamePluginId
    const sdk::DrivetrainRegistry* drivetrains = nullptr;
    const sdk::TireModelRegistry* tireModels = nullptr;
    sdk::PhysicsWorldFactory worldFactory;                    // empty: Box3D
    HudKeyTable hudKeys;                                      // names the HUD key ids (hud_keys.h); empty: ids only
    // Opens a new transport to the host after the link broke (the app dials the same endpoint again). Optional.
    std::function<sdk::Result<std::unique_ptr<sdk::proto::ITransport>>()> reconnect;
};

class MatchClient {
public:
    MatchClient(MatchClientConfig config, MatchClientServices services);
    ~MatchClient();
    MatchClient(const MatchClient&) = delete;
    MatchClient& operator=(const MatchClient&) = delete;

    // ---- connection (one thread) ----
    // Joins with `robot` over `transport` (the caller already did the secure handshake). Fails without sending anything when
    // the description does not pass the host's own intake (state Disconnected, reason BadSpec) or the client is connected.
    sdk::Status connect(std::unique_ptr<sdk::proto::ITransport> transport, const sdk::RobotConfig& robot);
    // Rejoins over a new transport with the resume token of the last JoinAccept (after TransportLost / HostSilent). The host
    // keeps the robot for its resume window; later it is a fresh join.
    sdk::Status resume(std::unique_ptr<sdk::proto::ITransport> transport);
    // Tells the host and closes the link.
    void leave(std::string detail = {});

    // Reads the network, advances timers and the prediction, rebuilds the view. Call it often.
    void update();

    // ---- the player's command (any thread) ----
    // Wheel setpoints in the accepted spec's wheel order. A wrong count is refused.
    sdk::Status setWheelSetpoints(std::span<const sdk::WheelSetpoint> setpoints);
    // Chassis speeds to wheel setpoints with the shared kinematics (swerve, tank, mecanum); false-kind drivetrains refuse.
    sdk::Status driveRobotRelative(const sdk::ChassisSpeeds& speeds);
    // Field-frame velocities turned robot-relative with the predicted heading, then as above.
    sdk::Status driveFieldRelative(const sdk::ChassisSpeeds& fieldSpeeds);
    void setZoneFlags(std::uint32_t flags);
    void setZoneActive(std::size_t zone, bool active);
    // Moves a zone (chassis frame); the host clamps it to the robot's envelope. Held until cleared; every input carries it.
    sdk::Status setZonePose(std::size_t zone, const sdk::Pose3& chassisPose);
    void clearZonePose(std::size_t zone);

    // ---- queries (any thread) ----
    std::shared_ptr<const MatchClientView> view() const;
    ConnectionStatus status() const;
    // The static field the prediction world holds (for drawing); null until the robot joined with a prediction world.
    std::shared_ptr<const sdk::FieldDescription> fieldDescription() const;
    // The spec the host accepted (decoded, defaulted, clamped); empty before connect().
    std::shared_ptr<const sdk::RobotConfig> acceptedSpec() const;
    std::size_t wheelCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::core::match
