#pragma once

// What a game client's UI binds to: one immutable, self-contained picture of the match as the player should see it NOW,
// rebuilt by MatchClient::update() and handed out as std::shared_ptr<const MatchClientView> (safe to read on another thread).
// Plain data, field frame (right-handed, +X toward the far wall, +Y left, Z up), SI units, quaternions (x, y, z, w). The
// scene-frame conversion belongs to the renderer (QML), as everywhere else.
//
//   own robot      predicted locally (zero added latency), corrected smoothly toward the host's authoritative state;
//   remote robots  interpolated a little in the past (diagnostics.bufferDelayMs), extrapolated for at most 250 ms if starved;
//   entities       fuel and other numerous things, extrapolated by the shared dead-reckoning model;
//   hud            match phase, time, enable state and the game's HUD values.

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/match/join_messages.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/robot_appearance.h"

namespace wpiemu::core::match {

enum class ConnectionState : std::uint8_t {
    Connecting,    // the transport is up (or being retried) and the Join was sent or is being resent
    Lobby,         // joined; the host has not started the match
    Live,          // joined and the world is running
    Paused,        // the host paused
    HostStalled,   // the host's robot stopped answering, or the world stopped advancing
    Disconnected,  // not connected (see reason)
};

enum class DisconnectReason : std::uint8_t {
    None,           // never connected, or connected
    Left,           // the player left (MatchClient::leave)
    JoinRejected,   // the host refused the Join (ConnectionStatus::rejectReason, detail)
    JoinTimeout,    // no JoinAccept in time
    BadSpec,        // the robot description failed the local decode, validation or clamps: the Join was never sent
    Kicked,         // removed by the host
    ServerClosing,  // the host closed the room
    TransportLost,  // the connection broke (and no reconnect succeeded)
    HostSilent,     // joined but the host sent nothing for too long
    ProtocolError,  // the host sent something the protocol does not allow
};

struct ConnectionStatus {
    ConnectionState state = ConnectionState::Disconnected;
    DisconnectReason reason = DisconnectReason::None;
    std::optional<sdk::match::JoinRejectReason> rejectReason;   // with JoinRejected
    std::string detail;                                         // the host's or the local explanation
    bool reconnecting = false;                                  // Connecting because a lost link is being re-established
    std::uint32_t reconnectAttempt = 0;

    bool operator==(const ConnectionStatus&) const = default;
};

struct WheelView {
    float steerRad = 0.f;   // module azimuth from chassis +x (0 for fixed wheels)
    float spinRad = 0.f;    // accumulated rotation, wrapped to [-pi, pi)
};

struct HudEntry {
    std::uint16_t keyId = 0;
    std::string key;        // "" when the client has no key table or the id is unknown
    float value = 0.f;
};

struct OwnRobotView {
    sdk::match::ParticipantId id = sdk::match::kNoParticipant;
    std::string name;
    sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
    std::shared_ptr<const sdk::match::RobotAppearance> appearance;   // null until the roster carried it
    sdk::Pose3 pose;                    // body origin (centre of mass); includes the smoothing offset
    sdk::Vec3 linearVelocity;
    sdk::Vec3 angularVelocity;
    std::vector<WheelView> wheels;
    std::uint32_t zoneFlags = 0;        // what the host will see active (the command, masked by the enable state)
    std::vector<sdk::Pose3> zonePoses;  // effective chassis-frame pose of each zone of the accepted spec
    std::vector<HudEntry> hud;          // from the newest snapshot
    bool predicted = false;             // false: drawn from snapshots like a remote robot (no prediction world)
    bool enabled = false;
    double correctionM = 0.0;           // visual offset still being absorbed, metres
};

struct RemoteRobotView {
    sdk::match::ParticipantId id = sdk::match::kNoParticipant;
    std::string name;
    sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
    bool isHost = false;
    std::shared_ptr<const sdk::match::RobotAppearance> appearance;
    sdk::Pose3 pose;                    // yaw only; z is the centre-of-mass height of the appearance (0 when unknown)
    sdk::Vec3 linearVelocity;
    sdk::Vec3 angularVelocity;
    std::vector<WheelView> wheels;
    std::uint32_t zoneFlags = 0;
    std::vector<sdk::Pose3> zonePoses;
    std::vector<HudEntry> hud;
    bool extrapolating = false;
    bool starved = false;               // held: nothing newer than the extrapolation limit
};

struct EntityView {
    std::uint32_t id = 0;
    float x = 0.f, y = 0.f, z = 0.f;    // metres, field frame
};

struct EntityGroupView {
    std::uint16_t groupId = 0;
    std::vector<EntityView> entities;
};

struct MatchHudView {
    sdk::match::MatchState state = sdk::match::MatchState::Lobby;
    bool enabled = false;
    sdk::proto::RobotMode mode = sdk::proto::RobotMode::Teleop;
    bool eStop = false;
    float matchTimeS = -1.f;
    std::string phaseKey;
    std::vector<HudEntry> game;
};

struct ClientDiagnostics {
    double rttMs = 0.0;                 // smoothed ping round trip
    double rttMinMs = 0.0;
    double jitterMs = 0.0;              // one-way, from the slow round trips of the window
    double snapshotJitterMs = 0.0;      // of the snapshot arrivals
    double lossPermille = 0.0;          // snapshots that never arrived
    double snapshotRateHz = 0.0;
    double snapshotAgeMs = 0.0;         // since the newest snapshot arrived
    double bufferDelayMs = 0.0;         // how far in the past remote robots are drawn
    double clockDriftPpm = 0.0;
    double clockAsymmetryBoundMs = 0.0; // half the minimum RTT: the bias path asymmetry could add to the clock offset
    bool clockSynced = false;
    double predictionErrorM = 0.0;      // own robot, newest snapshot: predicted vs authoritative position
    double predictionErrorP95M = 0.0;   // over the last 128 snapshots
    double predictionErrorYawRad = 0.0;
    double predictionAheadMs = 0.0;     // how far the prediction runs ahead of the host's sim time
    double predictedTick = 0.0;         // the host tick (fractional) the own robot is drawn at; 0 without prediction
    std::int64_t hostSimEstimateUs = 0; // the client's estimate of the host's sim time right now (0 until synced)
    int inputLeadTicks = 0;             // the host tick lead the client stamps its inputs with
    int hostReportedLeadTicks = 0;      // what the host measured for the newest input it received
    std::uint64_t snapshotsReceived = 0;
    std::uint64_t snapshotsStale = 0;
    std::uint64_t snapshotsDropped = 0; // wrong session or older epoch
    std::uint64_t reconciliations = 0;  // snapshots that triggered a rewind and replay
    std::uint64_t snaps = 0;            // reconciliations beyond the smoothing limits (a visible jump)
    std::uint64_t replayedTicks = 0;
    std::uint64_t inputsSent = 0;
    std::uint64_t lateInputs = 0;       // the host reported an input that arrived after its tick
    std::uint64_t entityBaselineMisses = 0;
    std::uint64_t resyncs = 0;          // prediction timeline restarts
    std::uint64_t pingsSent = 0;
    std::uint64_t pingsReceived = 0;
};

struct MatchClientView {
    std::uint64_t revision = 0;         // increases with every rebuild
    ConnectionStatus connection;
    std::uint64_t sessionId = 0;
    std::uint64_t runEpoch = 0;
    std::int64_t renderSimTimeUs = 0;   // the host sim time remote robots and entities are drawn at
    bool hasOwn = false;
    OwnRobotView own;
    std::vector<RemoteRobotView> remotes;   // the host's robot and the other clients' robots
    std::vector<EntityGroupView> entities;
    MatchHudView hud;
    std::vector<sdk::match::RosterEntry> roster;
    std::vector<std::string> specClamps;    // fields of the robot description the host's fairness limits corrected
    ClientDiagnostics diagnostics;
};

}  // namespace wpiemu::core::match
