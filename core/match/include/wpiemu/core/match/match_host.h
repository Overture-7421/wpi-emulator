#pragma once

// The server side of a practice match: a Qt-free, transport-agnostic game server that feeds remote clients into the
// host's SimulationSession and publishes what happens in the world back to them.
//
// ---- Ownership and threading ------------------------------------------------------------------------------------
// The host app owns the simulated world (SimulationSession), driven by SessionRunner on the SIM THREAD. MatchHost never
// touches the session from another thread:
//   * connections: ChannelMessageLink reader threads (one per client) decode frames into the host's own MessageInbox;
//     a MessageLinkAcceptor thread accepts connections when listen() is used. addConnection() may be called from any
//     thread. These threads never touch the session or the host's state.
//   * the SIM THREAD calls onPoll() / onTickEnd() (the runtime::ISimTickObserver hooks, SessionRunner::setTickObserver).
//     Everything else the host does happens inside those two calls: drain the inbox, join/leave/resume/kick, submit
//     inputs, add/remove remote robots, build and send snapshots.
//   * sending: the sim thread only enqueues messages. Each client has a bounded PeerOutbox with its own sender thread, so a
//     slow or stuck client costs the sim thread nothing: its snapshots are dropped oldest-first and, if its reliable queue
//     overflows or a send is stuck for outbox.stallTimeout, it is dropped (the participant stays resumable).
//   * thread-safe from anywhere: addConnection, kick, close, constraints, joinedCount, runEpoch.
//   * sim thread only: onPoll, onTickEnd, closeNow, participants, stats, waitOutboundIdle.
//
// ---- Call sequence of the sim loop (SessionRunner::poll) ---------------------------------------------------------
//   poll(): apply UI commands -> drain the robot link -> watchdog -> onPoll(obs) -> [tick: session.tick -> send
//   SensorFrame -> onTickEnd(obs)] -> publish the UI snapshot.
//   onPoll   drain the inbox and apply it (joins, leaves, inputs, acks, pings, kicks), timeouts, epoch/state changes
//            (Reset, Pause, Resume broadcasts), idle snapshots while the world is not ticking, the roster every second.
//   onTickEnd  send the snapshots that are due on the sim-time lattice. Wall clock is used only for the explicit timeouts
//            and for snapshots while the world is not ticking; while it ticks, snapshot pacing is sim time.
//
// ---- Wire rules the host applies --------------------------------------------------------------------------------
// See the member docs below and sdk/match/*.h. MatchState mapping: Lobby = the robot program is not running in lock-step
// (no robot, armed, not started, session over), Running, Paused = the host paused, HostStalled = the robot watchdog fired.
// The sim runner supplies it in RunnerObservation::matchState.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/match/host_session.h"
#include "wpiemu/core/match/peer_outbox.h"
#include "wpiemu/core/match/snapshot_rate.h"
#include "wpiemu/core/runtime/session_runner.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/match/join_messages.h"
#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::core::match {

struct MatchHostConfig {
    // Wall-clock timeouts (the injected IWallClock).
    std::chrono::milliseconds joinTimeout{2000};      // first message of a connection must be a Join within this
    std::chrono::milliseconds peerTimeout{5000};      // a joined client silent this long is dropped (its robot stays resumable)
    std::chrono::milliseconds resumeWindow{30000};    // a dropped participant keeps its id, station and robot this long
    std::chrono::milliseconds rosterInterval{1000};   // roster refresh (ping and loss figures)
    std::chrono::milliseconds idleSnapshotInterval{100};  // snapshots while the world is not ticking (lobby, paused, stalled)
    std::chrono::milliseconds closeFlushTimeout{500};     // how long close()/the destructor waits for outboxes to drain

    // Limits.
    std::size_t maxConnections = sdk::match::kMaxRobotsPerMatch + 3;  // joined plus unjoined
    std::size_t maxStrikes = 3;                       // malformed frames or protocol violations before a disconnect
    std::uint32_t maxInputsPerSecond = sdk::match::kMaxInputRateHz;   // per client, token bucket
    std::uint32_t inputBurst = 12;
    std::size_t joinsPerPoll = 1;                     // joins decoded (JSON + validation) per onPoll: bounds the sim-thread hiccup
    OutboxLimits outbox;

    // Snapshots.
    SnapshotRateConfig rate;
    std::uint16_t inputRateHz = sdk::match::kDefaultInputRateHz;      // stated in JoinAccept
    std::size_t snapshotBudgetBytes = sdk::match::kSnapshotBudgetBytes;
    // HUD key of the game's match phase as a text value; copied into MatchInfo::phaseKey when set. Empty = none.
    std::string phaseHudKey;
    // Extra HUD keys for the key table next to the plugin's descriptors (hud_keys.h).
    std::vector<std::string> extraHudKeys;
};

struct MatchHostServices {
    std::shared_ptr<const sdk::proto::IMessageCodec> codec;   // required
    std::shared_ptr<sdk::IWallClock> clock;                   // required
    sdk::ILogger* logger = nullptr;
    // The real config pipeline that decodes a Join's robot description (required to accept a join).
    const config::ConfigPipeline* specPipeline = nullptr;
    // Cryptographic randomness for resume tokens (net::fillRandom in the app; deterministic in tests). Required: without it
    // joins are refused, there is deliberately no weak fallback in core.
    std::function<sdk::Status(std::span<std::byte>)> randomFill;
};

// What the composition root must enforce while clients are connected (the host only publishes the flags).
struct HostConstraints {
    bool peersConnected = false;
    bool lockTimeScale = false;      // the sim must run at 1x (clients cannot follow another speed)
    bool stepDisallowed = false;     // single-stepping is disabled
};

struct MatchHostStats {
    std::uint64_t connections = 0;            // accepted into the table
    std::uint64_t connectionsRefused = 0;     // over maxConnections
    std::uint64_t joinsAccepted = 0;
    std::uint64_t joinsRejected = 0;
    std::uint64_t resumes = 0;
    std::uint64_t leaves = 0;
    std::uint64_t kicks = 0;
    std::uint64_t joinTimeouts = 0;
    std::uint64_t peerTimeouts = 0;
    std::uint64_t slowPeersDropped = 0;
    std::uint64_t malformedFrames = 0;
    std::uint64_t protocolViolations = 0;
    std::uint64_t inputPackets = 0;
    std::uint64_t inputsSubmitted = 0;        // handed to the session
    std::uint64_t inputsRedundant = 0;        // repeated copies of an input already seen
    std::uint64_t inputsRateLimited = 0;
    std::uint64_t inputsRejected = 0;         // the session refused them (wrong wheel count, sequence 0, too far ahead)
    std::uint64_t framesStaleEpoch = 0;
    std::uint64_t framesForeignSession = 0;
    std::uint64_t acks = 0;
    std::uint64_t acksIgnored = 0;            // unknown, older or outside the history
    std::uint64_t pings = 0;
    std::uint64_t snapshotsSent = 0;
    std::uint64_t snapshotsTruncated = 0;     // entity changes were left for later snapshots to fit the budget
    std::uint64_t entityChangesDeferred = 0;
    std::uint64_t snapshotsOverBudget = 0;    // robots + info alone exceeded the budget (they are always sent)
    std::uint64_t snapshotsDroppedSlow = 0;   // unreliable snapshots dropped from a slow client's queue
    std::uint64_t hudKeysUnknown = 0;
    std::size_t maxSnapshotBytes = 0;         // largest encoded snapshot so far
};

struct ParticipantInfo {
    sdk::match::ParticipantId id = sdk::match::kNoParticipant;
    std::string name;
    sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
    bool connected = false;               // false: dropped, resumable until the window closes
    std::uint16_t snapshotRateHz = 0;
    std::uint64_t inputsSubmitted = 0;
    std::uint64_t inputsRateLimited = 0;
    std::uint64_t snapshotsSent = 0;
    std::uint32_t baselineId = sdk::match::kNoBaseline;   // snapshotSeq the client's entity state is relative to
};

class MatchHost final : public runtime::ISimTickObserver {
public:
    MatchHost(MatchHostConfig config, IMatchHostSession& session, MatchHostServices services);
    ~MatchHost() override;  // closes every connection (best effort ServerClosing first); never touches the session
    MatchHost(const MatchHost&) = delete;
    MatchHost& operator=(const MatchHost&) = delete;

    // ---- connections (any thread) ----
    // Starts accepting on `server` (one reader thread per client). The host owns the server.
    sdk::Status listen(std::unique_ptr<sdk::proto::ITransportServer> server);
    std::string endpoint() const;
    // Adopts an already connected transport (relay, tests).
    void addConnection(std::unique_ptr<sdk::proto::ITransport> transport);

    // ---- host commands (any thread; applied by the next onPoll) ----
    // Removes `participant`: MatchControl::Kick to it, its robot is removed and its token refused for the rest of the run.
    void kick(sdk::match::ParticipantId participant, std::string detail = {});
    // Tells everyone ServerClosing, removes their robots and closes every connection. Later joins are refused (HostClosing).
    void close(std::string detail = {});
    // The same, immediately (sim thread; the session must be alive). Used at shutdown by the composition root.
    void closeNow(std::string detail = {});

    // ---- sim thread hooks ----
    void onPoll(const runtime::RunnerObservation& observation) override;
    void onTickEnd(const runtime::RunnerObservation& observation) override;

    // ---- queries ----
    HostConstraints constraints() const;                     // any thread
    std::size_t joinedCount() const;                         // any thread: participants holding a robot (connected or resumable)
    std::uint64_t runEpoch() const;                          // any thread
    // Waits until every outbound queue is empty (tests, shutdown). Sim thread.
    bool waitOutboundIdle(std::chrono::milliseconds timeout) const;
    std::vector<ParticipantInfo> participants() const;       // sim thread
    MatchHostStats stats() const;                            // sim thread

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::core::match
