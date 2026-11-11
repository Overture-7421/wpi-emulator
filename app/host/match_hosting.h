#pragma once

// The host side of a practice match, composed for the app: invite secret, listener (direct ENet or the relay), the
// end-to-end SecureTransport on EVERY connection, the MatchHost, the host constraints and the published status. Qt-free.
//
// Composition (nothing here is game logic, see core/match for that):
//   invite secret (net::generateInvite)  ->  roomId + hostAuth (relay registration) and the channel key (SecureTransport)
//   ITransportServer (EnetServerTransport | RelayHostTransport)  --accept-->  raw peer transport
//   --SecureTransport::acceptAsHost (bounded handshake, off the accept thread)-->  MatchHost::addConnection
// A connection that cannot prove it holds the invite is closed before any application frame is read, so MatchHost never sees
// a plaintext peer.
//
// Threading:
//   * create/start/status/kick/constraints/shutdown-from-any-thread: any thread.
//   * one "hosting" thread opens the listener (the relay registration can take seconds) and accepts peers; each accepted
//     peer's handshake runs on its own short-lived thread (at most kMaxPendingHandshakes at once, the rest are refused).
//   * the SIM THREAD calls onPoll/onTickEnd (SessionRunner::setTickObserver). They forward to MatchHost and then publish the
//     status snapshot (at most every kStatusRefresh, and at once when the phase or the roster size changes).
//   * shutdown() must run where the session may be touched (the sim thread, or after it has stopped): it stops accepting,
//     tells the clients ServerClosing, removes their robots from the session and closes the listener (a relay room is
//     closed with CloseRoom). The destructor calls it too but never touches the session once it was shut down.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>

#include "host/hosting.h"
#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/match/match_host.h"
#include "wpiemu/core/runtime/session_runner.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::app::host {

inline constexpr std::size_t kMaxPendingHandshakes = 8;
inline constexpr std::chrono::milliseconds kStatusRefresh{250};

struct MatchHostingServices {
    core::match::IMatchHostSession* session = nullptr;       // required; outlives the hosting
    const core::config::ConfigPipeline* pipeline = nullptr;   // required: decodes a Join's robot description
    sdk::ILogger* logger = nullptr;                           // null = discard
    // Receives the constraints on every change, from the sim thread (ConstrainedController::apply).
    std::function<void(const core::match::HostConstraints&)> onConstraints;
    // Defaults: the steady clock, net::fillRandom (resume tokens and the invite). Tests inject others.
    std::shared_ptr<sdk::IWallClock> clock;
    std::function<sdk::Status(std::span<std::byte>)> randomFill;
    std::chrono::milliseconds handshakeTimeout{3000};
    core::match::MatchHostConfig matchConfig;                 // limits and rates; maxPlayers and maxConnections come from the options
};

class MatchHosting final : public core::runtime::ISimTickObserver {
public:
    // Validates the options (InvalidArgument), draws the invite and composes the MatchHost. Opens nothing yet. A mode of Off
    // is an InvalidArgument: callers do not create a MatchHosting then.
    static sdk::Result<std::unique_ptr<MatchHosting>> create(HostingOptions options, MatchHostingServices services);
    ~MatchHosting() override;
    MatchHosting(const MatchHosting&) = delete;
    MatchHosting& operator=(const MatchHosting&) = delete;

    // Starts the hosting thread: open the listener, then accept. Never blocks; the status reports Starting, then Lobby or Error.
    void start();

    // The latest published status; never null.
    std::shared_ptr<const HostingStatus> status() const;
    const HostingOptions& options() const { return options_; }
    core::match::HostConstraints constraints() const;

    // Removes a participant (MatchControl::Kick). Applied by the next poll of the sim thread.
    void kick(sdk::match::ParticipantId participant, std::string detail = {});

    // Sim thread hooks (see the threading note).
    void onPoll(const core::runtime::RunnerObservation& observation) override;
    void onTickEnd(const core::runtime::RunnerObservation& observation) override;

    // Stops accepting, closes every client (ServerClosing, robots removed) and the listener. Idempotent.
    void shutdown();

private:
    struct Impl;
    explicit MatchHosting(std::unique_ptr<Impl> impl, HostingOptions options);

    HostingOptions options_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::app::host
