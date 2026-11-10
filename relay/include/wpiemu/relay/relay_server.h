#pragma once

// The relay server: owns an ITransportServer (ENet in production, an in-memory fake in tests), a single-threaded event
// loop and a RoomTable. It forwards opaque, end-to-end-encrypted game frames between a host and its clients and enforces
// the limits. It never parses game frames and never logs their content.
//
// Per connection: Unregistered -> Host(room) | Client(room, peerId). Only RegisterRoom and JoinRoom (control channel 0) are
// valid from an Unregistered connection; the only control frame valid from a registered one is CloseRoom from the host (it
// closes the room at once); any other control frame, an undecodable or oversize one, or any control frame
// from a registered connection counts as a violation and disconnects the peer. Game frames from an Unregistered
// connection are dropped and counted. A connection that stays Unregistered is closed after `unregisteredTimeout` (the
// transport's own unauthenticatedTimeout already drops one that sends nothing at all), and after `maxAttemptsPerConnection`
// rejected RegisterRoom / JoinRoom attempts.
//
// Threading: run() / poll() / start() on one thread; stop() and stats() may be called from any thread. Every transport call
// uses a zero timeout, so the loop never blocks; run() sleeps `idleSleep` only when a pass found nothing to do.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include "wpiemu/relay/relay_log.h"
#include "wpiemu/relay/relay_stats.h"
#include "wpiemu/relay/room_table.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::relay {

struct RelayServerConfig {
    RoomTableConfig table;
    std::chrono::milliseconds unregisteredTimeout{10'000};
    std::size_t maxAttemptsPerConnection = 3;
    // Loopback sources (the container's own liveness probe, local tests) are never tracked for the lockout.
    bool exemptLoopbackFromLockout = true;
    // Period of the "stats" log line; 0 disables it.
    std::chrono::milliseconds statsInterval{60'000};
    // How often the room table expires rooms and the timeouts are checked.
    std::chrono::milliseconds tickInterval{50};
    std::chrono::milliseconds idleSleep{1};
    // Fairness: frames read from one connection and connections accepted in one pass.
    std::size_t maxFramesPerConnectionPerPoll = 64;
    std::size_t maxAcceptsPerPoll = 32;
};

class RelayServer {
public:
    RelayServer(std::unique_ptr<sdk::proto::ITransportServer> server, RelayServerConfig config,
                std::shared_ptr<const sdk::IWallClock> clock, std::shared_ptr<RelayLog> log);
    ~RelayServer();
    RelayServer(const RelayServer&) = delete;
    RelayServer& operator=(const RelayServer&) = delete;

    // Starts listening. IoError when the address cannot be bound.
    sdk::Status start();
    // One pass of the loop; true when it did any work (accepted, read, forwarded, expired). start() first.
    bool poll();
    // Loops until stop() (or until the transport server is closed), then disconnects everyone. Calls start() first when
    // it has not been called.
    sdk::Status run();
    // Asks run() to return; safe from any thread and from a signal handler's helper thread.
    void stop();
    // Disconnects every connection and closes the transport server. run() does it on return; tests may call it directly.
    void shutdown();

    std::string endpoint() const;
    // A snapshot refreshed by the loop (at most one tick interval old).
    RelayStats stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::relay
