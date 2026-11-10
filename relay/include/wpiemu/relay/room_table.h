#pragma once

// The relay's rooms as pure logic: no sockets, no threads, no wall clock of its own (an injected sdk::IWallClock) and no
// randomness, so the same sequence of calls and clock readings always yields the same decisions.
//
// Every operation returns what the relay must DO as data (`Effects`): a control reply to send, a game frame to forward, a
// connection to close. RelayServer executes them; tests inspect them. A connection id is whatever the caller chose.
//
// A connection is Unregistered until RegisterRoom (becomes the room's host) or JoinRoom (becomes a client with a peer id)
// succeeds. The relay never parses game frames: route() looks at nothing but the 2-byte envelope of a host frame
// (sdk/match/relay_messages.h), and forwards the rest unchanged on the delivery class it arrived on.
//
// Rooms. A host that leaves starts the room's TTL: clients stay connected but their frames are dropped. The same hostAuth
// registering again within the TTL resumes the room (clients keep their peer ids, the host is told about them with
// PeerJoined). The same hostAuth registering while the previous host connection is still considered live takes the room
// over (the old connection is closed): a host whose network dropped usually reconnects before the old connection times
// out. A different hostAuth is RoomTaken (live host) or BadAuth (host away). When the TTL expires the room closes and its
// clients are closed.
//
// Limits. Per-source lockout after repeated failed JoinRoom / RegisterRoom (NoSuchRoom, BadAuth, RoomTaken); the tracking
// table is bounded. Per-peer token bucket (burst = burstMs of credit, never less than one maximum reliable frame) charged
// with every game frame a peer sends; the host's bucket is `maxPeersPerRoom` times larger because it fans out to every
// client. Per-room token bucket charged with every forwarded copy. Frames above the datagram or reliable cap are dropped.
// A peer that keeps violating the size or rate limit (`abuseDropsPerSecond` drops within one second) is disconnected.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "wpiemu/relay/relay_stats.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/ipc/delivery.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::relay {

using ConnId = std::uint64_t;

struct RoomTableConfig {
    std::size_t maxRooms = sdk::match::relay::kRelayMaxRooms;
    std::size_t maxPeersPerRoom = sdk::match::relay::kRelayMaxPeersPerRoom;
    std::uint32_t roomTtlWithoutHostMs = sdk::match::relay::kRelayRoomTtlWithoutHostMs;

    std::size_t maxDatagramBytes = sdk::match::relay::kRelayMaxDatagram;
    std::size_t maxReliableBytes = sdk::match::relay::kRelayMaxReliablePacket;

    // Bytes per second a client may send. 0 disables the per-peer cap.
    std::uint32_t peerBytesPerSecond = sdk::match::relay::kRelayPeerBytesPerSecond;
    // The host's rate; 0 means peerBytesPerSecond * maxPeersPerRoom.
    std::uint32_t hostBytesPerSecond = 0;
    // Bytes per second the whole room may forward (every delivered copy counts); 0 means 2 * peerBytesPerSecond *
    // maxPeersPerRoom. When peerBytesPerSecond is 0 the room cap is off too.
    std::uint32_t roomBytesPerSecond = 0;
    std::uint32_t burstMs = 2000;
    // Rate or size drops within one second after which the peer is disconnected; 0 never disconnects.
    std::uint32_t abuseDropsPerSecond = 512;

    // Failed attempts from one source within the window that cause the lockout; 0 disables the lockout.
    std::uint32_t lockoutMaxFailures = 10;
    std::uint32_t lockoutWindowMs = 60'000;
    std::uint32_t lockoutDurationMs = 300'000;
    std::size_t maxTrackedSources = 4096;

    // Events kept for drainEvents(); further ones are dropped.
    std::size_t maxPendingEvents = 4096;
};

enum class DisconnectReason : std::uint8_t {
    HostReplaced,       // the same host registered again from a new connection
    RoomExpired,        // the host did not return within the TTL
    RateLimited,        // sustained token-bucket violations
    Oversized,          // sustained oversize frames
    LockedOut,          // RegisterRoom from a locked-out source (RoomRejected has no reason for it)
    ProtocolViolation,  // caller-side reasons below
    UnregisteredTimeout,
    TooManyAttempts,
    Shutdown,
    PeerGone,
    TransportError,
};

inline const char* disconnectReasonName(DisconnectReason reason) {
    switch (reason) {
        case DisconnectReason::HostReplaced: return "host_replaced";
        case DisconnectReason::RoomExpired: return "room_expired";
        case DisconnectReason::RateLimited: return "rate_limited";
        case DisconnectReason::Oversized: return "oversized";
        case DisconnectReason::LockedOut: return "locked_out";
        case DisconnectReason::ProtocolViolation: return "protocol_violation";
        case DisconnectReason::UnregisteredTimeout: return "unregistered_timeout";
        case DisconnectReason::TooManyAttempts: return "too_many_attempts";
        case DisconnectReason::Shutdown: return "shutdown";
        case DisconnectReason::PeerGone: return "peer_gone";
        case DisconnectReason::TransportError: return "transport_error";
    }
    return "unknown";
}

// Effects: what the caller must do, in order.
struct SendControl {
    ConnId to = 0;
    sdk::proto::Message message;
};
struct SendGame {
    ConnId to = 0;
    sdk::proto::Delivery delivery = sdk::proto::Delivery::Reliable;
    std::vector<std::byte> frame;
};
// The table has already forgotten `to`; the caller closes the transport.
struct CloseConnection {
    ConnId to = 0;
    DisconnectReason reason = DisconnectReason::ProtocolViolation;
};
using Effect = std::variant<SendControl, SendGame, CloseConnection>;
using Effects = std::vector<Effect>;

enum class Role : std::uint8_t { None, Host, Client };

struct RouteResult {
    std::optional<DropReason> dropped;  // set when the frame was not forwarded
    Effects effects;
};

// Things worth a log line, produced by the table and drained by the server (the table itself never logs).
struct RoomEvent {
    enum class Kind : std::uint8_t {
        RoomCreated,
        RoomResumed,
        HostLeft,
        RoomExpired,
        PeerJoined,
        PeerLeft,
        JoinRejected,
        RegisterRejected,
        LockedOut,       // a source just became locked out
        PeerKicked,
    };
    Kind kind = Kind::RoomCreated;
    ConnId conn = 0;
    std::string roomId;
    sdk::match::relay::RelayPeerId peerId = 0;
    std::string detail;  // the reject reason, kick reason, or the number of clients on close
};

inline const char* roomEventName(RoomEvent::Kind kind) {
    switch (kind) {
        case RoomEvent::Kind::RoomCreated: return "room_created";
        case RoomEvent::Kind::RoomResumed: return "room_resumed";
        case RoomEvent::Kind::HostLeft: return "host_left";
        case RoomEvent::Kind::RoomExpired: return "room_expired";
        case RoomEvent::Kind::PeerJoined: return "peer_joined";
        case RoomEvent::Kind::PeerLeft: return "peer_left";
        case RoomEvent::Kind::JoinRejected: return "join_rejected";
        case RoomEvent::Kind::RegisterRejected: return "register_rejected";
        case RoomEvent::Kind::LockedOut: return "lockout";
        case RoomEvent::Kind::PeerKicked: return "peer_kicked";
    }
    return "unknown";
}

class RoomTable {
public:
    RoomTable(RoomTableConfig config, std::shared_ptr<const sdk::IWallClock> clock);
    ~RoomTable();
    RoomTable(const RoomTable&) = delete;
    RoomTable& operator=(const RoomTable&) = delete;

    // `source` is the lockout key of the remote address (empty: unknown or exempt, never locked out).
    Effects registerRoom(ConnId conn, const std::string& source, const sdk::match::relay::RegisterRoom& request);
    Effects joinRoom(ConnId conn, const std::string& source, const sdk::match::relay::JoinRoom& request);
    // A registered connection went away (idempotent; unknown connections are ignored).
    Effects onDisconnect(ConnId conn, sdk::match::relay::PeerLeftReason reason = sdk::match::relay::PeerLeftReason::Disconnected);
    // One game frame from `conn`.
    RouteResult route(ConnId conn, sdk::proto::Delivery delivery, std::span<const std::byte> frame);
    // Expires rooms whose host stayed away for the TTL and prunes the lockout table.
    Effects tick();

    Role roleOf(ConnId conn) const;
    // The peer id of a client connection (kRelayBroadcastPeer for a connection that is not a client).
    sdk::match::relay::RelayPeerId peerIdOf(ConnId conn) const;
    std::string roomOf(ConnId conn) const;
    bool hasRoom(const std::string& roomId) const;
    bool hostPresent(const std::string& roomId) const;
    std::size_t clientCount(const std::string& roomId) const;
    bool isLockedOut(const std::string& source) const;
    // Every connection the table tracks (hosts and clients).
    std::size_t connectionCount() const;

    // Counters that belong to the table; the server merges its own.
    RelayStats stats() const;
    std::vector<RoomEvent> drainEvents();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::relay
