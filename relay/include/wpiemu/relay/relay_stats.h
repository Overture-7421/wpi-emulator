#pragma once

// Counters of the relay, for the periodic stats log line and for tests. Plain data; the room table fills the routing part,
// the server adds the connection part.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace wpiemu::relay {

// Why a game frame was not forwarded.
enum class DropReason : std::uint8_t {
    Unregistered = 0,  // the sender has not completed RegisterRoom / JoinRoom
    Oversized,         // above kRelayMaxDatagram or kRelayMaxReliablePacket (a host frame counts its envelope, a client frame does not)
    RateLimited,       // the sender's token bucket was empty
    RoomByteCap,       // the room's byte budget was empty
    TooShort,          // a host frame without the envelope (or with nothing after it)
    UnknownPeer,       // a host frame addressed to a peer that is not in the room
    NoRecipients,      // a broadcast in a room without clients
    HostAbsent,        // a client frame while the host is away
};
inline constexpr std::size_t kDropReasonCount = 8;

inline std::string_view dropReasonName(DropReason reason) {
    switch (reason) {
        case DropReason::Unregistered: return "unregistered";
        case DropReason::Oversized: return "oversized";
        case DropReason::RateLimited: return "rate_limited";
        case DropReason::RoomByteCap: return "room_byte_cap";
        case DropReason::TooShort: return "too_short";
        case DropReason::UnknownPeer: return "unknown_peer";
        case DropReason::NoRecipients: return "no_recipients";
        case DropReason::HostAbsent: return "host_absent";
    }
    return "unknown";
}

struct RelayStats {
    // Gauges.
    std::uint64_t rooms = 0;
    std::uint64_t roomsWithoutHost = 0;
    std::uint64_t peers = 0;  // clients in rooms
    std::uint64_t trackedSources = 0;
    std::uint64_t connections = 0;  // every open connection, registered or not

    // Routing.
    std::uint64_t framesForwarded = 0;  // delivered copies
    std::uint64_t bytesForwarded = 0;   // bytes of delivered copies, envelope included
    std::uint64_t bytesReceived = 0;    // game bytes that arrived from registered peers
    std::array<std::uint64_t, kDropReasonCount> framesDropped{};

    // Rooms.
    std::uint64_t roomsCreated = 0;
    std::uint64_t roomsResumed = 0;
    std::uint64_t roomsExpired = 0;
    std::uint64_t peersJoined = 0;
    std::uint64_t peersKicked = 0;  // disconnected for sustained rate or size violations
    std::uint64_t joinsRejected = 0;
    std::uint64_t registersRejected = 0;
    std::uint64_t lockouts = 0;         // sources that became locked out
    std::uint64_t lockedOutAttempts = 0;  // attempts refused while locked out
    std::uint64_t lockoutTableFull = 0;   // failures that could not be tracked because the table was full of lockouts

    // Server.
    std::uint64_t connectionsAccepted = 0;
    std::uint64_t controlViolations = 0;  // undecodable, oversize, wrong type or wrong state: the peer is disconnected
    std::uint64_t unregisteredTimeouts = 0;
    std::uint64_t unknownSources = 0;  // connections whose source address could not be determined
    std::uint64_t sendFailures = 0;

    std::uint64_t framesDroppedTotal() const {
        std::uint64_t total = 0;
        for (const std::uint64_t n : framesDropped) total += n;
        return total;
    }
    std::uint64_t dropped(DropReason reason) const { return framesDropped[static_cast<std::size_t>(reason)]; }
};

}  // namespace wpiemu::relay
