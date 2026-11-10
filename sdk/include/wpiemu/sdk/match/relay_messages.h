#pragma once

// The PLAINTEXT relay control messages and the relay envelope constants. The relay (relay/, hosted on Azure) lets a host
// and its clients meet without port forwarding: everyone connects OUTBOUND to it over ENet. It understands only these
// messages, only on channel kRelayChannelControl. Everything on the game channels is end-to-end encrypted opaque bytes
// that it forwards without parsing, so match protocol changes never need a relay redeploy. No game data is ever in this
// header. Plain structs; see match_protocol.h and docs for the encryption design.
//
// Flow. Host: RegisterRoom -> RoomRegistered | RoomRejected; CloseRoom frees the room at once. Client: JoinRoom -> JoinResult (with
// its peerId). The relay tells the host PeerJoined / PeerLeft. Game channels then carry a 2-byte envelope (little-endian peer id): client -> host
// frames arrive at the host prefixed with the sender's peerId; host -> client frames are prefixed by the host with the
// target peerId (the relay strips it) or kRelayBroadcastPeer (every client). A client sends its frames unprefixed.
//
// Identifiers. roomId = 8 Crockford base32 characters derived by the host from the invite secret; hostAuth = 32 bytes
// derived from the same secret. The relay holds only these two.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::sdk::match::relay {

inline constexpr std::uint32_t kRelayProtocolVersion = 1;
inline constexpr std::uint32_t kMinCompatibleRelayProtocolVersion = 1;

// ---- ENet channels to the relay ---------------------------------------------------------------------------------
inline constexpr std::uint8_t kRelayChannelControl = 0;         // reliable: the control messages of this header
inline constexpr std::uint8_t kRelayChannelGameReliable = 1;    // reliable: opaque game frames
inline constexpr std::uint8_t kRelayChannelGameUnreliable = 2;  // unreliable-sequenced: opaque game frames
inline constexpr std::uint8_t kRelayChannelCount = 3;

// ---- envelope and peer ids ---------------------------------------------------------------------------------------
using RelayPeerId = std::uint16_t;
inline constexpr RelayPeerId kRelayHostPeer = 0;             // the host's side of a room; clients get ids from 1
inline constexpr RelayPeerId kRelayBroadcastPeer = 0xFFFF;   // host -> all clients
inline constexpr std::size_t kRelayEnvelopeBytes = 2;        // the u16 peer id prefix

// ---- limits (the relay reads them from the environment; these are the defaults and the protocol's bounds) -----------
inline constexpr std::size_t kRelayRoomIdChars = 8;
inline constexpr std::size_t kRelayHostAuthBytes = 32;
inline constexpr std::size_t kRelayMaxPeersPerRoom = 6;   // clients per room; the host app still enforces kMaxRobotsPerMatch
inline constexpr std::size_t kRelayMaxRooms = 100;
inline constexpr std::size_t kRelayMaxDatagram = 1400;    // largest unreliable game packet as a CLIENT sends it; a frame from the host counts its 2-byte envelope (so 1398 payload). 1200 snapshot payload + 24 crypto + slack
inline constexpr std::size_t kRelayMaxReliablePacket = 128 * 1024;  // largest reliable packet (a Join with a big robot spec)
inline constexpr std::uint32_t kRelayPeerBytesPerSecond = 64 * 1024;  // per-peer token bucket rate
inline constexpr std::uint32_t kRelayRoomTtlWithoutHostMs = 30'000;   // a room whose host left is kept this long for re-register

// The alphabet of a room id (Crockford base32: no I, L, O, U).
inline constexpr const char* kRoomIdAlphabet = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";

// The well-known health-ping room. It is a valid room id that no room ever has: the relay answers a JoinRoom for it with
// JoinResult{ok = false, NoSuchRoom} without counting a failed join toward the source's lockout, so liveness probes (the
// relay's --selfcheck, wpiemu_relay_probe) can run as often as they like. RegisterRoom for it is refused and invites never
// derive it.
inline constexpr const char* kRelayPingRoomId = "00000000";

inline bool isPingRoomId(const std::string& id) { return id == kRelayPingRoomId; }

inline bool isValidRoomId(const std::string& id) {
    if (id.size() != kRelayRoomIdChars) return false;
    for (const char c : id) {
        bool found = false;
        for (const char* a = kRoomIdAlphabet; *a != '\0'; ++a) found = found || *a == c;
        if (!found) return false;
    }
    return true;
}

using HostAuth = std::array<std::uint8_t, kRelayHostAuthBytes>;

// ---- enums (last value noted for the decoder's bound) -----------------------------------------------------------------
enum class RoomRejectReason : std::uint8_t {
    RoomTaken = 0,   // the room id is registered by another host (the host picks a new invite secret)
    BadAuth = 1,     // re-register with a different hostAuth
    RelayFull = 2,   // kRelayMaxRooms reached
    BadVersion = 3,  // last value 3
};

enum class JoinRoomFailure : std::uint8_t {
    None = 0,
    NoSuchRoom = 1,
    RoomFull = 2,
    BadVersion = 3,
    LockedOut = 4,   // too many failed joins from this source address
    HostAbsent = 5,  // the room exists but its host is not connected right now; last value 5
};

enum class PeerLeftReason : std::uint8_t {
    Disconnected = 0,
    TimedOut = 1,
    RateLimited = 2,  // exceeded its token bucket or datagram cap
    Oversized = 3,    // last value 3
};

// ---- messages ------------------------------------------------------------------------------------------------------

// Host -> relay.
struct RegisterRoom {
    std::uint32_t protocolVersion = kRelayProtocolVersion;
    std::string roomId;  // kRelayRoomIdChars characters of kRoomIdAlphabet
    HostAuth hostAuth{};

    bool operator==(const RegisterRoom&) const = default;
};

// Relay -> host.
struct RoomRegistered {
    bool resumed = false;  // true when the host re-registered an existing room within the grace period
    std::uint16_t maxPeers = static_cast<std::uint16_t>(kRelayMaxPeersPerRoom);

    bool operator==(const RoomRegistered&) const = default;
};

struct RoomRejected {
    RoomRejectReason reason = RoomRejectReason::RoomTaken;

    bool operator==(const RoomRejected&) const = default;
};

// Client -> relay.
struct JoinRoom {
    std::uint32_t protocolVersion = kRelayProtocolVersion;
    std::string roomId;

    bool operator==(const JoinRoom&) const = default;
};

// Relay -> client. On failure `peerId` is kRelayBroadcastPeer (unassigned).
struct JoinResult {
    bool ok = false;
    JoinRoomFailure reason = JoinRoomFailure::None;
    RelayPeerId peerId = kRelayBroadcastPeer;

    bool operator==(const JoinResult&) const = default;
};

// Host -> relay, after RegisterRoom succeeded on the same connection (that connection IS the authentication): the room is
// closed at once instead of waiting out kRelayRoomTtlWithoutHostMs. Its clients are disconnected and the host's connection
// ends. From any other connection it is a control violation.
struct CloseRoom {
    bool operator==(const CloseRoom&) const = default;
};

// Relay -> host.
struct PeerJoined {
    RelayPeerId peerId = kRelayBroadcastPeer;

    bool operator==(const PeerJoined&) const = default;
};

struct PeerLeft {
    RelayPeerId peerId = kRelayBroadcastPeer;
    PeerLeftReason reason = PeerLeftReason::Disconnected;

    bool operator==(const PeerLeft&) const = default;
};

}  // namespace wpiemu::sdk::match::relay
