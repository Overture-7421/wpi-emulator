#include <doctest/doctest.h>

#include <random>
#include <sstream>

#include "relay_test_support.h"

using namespace std::chrono_literals;
using namespace relaytest;
using wpiemu::relay::CloseConnection;
using wpiemu::relay::ConnId;
using wpiemu::relay::DisconnectReason;
using wpiemu::relay::DropReason;
using wpiemu::relay::Role;
using wpiemu::relay::RoomTable;
using wpiemu::relay::RoomTableConfig;

namespace {

struct Fixture {
    std::shared_ptr<wpiemu::sdk::testing::ManualWallClock> clock = std::make_shared<wpiemu::sdk::testing::ManualWallClock>();
    RoomTable table;
    explicit Fixture(const RoomTableConfig& config = {}) : table(config, clock) {}

    std::span<const std::byte> view(const Bytes& b) const { return b; }

    // Registers `conn` as host of `room` and returns the reply.
    std::optional<Message> registerHost(ConnId conn, const std::string& room, std::uint8_t auth = 1, const std::string& source = "") {
        const auto effects = table.registerRoom(conn, source, std::get<rl::RegisterRoom>(registerMessage(room, auth)));
        const auto replies = controlsTo(effects, conn);
        if (replies.empty()) return std::nullopt;
        return replies.front();
    }
    // Joins `conn` and returns its peer id (kRelayBroadcastPeer on failure).
    rl::RelayPeerId join(ConnId conn, const std::string& room, const std::string& source = "") {
        const auto effects = table.joinRoom(conn, source, std::get<rl::JoinRoom>(joinMessage(room)));
        const auto result = firstOf<rl::JoinResult>(controlsTo(effects, conn));
        return result && result->ok ? result->peerId : rl::kRelayBroadcastPeer;
    }
    rl::JoinResult joinResult(ConnId conn, const std::string& room, const std::string& source = "") {
        const auto effects = table.joinRoom(conn, source, std::get<rl::JoinRoom>(joinMessage(room)));
        return firstOf<rl::JoinResult>(controlsTo(effects, conn)).value();
    }
};

Bytes payload(std::size_t size, std::uint8_t fill = 0xAB) { return Bytes(size, static_cast<std::byte>(fill)); }

Bytes withEnvelope(rl::RelayPeerId target, const Bytes& body) {
    Bytes out{static_cast<std::byte>(target & 0xFF), static_cast<std::byte>(target >> 8)};
    out.insert(out.end(), body.begin(), body.end());
    return out;
}

}  // namespace

TEST_CASE("relay table: a host registers a room") {
    Fixture f;
    const auto reply = f.registerHost(1, roomIdFor(1));
    REQUIRE(reply.has_value());
    const auto registered = std::get<rl::RoomRegistered>(*reply);
    CHECK_FALSE(registered.resumed);
    CHECK(registered.maxPeers == rl::kRelayMaxPeersPerRoom);
    CHECK(f.table.hasRoom(roomIdFor(1)));
    CHECK(f.table.hostPresent(roomIdFor(1)));
    CHECK(f.table.roleOf(1) == Role::Host);
    CHECK(f.table.roomOf(1) == roomIdFor(1));
}

TEST_CASE("relay table: register rejects a bad protocol version") {
    Fixture f;
    for (const std::uint32_t version : {0u, rl::kRelayProtocolVersion + 1, 0xFFFFFFFFu}) {
        const auto effects = f.table.registerRoom(1, "", std::get<rl::RegisterRoom>(registerMessage(roomIdFor(1), 1, version)));
        const auto rejected = firstOf<rl::RoomRejected>(controlsTo(effects, 1));
        REQUIRE(rejected.has_value());
        CHECK(rejected->reason == rl::RoomRejectReason::BadVersion);
    }
    CHECK_FALSE(f.table.hasRoom(roomIdFor(1)));
    CHECK(f.table.roleOf(1) == Role::None);
}

TEST_CASE("relay table: a second host with another secret gets RoomTaken") {
    Fixture f;
    f.registerHost(1, roomIdFor(1), 1);
    const auto reply = f.registerHost(2, roomIdFor(1), 2);
    REQUIRE(reply.has_value());
    CHECK(std::get<rl::RoomRejected>(*reply).reason == rl::RoomRejectReason::RoomTaken);
    CHECK(f.table.roleOf(2) == Role::None);
    CHECK(f.table.roleOf(1) == Role::Host);
}

TEST_CASE("relay table: re-register with another secret while the host is away gets BadAuth") {
    Fixture f;
    f.registerHost(1, roomIdFor(1), 1);
    f.table.onDisconnect(1);
    const auto reply = f.registerHost(2, roomIdFor(1), 9);
    REQUIRE(reply.has_value());
    CHECK(std::get<rl::RoomRejected>(*reply).reason == rl::RoomRejectReason::BadAuth);
    CHECK(f.table.hasRoom(roomIdFor(1)));
    CHECK_FALSE(f.table.hostPresent(roomIdFor(1)));
}

TEST_CASE("relay table: the room cap rejects with RelayFull") {
    RoomTableConfig config;
    config.maxRooms = 2;
    Fixture f(config);
    CHECK(std::holds_alternative<rl::RoomRegistered>(*f.registerHost(1, roomIdFor(1))));
    CHECK(std::holds_alternative<rl::RoomRegistered>(*f.registerHost(2, roomIdFor(2))));
    const auto reply = f.registerHost(3, roomIdFor(3));
    CHECK(std::get<rl::RoomRejected>(*reply).reason == rl::RoomRejectReason::RelayFull);
    // Resuming an existing room is not a new room.
    f.table.onDisconnect(1);
    CHECK(std::holds_alternative<rl::RoomRegistered>(*f.registerHost(4, roomIdFor(1))));
}

TEST_CASE("relay table: a malformed room id on register is a protocol violation") {
    Fixture f;
    const auto effects = f.table.registerRoom(1, "", std::get<rl::RegisterRoom>(registerMessage("short")));
    const auto closes = closesIn(effects);
    REQUIRE(closes.size() == 1);
    CHECK(closes[0].reason == DisconnectReason::ProtocolViolation);
    CHECK_FALSE(f.table.hasRoom("short"));
}

TEST_CASE("relay table: joins get the lowest free peer id and the host is told") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    for (rl::RelayPeerId expected = 1; expected <= rl::kRelayMaxPeersPerRoom; ++expected) {
        const ConnId conn = 10 + expected;
        const auto effects = f.table.joinRoom(conn, "", std::get<rl::JoinRoom>(joinMessage(roomIdFor(1))));
        const auto result = firstOf<rl::JoinResult>(controlsTo(effects, conn));
        REQUIRE(result.has_value());
        CHECK(result->ok);
        CHECK(result->reason == rl::JoinRoomFailure::None);
        CHECK(result->peerId == expected);
        const auto joined = firstOf<rl::PeerJoined>(controlsTo(effects, 1));
        REQUIRE(joined.has_value());
        CHECK(joined->peerId == expected);
        CHECK(f.table.peerIdOf(conn) == expected);
    }
    CHECK(f.table.clientCount(roomIdFor(1)) == rl::kRelayMaxPeersPerRoom);
}

TEST_CASE("relay table: join failures are unknown room, room full, bad version and host absent") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    CHECK(f.joinResult(20, roomIdFor(99)).reason == rl::JoinRoomFailure::NoSuchRoom);
    CHECK(f.joinResult(21, "bad").reason == rl::JoinRoomFailure::NoSuchRoom);
    {
        const auto effects = f.table.joinRoom(22, "", std::get<rl::JoinRoom>(joinMessage(roomIdFor(1), 77)));
        CHECK(firstOf<rl::JoinResult>(controlsTo(effects, 22))->reason == rl::JoinRoomFailure::BadVersion);
    }
    for (unsigned i = 0; i < rl::kRelayMaxPeersPerRoom; ++i) CHECK(f.join(30 + i, roomIdFor(1)) != rl::kRelayBroadcastPeer);
    const rl::JoinResult full = f.joinResult(40, roomIdFor(1));
    CHECK_FALSE(full.ok);
    CHECK(full.reason == rl::JoinRoomFailure::RoomFull);
    CHECK(full.peerId == rl::kRelayBroadcastPeer);

    f.table.onDisconnect(1);
    CHECK(f.joinResult(41, roomIdFor(1)).reason == rl::JoinRoomFailure::HostAbsent);
    CHECK(f.table.roleOf(40) == Role::None);
}

TEST_CASE("relay table: peer ids are reused lowest first") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    CHECK(f.join(11, roomIdFor(1)) == 1);
    CHECK(f.join(12, roomIdFor(1)) == 2);
    CHECK(f.join(13, roomIdFor(1)) == 3);
    f.table.onDisconnect(13);
    f.table.onDisconnect(11);
    CHECK(f.join(14, roomIdFor(1)) == 1);
    CHECK(f.join(15, roomIdFor(1)) == 3);
    CHECK(f.join(16, roomIdFor(1)) == 4);
}

TEST_CASE("relay table: a client leaving yields PeerLeft for the host") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    f.join(12, roomIdFor(1));
    const auto effects = f.table.onDisconnect(12, rl::PeerLeftReason::TimedOut);
    const auto left = firstOf<rl::PeerLeft>(controlsTo(effects, 1));
    REQUIRE(left.has_value());
    CHECK(left->peerId == 2);
    CHECK(left->reason == rl::PeerLeftReason::TimedOut);
    CHECK(f.table.clientCount(roomIdFor(1)) == 1);
    CHECK(f.table.roleOf(12) == Role::None);
    CHECK(f.table.onDisconnect(12).empty());  // idempotent
}

TEST_CASE("relay table: a host leaving keeps clients and drops their frames until it returns") {
    Fixture f;
    f.registerHost(1, roomIdFor(1), 5);
    CHECK(f.join(11, roomIdFor(1)) == 1);
    CHECK(f.join(12, roomIdFor(1)) == 2);
    const auto leaving = f.table.onDisconnect(1);
    CHECK(leaving.empty());
    CHECK(f.table.roleOf(11) == Role::Client);

    const auto dropped = f.table.route(11, Delivery::Reliable, payload(10));
    CHECK(dropped.dropped == DropReason::HostAbsent);
    CHECK(dropped.effects.empty());

    f.clock->advance(10s);
    CHECK(f.table.tick().empty());
    const auto effects = f.table.registerRoom(7, "", std::get<rl::RegisterRoom>(registerMessage(roomIdFor(1), 5)));
    const auto replies = controlsTo(effects, 7);
    REQUIRE(replies.size() == 3);
    CHECK(std::get<rl::RoomRegistered>(replies[0]).resumed);
    CHECK(std::get<rl::PeerJoined>(replies[1]).peerId == 1);
    CHECK(std::get<rl::PeerJoined>(replies[2]).peerId == 2);
    CHECK(f.table.peerIdOf(11) == 1);

    const auto forwarded = f.table.route(11, Delivery::Reliable, payload(10));
    REQUIRE_FALSE(forwarded.dropped.has_value());
    CHECK(gamesIn(forwarded.effects).front().to == 7);
}

TEST_CASE("relay table: the room closes and its clients are closed when the TTL expires") {
    RoomTableConfig config;
    config.roomTtlWithoutHostMs = 30'000;
    Fixture f(config);
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    f.join(12, roomIdFor(1));
    f.table.onDisconnect(1);

    f.clock->advance(29'999ms);
    CHECK(f.table.tick().empty());
    CHECK(f.table.hasRoom(roomIdFor(1)));
    f.clock->advance(1ms);
    const auto closes = closesIn(f.table.tick());
    REQUIRE(closes.size() == 2);
    for (const auto& c : closes) CHECK(c.reason == DisconnectReason::RoomExpired);
    CHECK_FALSE(f.table.hasRoom(roomIdFor(1)));
    CHECK(f.table.connectionCount() == 0);
    CHECK(f.table.stats().roomsExpired == 1);

    // The id is free again, with any secret.
    CHECK(std::holds_alternative<rl::RoomRegistered>(*f.registerHost(2, roomIdFor(1), 77)));
}

TEST_CASE("relay table: the same secret registering again while the old host is live takes the room over") {
    Fixture f;
    f.registerHost(1, roomIdFor(1), 3);
    f.join(11, roomIdFor(1));
    const auto effects = f.table.registerRoom(2, "", std::get<rl::RegisterRoom>(registerMessage(roomIdFor(1), 3)));
    const auto closes = closesIn(effects);
    REQUIRE(closes.size() == 1);
    CHECK(closes[0].to == 1);
    CHECK(closes[0].reason == DisconnectReason::HostReplaced);
    CHECK(std::get<rl::RoomRegistered>(controlsTo(effects, 2).front()).resumed);
    CHECK(f.table.roleOf(1) == Role::None);
    CHECK(f.table.roleOf(2) == Role::Host);
    CHECK(f.table.peerIdOf(11) == 1);
}

TEST_CASE("relay table: client frames reach the host with the peer id prefix") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    CHECK(f.join(12, roomIdFor(1)) == 2);

    const Bytes body = {std::byte{1}, std::byte{2}, std::byte{3}};
    const auto result = f.table.route(12, Delivery::Unreliable, body);
    REQUIRE_FALSE(result.dropped.has_value());
    const auto games = gamesIn(result.effects);
    REQUIRE(games.size() == 1);
    CHECK(games[0].to == 1);
    CHECK(games[0].delivery == Delivery::Unreliable);
    CHECK(games[0].frame == Bytes{std::byte{2}, std::byte{0}, std::byte{1}, std::byte{2}, std::byte{3}});
}

TEST_CASE("relay table: host frames go to the addressed client with the envelope stripped") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    f.join(12, roomIdFor(1));
    const Bytes body = {std::byte{9}, std::byte{8}};

    const auto toSecond = f.table.route(1, Delivery::Reliable, withEnvelope(2, body));
    const auto games = gamesIn(toSecond.effects);
    REQUIRE(games.size() == 1);
    CHECK(games[0].to == 12);
    CHECK(games[0].delivery == Delivery::Reliable);
    CHECK(games[0].frame == body);
}

TEST_CASE("relay table: a broadcast reaches every client once with the delivery class kept") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    f.join(12, roomIdFor(1));
    f.join(13, roomIdFor(1));
    f.table.onDisconnect(12);
    const Bytes body = {std::byte{7}};
    for (const Delivery d : {Delivery::Reliable, Delivery::Unreliable}) {
        const auto result = f.table.route(1, d, withEnvelope(rl::kRelayBroadcastPeer, body));
        const auto games = gamesIn(result.effects);
        REQUIRE(games.size() == 2);
        CHECK(games[0].to == 11);
        CHECK(games[1].to == 13);
        for (const auto& g : games) {
            CHECK(g.delivery == d);
            CHECK(g.frame == body);
        }
    }
}

TEST_CASE("relay table: unknown targets, short frames and empty broadcasts are dropped and counted") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    CHECK(f.table.route(1, Delivery::Reliable, withEnvelope(rl::kRelayBroadcastPeer, payload(4))).dropped == DropReason::NoRecipients);
    f.join(11, roomIdFor(1));
    CHECK(f.table.route(1, Delivery::Reliable, withEnvelope(5, payload(4))).dropped == DropReason::UnknownPeer);
    CHECK(f.table.route(1, Delivery::Reliable, withEnvelope(0, payload(4))).dropped == DropReason::UnknownPeer);
    CHECK(f.table.route(1, Delivery::Reliable, Bytes{}).dropped == DropReason::TooShort);
    CHECK(f.table.route(1, Delivery::Reliable, Bytes{std::byte{1}}).dropped == DropReason::TooShort);
    CHECK(f.table.route(1, Delivery::Reliable, withEnvelope(1, {})).dropped == DropReason::TooShort);

    const auto stats = f.table.stats();
    CHECK(stats.dropped(DropReason::NoRecipients) == 1);
    CHECK(stats.dropped(DropReason::UnknownPeer) == 2);
    CHECK(stats.dropped(DropReason::TooShort) == 3);
    CHECK(stats.framesForwarded == 0);
}

TEST_CASE("relay table: game frames before registering are dropped and counted") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    const auto result = f.table.route(99, Delivery::Reliable, payload(10));
    CHECK(result.dropped == DropReason::Unregistered);
    CHECK(result.effects.empty());
    // A connection whose join was refused is still unregistered.
    f.joinResult(50, roomIdFor(2));
    CHECK(f.table.route(50, Delivery::Unreliable, payload(10)).dropped == DropReason::Unregistered);
    CHECK(f.table.stats().dropped(DropReason::Unregistered) == 2);
}

TEST_CASE("relay table: the host never receives frames of a room it does not own") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    f.registerHost(2, roomIdFor(2));
    f.join(11, roomIdFor(1));
    f.join(21, roomIdFor(2));
    for (const ConnId client : {ConnId{11}, ConnId{21}}) {
        const auto games = gamesIn(f.table.route(client, Delivery::Reliable, payload(5)).effects);
        REQUIRE(games.size() == 1);
        CHECK(games[0].to == (client == 11 ? 1u : 2u));
    }
    // Host 1 addressing peer 1 reaches only its own client 11, never client 21 (also peer id 1 in room 2).
    const auto games = gamesIn(f.table.route(1, Delivery::Reliable, withEnvelope(1, payload(5))).effects);
    REQUIRE(games.size() == 1);
    CHECK(games[0].to == 11);
}

TEST_CASE("relay table: size caps apply to a client's frame as sent and to a host's frame with its envelope") {
    RoomTableConfig config;
    config.peerBytesPerSecond = 0;  // no rate limit here
    Fixture f(config);
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));

    // A client may use the full limits: the relay adds the envelope toward the host.
    const auto fromClient = f.table.route(11, Delivery::Unreliable, payload(rl::kRelayMaxDatagram));
    CHECK_FALSE(fromClient.dropped.has_value());
    const auto games = gamesIn(fromClient.effects);
    REQUIRE(games.size() == 1);
    CHECK(games[0].frame.size() == rl::kRelayMaxDatagram + rl::kRelayEnvelopeBytes);
    CHECK(f.table.route(11, Delivery::Unreliable, payload(rl::kRelayMaxDatagram + 1)).dropped == DropReason::Oversized);
    // A host datagram carries the envelope itself.
    CHECK_FALSE(f.table.route(1, Delivery::Unreliable, withEnvelope(1, payload(rl::kRelayMaxDatagram - rl::kRelayEnvelopeBytes))).dropped.has_value());
    CHECK(f.table.route(1, Delivery::Unreliable, withEnvelope(1, payload(rl::kRelayMaxDatagram - rl::kRelayEnvelopeBytes + 1))).dropped == DropReason::Oversized);
    // Reliable.
    CHECK_FALSE(f.table.route(11, Delivery::Reliable, payload(rl::kRelayMaxReliablePacket)).dropped.has_value());
    CHECK(f.table.route(11, Delivery::Reliable, payload(rl::kRelayMaxReliablePacket + 1)).dropped == DropReason::Oversized);
    CHECK_FALSE(f.table.route(1, Delivery::Reliable, withEnvelope(1, payload(rl::kRelayMaxReliablePacket - rl::kRelayEnvelopeBytes))).dropped.has_value());
    CHECK(f.table.route(1, Delivery::Reliable, withEnvelope(1, payload(rl::kRelayMaxReliablePacket - rl::kRelayEnvelopeBytes + 1))).dropped == DropReason::Oversized);
    CHECK(f.table.stats().dropped(DropReason::Oversized) == 4);
}

TEST_CASE("relay table: the token bucket allows a burst of two seconds, refills, and drops a sustained flood") {
    RoomTableConfig config;
    config.peerBytesPerSecond = 1000;
    config.maxReliableBytes = 100;  // keeps the bucket's minimum capacity below the 2 s burst
    config.maxDatagramBytes = 100;
    config.abuseDropsPerSecond = 0;
    config.roomBytesPerSecond = 1'000'000;
    Fixture f(config);
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    const Bytes frame = payload(98);  // 100 bytes with the envelope is irrelevant: the bucket charges what the peer sent

    int passed = 0;
    for (int i = 0; i < 100; ++i)
        if (!f.table.route(11, Delivery::Reliable, frame).dropped) ++passed;
    CHECK(passed == 20);  // 2000 bytes of burst / 98
    CHECK(f.table.route(11, Delivery::Reliable, frame).dropped == DropReason::RateLimited);

    f.clock->advance(1s);  // +1000 bytes
    passed = 0;
    for (int i = 0; i < 100; ++i)
        if (!f.table.route(11, Delivery::Reliable, frame).dropped) ++passed;
    CHECK(passed == 10);

    // Sustained flood at twice the rate: about the rate gets through.
    f.clock->advance(10s);
    int sent = 0;
    passed = 0;
    for (int tick = 0; tick < 200; ++tick) {
        f.clock->advance(49ms);  // 98 bytes per 49 ms = 2000 B/s offered
        ++sent;
        if (!f.table.route(11, Delivery::Reliable, frame).dropped) ++passed;
    }
    CHECK(passed < sent);
    CHECK(passed >= 100);
    CHECK(passed <= 121);  // ~1000 B/s for ~9.8 s plus the 2000 byte burst
}

TEST_CASE("relay table: the host has a larger bucket than a client and each peer has its own") {
    RoomTableConfig config;
    config.peerBytesPerSecond = 1000;
    config.maxReliableBytes = 100;
    config.maxDatagramBytes = 100;
    config.maxPeersPerRoom = 4;
    config.abuseDropsPerSecond = 0;
    config.roomBytesPerSecond = 1'000'000;
    Fixture f(config);
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    f.join(12, roomIdFor(1));
    const Bytes frame = payload(98);

    for (int i = 0; i < 20; ++i) CHECK_FALSE(f.table.route(11, Delivery::Reliable, frame).dropped.has_value());
    CHECK(f.table.route(11, Delivery::Reliable, frame).dropped == DropReason::RateLimited);
    // Another client is unaffected.
    CHECK_FALSE(f.table.route(12, Delivery::Reliable, frame).dropped.has_value());
    // The host's bucket is maxPeersPerRoom times larger: 8000 bytes of burst.
    const Bytes hostFrame = withEnvelope(1, payload(96));
    int hostPassed = 0;
    for (int i = 0; i < 100; ++i)
        if (!f.table.route(1, Delivery::Reliable, hostFrame).dropped) ++hostPassed;
    CHECK(hostPassed == 81);  // 8000 bytes of burst / 98
}

TEST_CASE("relay table: the room cap bounds what a room forwards") {
    RoomTableConfig config;
    config.peerBytesPerSecond = 1'000'000;
    config.maxReliableBytes = 100;
    config.maxDatagramBytes = 100;
    config.roomBytesPerSecond = 1000;  // burst 2000 bytes
    config.abuseDropsPerSecond = 0;
    Fixture f(config);
    f.registerHost(1, roomIdFor(1));
    for (ConnId c = 11; c <= 15; ++c) f.join(c, roomIdFor(1));
    const Bytes broadcast = withEnvelope(rl::kRelayBroadcastPeer, payload(98));  // 98 bytes x 5 clients = 490 per frame
    int passed = 0;
    for (int i = 0; i < 20; ++i)
        if (!f.table.route(1, Delivery::Reliable, broadcast).dropped) ++passed;
    CHECK(passed == 4);  // 2000 / 490
    CHECK(f.table.stats().dropped(DropReason::RoomByteCap) == 16);
    f.clock->advance(1s);
    CHECK_FALSE(f.table.route(1, Delivery::Reliable, broadcast).dropped.has_value());
}

TEST_CASE("relay table: a peer that keeps violating the limits is disconnected") {
    RoomTableConfig config;
    config.abuseDropsPerSecond = 5;
    Fixture f(config);
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    for (int i = 0; i < 5; ++i) CHECK(f.table.route(11, Delivery::Unreliable, payload(rl::kRelayMaxDatagram + 1)).effects.empty());
    const auto kick = f.table.route(11, Delivery::Unreliable, payload(rl::kRelayMaxDatagram + 1));
    const auto closes = closesIn(kick.effects);
    REQUIRE(closes.size() == 1);
    CHECK(closes[0].to == 11);
    CHECK(closes[0].reason == DisconnectReason::Oversized);
    const auto left = firstOf<rl::PeerLeft>(controlsTo(kick.effects, 1));
    REQUIRE(left.has_value());
    CHECK(left->reason == rl::PeerLeftReason::Oversized);
    CHECK(f.table.roleOf(11) == Role::None);
    CHECK(f.table.stats().peersKicked == 1);
}

TEST_CASE("relay table: drops spread over time do not disconnect") {
    RoomTableConfig config;
    config.abuseDropsPerSecond = 5;
    Fixture f(config);
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    for (int i = 0; i < 50; ++i) {
        f.clock->advance(300ms);
        CHECK(f.table.route(11, Delivery::Unreliable, payload(rl::kRelayMaxDatagram + 1)).effects.empty());
    }
    CHECK(f.table.roleOf(11) == Role::Client);
}

TEST_CASE("relay table: repeated failures from one source lock it out and the lockout expires") {
    RoomTableConfig config;
    config.lockoutMaxFailures = 10;
    config.lockoutWindowMs = 60'000;
    config.lockoutDurationMs = 300'000;
    Fixture f(config);
    f.registerHost(1, roomIdFor(1));
    for (unsigned i = 0; i < 10; ++i) CHECK(f.joinResult(100 + i, roomIdFor(50 + i), "1.2.3.4").reason == rl::JoinRoomFailure::NoSuchRoom);
    CHECK(f.table.isLockedOut("1.2.3.4"));
    CHECK(f.table.stats().lockouts == 1);

    // Even a correct join is refused, and registering is refused by closing the connection.
    CHECK(f.joinResult(200, roomIdFor(1), "1.2.3.4").reason == rl::JoinRoomFailure::LockedOut);
    const auto closes = closesIn(f.table.registerRoom(201, "1.2.3.4", std::get<rl::RegisterRoom>(registerMessage(roomIdFor(9)))));
    REQUIRE(closes.size() == 1);
    CHECK(closes[0].reason == DisconnectReason::LockedOut);
    CHECK_FALSE(f.table.hasRoom(roomIdFor(9)));
    CHECK(f.table.stats().lockedOutAttempts == 2);

    // Another source is unaffected.
    CHECK(f.join(202, roomIdFor(1), "5.6.7.8") != rl::kRelayBroadcastPeer);

    f.clock->advance(299s);
    CHECK(f.table.isLockedOut("1.2.3.4"));
    f.clock->advance(1s);
    CHECK_FALSE(f.table.isLockedOut("1.2.3.4"));
    CHECK(f.join(203, roomIdFor(1), "1.2.3.4") != rl::kRelayBroadcastPeer);
}

TEST_CASE("relay table: failures outside the window do not add up") {
    RoomTableConfig config;
    config.lockoutMaxFailures = 3;
    config.lockoutWindowMs = 60'000;
    Fixture f(config);
    for (int round = 0; round < 10; ++round) {
        f.joinResult(100 + round * 2, roomIdFor(77), "9.9.9.9");
        f.joinResult(101 + round * 2, roomIdFor(77), "9.9.9.9");
        f.clock->advance(61s);
    }
    CHECK_FALSE(f.table.isLockedOut("9.9.9.9"));
}

TEST_CASE("relay table: register failures count toward the lockout, other rejections do not") {
    RoomTableConfig config;
    config.lockoutMaxFailures = 3;
    config.maxRooms = 1;
    Fixture f(config);
    f.registerHost(1, roomIdFor(1), 1);
    // RelayFull and HostAbsent and RoomFull are not the client's fault.
    for (unsigned i = 0; i < 10; ++i) {
        f.registerHost(10 + i, roomIdFor(2 + i), 1, "6.6.6.6");  // RelayFull
    }
    CHECK_FALSE(f.table.isLockedOut("6.6.6.6"));
    // RoomTaken counts.
    for (unsigned i = 0; i < 3; ++i) f.registerHost(30 + i, roomIdFor(1), 2, "6.6.6.6");
    CHECK(f.table.isLockedOut("6.6.6.6"));
}

TEST_CASE("relay table: the lockout table is bounded and never evicts a locked source for a new one") {
    RoomTableConfig config;
    config.lockoutMaxFailures = 2;
    config.maxTrackedSources = 8;
    Fixture f(config);
    for (unsigned i = 0; i < 1000; ++i) {
        f.joinResult(1000 + i, roomIdFor(1), "10.0.0." + std::to_string(i % 250) + "/" + std::to_string(i));
        CHECK(f.table.stats().trackedSources <= 8);
        f.clock->advance(1ms);
    }
    CHECK(f.table.stats().trackedSources <= 8);

    // Fill the table with locked sources; a ninth cannot be tracked and so cannot be locked, but the locked ones stay.
    Fixture g(config);
    for (unsigned s = 0; s < 8; ++s)
        for (unsigned k = 0; k < 2; ++k) g.joinResult(1 + s * 10 + k, roomIdFor(1), "L" + std::to_string(s));
    for (unsigned s = 0; s < 8; ++s) CHECK(g.table.isLockedOut("L" + std::to_string(s)));
    for (unsigned k = 0; k < 5; ++k) g.joinResult(500 + k, roomIdFor(1), "newcomer");
    CHECK(g.table.stats().trackedSources == 8);
    CHECK(g.table.stats().lockoutTableFull == 5);
    for (unsigned s = 0; s < 8; ++s) CHECK(g.table.isLockedOut("L" + std::to_string(s)));
    // Once locks and windows have expired the table is swept.
    g.clock->advance(24h);
    g.table.tick();
    CHECK(g.table.stats().trackedSources == 0);
}

TEST_CASE("relay table: an empty source is never tracked") {
    Fixture f;
    for (unsigned i = 0; i < 100; ++i) CHECK(f.joinResult(100 + i, roomIdFor(5), "").reason == rl::JoinRoomFailure::NoSuchRoom);
    CHECK(f.table.stats().trackedSources == 0);
}

TEST_CASE("relay table: events never carry a room id that is not a valid one") {
    Fixture f;
    const std::string hostile = "\"}{\"evil\":\"" + std::string(500, 'x');
    CHECK_FALSE(f.joinResult(1, hostile).ok);
    f.table.registerRoom(2, "", std::get<rl::RegisterRoom>(registerMessage(hostile, 1, 99)));
    const auto events = f.table.drainEvents();
    REQUIRE(events.size() == 2);
    for (const auto& e : events) CHECK(e.roomId.empty());
}

TEST_CASE("relay table: a registered connection registering or joining again is a protocol violation") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    const auto again = f.table.joinRoom(11, "", std::get<rl::JoinRoom>(joinMessage(roomIdFor(1))));
    const auto closes = closesIn(again);
    REQUIRE(closes.size() == 1);
    CHECK(closes[0].reason == DisconnectReason::ProtocolViolation);
    CHECK(firstOf<rl::PeerLeft>(controlsTo(again, 1)).has_value());
    CHECK(f.table.roleOf(11) == Role::None);
}

TEST_CASE("relay table: events describe what happened") {
    Fixture f;
    f.registerHost(1, roomIdFor(1));
    f.join(11, roomIdFor(1));
    f.table.onDisconnect(11);
    f.table.onDisconnect(1);
    const auto events = f.table.drainEvents();
    REQUIRE(events.size() == 4);
    CHECK(events[0].kind == wpiemu::relay::RoomEvent::Kind::RoomCreated);
    CHECK(events[1].kind == wpiemu::relay::RoomEvent::Kind::PeerJoined);
    CHECK(events[2].kind == wpiemu::relay::RoomEvent::Kind::PeerLeft);
    CHECK(events[3].kind == wpiemu::relay::RoomEvent::Kind::HostLeft);
    CHECK(events[0].roomId == roomIdFor(1));
    CHECK(f.table.drainEvents().empty());
}

namespace {

std::string describe(const wpiemu::relay::Effects& effects) {
    std::ostringstream out;
    for (const auto& e : effects) {
        if (const auto* c = std::get_if<wpiemu::relay::SendControl>(&e)) out << "C" << c->to << ":" << c->message.index() << ";";
        else if (const auto* g = std::get_if<wpiemu::relay::SendGame>(&e)) {
            out << "G" << g->to << ":" << static_cast<int>(g->delivery) << ":";
            for (const std::byte b : g->frame) out << static_cast<int>(b) << ",";
            out << ";";
        } else if (const auto* x = std::get_if<wpiemu::relay::CloseConnection>(&e)) out << "X" << x->to << ":" << static_cast<int>(x->reason) << ";";
    }
    return out.str();
}

// A scripted pseudo-random sequence of table calls; returns a transcript of every decision.
std::string runScript(std::uint32_t seed) {
    RoomTableConfig config;
    config.maxRooms = 4;
    config.maxPeersPerRoom = 3;
    config.peerBytesPerSecond = 2000;
    config.maxReliableBytes = 300;
    config.roomTtlWithoutHostMs = 5000;
    config.lockoutMaxFailures = 4;
    Fixture f(config);
    std::mt19937 rng(seed);
    std::ostringstream transcript;
    for (int step = 0; step < 3000; ++step) {
        const ConnId conn = rng() % 20 + 1;
        const std::string room = roomIdFor(rng() % 6);
        const std::string source = "src" + std::to_string(rng() % 5);
        wpiemu::relay::Effects effects;
        switch (rng() % 7) {
            case 0: effects = f.table.registerRoom(conn, source, std::get<rl::RegisterRoom>(registerMessage(room, static_cast<std::uint8_t>(rng() % 3)))); break;
            case 1:
            case 2: effects = f.table.joinRoom(conn, source, std::get<rl::JoinRoom>(joinMessage(room))); break;
            case 3: effects = f.table.onDisconnect(conn); break;
            case 4: f.clock->advance(std::chrono::milliseconds(rng() % 3000)); effects = f.table.tick(); break;
            default: {
                Bytes frame(rng() % 40);
                for (auto& b : frame) b = static_cast<std::byte>(rng() % 4);
                const auto result = f.table.route(conn, rng() % 2 ? Delivery::Reliable : Delivery::Unreliable, frame);
                transcript << (result.dropped ? static_cast<int>(*result.dropped) : -1) << "|";
                effects = result.effects;
            }
        }
        transcript << describe(effects) << "\n";
    }
    const auto stats = f.table.stats();
    transcript << stats.rooms << "," << stats.peers << "," << stats.framesForwarded << "," << stats.bytesForwarded << "," << stats.lockouts;
    return transcript.str();
}

}  // namespace

TEST_CASE("relay table: the same event sequence yields the same decisions") {
    const std::string first = runScript(42);
    CHECK(first == runScript(42));
    CHECK(first != runScript(43));
    CHECK(first.size() > 1000);
}
