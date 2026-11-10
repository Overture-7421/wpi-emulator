#include <doctest/doctest.h>

#include <algorithm>
#include <atomic>
#include <map>
#include <random>
#include <thread>

#include "relay_test_support.h"

using namespace std::chrono_literals;
using namespace relaytest;
using wpiemu::relay::DropReason;
using wpiemu::relay::RelayServerConfig;

namespace {

Bytes payload(std::size_t size, std::uint8_t fill = 0xAB) { return Bytes(size, static_cast<std::byte>(fill)); }

Bytes withEnvelope(rl::RelayPeerId target, const Bytes& body) {
    Bytes out{static_cast<std::byte>(target & 0xFF), static_cast<std::byte>(target >> 8)};
    out.insert(out.end(), body.begin(), body.end());
    return out;
}

template <class T>
std::optional<T> replyOf(FakePeer& peer) {
    return firstOf<T>(peer.takeControl());
}

}  // namespace

TEST_CASE("relay server: register, join and the announcements") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    const auto registered = replyOf<rl::RoomRegistered>(*host);
    REQUIRE(registered.has_value());
    CHECK_FALSE(registered->resumed);

    auto client = h.client(roomIdFor(1));
    const auto joined = replyOf<rl::JoinResult>(*client);
    REQUIRE(joined.has_value());
    CHECK(joined->ok);
    CHECK(joined->peerId == 1);
    const auto announced = replyOf<rl::PeerJoined>(*host);
    REQUIRE(announced.has_value());
    CHECK(announced->peerId == 1);

    client->disconnect();
    h.pump();
    const auto left = replyOf<rl::PeerLeft>(*host);
    REQUIRE(left.has_value());
    CHECK(left->peerId == 1);
    CHECK(h.server->stats().peers == 0);
    CHECK(h.server->stats().rooms == 1);
}

TEST_CASE("relay server: every reject reason reaches the peer") {
    Harness h;
    auto host = h.host(roomIdFor(1), 1);
    host->takeControl();

    auto taken = h.host(roomIdFor(1), 2, "10.0.0.9");
    CHECK(replyOf<rl::RoomRejected>(*taken)->reason == rl::RoomRejectReason::RoomTaken);

    auto badVersion = h.connect("10.0.0.10");
    badVersion->sendControl(registerMessage(roomIdFor(2), 1, 99));
    h.pump();
    CHECK(replyOf<rl::RoomRejected>(*badVersion)->reason == rl::RoomRejectReason::BadVersion);

    auto missing = h.client(roomIdFor(77), "10.0.0.11");
    CHECK(replyOf<rl::JoinResult>(*missing)->reason == rl::JoinRoomFailure::NoSuchRoom);

    std::vector<std::shared_ptr<FakePeer>> clients;
    for (unsigned i = 0; i < rl::kRelayMaxPeersPerRoom; ++i) clients.push_back(h.client(roomIdFor(1), "10.0.1." + std::to_string(i)));
    auto full = h.client(roomIdFor(1), "10.0.2.1");
    CHECK(replyOf<rl::JoinResult>(*full)->reason == rl::JoinRoomFailure::RoomFull);

    host->disconnect();
    h.pump();
    auto absent = h.client(roomIdFor(1), "10.0.2.2");
    CHECK(replyOf<rl::JoinResult>(*absent)->reason == rl::JoinRoomFailure::HostAbsent);
    auto badAuth = h.host(roomIdFor(1), 9, "10.0.2.3");
    CHECK(replyOf<rl::RoomRejected>(*badAuth)->reason == rl::RoomRejectReason::BadAuth);
}

TEST_CASE("relay server: the room cap and RelayFull") {
    RelayServerConfig config;
    config.table.maxRooms = 3;
    Harness h(config);
    for (unsigned i = 0; i < 3; ++i) CHECK(replyOf<rl::RoomRegistered>(*h.host(roomIdFor(i))).has_value());
    auto extra = h.host(roomIdFor(10), 1, "10.0.0.5");
    CHECK(replyOf<rl::RoomRejected>(*extra)->reason == rl::RoomRejectReason::RelayFull);
    CHECK(h.server->stats().rooms == 3);
}

TEST_CASE("relay server: client to host and host to client forwarding with the envelope") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    auto a = h.client(roomIdFor(1), "10.0.0.2");
    auto b = h.client(roomIdFor(1), "10.0.0.3");
    host->takeControl();
    a->takeControl();
    b->takeControl();

    a->sendGame(Delivery::Reliable, bytesOf("hello"));
    b->sendGame(Delivery::Unreliable, bytesOf("dgram"));
    h.pump();
    const auto atHost = host->takeGame();
    REQUIRE(atHost.size() == 2);
    CHECK(atHost[0].delivery == Delivery::Reliable);
    CHECK(atHost[0].bytes == withEnvelope(1, bytesOf("hello")));
    CHECK(atHost[1].delivery == Delivery::Unreliable);
    CHECK(atHost[1].bytes == withEnvelope(2, bytesOf("dgram")));

    host->sendGame(Delivery::Reliable, withEnvelope(2, bytesOf("to-b")));
    host->sendGame(Delivery::Unreliable, withEnvelope(rl::kRelayBroadcastPeer, bytesOf("all")));
    h.pump();
    const auto atA = a->takeGame();
    const auto atB = b->takeGame();
    REQUIRE(atA.size() == 1);
    CHECK(atA[0].bytes == bytesOf("all"));
    CHECK(atA[0].delivery == Delivery::Unreliable);
    REQUIRE(atB.size() == 2);
    CHECK(atB[0].bytes == bytesOf("to-b"));
    CHECK(atB[0].delivery == Delivery::Reliable);
    CHECK(atB[1].bytes == bytesOf("all"));
    CHECK(h.server->stats().framesForwarded == 5);
}

TEST_CASE("relay server: unknown peers, short frames and frames before registering are dropped") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    auto client = h.client(roomIdFor(1));
    host->takeControl();
    client->takeControl();

    host->sendGame(Delivery::Reliable, withEnvelope(9, bytesOf("x")));
    host->sendGame(Delivery::Reliable, Bytes{std::byte{1}});
    auto stranger = h.connect("10.0.0.7");
    stranger->sendGame(Delivery::Reliable, bytesOf("x"));
    h.pump();
    CHECK(client->takeGame().empty());
    CHECK(host->takeGame().empty());
    const auto stats = h.server->stats();
    CHECK(stats.dropped(DropReason::UnknownPeer) == 1);
    CHECK(stats.dropped(DropReason::TooShort) == 1);
    CHECK(stats.dropped(DropReason::Unregistered) == 1);
    CHECK_FALSE(stranger->closedByRelay);  // dropped, not disconnected
    CHECK(host->closedByRelay == false);
}

TEST_CASE("relay server: control abuse disconnects") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    host->takeControl();

    struct Case {
        const char* name;
        std::function<void(FakePeer&)> act;
    };
    const std::vector<Case> cases = {
        {"garbage", [](FakePeer& p) { p.sendControlRaw(bytesOf("\x07garbage that is not a frame")); }},
        {"empty-ish", [](FakePeer& p) { p.sendControlRaw(Bytes{std::byte{0}}); }},
        {"wrong type", [](FakePeer& p) { p.sendControl(rl::RoomRegistered{}); }},
        {"another wrong type", [](FakePeer& p) { p.sendControl(rl::PeerLeft{}); }},
        {"oversize", [](FakePeer& p) { p.sendControlRaw(Bytes(wpiemu::net::kMaxControlFrameBytes + 1, std::byte{0x41})); }},
        {"unknown type", [](FakePeer& p) { p.sendControlRaw(Bytes{std::byte{1}, std::byte{0xFF}, std::byte{0x7F}, std::byte{0}}); }},
    };
    std::uint64_t violations = 0;
    for (const Case& c : cases) {
        INFO(c.name);
        auto peer = h.connect("10.0.3." + std::to_string(violations + 1));
        c.act(*peer);
        h.pump();
        CHECK(peer->closedByRelay);
        CHECK(peer->takeControl().empty());
        CHECK(h.server->stats().controlViolations == ++violations);
    }
    CHECK(h.server->stats().rooms == 1);
}

TEST_CASE("relay server: control frames from a registered connection disconnect it") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    auto client = h.client(roomIdFor(1));
    host->takeControl();
    client->sendControl(joinMessage(roomIdFor(1)));
    h.pump();
    CHECK(client->closedByRelay);
    CHECK(replyOf<rl::PeerLeft>(*host).has_value());

    host->sendControl(registerMessage(roomIdFor(2)));
    h.pump();
    CHECK(host->closedByRelay);
    CHECK(h.server->stats().controlViolations == 2);
}

TEST_CASE("relay server: a connection that never registers is dropped") {
    RelayServerConfig config;
    config.unregisteredTimeout = 5s;
    Harness h(config);
    auto idle = h.connect();
    auto registered = h.host(roomIdFor(1), 1, "10.0.0.8");
    h.advance(4s);
    CHECK_FALSE(idle->closedByRelay);
    h.advance(1100ms);
    CHECK(idle->closedByRelay);
    CHECK_FALSE(registered->closedByRelay);
    CHECK(h.server->stats().unregisteredTimeouts == 1);
    CHECK(h.server->stats().connections == 1);
}

TEST_CASE("relay server: a connection is dropped after too many refused attempts") {
    Harness h;
    auto peer = h.connect("10.0.0.20");
    for (int i = 0; i < 3; ++i) {
        peer->sendControl(joinMessage(roomIdFor(60 + i)));
        h.pump();
    }
    CHECK(peer->closedByRelay);
    const auto replies = peer->takeControl();
    CHECK(replies.size() == 3);  // each attempt was answered before the connection was closed
}

TEST_CASE("relay server: host TTL grace, resume with the same secret and expiry") {
    Harness h;
    auto host = h.host(roomIdFor(1), 4);
    auto a = h.client(roomIdFor(1), "10.0.0.2");
    auto b = h.client(roomIdFor(1), "10.0.0.3");
    host->disconnect();
    h.pump();
    host->takeControl();

    a->sendGame(Delivery::Reliable, bytesOf("lost"));
    h.advance(10s);
    CHECK(h.server->stats().dropped(DropReason::HostAbsent) == 1);
    CHECK_FALSE(a->closedByRelay);

    auto wrong = h.host(roomIdFor(1), 5, "10.0.0.50");
    CHECK(replyOf<rl::RoomRejected>(*wrong)->reason == rl::RoomRejectReason::BadAuth);

    auto back = h.host(roomIdFor(1), 4, "10.0.0.51");
    const auto replies = back->takeControl();
    REQUIRE(replies.size() == 3);
    CHECK(std::get<rl::RoomRegistered>(replies[0]).resumed);
    CHECK(std::get<rl::PeerJoined>(replies[1]).peerId == 1);
    CHECK(std::get<rl::PeerJoined>(replies[2]).peerId == 2);
    b->sendGame(Delivery::Reliable, bytesOf("again"));
    h.pump();
    const auto game = back->takeGame();
    REQUIRE(game.size() == 1);
    CHECK(game[0].bytes == withEnvelope(2, bytesOf("again")));
    CHECK(h.server->stats().roomsResumed == 1);

    // Expiry: the host leaves for good.
    back->disconnect();
    h.pump();
    h.advance(29s);
    CHECK_FALSE(a->closedByRelay);
    h.advance(1100ms);
    CHECK(a->closedByRelay);
    CHECK(b->closedByRelay);
    CHECK(h.server->stats().rooms == 0);
    CHECK(h.server->stats().roomsExpired == 1);
    CHECK(h.server->stats().connections == 0);
}

TEST_CASE("relay server: a host reconnecting before the old connection timed out takes over") {
    Harness h;
    auto oldHost = h.host(roomIdFor(1), 4);
    auto client = h.client(roomIdFor(1));
    auto newHost = h.host(roomIdFor(1), 4, "10.0.0.60");
    CHECK(oldHost->closedByRelay);
    const auto replies = newHost->takeControl();
    REQUIRE_FALSE(replies.empty());
    CHECK(std::get<rl::RoomRegistered>(replies[0]).resumed);
    client->sendGame(Delivery::Reliable, bytesOf("x"));
    h.pump();
    CHECK(newHost->takeGame().size() == 1);
    CHECK(oldHost->takeGame().empty());
}

TEST_CASE("relay server: failed joins from one address lock it out across connections") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    int lockedOut = 0;
    for (int i = 0; i < 14; ++i) {
        auto peer = h.client(roomIdFor(50), "203.0.113.9", 7000 + i);
        const auto result = replyOf<rl::JoinResult>(*peer);
        REQUIRE(result.has_value());
        if (result->reason == rl::JoinRoomFailure::LockedOut) ++lockedOut;
    }
    CHECK(lockedOut == 4);
    auto valid = h.client(roomIdFor(1), "203.0.113.9", 8000);
    CHECK(replyOf<rl::JoinResult>(*valid)->reason == rl::JoinRoomFailure::LockedOut);
    auto other = h.client(roomIdFor(1), "203.0.113.10", 8001);
    CHECK(replyOf<rl::JoinResult>(*other)->ok);
    CHECK(h.server->stats().lockouts == 1);

    h.advance(301s);
    auto later = h.client(roomIdFor(1), "203.0.113.9", 8002);
    CHECK(replyOf<rl::JoinResult>(*later)->ok);
}

TEST_CASE("relay server: loopback sources and unknown sources are never locked out") {
    Harness h;
    for (int i = 0; i < 30; ++i) {
        auto local = h.client(roomIdFor(50), "127.0.0.1", 7000 + i);
        CHECK(replyOf<rl::JoinResult>(*local)->reason == rl::JoinRoomFailure::NoSuchRoom);
        auto unknown = h.client(roomIdFor(50), "", 0);
        CHECK(replyOf<rl::JoinResult>(*unknown)->reason == rl::JoinRoomFailure::NoSuchRoom);
    }
    const auto stats = h.server->stats();
    CHECK(stats.lockouts == 0);
    CHECK(stats.unknownSources == 30);
    CHECK(stats.trackedSources == 0);
}

TEST_CASE("relay server: IPv6 sources are tracked per /64 and IPv4 mapped ones as IPv4") {
    Harness h;
    for (int i = 0; i < 10; ++i) {
        auto peer = h.client(roomIdFor(50), i % 2 ? "[2001:db8:1:2::5]" : "[2001:db8:1:2:aaaa::9]", 7000 + i);
        peer->takeControl();
    }
    CHECK(h.server->stats().lockouts == 1);
    for (int i = 0; i < 10; ++i) h.client(roomIdFor(50), "[::ffff:198.51.100.7]", 7100 + i)->takeControl();
    CHECK(h.server->stats().lockouts == 2);
    auto v4 = h.client(roomIdFor(50), "198.51.100.7", 7200);
    CHECK(replyOf<rl::JoinResult>(*v4)->reason == rl::JoinRoomFailure::LockedOut);
}

TEST_CASE("relay server: a flooding client is rate limited and then disconnected") {
    RelayServerConfig config;
    config.table.abuseDropsPerSecond = 50;
    Harness h(config);
    auto host = h.host(roomIdFor(1));
    auto client = h.client(roomIdFor(1));
    auto bystander = h.client(roomIdFor(1), "10.0.0.3");
    host->takeControl();
    for (int i = 0; i < 400; ++i) client->sendGame(Delivery::Reliable, payload(1000));
    h.pump();
    CHECK(client->closedByRelay);
    CHECK_FALSE(bystander->closedByRelay);
    CHECK(h.server->stats().peersKicked == 1);
    CHECK(h.server->stats().dropped(DropReason::RateLimited) >= 50);
    bool sawLeft = false;
    for (const Message& m : host->takeControl())
        if (const auto* left = std::get_if<rl::PeerLeft>(&m)) sawLeft = sawLeft || left->reason == rl::PeerLeftReason::RateLimited;
    CHECK(sawLeft);
}

TEST_CASE("relay server: oversize datagrams are dropped and counted") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    auto client = h.client(roomIdFor(1));
    host->takeControl();
    client->sendGame(Delivery::Unreliable, payload(rl::kRelayMaxDatagram));
    client->sendGame(Delivery::Unreliable, payload(rl::kRelayMaxDatagram - rl::kRelayEnvelopeBytes));
    h.pump();
    CHECK(host->takeGame().size() == 1);
    CHECK(h.server->stats().dropped(DropReason::Oversized) == 1);
}

TEST_CASE("relay server: the relay forwards opaque bytes without parsing them") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    auto client = h.client(roomIdFor(1));
    host->takeControl();
    client->takeControl();

    std::mt19937 rng(7);
    std::vector<Bytes> samples;
    for (int i = 0; i < 40; ++i) {
        Bytes random(1 + rng() % 600);
        for (auto& b : random) b = static_cast<std::byte>(rng());
        samples.push_back(std::move(random));
    }
    // Bytes that ARE valid protocol messages, relay control messages included, are just payload on the game channels.
    samples.push_back(encoded(registerMessage(roomIdFor(2))));
    samples.push_back(encoded(joinMessage(roomIdFor(3))));
    samples.push_back(encoded(rl::PeerLeft{1, rl::PeerLeftReason::Disconnected}));
    samples.push_back(encoded(rl::RoomRegistered{true, 6}));
    samples.push_back(Bytes(300, std::byte{0}));

    for (const Bytes& sample : samples) {
        client->sendGame(Delivery::Reliable, sample);
        host->sendGame(Delivery::Reliable, withEnvelope(1, sample));
    }
    h.pump();
    const auto atHost = host->takeGame();
    const auto atClient = client->takeGame();
    REQUIRE(atHost.size() == samples.size());
    REQUIRE(atClient.size() == samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
        CHECK(atHost[i].bytes == withEnvelope(1, samples[i]));
        CHECK(atClient[i].bytes == samples[i]);
    }
    CHECK(h.server->stats().rooms == 1);  // the embedded RegisterRoom created nothing
    CHECK(h.server->stats().controlViolations == 0);
    CHECK(host->takeControl().empty());
    CHECK(client->takeControl().empty());
}

TEST_CASE("relay server: logs name rooms and truncated addresses but never frame contents") {
    Harness h;
    auto host = h.host(roomIdFor(1), 1, "198.51.100.77", 4444);
    auto client = h.client(roomIdFor(1), "203.0.113.5", 5555);
    const std::string secret = "TOPSECRETMARKER";
    client->sendGame(Delivery::Reliable, bytesOf(secret));
    host->sendGame(Delivery::Reliable, withEnvelope(1, bytesOf(secret)));
    h.pump();
    h.advance(61s);
    CHECK(h.logContains("room_created"));
    CHECK(h.logContains("peer_joined"));
    CHECK(h.logContains("\"event\":\"stats\""));
    CHECK(h.logContains(roomIdFor(1)));
    CHECK(h.logContains("198.51.100.x"));
    CHECK_FALSE(h.logContains("198.51.100.77"));
    CHECK_FALSE(h.logContains("203.0.113.5"));
    CHECK_FALSE(h.logContains("4444"));
    CHECK_FALSE(h.logContains(secret));
    CHECK_FALSE(h.logContains("VE9QU0VDUkVU"));
    for (const std::string& line : *h.logLines) {
        CHECK(line.front() == '{');
        CHECK(line.back() == '}');
    }
}

TEST_CASE("relay server: a failing send to a peer closes that peer and tells the host") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    auto client = h.client(roomIdFor(1));
    host->takeControl();
    client->failSends = true;
    host->sendGame(Delivery::Reliable, withEnvelope(1, bytesOf("x")));
    h.pump();
    CHECK(client->closedByRelay);
    CHECK(replyOf<rl::PeerLeft>(*host).has_value());
    CHECK(h.server->stats().sendFailures >= 1);
}

TEST_CASE("relay server: a soak of 100 rooms with 6 clients each never leaks across rooms") {
    Harness h;
    constexpr unsigned kRooms = 100;
    constexpr unsigned kClients = rl::kRelayMaxPeersPerRoom;
    struct Room {
        std::shared_ptr<FakePeer> host;
        std::vector<std::shared_ptr<FakePeer>> clients;
    };
    std::vector<Room> rooms(kRooms);
    for (unsigned r = 0; r < kRooms; ++r) {
        rooms[r].host = h.host(roomIdFor(r), static_cast<std::uint8_t>(r), "10." + std::to_string(r / 200) + "." + std::to_string(r % 200) + ".1");
        for (unsigned c = 0; c < kClients; ++c)
            rooms[r].clients.push_back(h.client(roomIdFor(r), "10.1." + std::to_string(r) + "." + std::to_string(c + 1), 6000 + c));
    }
    auto stats = h.server->stats();
    CHECK(stats.rooms == kRooms);
    CHECK(stats.peers == kRooms * kClients);
    CHECK(stats.connections == kRooms * (kClients + 1));
    for (Room& room : rooms) {
        const auto control = room.host->takeControl();
        CHECK(control.size() == 1 + kClients);
        for (auto& c : room.clients) c->takeControl();
    }

    // Every payload starts with its room index; the content after it is a unique counter.
    std::mt19937 rng(2024);
    std::uint64_t counter = 0;
    std::vector<std::uint64_t> sentToHost(kRooms), sentToClients(kRooms);
    for (int round = 0; round < 20; ++round) {
        for (unsigned r = 0; r < kRooms; ++r) {
            for (int k = 0; k < 3; ++k) {
                const unsigned c = rng() % kClients;
                Bytes frame{static_cast<std::byte>(r)};
                for (int i = 0; i < 8; ++i) frame.push_back(static_cast<std::byte>(counter >> (8 * i)));
                ++counter;
                frame.resize(frame.size() + rng() % 100, std::byte{0x55});
                rooms[r].clients[c]->sendGame(rng() % 2 ? Delivery::Reliable : Delivery::Unreliable, frame);
                ++sentToHost[r];
            }
            const bool broadcast = rng() % 2;
            const rl::RelayPeerId target = broadcast ? rl::kRelayBroadcastPeer : static_cast<rl::RelayPeerId>(1 + rng() % kClients);
            Bytes frame{static_cast<std::byte>(r)};
            frame.resize(frame.size() + rng() % 100, std::byte{0x66});
            rooms[r].host->sendGame(Delivery::Reliable, withEnvelope(target, frame));
            sentToClients[r] += broadcast ? kClients : 1;
        }
        h.pump();
    }
    for (unsigned r = 0; r < kRooms; ++r) {
        const auto atHost = rooms[r].host->takeGame();
        CHECK(atHost.size() == sentToHost[r]);
        for (const GameFrame& g : atHost) {
            REQUIRE(g.bytes.size() >= 3);
            CHECK(static_cast<unsigned>(g.bytes[2]) == r);
        }
        std::uint64_t received = 0;
        for (auto& c : rooms[r].clients)
            for (const GameFrame& g : c->takeGame()) {
                ++received;
                REQUIRE_FALSE(g.bytes.empty());
                CHECK(static_cast<unsigned>(g.bytes[0]) == r);
            }
        CHECK(received == sentToClients[r]);
    }
    stats = h.server->stats();
    CHECK(stats.framesDroppedTotal() == 0);
    CHECK(stats.connections == kRooms * (kClients + 1));
    CHECK(stats.trackedSources == 0);

    // Everything goes away: no state is left behind.
    for (Room& room : rooms) {
        room.host->disconnect();
        for (auto& c : room.clients) c->disconnect();
    }
    h.pump();
    h.advance(31s);
    stats = h.server->stats();
    CHECK(stats.rooms == 0);
    CHECK(stats.connections == 0);
    CHECK(stats.peers == 0);
}

TEST_CASE("relay server: connection churn leaves no state behind") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    for (int i = 0; i < 2000; ++i) {
        auto client = h.client(roomIdFor(1), "10.0.5." + std::to_string(i % 200), 9000 + i % 1000);
        client->sendGame(Delivery::Reliable, bytesOf("x"));
        h.pump();
        client->disconnect();
        h.pump();
        host->takeControl();
        host->takeGame();
        h.advance(100ms);
    }
    const auto stats = h.server->stats();
    CHECK(stats.rooms == 1);
    CHECK(stats.peers == 0);
    CHECK(stats.connections == 1);
    CHECK(stats.trackedSources == 0);
}

TEST_CASE("relay server: stats lines are logged on the interval") {
    RelayServerConfig config;
    config.statsInterval = 10s;
    Harness h(config);
    h.host(roomIdFor(1));
    const std::size_t before = std::count_if(h.logLines->begin(), h.logLines->end(), [](const std::string& l) { return l.find("\"stats\"") != std::string::npos; });
    CHECK(before == 0);
    h.advance(11s);
    h.advance(10s);
    const std::size_t after = std::count_if(h.logLines->begin(), h.logLines->end(), [](const std::string& l) { return l.find("\"stats\"") != std::string::npos; });
    CHECK(after == 2);
}

TEST_CASE("relay server: shutdown disconnects everyone and run returns on stop") {
    Harness h;
    auto host = h.host(roomIdFor(1));
    auto client = h.client(roomIdFor(1));
    h.server->shutdown();
    CHECK(host->closedByRelay);
    CHECK(client->closedByRelay);
    CHECK(h.network->closed);
    CHECK(h.server->stats().connections == 0);

    // run() on its own thread returns promptly after stop().
    Harness idle;
    std::atomic<bool> returned{false};
    std::thread runner([&] {
        (void)idle.server->run();
        returned = true;
    });
    std::this_thread::sleep_for(50ms);
    CHECK_FALSE(returned.load());
    idle.server->stop();
    runner.join();
    CHECK(returned.load());
    CHECK(idle.network->closed);
}
