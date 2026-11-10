// The relay over REAL ENet sockets on the loopback interface: RelayServer on net::EnetServerTransport (ephemeral port) with
// net::EnetClientTransport peers. The fake-network tests cover the logic; these prove the pieces fit together on a wire.
#include <doctest/doctest.h>

#include <chrono>
#include <future>
#include <mutex>
#include <random>
#include <thread>

#include "relay_test_support.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/relay/relay_options.h"
#include "wpiemu/relay/relay_selfcheck.h"

using namespace std::chrono_literals;
using namespace relaytest;
using wpiemu::relay::DropReason;
using wpiemu::relay::RelayServerConfig;

namespace {

namespace net = wpiemu::net;
namespace wr = wpiemu::relay;

constexpr auto kWait = 4s;

struct LiveRelay {
    std::shared_ptr<std::mutex> mutex = std::make_shared<std::mutex>();
    std::shared_ptr<std::vector<std::string>> lines = std::make_shared<std::vector<std::string>>();
    std::unique_ptr<wr::RelayServer> server;
    std::thread thread;
    std::future<wpiemu::sdk::Status> result;
    std::string endpoint;
    std::uint16_t port = 0;

    explicit LiveRelay(RelayServerConfig config = {}, const std::string& bindHost = "127.0.0.1") {
        wr::RelayOptions production;
        production.server = config;
        production.bindHost = bindHost;
        production.port = 0;
        production.maxConnections = 64;
        const net::EnetServerOptions options = wr::transportOptionsFor(production);
        auto sink = [m = mutex, l = lines](const std::string& line) {
            std::lock_guard lock(*m);
            l->push_back(line);
        };
        auto log = std::make_shared<wr::RelayLog>(wr::LogFormat::Json, sink, nullptr, wr::LogLevel::Debug);
        server = std::make_unique<wr::RelayServer>(std::make_unique<net::EnetServerTransport>(options), std::move(config),
                                                      std::make_shared<wpiemu::sdk::SteadyWallClock>(), log);
        REQUIRE(server->start());
        endpoint = server->endpoint();
        port = net::NetEndpoint::parse(endpoint).value().port;
        std::promise<wpiemu::sdk::Status> promise;
        result = promise.get_future();
        thread = std::thread([this, p = std::move(promise)]() mutable { p.set_value(server->run()); });
    }

    // The relay is bound to the loopback interface; clients dial IPv4 loopback with the port the OS chose.
    std::string address() const { return "enet://127.0.0.1:" + std::to_string(port); }

    bool stopAndJoin() {
        server->stop();
        const bool finished = result.wait_for(kWait) == std::future_status::ready;
        if (thread.joinable()) thread.join();
        return finished;
    }
    ~LiveRelay() {
        if (thread.joinable()) stopAndJoin();
    }

    bool logContains(const std::string& text) {
        std::lock_guard lock(*mutex);
        for (const std::string& line : *lines)
            if (line.find(text) != std::string::npos) return true;
        return false;
    }
    template <class Pred>
    bool waitFor(Pred pred, std::chrono::milliseconds limit = kWait) {
        const auto deadline = std::chrono::steady_clock::now() + limit;
        while (!pred()) {
            if (std::chrono::steady_clock::now() >= deadline) return false;
            std::this_thread::sleep_for(5ms);
        }
        return true;
    }
};

struct Peer {
    std::unique_ptr<wpiemu::sdk::proto::ITransport> transport;
    wpiemu::net::IControlLink* control = nullptr;

    // A host link carries the 2-byte envelope on top of the game limits, like net::RelayHostTransport sets it up.
    explicit Peer(const std::string& endpoint, bool asHost = false) {
        net::EnetClientOptions link;
        if (asHost) {
            link.maxReliableFrameBytes = rl::kRelayMaxReliablePacket + rl::kRelayEnvelopeBytes;
            link.maxUnreliableFrameBytes = rl::kRelayMaxDatagram + rl::kRelayEnvelopeBytes;
        }
        net::EnetClientTransport client(link);
        auto connected = client.connect(endpoint, kWait);
        REQUIRE(connected);
        transport = std::move(connected).value();
        control = net::controlLinkOf(*transport);
        REQUIRE(control != nullptr);
    }

    void send(const Message& message) { REQUIRE(control->sendControl(encoded(message))); }

    std::optional<Message> next(std::chrono::milliseconds timeout = kWait) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        Bytes frame;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto got = control->receiveControl(frame, wpiemu::sdk::proto::timeLeft(deadline));
            if (!got) return std::nullopt;
            if (got.value()) {
                auto decoded = codec().decode(frame);
                if (decoded) return std::move(decoded).value();
                return std::nullopt;
            }
        }
        return std::nullopt;
    }
    template <class T>
    std::optional<T> expect(std::chrono::milliseconds timeout = kWait) {
        const auto message = next(timeout);
        if (!message) return std::nullopt;
        if (const auto* value = std::get_if<T>(&*message)) return *value;
        return std::nullopt;
    }

    std::optional<GameFrame> nextGame(std::chrono::milliseconds timeout = kWait) {
        Bytes frame;
        Delivery delivery = Delivery::Reliable;
        const auto got = control->receiveGame(frame, delivery, timeout);
        if (!got || !got.value()) return std::nullopt;
        return GameFrame{delivery, std::move(frame)};
    }

    bool disconnectsWithin(std::chrono::milliseconds timeout = kWait) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        Bytes frame;
        while (std::chrono::steady_clock::now() < deadline) {
            if (!control->receiveControl(frame, wpiemu::sdk::proto::timeLeft(deadline))) return true;
        }
        return false;
    }

    // Registers as the host of `room`; the reply is returned.
    std::optional<rl::RoomRegistered> registerHost(const std::string& room, std::uint8_t auth = 1) {
        send(registerMessage(room, auth));
        return expect<rl::RoomRegistered>();
    }
    std::optional<rl::JoinResult> join(const std::string& room) {
        send(joinMessage(room));
        return expect<rl::JoinResult>();
    }
};

Bytes withEnvelope(rl::RelayPeerId target, const Bytes& body) {
    Bytes out{static_cast<std::byte>(target & 0xFF), static_cast<std::byte>(target >> 8)};
    out.insert(out.end(), body.begin(), body.end());
    return out;
}

// Unreliable frames may in principle be lost: send until the other end has it (a loopback delivers the first one).
std::optional<GameFrame> sendUnreliableUntilReceived(Peer& from, const Bytes& frame, Peer& to) {
    for (int attempt = 0; attempt < 20; ++attempt) {
        REQUIRE(from.transport->sendUnreliable(frame));
        if (auto got = to.nextGame(150ms)) return got;
    }
    return std::nullopt;
}

}  // namespace

TEST_CASE("relay enet: a host and two clients exchange control and game frames through the relay") {
    LiveRelay relay;
    Peer host(relay.address(), true);
    const auto registered = host.registerHost(roomIdFor(1));
    REQUIRE(registered.has_value());
    CHECK_FALSE(registered->resumed);

    Peer one(relay.address());
    const auto joinedOne = one.join(roomIdFor(1));
    REQUIRE(joinedOne.has_value());
    CHECK(joinedOne->ok);
    CHECK(joinedOne->peerId == 1);
    REQUIRE(host.expect<rl::PeerJoined>().value().peerId == 1);

    Peer two(relay.address());
    const auto joinedTwo = two.join(roomIdFor(1));
    REQUIRE(joinedTwo.has_value());
    CHECK(joinedTwo->peerId == 2);
    REQUIRE(host.expect<rl::PeerJoined>().value().peerId == 2);

    SUBCASE("client to host keeps the delivery class and prepends the peer id") {
        const Bytes reliable = bytesOf("client one, reliable");
        REQUIRE(one.transport->send(reliable));
        auto got = host.nextGame();
        REQUIRE(got.has_value());
        CHECK(got->delivery == Delivery::Reliable);
        CHECK(got->bytes == withEnvelope(1, reliable));

        const Bytes unreliable = bytesOf("client two, unreliable");
        got = sendUnreliableUntilReceived(two, unreliable, host);
        REQUIRE(got.has_value());
        CHECK(got->delivery == Delivery::Unreliable);
        CHECK(got->bytes == withEnvelope(2, unreliable));
    }

    SUBCASE("host to one client strips the envelope and reaches nobody else") {
        const Bytes body = bytesOf("for client two only");
        REQUIRE(host.transport->send(withEnvelope(2, body)));
        auto got = two.nextGame();
        REQUIRE(got.has_value());
        CHECK(got->delivery == Delivery::Reliable);
        CHECK(got->bytes == body);
        CHECK_FALSE(one.nextGame(300ms).has_value());
    }

    SUBCASE("a broadcast reaches every client on both delivery classes") {
        const Bytes body = bytesOf("to everyone");
        REQUIRE(host.transport->send(withEnvelope(rl::kRelayBroadcastPeer, body)));
        for (Peer* client : {&one, &two}) {
            const auto got = client->nextGame();
            REQUIRE(got.has_value());
            CHECK(got->delivery == Delivery::Reliable);
            CHECK(got->bytes == body);
        }
        bool seenByOne = false;
        bool seenByTwo = false;
        for (int attempt = 0; attempt < 20 && !(seenByOne && seenByTwo); ++attempt) {
            REQUIRE(host.transport->sendUnreliable(withEnvelope(rl::kRelayBroadcastPeer, body)));
            if (const auto got = one.nextGame(100ms)) seenByOne = seenByOne || got->delivery == Delivery::Unreliable;
            if (const auto got = two.nextGame(100ms)) seenByTwo = seenByTwo || got->delivery == Delivery::Unreliable;
        }
        CHECK(seenByOne);
        CHECK(seenByTwo);
    }

    SUBCASE("large opaque frames cross the relay unchanged") {
        std::mt19937 rng(11);
        Bytes big(100'000);
        for (auto& b : big) b = static_cast<std::byte>(rng());
        REQUIRE(one.transport->send(big));
        const auto atHost = host.nextGame();
        REQUIRE(atHost.has_value());
        CHECK(atHost->bytes == withEnvelope(1, big));
        REQUIRE(host.transport->send(withEnvelope(2, big)));
        const auto atClient = two.nextGame();
        REQUIRE(atClient.has_value());
        CHECK(atClient->bytes == big);
    }

    SUBCASE("a client that leaves is announced to the host") {
        two.transport.reset();
        const auto left = host.expect<rl::PeerLeft>();
        REQUIRE(left.has_value());
        CHECK(left->peerId == 2);
        CHECK(relay.waitFor([&] { return relay.server->stats().peers == 1; }));
    }

    const auto stats = relay.server->stats();
    CHECK(stats.rooms == 1);
    CHECK(stats.unknownSources == 0);  // ENet reports the UDP source address of every peer
    CHECK(stats.controlViolations == 0);
    CHECK(relay.logContains("127.0.0.x"));
    CHECK(relay.logContains(roomIdFor(1)));
}

TEST_CASE("relay enet: refusals, abuse and size caps over real sockets") {
    LiveRelay relay;
    Peer host(relay.address(), true);
    REQUIRE(host.registerHost(roomIdFor(1), 1).has_value());

    SUBCASE("an unknown room and a taken room are refused") {
        Peer client(relay.address());
        const auto result = client.join(roomIdFor(9));
        REQUIRE(result.has_value());
        CHECK_FALSE(result->ok);
        CHECK(result->reason == rl::JoinRoomFailure::NoSuchRoom);

        Peer rival(relay.address());
        rival.send(registerMessage(roomIdFor(1), 2));
        const auto rejected = rival.expect<rl::RoomRejected>();
        REQUIRE(rejected.has_value());
        CHECK(rejected->reason == rl::RoomRejectReason::RoomTaken);
    }

    SUBCASE("a game frame before registering is dropped and the peer can still join") {
        Peer client(relay.address());
        REQUIRE(client.transport->send(bytesOf("too early")));
        REQUIRE(relay.waitFor([&] { return relay.server->stats().dropped(DropReason::Unregistered) == 1; }));
        const auto result = client.join(roomIdFor(1));
        REQUIRE(result.has_value());
        CHECK(result->ok);
        REQUIRE(host.expect<rl::PeerJoined>().has_value());
        CHECK_FALSE(host.nextGame(200ms).has_value());
    }

    SUBCASE("an undecodable control frame disconnects the peer") {
        Peer client(relay.address());
        REQUIRE(client.control->sendControl(Bytes(30, std::byte{0xFF})));
        CHECK(client.disconnectsWithin());
        CHECK(relay.waitFor([&] { return relay.server->stats().controlViolations == 1; }));
    }

    SUBCASE("a control message that only a relay may send disconnects the peer") {
        Peer client(relay.address());
        client.send(rl::PeerJoined{3});
        CHECK(client.disconnectsWithin());
    }

    SUBCASE("a datagram one byte over the cap is dropped and counted") {
        Peer client(relay.address(), true);  // a link that, unlike a real client's, can send above the cap
        REQUIRE(client.join(roomIdFor(1))->ok);
        REQUIRE(host.expect<rl::PeerJoined>().has_value());
        REQUIRE(client.transport->sendUnreliable(Bytes(rl::kRelayMaxDatagram + 1, std::byte{1})));
        REQUIRE(relay.waitFor([&] { return relay.server->stats().dropped(DropReason::Oversized) == 1; }));
        const auto got = sendUnreliableUntilReceived(client, bytesOf("small"), host);
        REQUIRE(got.has_value());
        CHECK(got->bytes == withEnvelope(1, bytesOf("small")));
    }

    SUBCASE("a full-size datagram from a client reaches the host, a normal one still passes") {
        Peer client(relay.address());
        REQUIRE(client.join(roomIdFor(1))->ok);
        REQUIRE(host.expect<rl::PeerJoined>().has_value());
        const auto big = sendUnreliableUntilReceived(client, Bytes(rl::kRelayMaxDatagram, std::byte{1}), host);
        REQUIRE(big.has_value());
        CHECK(big->bytes.size() == rl::kRelayMaxDatagram + rl::kRelayEnvelopeBytes);
        CHECK(relay.server->stats().dropped(DropReason::Oversized) == 0);
        const auto got = sendUnreliableUntilReceived(client, bytesOf("small"), host);
        REQUIRE(got.has_value());
        CHECK(got->bytes == withEnvelope(1, bytesOf("small")));
    }
}

TEST_CASE("relay enet: a host that reconnects within the grace period resumes its room") {
    RelayServerConfig config;
    config.table.roomTtlWithoutHostMs = 600;
    LiveRelay relay(config);

    auto host = std::make_unique<Peer>(relay.address());
    REQUIRE(host->registerHost(roomIdFor(1), 5).has_value());
    Peer client(relay.address());
    REQUIRE(client.join(roomIdFor(1))->peerId == 1);
    REQUIRE(host->expect<rl::PeerJoined>().has_value());

    host.reset();
    REQUIRE(relay.waitFor([&] { return relay.server->stats().roomsWithoutHost == 1; }));
    REQUIRE(client.transport->send(bytesOf("nobody is listening")));
    REQUIRE(relay.waitFor([&] { return relay.server->stats().dropped(DropReason::HostAbsent) == 1; }));

    Peer wrong(relay.address());
    wrong.send(registerMessage(roomIdFor(1), 6));
    CHECK(wrong.expect<rl::RoomRejected>().value().reason == rl::RoomRejectReason::BadAuth);

    Peer again(relay.address());
    const auto resumed = again.registerHost(roomIdFor(1), 5);
    REQUIRE(resumed.has_value());
    CHECK(resumed->resumed);
    CHECK(again.expect<rl::PeerJoined>().value().peerId == 1);  // the client kept its id
    REQUIRE(client.transport->send(bytesOf("welcome back")));
    const auto got = again.nextGame();
    REQUIRE(got.has_value());
    CHECK(got->bytes == withEnvelope(1, bytesOf("welcome back")));

    SUBCASE("a host that never returns closes the room and its clients are disconnected") {
        again.transport.reset();
        CHECK(client.disconnectsWithin());
        CHECK(relay.waitFor([&] { return relay.server->stats().rooms == 0; }));
        CHECK(relay.server->stats().roomsExpired == 1);
    }
}

TEST_CASE("relay enet: the lockout is keyed on the real remote address") {
    SUBCASE("loopback is exempt by default") {
        RelayServerConfig config;
        config.table.lockoutMaxFailures = 3;
        LiveRelay relay(config);
        for (int i = 0; i < 6; ++i) {
            Peer client(relay.address());
            CHECK(client.join(roomIdFor(100 + i)).value().reason == rl::JoinRoomFailure::NoSuchRoom);
        }
        CHECK(relay.server->stats().lockouts == 0);
        CHECK(relay.server->stats().unknownSources == 0);
    }
    SUBCASE("when not exempt, repeated failures from the same address are locked out across connections") {
        RelayServerConfig config;
        config.table.lockoutMaxFailures = 3;
        config.exemptLoopbackFromLockout = false;
        LiveRelay relay(config);
        for (int i = 0; i < 3; ++i) {
            Peer client(relay.address());
            CHECK(client.join(roomIdFor(100 + i)).value().reason == rl::JoinRoomFailure::NoSuchRoom);
        }
        Peer locked(relay.address());
        CHECK(locked.join(roomIdFor(200)).value().reason == rl::JoinRoomFailure::LockedOut);
        CHECK(relay.server->stats().lockouts == 1);
        CHECK(relay.server->stats().unknownSources == 0);
    }
}

TEST_CASE("relay enet: selfcheck passes against a live relay and changes nothing") {
    LiveRelay relay;
    const wr::SelfCheckResult result = wr::runSelfCheck("127.0.0.1", relay.port, 2000ms);
    CHECK_MESSAGE(result.ok, result.detail);
    REQUIRE(relay.waitFor([&] { return relay.server->stats().pingsAnswered == 1; }));
    const auto stats = relay.server->stats();
    CHECK(stats.joinsRejected == 0);
    CHECK(stats.rooms == 0);
    CHECK(stats.peers == 0);
    CHECK(stats.roomsCreated == 0);
    CHECK(stats.peersJoined == 0);
    CHECK(relay.waitFor([&] { return relay.server->stats().connections == 0; }));
}

TEST_CASE("relay enet: selfcheck reaches a dual stack relay through IPv4 loopback") {
    LiveRelay relay({}, "::");
    const wr::SelfCheckResult result = wr::runSelfCheck("127.0.0.1", relay.port, 2000ms);
    CHECK_MESSAGE(result.ok, result.detail);
    CHECK(relay.server->stats().unknownSources == 0);
    CHECK(relay.server->stats().lockouts == 0);
}

TEST_CASE("relay enet: selfcheck fails when nothing answers") {
    std::uint16_t deadPort = 0;
    {
        net::EnetServerTransport transient;
        REQUIRE(transient.listen());
        deadPort = net::NetEndpoint::parse(transient.endpoint()).value().port;
    }
    const auto started = std::chrono::steady_clock::now();
    const wr::SelfCheckResult result = wr::runSelfCheck("127.0.0.1", deadPort, 600ms);
    CHECK_FALSE(result.ok);
    CHECK_FALSE(result.detail.empty());
    CHECK(std::chrono::steady_clock::now() - started < 3s);
}

TEST_CASE("relay enet: stop shuts down gracefully and disconnects every peer") {
    LiveRelay relay;
    Peer host(relay.address(), true);
    REQUIRE(host.registerHost(roomIdFor(1)).has_value());
    Peer client(relay.address());
    REQUIRE(client.join(roomIdFor(1))->ok);
    REQUIRE(host.expect<rl::PeerJoined>().has_value());

    REQUIRE(relay.stopAndJoin());
    CHECK(relay.result.get());
    CHECK(host.disconnectsWithin());
    CHECK(client.disconnectsWithin());
    CHECK(relay.logContains("\"event\":\"stopping\""));
}
