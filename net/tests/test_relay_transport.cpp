#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "enet_test_support.h"
#include "fake_relay.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/net/secure_transport.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace wpiemu::net;
using namespace wpiemu::net::test;
using wpiemu::sdk::testing::waitUntil;
namespace sdk = wpiemu::sdk;
namespace ipc = wpiemu::ipc;
namespace relay = wpiemu::sdk::match::relay;
namespace proto = wpiemu::sdk::proto;

namespace {

InviteSecret secretN(std::uint8_t n) {
    InviteSecret::Bytes bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i) bytes[i] = static_cast<std::uint8_t>(n * 31 + i * 7 + 1);
    bytes.back() &= 0xF0;
    return InviteSecret::fromBytes(bytes).value();
}

RelayHostOptions fastOptions(const FakeRelay& fake, const InviteSecret& secret) {
    RelayHostOptions options = relayHostOptionsFor(fake.host(), fake.port(), secret);
    options.connectTimeout = 2000ms;
    options.registerTimeout = 2000ms;
    options.reconnectBackoffInitial = 20ms;
    options.reconnectBackoffMax = 100ms;
    options.reconnectGiveUp = 10s;
    return options;
}

using Tweak = std::function<void(RelayHostOptions&)>;

// A fake relay with one registered host.
struct Rig {
    explicit Rig(const Tweak& tweak = {}, FakeRelayOptions relayOptions = {}, std::uint8_t secretSeed = 1)
        : relay(relayOptions), secret(secretN(secretSeed)), room(roomId(secret)) {
        REQUIRE_MESSAGE(relay.listenError().empty(), relay.listenError());
        RelayHostOptions options = fastOptions(relay, secret);
        if (tweak) tweak(options);
        host = std::make_unique<RelayHostTransport>(options);
        const auto status = host->listen();
        REQUIRE_MESSAGE(status.ok(), (status.ok() ? "" : status.error().message));
    }

    std::unique_ptr<ITransport> join() {
        RelayClientTransport client;
        auto connected = client.connect(relay.endpoint(room), 3000ms);
        REQUIRE_MESSAGE(connected.ok(), (connected.ok() ? "" : connected.error().message));
        return std::move(connected).value();
    }

    std::unique_ptr<ITransport> accept(std::chrono::milliseconds timeout = kLong) {
        auto accepted = host->accept(timeout);
        REQUIRE_MESSAGE(accepted.ok(), (accepted.ok() ? "" : accepted.error().message));
        return std::move(accepted).value();
    }

    FakeRelay relay;
    InviteSecret secret;
    std::string room;
    std::unique_ptr<RelayHostTransport> host;
};

struct Joined {
    std::unique_ptr<ITransport> client;
    std::unique_ptr<ITransport> peer;
};

Joined joinAndAccept(Rig& rig) {
    Joined joined;
    joined.client = rig.join();
    joined.peer = rig.accept();
    REQUIRE(joined.peer != nullptr);
    return joined;
}

Bytes sequenced(std::uint32_t seq, std::uint8_t tag, std::size_t size = 8) {
    Bytes out(std::max<std::size_t>(size, 5));
    for (int i = 0; i < 4; ++i) out[static_cast<std::size_t>(i)] = static_cast<std::byte>((seq >> (8 * i)) & 0xFF);
    out[4] = static_cast<std::byte>(tag);
    return out;
}

std::uint32_t sequenceOf(const Bytes& frame) {
    std::uint32_t seq = 0;
    for (int i = 0; i < 4; ++i) seq |= static_cast<std::uint32_t>(frame[static_cast<std::size_t>(i)]) << (8 * i);
    return seq;
}

bool noFrame(ITransport& transport, std::chrono::milliseconds wait = 150ms) { return !receiveFrame(transport, wait).has_value(); }

// Unreliable frames may be lost (a loaded machine loses some even on loopback): resend until one arrives.
std::optional<Bytes> unreliableRoundTrip(ITransport& from, ITransport& to, const Bytes& frame) {
    for (int attempt = 0; attempt < 40; ++attempt) {
        REQUIRE(from.sendUnreliable(frame).ok());
        if (auto got = receiveFrame(to, 250ms)) return got;
    }
    return std::nullopt;
}

}  // namespace

// ---- registering -------------------------------------------------------------------------------------------------------

TEST_CASE("net relay: a host registers its room and reports the relay endpoint") {
    Rig rig;
    CHECK(rig.host->endpoint() == rig.relay.endpoint(rig.room));
    CHECK(rig.host->registered());
    CHECK(rig.relay.hasRoom(rig.room));
    CHECK(rig.relay.hostPresent(rig.room));
    CHECK(rig.relay.traffic().registrations == 1);
    CHECK(rig.host->listen().error().code == ErrorCode::InvalidState);
}

TEST_CASE("net relay: accept before listen is an error and accept times out with nullptr") {
    FakeRelay fake;
    RelayHostTransport unlistened(fastOptions(fake, secretN(2)));
    CHECK(unlistened.accept(10ms).error().code == ErrorCode::InvalidState);

    Rig rig;
    auto none = rig.host->accept(80ms);
    REQUIRE(none.ok());
    CHECK(none.value() == nullptr);
}

TEST_CASE("net relay: registering a taken room id is refused with AlreadyExists") {
    Rig rig;
    RelayHostOptions options = fastOptions(rig.relay, secretN(9));
    options.roomId = rig.room;  // the same room, another host
    RelayHostTransport other(options);
    const auto status = other.listen();
    REQUIRE_FALSE(status.ok());
    CHECK(status.error().code == ErrorCode::AlreadyExists);
    CHECK(rig.host->registered());
}

TEST_CASE("net relay: re-registering an absent room with another hostAuth is BadAuth") {
    Rig rig([](RelayHostOptions& o) { o.autoReregister = false; });
    rig.relay.dropHost(rig.room);
    REQUIRE(waitUntil([&] { return !rig.relay.hostPresent(rig.room); }, kLong, 2ms));
    RelayHostOptions options = fastOptions(rig.relay, secretN(9));
    options.roomId = rig.room;
    RelayHostTransport other(options);
    const auto status = other.listen();
    REQUIRE_FALSE(status.ok());
    CHECK(status.error().code == ErrorCode::InvalidArgument);
    CHECK(status.error().message.find("authentication") != std::string::npos);
}

TEST_CASE("net relay: a full relay refuses the registration with OutOfRange") {
    FakeRelayOptions relayOptions;
    relayOptions.maxRooms = 1;
    Rig rig({}, relayOptions);
    RelayHostTransport second(fastOptions(rig.relay, secretN(9)));
    const auto status = second.listen();
    REQUIRE_FALSE(status.ok());
    CHECK(status.error().code == ErrorCode::OutOfRange);
}

TEST_CASE("net relay: an unsupported relay protocol version is VersionMismatch for host and client") {
    FakeRelay fake;
    RelayHostOptions options = fastOptions(fake, secretN(1));
    options.protocolVersion = 99;
    RelayHostTransport host(options);
    CHECK(host.listen().error().code == ErrorCode::VersionMismatch);

    Rig rig;
    RelayClientOptions clientOptions;
    clientOptions.protocolVersion = 99;
    RelayClientTransport client(clientOptions);
    auto joined = client.connect(rig.relay.endpoint(rig.room), 3000ms);
    REQUIRE_FALSE(joined.ok());
    CHECK(joined.error().code == ErrorCode::VersionMismatch);
}

TEST_CASE("net relay: listen validates the room id and the relay address") {
    FakeRelay fake;
    RelayHostOptions options = fastOptions(fake, secretN(1));
    options.roomId = "short";
    CHECK(RelayHostTransport(options).listen().error().code == ErrorCode::InvalidArgument);
    options = fastOptions(fake, secretN(1));
    options.relayPort = 0;
    CHECK(RelayHostTransport(options).listen().error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("net relay: listen times out when no relay answers and may be retried") {
    std::uint16_t deadPort = 0;
    {
        FakeRelay gone;
        deadPort = gone.port();
    }
    RelayHostOptions options = relayHostOptionsFor("127.0.0.1", deadPort, secretN(1));
    options.connectTimeout = 300ms;
    RelayHostTransport host(options);
    const auto started = std::chrono::steady_clock::now();
    const auto status = host.listen();
    REQUIRE_FALSE(status.ok());
    CHECK(status.error().code == ErrorCode::Timeout);
    CHECK(std::chrono::steady_clock::now() - started < 3s);
    CHECK(host.listen().error().code == ErrorCode::Timeout);  // not stuck in a half-started state
}

// ---- joining -----------------------------------------------------------------------------------------------------------

TEST_CASE("net relay: a client joins and the host accepts a virtual peer") {
    Rig rig;
    auto client = rig.join();
    REQUIRE(client != nullptr);
    CHECK(client->connected());
    CHECK(client->describe() == rig.relay.endpoint(rig.room) + "(peer 1)");
    auto peer = rig.accept();
    REQUIRE(peer != nullptr);
    CHECK(peer->connected());
    CHECK(peer->describe() == rig.relay.endpoint(rig.room) + "(peer 1)");
    CHECK(rig.host->peerCount() == 1);
    CHECK(rig.relay.clientIds(rig.room) == std::vector<relay::RelayPeerId>{1});
    CHECK(peer->linkStats().has_value());
}

TEST_CASE("net relay: join errors name the reason") {
    FakeRelayOptions relayOptions;
    relayOptions.maxPeersPerRoom = 1;
    Rig rig({}, relayOptions);

    RelayClientTransport client;
    auto missing = client.connect(rig.relay.endpoint("0123456Z"), 3000ms);
    REQUIRE_FALSE(missing.ok());
    CHECK(missing.error().code == ErrorCode::NotFound);

    auto first = rig.join();
    auto full = client.connect(rig.relay.endpoint(rig.room), 3000ms);
    REQUIRE_FALSE(full.ok());
    CHECK(full.error().code == ErrorCode::OutOfRange);
    CHECK(full.error().message.find("full") != std::string::npos);

    auto wrongScheme = client.connect("enet://127.0.0.1:1234", 100ms);
    CHECK(wrongScheme.error().code == ErrorCode::InvalidArgument);
    CHECK(client.connect("not an endpoint", 100ms).error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("net relay: joining a room whose host is away is refused") {
    Rig rig([](RelayHostOptions& o) { o.autoReregister = false; });
    rig.relay.dropHost(rig.room);
    REQUIRE(waitUntil([&] { return !rig.relay.hostPresent(rig.room); }, kLong, 2ms));
    RelayClientTransport client;
    auto joined = client.connect(rig.relay.endpoint(rig.room), 3000ms);
    REQUIRE_FALSE(joined.ok());
    CHECK(joined.error().code == ErrorCode::Disconnected);
    CHECK(joined.error().message.find("host") != std::string::npos);
}

TEST_CASE("net relay: a client times out when nothing listens or the relay stays silent") {
    {
        std::uint16_t deadPort = 0;
        {
            FakeRelay gone;
            deadPort = gone.port();
        }
        RelayClientTransport client;
        const auto started = std::chrono::steady_clock::now();
        auto joined = client.connect("relay://127.0.0.1:" + std::to_string(deadPort) + "/0123456Z", 300ms);
        REQUIRE_FALSE(joined.ok());
        CHECK(joined.error().code == ErrorCode::Timeout);
        CHECK(std::chrono::steady_clock::now() - started < 3s);
    }
    {
        auto silent = listening(loopbackServer());  // accepts the ENet connection, never answers JoinRoom
        const std::string endpoint = silent->endpoint();
        const std::string port = endpoint.substr(endpoint.rfind(':') + 1);
        RelayClientTransport client;
        auto joined = client.connect("relay://127.0.0.1:" + port + "/0123456Z", 400ms);
        REQUIRE_FALSE(joined.ok());
        CHECK(joined.error().code == ErrorCode::Timeout);
        CHECK(joined.error().message.find("did not answer") != std::string::npos);
    }
}

// ---- frames ------------------------------------------------------------------------------------------------------------

TEST_CASE("net relay: two clients reach their own virtual peers and get their own replies") {
    Rig rig;
    auto c1 = rig.join();
    auto p1 = rig.accept();
    auto c2 = rig.join();
    auto p2 = rig.accept();
    REQUIRE((p1 != nullptr && p2 != nullptr));
    CHECK(p1->describe().find("(peer 1)") != std::string::npos);
    CHECK(p2->describe().find("(peer 2)") != std::string::npos);

    REQUIRE(c1->send(bytesOf("from-1")).ok());
    REQUIRE(c2->send(bytesOf("from-2")).ok());
    CHECK(textOf(*receiveFrame(*p1)) == "from-1");
    CHECK(textOf(*receiveFrame(*p2)) == "from-2");
    CHECK(noFrame(*p1));
    CHECK(noFrame(*p2));

    REQUIRE(p1->send(bytesOf("to-1")).ok());
    REQUIRE(p2->send(bytesOf("to-2")).ok());
    CHECK(textOf(*receiveFrame(*c1)) == "to-1");
    CHECK(textOf(*receiveFrame(*c2)) == "to-2");
    CHECK(noFrame(*c1));
    CHECK(noFrame(*c2));
}

TEST_CASE("net relay: the delivery class survives both ways") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);

    REQUIRE(client->send(bytesOf("r-up")).ok());
    REQUIRE(peer->send(bytesOf("r-down")).ok());
    CHECK(textOf(*receiveFrame(*peer)) == "r-up");
    CHECK(textOf(*receiveFrame(*client)) == "r-down");
    auto up = unreliableRoundTrip(*client, *peer, bytesOf("u-up"));
    auto down = unreliableRoundTrip(*peer, *client, bytesOf("u-down"));
    REQUIRE((up.has_value() && down.has_value()));
    CHECK(textOf(*up) == "u-up");
    CHECK(textOf(*down) == "u-down");

    const FakeRelayTraffic traffic = rig.relay.traffic();
    CHECK(traffic.toHostReliable == 1);
    CHECK(traffic.toHostUnreliable >= 1);
    CHECK(traffic.toClientReliable == 1);
    CHECK(traffic.toClientUnreliable >= 1);
    CHECK(traffic.malformedControl == 0);
    CHECK(traffic.unexpectedControl == 0);
}

TEST_CASE("net relay: large reliable frames and the size limits") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);

    const Bytes up = patterned(relay::kRelayMaxReliablePacket, 11);  // a client may use the whole limit
    REQUIRE(client->send(up).ok());
    auto gotUp = receiveFrame(*peer, 10s);
    REQUIRE(gotUp.has_value());
    CHECK(*gotUp == up);

    const Bytes down = patterned(relay::kRelayMaxReliablePacket - relay::kRelayEnvelopeBytes, 12);
    REQUIRE(peer->send(down).ok());
    auto gotDown = receiveFrame(*client, 10s);
    REQUIRE(gotDown.has_value());
    CHECK(*gotDown == down);

    CHECK(peer->send(patterned(down.size() + 1, 1)).error().code == ErrorCode::InvalidArgument);
    CHECK(peer->sendUnreliable(patterned(relay::kRelayMaxDatagram - relay::kRelayEnvelopeBytes + 1, 1)).error().code ==
          ErrorCode::InvalidArgument);
    CHECK(peer->send(Bytes{}).error().code == ErrorCode::InvalidArgument);
    auto toClient = unreliableRoundTrip(*peer, *client, patterned(relay::kRelayMaxDatagram - relay::kRelayEnvelopeBytes, 2));
    REQUIRE(toClient.has_value());
    CHECK(toClient->size() == relay::kRelayMaxDatagram - relay::kRelayEnvelopeBytes);
    auto datagram = unreliableRoundTrip(*client, *peer, patterned(relay::kRelayMaxDatagram, 3));
    REQUIRE(datagram.has_value());
    CHECK(datagram->size() == relay::kRelayMaxDatagram);
}

TEST_CASE("net relay: many small reliable frames arrive in order for two clients at once") {
    Rig rig;
    auto c1 = rig.join();
    auto p1 = rig.accept();
    auto c2 = rig.join();
    auto p2 = rig.accept();
    REQUIRE((p1 != nullptr && p2 != nullptr));
    constexpr std::uint32_t kCount = 1500;

    const auto sender = [&](ITransport& transport, std::uint8_t tag) {
        for (std::uint32_t i = 0; i < kCount; ++i) REQUIRE(transport.send(sequenced(i, tag)).ok());
    };
    std::atomic<int> bad{0};
    const auto checker = [&](ITransport& transport, std::uint8_t tag) {
        for (std::uint32_t i = 0; i < kCount; ++i) {
            auto got = receiveFrame(transport, 10s);
            if (!got || sequenceOf(*got) != i || (*got)[4] != static_cast<std::byte>(tag)) {
                ++bad;
                return;
            }
        }
    };
    std::vector<std::thread> threads;
    threads.emplace_back(sender, std::ref(*c1), std::uint8_t{1});
    threads.emplace_back(sender, std::ref(*c2), std::uint8_t{2});
    threads.emplace_back(sender, std::ref(*p1), std::uint8_t{3});
    threads.emplace_back(sender, std::ref(*p2), std::uint8_t{4});
    threads.emplace_back(checker, std::ref(*p1), std::uint8_t{1});
    threads.emplace_back(checker, std::ref(*p2), std::uint8_t{2});
    threads.emplace_back(checker, std::ref(*c1), std::uint8_t{3});
    threads.emplace_back(checker, std::ref(*c2), std::uint8_t{4});
    for (auto& t : threads) t.join();
    CHECK(bad.load() == 0);
}

TEST_CASE("net relay: link stats count the frames of one virtual peer") {
    Rig rig;
    auto c1 = rig.join();
    auto p1 = rig.accept();
    auto c2 = rig.join();
    auto p2 = rig.accept();
    REQUIRE(c1->send(patterned(100, 1)).ok());
    REQUIRE(receiveFrame(*p1).has_value());
    REQUIRE(p1->send(patterned(40, 2)).ok());
    REQUIRE(receiveFrame(*c1).has_value());
    const auto stats = p1->linkStats();
    REQUIRE(stats.has_value());
    CHECK(stats->bytesIn == 100);
    CHECK(stats->bytesOut == 40);
    CHECK(p2->linkStats()->bytesIn == 0);
}

// ---- leaving -----------------------------------------------------------------------------------------------------------

TEST_CASE("net relay: PeerLeft closes only that virtual peer and delivers what it had buffered") {
    Rig rig;
    auto c1 = rig.join();
    auto p1 = rig.accept();
    auto c2 = rig.join();
    auto p2 = rig.accept();
    REQUIRE((p1 != nullptr && p2 != nullptr));

    REQUIRE(c1->send(bytesOf("a")).ok());
    REQUIRE(c1->send(bytesOf("b")).ok());
    REQUIRE(waitUntil([&] { return p1->linkStats()->bytesIn == 2; }, kLong, 2ms));
    rig.relay.dropClient(rig.room, 1);
    CHECK(textOf(*receiveFrame(*p1)) == "a");
    CHECK(textOf(*receiveFrame(*p1)) == "b");
    CHECK(receiveError(*p1).code == ErrorCode::Disconnected);
    CHECK_FALSE(p1->connected());
    CHECK(p1->send(bytesOf("x")).error().code == ErrorCode::Disconnected);

    CHECK(p2->connected());
    REQUIRE(c2->send(bytesOf("still here")).ok());
    CHECK(textOf(*receiveFrame(*p2)) == "still here");
    REQUIRE(p2->send(bytesOf("back")).ok());
    CHECK(textOf(*receiveFrame(*c2)) == "back");
    CHECK(rig.host->peerCount() == 1);
}

TEST_CASE("net relay: a client the relay ends reports Disconnected") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    rig.relay.endClient(rig.room, 1);
    CHECK(receiveError(*client).code == ErrorCode::Disconnected);
    CHECK(waitUntil([&] { return !client->connected(); }, kLong, 2ms));
    CHECK(client->send(bytesOf("x")).error().code == ErrorCode::Disconnected);
    CHECK(receiveError(*peer).code == ErrorCode::Disconnected);  // the client closed its link, so the relay tells the host
}

TEST_CASE("net relay: an expired room or a lost relay disconnects the client") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    rig.relay.expireRoom(rig.room);
    CHECK(receiveError(*client).code == ErrorCode::Disconnected);
    CHECK(waitUntil([&] { return !client->connected(); }, kLong, 2ms));
}

TEST_CASE("net relay: closing a registered host sends CloseRoom and the room is gone") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    CHECK(rig.relay.hasRoom(rig.room));
    rig.host->close();
    CHECK(waitUntil([&] { return rig.relay.traffic().roomsClosed == 1; }, kLong, 2ms));
    CHECK(waitUntil([&] { return !rig.relay.hasRoom(rig.room); }, kLong, 2ms));
    CHECK(receiveError(*client).code == ErrorCode::Disconnected);
    CHECK(rig.relay.traffic().unexpectedControl == 0);
    rig.host->close();  // idempotent: no second CloseRoom
    CHECK(rig.relay.traffic().roomsClosed == 1);
}

TEST_CASE("net relay: destroying the host transport also frees the room") {
    FakeRelay fake;
    const InviteSecret secret = secretN(3);
    {
        RelayHostTransport host(fastOptions(fake, secret));
        REQUIRE(host.listen().ok());
        CHECK(fake.hasRoom(roomId(secret)));
    }
    CHECK(waitUntil([&] { return !fake.hasRoom(roomId(secret)); }, kLong, 2ms));
    CHECK(fake.traffic().roomsClosed == 1);
}

TEST_CASE("net relay: closeRoomOnClose false leaves the room for its grace period") {
    Rig rig([](RelayHostOptions& options) { options.closeRoomOnClose = false; });
    auto [client, peer] = joinAndAccept(rig);
    rig.host->close();
    CHECK(waitUntil([&] { return !rig.relay.hostPresent(rig.room); }, kLong, 2ms));
    CHECK(rig.relay.hasRoom(rig.room));
    CHECK(rig.relay.traffic().roomsClosed == 0);
}

TEST_CASE("net relay: a host that never registered or lost its link sends no CloseRoom") {
    FakeRelay fake;
    {
        RelayHostTransport unlistened(fastOptions(fake, secretN(4)));
    }
    RelayHostOptions taken = fastOptions(fake, secretN(5));
    RelayHostTransport first(taken);
    REQUIRE(first.listen().ok());
    {
        RelayHostOptions imposter = taken;
        imposter.hostAuth[0] ^= 0x33;
        imposter.autoReregister = false;
        RelayHostTransport second(imposter);
        CHECK(second.listen().error().code == ErrorCode::AlreadyExists);
    }
    CHECK(fake.hasRoom(roomId(secretN(5))));  // the refused imposter did not close the real room
    CHECK(fake.traffic().roomsClosed == 0);
}

TEST_CASE("net relay: a virtual peer close is local and a closed client stops its peer's frames") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    peer->close();
    CHECK_FALSE(peer->connected());
    CHECK(peer->send(bytesOf("x")).error().code == ErrorCode::Disconnected);
    CHECK(receiveError(*peer).code == ErrorCode::Disconnected);
    CHECK(rig.host->peerCount() == 0);
    REQUIRE(client->send(bytesOf("ignored")).ok());  // dropped by the host, the client is not told
    CHECK(client->connected());

    auto [client2, peer2] = joinAndAccept(rig);
    client2->close();
    CHECK(receiveError(*peer2).code == ErrorCode::Disconnected);
}

TEST_CASE("net relay: close wakes a blocked accept and a blocked receive") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    std::atomic<bool> acceptDone{false};
    std::atomic<bool> receiveDone{false};
    proto::ITransport& peerRef = *peer;
    ErrorCode acceptCode = ErrorCode::Internal;
    ErrorCode receiveCode = ErrorCode::Internal;
    std::thread acceptor([&] {
        auto got = rig.host->accept(30s);
        if (!got.ok()) acceptCode = got.error().code;
        acceptDone = true;
    });
    std::thread receiver([&] {
        Bytes frame;
        auto got = peerRef.receive(frame, 30s);
        if (!got.ok()) receiveCode = got.error().code;
        receiveDone = true;
    });
    std::this_thread::sleep_for(100ms);
    const auto started = std::chrono::steady_clock::now();
    rig.host->close();
    acceptor.join();
    receiver.join();
    CHECK(std::chrono::steady_clock::now() - started < 3s);
    CHECK(acceptDone.load());
    CHECK(receiveDone.load());
    CHECK(acceptCode == ErrorCode::Disconnected);
    CHECK(receiveCode == ErrorCode::Disconnected);
    CHECK_FALSE(rig.host->registered());
    rig.host->close();  // idempotent
}

TEST_CASE("net relay: a virtual peer outlives its host transport and reports Disconnected") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    rig.host.reset();
    CHECK(receiveError(*peer).code == ErrorCode::Disconnected);
    CHECK(peer->send(bytesOf("x")).error().code == ErrorCode::Disconnected);
    CHECK(waitUntil([&] { return !rig.relay.hostPresent(rig.room); }, kLong, 2ms));  // the relay saw the host leave
}

// ---- the relay link is lost --------------------------------------------------------------------------------------------

TEST_CASE("net relay: a lost relay link disconnects every peer and the host re-registers and resumes") {
    Rig rig;
    auto c1 = rig.join();
    auto p1 = rig.accept();
    auto c2 = rig.join();
    auto p2 = rig.accept();
    REQUIRE((p1 != nullptr && p2 != nullptr));

    rig.relay.dropHost(rig.room);
    CHECK(receiveError(*p1).code == ErrorCode::Disconnected);
    CHECK(receiveError(*p2).code == ErrorCode::Disconnected);
    REQUIRE(waitUntil([&] { return rig.host->reregisterCount() == 1; }, kLong, 2ms));
    CHECK(rig.relay.traffic().registrations == 2);

    // The relay reports the clients that are still there: accepted as NEW virtual peers with the same ids.
    auto n1 = rig.accept();
    auto n2 = rig.accept();
    REQUIRE((n1 != nullptr && n2 != nullptr));
    CHECK(n1->describe().find("(peer 1)") != std::string::npos);
    CHECK(n2->describe().find("(peer 2)") != std::string::npos);
    REQUIRE(c1->send(bytesOf("after-1")).ok());
    REQUIRE(c2->send(bytesOf("after-2")).ok());
    CHECK(textOf(*receiveFrame(*n1)) == "after-1");
    CHECK(textOf(*receiveFrame(*n2)) == "after-2");
    REQUIRE(n2->send(bytesOf("down-2")).ok());
    CHECK(textOf(*receiveFrame(*c2)) == "down-2");
    CHECK(rig.host->registered());
    CHECK(p1->send(bytesOf("old")).error().code == ErrorCode::Disconnected);  // the old object stays dead
}

TEST_CASE("net relay: a client that left while the host was away is not reported after the resume") {
    Rig rig;
    auto c1 = rig.join();
    auto p1 = rig.accept();
    auto c2 = rig.join();
    auto p2 = rig.accept();
    REQUIRE((p1 != nullptr && p2 != nullptr));
    rig.relay.dropHostAndClient(rig.room, 2);
    REQUIRE(waitUntil([&] { return rig.host->reregisterCount() == 1; }, kLong, 2ms));
    auto n1 = rig.accept();
    REQUIRE(n1 != nullptr);
    CHECK(n1->describe().find("(peer 1)") != std::string::npos);
    auto none = rig.host->accept(300ms);
    REQUIRE(none.ok());
    CHECK(none.value() == nullptr);
    CHECK(rig.host->peerCount() == 1);
}

TEST_CASE("net relay: buffered frames are delivered before a lost link reports Disconnected") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    REQUIRE(client->send(bytesOf("one")).ok());
    REQUIRE(client->send(bytesOf("two")).ok());
    REQUIRE(waitUntil([&] { return peer->linkStats()->bytesIn == 6; }, kLong, 2ms));
    rig.relay.dropHost(rig.room);
    CHECK(textOf(*receiveFrame(*peer)) == "one");
    CHECK(textOf(*receiveFrame(*peer)) == "two");
    CHECK(receiveError(*peer).code == ErrorCode::Disconnected);
}

TEST_CASE("net relay: without autoReregister the first loss is final and accept reports it") {
    Rig rig([](RelayHostOptions& o) { o.autoReregister = false; });
    auto [client, peer] = joinAndAccept(rig);
    rig.relay.dropHost(rig.room);
    CHECK(receiveError(*peer).code == ErrorCode::Disconnected);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    std::optional<sdk::Error> error;
    while (std::chrono::steady_clock::now() < deadline && !error) {
        auto got = rig.host->accept(50ms);
        if (!got.ok()) error = got.error();
    }
    REQUIRE(error.has_value());
    CHECK(error->code == ErrorCode::Disconnected);
    CHECK_FALSE(rig.host->registered());
    CHECK(rig.host->reregisterCount() == 0);
    CHECK(rig.relay.traffic().registrations == 1);
}

TEST_CASE("net relay: a re-registration the relay refuses for good ends the host transport") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    rig.relay.overrideRegisterRejection(relay::RoomRejectReason::BadAuth);
    rig.relay.dropHost(rig.room);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    std::optional<sdk::Error> error;
    while (std::chrono::steady_clock::now() < deadline && !error) {
        auto got = rig.host->accept(50ms);
        if (!got.ok()) error = got.error();
    }
    REQUIRE(error.has_value());
    CHECK(error->code == ErrorCode::InvalidArgument);
    CHECK(rig.host->reregisterCount() == 0);
}

TEST_CASE("net relay: a relay that stays unreachable ends re-registration after the give up time") {
    auto fake = std::make_unique<FakeRelay>();
    const std::uint16_t port = fake->port();
    RelayHostOptions options = relayHostOptionsFor("127.0.0.1", port, secretN(4));
    options.connectTimeout = 100ms;
    options.registerTimeout = 100ms;
    options.reconnectBackoffInitial = 20ms;
    options.reconnectBackoffMax = 50ms;
    options.reconnectGiveUp = 600ms;
    RelayHostTransport host(options);
    REQUIRE(host.listen().ok());
    fake.reset();

    const auto started = std::chrono::steady_clock::now();
    std::optional<sdk::Error> error;
    while (std::chrono::steady_clock::now() - started < 10s && !error) {
        auto got = host.accept(50ms);
        if (!got.ok()) error = got.error();
    }
    REQUIRE(error.has_value());
    CHECK(error->code == ErrorCode::Timeout);
    CHECK(std::chrono::steady_clock::now() - started >= 500ms);
    CHECK(host.reregisterCount() == 0);
}

TEST_CASE("net relay: a restarted relay gets the room registered afresh") {
    auto fake = std::make_unique<FakeRelay>();
    const std::uint16_t port = fake->port();
    const InviteSecret secret = secretN(5);
    RelayHostOptions options = fastOptions(*fake, secret);
    options.connectTimeout = 200ms;
    RelayHostTransport host(options);
    REQUIRE(host.listen().ok());
    RelayClientTransport joiner;
    auto client = joiner.connect(fake->endpoint(roomId(secret)), 3000ms);
    REQUIRE(client.ok());
    auto peer = host.accept(kLong);
    REQUIRE((peer.ok() && peer.value() != nullptr));

    fake.reset();  // the relay process goes away, taking its rooms
    CHECK(receiveError(*peer.value()).code == ErrorCode::Disconnected);
    CHECK(receiveError(*client.value()).code == ErrorCode::Disconnected);

    FakeRelayOptions again;
    again.port = port;
    std::unique_ptr<FakeRelay> restarted;
    REQUIRE(waitUntil(
        [&] {
            restarted = std::make_unique<FakeRelay>(again);
            if (restarted->listenError().empty()) return true;
            restarted.reset();
            return false;
        },
        kLong, 50ms));
    REQUIRE(waitUntil([&] { return host.reregisterCount() == 1; }, 10s, 5ms));
    CHECK(restarted->hasRoom(roomId(secret)));
    CHECK(host.peerCount() == 0);

    auto fresh = joiner.connect(restarted->endpoint(roomId(secret)), 3000ms);
    REQUIRE(fresh.ok());
    auto freshPeer = host.accept(kLong);
    REQUIRE((freshPeer.ok() && freshPeer.value() != nullptr));
    REQUIRE(fresh.value()->send(bytesOf("hello again")).ok());
    CHECK(textOf(*receiveFrame(*freshPeer.value())) == "hello again");
}

TEST_CASE("net relay: close during reconnection returns promptly") {
    auto fake = std::make_unique<FakeRelay>();
    RelayHostOptions options = relayHostOptionsFor("127.0.0.1", fake->port(), secretN(6));
    options.connectTimeout = 5000ms;
    options.reconnectBackoffInitial = 20ms;
    RelayHostTransport host(options);
    REQUIRE(host.listen().ok());
    fake.reset();
    std::this_thread::sleep_for(300ms);  // now inside a connect attempt that would last 5 s
    const auto started = std::chrono::steady_clock::now();
    host.close();
    CHECK(std::chrono::steady_clock::now() - started < 2s);
    CHECK(host.accept(10ms).error().code == ErrorCode::Disconnected);
}

// ---- hostile and unusual relay input -----------------------------------------------------------------------------------

TEST_CASE("net relay: a duplicate PeerJoined replaces the peer and invalid ids are ignored") {
    Rig rig;
    auto [client, peer] = joinAndAccept(rig);
    rig.relay.injectControlToHost(rig.room, relay::PeerJoined{1});
    CHECK(receiveError(*peer).code == ErrorCode::Disconnected);
    auto replacement = rig.accept();
    REQUIRE(replacement != nullptr);
    REQUIRE(client->send(bytesOf("to the new one")).ok());
    CHECK(textOf(*receiveFrame(*replacement)) == "to the new one");

    rig.relay.injectControlToHost(rig.room, relay::PeerJoined{relay::kRelayHostPeer});
    rig.relay.injectControlToHost(rig.room, relay::PeerJoined{relay::kRelayBroadcastPeer});
    rig.relay.injectControlToHost(rig.room, relay::PeerLeft{77, relay::PeerLeftReason::TimedOut});
    auto none = rig.host->accept(200ms);
    REQUIRE(none.ok());
    CHECK(none.value() == nullptr);
    CHECK(replacement->connected());
}

TEST_CASE("net relay: more peers than maxPeers are ignored") {
    Rig rig([](RelayHostOptions& o) { o.maxPeers = 2; });
    for (const relay::RelayPeerId id : std::initializer_list<relay::RelayPeerId>{10, 11, 12}) rig.relay.injectControlToHost(rig.room, relay::PeerJoined{id});
    auto first = rig.accept();
    auto second = rig.accept();
    CHECK((first != nullptr && second != nullptr));
    auto none = rig.host->accept(200ms);
    REQUIRE(none.ok());
    CHECK(none.value() == nullptr);
    CHECK(rig.host->peerCount() == 2);
}

TEST_CASE("net relay: a frame that overtakes its PeerJoined is kept and frames for unknown peers are dropped") {
    Rig rig;
    rig.relay.injectGameToHost(rig.room, 9, bytesOf("early"));
    rig.relay.injectGameToHost(rig.room, 7, bytesOf("nobody"));
    CHECK(rig.host->peerCount() == 0);
    rig.relay.injectControlToHost(rig.room, relay::PeerJoined{9});
    auto peer = rig.accept();
    REQUIRE(peer != nullptr);
    CHECK(textOf(*receiveFrame(*peer)) == "early");
    CHECK(noFrame(*peer));
    auto none = rig.host->accept(100ms);
    REQUIRE(none.ok());
    CHECK(none.value() == nullptr);  // peer 7 was never announced
}

TEST_CASE("net relay: late frames of a peer that left do not reach a peer that reuses the id") {
    Rig rig;
    rig.relay.injectControlToHost(rig.room, relay::PeerJoined{3});
    auto old = rig.accept();
    REQUIRE(old != nullptr);
    rig.relay.injectControlToHost(rig.room, relay::PeerLeft{3, relay::PeerLeftReason::Disconnected});
    CHECK(receiveError(*old).code == ErrorCode::Disconnected);
    rig.relay.injectGameToHost(rig.room, 3, bytesOf("stale"));
    std::this_thread::sleep_for(100ms);
    rig.relay.injectControlToHost(rig.room, relay::PeerJoined{3});
    auto fresh = rig.accept();
    REQUIRE(fresh != nullptr);
    CHECK(noFrame(*fresh, 200ms));
}

TEST_CASE("net relay: a flooding unreliable peer cannot grow the queue past its caps") {
    Rig rig([](RelayHostOptions& o) {
        o.maxQueuedFramesPerPeer = 8;
        o.maxQueuedBytesPerPeer = 1000;
    });
    auto [client, peer] = joinAndAccept(rig);
    // Everything the host side saw is either queued (and counted in bytesIn) or dropped; the consumer reads nothing until
    // the whole burst has been processed, so the queue length is exactly what the caps allow.
    const auto settled = [&](std::size_t frameBytes, std::uint64_t total, std::uint64_t bytesBefore, std::uint64_t droppedBefore) {
        const auto stats = peer->linkStats();
        return (stats->bytesIn - bytesBefore) / frameBytes + (stats->droppedFrames - droppedBefore) >= total;
    };
    for (int i = 0; i < 60; ++i) REQUIRE(client->sendUnreliable(patterned(400, static_cast<std::uint32_t>(i))).ok());
    REQUIRE(waitUntil([&] { return settled(400, 60, 0, 0); }, kLong, 2ms));
    std::size_t received = 0;
    while (receiveFrame(*peer, 200ms).has_value()) ++received;
    CHECK(received == 2);  // 1000 bytes hold two 400-byte frames
    CHECK(peer->linkStats()->droppedFrames >= 50);

    const auto before = *peer->linkStats();
    for (int i = 0; i < 60; ++i) REQUIRE(client->sendUnreliable(patterned(1, static_cast<std::uint32_t>(i))).ok());
    REQUIRE(waitUntil([&] { return settled(1, 60, before.bytesIn, before.droppedFrames); }, kLong, 2ms));
    received = 0;
    while (receiveFrame(*peer, 200ms).has_value()) ++received;
    CHECK(received == 8);
}

TEST_CASE("net relay: a reliable backlog past the cap closes that peer only") {
    Rig rig([](RelayHostOptions& o) { o.maxQueuedFramesPerPeer = 8; });
    auto c1 = rig.join();
    auto p1 = rig.accept();
    auto c2 = rig.join();
    auto p2 = rig.accept();
    REQUIRE((p1 != nullptr && p2 != nullptr));
    for (int i = 0; i < 50; ++i) REQUIRE(c1->send(sequenced(static_cast<std::uint32_t>(i), 1)).ok());
    CHECK(waitUntil([&] { return !p1->connected(); }, kLong, 2ms));
    CHECK(receiveError(*p1).code == ErrorCode::Disconnected);
    REQUIRE(c2->send(bytesOf("fine")).ok());
    CHECK(textOf(*receiveFrame(*p2)) == "fine");
    CHECK(p2->connected());
}

// ---- threads -----------------------------------------------------------------------------------------------------------

TEST_CASE("net relay: destroying hosts, clients and peers leaves no threads behind") {
    const int baseline = processThreadCount();
    {
        for (int round = 0; round < 3; ++round) {
            Rig rig;
            auto c1 = rig.join();
            auto p1 = rig.accept();
            auto c2 = rig.join();
            auto p2 = rig.accept();
            REQUIRE(c1->send(bytesOf("x")).ok());
            REQUIRE(receiveFrame(*p1).has_value());
            rig.relay.dropHost(rig.room);  // go through a re-registration as well
            REQUIRE(waitUntil([&] { return rig.host->reregisterCount() == 1; }, kLong, 2ms));
            auto n1 = rig.accept();
            REQUIRE(n1 != nullptr);
        }
        FakeRelay fake;
        RelayHostTransport failed(relayHostOptionsFor(fake.host(), fake.port(), secretN(1)));
        RelayClientTransport client;
        CHECK_FALSE(client.connect(fake.endpoint("0123456Z"), 1000ms).ok());
    }
    if (baseline < 0) {
        MESSAGE("thread counting unavailable on this platform");
        return;
    }
    CHECK(waitUntil([&] { return processThreadCount() <= baseline; }, kLong, 5ms));
}

// ---- the whole stack ---------------------------------------------------------------------------------------------------

TEST_CASE("net relay: invite to relay to SecureTransport to MessageChannel never shows the relay a plaintext marker") {
    const std::string marker = "PLAINTEXT-MARKER-0123456789-xyz";
    const InviteSecret secret = secretN(7);
    FakeRelay fake;
    RelayHostTransport host(relayHostOptionsFor(fake.host(), fake.port(), secret));
    REQUIRE(host.listen().ok());
    const std::string room = roomId(secret);
    CHECK(host.endpoint() == fake.endpoint(room));

    const auto codec = std::make_shared<const ipc::BinaryMessageCodec>();
    struct Side {
        std::unique_ptr<proto::MessageChannel> channel;
        sdk::Error error;
    };
    std::vector<Side> clients(2);
    std::vector<Side> hosts(2);

    for (std::size_t i = 0; i < clients.size(); ++i) {
        std::thread joiner([&, i] {
            RelayClientTransport client;
            auto raw = client.connect(host.endpoint(), 5000ms);
            if (!raw) {
                clients[i].error = raw.error();
                return;
            }
            auto secured = SecureTransport::connectAsClient(std::move(raw).value(), secret, 5000ms);
            if (!secured) {
                clients[i].error = secured.error();
                return;
            }
            clients[i].channel = std::make_unique<proto::MessageChannel>(std::move(secured).value(), codec);
        });
        auto accepted = host.accept(kLong);
        REQUIRE((accepted.ok() && accepted.value() != nullptr));
        auto secured = SecureTransport::acceptAsHost(std::move(accepted).value(), secret, 5000ms);
        joiner.join();
        REQUIRE_MESSAGE(clients[i].channel != nullptr, clients[i].error.message);
        REQUIRE_MESSAGE(secured.ok(), (secured.ok() ? "" : secured.error().message));
        hosts[i].channel = std::make_unique<proto::MessageChannel>(std::move(secured).value(), codec);
    }

    for (std::size_t i = 0; i < clients.size(); ++i) {
        proto::Goodbye goodbye;
        goodbye.reason = marker + "-client-" + std::to_string(i);
        REQUIRE(clients[i].channel->send(proto::Message{goodbye}, proto::Delivery::Reliable).ok());
        proto::Message received;
        auto got = hosts[i].channel->receive(received, 5s);
        REQUIRE((got.ok() && got.value()));
        const auto* bye = std::get_if<proto::Goodbye>(&received);
        REQUIRE(bye != nullptr);
        CHECK(bye->reason == goodbye.reason);

        proto::Log log;
        log.source = "host";
        log.message = marker + "-host-" + std::to_string(i);
        bool arrived = false;
        for (int attempt = 0; attempt < 40 && !arrived; ++attempt) {  // unreliable: resend until one arrives
            REQUIRE(hosts[i].channel->send(proto::Message{log}, proto::Delivery::Unreliable).ok());
            auto back = clients[i].channel->receive(received, 250ms);
            REQUIRE(back.ok());
            arrived = back.value();
        }
        REQUIRE(arrived);
        const auto* echoed = std::get_if<proto::Log>(&received);
        REQUIRE(echoed != nullptr);
        CHECK(echoed->message == log.message);
    }

    const FakeRelayTraffic traffic = fake.traffic();
    CHECK(traffic.toHostReliable + traffic.toHostUnreliable > 0);
    CHECK(traffic.toClientReliable + traffic.toClientUnreliable > 0);
    CHECK(traffic.malformedControl == 0);
    CHECK(traffic.unexpectedControl == 0);
    CHECK_FALSE(fake.sawBytes(marker));
    CHECK_FALSE(fake.sawBytes("Goodbye"));
}

TEST_CASE("net relay: a client with the wrong invite fails the handshake through the relay") {
    const InviteSecret secret = secretN(7);
    FakeRelay fake;
    RelayHostTransport host(relayHostOptionsFor(fake.host(), fake.port(), secret));
    REQUIRE(host.listen().ok());
    sdk::Result<std::unique_ptr<ITransport>> clientSide = sdk::makeError(ErrorCode::Internal, "not run");
    std::thread joiner([&] {
        RelayClientTransport client;
        auto raw = client.connect(host.endpoint(), 5000ms);
        if (!raw) {
            clientSide = raw.error();
            return;
        }
        clientSide = SecureTransport::connectAsClient(std::move(raw).value(), secretN(8), 1500ms);
    });
    auto accepted = host.accept(kLong);
    REQUIRE((accepted.ok() && accepted.value() != nullptr));
    auto secured = SecureTransport::acceptAsHost(std::move(accepted).value(), secret, 1500ms);
    joiner.join();
    CHECK_FALSE(secured.ok());
    CHECK_FALSE(clientSide.ok());
}
