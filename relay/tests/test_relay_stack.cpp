// The whole stack against the REAL relay on loopback sockets: RelayServer over EnetServerTransport (configured as
// wpiemu_relay is), RelayHostTransport, RelayClientTransport, SecureTransport, MessageChannel and a real invite. The
// transports are otherwise only tested against a fake relay (net/tests/fake_relay.h).

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "relay_stack_support.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/sdk/match/match_control.h"

using namespace relaystack;
using wpiemu::sdk::ErrorCode;
using wpiemu::sdk::match::MatchControl;
using wpiemu::sdk::match::MatchControlKind;
namespace rl = wpiemu::sdk::match::relay;

namespace {

constexpr std::size_t kSecureOverhead = net::SecureTransport::kRecordOverhead;

std::shared_ptr<const proto::IMessageCodec> codec() { return std::make_shared<wpiemu::ipc::BinaryMessageCodec>(); }

MatchControl pingOf(MatchControlKind kind, std::uint32_t seq, const std::string& detail) {
    MatchControl message;
    message.kind = kind;
    message.pingSeq = seq;
    message.clientTimeUs = 1'000'000 + seq;
    message.detail = detail;
    return message;
}

// The next MatchControl of kind `kind` with sequence `seq` (anything else, such as a late duplicate, is skipped).
std::optional<MatchControl> receivePing(proto::MessageChannel& channel, MatchControlKind kind, std::uint32_t seq,
                                        std::chrono::milliseconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        proto::Message message;
        const auto got = channel.receive(message, std::min<std::chrono::milliseconds>(100ms, proto::timeLeft(deadline)));
        if (!got) return std::nullopt;
        if (!got.value()) continue;
        if (const auto* control = std::get_if<MatchControl>(&message))
            if (control->kind == kind && control->pingSeq == seq) return *control;
    }
    return std::nullopt;
}

// A request from `client` answered by `host`, both over `delivery`. An unreliable one may be lost, so it is retried.
bool pingEcho(proto::MessageChannel& client, proto::MessageChannel& host, Delivery delivery, std::uint32_t seq,
              const std::string& detail) {
    const int attempts = delivery == Delivery::Reliable ? 1 : 20;
    for (int i = 0; i < attempts; ++i) {
        if (!client.send(pingOf(MatchControlKind::PingRequest, seq, detail), delivery)) return false;
        const auto request = receivePing(host, MatchControlKind::PingRequest, seq, delivery == Delivery::Reliable ? 5s : 200ms);
        if (!request || request->detail != detail) continue;
        if (!host.send(pingOf(MatchControlKind::PingReply, seq, request->detail), delivery)) return false;
        const auto reply = receivePing(client, MatchControlKind::PingReply, seq, delivery == Delivery::Reliable ? 5s : 500ms);
        if (reply && reply->detail == detail && reply->clientTimeUs == 1'000'000 + seq) return true;
    }
    return false;
}

struct Stack {
    StackRelay relay;
    Invite invite = newInvite();
    std::unique_ptr<net::RelayHostTransport> host;

    explicit Stack(const StackRelay::Tweak& tweak = {}) : relay(tweak) {
        REQUIRE(relay.started());
        host = startHost(relay, invite.secret);
    }

    SecureSession join() { return joinSecure(relay, *host, invite.room, invite.secret, invite.secret); }
};

std::string describeOf(const SecureSession& session) { return session.ok() ? session.client->describe() : session.clientError + "/" + session.hostError; }

}  // namespace

TEST_CASE("relay stack: the production configuration is what the tests run") {
    const wr::RelayOptions options = StackRelay::productionOptions();
    const net::EnetServerOptions transport = wr::transportOptionsFor(options);
    CHECK(transport.maxReliableFrameBytes == rl::kRelayMaxReliablePacket + rl::kRelayEnvelopeBytes);
    CHECK(transport.maxUnreliableFrameBytes == rl::kRelayMaxDatagram + rl::kRelayEnvelopeBytes);
    CHECK(options.server.table.maxPeersPerRoom == rl::kRelayMaxPeersPerRoom);
    CHECK(options.server.exemptLoopbackFromLockout);
    CHECK(transport.maxPeers == options.maxConnections);
}

TEST_CASE("relay stack: two clients join, handshake and exchange encrypted messages in both directions on both classes") {
    Stack stack;
    const std::string marker = "PLAINTEXT-MARKER-4f7a91c3";

    SecureSession one = stack.join();
    REQUIRE_MESSAGE(one.ok(), describeOf(one));
    SecureSession two = stack.join();
    REQUIRE_MESSAGE(two.ok(), describeOf(two));
    CHECK(one.peer->describe().find("(peer 1)") != std::string::npos);
    CHECK(two.peer->describe().find("(peer 2)") != std::string::npos);

    proto::MessageChannel client1(std::move(one.client), codec());
    proto::MessageChannel host1(std::move(one.peer), codec());
    proto::MessageChannel client2(std::move(two.client), codec());
    proto::MessageChannel host2(std::move(two.peer), codec());

    for (std::uint32_t seq = 1; seq <= 5; ++seq) {
        CHECK(pingEcho(client1, host1, Delivery::Reliable, seq, marker));
        CHECK(pingEcho(client1, host1, Delivery::Unreliable, 100 + seq, marker));
        CHECK(pingEcho(client2, host2, Delivery::Reliable, 200 + seq, marker));
        CHECK(pingEcho(client2, host2, Delivery::Unreliable, 300 + seq, marker));
    }

    // Each client reaches only its own virtual peer: a frame of client 1 never shows up at the host's peer of client 2.
    proto::Message stray;
    const auto none = host2.receive(stray, 200ms);
    REQUIRE(none.ok());
    CHECK_FALSE(none.value());

    CHECK(stack.host->peerCount() == 2);
    CHECK(stack.relay.stats().peers == 2);
    CHECK(stack.relay.stats().framesForwarded >= 40);
    CHECK(stack.relay.stats().framesDroppedTotal() == 0);
}

TEST_CASE("relay stack: the relay sees no plaintext and its log never carries the invite or a frame") {
    Stack stack;
    const std::string marker = "PLAINTEXT-MARKER-9be02d71";
    const Bytes markerBytes = bytesOf(marker);

    SecureSession session = stack.join();
    REQUIRE_MESSAGE(session.ok(), describeOf(session));
    proto::MessageChannel client(std::move(session.client), codec());
    proto::MessageChannel host(std::move(session.peer), codec());
    REQUIRE(pingEcho(client, host, Delivery::Reliable, 1, marker));
    REQUIRE(pingEcho(client, host, Delivery::Unreliable, 2, marker));
    REQUIRE(stack.relay.capture->gameFrames() >= 4);

    // What crossed the relay is ciphertext: neither the marker nor the message encoding of it is readable.
    CHECK_FALSE(stack.relay.capture->anyContains(markerBytes));
    CHECK_FALSE(stack.relay.capture->anyContains(bytesOf("PLAINTEXT")));

    // The relay holds the room id and the host authenticator and nothing else of the invite.
    const net::InviteSecret& secret = stack.invite.secret;
    CHECK_FALSE(stack.relay.capture->anyContains(std::span<const std::byte>(
        reinterpret_cast<const std::byte*>(secret.bytes().data()), secret.bytes().size())));
    const auto channelKey = net::channelKey(secret);
    CHECK_FALSE(stack.relay.capture->anyContains(std::span<const std::byte>(
        reinterpret_cast<const std::byte*>(channelKey.data()), channelKey.size())));
    const Bytes room = bytesOf(stack.invite.room);
    CHECK(stack.relay.capture->anyContains(room));

    // The relay's own log (debug level) is free of secrets, frame contents and the host authenticator.
    const std::string text = net::inviteText(secret);
    std::string plainGroups = text;
    plainGroups.erase(std::remove(plainGroups.begin(), plainGroups.end(), '-'), plainGroups.end());
    const auto auth = net::hostAuth(secret);
    CHECK(stack.relay.logs->count() > 0);
    CHECK(stack.relay.logs->contains(stack.invite.room));  // the room id is what a log line may name
    CHECK_FALSE(stack.relay.logs->contains(text));
    CHECK_FALSE(stack.relay.logs->contains(plainGroups));
    CHECK_FALSE(stack.relay.logs->contains(marker));
    CHECK_FALSE(stack.relay.logs->contains("PLAINTEXT"));
    CHECK_FALSE(stack.relay.logs->contains(hexOf(secret.bytes())));
    CHECK_FALSE(stack.relay.logs->contains(hexOf(secret.bytes(), true)));
    CHECK_FALSE(stack.relay.logs->contains(hexOf(auth)));
    CHECK_FALSE(stack.relay.logs->contains(hexOf(auth, true)));
    CHECK_FALSE(stack.relay.logs->contains(hexOf(channelKey)));
}

TEST_CASE("relay stack: boundary frame sizes through the relay, plaintext") {
    // A client's frame as sent can use the full limits (the relay adds the envelope toward the host); the host's frame
    // carries its envelope itself, so its payload is two bytes less. One byte more is refused by the sender.
    Stack stack([](wr::RelayOptions& o) { o.server.table.peerBytesPerSecond = 16 * 1024 * 1024; });
    auto joined = joinRaw(stack.relay, stack.invite.room);
    REQUIRE_MESSAGE(joined.ok(), (joined.ok() ? "" : joined.error().message));
    auto client = std::move(joined).value();
    auto peer = acceptPeer(*stack.host);
    REQUIRE(peer != nullptr);

    constexpr std::size_t kEnv = rl::kRelayEnvelopeBytes;
    const std::size_t hostReliable = rl::kRelayMaxReliablePacket - kEnv;
    const std::size_t hostUnreliable = rl::kRelayMaxDatagram - kEnv;
    const std::size_t clientReliable = rl::kRelayMaxReliablePacket;
    const std::size_t clientUnreliable = rl::kRelayMaxDatagram;

    SUBCASE("host to client") {
        CHECK(reliableRoundTrip(*peer, *client, patterned(hostReliable, 1)));
        CHECK(unreliableRoundTrip(*peer, *client, patterned(hostUnreliable, 2)));
        CHECK(peer->send(patterned(hostReliable + 1, 3)).error().code == ErrorCode::InvalidArgument);
        CHECK(peer->sendUnreliable(patterned(hostUnreliable + 1, 4)).error().code == ErrorCode::InvalidArgument);
        // On the wire the relay read the envelope-included frame and wrote the bare payload.
        CHECK(stack.relay.capture->hasGameFrameOfSize(rl::kRelayMaxReliablePacket, true));
        CHECK(stack.relay.capture->hasGameFrameOfSize(rl::kRelayMaxDatagram, true));
        CHECK(stack.relay.capture->hasGameFrameOfSize(hostReliable, false));
        CHECK(stack.relay.capture->hasGameFrameOfSize(hostUnreliable, false));
    }

    SUBCASE("client to host") {
        CHECK(reliableRoundTrip(*client, *peer, patterned(clientReliable, 5)));
        CHECK(unreliableRoundTrip(*client, *peer, patterned(clientUnreliable, 6)));
        CHECK(client->send(patterned(clientReliable + 1, 7)).error().code == ErrorCode::InvalidArgument);
        CHECK(client->sendUnreliable(patterned(clientUnreliable + 1, 8)).error().code == ErrorCode::InvalidArgument);
        // The relay read the frame as sent and wrote it to the host with the envelope: the link toward the host carries
        // limit + envelope.
        CHECK(stack.relay.capture->hasGameFrameOfSize(clientReliable, true));
        CHECK(stack.relay.capture->hasGameFrameOfSize(clientUnreliable, true));
        CHECK(stack.relay.capture->hasGameFrameOfSize(clientReliable + kEnv, false));
        CHECK(stack.relay.capture->hasGameFrameOfSize(clientUnreliable + kEnv, false));
    }

    CHECK(stack.relay.stats().dropped(wr::DropReason::Oversized) == 0);
    CHECK(stack.relay.stats().framesDroppedTotal() == 0);
    CHECK(stack.host->peerCount() == 1);
}

TEST_CASE("relay stack: boundary frame sizes through the relay, encrypted") {
    // SecureTransport adds 24 bytes to every frame, so the largest application payload is the limit minus 24 (and the
    // envelope on the host side).
    Stack stack([](wr::RelayOptions& o) { o.server.table.peerBytesPerSecond = 16 * 1024 * 1024; });
    SecureSession session = stack.join();
    REQUIRE_MESSAGE(session.ok(), describeOf(session));
    proto::ITransport& client = *session.client;
    proto::ITransport& peer = *session.peer;

    constexpr std::size_t kEnv = rl::kRelayEnvelopeBytes;
    const std::size_t hostReliable = rl::kRelayMaxReliablePacket - kEnv - kSecureOverhead;
    const std::size_t hostUnreliable = rl::kRelayMaxDatagram - kEnv - kSecureOverhead;
    const std::size_t clientReliable = rl::kRelayMaxReliablePacket - kSecureOverhead;
    const std::size_t clientUnreliable = rl::kRelayMaxDatagram - kSecureOverhead;
    REQUIRE(hostUnreliable == 1374);
    REQUIRE(clientUnreliable == 1376);

    SUBCASE("host to client") {
        CHECK(reliableRoundTrip(peer, client, patterned(hostReliable, 11)));
        CHECK(unreliableRoundTrip(peer, client, patterned(hostUnreliable, 12)));
        CHECK(peer.send(patterned(hostReliable + 1, 13)).error().code == ErrorCode::InvalidArgument);
        CHECK(peer.sendUnreliable(patterned(hostUnreliable + 1, 14)).error().code == ErrorCode::InvalidArgument);
        CHECK(stack.relay.capture->hasGameFrameOfSize(rl::kRelayMaxReliablePacket, true));
        CHECK(stack.relay.capture->hasGameFrameOfSize(rl::kRelayMaxDatagram, true));
    }

    SUBCASE("client to host") {
        CHECK(reliableRoundTrip(client, peer, patterned(clientReliable, 15)));
        CHECK(unreliableRoundTrip(client, peer, patterned(clientUnreliable, 16)));
        CHECK(client.send(patterned(clientReliable + 1, 17)).error().code == ErrorCode::InvalidArgument);
        CHECK(client.sendUnreliable(patterned(clientUnreliable + 1, 18)).error().code == ErrorCode::InvalidArgument);
        CHECK(stack.relay.capture->hasGameFrameOfSize(rl::kRelayMaxReliablePacket + kEnv, false));
        CHECK(stack.relay.capture->hasGameFrameOfSize(rl::kRelayMaxDatagram + kEnv, false));
    }

    CHECK(stack.relay.stats().framesDroppedTotal() == 0);
}

TEST_CASE("relay stack: a maximum reliable frame passes under the production rate limits") {
    Stack stack;  // the default per-peer bucket (64 KiB/s, 2 s burst) holds exactly one maximum frame
    auto joined = joinRaw(stack.relay, stack.invite.room);
    REQUIRE(joined.ok());
    auto client = std::move(joined).value();
    auto peer = acceptPeer(*stack.host);
    REQUIRE(peer != nullptr);
    CHECK(reliableRoundTrip(*client, *peer, patterned(rl::kRelayMaxReliablePacket, 21)));
    CHECK(reliableRoundTrip(*peer, *client, patterned(rl::kRelayMaxReliablePacket - rl::kRelayEnvelopeBytes, 22)));
    CHECK(stack.relay.stats().framesDroppedTotal() == 0);
}

TEST_CASE("relay stack: room full, unknown room and a wrong invite") {
    Stack stack;
    std::vector<std::unique_ptr<proto::ITransport>> clients;
    for (std::size_t i = 0; i < rl::kRelayMaxPeersPerRoom; ++i) {
        auto joined = joinRaw(stack.relay, stack.invite.room);
        REQUIRE_MESSAGE(joined.ok(), (joined.ok() ? "" : joined.error().message));
        clients.push_back(std::move(joined).value());
    }
    const auto seventh = joinRaw(stack.relay, stack.invite.room);
    REQUIRE_FALSE(seventh.ok());
    CHECK(seventh.error().code == ErrorCode::OutOfRange);
    CHECK(stack.relay.stats().peers == rl::kRelayMaxPeersPerRoom);

    const auto wrongRoom = joinRaw(stack.relay, newInvite().room);
    REQUIRE_FALSE(wrongRoom.ok());
    CHECK(wrongRoom.error().code == ErrorCode::NotFound);

    // The six that fit are all announced to the host with their own ids.
    std::vector<std::string> described;
    for (std::size_t i = 0; i < clients.size(); ++i) {
        auto peer = acceptPeer(*stack.host);
        REQUIRE(peer != nullptr);
        described.push_back(peer->describe());
    }
    for (std::size_t id = 1; id <= rl::kRelayMaxPeersPerRoom; ++id) {
        const std::string tag = "(peer " + std::to_string(id) + ")";
        CHECK(std::any_of(described.begin(), described.end(), [&](const std::string& d) { return d.find(tag) != std::string::npos; }));
    }
}

TEST_CASE("relay stack: a client with the wrong invite fails the handshake and the relay learns nothing") {
    Stack stack;
    const Invite other = newInvite();
    const std::string marker = "PLAINTEXT-MARKER-c2d8e5a0";

    // It has the room id (the relay only checks that) but not the secret.
    SecureSession bad = joinSecure(stack.relay, *stack.host, stack.invite.room, other.secret, stack.invite.secret, 1500ms);
    CHECK_FALSE(bad.ok());
    CHECK_FALSE(bad.clientError.empty());
    CHECK_FALSE(bad.hostError.empty());
    CHECK(stack.relay.capture->gameFrames() >= 2);  // the handshake frames did cross
    CHECK_FALSE(stack.relay.capture->anyContains(std::span<const std::byte>(
        reinterpret_cast<const std::byte*>(other.secret.bytes().data()), other.secret.bytes().size())));
    CHECK_FALSE(stack.relay.logs->contains(net::inviteText(other.secret)));

    // The right invite still gets in afterwards and the failure left nothing behind.
    SecureSession good = stack.join();
    REQUIRE_MESSAGE(good.ok(), describeOf(good));
    proto::MessageChannel client(std::move(good.client), codec());
    proto::MessageChannel host(std::move(good.peer), codec());
    CHECK(pingEcho(client, host, Delivery::Reliable, 1, marker));
    CHECK_FALSE(stack.relay.capture->anyContains(bytesOf(marker)));
}

TEST_CASE("relay stack: duplicate registration is RoomTaken and a wrong hostAuth after leaving is BadAuth") {
    StackRelay relay;
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    auto host = startHost(relay, invite.secret, false);

    net::RelayHostOptions imposter = hostOptions(relay, invite.secret);
    imposter.hostAuth[0] ^= 0x5A;
    imposter.autoReregister = false;
    {
        net::RelayHostTransport second(imposter);
        const sdk::Status status = second.listen();
        REQUIRE_FALSE(status.ok());
        CHECK(status.error().code == ErrorCode::AlreadyExists);
    }
    CHECK(host->registered());
    CHECK(relay.stats().registersRejected == 1);

    // The real host leaves; the room is kept for the TTL and only its hostAuth may take it back.
    host->close();
    host.reset();
    REQUIRE(waitFor([&] { return relay.stats().roomsWithoutHost == 1; }));
    {
        net::RelayHostTransport second(imposter);
        const sdk::Status status = second.listen();
        REQUIRE_FALSE(status.ok());
        CHECK(status.error().code == ErrorCode::InvalidArgument);  // BadAuth
    }
    CHECK(relay.stats().registersRejected == 2);

    auto back = startHost(relay, invite.secret);
    CHECK(back->registered());
    CHECK(relay.stats().roomsResumed == 1);
    CHECK(relay.stats().rooms == 1);
}

TEST_CASE("relay stack: a host that leaves and re-registers within the TTL gets its clients back and the handshake reruns") {
    StackRelay relay;
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    auto host = startHost(relay, invite.secret, false);

    std::vector<std::unique_ptr<proto::ITransport>> clients;
    for (int i = 0; i < 2; ++i) {
        auto joined = joinRaw(relay, invite.room);
        REQUIRE(joined.ok());
        clients.push_back(std::move(joined).value());
        REQUIRE(acceptPeer(*host) != nullptr);
    }

    host->close();
    host.reset();
    REQUIRE(waitFor([&] { return relay.stats().roomsWithoutHost == 1; }));
    CHECK(relay.stats().peers == 2);  // the clients stay in the room
    for (const auto& client : clients) CHECK(client->connected());

    auto resumed = startHost(relay, invite.secret);
    CHECK(relay.stats().roomsResumed == 1);
    CHECK(relay.stats().roomsCreated == 1);

    // The relay announces the clients that are still there: new virtual peers with the same ids.
    std::vector<std::unique_ptr<proto::ITransport>> peers(2);
    for (int i = 0; i < 2; ++i) {
        auto peer = acceptPeer(*resumed);
        REQUIRE(peer != nullptr);
        const std::string description = peer->describe();
        const std::size_t index = description.find("(peer 1)") != std::string::npos ? 0 : 1;
        REQUIRE_MESSAGE(description.find("(peer " + std::to_string(index + 1) + ")") != std::string::npos, description);
        peers[index] = std::move(peer);
    }

    // Nothing ran on the old encryption session; the handshake runs again over the same relay links.
    for (std::size_t i = 0; i < 2; ++i) {
        auto clientSide = std::async(std::launch::async, [&, i] {
            return net::SecureTransport::connectAsClient(std::move(clients[i]), invite.secret, 5s);
        });
        auto hostSide = net::SecureTransport::acceptAsHost(std::move(peers[i]), invite.secret, 5s);
        auto clientResult = clientSide.get();
        REQUIRE_MESSAGE(hostSide.ok(), (hostSide.ok() ? "" : hostSide.error().message));
        REQUIRE_MESSAGE(clientResult.ok(), (clientResult.ok() ? "" : clientResult.error().message));
        CHECK(reliableRoundTrip(*clientResult.value(), *hostSide.value(), patterned(4000, 31 + static_cast<std::uint32_t>(i))));
        CHECK(reliableRoundTrip(*hostSide.value(), *clientResult.value(), patterned(100, 41 + static_cast<std::uint32_t>(i))));
        CHECK(unreliableRoundTrip(*hostSide.value(), *clientResult.value(), patterned(500, 51 + static_cast<std::uint32_t>(i))));
    }
}

TEST_CASE("relay stack: the TTL without a host closes the room and its clients") {
    StackRelay relay([](wr::RelayOptions& o) { o.server.table.roomTtlWithoutHostMs = 500; });
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    auto host = startHost(relay, invite.secret, false);
    auto joined = joinRaw(relay, invite.room);
    REQUIRE(joined.ok());
    auto client = std::move(joined).value();
    REQUIRE(acceptPeer(*host) != nullptr);

    host->close();
    host.reset();
    REQUIRE(waitFor([&] { return relay.stats().roomsWithoutHost == 1; }));
    CHECK(client->connected());
    CHECK(receiveError(*client) == ErrorCode::Disconnected);
    REQUIRE(waitFor([&] { return relay.stats().roomsExpired == 1; }));
    CHECK(relay.stats().rooms == 0);

    const auto again = joinRaw(relay, invite.room);
    REQUIRE_FALSE(again.ok());
    CHECK(again.error().code == ErrorCode::NotFound);
}

TEST_CASE("relay stack: a restarted relay gets the room registered afresh and old links report Disconnected") {
    auto relay = std::make_unique<StackRelay>();
    REQUIRE(relay->started());
    const std::uint16_t port = relay->port();
    const Invite invite = newInvite();
    net::RelayHostOptions options = hostOptions(*relay, invite.secret);
    options.connectTimeout = 300ms;
    net::RelayHostTransport host(options);
    REQUIRE(host.listen().ok());
    auto joined = joinRaw(*relay, invite.room);
    REQUIRE(joined.ok());
    auto client = std::move(joined).value();
    auto peer = acceptPeer(host);
    REQUIRE(peer != nullptr);

    relay.reset();  // the process goes away with every room
    CHECK(receiveError(*peer) == ErrorCode::Disconnected);
    CHECK(receiveError(*client) == ErrorCode::Disconnected);

    std::unique_ptr<StackRelay> restarted;
    REQUIRE(waitFor(
        [&] {
            restarted = std::make_unique<StackRelay>(StackRelay::Tweak{}, port);
            if (restarted->started()) return true;
            restarted.reset();
            return false;
        },
        10s));
    REQUIRE(waitFor([&] { return host.reregisterCount() == 1; }, 15s));
    CHECK(restarted->stats().roomsCreated == 1);
    CHECK(restarted->stats().roomsResumed == 0);  // a new room, not a resumed one
    CHECK(host.peerCount() == 0);

    auto fresh = joinRaw(*restarted, invite.room);
    REQUIRE_MESSAGE(fresh.ok(), (fresh.ok() ? "" : fresh.error().message));
    auto freshPeer = acceptPeer(host);
    REQUIRE(freshPeer != nullptr);
    CHECK(reliableRoundTrip(*fresh.value(), *freshPeer, bytesOf("hello again")));
}

TEST_CASE("relay stack: a client that floods is limited per peer and does not starve the other client") {
    Stack stack;
    auto joinedA = joinRaw(stack.relay, stack.invite.room);
    REQUIRE(joinedA.ok());
    auto flooder = std::move(joinedA).value();
    auto peerA = acceptPeer(*stack.host);
    auto joinedB = joinRaw(stack.relay, stack.invite.room);
    REQUIRE(joinedB.ok());
    auto polite = std::move(joinedB).value();
    auto peerB = acceptPeer(*stack.host);
    REQUIRE((peerA != nullptr && peerB != nullptr));

    std::atomic<bool> floodDone{false};
    std::thread flood([&] {
        const Bytes frame(rl::kRelayMaxDatagram, std::byte{0x33});
        for (int i = 0; i < 400; ++i) (void)flooder->sendUnreliable(frame);
        floodDone = true;
    });

    constexpr int kMessages = 30;
    for (int i = 0; i < kMessages; ++i) {
        REQUIRE(polite->send(bytesOf("polite-" + std::to_string(i))).ok());
        std::this_thread::sleep_for(2ms);
    }
    flood.join();
    CHECK(floodDone.load());

    // Every message of the polite client arrives, in order, although the other peer burst far past its own limit.
    for (int i = 0; i < kMessages; ++i) {
        Bytes frame;
        const auto got = peerB->receive(frame, 5s);
        REQUIRE(got.ok());
        REQUIRE(got.value());
        CHECK(frame == bytesOf("polite-" + std::to_string(i)));
    }
    CHECK(waitFor([&] { return stack.relay.stats().dropped(wr::DropReason::RateLimited) > 0; }));
    CHECK(stack.host->peerCount() >= 1);
}

TEST_CASE("relay stack: repeated unknown-room joins lock a source out when loopback is not exempt") {
    StackRelay relay([](wr::RelayOptions& o) {
        o.server.exemptLoopbackFromLockout = false;
        o.server.table.lockoutMaxFailures = 3;
    });
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    auto host = startHost(relay, invite.secret);

    for (int i = 0; i < 3; ++i) {
        const auto attempt = joinRaw(relay, newInvite().room);
        REQUIRE_FALSE(attempt.ok());
        CHECK(attempt.error().code == ErrorCode::NotFound);
    }
    REQUIRE(waitFor([&] { return relay.stats().lockouts == 1; }));

    // The source is locked out whatever room it asks for, even the real one.
    const auto locked = joinRaw(relay, invite.room);
    REQUIRE_FALSE(locked.ok());
    CHECK(locked.error().code == ErrorCode::InvalidState);
    CHECK(relay.stats().lockedOutAttempts >= 1);
    CHECK(relay.stats().peers == 0);
}

TEST_CASE("relay stack: repeated unknown-room joins from loopback are exempt by default") {
    Stack stack;
    for (int i = 0; i < 12; ++i) {
        const auto attempt = joinRaw(stack.relay, newInvite().room);
        REQUIRE_FALSE(attempt.ok());
        CHECK(attempt.error().code == ErrorCode::NotFound);
    }
    CHECK(stack.relay.stats().lockouts == 0);
    auto joined = joinRaw(stack.relay, stack.invite.room);
    CHECK_MESSAGE(joined.ok(), (joined.ok() ? "" : joined.error().message));
}

TEST_CASE("relay stack: the health-ping room never counts as a failed join, even when loopback is not exempt") {
    StackRelay relay([](wr::RelayOptions& o) {
        o.server.exemptLoopbackFromLockout = false;
        o.server.table.lockoutMaxFailures = 3;
    });
    REQUIRE(relay.started());
    for (int i = 0; i < 12; ++i) {
        const auto attempt = joinRaw(relay, rl::kRelayPingRoomId);
        REQUIRE_FALSE(attempt.ok());
        CHECK(attempt.error().code == ErrorCode::NotFound);
    }
    CHECK(relay.stats().lockouts == 0);
    CHECK(relay.stats().pingsAnswered == 12);
    CHECK(relay.stats().joinsRejected == 0);
    CHECK(relay.stats().rooms == 0);

    // The real room is reachable straight afterwards.
    const Invite invite = newInvite();
    auto host = startHost(relay, invite.secret);
    auto joined = joinRaw(relay, invite.room);
    CHECK_MESSAGE(joined.ok(), (joined.ok() ? "" : joined.error().message));
}

TEST_CASE("relay stack: a host cannot register the health-ping room") {
    StackRelay relay;
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    net::RelayHostOptions options = hostOptions(relay, invite.secret);
    options.roomId = rl::kRelayPingRoomId;
    options.autoReregister = false;
    net::RelayHostTransport host(options);
    const sdk::Status status = host.listen();
    REQUIRE_FALSE(status.ok());
    CHECK(status.error().code == ErrorCode::AlreadyExists);
    CHECK(relay.stats().rooms == 0);
    CHECK(relay.stats().lockouts == 0);
}

TEST_CASE("relay stack: a clean host close sends CloseRoom, so the room is gone at once and its clients are disconnected") {
    StackRelay relay;  // the default TTL is 30 s: anything faster than that is CloseRoom
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    auto host = startHost(relay, invite.secret);
    std::vector<std::unique_ptr<proto::ITransport>> clients;
    for (int i = 0; i < 2; ++i) {
        auto joined = joinRaw(relay, invite.room);
        REQUIRE(joined.ok());
        clients.push_back(std::move(joined).value());
        REQUIRE(acceptPeer(*host) != nullptr);
    }
    CHECK(relay.stats().rooms == 1);

    host->close();
    REQUIRE(waitFor([&] { return relay.stats().rooms == 0; }));
    const wr::RelayStats stats = relay.stats();
    CHECK(stats.roomsClosed == 1);
    CHECK(stats.roomsExpired == 0);
    CHECK(stats.roomsWithoutHost == 0);
    CHECK(stats.peers == 0);
    CHECK(stats.controlViolations == 0);
    for (const auto& client : clients) CHECK(receiveError(*client) == ErrorCode::Disconnected);
    REQUIRE(waitFor([&] { return relay.stats().connections == 0; }));

    const auto again = joinRaw(relay, invite.room);
    REQUIRE_FALSE(again.ok());
    CHECK(again.error().code == ErrorCode::NotFound);  // not HostAbsent: the room does not exist any more
    CHECK(relay.logs->contains("room_closed"));

    // The same invite hosts a fresh room right away.
    auto next = startHost(relay, invite.secret);
    CHECK(next->registered());
    CHECK(relay.stats().roomsResumed == 0);
    CHECK(relay.stats().roomsCreated == 2);
}

TEST_CASE("relay stack: a host that leaves without CloseRoom keeps the room for its TTL") {
    StackRelay relay;
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    auto host = startHost(relay, invite.secret, false);
    host->close();
    host.reset();
    REQUIRE(waitFor([&] { return relay.stats().roomsWithoutHost == 1; }));
    CHECK(relay.stats().roomsClosed == 0);
    const auto absent = joinRaw(relay, invite.room);
    REQUIRE_FALSE(absent.ok());
    CHECK(absent.error().code == ErrorCode::Disconnected);  // HostAbsent
}

TEST_CASE("relay stack: a host that closes while its relay link is down does not need CloseRoom to shut down") {
    auto relay = std::make_unique<StackRelay>();
    REQUIRE(relay->started());
    const Invite invite = newInvite();
    net::RelayHostOptions options = hostOptions(*relay, invite.secret);
    options.connectTimeout = 300ms;
    net::RelayHostTransport host(options);
    REQUIRE(host.listen().ok());
    relay.reset();
    const auto started = std::chrono::steady_clock::now();
    host.close();
    CHECK(std::chrono::steady_clock::now() - started < 5s);
}
