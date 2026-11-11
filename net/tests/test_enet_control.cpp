#include <doctest/doctest.h>

#include <atomic>
#include <thread>

#include "enet_test_support.h"
#include "wpiemu/net/control_link.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace wpiemu::net::test;
using wpiemu::net::EnetClientOptions;
using wpiemu::net::EnetServerOptions;
using wpiemu::net::IControlLink;
using wpiemu::net::kMaxControlFrameBytes;
using wpiemu::sdk::proto::Delivery;
using wpiemu::sdk::testing::waitUntil;

namespace {

IControlLink& controlOf(ITransport& transport) {
    IControlLink* link = wpiemu::net::controlLinkOf(transport);
    REQUIRE(link != nullptr);
    return *link;
}

std::optional<Bytes> receiveControlFrame(IControlLink& link, std::chrono::milliseconds timeout = kLong) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    Bytes frame;
    for (;;) {
        auto got = link.receiveControl(frame, std::min<std::chrono::milliseconds>(50ms, wpiemu::sdk::proto::timeLeft(deadline)));
        if (!got) return std::nullopt;
        if (got.value()) return frame;
        if (std::chrono::steady_clock::now() >= deadline) return std::nullopt;
    }
}

ErrorCode controlErrorWithin(IControlLink& link, std::chrono::milliseconds timeout = kLong) {
    Bytes frame;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto got = link.receiveControl(frame, 20ms);
        if (!got) return got.error().code;
    }
    return ErrorCode::Internal;
}

struct Received {
    Bytes bytes;
    Delivery delivery;
};

std::optional<Received> receiveGameFrame(IControlLink& link, std::chrono::milliseconds timeout = kLong) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    Received out{};
    for (;;) {
        auto got = link.receiveGame(out.bytes, out.delivery,
                                    std::min<std::chrono::milliseconds>(50ms, wpiemu::sdk::proto::timeLeft(deadline)));
        if (!got) return std::nullopt;
        if (got.value()) return out;
        if (std::chrono::steady_clock::now() >= deadline) return std::nullopt;
    }
}

}  // namespace

TEST_CASE("net enet control: accepted peers and client connections are control links") {
    Pair pair = makePair();
    CHECK(wpiemu::net::controlLinkOf(*pair.client) != nullptr);
    CHECK(wpiemu::net::controlLinkOf(*pair.peer) != nullptr);
}

TEST_CASE("net enet control: control and game frames interleave without mixing, in order, both directions") {
    Pair pair = makePair();
    IControlLink& client = controlOf(*pair.client);
    IControlLink& peer = controlOf(*pair.peer);
    constexpr int kCount = 20;
    for (int i = 0; i < kCount; ++i) {
        REQUIRE(client.sendControl(bytesOf("cu" + std::to_string(i))).ok());
        REQUIRE(pair.client->send(bytesOf("gu" + std::to_string(i))).ok());
        REQUIRE(peer.sendControl(bytesOf("cd" + std::to_string(i))).ok());
        REQUIRE(pair.peer->send(bytesOf("gd" + std::to_string(i))).ok());
    }
    for (int i = 0; i < kCount; ++i) {
        auto up = receiveControlFrame(peer);
        auto down = receiveControlFrame(client);
        REQUIRE(up.has_value());
        REQUIRE(down.has_value());
        CHECK(textOf(*up) == "cu" + std::to_string(i));
        CHECK(textOf(*down) == "cd" + std::to_string(i));
    }
    for (int i = 0; i < kCount; ++i) {
        auto up = receiveFrame(*pair.peer);
        auto down = receiveFrame(*pair.client);
        REQUIRE(up.has_value());
        REQUIRE(down.has_value());
        CHECK(textOf(*up) == "gu" + std::to_string(i));  // no control frame ever shows up in receive()
        CHECK(textOf(*down) == "gd" + std::to_string(i));
    }
    Bytes frame;
    CHECK_FALSE(pair.peer->receive(frame, 20ms).value());
    CHECK_FALSE(peer.receiveControl(frame, 20ms).value());
}

TEST_CASE("net enet control: receiveGame reports the delivery class of each channel") {
    Pair pair = makePair();
    IControlLink& peer = controlOf(*pair.peer);
    IControlLink& client = controlOf(*pair.client);
    REQUIRE(pair.client->send(bytesOf("rel")).ok());
    REQUIRE(pair.client->sendUnreliable(bytesOf("unrel")).ok());
    std::optional<Delivery> reliable;
    std::optional<Delivery> unreliable;
    for (int i = 0; i < 2; ++i) {
        auto got = receiveGameFrame(peer);
        REQUIRE(got.has_value());
        (textOf(got->bytes) == "rel" ? reliable : unreliable) = got->delivery;
    }
    REQUIRE(reliable.has_value());
    REQUIRE(unreliable.has_value());
    CHECK(*reliable == Delivery::Reliable);
    CHECK(*unreliable == Delivery::Unreliable);

    REQUIRE(pair.peer->sendUnreliable(bytesOf("down")).ok());
    auto down = receiveGameFrame(client);
    REQUIRE(down.has_value());
    CHECK(textOf(down->bytes) == "down");
    CHECK(down->delivery == Delivery::Unreliable);
}

TEST_CASE("net enet control: receive and receiveGame read the same queue") {
    Pair pair = makePair();
    IControlLink& peer = controlOf(*pair.peer);
    REQUIRE(pair.client->send(bytesOf("first")).ok());
    REQUIRE(pair.client->send(bytesOf("second")).ok());
    CHECK(textOf(receiveFrame(*pair.peer).value_or(Bytes{})) == "first");
    auto second = receiveGameFrame(peer);
    REQUIRE(second.has_value());
    CHECK(textOf(second->bytes) == "second");
    CHECK(second->delivery == Delivery::Reliable);
}

TEST_CASE("net enet control: oversize and empty control frames are rejected, the maximum is accepted") {
    Pair pair = makePair();
    IControlLink& client = controlOf(*pair.client);
    IControlLink& peer = controlOf(*pair.peer);
    CHECK(client.sendControl({}).error().code == ErrorCode::InvalidArgument);
    CHECK(client.sendControl(patterned(kMaxControlFrameBytes + 1, 1)).error().code == ErrorCode::InvalidArgument);
    const Bytes biggest = patterned(kMaxControlFrameBytes, 2);
    REQUIRE(client.sendControl(biggest).ok());
    auto got = receiveControlFrame(peer);
    REQUIRE(got.has_value());
    CHECK(*got == biggest);
    CHECK(pair.client->linkStats()->droppedFrames == 0);
}

TEST_CASE("net enet control: the control rate cap drops frames and counts them") {
    auto clock = std::make_shared<ManualClock>();
    EnetServerOptions options = loopbackServer();
    options.clock = clock;
    options.maxControlFramesPerSecond = 10;
    Pair pair = makePair(options);
    IControlLink& client = controlOf(*pair.client);
    IControlLink& peer = controlOf(*pair.peer);
    for (int i = 0; i < 25; ++i) REQUIRE(client.sendControl(bytesOf(std::to_string(i))).ok());
    CHECK(waitUntil([&] { return pair.peer->linkStats()->droppedFrames == 15; }, kLong, 2ms));
    for (int i = 0; i < 10; ++i) {
        auto got = receiveControlFrame(peer);
        REQUIRE(got.has_value());
        CHECK(textOf(*got) == std::to_string(i));  // the first ten got through
    }
    Bytes frame;
    CHECK_FALSE(peer.receiveControl(frame, 20ms).value());

    // Game frames have their own cap and are unaffected.
    REQUIRE(pair.client->send(bytesOf("game")).ok());
    CHECK(textOf(receiveFrame(*pair.peer).value_or(Bytes{})) == "game");
    CHECK(pair.peer->linkStats()->droppedFrames == 15);

    clock->advance(1000ms);  // a second of credit comes back
    for (int i = 0; i < 12; ++i) REQUIRE(client.sendControl(bytesOf("again")).ok());
    CHECK(waitUntil([&] { return pair.peer->linkStats()->droppedFrames == 17; }, kLong, 2ms));
    CHECK(pair.peer->connected());
}

TEST_CASE("net enet control: the control byte cap drops frames and counts them") {
    auto clock = std::make_shared<ManualClock>();
    EnetServerOptions options = loopbackServer();
    options.clock = clock;
    options.maxControlBytesPerSecond = 1000;  // the bucket still holds one maximum frame, 4096 bytes
    Pair pair = makePair(options);
    IControlLink& client = controlOf(*pair.client);
    for (int i = 0; i < 3; ++i) REQUIRE(client.sendControl(patterned(2000, 30 + static_cast<unsigned>(i))).ok());
    CHECK(waitUntil([&] { return pair.peer->linkStats()->droppedFrames == 1; }, kLong, 2ms));
    IControlLink& peer = controlOf(*pair.peer);
    CHECK(receiveControlFrame(peer).has_value());
    CHECK(receiveControlFrame(peer).has_value());
    Bytes frame;
    CHECK_FALSE(peer.receiveControl(frame, 20ms).value());
}

TEST_CASE("net enet control: a control frame counts as activity for the unauthenticated timeout") {
    auto clock = std::make_shared<ManualClock>();
    EnetServerOptions options = loopbackServer();
    options.clock = clock;
    options.unauthenticatedTimeout = 2000ms;
    Pair silent = makePair(options);
    auto talker = connectTo(silent.server->endpoint());
    auto talkerPeer = acceptOne(*silent.server);
    REQUIRE(talkerPeer != nullptr);
    REQUIRE(controlOf(*talker).sendControl(bytesOf("JoinRoom")).ok());
    REQUIRE(receiveControlFrame(controlOf(*talkerPeer)).has_value());

    clock->advance(3000ms);
    CHECK(receiveError(*silent.client).code == ErrorCode::Disconnected);  // the silent one is dropped
    CHECK(talker->connected());
    CHECK(talkerPeer->connected());
    CHECK(waitUntil([&] { return silent.server->peerCount() == 1; }, kLong, 2ms));
}

TEST_CASE("net enet control: a disconnecting peer delivers buffered control frames then Disconnected") {
    Pair pair = makePair();
    IControlLink& client = controlOf(*pair.client);
    for (int i = 0; i < 3; ++i) REQUIRE(controlOf(*pair.peer).sendControl(bytesOf("bye " + std::to_string(i))).ok());
    REQUIRE(pair.peer->send(bytesOf("game bye")).ok());
    pair.peer.reset();
    for (int i = 0; i < 3; ++i) {
        auto got = receiveControlFrame(client);
        REQUIRE(got.has_value());
        CHECK(textOf(*got) == "bye " + std::to_string(i));
    }
    CHECK(controlErrorWithin(client) == ErrorCode::Disconnected);
    CHECK(textOf(receiveFrame(*pair.client).value_or(Bytes{})) == "game bye");
    CHECK(receiveError(*pair.client).code == ErrorCode::Disconnected);
}

TEST_CASE("net enet control: close wakes a blocked receiveControl and receiveGame") {
    Pair pair = makePair();
    for (ITransport* side : {pair.client.get(), pair.peer.get()}) {
        IControlLink& link = controlOf(*side);
        std::atomic<int> returned{0};
        ErrorCode controlCode = ErrorCode::Internal;
        ErrorCode gameCode = ErrorCode::Internal;
        std::thread control([&] {
            Bytes frame;
            auto got = link.receiveControl(frame, 20s);
            if (!got) controlCode = got.error().code;
            ++returned;
        });
        std::thread game([&] {
            Bytes frame;
            Delivery delivery{};
            auto got = link.receiveGame(frame, delivery, 20s);
            if (!got) gameCode = got.error().code;
            ++returned;
        });
        std::this_thread::sleep_for(50ms);
        side->close();
        CHECK(waitUntil([&] { return returned.load() == 2; }, 3000ms, 1ms));
        control.join();
        game.join();
        CHECK(controlCode == ErrorCode::Disconnected);
        CHECK(gameCode == ErrorCode::Disconnected);
        CHECK(link.sendControl(bytesOf("x")).error().code == ErrorCode::Disconnected);
    }
}

TEST_CASE("net enet control: closing the server ends receiveControl on its peers and clients") {
    Pair pair = makePair();
    pair.server->close();
    CHECK(controlErrorWithin(controlOf(*pair.client)) == ErrorCode::Disconnected);
    CHECK(controlErrorWithin(controlOf(*pair.peer)) == ErrorCode::Disconnected);
}

TEST_CASE("net enet control: an unread control backlog beyond the cap disconnects the peer") {
    EnetServerOptions options = loopbackServer();
    options.maxQueuedControlFrames = 4;
    Pair pair = makePair(options);
    IControlLink& client = controlOf(*pair.client);
    for (int i = 0; i < 30; ++i)
        if (!client.sendControl(bytesOf(std::to_string(i))).ok()) break;
    CHECK(waitUntil([&] { return !pair.client->connected(); }, kLong, 2ms));
    IControlLink& peer = controlOf(*pair.peer);
    int buffered = 0;
    Bytes frame;
    for (;;) {
        auto got = peer.receiveControl(frame, 20ms);
        if (!got) {
            CHECK(got.error().code == ErrorCode::Disconnected);
            break;
        }
        if (got.value()) ++buffered;
    }
    CHECK(buffered <= 4);
}
