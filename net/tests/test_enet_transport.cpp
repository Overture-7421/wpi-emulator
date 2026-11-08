#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <tlhelp32.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "enet_test_support.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/testing/wait.h"

using namespace wpiemu::net::test;
using wpiemu::net::EnetClientOptions;
using wpiemu::net::EnetClientTransport;
using wpiemu::net::EnetServerOptions;
using wpiemu::net::EnetServerTransport;
using wpiemu::sdk::testing::waitUntil;

namespace {

int processThreadCount() {
#ifdef _WIN32
    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snapshot == INVALID_HANDLE_VALUE) return -1;
    THREADENTRY32 entry{};
    entry.dwSize = sizeof entry;
    int count = 0;
    if (Thread32First(snapshot, &entry)) {
        do {
            if (entry.th32OwnerProcessID == GetCurrentProcessId()) ++count;
        } while (Thread32Next(snapshot, &entry));
    }
    CloseHandle(snapshot);
    return count;
#elif defined(__linux__)
    std::error_code ec;
    int count = 0;
    for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task", ec)) {
        (void)entry;
        ++count;
    }
    return ec ? -1 : count;
#else
    return -1;
#endif
}

std::uint16_t portOf(const std::string& endpoint) {
    return static_cast<std::uint16_t>(std::stoi(endpoint.substr(endpoint.rfind(':') + 1)));
}

// Sends datagrams that are not ENet to a UDP port on 127.0.0.1.
void sendGarbageDatagrams(std::uint16_t port) {
#ifdef _WIN32
    WSADATA data{};
    REQUIRE(WSAStartup(MAKEWORD(2, 2), &data) == 0);
    const SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    REQUIRE(sock != INVALID_SOCKET);
#else
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    REQUIRE(sock >= 0);
#endif
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &target.sin_addr);
    std::mt19937 rng(7);
    std::vector<std::vector<char>> packets;
    packets.push_back({});
    packets.push_back({'\x00'});
    packets.push_back(std::vector<char>(4, '\0'));
    packets.push_back(std::vector<char>(1400, '\xFF'));
    packets.push_back(std::vector<char>(12, '\x80'));  // looks like an ENet header with the sentinel peer id
    for (int i = 0; i < 300; ++i) {
        std::vector<char> packet(1 + rng() % 1400);
        for (auto& c : packet) c = static_cast<char>(rng() & 0xFF);
        if (i % 3 == 0 && packet.size() > 4) packet[0] = '\xFF', packet[1] = '\xFF';  // peer id 0xFFFF: a connect
        packets.push_back(std::move(packet));
    }
    for (const auto& packet : packets)
        (void)sendto(sock, packet.data(), static_cast<int>(packet.size()), 0, reinterpret_cast<const sockaddr*>(&target),
                     sizeof target);
#ifdef _WIN32
    closesocket(sock);
    WSACleanup();
#else
    close(sock);
#endif
}

struct NoStatsTransport final : ITransport {
    wpiemu::sdk::Status send(std::span<const std::byte>) override { return wpiemu::sdk::okStatus(); }
    wpiemu::sdk::Result<bool> receive(Bytes&, std::chrono::milliseconds) override { return false; }
    void close() override {}
    bool connected() const override { return true; }
    std::string describe() const override { return "none"; }
};

}  // namespace

TEST_CASE("net enet: an ITransport without link measurement reports none") {
    NoStatsTransport transport;
    CHECK_FALSE(transport.linkStats().has_value());
}

TEST_CASE("net enet: connect accept and reliable frames both directions over IPv4") {
    Pair pair = makePair();
    CHECK(pair.server->endpoint().rfind("enet://127.0.0.1:", 0) == 0);
    CHECK(portOf(pair.server->endpoint()) != 0);
    CHECK(pair.client->connected());
    CHECK(pair.peer->connected());
    CHECK(pair.client->describe().rfind("enet://127.0.0.1:", 0) == 0);

    REQUIRE(pair.client->send(bytesOf("hello server")).ok());
    REQUIRE(pair.peer->send(bytesOf("hello client")).ok());
    auto atServer = receiveFrame(*pair.peer);
    auto atClient = receiveFrame(*pair.client);
    REQUIRE(atServer.has_value());
    REQUIRE(atClient.has_value());
    CHECK(textOf(*atServer) == "hello server");
    CHECK(textOf(*atClient) == "hello client");

    REQUIRE(pair.client->send({}).error().code == ErrorCode::InvalidArgument);  // empty frames are rejected
    REQUIRE(pair.client->sendUnreliable({}).error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("net enet: connect accept and frames over IPv6 loopback when available") {
    EnetServerTransport server(loopbackServer("::1"));
    if (!server.listen().ok()) {
        MESSAGE("IPv6 loopback unavailable; skipped");
        return;
    }
    CHECK(server.endpoint().rfind("enet://[::1]:", 0) == 0);
    auto client = connectTo(server.endpoint());
    auto peer = acceptOne(server);
    REQUIRE(peer != nullptr);
    REQUIRE(client->send(bytesOf("v6 up")).ok());
    REQUIRE(peer->send(bytesOf("v6 down")).ok());
    CHECK(textOf(receiveFrame(*peer).value_or(Bytes{})) == "v6 up");
    CHECK(textOf(receiveFrame(*client).value_or(Bytes{})) == "v6 down");
    CHECK(client->describe().rfind("enet://[::1]:", 0) == 0);
}

TEST_CASE("net enet: a dual-stack server accepts IPv4 and IPv6 clients") {
    EnetServerTransport server{EnetServerOptions{}};
    REQUIRE(server.listen().ok());
    const std::uint16_t port = portOf(server.endpoint());
    auto v4 = connectTo("enet://127.0.0.1:" + std::to_string(port));
    auto peer4 = acceptOne(server);
    REQUIRE(peer4 != nullptr);
    REQUIRE(v4->send(bytesOf("four")).ok());
    CHECK(textOf(receiveFrame(*peer4).value_or(Bytes{})) == "four");
    REQUIRE(peer4->send(bytesOf("four back")).ok());
    CHECK(textOf(receiveFrame(*v4).value_or(Bytes{})) == "four back");

    if (server.endpoint().rfind("enet://[::]:", 0) != 0) {
        MESSAGE("no IPv6 on this machine: the server fell back to IPv4 only");
        return;
    }
    EnetClientTransport client6;
    auto v6 = client6.connect("enet://[::1]:" + std::to_string(port), 3000ms);
    REQUIRE(v6.ok());
    auto peer6 = acceptOne(server);
    REQUIRE(peer6 != nullptr);
    REQUIRE(v6.value()->send(bytesOf("six")).ok());
    CHECK(textOf(receiveFrame(*peer6).value_or(Bytes{})) == "six");
}

TEST_CASE("net enet: hostnames resolve on connect") {
    auto server = listening(loopbackServer());
    auto client = connectTo("enet://localhost:" + std::to_string(portOf(server->endpoint())));
    auto peer = acceptOne(*server);
    REQUIRE(peer != nullptr);
    REQUIRE(client->send(bytesOf("by name")).ok());
    CHECK(textOf(receiveFrame(*peer).value_or(Bytes{})) == "by name");
}

TEST_CASE("net enet: unreliable frames arrive in order under no loss and the size limit is enforced") {
    Pair pair = makePair();
    for (int i = 0; i < 100; ++i) {
        REQUIRE(pair.client->sendUnreliable(bytesOf("u" + std::to_string(i))).ok());
        REQUIRE(pair.peer->sendUnreliable(bytesOf("d" + std::to_string(i))).ok());
    }
    int nextUp = 0;
    int nextDown = 0;
    CHECK(waitUntil(
        [&] {
            Bytes frame;
            while (pair.peer->receive(frame, 0ms).valueOr(false)) {
                CHECK(textOf(frame) == "u" + std::to_string(nextUp));
                ++nextUp;
            }
            while (pair.client->receive(frame, 0ms).valueOr(false)) {
                CHECK(textOf(frame) == "d" + std::to_string(nextDown));
                ++nextDown;
            }
            return nextUp == 100 && nextDown == 100;
        },
        kLong, 2ms));

    const Bytes maxDatagram = patterned(wpiemu::sdk::match::relay::kRelayMaxDatagram, 5);
    REQUIRE(pair.client->sendUnreliable(maxDatagram).ok());
    auto got = receiveFrame(*pair.peer);
    REQUIRE(got.has_value());
    CHECK(*got == maxDatagram);

    const Bytes tooBig = patterned(wpiemu::sdk::match::relay::kRelayMaxDatagram + 1, 6);
    auto rejected = pair.client->sendUnreliable(tooBig);
    REQUIRE_FALSE(rejected.ok());
    CHECK(rejected.error().code == ErrorCode::InvalidArgument);
    // The same frame is fine on the reliable channel, and nothing was queued by the rejected send.
    REQUIRE(pair.client->send(tooBig).ok());
    auto reliable = receiveFrame(*pair.peer);
    REQUIRE(reliable.has_value());
    CHECK(*reliable == tooBig);
}

TEST_CASE("net enet: a MessageChannel with the binary codec round trips Goodbye reliably and unreliably") {
    Pair pair = makePair();
    auto codec = std::make_shared<const wpiemu::ipc::BinaryMessageCodec>();
    wpiemu::sdk::proto::MessageChannel client(std::move(pair.client), codec);
    wpiemu::sdk::proto::MessageChannel server(std::move(pair.peer), codec);

    wpiemu::sdk::proto::Goodbye bye;
    bye.reason = "all done";
    for (const auto delivery : {wpiemu::sdk::proto::Delivery::Reliable, wpiemu::sdk::proto::Delivery::Unreliable}) {
        REQUIRE(client.send(wpiemu::sdk::proto::Message{bye}, delivery).ok());
        wpiemu::sdk::proto::Message received;
        bool got = false;
        CHECK(waitUntil(
            [&] {
                auto r = server.receive(received, 20ms);
                REQUIRE(r.ok());
                got = r.value();
                return got;
            },
            kLong, 0ms));
        REQUIRE(std::holds_alternative<wpiemu::sdk::proto::Goodbye>(received));
        CHECK(std::get<wpiemu::sdk::proto::Goodbye>(received).reason == "all done");
    }
}

TEST_CASE("net enet: many small reliable frames keep their order") {
    Pair pair = makePair();
    constexpr int kFrames = 1500;
    std::thread sender([&] {
        for (int i = 0; i < kFrames; ++i) {
            std::string text = std::to_string(i);
            REQUIRE(pair.client->send(bytesOf(text)).ok());
        }
    });
    int expected = 0;
    while (expected < kFrames) {
        auto frame = receiveFrame(*pair.peer, kLong);
        REQUIRE(frame.has_value());
        REQUIRE(textOf(*frame) == std::to_string(expected));
        ++expected;
    }
    sender.join();
}

TEST_CASE("net enet: a 128 KiB reliable frame is fragmented and reassembled both ways") {
    Pair pair = makePair();
    const Bytes big = patterned(wpiemu::sdk::match::relay::kRelayMaxReliablePacket, 11);
    const Bytes other = patterned(wpiemu::sdk::match::relay::kRelayMaxReliablePacket, 12);
    REQUIRE(pair.client->send(big).ok());
    REQUIRE(pair.peer->send(other).ok());
    auto atServer = receiveFrame(*pair.peer, 10s);
    auto atClient = receiveFrame(*pair.client, 10s);
    REQUIRE(atServer.has_value());
    REQUIRE(atClient.has_value());
    CHECK(*atServer == big);
    CHECK(*atClient == other);

    const Bytes over = patterned(wpiemu::sdk::match::relay::kRelayMaxReliablePacket + 1, 13);
    auto rejected = pair.client->send(over);
    REQUIRE_FALSE(rejected.ok());
    CHECK(rejected.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("net enet: concurrent senders and receivers on both ends") {
    Pair pair = makePair();
    constexpr int kFrames = 600;
    std::atomic<int> receivedAtServer{0};
    std::atomic<int> receivedAtClient{0};
    std::atomic<bool> ordered{true};
    auto drain = [&](ITransport& transport, std::atomic<int>& count) {
        const auto deadline = std::chrono::steady_clock::now() + 10s;
        Bytes frame;
        while (count.load() < kFrames && std::chrono::steady_clock::now() < deadline) {
            auto got = transport.receive(frame, 50ms);
            if (!got) return;
            if (got.value()) {
                if (textOf(frame) != std::to_string(count.load())) ordered = false;
                ++count;
            }
        }
    };
    auto push = [&](ITransport& transport) {
        for (int i = 0; i < kFrames; ++i)
            if (!transport.send(bytesOf(std::to_string(i))).ok()) return;
    };
    std::thread t1(push, std::ref(*pair.client));
    std::thread t2(push, std::ref(*pair.peer));
    std::thread t3(drain, std::ref(*pair.peer), std::ref(receivedAtServer));
    std::thread t4(drain, std::ref(*pair.client), std::ref(receivedAtClient));
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    CHECK(receivedAtServer == kFrames);
    CHECK(receivedAtClient == kFrames);
    CHECK(ordered.load());
}

TEST_CASE("net enet: close wakes a blocked receive on either end") {
    Pair pair = makePair();
    for (ITransport* side : {pair.client.get(), pair.peer.get()}) {
        std::atomic<bool> returned{false};
        wpiemu::sdk::Error error;
        std::thread waiter([&] {
            Bytes frame;
            auto got = side->receive(frame, 20s);
            if (!got) error = got.error();
            returned = true;
        });
        std::this_thread::sleep_for(50ms);
        const auto start = std::chrono::steady_clock::now();
        side->close();
        CHECK(waitUntil([&] { return returned.load(); }, 3000ms, 1ms));
        waiter.join();
        CHECK(std::chrono::steady_clock::now() - start < 3000ms);
        CHECK(error.code == ErrorCode::Disconnected);
        CHECK_FALSE(side->connected());
        CHECK(side->send(bytesOf("x")).error().code == ErrorCode::Disconnected);
    }
}

TEST_CASE("net enet: a disconnecting peer delivers its buffered frames then Disconnected") {
    Pair pair = makePair();
    for (int i = 0; i < 3; ++i) REQUIRE(pair.peer->send(bytesOf("last words " + std::to_string(i))).ok());
    pair.peer.reset();  // polite: frames already sent are flushed before the disconnect
    for (int i = 0; i < 3; ++i) {
        auto frame = receiveFrame(*pair.client);
        REQUIRE(frame.has_value());
        CHECK(textOf(*frame) == "last words " + std::to_string(i));
    }
    CHECK(receiveError(*pair.client).code == ErrorCode::Disconnected);
    CHECK_FALSE(pair.client->connected());
    CHECK(waitUntil([&] { return pair.server->peerCount() == 0; }, kLong, 2ms));
}

TEST_CASE("net enet: a client that goes away is seen by the server") {
    Pair pair = makePair();
    REQUIRE(pair.client->send(bytesOf("bye")).ok());
    pair.client.reset();
    auto frame = receiveFrame(*pair.peer);
    REQUIRE(frame.has_value());
    CHECK(textOf(*frame) == "bye");
    CHECK(receiveError(*pair.peer).code == ErrorCode::Disconnected);
}

TEST_CASE("net enet: closing the server disconnects its peers and ends accept") {
    Pair pair = makePair();
    pair.server->close();
    CHECK(receiveError(*pair.client).code == ErrorCode::Disconnected);
    CHECK(receiveError(*pair.peer).code == ErrorCode::Disconnected);
    CHECK_FALSE(pair.client->connected());
    CHECK(pair.server->accept(10ms).error().code == ErrorCode::Disconnected);
    pair.server->close();  // idempotent
}

TEST_CASE("net enet: destroying the server leaves live peer transports safe to use") {
    Pair pair = makePair();
    REQUIRE(pair.peer->send(bytesOf("before")).ok());
    auto before = receiveFrame(*pair.client);
    REQUIRE(before.has_value());
    pair.server.reset();
    CHECK(receiveError(*pair.peer).code == ErrorCode::Disconnected);
    CHECK_FALSE(pair.peer->connected());
    CHECK(pair.peer->send(bytesOf("after")).error().code == ErrorCode::Disconnected);
    CHECK(pair.peer->linkStats().has_value());
    pair.peer.reset();
    pair.client.reset();
}

TEST_CASE("net enet: a peer over the limit is dropped and only maxPeers are accepted") {
    EnetServerOptions options = loopbackServer();
    options.maxPeers = 2;
    auto server = listening(options);
    auto a = connectTo(server->endpoint());
    auto b = connectTo(server->endpoint());
    auto peerA = acceptOne(*server);
    auto peerB = acceptOne(*server);
    REQUIRE(peerA != nullptr);
    REQUIRE(peerB != nullptr);

    EnetClientTransport extra;
    auto third = extra.connect(server->endpoint(), 3000ms);
    if (third.ok()) {  // a refusal may arrive just after the connection is acknowledged
        CHECK(receiveError(*third.value()).code == ErrorCode::Disconnected);
        CHECK_FALSE(third.value()->connected());
    } else {
        CHECK((third.error().code == ErrorCode::Disconnected || third.error().code == ErrorCode::Timeout));
    }
    auto unexpected = server->accept(100ms);
    REQUIRE(unexpected.ok());
    CHECK(unexpected.value() == nullptr);
    CHECK(server->peerCount() == 2);

    // The accepted peers still work, and a freed slot is reusable.
    REQUIRE(a->send(bytesOf("still here")).ok());
    CHECK(textOf(receiveFrame(*peerA).value_or(Bytes{})) == "still here");
    peerB.reset();
    CHECK(waitUntil([&] { return server->peerCount() == 1; }, kLong, 2ms));
    auto again = connectTo(server->endpoint());
    auto peerC = acceptOne(*server);
    CHECK(peerC != nullptr);
}

TEST_CASE("net enet: a peer that sends nothing is dropped after the unauthenticated timeout") {
    auto clock = std::make_shared<ManualClock>();
    EnetServerOptions options = loopbackServer();
    options.clock = clock;
    options.unauthenticatedTimeout = 2000ms;
    Pair pair = makePair(options);
    CHECK(pair.client->connected());

    // A peer that has sent a frame is past the gate.
    auto talker = connectTo(pair.server->endpoint());
    auto talkerPeer = acceptOne(*pair.server);
    REQUIRE(talker->send(bytesOf("join")).ok());
    REQUIRE(receiveFrame(*talkerPeer).has_value());

    clock->advance(3000ms);
    CHECK(receiveError(*pair.client).code == ErrorCode::Disconnected);
    CHECK(receiveError(*pair.peer).code == ErrorCode::Disconnected);
    CHECK(talker->connected());
    CHECK(talkerPeer->connected());
    CHECK(waitUntil([&] { return pair.server->peerCount() == 1; }, kLong, 2ms));
}

TEST_CASE("net enet: the unauthenticated timeout also works on the real clock") {
    EnetServerOptions options = loopbackServer();
    options.unauthenticatedTimeout = 150ms;
    Pair pair = makePair(options);
    const auto start = std::chrono::steady_clock::now();
    CHECK(receiveError(*pair.client).code == ErrorCode::Disconnected);
    CHECK(std::chrono::steady_clock::now() - start < 3000ms);
}

TEST_CASE("net enet: the receive rate cap drops frames and counts them") {
    auto clock = std::make_shared<ManualClock>();
    EnetServerOptions options = loopbackServer();
    options.clock = clock;
    options.maxFramesPerSecond = 10;
    Pair pair = makePair(options);
    for (int i = 0; i < 40; ++i) REQUIRE(pair.client->send(bytesOf(std::to_string(i))).ok());
    CHECK(waitUntil([&] { return pair.peer->linkStats()->droppedFrames == 30; }, kLong, 2ms));
    int received = 0;
    Bytes frame;
    while (pair.peer->receive(frame, 0ms).valueOr(false)) {
        CHECK(textOf(frame) == std::to_string(received));  // the first ten got through
        ++received;
    }
    CHECK(received == 10);
    CHECK(pair.peer->linkStats()->bytesIn > 0);

    clock->advance(1000ms);  // a second of credit comes back
    for (int i = 0; i < 12; ++i) REQUIRE(pair.client->send(bytesOf("again")).ok());
    CHECK(waitUntil([&] { return pair.peer->linkStats()->droppedFrames == 32; }, kLong, 2ms));
    CHECK(pair.peer->connected());
}

TEST_CASE("net enet: the receive byte cap drops frames and counts them") {
    auto clock = std::make_shared<ManualClock>();
    EnetServerOptions options = loopbackServer();
    options.clock = clock;
    options.maxBytesPerSecond = 1000;
    options.maxReliableFrameBytes = 400;
    EnetClientOptions clientOptions;
    clientOptions.maxReliableFrameBytes = 400;
    Pair pair = makePair(options, clientOptions);
    for (int i = 0; i < 5; ++i) REQUIRE(pair.client->send(patterned(300, 20 + static_cast<unsigned>(i))).ok());
    CHECK(waitUntil([&] { return pair.peer->linkStats()->droppedFrames == 2; }, kLong, 2ms));
    int received = 0;
    Bytes frame;
    while (pair.peer->receive(frame, 0ms).valueOr(false)) ++received;
    CHECK(received == 3);
}

TEST_CASE("net enet: an unread backlog beyond the cap disconnects the peer") {
    EnetServerOptions options = loopbackServer();
    options.maxQueuedInboundFrames = 8;
    Pair pair = makePair(options);
    for (int i = 0; i < 100; ++i)
        if (!pair.client->send(bytesOf(std::to_string(i))).ok()) break;
    CHECK(waitUntil([&] { return !pair.client->connected(); }, kLong, 2ms));
    int buffered = 0;
    Bytes frame;
    for (;;) {
        auto got = pair.peer->receive(frame, 20ms);
        if (!got) {
            CHECK(got.error().code == ErrorCode::Disconnected);
            break;
        }
        if (got.value()) ++buffered;
    }
    CHECK(buffered <= 8);
}

TEST_CASE("net enet: send backlog limits fail reliable and drop unreliable") {
    EnetClientOptions clientOptions;
    clientOptions.maxQueuedOutboundBytes = 1000;
    Pair pair = makePair(loopbackServer(), clientOptions);
    auto reliable = pair.client->send(patterned(2000, 1));
    REQUIRE_FALSE(reliable.ok());
    CHECK(reliable.error().code == ErrorCode::IoError);
    REQUIRE(pair.client->sendUnreliable(patterned(1200, 2)).ok());
    CHECK(pair.client->linkStats()->droppedFrames == 1);
    REQUIRE(pair.client->send(bytesOf("small")).ok());
    CHECK(textOf(receiveFrame(*pair.peer).value_or(Bytes{})) == "small");
}

TEST_CASE("net enet: garbage datagrams do not disturb the server") {
    auto server = listening(loopbackServer());
    sendGarbageDatagrams(portOf(server->endpoint()));
    auto client = connectTo(server->endpoint());
    auto peer = acceptOne(*server);
    REQUIRE(peer != nullptr);
    sendGarbageDatagrams(portOf(server->endpoint()));
    REQUIRE(client->send(bytesOf("still fine")).ok());
    CHECK(textOf(receiveFrame(*peer).value_or(Bytes{})) == "still fine");
    CHECK(server->peerCount() == 1);
}

TEST_CASE("net enet: servers and clients can be created and torn down repeatedly and overlapped") {
    for (int round = 0; round < 4; ++round) {
        auto first = listening(loopbackServer());
        auto second = listening(loopbackServer());
        auto c1 = connectTo(first->endpoint());
        auto c2 = connectTo(second->endpoint());
        auto p1 = acceptOne(*first);
        auto p2 = acceptOne(*second);
        REQUIRE(c1->send(bytesOf("1")).ok());
        REQUIRE(c2->send(bytesOf("2")).ok());
        CHECK(textOf(receiveFrame(*p1).value_or(Bytes{})) == "1");
        CHECK(textOf(receiveFrame(*p2).value_or(Bytes{})) == "2");
        first.reset();  // the first server goes while the second is still serving
        c2->send(bytesOf("again")).ok();
        CHECK(textOf(receiveFrame(*p2).value_or(Bytes{})) == "again");
    }
    // After everything is gone a fresh one still works (ENet re-initializes).
    Pair pair = makePair();
    REQUIRE(pair.client->send(bytesOf("fresh")).ok());
    CHECK(textOf(receiveFrame(*pair.peer).value_or(Bytes{})) == "fresh");
}

TEST_CASE("net enet: repeated connect and disconnect leaves no threads behind") {
    const int baseline = processThreadCount();
    {
        auto server = listening(loopbackServer());
        for (int i = 0; i < 25; ++i) {
            auto client = connectTo(server->endpoint());
            auto peer = acceptOne(*server);
            REQUIRE(peer != nullptr);
            REQUIRE(client->send(bytesOf("x")).ok());
            REQUIRE(receiveFrame(*peer).has_value());
        }
        CHECK(waitUntil([&] { return server->peerCount() == 0; }, kLong, 2ms));
    }
    if (baseline < 0) {
        MESSAGE("thread counting unavailable on this platform");
        return;
    }
    CHECK(waitUntil([&] { return processThreadCount() <= baseline; }, kLong, 5ms));
}

TEST_CASE("net enet: linkStats reports a plausible round trip time and byte counts") {
    Pair pair = makePair();
    REQUIRE(pair.client->send(patterned(1000, 3)).ok());
    REQUIRE(receiveFrame(*pair.peer).has_value());
    REQUIRE(pair.peer->send(patterned(500, 4)).ok());
    REQUIRE(receiveFrame(*pair.client).has_value());

    for (ITransport* side : {pair.client.get(), pair.peer.get()}) {
        CHECK(waitUntil([&] { return side->linkStats()->rttMs < 100; }, kLong, 5ms));
        const auto stats = side->linkStats();
        REQUIRE(stats.has_value());
        CHECK(stats->lossPermille == 0);
        CHECK(stats->droppedFrames == 0);
        MESSAGE("loopback rtt " << stats->rttMs << " ms");
    }
    CHECK(pair.client->linkStats()->bytesOut == 1000);
    CHECK(pair.client->linkStats()->bytesIn == 500);
    CHECK(pair.peer->linkStats()->bytesIn == 1000);
    CHECK(pair.peer->linkStats()->bytesOut == 500);
}

TEST_CASE("net enet: connecting to a closed port times out within the deadline") {
    std::uint16_t port = 0;
    {
        auto server = listening(loopbackServer());
        port = portOf(server->endpoint());
    }
    EnetClientTransport client;
    const auto start = std::chrono::steady_clock::now();
    auto result = client.connect("enet://127.0.0.1:" + std::to_string(port), 400ms);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    REQUIRE_FALSE(result.ok());
    CHECK(result.error().code == ErrorCode::Timeout);
    CHECK(elapsed >= 350ms);
    CHECK(elapsed < 2000ms);
}

TEST_CASE("net enet: bad endpoints and misuse are rejected") {
    EnetClientTransport client;
    CHECK(client.connect("garbage", 100ms).error().code == ErrorCode::InvalidArgument);
    CHECK(client.connect("relay://example.org:1/ABCDEFGH", 100ms).error().code == ErrorCode::InvalidArgument);
    CHECK(client.connect("enet://127.0.0.1:0", 100ms).error().code == ErrorCode::InvalidArgument);

    EnetServerTransport server(loopbackServer());
    CHECK(server.accept(10ms).error().code == ErrorCode::InvalidState);
    CHECK(server.endpoint() == "enet://127.0.0.1:0");
    REQUIRE(server.listen().ok());
    CHECK(server.listen().error().code == ErrorCode::InvalidState);
    CHECK(server.endpoint() != "enet://127.0.0.1:0");

    EnetServerOptions clash = loopbackServer();
    clash.port = portOf(server.endpoint());
    EnetServerTransport second(clash);
    CHECK_FALSE(second.listen().ok());
}

TEST_CASE("net enet: accept times out with nullptr when nobody connects") {
    auto server = listening(loopbackServer());
    const auto start = std::chrono::steady_clock::now();
    auto accepted = server->accept(100ms);
    REQUIRE(accepted.ok());
    CHECK(accepted.value() == nullptr);
    CHECK(std::chrono::steady_clock::now() - start >= 90ms);
}

TEST_CASE("net enet: close from another thread wakes a blocked accept") {
    auto server = listening(loopbackServer());
    std::atomic<bool> returned{false};
    ErrorCode code = ErrorCode::Internal;
    std::thread waiter([&] {
        auto accepted = server->accept(20s);
        if (!accepted) code = accepted.error().code;
        returned = true;
    });
    std::this_thread::sleep_for(50ms);
    server->close();
    CHECK(waitUntil([&] { return returned.load(); }, 3000ms, 1ms));
    waiter.join();
    CHECK(code == ErrorCode::Disconnected);
}

TEST_CASE("net enet: range coder compression shrinks the wire bytes and both ends still agree") {
    auto measure = [](bool compression) {
        EnetServerOptions serverOptions = loopbackServer();
        serverOptions.compression = compression;
        EnetClientOptions clientOptions;
        clientOptions.compression = compression;
        Pair pair = makePair(serverOptions, clientOptions);
        Bytes compressible(100 * 1024);
        for (std::size_t i = 0; i < compressible.size(); ++i) compressible[i] = static_cast<std::byte>("robot state "[i % 12]);
        REQUIRE(pair.peer->send(compressible).ok());
        REQUIRE(pair.client->send(compressible).ok());
        auto atClient = receiveFrame(*pair.client, 10s);
        auto atServer = receiveFrame(*pair.peer, 10s);
        REQUIRE(atClient.has_value());
        REQUIRE(atServer.has_value());
        CHECK(*atClient == compressible);
        CHECK(*atServer == compressible);
        REQUIRE(waitUntil([&] { return pair.server->wireStats().datagramBytesReceived > 0; }, kLong, 2ms));
        // The counters are updated by the pump after each service pass, so wait for them to settle.
        auto last = pair.server->wireStats();
        int stable = 0;
        while (stable < 5) {
            std::this_thread::sleep_for(10ms);
            const auto now = pair.server->wireStats();
            stable = now.datagramBytesSent == last.datagramBytesSent && now.datagramBytesReceived == last.datagramBytesReceived ? stable + 1 : 0;
            last = now;
        }
        return last;
    };
    const auto plain = measure(false);
    const auto packed = measure(true);
    MESSAGE("wire bytes sent " << plain.datagramBytesSent << " plain vs " << packed.datagramBytesSent << " compressed");
    CHECK(plain.datagramBytesSent > 100 * 1024);
    CHECK(packed.datagramBytesSent < plain.datagramBytesSent / 2);
    CHECK(packed.datagramBytesReceived < plain.datagramBytesReceived / 2);
}
