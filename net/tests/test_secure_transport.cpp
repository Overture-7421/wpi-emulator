#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "wpiemu/net/invite.h"
#include "wpiemu/net/secure_transport.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/impaired_transport.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu;
using namespace wpiemu::net;
using namespace std::chrono_literals;
using sdk::proto::Delivery;
using sdk::proto::ITransport;
using Bytes = std::vector<std::byte>;
using Connected = sdk::Result<std::unique_ptr<ITransport>>;

namespace {

Bytes bytesOf(const std::string& text) {
    Bytes out(text.size());
    std::memcpy(out.data(), text.data(), text.size());
    return out;
}

std::string textOf(const Bytes& bytes) {
    return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

std::string hex(std::span<const std::byte> bytes) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    for (const std::byte b : bytes) {
        out.push_back(digits[std::to_integer<unsigned>(b) >> 4]);
        out.push_back(digits[std::to_integer<unsigned>(b) & 15]);
    }
    return out;
}

bool contains(std::span<const std::byte> haystack, const std::string& needle) {
    return std::search(haystack.begin(), haystack.end(), reinterpret_cast<const std::byte*>(needle.data()),
                       reinterpret_cast<const std::byte*>(needle.data()) + needle.size()) != haystack.end();
}

InviteSecret inviteFrom(std::string_view text) {
    auto secret = parseInvite(text);
    REQUIRE(secret.ok());
    return std::move(secret).value();
}

const InviteSecret& testSecret() {
    static const InviteSecret secret = inviteFrom("ABCDE-FGHJK-MNPQR-STVWX");
    return secret;
}

const InviteSecret& otherSecret() {
    static const InviteSecret secret = inviteFrom("ABCDE-FGHJK-MNPQR-STVW0");
    return secret;
}

// Records every frame sent through it, can rewrite or multiply what goes out, and lets a test see the delivery class.
struct TapState {
    std::mutex mutex;
    std::vector<Bytes> sent;
    std::vector<Delivery> deliveries;
    std::function<std::vector<Bytes>(const Bytes&)> rewrite;  // empty = pass through
};

class TapTransport final : public ITransport {
public:
    TapTransport(std::unique_ptr<ITransport> inner, std::shared_ptr<TapState> state)
        : inner_(std::move(inner)), state_(std::move(state)) {}

    sdk::Status send(std::span<const std::byte> frame) override { return forward(frame, Delivery::Reliable); }
    sdk::Status sendUnreliable(std::span<const std::byte> frame) override {
        return forward(frame, Delivery::Unreliable);
    }
    sdk::Result<bool> receive(Bytes& frame, std::chrono::milliseconds timeout) override {
        return inner_->receive(frame, timeout);
    }
    void close() override { inner_->close(); }
    bool connected() const override { return inner_->connected(); }
    std::string describe() const override { return "tap(" + inner_->describe() + ")"; }

private:
    sdk::Status forward(std::span<const std::byte> frame, Delivery delivery) {
        Bytes copy(frame.begin(), frame.end());
        std::vector<Bytes> out;
        {
            std::lock_guard lock(state_->mutex);
            state_->sent.push_back(copy);
            state_->deliveries.push_back(delivery);
            if (state_->rewrite)
                out = state_->rewrite(copy);
            else
                out.push_back(copy);
        }
        sdk::Status last = sdk::okStatus();
        for (const Bytes& f : out) {
            last = delivery == Delivery::Unreliable ? inner_->sendUnreliable(f) : inner_->send(f);
            if (!last) return last;
        }
        return last;
    }

    std::unique_ptr<ITransport> inner_;
    std::shared_ptr<TapState> state_;
};

struct SecurePair {
    std::unique_ptr<ITransport> client;
    std::unique_ptr<ITransport> host;
    std::optional<sdk::Error> clientError;
    std::optional<sdk::Error> hostError;
    bool ok() const { return client && host; }
};

SecurePair handshake(std::unique_ptr<ITransport> clientInner, std::unique_ptr<ITransport> hostInner,
                     const InviteSecret& clientSecret, const InviteSecret& hostSecret,
                     std::chrono::milliseconds timeout = 5000ms,
                     const SecureTransport::RandomFill* clientFill = nullptr,
                     const SecureTransport::RandomFill* hostFill = nullptr) {
    SecurePair out;
    Connected hostResult = sdk::makeError(sdk::ErrorCode::Internal, "not run");
    std::thread host([&] {
        hostResult = hostFill ? SecureTransport::acceptAsHostWith(std::move(hostInner), hostSecret, timeout, *hostFill)
                              : SecureTransport::acceptAsHost(std::move(hostInner), hostSecret, timeout);
    });
    Connected clientResult =
        clientFill ? SecureTransport::connectAsClientWith(std::move(clientInner), clientSecret, timeout, *clientFill)
                   : SecureTransport::connectAsClient(std::move(clientInner), clientSecret, timeout);
    host.join();
    if (clientResult)
        out.client = std::move(clientResult).value();
    else
        out.clientError = clientResult.error();
    if (hostResult)
        out.host = std::move(hostResult).value();
    else
        out.hostError = hostResult.error();
    return out;
}

SecurePair plainHandshake() {
    auto raw = sdk::testing::makeInMemoryTransportPair();
    return handshake(std::move(raw.a), std::move(raw.b), testSecret(), testSecret());
}

SecureTransport& secure(const std::unique_ptr<ITransport>& t) {
    auto* s = dynamic_cast<SecureTransport*>(t.get());
    REQUIRE(s != nullptr);
    return *s;
}

bool receiveOne(ITransport& t, Bytes& frame, std::chrono::milliseconds timeout = 1000ms) {
    auto got = t.receive(frame, timeout);
    REQUIRE(got.ok());
    return got.value();
}

// Pair with a tap on the client's sending side; `rawClient` talks to the host's inner transport directly.
struct TappedPair {
    SecurePair pair;
    std::shared_ptr<TapState> tap = std::make_shared<TapState>();
    ITransport* rawClient = nullptr;
};

TappedPair tappedHandshake() {
    TappedPair t;
    auto raw = sdk::testing::makeInMemoryTransportPair();
    t.rawClient = raw.a.get();
    t.pair = handshake(std::make_unique<TapTransport>(std::move(raw.a), t.tap), std::move(raw.b), testSecret(),
                       testSecret());
    REQUIRE(t.pair.ok());
    return t;
}

Bytes helloFrame(std::uint8_t version, std::size_t keyFill) {
    Bytes frame{std::byte{'W'}, std::byte{'P'}, std::byte{'S'}, std::byte{'C'}, std::byte{version}};
    frame.insert(frame.end(), 32, static_cast<std::byte>(keyFill));
    return frame;
}

}  // namespace

TEST_CASE("secure transport: handshake succeeds and carries data both ways") {
    SecurePair p = plainHandshake();
    REQUIRE(p.ok());
    CHECK(p.client->connected());
    CHECK(p.host->connected());
    CHECK(p.client->describe() == "secure(in-memory)");

    REQUIRE(p.client->send(bytesOf("from client")).ok());
    REQUIRE(p.host->send(bytesOf("from host")).ok());
    Bytes frame;
    REQUIRE(receiveOne(*p.host, frame));
    CHECK(textOf(frame) == "from client");
    REQUIRE(receiveOne(*p.client, frame));
    CHECK(textOf(frame) == "from host");
    CHECK(secure(p.client).stats().sent == 1);
    CHECK(secure(p.host).stats().received == 1);
    CHECK_FALSE(receiveOne(*p.host, frame, 20ms));
}

TEST_CASE("secure transport: handshake and data over an impaired in-memory pair") {
    sdk::testing::ManualWallClock clock;
    auto link = sdk::testing::makeImpairedPair(clock, {}, 7);
    SecurePair p = handshake(std::move(link.a), std::move(link.b), testSecret(), testSecret());
    REQUIRE(p.ok());
    REQUIRE(p.client->send(bytesOf("one")).ok());
    REQUIRE(p.client->sendUnreliable(bytesOf("two")).ok());
    Bytes frame;
    REQUIRE(receiveOne(*p.host, frame));
    CHECK(textOf(frame) == "one");
    REQUIRE(receiveOne(*p.host, frame));
    CHECK(textOf(frame) == "two");
}

TEST_CASE("secure transport: a wrong secret fails on both sides and delivers no data") {
    auto raw = sdk::testing::makeInMemoryTransportPair();
    SecurePair p = handshake(std::move(raw.a), std::move(raw.b), testSecret(), otherSecret(), 2000ms);
    CHECK_FALSE(p.client);
    CHECK_FALSE(p.host);
    REQUIRE(p.clientError.has_value());
    REQUIRE(p.hostError.has_value());
    CHECK(p.clientError->code == sdk::ErrorCode::ProtocolError);
    CHECK(p.clientError->message.find("invite") != std::string::npos);
}

TEST_CASE("secure transport: malformed first frames are rejected by the host") {
    struct Case {
        const char* name;
        Bytes frame;
        sdk::ErrorCode code;
    };
    Bytes wrongMagic = helloFrame(1, 0x42);
    wrongMagic[0] = std::byte{'X'};
    Bytes truncated = helloFrame(1, 0x42);
    truncated.pop_back();
    Bytes oversize = helloFrame(1, 0x42);
    oversize.push_back(std::byte{0});
    const std::vector<Case> cases = {
        {"truncated", truncated, sdk::ErrorCode::ProtocolError},
        {"oversize", oversize, sdk::ErrorCode::ProtocolError},
        {"empty", {}, sdk::ErrorCode::ProtocolError},
        {"two bytes", {std::byte{'W'}, std::byte{'P'}}, sdk::ErrorCode::ProtocolError},
        {"wrong magic", wrongMagic, sdk::ErrorCode::ProtocolError},
        {"wrong version", helloFrame(2, 0x42), sdk::ErrorCode::VersionMismatch},
        {"version zero", helloFrame(0, 0x42), sdk::ErrorCode::VersionMismatch},
        {"all zero key", helloFrame(1, 0x00), sdk::ErrorCode::ProtocolError},
        {"garbage", bytesOf(std::string(37, 'z')), sdk::ErrorCode::ProtocolError},
    };
    for (const Case& c : cases) {
        CAPTURE(c.name);
        auto raw = sdk::testing::makeInMemoryTransportPair();
        REQUIRE(raw.a->send(c.frame).ok());
        Connected result = SecureTransport::acceptAsHost(std::move(raw.b), testSecret(), 1000ms);
        REQUIRE_FALSE(result.ok());
        CHECK(result.error().code == c.code);
        Bytes frame;
        auto after = raw.a->receive(frame, 50ms);  // the host closed its end: nothing was answered
        CHECK_FALSE(after.ok());
    }
}

TEST_CASE("secure transport: malformed replies are rejected by the client") {
    struct Case {
        const char* name;
        std::function<Bytes()> make;
        sdk::ErrorCode code;
    };
    const auto goodSized = [](std::uint8_t version) {
        Bytes frame = helloFrame(version, 0x42);
        frame.insert(frame.end(), 40, std::byte{0x11});
        return frame;
    };
    const std::vector<Case> cases = {
        {"truncated", [&] { Bytes f = goodSized(1); f.pop_back(); return f; }, sdk::ErrorCode::ProtocolError},
        {"oversize", [&] { Bytes f = goodSized(1); f.push_back(std::byte{0}); return f; }, sdk::ErrorCode::ProtocolError},
        {"wrong version", [&] { return goodSized(9); }, sdk::ErrorCode::VersionMismatch},
        {"wrong magic", [&] { Bytes f = goodSized(1); f[1] = std::byte{'Q'}; return f; }, sdk::ErrorCode::ProtocolError},
        {"bad confirmation", [&] { return goodSized(1); }, sdk::ErrorCode::ProtocolError},
        {"all zero key", [&] { Bytes f = goodSized(1); std::fill(f.begin() + 5, f.begin() + 37, std::byte{0}); return f; },
         sdk::ErrorCode::ProtocolError},
    };
    for (const Case& c : cases) {
        CAPTURE(c.name);
        auto raw = sdk::testing::makeInMemoryTransportPair();
        Connected result = sdk::makeError(sdk::ErrorCode::Internal, "not run");
        std::thread client([&] { result = SecureTransport::connectAsClient(std::move(raw.a), testSecret(), 2000ms); });
        Bytes hello;
        REQUIRE(receiveOne(*raw.b, hello, 2000ms));
        CHECK(hello.size() == SecureTransport::kHelloBytes);
        REQUIRE(raw.b->send(c.make()).ok());
        client.join();
        REQUIRE_FALSE(result.ok());
        CHECK(result.error().code == c.code);
    }
}

TEST_CASE("secure transport: a silent peer times out the handshake") {
    {
        auto raw = sdk::testing::makeInMemoryTransportPair();
        const auto start = std::chrono::steady_clock::now();
        Connected host = SecureTransport::acceptAsHost(std::move(raw.b), testSecret(), 80ms);
        REQUIRE_FALSE(host.ok());
        CHECK(host.error().code == sdk::ErrorCode::Timeout);
        CHECK(std::chrono::steady_clock::now() - start < 3s);
    }
    {
        auto raw = sdk::testing::makeInMemoryTransportPair();
        const auto start = std::chrono::steady_clock::now();
        Connected client = SecureTransport::connectAsClient(std::move(raw.a), testSecret(), 80ms);
        REQUIRE_FALSE(client.ok());
        CHECK(client.error().code == sdk::ErrorCode::Timeout);
        CHECK(std::chrono::steady_clock::now() - start < 3s);
    }
}

TEST_CASE("secure transport: a host that goes silent after the client hello times out the host") {
    auto raw = sdk::testing::makeInMemoryTransportPair();
    Connected host = sdk::makeError(sdk::ErrorCode::Internal, "not run");
    std::thread t([&] { host = SecureTransport::acceptAsHost(std::move(raw.b), testSecret(), 150ms); });
    REQUIRE(raw.a->send(helloFrame(1, 0x42)).ok());  // valid hello, but the "client" never confirms
    t.join();
    REQUIRE_FALSE(host.ok());
    CHECK(host.error().code == sdk::ErrorCode::Timeout);
}

TEST_CASE("secure transport: a tampered ciphertext or tag is dropped and counted") {
    TappedPair t = tappedHandshake();
    int call = 0;
    t.tap->rewrite = [&](const Bytes& frame) {
        Bytes f = frame;
        ++call;
        if (call == 1) f[8] ^= std::byte{1};          // ciphertext
        if (call == 2) f.back() ^= std::byte{0x80};  // tag
        if (call == 3) f[0] ^= std::byte{4};          // counter (changes the nonce)
        return std::vector<Bytes>{f};
    };
    for (int i = 0; i < 4; ++i) REQUIRE(t.pair.client->send(bytesOf("payload " + std::to_string(i))).ok());
    Bytes frame;
    REQUIRE(receiveOne(*t.pair.host, frame));
    CHECK(textOf(frame) == "payload 3");
    CHECK_FALSE(receiveOne(*t.pair.host, frame, 20ms));
    CHECK(secure(t.pair.host).stats().droppedBadTag == 3);
    CHECK(secure(t.pair.host).stats().received == 1);
    CHECK(t.pair.host->connected());
    REQUIRE(t.pair.client->send(bytesOf("still works")).ok());
    REQUIRE(receiveOne(*t.pair.host, frame));
    CHECK(textOf(frame) == "still works");
}

TEST_CASE("secure transport: replayed and duplicated records are dropped") {
    TappedPair t = tappedHandshake();
    REQUIRE(t.pair.client->send(bytesOf("first")).ok());
    REQUIRE(t.pair.client->sendUnreliable(bytesOf("second")).ok());
    Bytes frame;
    REQUIRE(receiveOne(*t.pair.host, frame));
    REQUIRE(receiveOne(*t.pair.host, frame));
    Bytes firstRecord;
    {
        std::lock_guard lock(t.tap->mutex);
        firstRecord = t.tap->sent[t.tap->sent.size() - 2];
    }
    REQUIRE(t.rawClient->send(firstRecord).ok());  // replay of the first record
    REQUIRE(t.rawClient->send(firstRecord).ok());
    CHECK_FALSE(receiveOne(*t.pair.host, frame, 30ms));
    CHECK(secure(t.pair.host).stats().droppedReplay == 2);

    t.tap->rewrite = [](const Bytes& f) { return std::vector<Bytes>{f, f, f}; };  // a link that triplicates
    REQUIRE(t.pair.client->sendUnreliable(bytesOf("third")).ok());
    REQUIRE(receiveOne(*t.pair.host, frame));
    CHECK(textOf(frame) == "third");
    CHECK_FALSE(receiveOne(*t.pair.host, frame, 30ms));
    CHECK(secure(t.pair.host).stats().droppedReplay == 4);
    CHECK(secure(t.pair.host).stats().received == 3);
}

TEST_CASE("secure transport: short and foreign frames are dropped without a disconnect") {
    TappedPair t = tappedHandshake();
    for (const std::size_t size : {0u, 1u, 7u, 23u}) REQUIRE(t.rawClient->send(Bytes(size, std::byte{0x5a})).ok());
    REQUIRE(t.rawClient->send(Bytes(24, std::byte{0})).ok());  // record-sized but not authentic (counter 0)
    REQUIRE(t.rawClient->send(Bytes(200, std::byte{0x77})).ok());
    Bytes frame;
    CHECK_FALSE(receiveOne(*t.pair.host, frame, 30ms));
    const SecureTransportStats s = secure(t.pair.host).stats();
    CHECK(s.droppedShort == 4);
    CHECK(s.droppedBadTag + s.droppedReplay == 2);
    CHECK(t.pair.host->connected());
    REQUIRE(t.pair.client->send(bytesOf("fine")).ok());
    REQUIRE(receiveOne(*t.pair.host, frame));
    CHECK(textOf(frame) == "fine");
}

TEST_CASE("secure transport: the replay window accepts late records and rejects ones that are too old") {
    TappedPair t = tappedHandshake();
    constexpr int kTotal = 2100;
    for (int i = 0; i < kTotal; ++i) REQUIRE(t.pair.client->sendUnreliable(bytesOf(std::to_string(i))).ok());
    Bytes frame;
    for (int i = 0; i < kTotal; ++i) REQUIRE(receiveOne(*t.pair.host, frame));
    CHECK(textOf(frame) == std::to_string(kTotal - 1));
    Bytes oldest;
    Bytes inWindow;
    {
        std::lock_guard lock(t.tap->mutex);
        oldest = t.tap->sent[t.tap->sent.size() - kTotal];                                 // counter 1
        inWindow = t.tap->sent[t.tap->sent.size() - SecureTransport::kReplayWindow + 5];  // recent, already seen
    }
    REQUIRE(t.rawClient->send(oldest).ok());
    REQUIRE(t.rawClient->send(inWindow).ok());
    CHECK_FALSE(receiveOne(*t.pair.host, frame, 30ms));
    CHECK(secure(t.pair.host).stats().droppedReplay == 2);
    CHECK(secure(t.pair.host).stats().received == kTotal);
}

TEST_CASE("secure transport: records arriving late inside the window are accepted once") {
    TappedPair t = tappedHandshake();
    std::vector<Bytes> held;
    t.tap->rewrite = [&](const Bytes& f) {
        held.push_back(f);
        return std::vector<Bytes>{};  // hold everything back
    };
    for (int i = 0; i < 50; ++i) REQUIRE(t.pair.client->sendUnreliable(bytesOf("m" + std::to_string(i))).ok());
    Bytes frame;
    for (int i = 49; i >= 0; --i) {  // fully reversed
        REQUIRE(t.rawClient->send(held[static_cast<std::size_t>(i)]).ok());
        REQUIRE(receiveOne(*t.pair.host, frame));
        CHECK(textOf(frame) == "m" + std::to_string(i));
    }
    REQUIRE(t.rawClient->send(held[10]).ok());
    CHECK_FALSE(receiveOne(*t.pair.host, frame, 30ms));
    CHECK(secure(t.pair.host).stats().droppedReplay == 1);
}

TEST_CASE("secure transport: reordered, duplicated and dropped unreliable records over an impaired link") {
    sdk::testing::ManualWallClock clock;
    sdk::testing::ImpairmentProfile profile;
    profile.lossProbability = 0.2;
    profile.reorderProbability = 0.3;
    profile.duplicateProbability = 0.2;
    auto link = sdk::testing::makeImpairedPair(clock, sdk::testing::ImpairmentProfile{}, profile, 42);
    // The host-to-client direction is the impaired one.
    sdk::testing::ImpairedTransport* hostSide = link.b.get();
    SecurePair p = handshake(std::move(link.a), std::move(link.b), testSecret(), testSecret());
    REQUIRE(p.ok());

    const sdk::testing::ImpairmentStats before = hostSide->stats();

    constexpr int kCount = 200;
    for (int i = 0; i < kCount; ++i) REQUIRE(p.host->sendUnreliable(bytesOf(std::to_string(i))).ok());
    clock.advance(1s);
    hostSide->pump();
    sdk::testing::ImpairmentStats net = hostSide->stats();
    net.delivered -= before.delivered;
    REQUIRE(net.reordered > 0);
    REQUIRE(net.dropped > 0);
    REQUIRE(net.duplicated > 0);

    std::vector<int> got;
    Bytes frame;
    while (receiveOne(*p.client, frame, 20ms)) got.push_back(std::stoi(textOf(frame)));
    std::vector<int> sorted = got;
    std::sort(sorted.begin(), sorted.end());
    CHECK(std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end());  // no record delivered twice
    CHECK_FALSE(std::is_sorted(got.begin(), got.end()));                       // and the reordering was real
    const SecureTransportStats s = secure(p.client).stats();
    CHECK(s.received == got.size());
    CHECK(s.droppedBadTag == 0);
    CHECK(s.droppedReplay == net.duplicated);
    CHECK(s.received + s.droppedReplay == net.delivered);
    CHECK(p.client->connected());
}

TEST_CASE("secure transport: the delivery class survives end to end") {
    sdk::testing::ManualWallClock clock;
    sdk::testing::ImpairmentProfile lossy;
    lossy.lossProbability = 1.0;  // every unreliable frame is lost, reliable ones are untouched
    auto link = sdk::testing::makeImpairedPair(clock, sdk::testing::ImpairmentProfile{}, lossy, 3);
    SecurePair p = handshake(std::move(link.a), std::move(link.b), testSecret(), testSecret());
    REQUIRE(p.ok());
    for (int i = 0; i < 10; ++i) {
        REQUIRE(p.host->sendUnreliable(bytesOf("lost")).ok());
        REQUIRE(p.host->send(bytesOf("kept " + std::to_string(i))).ok());
    }
    Bytes frame;
    for (int i = 0; i < 10; ++i) {
        REQUIRE(receiveOne(*p.client, frame));
        CHECK(textOf(frame) == "kept " + std::to_string(i));
    }
    CHECK_FALSE(receiveOne(*p.client, frame, 20ms));

    TappedPair t = tappedHandshake();
    REQUIRE(t.pair.client->send(bytesOf("r")).ok());
    REQUIRE(t.pair.client->sendUnreliable(bytesOf("u")).ok());
    std::lock_guard lock(t.tap->mutex);
    const std::size_t n = t.tap->deliveries.size();
    CHECK(t.tap->deliveries[n - 2] == Delivery::Reliable);
    CHECK(t.tap->deliveries[n - 1] == Delivery::Unreliable);
}

TEST_CASE("secure transport: reliable ordering is preserved over a jittery link") {
    sdk::testing::ManualWallClock clock;
    sdk::testing::ImpairmentProfile profile;
    profile.latency = 20ms;
    profile.jitter = 15ms;
    profile.lossProbability = 0.3;
    profile.reorderProbability = 0.3;
    profile.duplicateProbability = 0.3;
    auto link = sdk::testing::makeImpairedPair(clock, profile, 11);
    sdk::testing::ImpairedTransport* clientSide = link.a.get();
    std::atomic<bool> advance{true};
    std::thread ticker([&] {  // the handshake needs the manual clock to move because the link has latency
        while (advance) {
            clock.advance(5ms);
            clientSide->pump();
            std::this_thread::sleep_for(1ms);
        }
    });
    SecurePair p = handshake(std::move(link.a), std::move(link.b), testSecret(), testSecret());
    REQUIRE(p.ok());
    for (int i = 0; i < 100; ++i) REQUIRE(p.client->send(bytesOf(std::to_string(i))).ok());
    Bytes frame;
    for (int i = 0; i < 100; ++i) {
        REQUIRE(receiveOne(*p.host, frame, 5000ms));
        CHECK(textOf(frame) == std::to_string(i));
    }
    advance = false;
    ticker.join();
    CHECK(secure(p.host).stats().droppedBadTag == 0);
}

TEST_CASE("secure transport: large and empty payloads") {
    SecurePair p = plainHandshake();
    REQUIRE(p.ok());
    Bytes large(128 * 1024);
    for (std::size_t i = 0; i < large.size(); ++i) large[i] = static_cast<std::byte>((i * 31 + 7) & 0xff);
    REQUIRE(p.client->send(large).ok());
    REQUIRE(p.client->send(std::span<const std::byte>{}).ok());
    Bytes frame;
    REQUIRE(receiveOne(*p.host, frame));
    CHECK(frame == large);
    REQUIRE(receiveOne(*p.host, frame));
    CHECK(frame.empty());
    CHECK(secure(p.host).stats().received == 2);
}

TEST_CASE("secure transport: send and receive after close fail, and close reaches the peer") {
    SecurePair p = plainHandshake();
    REQUIRE(p.ok());
    p.client->close();
    CHECK_FALSE(p.client->connected());
    CHECK_FALSE(p.host->connected());
    CHECK(p.client->send(bytesOf("x")).ok() == false);
    CHECK(p.client->sendUnreliable(bytesOf("x")).ok() == false);
    Bytes frame;
    auto mine = p.client->receive(frame, 10ms);
    REQUIRE_FALSE(mine.ok());
    CHECK(mine.error().code == sdk::ErrorCode::Disconnected);
    auto theirs = p.host->receive(frame, 200ms);
    REQUIRE_FALSE(theirs.ok());
    CHECK(theirs.error().code == sdk::ErrorCode::Disconnected);
    CHECK(p.host->send(bytesOf("x")).ok() == false);
}

TEST_CASE("secure transport: an exhausted send counter closes instead of wrapping") {
    SecurePair p = plainHandshake();
    REQUIRE(p.ok());
    secure(p.client).setNextSendCounterForTest(std::numeric_limits<std::uint64_t>::max());
    REQUIRE(p.client->send(bytesOf("last")).ok());
    Bytes frame;
    REQUIRE(receiveOne(*p.host, frame));
    CHECK(textOf(frame) == "last");
    CHECK_FALSE(p.client->send(bytesOf("one too many")).ok());
    CHECK_FALSE(p.client->connected());
}

TEST_CASE("secure transport: concurrent senders and receivers in both directions") {
    SecurePair p = plainHandshake();
    REQUIRE(p.ok());
    constexpr int kCount = 2000;
    const auto pump = [&](ITransport& from, ITransport& to, const char* tag, std::atomic<int>& bad) {
        std::thread sender([&] {
            for (int i = 0; i < kCount; ++i) {
                const Bytes payload = bytesOf(std::string(tag) + std::to_string(i));
                if (i % 2 == 0) {
                    if (!from.send(payload).ok()) bad++;
                } else if (!from.sendUnreliable(payload).ok()) {
                    bad++;
                }
            }
        });
        std::thread receiver([&] {
            Bytes frame;
            for (int i = 0; i < kCount; ++i) {
                auto got = to.receive(frame, 5000ms);
                if (!got || !got.value() || textOf(frame) != std::string(tag) + std::to_string(i)) bad++;
            }
        });
        sender.join();
        receiver.join();
    };
    std::atomic<int> badA{0};
    std::atomic<int> badB{0};
    std::thread forward([&] { pump(*p.client, *p.host, "c", badA); });
    pump(*p.host, *p.client, "h", badB);
    forward.join();
    CHECK(badA == 0);
    CHECK(badB == 0);
}

TEST_CASE("secure transport: no plaintext ever crosses the inner transport") {
    const std::string marker = "TOP-SECRET-MARKER-0123456789";
    auto raw = sdk::testing::makeInMemoryTransportPair();
    auto clientTap = std::make_shared<TapState>();
    auto hostTap = std::make_shared<TapState>();
    SecurePair p = handshake(std::make_unique<TapTransport>(std::move(raw.a), clientTap),
                             std::make_unique<TapTransport>(std::move(raw.b), hostTap), testSecret(), testSecret());
    REQUIRE(p.ok());
    REQUIRE(p.client->send(bytesOf(marker)).ok());
    REQUIRE(p.client->sendUnreliable(bytesOf("u:" + marker)).ok());
    REQUIRE(p.host->send(bytesOf(marker + marker)).ok());
    REQUIRE(p.host->sendUnreliable(bytesOf(marker + "!")).ok());

    Bytes frame;
    REQUIRE(receiveOne(*p.host, frame));
    CHECK(textOf(frame) == marker);
    REQUIRE(receiveOne(*p.host, frame));
    CHECK(textOf(frame) == "u:" + marker);
    REQUIRE(receiveOne(*p.client, frame));
    CHECK(textOf(frame) == marker + marker);
    REQUIRE(receiveOne(*p.client, frame));
    CHECK(textOf(frame) == marker + "!");

    std::size_t crossed = 0;
    for (const auto& tap : {clientTap, hostTap}) {
        std::lock_guard lock(tap->mutex);
        for (const Bytes& f : tap->sent) {
            ++crossed;
            CHECK_FALSE(contains(f, "MARKER"));
            CHECK_FALSE(contains(f, "TOP-SECRET"));
            CHECK_FALSE(contains(f, "0123456789"));
        }
    }
    CHECK(crossed == 7);  // hello, reply, confirmation and four records
    CHECK(contains(bytesOf("xx" + marker), marker));  // the search itself works
}

TEST_CASE("secure transport: known-answer handshake and record framing") {
    const auto fixed = [](std::uint8_t value) {
        return SecureTransport::RandomFill([value](std::span<std::byte> out) {
            std::fill(out.begin(), out.end(), static_cast<std::byte>(value));
            return sdk::okStatus();
        });
    };
    const SecureTransport::RandomFill clientFill = fixed(0x11);
    const SecureTransport::RandomFill hostFill = fixed(0x22);
    auto raw = sdk::testing::makeInMemoryTransportPair();
    auto clientTap = std::make_shared<TapState>();
    auto hostTap = std::make_shared<TapState>();
    SecurePair p = handshake(std::make_unique<TapTransport>(std::move(raw.a), clientTap),
                             std::make_unique<TapTransport>(std::move(raw.b), hostTap), testSecret(), testSecret(), 5000ms,
                             &clientFill, &hostFill);
    REQUIRE(p.ok());
    REQUIRE(p.client->send(bytesOf("hello host")).ok());
    REQUIRE(p.host->sendUnreliable(bytesOf("hello client")).ok());

    REQUIRE(clientTap->sent.size() == 3);
    REQUIRE(hostTap->sent.size() == 2);
    // Cross-checked against an independent implementation (python: hashlib blake2b, cryptography X25519, HChaCha20 +
    // ChaCha20-Poly1305), ephemeral secrets 32 x 0x11 (client) and 32 x 0x22 (host), invite ABCDE-FGHJK-MNPQR-STVWX.
    CHECK(hex(clientTap->sent[0]) ==
          "57505343017b4e909bbe7ffe44c465a220037d608ee35897d31ef972f07f74892cb0f73f13");
    CHECK(hex(hostTap->sent[0]) ==
          "57505343010faa684ed28867b97f4a6a2dee5df8ce974e76b7018e3f22a1c4cf2678570f200000000000000000411184636b035995d1ed2e"
          "067498a9e856b328332d6fef0ca8252e891d1c5eb6");
    CHECK(hex(clientTap->sent[1]) ==
          "00000000000000006c70e4f1078c72e58726ec4fc549d9e754ddd03afa065d18bb48187c960add84");
    CHECK(hex(clientTap->sent[2]) == "01000000000000003e5c505aff19fdeeb86539ede17c7c6bb1c842dd17a41e687efd");
    CHECK(hex(hostTap->sent[1]) == "01000000000000001f1bf954090909d6d8e34d51b1312cb07eac48a687ae72786289edcd");
}
