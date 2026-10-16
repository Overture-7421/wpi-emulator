#include <doctest/doctest.h>

#include <atomic>
#include <random>
#include <thread>

#include "wpiemu/ipc/framed_transport.h"
#include "wpiemu/ipc/framing.h"
#include "wpiemu/ipc/memory_stream.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::sdk;
using namespace std::chrono_literals;

namespace {

std::vector<std::byte> payload(std::size_t n, unsigned seed) {
    std::mt19937 rng(seed);
    std::vector<std::byte> v(n);
    for (auto& b : v) b = static_cast<std::byte>(rng() & 0xFF);
    return v;
}

struct Pair {
    std::unique_ptr<FramedTransport> a;
    std::unique_ptr<FramedTransport> b;
    IByteStream* rawA = nullptr;  // for injecting raw bytes from a's side
};

Pair makePair(std::size_t maxReadChunk = 0, FramedTransportOptions options = {}) {
    MemoryStreamPair streams = makeMemoryStreamPair({maxReadChunk});
    Pair p;
    p.rawA = streams.first.get();
    p.a = std::make_unique<FramedTransport>(std::move(streams.first), options);
    p.b = std::make_unique<FramedTransport>(std::move(streams.second), options);
    return p;
}

// A stream whose writes never complete (peer not reading).
class StuckStream final : public IByteStream {
public:
    Result<std::size_t> read(std::span<std::byte>, std::chrono::milliseconds t) override {
        std::this_thread::sleep_for(t);
        return std::size_t{0};
    }
    Status write(std::span<const std::byte>, std::chrono::milliseconds) override {
        return makeError(ErrorCode::Timeout, "peer is not reading");
    }
    void close() override { closed = true; }
    bool isOpen() const override { return !closed; }
    std::string describe() const override { return "stuck"; }
    bool closed = false;
};

}  // namespace

TEST_CASE("framing: a frame is a u32 little-endian length plus payload") {
    std::vector<std::byte> out;
    const std::vector<std::byte> p = {std::byte{1}, std::byte{2}, std::byte{3}};
    appendFrame(p, out);
    REQUIRE(out.size() == 7);
    CHECK(out[0] == std::byte{3});
    CHECK(out[1] == std::byte{0});
    CHECK(out[3] == std::byte{0});
    CHECK(out[4] == std::byte{1});
}

TEST_CASE("framing: assembler reassembles frames fed one byte at a time") {
    std::vector<std::byte> wire;
    const auto f1 = payload(100, 1);
    const auto f2 = payload(0, 2);
    const auto f3 = payload(1, 3);
    appendFrame(f1, wire);
    appendFrame(f2, wire);
    appendFrame(f3, wire);

    FrameAssembler asmb(1024);
    std::vector<std::vector<std::byte>> got;
    std::vector<std::byte> frame;
    for (std::byte b : wire) {
        REQUIRE(asmb.append(std::span<const std::byte>(&b, 1)).ok());
        for (;;) {
            auto r = asmb.next(frame);
            REQUIRE(r.ok());
            if (!r.value()) break;
            got.push_back(frame);
        }
    }
    REQUIRE(got.size() == 3);
    CHECK(got[0] == f1);
    CHECK(got[1] == f2);
    CHECK(got[2] == f3);
    CHECK_FALSE(asmb.hasPartialFrame());
}

TEST_CASE("framing: several frames in one chunk and a trailing partial frame") {
    std::vector<std::byte> wire;
    for (int i = 0; i < 5; ++i) appendFrame(payload(10 + static_cast<std::size_t>(i), static_cast<unsigned>(i)), wire);
    std::vector<std::byte> tail;
    appendFrame(payload(50, 9), tail);
    wire.insert(wire.end(), tail.begin(), tail.begin() + 20);

    FrameAssembler asmb(1024);
    REQUIRE(asmb.append(wire).ok());
    std::vector<std::byte> frame;
    for (int i = 0; i < 5; ++i) {
        REQUIRE(asmb.next(frame).value());
        CHECK(frame == payload(10 + static_cast<std::size_t>(i), static_cast<unsigned>(i)));
    }
    CHECK_FALSE(asmb.next(frame).value());
    CHECK(asmb.hasPartialFrame());
    REQUIRE(asmb.append(std::span<const std::byte>(tail).subspan(20)).ok());
    REQUIRE(asmb.next(frame).value());
    CHECK(frame == payload(50, 9));
}

TEST_CASE("framing: oversized length is a protocol error, detected as soon as the header is complete") {
    FrameAssembler asmb(100);
    std::vector<std::byte> hdr = {std::byte{101}, std::byte{0}, std::byte{0}};
    CHECK(asmb.append(hdr).ok());  // header incomplete: nothing to judge yet
    const std::byte last{0};
    const Status s = asmb.append(std::span<const std::byte>(&last, 1));
    CHECK_FALSE(s.ok());
    CHECK(s.error().code == ErrorCode::ProtocolError);

    FrameAssembler exact(100);
    std::vector<std::byte> wire;
    appendFrame(payload(100, 1), wire);
    CHECK(exact.append(wire).ok());

    // An oversize header behind a good frame is caught by next().
    FrameAssembler second(100);
    std::vector<std::byte> two;
    appendFrame(payload(5, 1), two);
    const std::vector<std::byte> bad = {std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0x7F}};
    two.insert(two.end(), bad.begin(), bad.end());
    REQUIRE(second.append(two).ok());
    std::vector<std::byte> frame;
    CHECK(second.next(frame).value());
    auto r = second.next(frame);
    CHECK_FALSE(r.ok());
}

TEST_CASE("framing: many frames keep the buffer bounded") {
    FrameAssembler asmb(4096);
    std::vector<std::byte> frame;
    for (int i = 0; i < 20000; ++i) {
        std::vector<std::byte> wire;
        appendFrame(payload(37, static_cast<unsigned>(i)), wire);
        REQUIRE(asmb.append(wire).ok());
        REQUIRE(asmb.next(frame).value());
        CHECK(frame.size() == 37);
    }
    CHECK(asmb.buffered() == 0);
}

TEST_CASE("transport: frames survive 1/3/7-byte partial reads and large frames") {
    for (std::size_t chunk : {std::size_t{1}, std::size_t{3}, std::size_t{7}, std::size_t{0}}) {
        INFO("read chunk " << chunk);
        Pair p = makePair(chunk);
        std::vector<std::vector<std::byte>> sent;
        for (std::size_t n : {std::size_t{0}, std::size_t{1}, std::size_t{4}, std::size_t{255}, std::size_t{70000}}) {
            sent.push_back(payload(n, static_cast<unsigned>(n)));
            if (chunk == 1 && n > 255) sent.back().resize(300);  // keep the 1-byte case quick
        }
        std::atomic<bool> sendFailed{false};
        std::thread writer([&] {
            for (const auto& f : sent)
                if (!p.a->send(f).ok()) sendFailed = true;
        });
        for (const auto& expect : sent) {
            std::vector<std::byte> got;
            auto r = p.b->receive(got, 5000ms);
            REQUIRE(r.ok());
            REQUIRE(r.value());
            CHECK(got == expect);
        }
        writer.join();
        CHECK_FALSE(sendFailed);
    }
}

TEST_CASE("transport: receive times out without losing data and can be retried") {
    Pair p = makePair();
    std::vector<std::byte> got;
    const auto t0 = std::chrono::steady_clock::now();
    auto r = p.b->receive(got, 60ms);
    const auto dt = std::chrono::steady_clock::now() - t0;
    REQUIRE(r.ok());
    CHECK_FALSE(r.value());
    CHECK(dt >= 50ms);
    CHECK(dt < 2000ms);
    CHECK(p.b->connected());

    // Half a frame, a timeout, then the other half: nothing is lost.
    std::vector<std::byte> wire;
    appendFrame(payload(20, 5), wire);
    REQUIRE(p.rawA->write(std::span<const std::byte>(wire).first(10), 100ms).ok());
    CHECK_FALSE(p.b->receive(got, 20ms).value());
    REQUIRE(p.rawA->write(std::span<const std::byte>(wire).subspan(10), 100ms).ok());
    REQUIRE(p.b->receive(got, 1000ms).value());
    CHECK(got == payload(20, 5));

    // Zero timeout = poll.
    CHECK_FALSE(p.b->receive(got, 0ms).value());
}

TEST_CASE("transport: close() from another thread wakes a blocked receive with Disconnected") {
    Pair p = makePair();
    std::atomic<bool> woke{false};
    ErrorCode code = ErrorCode::Internal;
    std::thread reader([&] {
        std::vector<std::byte> got;
        auto r = p.b->receive(got, 30000ms);
        if (!r) code = r.error().code;
        woke = true;
    });
    std::this_thread::sleep_for(50ms);
    CHECK_FALSE(woke);
    const auto t0 = std::chrono::steady_clock::now();
    p.b->close();
    reader.join();
    CHECK(std::chrono::steady_clock::now() - t0 < 2000ms);
    CHECK(woke);
    CHECK(code == ErrorCode::Disconnected);
    CHECK_FALSE(p.b->connected());
    const bool peerNoticed = !p.a->connected() || !p.a->send(payload(1, 1)).ok();
    CHECK(peerNoticed);  // the peer notices
    CHECK(p.b->send(payload(1, 1)).error().code == ErrorCode::Disconnected);
}

TEST_CASE("transport: queued frames are delivered before the disconnect is reported") {
    Pair p = makePair();
    REQUIRE(p.a->send(payload(10, 1)).ok());
    REQUIRE(p.a->send(payload(20, 2)).ok());
    p.a->close();
    std::vector<std::byte> got;
    REQUIRE(p.b->receive(got, 1000ms).value());
    CHECK(got == payload(10, 1));
    REQUIRE(p.b->receive(got, 1000ms).value());
    CHECK(got == payload(20, 2));
    auto r = p.b->receive(got, 1000ms);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::Disconnected);
    // The error is sticky.
    CHECK(p.b->receive(got, 0ms).error().code == ErrorCode::Disconnected);
    CHECK_FALSE(p.b->connected());
}

TEST_CASE("transport: a connection closed in the middle of a frame is reported as such") {
    Pair p = makePair();
    std::vector<std::byte> wire;
    appendFrame(payload(100, 1), wire);
    REQUIRE(p.rawA->write(std::span<const std::byte>(wire).first(50), 100ms).ok());
    p.a->close();
    std::vector<std::byte> got;
    auto r = p.b->receive(got, 1000ms);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::Disconnected);
    CHECK(r.error().message.find("middle of a frame") != std::string::npos);
}

TEST_CASE("transport: an oversized frame from the peer is a protocol error and ends the connection") {
    FramedTransportOptions small;
    small.maxFrameBytes = 64;
    Pair p = makePair(0, small);
    std::vector<std::byte> wire;
    appendFrame(payload(65, 1), wire);  // bypasses a's own send limit
    REQUIRE(p.rawA->write(wire, 100ms).ok());
    std::vector<std::byte> got;
    auto r = p.b->receive(got, 1000ms);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::ProtocolError);
    CHECK_FALSE(p.b->connected());

    const Status s = p.a->send(payload(65, 1));
    CHECK_FALSE(s.ok());
    CHECK(s.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("transport: a send that times out closes the connection (a half-written frame corrupts the stream)") {
    auto stuck = std::make_unique<StuckStream>();
    StuckStream* raw = stuck.get();
    FramedTransportOptions opts;
    opts.sendTimeout = 10ms;
    FramedTransport t(std::move(stuck), opts);
    const Status s = t.send(payload(8, 1));
    REQUIRE_FALSE(s.ok());
    CHECK(s.error().code == ErrorCode::Timeout);
    CHECK_FALSE(t.connected());
    CHECK(raw->closed);
    CHECK(t.send(payload(8, 1)).error().code == ErrorCode::Disconnected);
}

TEST_CASE("transport: concurrent senders never interleave frames") {
    Pair p = makePair(5);
    constexpr int kThreads = 4;
    constexpr int kPerThread = 200;
    std::atomic<bool> sendFailed{false};
    std::vector<std::thread> writers;
    for (int t = 0; t < kThreads; ++t)
        writers.emplace_back([&, t] {
            for (int i = 0; i < kPerThread; ++i) {
                std::vector<std::byte> f(static_cast<std::size_t>(30 + t), static_cast<std::byte>(t));
                if (!p.a->send(f).ok()) sendFailed = true;
            }
        });
    std::vector<int> counts(kThreads, 0);
    for (int i = 0; i < kThreads * kPerThread; ++i) {
        std::vector<std::byte> got;
        REQUIRE(p.b->receive(got, 5000ms).value());
        const int t = static_cast<int>(got[0]);
        REQUIRE(t >= 0);
        REQUIRE(t < kThreads);
        CHECK(got.size() == static_cast<std::size_t>(30 + t));
        for (std::byte b : got) REQUIRE(static_cast<int>(b) == t);
        ++counts[static_cast<std::size_t>(t)];
    }
    for (auto& w : writers) w.join();
    CHECK_FALSE(sendFailed);
    for (int c : counts) CHECK(c == kPerThread);
}

TEST_CASE("memory stream: read limit, timeout and close semantics") {
    MemoryStreamPair s = makeMemoryStreamPair({4});
    std::vector<std::byte> data = payload(10, 1);
    REQUIRE(s.first->write(data, 10ms).ok());
    std::vector<std::byte> buf(100);
    CHECK(s.second->read(buf, 10ms).value() == 4);
    CHECK(s.second->read(buf, 10ms).value() == 4);
    CHECK(s.second->read(buf, 10ms).value() == 2);
    CHECK(s.second->read(buf, 10ms).value() == 0);
    s.first->close();
    CHECK(s.second->read(buf, 10ms).error().code == ErrorCode::Disconnected);
    CHECK(s.second->write(data, 10ms).error().code == ErrorCode::Disconnected);
    CHECK_FALSE(s.first->isOpen());
}
