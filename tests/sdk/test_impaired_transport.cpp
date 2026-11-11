#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <set>
#include <vector>

#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/impaired_transport.h"

using namespace wpiemu::sdk;
using namespace wpiemu::sdk::testing;
using namespace std::chrono_literals;

namespace {

std::vector<std::byte> frameOf(int id) { return {static_cast<std::byte>(id)}; }

std::vector<int> drain(proto::ITransport& transport) {
    std::vector<int> ids;
    std::vector<std::byte> frame;
    for (;;) {
        auto got = transport.receive(frame, 0ms);
        REQUIRE(got.ok());
        if (!got.value()) return ids;
        ids.push_back(std::to_integer<int>(frame.at(0)));
    }
}

std::vector<int> range(int n) {
    std::vector<int> ids;
    for (int i = 0; i < n; ++i) ids.push_back(i);
    return ids;
}

// Sends 0..n-1 unreliably through a fresh pair, lets everything arrive and returns what b received in arrival order.
std::vector<int> unreliableRun(const ImpairmentProfile& profile, std::uint64_t seed, int n = 200) {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, profile, seed);
    for (int i = 0; i < n; ++i) REQUIRE(link.a->sendUnreliable(frameOf(i)).ok());
    clock.advance(10s);
    link.pump();
    return drain(*link.b);
}

}  // namespace

TEST_CASE("impaired transport: with no impairment frames pass straight through") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {}, 1);
    CHECK(link.a->send(frameOf(1)).ok());
    CHECK(link.a->sendUnreliable(frameOf(2)).ok());
    CHECK(drain(*link.b) == std::vector<int>{1, 2});
    CHECK(link.a->inFlight() == 0);
    CHECK(link.a->stats().sent == 2);
    CHECK(link.a->stats().delivered == 2);
}

TEST_CASE("impaired transport: latency is honored with a manual clock and never sleeps") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.latency = 40ms}, 1);
    CHECK(link.a->send(frameOf(7)).ok());
    CHECK(link.a->inFlight() == 1);
    CHECK(drain(*link.b).empty());

    clock.advance(39ms);
    link.pump();
    CHECK(drain(*link.b).empty());

    clock.advance(1ms);
    link.pump();
    CHECK(drain(*link.b) == std::vector<int>{7});
    CHECK(link.a->inFlight() == 0);
}

TEST_CASE("impaired transport: receive releases frames that became due without a pump") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.latency = 25ms}, 1);
    CHECK(link.a->sendUnreliable(frameOf(3)).ok());
    clock.advance(25ms);
    CHECK(drain(*link.b) == std::vector<int>{3});
}

TEST_CASE("impaired transport: each direction has its own profile") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.latency = 10ms}, {.latency = 50ms}, 1);
    CHECK(link.a->send(frameOf(1)).ok());
    CHECK(link.b->send(frameOf(2)).ok());
    clock.advance(10ms);
    CHECK(drain(*link.b) == std::vector<int>{1});
    CHECK(drain(*link.a).empty());
    clock.advance(40ms);
    CHECK(drain(*link.a) == std::vector<int>{2});
}

TEST_CASE("impaired transport: jitter stays within its bound") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.latency = 40ms, .jitter = 10ms}, 5);
    for (int i = 0; i < 100; ++i) CHECK(link.a->sendUnreliable(frameOf(i)).ok());
    clock.advance(39ms);
    CHECK(drain(*link.b).empty());
    clock.advance(11ms);
    CHECK(drain(*link.b).size() == 100);
}

TEST_CASE("impaired transport: reliable frames are never lost, even at total loss") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.lossProbability = 1.0, .duplicateProbability = 1.0}, 3);
    for (int i = 0; i < 50; ++i) CHECK(link.a->send(frameOf(i)).ok());
    CHECK(drain(*link.b) == range(50));
    CHECK(link.a->stats().dropped == 0);
    CHECK(link.a->stats().duplicated == 0);
}

TEST_CASE("impaired transport: reliable frames keep their order under jitter and reordering settings") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.latency = 5ms, .jitter = 80ms, .reorderProbability = 1.0}, 9);
    for (int i = 0; i < 100; ++i) {
        CHECK(link.a->send(frameOf(i)).ok());
        clock.advance(1ms);
    }
    clock.advance(1s);
    link.pump();
    CHECK(drain(*link.b) == range(100));
}

TEST_CASE("impaired transport: a delayed reliable frame holds back the ones behind it") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.jitter = 100ms}, 11);
    std::vector<int> seen;
    for (int i = 0; i < 40; ++i) {
        CHECK(link.a->send(frameOf(i)).ok());
        clock.advance(2ms);
        link.pump();
        for (int id : drain(*link.b)) seen.push_back(id);
    }
    clock.advance(1s);
    link.pump();
    for (int id : drain(*link.b)) seen.push_back(id);
    CHECK(seen == range(40));
}

TEST_CASE("impaired transport: unreliable frames are dropped at the configured rate") {
    const std::vector<int> none = unreliableRun({.lossProbability = 1.0}, 7);
    CHECK(none.empty());

    const std::vector<int> all = unreliableRun({.lossProbability = 0.0}, 7);
    CHECK(all == range(200));

    const std::vector<int> some = unreliableRun({.lossProbability = 0.5}, 7, 250);
    CHECK(some.size() > 90);
    CHECK(some.size() < 160);
    const std::set<int> unique(some.begin(), some.end());
    CHECK(unique.size() == some.size());
}

TEST_CASE("impaired transport: loss statistics add up") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.lossProbability = 0.3}, 21);
    for (int i = 0; i < 100; ++i) CHECK(link.a->sendUnreliable(frameOf(i)).ok());
    const ImpairmentStats stats = link.a->stats();
    CHECK(stats.sent == 100);
    CHECK(stats.dropped > 0);
    CHECK(stats.delivered + stats.dropped == 100);
    CHECK(drain(*link.b).size() == stats.delivered);
}

TEST_CASE("impaired transport: unreliable frames are reordered, reliable frames are not") {
    const ImpairmentProfile profile{.reorderProbability = 0.3, .reorderDelay = 30ms};
    const std::vector<int> arrived = unreliableRun(profile, 13);
    CHECK(arrived.size() == 200);
    CHECK_FALSE(std::is_sorted(arrived.begin(), arrived.end()));
    std::vector<int> sorted = arrived;
    std::sort(sorted.begin(), sorted.end());
    CHECK(sorted == range(200));  // reordered, nothing lost or repeated

    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, profile, 13);
    for (int i = 0; i < 200; ++i) CHECK(link.a->send(frameOf(i)).ok());
    clock.advance(10s);
    link.pump();
    CHECK(drain(*link.b) == range(200));
    CHECK(link.a->stats().reordered == 0);
}

TEST_CASE("impaired transport: duplicates only affect unreliable frames") {
    const std::vector<int> arrived = unreliableRun({.duplicateProbability = 1.0}, 2, 20);
    CHECK(arrived.size() == 40);
    std::vector<int> sorted = arrived;
    std::sort(sorted.begin(), sorted.end());
    for (int i = 0; i < 20; ++i) {
        CHECK(sorted[static_cast<std::size_t>(2 * i)] == i);
        CHECK(sorted[static_cast<std::size_t>(2 * i + 1)] == i);
    }
}

TEST_CASE("impaired transport: the same seed gives the same outcome and another seed a different one") {
    const ImpairmentProfile profile{.latency = 20ms,
                                    .jitter = 30ms,
                                    .lossProbability = 0.2,
                                    .reorderProbability = 0.2,
                                    .duplicateProbability = 0.1};
    const std::vector<int> first = unreliableRun(profile, 99);
    CHECK(unreliableRun(profile, 99) == first);
    CHECK(unreliableRun(profile, 100) != first);
}

TEST_CASE("impaired transport: close drops what was in flight and later sends fail") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.latency = 50ms}, 1);
    CHECK(link.a->send(frameOf(1)).ok());
    link.a->close();
    CHECK(link.a->inFlight() == 0);
    CHECK_FALSE(link.a->connected());
    CHECK_FALSE(link.a->send(frameOf(2)).ok());
    CHECK_FALSE(link.a->sendUnreliable(frameOf(2)).ok());
    clock.advance(1s);
    std::vector<std::byte> frame;
    auto got = link.b->receive(frame, 0ms);
    CHECK_FALSE(got.ok());  // the peer sees a disconnect, not the dropped frame
}

TEST_CASE("impaired transport: a lone decorator impairs what it sends to any transport") {
    ManualWallClock clock;
    InMemoryTransportPair raw = makeInMemoryTransportPair();
    ImpairedTransport impaired(std::move(raw.a), clock, {.latency = 15ms}, 4);
    CHECK(impaired.describe() == "impaired(in-memory)");
    CHECK(impaired.send(frameOf(9)).ok());
    CHECK(drain(*raw.b).empty());
    clock.advance(15ms);
    impaired.pump();
    CHECK(drain(*raw.b) == std::vector<int>{9});
}

TEST_CASE("impaired transport: negative or out of range settings are clamped") {
    ManualWallClock clock;
    ImpairedTransportPair link = makeImpairedPair(clock, {.latency = -5ms, .jitter = -1ms, .lossProbability = -1.0}, 1);
    CHECK(link.a->sendUnreliable(frameOf(1)).ok());
    CHECK(drain(*link.b) == std::vector<int>{1});
}
