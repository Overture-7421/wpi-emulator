#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "wpiemu/core/match/client_timing.h"
#include "wpiemu/core/match/clock_sync.h"

using namespace wpiemu::core::match;

namespace {

// A synthetic path between a client and a host: the host's sim time is `hostRate * t + hostOffset` for client time t, the
// request takes `up` and the reply `down` microseconds (plus random extra delay), and pings go out every `pingEveryUs`.
struct Path {
    std::int64_t upUs = 20'000;
    std::int64_t downUs = 20'000;
    std::int64_t extraUs = 0;        // uniform [0, extra] added to each direction
    double hostRate = 1.0;
    std::int64_t hostOffsetUs = 7'000'000;
    std::mt19937_64 rng{42};

    std::int64_t hostSimAt(std::int64_t clientUs) const {
        return static_cast<std::int64_t>(std::llround(hostRate * static_cast<double>(clientUs))) + hostOffsetUs;
    }
    std::int64_t jitter() { return extraUs == 0 ? 0 : static_cast<std::int64_t>(rng() % static_cast<std::uint64_t>(extraUs + 1)); }

    void ping(ClockSync& sync, std::int64_t sendUs) {
        const std::int64_t arrive = sendUs + upUs + jitter();
        const std::int64_t recv = arrive + downUs + jitter();
        sync.addPing(sendUs, recv, hostSimAt(arrive));
    }
    // Error of the estimate at client time t against the real host clock, microseconds.
    std::int64_t error(const ClockSync& sync, std::int64_t t) const { return sync.serverSimAt(t) - hostSimAt(t); }
};

}  // namespace

TEST_CASE("clock sync: a symmetric path converges to the host clock within a millisecond") {
    ClockSync sync;
    Path path;
    path.extraUs = 6000;
    std::int64_t t = 1'000'000;
    for (int i = 0; i < 12; ++i, t += 120'000) path.ping(sync, t);
    CHECK(sync.synced());
    CHECK(sync.pingCount() == 12);
    CHECK(std::llabs(path.error(sync, t)) < 1500);
    CHECK(sync.minRttUs() >= 40'000);
    CHECK(sync.minRttUs() < 46'000);
}

TEST_CASE("clock sync: slow round trips do not move the estimate") {
    ClockSync sync;
    Path path;
    std::int64_t t = 0;
    for (int i = 0; i < 10; ++i, t += 100'000) path.ping(sync, t);
    const std::int64_t before = path.error(sync, t);
    path.extraUs = 0;
    for (int i = 0; i < 6; ++i, t += 100'000) {
        // A queueing spike: the reply took 150 ms longer, so its midpoint is 75 ms off.
        path.downUs = 170'000;
        path.ping(sync, t);
        path.downUs = 20'000;
        path.ping(sync, t + 50'000);
    }
    CHECK(std::llabs(path.error(sync, t)) < 2000);
    CHECK(std::llabs(before) < 2000);
    CHECK(sync.jitterUs() > 0);
}

TEST_CASE("clock sync: an asymmetric path leaves a stable bias of half the asymmetry, bounded by half the minimum RTT") {
    ClockSync sync;
    Path path;
    path.upUs = 10'000;
    path.downUs = 50'000;
    path.extraUs = 4000;
    std::int64_t t = 0;
    std::vector<std::int64_t> errors;
    for (int i = 0; i < 80; ++i, t += 250'000) {
        path.ping(sync, t);
        if (i >= 40) errors.push_back(path.error(sync, t + 100'000));
    }
    // The estimate reads the host clock (down - up) / 2 = 20 ms too EARLY or late; whichever, it is stable and inside the bound.
    std::int64_t lo = errors.front(), hi = errors.front();
    for (const std::int64_t e : errors) {
        lo = std::min(lo, e);
        hi = std::max(hi, e);
    }
    CHECK(hi - lo < 4000);
    CHECK(std::llabs(errors.back()) <= sync.asymmetryBoundUs());
    CHECK(std::llabs(std::llabs(errors.back()) - 20'000) < 3500);
}

TEST_CASE("clock sync: a host clock that runs 300 ppm fast is tracked") {
    ClockSync sync;
    Path path;
    path.hostRate = 1.0003;
    path.extraUs = 3000;
    std::int64_t t = 0;
    for (int i = 0; i < 60; ++i, t += 250'000) path.ping(sync, t);
    CHECK(sync.driftPpm() > 150.0);
    CHECK(sync.driftPpm() < 450.0);
    // Ten seconds beyond the last ping the extrapolation is still close (an unfitted drift would be 3 ms off).
    CHECK(std::llabs(path.error(sync, t + 10'000'000)) < 2500);
}

TEST_CASE("clock sync: a jump of the host clock is followed at once and small corrections are slewed") {
    ClockSync sync;
    Path path;
    std::int64_t t = 0;
    for (int i = 0; i < 10; ++i, t += 100'000) path.ping(sync, t);
    path.hostOffsetUs += 2'000'000;   // the host's world restarted far ahead
    for (int i = 0; i < 6; ++i, t += 100'000) path.ping(sync, t);
    CHECK(std::llabs(path.error(sync, t)) < 5000);

    ClockSync slow;
    Path p2;
    std::int64_t t2 = 0;
    for (int i = 0; i < 12; ++i, t2 += 100'000) p2.ping(slow, t2);
    const std::int64_t beforeSlew = slow.serverSimAt(t2);
    p2.hostOffsetUs += 3000;   // 3 ms: below the step threshold
    for (int i = 0; i < 5; ++i) p2.ping(slow, t2);
    const std::int64_t moved = slow.serverSimAt(t2 + 120'000) - beforeSlew - 120'000;
    CHECK(moved > 0);
    CHECK(moved < 3000);
}

TEST_CASE("clock sync: reset forgets the model and a one-way seed serves until the first ping") {
    ClockSync sync;
    CHECK_FALSE(sync.synced());
    sync.seed(1'000'000, 5'000'000, 10'000);
    CHECK(sync.synced());
    CHECK_FALSE(sync.hasPings());
    CHECK(sync.serverSimAt(1'500'000) == 5'510'000);
    sync.addPing(1'600'000, 1'640'000, 5'620'000);
    CHECK(sync.hasPings());
    sync.seed(2'000'000, 9'999'999, 0);   // pings outrank seeds
    CHECK(std::llabs(sync.serverSimAt(1'620'000) - 5'620'000) < 2000);
    sync.reset();
    CHECK_FALSE(sync.synced());
    CHECK(sync.pingCount() == 0);
}

TEST_CASE("timeline tracker: follows a moving target at a bounded rate and jumps only when hopeless") {
    TimelineTracker t(TimelineTracker::Config{100.0, 0.05, 200.0});
    CHECK(t.advance(1.0, 50.0));   // the first call starts it on the target
    CHECK(t.value() == doctest::Approx(50.0));
    // The target is 20 ahead: the rate is 1.05 at most, so it takes ~400 steps to close the gap.
    double target = 70.0;
    int steps = 0;
    double maxStep = 0.0;
    while (std::fabs(target - t.value()) > 0.01 && steps < 2000) {
        const double before = t.value();
        CHECK_FALSE(t.advance(1.0, target));
        maxStep = std::max(maxStep, t.value() - before);
        target += 1.0;
        ++steps;
    }
    CHECK(maxStep <= 1.05 + 1e-9);
    CHECK(steps > 300);
    CHECK(steps < 1200);
    CHECK(t.advance(1.0, t.value() + 500.0));   // hopeless: a jump
    CHECK(t.value() > 500.0);
}

TEST_CASE("snapshot cadence: interval, jitter, loss and the interpolation delay of the plan") {
    SnapshotCadence lan;
    // 60 Hz snapshots (16.667 ms apart in sim time) arriving with a little jitter.
    std::mt19937_64 rng(7);
    std::int64_t sim = 0;
    std::uint32_t seq = 0;
    for (int i = 0; i < 200; ++i) {
        sim += 16'667;
        ++seq;
        lan.onSnapshot(sim + 100'000 + static_cast<std::int64_t>(rng() % 1500), sim, seq);
    }
    CHECK(lan.intervalUs() == doctest::Approx(16'667).epsilon(0.01));
    CHECK(lan.rateHz() == doctest::Approx(60.0).epsilon(0.02));
    CHECK(lan.lossPermille() == doctest::Approx(0.0));
    const std::int64_t lanDelay = lan.targetDelayUs();
    CHECK(lanDelay >= 2 * 16'600);        // at least two intervals
    CHECK(lanDelay < 45'000);             // about 35-40 ms on a LAN

    SnapshotCadence wan;
    sim = 0;
    seq = 0;
    int lost = 0;
    for (int i = 0; i < 400; ++i) {
        sim += 33'333;
        ++seq;
        if (i % 25 == 24) {   // 4 % loss
            ++lost;
            continue;
        }
        wan.onSnapshot(sim + 60'000 + static_cast<std::int64_t>(rng() % 20'000), sim, seq);
    }
    CHECK(wan.intervalUs() == doctest::Approx(33'333).epsilon(0.05));
    CHECK(wan.lossPermille() == doctest::Approx(40.0).epsilon(0.2));
    const std::int64_t wanDelay = wan.targetDelayUs();
    CHECK(wanDelay > lanDelay);
    CHECK(wanDelay >= 2 * 33'000);
    CHECK(wanDelay <= 250'000);
    CHECK(lost == 16);

    // A terrible path is clamped.
    SnapshotCadence awful;
    sim = 0;
    seq = 0;
    for (int i = 0; i < 100; ++i) {
        sim += 33'333;
        ++seq;
        awful.onSnapshot(sim + static_cast<std::int64_t>(rng() % 400'000), sim, seq);
    }
    CHECK(awful.targetDelayUs() == 250'000);
}
