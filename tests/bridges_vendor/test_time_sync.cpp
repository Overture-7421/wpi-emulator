#include <doctest/doctest.h>

#include <cstdint>
#include <vector>

#include "wpiemu/bridges/vendor/time_sync.h"

using wpiemu::bridges::vendor::RealtimePacer;

namespace {
struct FakeWall {
    std::int64_t nowUs = 1'000'000;
    RealtimePacer::NowUs fn() {
        return [this] { return nowUs; };
    }
};
}  // namespace

TEST_CASE("vendor pacer: sim time never runs ahead of the wall clock at factor 1") {
    FakeWall wall;
    RealtimePacer pacer(1.0, wall.fn());
    // First tick is due one step after the anchor.
    CHECK(pacer.microsToWait(5000) == 5000);
    wall.nowUs += 5000;
    CHECK(pacer.microsToWait(5000) == 5000);
    wall.nowUs += 2000;  // the robot loop was slow: only 2 ms elapsed before asking again
    CHECK(pacer.microsToWait(5000) == 8000);
}

TEST_CASE("vendor pacer: factor scales the schedule and late ticks never wait") {
    FakeWall wall;
    RealtimePacer pacer(2.0, wall.fn());
    CHECK(pacer.microsToWait(5000) == 2500);  // 2x real time
    wall.nowUs += 10'000;                     // we are 7.5 ms late
    CHECK(pacer.microsToWait(5000) == 0);
    CHECK(pacer.factor() == doctest::Approx(2.0));
    pacer.setFactor(-1.0);                    // invalid factors fall back to real time
    CHECK(pacer.factor() == doctest::Approx(1.0));
}

TEST_CASE("vendor pacer: a stall re-anchors instead of bursting, restart() forgets history") {
    FakeWall wall;
    RealtimePacer pacer(1.0, wall.fn());
    pacer.microsToWait(5000);
    wall.nowUs += 5'000'000;  // robot stalled for 5 s
    CHECK(pacer.microsToWait(5000) == 0);
    // After the stall the schedule continues from "now": next tick waits one step, not zero (no catch-up burst).
    CHECK(pacer.microsToWait(5000) == 5000);

    pacer.restart();
    wall.nowUs += 123;
    CHECK(pacer.microsToWait(5000) == 5000);
}
