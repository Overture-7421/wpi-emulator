// The host derives entity velocities from successive frames (core/match/entity_motion.h). These tests drive the tracker with
// exact physical trajectories (rest, a straight roll, a ballistic arc, a bounce, a teleport) at the irregular frame spacing a
// host with several peers sees, and check what the clients would be told.
#include <cmath>
#include <map>
#include <vector>

#include <doctest/doctest.h>

#include "wpiemu/core/match/entity_motion.h"
#include "wpiemu/sdk/match/entity_prediction.h"
#include "wpiemu/sdk/match/quantize.h"

using wpiemu::core::match::EntityMotionTracker;
using wpiemu::sdk::Vec3;
namespace wire = wpiemu::sdk::match;

namespace {

constexpr double kG = 9.81;

struct Observed {
    wire::EntityState state;
    bool unsettled = false;
};

Observed observeOne(EntityMotionTracker& tracker, std::int64_t tUs, std::uint32_t id, const Vec3& p) {
    wire::EntityState e;
    e.id = id;
    e.xCm = wire::quantizeCentiI16(p.x);
    e.yCm = wire::quantizeCentiI16(p.y);
    e.zCm = wire::quantizeCentiI16(p.z);
    std::vector<wire::EntityState> entities{e};
    std::vector<std::uint8_t> unsettled;
    const std::vector<Vec3> raw{p};
    tracker.observe(0, tUs, raw, entities, unsettled);
    return {entities[0], !unsettled.empty() && unsettled[0] != 0};
}

// Frame times of a host serving several peers: 15 and 20 ms apart.
std::int64_t frameTime(int k) { return 5'000 + (k / 2) * 35'000 + (k % 2) * 15'000; }

Vec3 arc(double t, const Vec3& p0, const Vec3& v0) {
    return {static_cast<float>(p0.x + v0.x * t), static_cast<float>(p0.y + v0.y * t),
            static_cast<float>(p0.z + v0.z * t - 0.5 * kG * t * t)};
}

}  // namespace

TEST_CASE("entity motion: a resting entity has no velocity and is not flying") {
    EntityMotionTracker tracker;
    for (int k = 0; k < 6; ++k) {
        const Observed o = observeOne(tracker, frameTime(k), 7, {3.f, 2.f, 0.075f});
        CHECK(o.state.vxCms == 0);
        CHECK(o.state.vyCms == 0);
        CHECK(o.state.vzCms == 0);
        CHECK_FALSE(o.state.ballistic);
        CHECK_FALSE(o.unsettled);
    }
}

TEST_CASE("entity motion: a straight roll is measured from the second frame on and is not ballistic") {
    EntityMotionTracker tracker;
    std::map<int, Observed> seen;
    for (int k = 0; k < 8; ++k) {
        const double t = static_cast<double>(frameTime(k)) / 1e6;
        seen[k] = observeOne(tracker, frameTime(k), 3, {static_cast<float>(1.0 + 2.5 * t), static_cast<float>(4.0 - 1.0 * t), 0.075f});
    }
    CHECK(seen[0].state.vxCms == 0);   // nothing to difference yet
    for (int k = 1; k < 8; ++k) {
        CAPTURE(k);
        CHECK(std::abs(seen[k].state.vxCms - 250) <= 2);
        CHECK(std::abs(seen[k].state.vyCms + 100) <= 2);
        CHECK(std::abs(seen[k].state.vzCms) <= 2);
        CHECK_FALSE(seen[k].state.ballistic);
        CHECK_FALSE(seen[k].unsettled);
    }
}

TEST_CASE("entity motion: a ballistic arc is recognised and its velocity is the one at the frame") {
    EntityMotionTracker tracker;
    const Vec3 p0{2.f, 1.f, 1.0f};
    const Vec3 v0{4.f, -2.f, 6.f};
    bool everBallistic = false;
    for (int k = 0; k < 30; ++k) {
        const double t = static_cast<double>(frameTime(k) - frameTime(0)) / 1e6;
        const Observed o = observeOne(tracker, frameTime(k), 11, arc(t, p0, v0));
        if (k < 1) continue;
        CAPTURE(k);
        // The apex is at 0.61 s (frame 24 or so): the vertical velocity changes sign and the entity still flies.
        CHECK(o.state.ballistic);
        everBallistic = everBallistic || o.state.ballistic;
        CHECK(std::abs(o.state.vxCms - 400) <= 2);
        CHECK(std::abs(o.state.vyCms + 200) <= 2);
        CHECK(std::abs(o.state.vzCms - static_cast<int>(std::lround((v0.z - kG * t) * 100.0))) <= 5);
        CHECK_FALSE(o.unsettled);
    }
    CHECK(everBallistic);
}

TEST_CASE("entity motion: the shared model, started from what the tracker reports, stays within a centimetre or two of the arc") {
    EntityMotionTracker tracker;
    const Vec3 p0{2.f, 1.f, 1.0f};
    const Vec3 v0{4.f, -2.f, 6.f};
    Observed last;
    std::int64_t lastTime = 0;
    for (int k = 0; k < 6; ++k) {
        const double t = static_cast<double>(frameTime(k) - frameTime(0)) / 1e6;
        last = observeOne(tracker, frameTime(k), 11, arc(t, p0, v0));
        lastTime = frameTime(k);
    }
    REQUIRE(last.state.ballistic);
    for (const std::int64_t dt : {50'000, 150'000, 300'000, 500'000}) {
        const double t = static_cast<double>(lastTime + dt - frameTime(0)) / 1e6;
        const Vec3 truth = arc(t, p0, v0);
        const wire::EntityState predicted = wire::predictEntity(last.state, dt);
        CAPTURE(dt);
        CHECK(std::abs(predicted.xCm - truth.x * 100.0) <= 3.0);
        CHECK(std::abs(predicted.yCm - truth.y * 100.0) <= 3.0);
        CHECK(std::abs(predicted.zCm - truth.z * 100.0) <= 4.0);
    }
}

TEST_CASE("entity motion: a bounce is held back for one frame and the next frame carries the velocity after it") {
    EntityMotionTracker tracker;
    // Falls from 1 m onto the floor (radius 7.5 cm) at t0 = 0.4 s and rebounds with 55 percent of the speed.
    const double radius = 0.075;
    const double h0 = 1.0;
    const double tHit = std::sqrt(2.0 * (h0 - radius) / kG);
    const double vHit = kG * tHit;
    const auto height = [&](double t) {
        if (t < tHit) return h0 - 0.5 * kG * t * t;
        const double s = t - tHit;
        return radius + 0.55 * vHit * s - 0.5 * kG * s * s;
    };
    std::map<int, Observed> seen;
    int hitFrame = -1;
    for (int k = 0; k < 40; ++k) {
        const double t = static_cast<double>(frameTime(k)) / 1e6;
        seen[k] = observeOne(tracker, frameTime(k), 5, {1.f, 1.f, static_cast<float>(height(t))});
        if (hitFrame < 0 && t > tHit) hitFrame = k;
    }
    REQUIRE(hitFrame > 3);
    // The frame whose interval holds the impact is unsettled exactly once; the one after it is clean and rising.
    int unsettledFrames = 0;
    for (int k = 2; k < 40; ++k) unsettledFrames += seen[k].unsettled ? 1 : 0;
    CHECK(unsettledFrames == 1);
    CHECK(seen[hitFrame].unsettled);
    const wire::EntityState& after = seen[hitFrame + 1].state;
    const double sAfter = static_cast<double>(frameTime(hitFrame + 1)) / 1e6 - tHit;
    CHECK(after.ballistic);
    CHECK(std::abs(after.vzCms - std::lround((0.55 * vHit - kG * sAfter) * 100.0)) <= 25);   // within the 15 to 20 ms interval's mean
    // Before the impact it falls ballistically.
    CHECK(seen[hitFrame - 2].state.ballistic);
    CHECK(seen[hitFrame - 2].state.vzCms < 0);
}

TEST_CASE("entity motion: teleports and gaps give no velocity, new entities wait one frame, gone ones are forgotten") {
    EntityMotionTracker tracker;
    observeOne(tracker, 5'000, 1, {0.f, 0.f, 0.075f});
    // 20 m in 15 ms is not a velocity.
    const Observed jump = observeOne(tracker, 20'000, 1, {20.f, 0.f, 0.075f});
    CHECK(jump.state.vxCms == 0);
    CHECK_FALSE(jump.state.ballistic);
    // A gap of a second (nothing was built) is not an interval either.
    observeOne(tracker, 40'000, 1, {20.f, 0.f, 0.075f});
    const Observed gap = observeOne(tracker, 1'040'000, 1, {21.f, 0.f, 0.075f});
    CHECK(gap.state.vxCms == 0);

    // A second entity appearing in a tracked group is unsettled for its first frame only.
    std::vector<wire::EntityState> entities(2);
    entities[0].id = 1;
    entities[1].id = 2;
    std::vector<Vec3> raw{{21.f, 0.f, 0.075f}, {5.f, 5.f, 0.075f}};
    std::vector<std::uint8_t> unsettled;
    tracker.observe(0, 1'060'000, raw, entities, unsettled);
    REQUIRE(unsettled.size() == 2);
    CHECK(unsettled[0] == 0);
    CHECK(unsettled[1] == 1);
    raw[1] = {5.1f, 5.f, 0.075f};
    tracker.observe(0, 1'075'000, raw, entities, unsettled);
    CHECK(unsettled[1] == 0);
    CHECK(std::abs(entities[1].vxCms - 667) <= 1);   // 10 cm in 15 ms

    // Entity 2 leaves for a frame and comes back: it is new again.
    entities.erase(entities.begin() + 1);
    raw.erase(raw.begin() + 1);
    tracker.observe(0, 1'090'000, raw, entities, unsettled);
    entities.push_back({});
    entities[1].id = 2;
    raw.push_back({5.f, 5.f, 0.075f});
    tracker.observe(0, 1'105'000, raw, entities, unsettled);
    CHECK(unsettled[1] == 1);
    CHECK(entities[1].vxCms == 0);
}

TEST_CASE("entity motion: the first frame of a group is not unsettled (a client joining mid-match gets everything at once)") {
    EntityMotionTracker tracker;
    std::vector<wire::EntityState> entities(3);
    std::vector<Vec3> raw;
    for (std::uint32_t i = 0; i < 3; ++i) {
        entities[i].id = i + 1;
        raw.push_back({static_cast<float>(i), 0.f, 0.075f});
    }
    std::vector<std::uint8_t> unsettled;
    tracker.observe(4, 10'000, raw, entities, unsettled);
    REQUIRE(unsettled.size() == 3);
    for (const std::uint8_t u : unsettled) CHECK(u == 0);
    // Groups are tracked separately.
    tracker.observe(5, 10'000, raw, entities, unsettled);
    for (const std::uint8_t u : unsettled) CHECK(u == 0);
}

TEST_CASE("entity motion: the result is a pure function of the frame sequence") {
    const auto run = [] {
        EntityMotionTracker tracker;
        std::vector<wire::EntityState> out;
        for (int k = 0; k < 20; ++k) {
            const double t = static_cast<double>(frameTime(k)) / 1e6;
            out.push_back(observeOne(tracker, frameTime(k), 9, arc(t, {0.f, 0.f, 1.f}, {3.f, 1.f, 5.f})).state);
        }
        return out;
    };
    CHECK(run() == run());
}
