#include <doctest/doctest.h>

#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>

#include "wpiemu/sdk/match/entity_prediction.h"

using namespace wpiemu::sdk::match;

namespace {

EntityState state(std::int16_t x, std::int16_t y, std::int16_t z, std::int16_t vx, std::int16_t vy, std::int16_t vz, bool ballistic) {
    EntityState e;
    e.id = 1;
    e.xCm = x;
    e.yCm = y;
    e.zCm = z;
    e.vxCms = vx;
    e.vyCms = vy;
    e.vzCms = vz;
    e.ballistic = ballistic;
    return e;
}

}  // namespace

TEST_CASE("entity prediction: x and y move at constant velocity, z is untouched without the ballistic flag") {
    const EntityState s = state(100, -50, 7, 400, -200, 999, false);
    CHECK(predictEntity(s, 0) == s);
    const EntityState p = predictEntity(s, 250'000);
    CHECK(p.xCm == 200);
    CHECK(p.yCm == -100);
    CHECK(p.zCm == 7);        // a rolling entity has no gravity, vz is ignored
    CHECK(p.vzCms == 999);
    CHECK(p.id == 1);
    CHECK_FALSE(p.ballistic);
    // Rounding is to the nearest centimetre, half away from zero: 1 cm/s for 0.5 s is half a centimetre.
    CHECK(predictEntity(state(0, 0, 0, 1, -1, 0, false), 500'000).xCm == 1);
    CHECK(predictEntity(state(0, 0, 0, 1, -1, 0, false), 500'000).yCm == -1);
    CHECK(predictEntity(state(0, 0, 0, 1, -1, 0, false), 499'999).xCm == 0);
}

TEST_CASE("entity prediction: a ballistic entity follows z + vz t - g t^2 / 2 and its vertical velocity falls with g") {
    const EntityState s = state(0, 0, 100, 0, 0, 600, true);
    const EntityState p = predictEntity(s, 500'000);
    CHECK(p.zCm == 277);      // 100 + 300 - 122.6 = 277.4
    CHECK(p.vzCms == 109);    // 600 - 490.5 (half away from zero)
    CHECK(p.ballistic);
    // At the apex (vz / g = 0.6116 s) the entity is at its highest point.
    const EntityState apex = predictEntity(s, 611'621);
    CHECK(apex.zCm == 283);   // 100 + 183.7
    CHECK(std::abs(apex.vzCms) <= 1);
    CHECK(predictEntity(s, 1'000'000).zCm == 210);   // 100 + 600 - 490.5, rounded half away from zero
}

TEST_CASE("entity prediction: a ballistic entity that reaches the floor lands and keeps rolling") {
    const EntityState s = state(0, 0, 10, 100, 0, 0, true);
    const EntityState p = predictEntity(s, 1'000'000);
    CHECK(p.zCm == kEntityFloorCm);
    CHECK(p.vzCms == 0);
    CHECK_FALSE(p.ballistic);
    CHECK(p.xCm == 100);
    // Just before it lands it is still in flight.
    const EntityState before = predictEntity(s, 100'000);
    CHECK(before.ballistic);
    CHECK(before.zCm == 5);   // 10 - 4.9 cm
}

TEST_CASE("entity prediction: the horizon, negative time and saturation are total") {
    const EntityState s = state(32000, -32000, 30000, 10000, -10000, 0, false);
    CHECK(predictEntity(s, -5'000'000) == s);
    const EntityState far = predictEntity(s, 5 * kEntityPredictHorizonUs);
    CHECK(far == predictEntity(s, kEntityPredictHorizonUs));   // it holds still beyond the horizon
    CHECK(far.xCm == std::numeric_limits<std::int16_t>::max());
    CHECK(far.yCm == std::numeric_limits<std::int16_t>::min());
    const EntityState fast = state(0, 0, 0, std::numeric_limits<std::int16_t>::max(), std::numeric_limits<std::int16_t>::min(), 0, false);
    const EntityState atHorizon = predictEntity(fast, kEntityPredictHorizonUs);
    CHECK(atHorizon.xCm == std::numeric_limits<std::int16_t>::max());   // 655 m saturates
    CHECK(atHorizon.yCm == std::numeric_limits<std::int16_t>::min());
    const EntityState rocket = state(0, 0, 32000, 0, 0, std::numeric_limits<std::int16_t>::max(), true);
    CHECK(predictEntity(rocket, kEntityPredictHorizonUs).zCm == std::numeric_limits<std::int16_t>::max());
}

TEST_CASE("entity prediction: pinned values, the same on every platform") {
    // These values are the contract between the host and every client: change the model and the protocol changes.
    struct Case {
        EntityState from;
        std::int64_t dtUs;
        std::int16_t x, y, z, vz;
    };
    const std::vector<Case> cases{
        {state(0, 0, 150, 800, -300, 450, true), 16'667, 13, -5, 157, 434},
        {state(0, 0, 150, 800, -300, 450, true), 100'000, 80, -30, 190, 352},
        {state(0, 0, 150, 800, -300, 450, true), 333'333, 267, -100, 245, 123},
        {state(0, 0, 150, 800, -300, 450, true), 1'000'000, 800, -300, 109, -531},
        {state(-1200, 800, 8, -25, 40, 0, false), 1'500'000, -1238, 860, 8, 0},
    };
    for (const Case& c : cases) {
        CAPTURE(c.dtUs);
        const EntityState p = predictEntity(c.from, c.dtUs);
        CHECK(p.xCm == c.x);
        CHECK(p.yCm == c.y);
        CHECK(p.zCm == c.z);
        CHECK(p.vzCms == c.vz);
    }
}

TEST_CASE("entity prediction: renderEntityCm blends the old extrapolation into the new update over the blend window") {
    const EntityState previous = state(0, 0, 0, 100, 0, 0, false);   // x = t * 100 cm/s
    const EntityState current = state(20, 0, 0, 100, 0, 0, false);   // the host corrected it 10 cm ahead at t = 100 ms
    constexpr std::int64_t stamp = 100'000;
    // No previous update: the extrapolation of the current one.
    CHECK(renderEntityCm(nullptr, 0, current, stamp, stamp).x == doctest::Approx(20.0));
    CHECK(renderEntityCm(nullptr, 0, current, stamp, stamp + 50'000).x == doctest::Approx(25.0));
    // At the stamp the picture is still the old extrapolation (no pop); after the window it is the new one.
    CHECK(renderEntityCm(&previous, 0, current, stamp, stamp).x == doctest::Approx(10.0));
    CHECK(renderEntityCm(&previous, 0, current, stamp, stamp + kEntityBlendUs / 2).x == doctest::Approx(20.0));   // (15 + 25) / 2
    CHECK(renderEntityCm(&previous, 0, current, stamp, stamp + kEntityBlendUs).x == doctest::Approx(30.0));
    CHECK(renderEntityCm(&previous, 0, current, stamp, stamp + 5 * kEntityBlendUs).x == doctest::Approx(70.0));
    // Rendering before the stamp (a late update) never moves backwards past it.
    CHECK(renderEntityCm(&previous, 0, current, stamp, stamp - 30'000).x == doctest::Approx(7.0));
}

TEST_CASE("entity prediction: the model is the same function of its input everywhere (no state, no clock)") {
    const EntityState s = state(321, -123, 77, 55, -66, 140, true);
    for (std::int64_t dt = 0; dt <= kEntityPredictHorizonUs; dt += 7'919) CHECK(predictEntity(s, dt) == predictEntity(s, dt));
    static_assert(predictEntity(EntityState{1, 10, 20, 30, 100, 0, 0, false}, 500'000).xCm == 60);   // usable at compile time
}
