#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <vector>

#include "physics_test_support.h"

using namespace physics_test;

namespace {

constexpr int kFuelCount = 200;

void scatterFuel(IPhysicsWorld& w, int count, float x0 = 1.0f) {
    for (int i = 0; i < count; ++i) {
        const float x = x0 + 0.17f * static_cast<float>(i % 10);
        const float y = -0.8f + 0.17f * static_cast<float>((i / 10) % 10);
        const float z = 0.1f + 0.16f * static_cast<float>(i / 100);
        addBall(w, {x, y, z});
    }
}

struct Run {
    std::uint64_t hash = 0;
    double meanStepMs = 0.0;
    double maxStepMs = 0.0;
};

// Swerve robot plowing through fuel while turning its modules; returns the world hash.
Run driveThroughFuel(int workers, int fuel, int steps, double voltage) {
    PhysicsConfig cfg;
    cfg.workerCount = workers;
    Rig rig(swerveSpec(60.f), testChassis(), cfg, {-1.0f, 0.f, 0.f});
    scatterFuel(*rig.world, fuel);
    rig.run(0.5f);
    Run out;
    double total = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) * kDt;
        rig.setDrive(voltage);
        const std::array<double, 4> angle{0.3 * std::sin(t), 0.3 * std::sin(t), 0.3 * std::sin(t), 0.3 * std::sin(t)};
        steerToward(rig, angle);
        const auto t0 = std::chrono::steady_clock::now();
        rig.step();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        total += ms;
        out.maxStepMs = std::max(out.maxStepMs, ms);
    }
    out.meanStepMs = total / steps;
    out.hash = rig.world->stateHash();
    return out;
}

}  // namespace

TEST_CASE("determinism: the same scenario hashes identically across repeated runs") {
    const std::uint64_t a = driveThroughFuel(1, 60, 800, 6.0).hash;
    const std::uint64_t b = driveThroughFuel(1, 60, 800, 6.0).hash;
    CHECK(a == b);
    CHECK(a != driveThroughFuel(1, 60, 800, 5.0).hash);  // the hash really depends on the trajectory
}

TEST_CASE("determinism: the hash does not depend on the physics worker count") {
    const std::uint64_t one = driveThroughFuel(1, 120, 800, 6.0).hash;
    CHECK(one == driveThroughFuel(2, 120, 800, 6.0).hash);
    CHECK(one == driveThroughFuel(4, 120, 800, 6.0).hash);
    CHECK(one == driveThroughFuel(8, 120, 800, 6.0).hash);
}

TEST_CASE("determinism: the substep count is part of the configuration") {
    auto hashFor = [](int subSteps) {
        PhysicsConfig cfg;
        cfg.subSteps = subSteps;
        Rig rig(swerveSpec(60.f), testChassis(), cfg, {-1.0f, 0.f, 0.f});
        scatterFuel(*rig.world, 40);
        rig.setDrive(6.0);
        rig.run(3.0f);
        return rig.world->stateHash();
    };
    CHECK(hashFor(4) == hashFor(4));
    CHECK(hashFor(4) != hashFor(1));
}

TEST_CASE("performance smoke: 200 fuel bodies with a robot plowing through fit the 5 ms step budget") {
    const Run r = driveThroughFuel(1, kFuelCount, 600, 8.0);
    MESSAGE("200 fuel, 1 worker: mean " << r.meanStepMs << " ms, max " << r.maxStepMs << " ms per tick");
    CHECK(std::isfinite(r.meanStepMs));
#ifdef NDEBUG
    CHECK(r.meanStepMs < 5.0);  // budget at 200 Hz
#else
    CHECK(r.meanStepMs < 50.0);
#endif
}

TEST_CASE("performance smoke: fuel spawn and destroy bursts are cheap and leave the world consistent") {
    auto w = makeWorld();
    addFloor(*w);
    std::vector<BodyId> fuel;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 1000; ++i) fuel.push_back(addBall(*w, {0.2f * static_cast<float>(i % 20), 0.2f * static_cast<float>(i / 20 % 10), 0.5f + 0.2f * static_cast<float>(i / 200)}));
    for (int i = 0; i < 60; ++i) w->step(kDt);
    for (BodyId b : fuel) w->destroyBody(b);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    CHECK(w->stats().bodies == 1);
    CHECK(ms < 5000.0);
    w->step(kDt);
}
