#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "physics_test_support.h"
#include "wpiemu/core/physics/quasi_static_loads.h"
#include "wpiemu/core/physics/tire_models.h"

using namespace physics_test;

namespace {

TireInput groundedInput(float load = 490.5f / 4.f) {
    TireInput in;
    in.normalLoadN = load;
    in.chassisShareMassKg = 12.5f;
    in.wheelRadiusM = 0.0508f;
    in.dt = kDt;
    return in;
}

WheelDrive freeWheel(float torque = 0.f, float omega = 0.f) {
    WheelDrive d;
    d.omegaRadPerSec = omega;
    d.torqueNm = torque;
    d.inertiaKgM2 = 0.0016f;
    return d;
}

}  // namespace

TEST_CASE("friction curves: peak at 1, slide asymptote, monotone rise") {
    for (auto curve : {&phys::brushCurve, &phys::pacejkaLiteCurve}) {
        CHECK(curve(0.f, 0.85f) == doctest::Approx(0.f).epsilon(1e-6));
        CHECK(curve(1.f, 0.85f) == doctest::Approx(1.f).epsilon(1e-3));
        CHECK(curve(50.f, 0.85f) == doctest::Approx(0.85f).epsilon(0.01));
        float prev = 0.f;
        for (float s = 0.05f; s <= 1.f; s += 0.05f) {
            const float g = curve(s, 0.85f);
            CHECK(g >= prev);
            prev = g;
        }
        CHECK(curve(2.f, 0.85f) < 1.f);
        CHECK(curve(2.f, 0.85f) > 0.85f - 1e-3f);
        CHECK(curve(-1.f, 0.85f) == doctest::Approx(0.f));  // negative slip clamps
    }
    // Pacejka slide ratio is clamped to a usable range.
    CHECK(std::isfinite(phys::pacejkaLiteCurve(1.f, 1.f)));
    CHECK(std::isfinite(phys::pacejkaLiteCurve(1.f, 0.f)));
}

TEST_CASE("tire model: no slip and no torque produce no force") {
    phys::BrushTireModel brush;
    phys::PacejkaLiteTireModel pac;
    for (const sdk::ITireModel* m : {static_cast<const sdk::ITireModel*>(&brush), static_cast<const sdk::ITireModel*>(&pac)}) {
        const TireOutput o = m->solve(TireParams{}, groundedInput(), freeWheel());
        CHECK(o.fLongN == doctest::Approx(0.f).epsilon(1e-6));
        CHECK(o.fLatN == doctest::Approx(0.f).epsilon(1e-6));
        CHECK_FALSE(o.saturated);
    }
}

TEST_CASE("tire model: drive torque pushes the chassis forward within the friction limit") {
    phys::BrushTireModel tire;
    const TireParams p;
    const TireInput in = groundedInput();
    const float limit = p.mu * in.normalLoadN;

    const TireOutput gentle = tire.solve(p, in, freeWheel(0.3f));
    CHECK(gentle.fLongN > 0.f);
    CHECK(gentle.fLongN < limit);
    CHECK_FALSE(gentle.saturated);
    CHECK(gentle.omegaNext > 0.f);

    const TireOutput hard = tire.solve(p, in, freeWheel(500.f));  // far more than traction can carry
    CHECK(hard.fLongN <= limit + 1e-3f);
    CHECK(hard.fLongN > 0.5f * limit);
    CHECK(hard.omegaNext > gentle.omegaNext);
}

TEST_CASE("tire model: a spinning wheel on a saturated contact slides at the sliding level") {
    phys::BrushTireModel tire;
    TireParams p;
    TireInput in = groundedInput();
    in.vLongMps = 1.f;
    const TireOutput o = tire.solve(p, in, freeWheel(0.f, 120.f));  // rim 6 m/s vs chassis 1 m/s
    CHECK(o.saturated);
    CHECK(o.slipRatio > 0.5f);
    CHECK(o.fLongN == doctest::Approx(p.slideRatio * p.mu * in.normalLoadN).epsilon(0.1));
    CHECK(o.omegaNext < 120.f);  // reaction torque slows the wheel
}

TEST_CASE("tire model: chassis moving over a stationary wheel is braked") {
    phys::BrushTireModel tire;
    TireInput in = groundedInput();
    in.vLongMps = 5.f;
    const TireOutput o = tire.solve(TireParams{}, in, freeWheel());
    CHECK(o.fLongN < 0.f);
    CHECK(o.omegaNext > 0.f);  // friction spins the wheel up
}

TEST_CASE("tire model: lateral velocity is opposed and capped by mu N times the lateral scale") {
    phys::BrushTireModel tire;
    TireParams p;
    TireInput in = groundedInput();
    in.vLatMps = 2.f;
    const TireOutput o = tire.solve(p, in, freeWheel());
    CHECK(o.fLatN < 0.f);
    CHECK(std::fabs(o.fLatN) <= p.mu * in.normalLoadN + 1e-3f);
    CHECK(o.slipAngleTan < 0.f);

    p.lateralScale = 0.15f;
    const TireOutput roller = tire.solve(p, in, freeWheel());
    CHECK(std::fabs(roller.fLatN) <= 0.15f * p.mu * in.normalLoadN + 1e-3f);
    CHECK(std::fabs(roller.fLatN) < std::fabs(o.fLatN));
}

TEST_CASE("tire model: airborne wheels spin freely and apply no force") {
    phys::PacejkaLiteTireModel tire;
    TireInput in = groundedInput(0.f);
    WheelDrive d = freeWheel(2.f);
    d.dampingNmS = 0.f;
    const TireOutput o = tire.solve(TireParams{}, in, d);
    CHECK(o.fLongN == 0.f);
    CHECK(o.fLatN == 0.f);
    CHECK(o.omegaNext == doctest::Approx(kDt * 2.f / 0.0016f).epsilon(1e-3));

    in.normalLoadN = -5.f;
    CHECK(tire.solve(TireParams{}, in, d).fLongN == 0.f);
}

TEST_CASE("tire model: back-EMF damping slows an airborne wheel's acceleration") {
    phys::BrushTireModel tire;
    WheelDrive free = freeWheel(2.f);
    WheelDrive damped = free;
    damped.dampingNmS = 0.05f;
    const TireInput air = groundedInput(0.f);
    CHECK(tire.solve(TireParams{}, air, damped).omegaNext < tire.solve(TireParams{}, air, free).omegaNext);
}

TEST_CASE("tire model: outputs stay finite and inside the friction limits over a grid of inputs") {
    phys::BrushTireModel brush;
    phys::PacejkaLiteTireModel pac;
    const sdk::ITireModel* models[] = {&brush, &pac};
    for (const sdk::ITireModel* m : models)
        for (float load : {1.f, 50.f, 400.f})
            for (float vl : {-6.f, -0.01f, 0.f, 0.02f, 3.f})
                for (float vt : {-3.f, 0.f, 0.001f, 2.f})
                    for (float omega : {-90.f, 0.f, 0.5f, 80.f})
                        for (float torque : {-30.f, 0.f, 4.f}) {
                            TireParams p;
                            p.lateralScale = vt > 1.f ? 0.15f : 1.f;
                            p.tractionAngleRad = vl > 1.f ? 0.7853982f : 0.f;
                            TireInput in = groundedInput(load);
                            in.vLongMps = vl;
                            in.vLatMps = vt;
                            const TireOutput o = m->solve(p, in, freeWheel(torque, omega));
                            REQUIRE(std::isfinite(o.fLongN));
                            REQUIRE(std::isfinite(o.fLatN));
                            REQUIRE(std::isfinite(o.omegaNext));
                            CHECK(std::fabs(o.fLongN) <= p.mu * load * 1.0001f);
                            CHECK(std::fabs(o.fLatN) <= p.mu * load * p.lateralScale * 1.0001f);
                        }
}

TEST_CASE("tire model: degenerate inputs do not produce NaN") {
    phys::BrushTireModel tire;
    TireInput in = groundedInput();
    in.dt = 0.f;
    CHECK(tire.solve(TireParams{}, in, freeWheel(1.f, 3.f)).omegaNext == 3.f);
    in = groundedInput();
    WheelDrive d;  // zero inertia and damping
    d.torqueNm = 1.f;
    CHECK(std::isfinite(tire.solve(TireParams{}, in, d).omegaNext));
    in.chassisShareMassKg = 0.f;
    CHECK(std::isfinite(tire.solve(TireParams{}, in, freeWheel(1.f)).fLongN));
}

TEST_CASE("tire model: non-finite parameters or inputs give no force and keep the wheel spin") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    phys::BrushTireModel brush;
    phys::PacejkaLiteTireModel pac;
    for (const sdk::ITireModel* m : {static_cast<const sdk::ITireModel*>(&brush), static_cast<const sdk::ITireModel*>(&pac)}) {
        auto expectSafe = [&](const TireParams& p, const TireInput& in, float omega, bool keepsSpin = true) {
            const TireOutput o = m->solve(p, in, freeWheel(1.f, omega));
            CHECK(o.fLongN == 0.f);
            CHECK(o.fLatN == 0.f);
            CHECK(std::isfinite(o.omegaNext));
            if (keepsSpin) CHECK(o.omegaNext == omega);  // otherwise the wheel is merely driven by the motor torque
            CHECK(std::isfinite(o.slipRatio));
            CHECK(std::isfinite(o.slipAngleTan));
        };
        TireParams p;
        p.peakSlipLong = nan;
        expectSafe(p, groundedInput(), 7.f, false);
        p = TireParams{};
        p.lateralScale = nan;
        expectSafe(p, groundedInput(), 7.f, false);
        p = TireParams{};
        p.vRefMps = 0.f;  // 0/0 slip at standstill
        p.peakSlipLat = 0.f;
        expectSafe(p, groundedInput(), 0.f);
        TireInput in = groundedInput();
        in.vLongMps = nan;
        expectSafe(TireParams{}, in, 7.f);
        in = groundedInput();
        in.vLatMps = std::numeric_limits<float>::infinity();
        expectSafe(TireParams{}, in, 7.f, false);
    }
}

TEST_CASE("tire models are registered under their ids and cannot be registered twice") {
    TireModelRegistry registry;
    REQUIRE(phys::registerTireModels(registry).ok());
    CHECK(registry.contains("brush"));
    CHECK(registry.contains("pacejka_lite"));
    auto brush = registry.create("brush");
    REQUIRE(brush.ok());
    CHECK(brush.value()->id() == "brush");
    CHECK(registry.create("pacejka_lite").value()->id() == "pacejka_lite");
    CHECK_FALSE(registry.create("nope").ok());
    CHECK_FALSE(phys::registerTireModels(registry).ok());
}

// ---- quasi-static loads --------------------------------------------------------------------------------------

namespace {
std::vector<phys::WheelXY> squareWheels(float half = 0.25f) {
    return {{half, half}, {half, -half}, {-half, half}, {-half, -half}};
}
}  // namespace

TEST_CASE("quasi-static loads: static equal split for a symmetric layout") {
    const auto w = squareWheels();
    std::vector<float> n(4);
    phys::quasiStaticLoads(w, 50.f, 0.2f, 0.f, 0.f, 9.81f, n);
    for (float v : n) CHECK(v == doctest::Approx(50.f * 9.81f / 4.f));
}

TEST_CASE("quasi-static loads: longitudinal load transfer equals m a h / L") {
    const auto w = squareWheels();
    std::vector<float> n(4);
    phys::quasiStaticLoads(w, 50.f, 0.2f, 9.24f, 0.f, 9.81f, n);
    const float shift = 50.f * 9.24f * 0.2f / 0.5f / 2.f;  // per wheel
    CHECK(n[0] == doctest::Approx(122.625f - shift).epsilon(0.01));  // front
    CHECK(n[1] == doctest::Approx(122.625f - shift).epsilon(0.01));
    CHECK(n[2] == doctest::Approx(122.625f + shift).epsilon(0.01));  // rear
    CHECK(n[3] == doctest::Approx(122.625f + shift).epsilon(0.01));
    CHECK(n[0] + n[1] + n[2] + n[3] == doctest::Approx(490.5f).epsilon(1e-4));
}

TEST_CASE("quasi-static loads: lateral transfer unloads the inside wheels") {
    const auto w = squareWheels();
    std::vector<float> n(4);
    phys::quasiStaticLoads(w, 50.f, 0.2f, 0.f, 3.f, 9.81f, n);  // accelerating to +y (left)
    CHECK(n[0] < n[1]);  // left wheels (y>0) are unloaded, right wheels loaded
    CHECK(n[2] < n[3]);
}

TEST_CASE("quasi-static loads: lifted wheels get zero and the sum is preserved") {
    const auto w = squareWheels();
    std::vector<float> n(4);
    phys::quasiStaticLoads(w, 50.f, 0.4f, 30.f, 0.f, 9.81f, n);
    CHECK(n[0] == 0.f);
    CHECK(n[1] == 0.f);
    CHECK(n[0] + n[1] + n[2] + n[3] == doctest::Approx(490.5f).epsilon(1e-4));
}

TEST_CASE("quasi-static loads: any layout satisfies the moment balance, tilt reduces the weight") {
    // Six-wheel tank-like layout, asymmetric.
    const std::vector<phys::WheelXY> w = {{0.3f, 0.3f}, {0.0f, 0.3f}, {-0.3f, 0.3f}, {0.3f, -0.3f}, {0.0f, -0.3f}, {-0.3f, -0.3f}};
    std::vector<float> n(6);
    phys::quasiStaticLoads(w, 60.f, 0.2f, 1.f, 0.5f, 9.81f, n);
    float sum = 0.f, mx = 0.f, my = 0.f;
    for (std::size_t i = 0; i < 6; ++i) {
        sum += n[i];
        mx += n[i] * w[i].x;
        my += n[i] * w[i].y;
    }
    CHECK(sum == doctest::Approx(60.f * 9.81f).epsilon(1e-4));
    CHECK(mx == doctest::Approx(-0.2f * 60.f * 1.f).epsilon(1e-3));
    CHECK(my == doctest::Approx(-0.2f * 60.f * 0.5f).epsilon(1e-3));

    phys::quasiStaticLoads(w, 60.f, 0.2f, 0.f, 0.f, 9.81f * std::cos(15.f * kPiF / 180.f), n);
    float tilted = 0.f;
    for (float v : n) tilted += v;
    CHECK(tilted == doctest::Approx(60.f * 9.81f * std::cos(15.f * kPiF / 180.f)).epsilon(1e-4));
}

TEST_CASE("quasi-static loads: degenerate layouts fall back to an equal split") {
    std::vector<float> n(3);
    const std::vector<phys::WheelXY> line = {{-0.3f, 0.f}, {0.f, 0.f}, {0.3f, 0.f}};
    phys::quasiStaticLoads(line, 30.f, 0.2f, 4.f, 0.f, 10.f, n);
    for (float v : n) CHECK(v == doctest::Approx(100.f));

    std::vector<float> two(2);
    phys::quasiStaticLoads(std::vector<phys::WheelXY>{{0.1f, 0.f}, {-0.1f, 0.f}}, 30.f, 0.2f, 4.f, 0.f, 10.f, two);
    CHECK(two[0] == doctest::Approx(150.f));
    std::vector<float> one(1);
    phys::quasiStaticLoads(std::vector<phys::WheelXY>{{0.f, 0.f}}, 30.f, 0.2f, 0.f, 0.f, 10.f, one);
    CHECK(one[0] == doctest::Approx(300.f));

    std::vector<float> none;
    phys::quasiStaticLoads({}, 30.f, 0.2f, 0.f, 0.f, 10.f, none);  // must not crash
    std::vector<float> flipped(2);
    phys::quasiStaticLoads(std::vector<phys::WheelXY>{{0.1f, 0.f}, {-0.1f, 0.f}}, 30.f, 0.2f, 0.f, 0.f, -5.f, flipped);
    CHECK(flipped[0] == 0.f);  // negative normal gravity (upside down) carries nothing
}
