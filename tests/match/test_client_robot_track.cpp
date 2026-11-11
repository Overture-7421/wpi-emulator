#include <doctest/doctest.h>

#include <cmath>

#include "wpiemu/core/match/robot_track.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/match/quantize.h"

using namespace wpiemu::core::match;
namespace wire = wpiemu::sdk::match;

namespace {

wire::RobotWireState wireState(double x, double y, double yaw, double vx, double vy, double omega) {
    wire::RobotWireState r;
    r.xMm = wire::quantizeMilliI32(x);
    r.yMm = wire::quantizeMilliI32(y);
    r.yawQ = wire::quantizeAngle(yaw);
    r.vxMmps = wire::quantizeMilliI16(vx);
    r.vyMmps = wire::quantizeMilliI16(vy);
    r.omegaMradps = wire::quantizeMilliI16(omega);
    return r;
}

// Samples of a robot moving at constant velocity, `spacingUs` apart.
RobotTrack straightTrack(int count, std::int64_t spacingUs, double vx) {
    RobotTrack track;
    for (int i = 0; i < count; ++i) {
        const std::int64_t t = static_cast<std::int64_t>(i) * spacingUs;
        track.push(sampleFromWire(wireState(vx * static_cast<double>(t) * 1e-6, 2.0, 0.0, vx, 0.0, 0.0), t, t));
    }
    return track;
}

}  // namespace

TEST_CASE("robot track: samples are strictly increasing in sim time") {
    RobotTrack track;
    CHECK(track.push(sampleFromWire(wireState(1, 0, 0, 0, 0, 0), 1000, 0)));
    CHECK_FALSE(track.push(sampleFromWire(wireState(2, 0, 0, 0, 0, 0), 1000, 0)));
    CHECK_FALSE(track.push(sampleFromWire(wireState(2, 0, 0, 0, 0, 0), 500, 0)));
    CHECK(track.size() == 1);
    CHECK(track.push(sampleFromWire(wireState(2, 0, 0, 0, 0, 0), 2000, 0)));
    CHECK(track.size() == 2);
    CHECK_FALSE(RobotTrack{}.at(0, 250'000).has_value());
}

TEST_CASE("robot track: interpolation is exact at samples and follows a straight path between them") {
    const RobotTrack track = straightTrack(6, 33'333, 2.0);
    for (std::int64_t t = 0; t <= 5 * 33'333; t += 4000) {
        const auto at = track.at(t, 250'000);
        REQUIRE(at.has_value());
        CHECK(at->x == doctest::Approx(2.0 * static_cast<double>(t) * 1e-6).epsilon(0.002).scale(1.0));
        CHECK(at->y == doctest::Approx(2.0));
        CHECK(at->vx == doctest::Approx(2.0).epsilon(0.01));
        CHECK_FALSE(at->extrapolating);
    }
}

TEST_CASE("robot track: a Hermite path through a turn stays close to the true arc") {
    // A robot on a circle of radius 2 m at 2 m/s, sampled at 30 Hz.
    constexpr double radius = 2.0, speed = 2.0;
    RobotTrack track;
    const auto truth = [&](std::int64_t t, double& x, double& y, double& yaw) {
        const double a = speed / radius * static_cast<double>(t) * 1e-6;
        x = radius * std::sin(a);
        y = radius * (1 - std::cos(a));
        yaw = a;
    };
    for (int i = 0; i < 20; ++i) {
        const std::int64_t t = i * 33'333;
        double x, y, yaw;
        truth(t, x, y, yaw);
        const double a = speed / radius * static_cast<double>(t) * 1e-6;
        track.push(sampleFromWire(wireState(x, y, yaw, speed * std::cos(a), speed * std::sin(a), speed / radius), t, t));
    }
    double worst = 0.0;
    for (std::int64_t t = 0; t < 19 * 33'333; t += 1000) {
        double x, y, yaw;
        truth(t, x, y, yaw);
        const auto at = track.at(t, 250'000);
        worst = std::max(worst, std::hypot(at->x - x, at->y - y));
    }
    CHECK(worst < 0.002);   // linear interpolation would be off by ~5 mm
}

TEST_CASE("robot track: yaw takes the short way across the wrap and steer angles too") {
    RobotTrack track;
    wire::RobotWireState a = wireState(0, 0, 3.10, 0, 0, 0);
    wire::RobotWireState b = wireState(0, 0, -3.10, 0, 0, 0);   // 0.083 rad further round
    a.wheels = {{wire::quantizeAngle(3.0), 0}};
    b.wheels = {{wire::quantizeAngle(-3.0), 0}};
    track.push(sampleFromWire(a, 0, 0));
    track.push(sampleFromWire(b, 100'000, 100'000));
    const auto mid = track.at(50'000, 250'000);
    REQUIRE(mid.has_value());
    CHECK(std::fabs(wire::angleDifferenceRad(mid->yaw, 3.141592653589793)) < 0.01);
    CHECK(std::fabs(wire::angleDifferenceRad(mid->wheels[0].steerRad, 3.141592653589793)) < 0.01);
}

TEST_CASE("robot track: a wheel spinning faster than half a turn per sample does not run backwards") {
    RobotTrack track;
    // The spin step per sample grows 2.0, 2.6, ... 5.0 rad: past pi per sample the shortest difference alone would wrap to a
    // negative step; the rate of the previous pair resolves it.
    double spin = 0.0, total = 0.0;
    std::int64_t t = 0;
    for (int i = 0; i < 6; ++i) {
        wire::RobotWireState r = wireState(0, 0, 0, 0, 0, 0);
        r.wheels = {{0, wire::quantizeAngle(spin)}};
        track.push(sampleFromWire(r, t, t));
        const double step = 2.0 + 0.6 * i;
        spin += step;
        total += step;
        t += 33'333;
    }
    const auto at = track.at(5 * 33'333, 250'000);
    REQUIRE(at.has_value());
    CHECK(at->wheels[0].spinRad == doctest::Approx(total - (2.0 + 0.6 * 5)).epsilon(0.01));
    const auto later = track.at(5 * 33'333 + 20'000, 250'000);
    CHECK(later->wheels[0].spinRad > at->wheels[0].spinRad);   // keeps turning forward while extrapolating
}

TEST_CASE("robot track: a starved buffer extrapolates for at most the limit and then holds") {
    const RobotTrack track = straightTrack(4, 33'333, 1.0);
    const std::int64_t last = 3 * 33'333;
    const auto fresh = track.at(last, 250'000);
    CHECK_FALSE(fresh->extrapolating);
    const auto soon = track.at(last + 100'000, 250'000);
    CHECK(soon->extrapolating);
    CHECK_FALSE(soon->starved);
    CHECK(soon->x == doctest::Approx(1.0 * static_cast<double>(last + 100'000) * 1e-6).epsilon(0.003).scale(1.0));
    const auto edge = track.at(last + 250'000, 250'000);
    CHECK_FALSE(edge->starved);
    const auto beyond = track.at(last + 400'000, 250'000);
    CHECK(beyond->starved);
    CHECK(beyond->extrapolatedUs == 250'000);
    CHECK(beyond->x == doctest::Approx(edge->x));   // held
}

TEST_CASE("robot track: zone flags hold the earlier sample and the zone samples are exposed for blending") {
    RobotTrack track;
    wire::RobotWireState a = wireState(0, 0, 0, 0, 0, 0);
    a.zoneFlags = 0b01;
    wire::RobotWireState b = wireState(1, 0, 0, 0, 0, 0);
    b.zoneFlags = 0b11;
    b.zonePoses = {wire::makeZonePoseInput(0, {{0.1f, 0.f, 0.f}, wpiemu::sdk::Quat{}})};
    track.push(sampleFromWire(a, 0, 0));
    track.push(sampleFromWire(b, 100'000, 0));
    const auto at = track.at(75'000, 250'000);
    CHECK(at->zoneFlags == 0b01);
    REQUIRE(at->from != nullptr);
    REQUIRE(at->to != nullptr);
    CHECK(at->blend == doctest::Approx(0.75));
    CHECK(at->to->zonePoses.size() == 1);
    CHECK(track.at(100'000, 250'000)->zoneFlags == 0b11);
}
