#include "wpiemu/core/match/robot_track.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/match/quantize.h"

namespace wpiemu::core::match {

namespace {

namespace wire = sdk::match;

struct Hermite {
    double h00, h10, h01, h11;
    double d00, d10, d01, d11;   // derivatives w.r.t. s
};

Hermite basis(double s) {
    const double s2 = s * s, s3 = s2 * s;
    return {2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + s, -2 * s3 + 3 * s2, s3 - s2,
            6 * s2 - 6 * s, 3 * s2 - 4 * s + 1, -6 * s2 + 6 * s, 3 * s2 - 2 * s};
}

double hermitePos(const Hermite& h, double p0, double p1, double v0, double v1, double t) {
    return h.h00 * p0 + h.h10 * t * v0 + h.h01 * p1 + h.h11 * t * v1;
}

double hermiteVel(const Hermite& h, double p0, double p1, double v0, double v1, double t) {
    return (h.d00 * p0 + h.d10 * t * v0 + h.d01 * p1 + h.d11 * t * v1) / t;
}

}  // namespace

RemoteSample sampleFromWire(const wire::RobotWireState& robot, std::int64_t simTimeUs, std::int64_t arrivalUs) {
    RemoteSample s;
    s.simTimeUs = simTimeUs;
    s.arrivalUs = arrivalUs;
    s.x = wire::dequantizeMilli(robot.xMm);
    s.y = wire::dequantizeMilli(robot.yMm);
    s.yaw = static_cast<double>(wire::dequantizeAngle(robot.yawQ));
    s.vx = wire::dequantizeMilli(robot.vxMmps);
    s.vy = wire::dequantizeMilli(robot.vyMmps);
    s.omega = wire::dequantizeMilli(robot.omegaMradps);
    s.wheels.reserve(robot.wheels.size());
    for (const wire::WheelWireState& w : robot.wheels) {
        RemoteWheelSample out;
        out.steerRad = static_cast<double>(wire::dequantizeAngle(w.steerAngleQ));
        out.spinRaw = static_cast<double>(wire::dequantizeAngle(w.spinAngleQ));
        out.spinCumRad = out.spinRaw;
        s.wheels.push_back(out);
    }
    s.zoneFlags = robot.zoneFlags;
    s.zonePoses = robot.zonePoses;
    return s;
}

bool RobotTrack::push(RemoteSample sample) {
    if (!samples_.empty() && sample.simTimeUs <= samples_.back().simTimeUs) return false;
    if (!samples_.empty()) {
        const RemoteSample& prev = samples_.back();
        const double dt = static_cast<double>(sample.simTimeUs - prev.simTimeUs) * 1e-6;
        const std::size_t n = std::min(sample.wheels.size(), prev.wheels.size());
        for (std::size_t i = 0; i < n; ++i) {
            const double d = wire::angleDifferenceRad(sample.wheels[i].spinRaw, prev.wheels[i].spinRaw);
            const double predicted = prev.wheels[i].spinRateRadPerSec * dt;
            const double k = std::round((predicted - d) / sdk::kTwoPi);
            const double delta = d + k * sdk::kTwoPi;
            sample.wheels[i].spinCumRad = prev.wheels[i].spinCumRad + delta;
            sample.wheels[i].spinRateRadPerSec = dt > 0.0 ? delta / dt : 0.0;
        }
    }
    samples_.push_back(std::move(sample));
    while (samples_.size() > kMaxSamples) samples_.pop_front();
    return true;
}

std::optional<RobotAt> RobotTrack::at(std::int64_t renderSimUs, std::int64_t maxExtrapolationUs) const {
    if (samples_.empty()) return std::nullopt;
    RobotAt out;
    const RemoteSample& first = samples_.front();
    const RemoteSample& last = samples_.back();

    const auto fillWheels = [&](const RemoteSample& s, double extraSeconds) {
        out.wheels.clear();
        for (const RemoteWheelSample& w : s.wheels)
            out.wheels.push_back({w.steerRad, w.spinCumRad + w.spinRateRadPerSec * extraSeconds});
    };

    if (renderSimUs <= first.simTimeUs || samples_.size() == 1) {
        const std::int64_t gap = renderSimUs - first.simTimeUs;
        if (samples_.size() == 1 && gap > 0) {
            const std::int64_t dtUs = std::min(gap, maxExtrapolationUs);
            const double dt = static_cast<double>(dtUs) * 1e-6;
            out.x = first.x + first.vx * dt;
            out.y = first.y + first.vy * dt;
            out.yaw = first.yaw + first.omega * dt;
            out.vx = first.vx;
            out.vy = first.vy;
            out.omega = first.omega;
            out.extrapolating = true;
            out.starved = gap > maxExtrapolationUs;
            out.extrapolatedUs = dtUs;
        } else {
            out.x = first.x;
            out.y = first.y;
            out.yaw = first.yaw;
            out.vx = first.vx;
            out.vy = first.vy;
            out.omega = first.omega;
        }
        fillWheels(first, 0.0);
        out.zoneFlags = first.zoneFlags;
        out.from = &first;
        return out;
    }

    if (renderSimUs >= last.simTimeUs) {
        const std::int64_t gap = renderSimUs - last.simTimeUs;
        const std::int64_t dtUs = std::min(gap, maxExtrapolationUs);
        const double dt = static_cast<double>(dtUs) * 1e-6;
        out.x = last.x + last.vx * dt;
        out.y = last.y + last.vy * dt;
        out.yaw = last.yaw + last.omega * dt;
        out.vx = last.vx;
        out.vy = last.vy;
        out.omega = last.omega;
        fillWheels(last, dt);
        out.zoneFlags = last.zoneFlags;
        out.from = &last;
        out.extrapolating = gap > 0;
        out.starved = gap > maxExtrapolationUs;
        out.extrapolatedUs = dtUs;
        return out;
    }

    // Bracketing pair: a.sim <= render < b.sim.
    std::size_t hi = 1;
    while (hi < samples_.size() && samples_[hi].simTimeUs <= renderSimUs) ++hi;
    const RemoteSample& a = samples_[hi - 1];
    const RemoteSample& b = samples_[hi];
    const double span = static_cast<double>(b.simTimeUs - a.simTimeUs) * 1e-6;
    const double w = static_cast<double>(renderSimUs - a.simTimeUs) / static_cast<double>(b.simTimeUs - a.simTimeUs);
    const Hermite h = basis(w);
    out.x = hermitePos(h, a.x, b.x, a.vx, b.vx, span);
    out.y = hermitePos(h, a.y, b.y, a.vy, b.vy, span);
    out.vx = hermiteVel(h, a.x, b.x, a.vx, b.vx, span);
    out.vy = hermiteVel(h, a.y, b.y, a.vy, b.vy, span);
    const double yawB = a.yaw + wire::angleDifferenceRad(b.yaw, a.yaw);
    out.yaw = hermitePos(h, a.yaw, yawB, a.omega, b.omega, span);
    out.omega = hermiteVel(h, a.yaw, yawB, a.omega, b.omega, span);

    const std::size_t n = std::min(a.wheels.size(), b.wheels.size());
    out.wheels.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double steer = a.wheels[i].steerRad + wire::angleDifferenceRad(b.wheels[i].steerRad, a.wheels[i].steerRad) * w;
        const double spin = a.wheels[i].spinCumRad + (b.wheels[i].spinCumRad - a.wheels[i].spinCumRad) * w;
        out.wheels.push_back({steer, spin});
    }
    out.zoneFlags = a.zoneFlags;
    out.from = &a;
    out.to = &b;
    out.blend = w;
    return out;
}

}  // namespace wpiemu::core::match
