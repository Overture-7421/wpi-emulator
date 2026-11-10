#include "wpiemu/core/match/own_predictor.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/match/quantize.h"

namespace wpiemu::core::match {

namespace {

namespace wire = sdk::match;

double wrap(double a) { return wire::angleDifferenceRad(a, 0.0); }

sdk::Pose3 lerpPose(const sdk::Pose3& a, const sdk::Pose3& b, float t) {
    sdk::Pose3 out;
    out.position = a.position + (b.position - a.position) * t;
    sdk::Quat qb = b.rotation;
    if (a.rotation.x * qb.x + a.rotation.y * qb.y + a.rotation.z * qb.z + a.rotation.w * qb.w < 0.f) qb = {-qb.x, -qb.y, -qb.z, -qb.w};
    out.rotation = sdk::Quat{a.rotation.x + (qb.x - a.rotation.x) * t, a.rotation.y + (qb.y - a.rotation.y) * t,
                             a.rotation.z + (qb.z - a.rotation.z) * t, a.rotation.w + (qb.w - a.rotation.w) * t}.normalized();
    return out;
}

// `base` with the planar state of `a` (what the wire carries); height, tilt, vertical motion and wheel spin speed stay.
OwnBodyState withAuthoritative(const OwnBodyState& base, const OwnAuthoritative& a) {
    OwnBodyState s = base;
    s.pose.position.x = static_cast<float>(a.x);
    s.pose.position.y = static_cast<float>(a.y);
    const sdk::Euler e = base.pose.rotation.toEuler();
    s.pose.rotation = sdk::Quat::fromEuler(e.rollRad, e.pitchRad, static_cast<float>(a.yaw));
    s.linearVelocity.x = static_cast<float>(a.vx);
    s.linearVelocity.y = static_cast<float>(a.vy);
    s.angularVelocity.z = static_cast<float>(a.omega);
    const std::size_t n = s.wheels.size();
    if (a.steerRad.size() == n)
        for (std::size_t i = 0; i < n; ++i)
            s.wheels[i].steerAngleRad = base.wheels[i].steerAngleRad + wrap(a.steerRad[i] - base.wheels[i].steerAngleRad);
    if (a.spinRad.size() == n)
        for (std::size_t i = 0; i < n; ++i)
            s.wheels[i].driveWheelAngleRad = base.wheels[i].driveWheelAngleRad + wrap(a.spinRad[i] - base.wheels[i].driveWheelAngleRad);
    return s;
}

}  // namespace

OwnAuthoritative authoritativeFromWire(const wire::RobotWireState& robot) {
    OwnAuthoritative a;
    a.x = wire::dequantizeMilli(robot.xMm);
    a.y = wire::dequantizeMilli(robot.yMm);
    a.yaw = static_cast<double>(wire::dequantizeAngle(robot.yawQ));
    a.vx = wire::dequantizeMilli(robot.vxMmps);
    a.vy = wire::dequantizeMilli(robot.vyMmps);
    a.omega = wire::dequantizeMilli(robot.omegaMradps);
    for (const wire::WheelWireState& w : robot.wheels) {
        a.steerRad.push_back(static_cast<double>(wire::dequantizeAngle(w.steerAngleQ)));
        a.spinRad.push_back(static_cast<double>(wire::dequantizeAngle(w.spinAngleQ)));
    }
    return a;
}

void OwnPredictor::Correction::clear(std::size_t wheels) {
    x = y = yaw = 0.0;
    steer.assign(wheels, 0.0);
    spin.assign(wheels, 0.0);
}

OwnPredictor::OwnPredictor(std::unique_ptr<PredictionWorld> world, OwnPredictorConfig config)
    : world_(std::move(world)), config_(config) {
    history_.push_back(world_->capture());
    correction_.clear(world_->wheelCount());
}

const OwnBodyState* OwnPredictor::stateAt(std::uint64_t tick) const {
    if (tick < firstTick_ || tick - firstTick_ >= history_.size()) return nullptr;
    return &history_[static_cast<std::size_t>(tick - firstTick_)];
}

void OwnPredictor::record(const OwnBodyState& s) {
    history_.push_back(s);
    while (history_.size() > config_.historyTicks) {
        history_.pop_front();
        ++firstTick_;
    }
}

void OwnPredictor::applyInput(const PredictedInput& in) { world_->setSetpoints(in.setpoints); }

void OwnPredictor::runTick(std::uint64_t tick, bool enabled) {
    if (hook_) hook_(*world_, tick);
    world_->step(enabled, static_cast<float>(config_.stepUs) * 1e-6f);
}

void OwnPredictor::prune(std::uint32_t ackInputSeq) {
    if (ackInputSeq == wire::kNoInputSeq) return;
    std::size_t keepFrom = 0;
    for (std::size_t i = 0; i < inputs_.size(); ++i)
        if (!wire::seqNewer(inputs_[i].seq, ackInputSeq)) keepFrom = i;
    inputs_.erase(inputs_.begin(), inputs_.begin() + static_cast<std::ptrdiff_t>(keepFrom));
}

void OwnPredictor::start(const OwnAuthoritative& authoritative, std::uint64_t hostTick, std::uint64_t toTick, bool enabled) {
    const OwnBodyState s = withAuthoritative(world_->capture(), authoritative);
    if (!world_->restore(s).ok()) {
        started_ = false;
        return;
    }
    history_.clear();
    firstTick_ = hostTick;
    tick_ = hostTick;
    history_.push_back(s);
    correction_.clear(world_->wheelCount());
    started_ = true;

    world_->setSetpoints(std::vector<sdk::WheelSetpoint>(world_->wheelCount()));
    std::size_t next = 0;
    for (; tick_ < toTick;) {
        const PredictedInput* apply = nullptr;
        while (next < inputs_.size() && std::max(inputs_[next].targetTick, hostTick) <= tick_) apply = &inputs_[next++];
        if (apply != nullptr) applyInput(*apply);
        runTick(tick_, enabled);
        record(world_->capture());
        ++tick_;
    }
}

void OwnPredictor::stepTick(const PredictedInput* input, bool enabled) {
    if (input != nullptr) {
        inputs_.push_back(*input);
        while (inputs_.size() > config_.maxInputs) inputs_.pop_front();
        applyInput(*input);
    }
    runTick(tick_, enabled);
    record(world_->capture());
    ++tick_;
}

ReconcileResult OwnPredictor::reconcile(const OwnAuthoritative& auth, std::uint64_t snapshotTick, std::uint32_t ackInputSeq,
                                        bool enabled) {
    ReconcileResult out;
    if (!started_) return out;
    if (snapshotTick > tick_) {
        out.kind = ReconcileKind::Ahead;
        return out;
    }
    const OwnBodyState* predicted = stateAt(snapshotTick);
    if (predicted == nullptr) {
        out.kind = ReconcileKind::TooOld;
        return out;
    }

    out.posErrM = std::hypot(auth.x - predicted->pose.position.x, auth.y - predicted->pose.position.y);
    out.yawErrRad = std::fabs(wrap(auth.yaw - predicted->pose.rotation.yaw()));
    out.velErrMps = std::hypot(auth.vx - predicted->linearVelocity.x, auth.vy - predicted->linearVelocity.y);
    if (auth.steerRad.size() == predicted->wheels.size())
        for (std::size_t i = 0; i < auth.steerRad.size(); ++i)
            out.steerErrRad = std::max(out.steerErrRad, std::fabs(wrap(auth.steerRad[i] - predicted->wheels[i].steerAngleRad)));

    const bool inBand = out.posErrM <= config_.deadbandPosM && out.yawErrRad <= config_.deadbandYawRad &&
                        out.velErrMps <= config_.deadbandVelMps && out.steerErrRad <= config_.deadbandSteerRad;
    if (inBand) {
        out.kind = ReconcileKind::InBand;
        prune(ackInputSeq);
        return out;
    }

    const OwnBodyState before = history_.back();
    const OwnBodyState merged = withAuthoritative(*predicted, auth);
    if (!world_->restore(merged).ok()) {
        out.kind = ReconcileKind::InBand;
        return out;
    }
    history_.resize(static_cast<std::size_t>(snapshotTick - firstTick_) + 1);
    history_.back() = merged;

    // The inputs the host has not acknowledged, in order; the newest acknowledged one is what drives the robot at the
    // snapshot's tick.
    const PredictedInput* active = nullptr;
    std::vector<const PredictedInput*> unacked;
    for (const PredictedInput& in : inputs_) {
        if (ackInputSeq != wire::kNoInputSeq && !wire::seqNewer(in.seq, ackInputSeq)) active = &in;
        else unacked.push_back(&in);
    }
    if (active != nullptr) applyInput(*active);
    else world_->setSetpoints(std::vector<sdk::WheelSetpoint>(world_->wheelCount()));

    std::size_t next = 0;
    for (std::uint64_t j = snapshotTick; j < tick_; ++j) {
        const PredictedInput* apply = nullptr;
        while (next < unacked.size() && std::max(unacked[next]->targetTick, snapshotTick) <= j) apply = unacked[next++];
        if (apply != nullptr) applyInput(*apply);
        runTick(j, enabled);
        history_.push_back(world_->capture());
        ++out.replayedTicks;
    }
    // The replay may have applied inputs newer than tick(); later forward steps continue with the last one applied.
    const OwnBodyState& after = history_.back();
    prune(ackInputSeq);

    const double shift = std::hypot(static_cast<double>(before.pose.position.x - after.pose.position.x),
                                    static_cast<double>(before.pose.position.y - after.pose.position.y));
    const double yawShift = wrap(static_cast<double>(before.pose.rotation.yaw() - after.pose.rotation.yaw()));
    if (shift >= config_.snapPosM || std::fabs(yawShift) >= config_.snapYawRad) {
        correction_.clear(world_->wheelCount());
        out.kind = ReconcileKind::Snapped;
        return out;
    }
    correction_.x += static_cast<double>(before.pose.position.x - after.pose.position.x);
    correction_.y += static_cast<double>(before.pose.position.y - after.pose.position.y);
    correction_.yaw += yawShift;
    for (std::size_t i = 0; i < correction_.steer.size() && i < after.wheels.size() && i < before.wheels.size(); ++i) {
        correction_.steer[i] += wrap(before.wheels[i].steerAngleRad - after.wheels[i].steerAngleRad);
        correction_.spin[i] += before.wheels[i].driveWheelAngleRad - after.wheels[i].driveWheelAngleRad;
    }
    if (correctionMagnitudeM() >= config_.snapPosM) correction_.clear(world_->wheelCount());
    out.kind = ReconcileKind::Corrected;
    return out;
}

OwnShown OwnPredictor::shown(double frac) const {
    const OwnBodyState& b = history_.back();
    const OwnBodyState& a = history_.size() >= 2 ? history_[history_.size() - 2] : b;
    const float t = static_cast<float>(std::clamp(frac, 0.0, 1.0));
    OwnShown out;
    out.pose = lerpPose(a.pose, b.pose, t);
    out.linearVelocity = a.linearVelocity + (b.linearVelocity - a.linearVelocity) * t;
    out.angularVelocity = a.angularVelocity + (b.angularVelocity - a.angularVelocity) * t;
    const std::size_t n = b.wheels.size();
    out.wheels.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const sdk::WheelKinematics& wa = i < a.wheels.size() ? a.wheels[i] : b.wheels[i];
        const sdk::WheelKinematics& wb = b.wheels[i];
        out.wheels[i].steerRad = wa.steerAngleRad + wrap(wb.steerAngleRad - wa.steerAngleRad) * static_cast<double>(t);
        out.wheels[i].spinRad = wa.driveWheelAngleRad + (wb.driveWheelAngleRad - wa.driveWheelAngleRad) * static_cast<double>(t);
        if (i < correction_.steer.size()) {
            out.wheels[i].steerRad += correction_.steer[i];
            out.wheels[i].spinRad += correction_.spin[i];
        }
    }
    out.pose.position.x += static_cast<float>(correction_.x);
    out.pose.position.y += static_cast<float>(correction_.y);
    if (correction_.yaw != 0.0) out.pose.rotation = (sdk::Quat::fromYaw(static_cast<float>(correction_.yaw)) * out.pose.rotation).normalized();
    return out;
}

void OwnPredictor::decayCorrection(std::int64_t dtUs) {
    if (dtUs <= 0) return;
    const double k = std::exp(-static_cast<double>(dtUs) / static_cast<double>(std::max<std::int64_t>(1, config_.correctionTauUs)));
    correction_.x *= k;
    correction_.y *= k;
    correction_.yaw *= k;
    for (double& v : correction_.steer) v *= k;
    for (double& v : correction_.spin) v *= k;
}

double OwnPredictor::correctionMagnitudeM() const { return std::hypot(correction_.x, correction_.y); }

}  // namespace wpiemu::core::match
