#pragma once

// Small timing helpers of the game client, header only.
//
//   TimelineTracker   a timeline (prediction ticks, the render time of remote robots) that follows a moving target without
//                     jumps: it advances at the real rate, sped up or slowed down by a bounded fraction to close the gap, and
//                     only snaps when the gap is hopeless.
//   SnapshotCadence   what the arrival of the host's snapshots says about the path: their interval, their jitter (RFC 3550
//                     style, against the sim-time spacing), the loss, and from them the interpolation delay
//                       clamp(2 x interval + 3 x jitter, 2 x interval, kMaxInterpolationDelayUs).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>

#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::core::match {

class TimelineTracker {
public:
    struct Config {
        double horizon = 100.0;        // the gap that asks for the full rate change (same unit as the value)
        double maxRateDelta = 0.05;    // the rate stays within 1 +- this
        double snapThreshold = 200.0;  // a gap beyond this is a jump
    };

    TimelineTracker() : config_(Config{}) {}
    explicit TimelineTracker(Config config) : config_(config) {}

    void reset(double value) {
        value_ = value;
        started_ = true;
    }
    bool started() const { return started_; }
    double value() const { return value_; }

    // Advances by `dt` (unit of the value, at the real rate) toward `target`. True when it jumped.
    bool advance(double dt, double target) {
        if (!started_) {
            reset(target);
            return true;
        }
        const double gap = target - value_;
        if (std::fabs(gap) > config_.snapThreshold) {
            value_ = target;
            return true;
        }
        const double rate = 1.0 + std::clamp(gap / config_.horizon, -config_.maxRateDelta, config_.maxRateDelta);
        value_ += dt * rate;
        return false;
    }

private:
    Config config_;
    double value_ = 0.0;
    bool started_ = false;
};

class SnapshotCadence {
public:
    static constexpr std::size_t kLossWindow = 128;

    void reset() { *this = SnapshotCadence{}; }

    // A snapshot newer than the last one arrived at client time `arrivalUs`; `simTimeUs` and `seq` are its own.
    void onSnapshot(std::int64_t arrivalUs, std::int64_t simTimeUs, std::uint32_t seq) {
        if (count_ > 0) {
            const std::int64_t arrivalGap = arrivalUs - lastArrivalUs_;
            const std::int64_t simGap = simTimeUs - lastSimUs_;
            const std::uint32_t seqGap = seq - lastSeq_;
            if (simGap > 0 && seqGap > 0 && seqGap < 64) {
                const double perSeq = static_cast<double>(simGap) / static_cast<double>(seqGap);
                intervalUs_ = count_ == 1 ? perSeq : intervalUs_ + (perSeq - intervalUs_) / 8.0;
                const double deviation = std::fabs(static_cast<double>(arrivalGap - simGap));
                jitterUs_ += (deviation - jitterUs_) / 16.0;
            }
        }
        lastArrivalUs_ = arrivalUs;
        lastSimUs_ = simTimeUs;
        lastSeq_ = seq;
        window_.push_back(seq);
        if (window_.size() > kLossWindow) window_.pop_front();
        ++count_;
    }

    std::size_t count() const { return count_; }
    std::int64_t intervalUs() const { return static_cast<std::int64_t>(std::llround(intervalUs_)); }
    std::int64_t jitterUs() const { return static_cast<std::int64_t>(std::llround(jitterUs_)); }
    double rateHz() const { return intervalUs_ > 0.0 ? 1e6 / intervalUs_ : 0.0; }
    std::int64_t lastArrivalUs() const { return lastArrivalUs_; }

    // Fraction of the snapshots of the window that never arrived, in parts per thousand.
    double lossPermille() const {
        if (window_.size() < 2) return 0.0;
        const std::uint32_t span = window_.back() - window_.front() + 1;
        if (span == 0 || span > 1000) return 0.0;
        const double lost = static_cast<double>(span) - static_cast<double>(window_.size());
        return std::max(0.0, 1000.0 * lost / static_cast<double>(span));
    }

    // The delay (behind the host's newest sim time) at which remote robots are drawn. Zero until two snapshots arrived.
    std::int64_t targetDelayUs(std::int64_t maxUs = sdk::match::kMaxInterpolationDelayUs) const {
        if (count_ < 2) return 0;
        const std::int64_t interval = intervalUs();
        const std::int64_t wanted = 2 * interval + 3 * jitterUs();
        return std::clamp(wanted, std::min(2 * interval, maxUs), maxUs);
    }

private:
    double intervalUs_ = 0.0;
    double jitterUs_ = 0.0;
    std::int64_t lastArrivalUs_ = 0;
    std::int64_t lastSimUs_ = 0;
    std::uint32_t lastSeq_ = 0;
    std::size_t count_ = 0;
    std::deque<std::uint32_t> window_;
};

}  // namespace wpiemu::core::match
