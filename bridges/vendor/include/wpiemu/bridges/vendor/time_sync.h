#pragma once

// Mitigation for ADR 0002 R3: the CTRE firmware simulation (CTRE_PhoenixTools_Sim) integrates ramp rates,
// PID derivative terms and Motion Magic with its OWN wall clock, not with HAL sim time. When the emulator
// pauses or steps HALSIM time faster/slower than real time, the Phoenix controllers see a different elapsed
// time than the robot loop and the response diverges (a 5x faster sim makes ramps/profiles effectively 5x
// slower relative to the robot, a paused sim lets the profile run on).
//
// What we do about it (all optional, injected, no globals):
//  * The vendor bridge reports a TimeSyncAdvice (ITimeSyncAdvisor): "while Phoenix devices are live, pace
//    lock-step ticks to the wall clock at <= maxRealtimeFactor".
//  * RealtimePacer is the small reusable helper the extension's clock loop (or any caller) uses to follow the
//    advice: it returns how long to sleep before releasing the next tick. Clock and sleep are injectable.
//  * The advice is a hint: the emulator can still run uncapped (e.g. headless regression tests with
//    Phoenix devices unmapped/looped back). Known residual error: with real-time pacing, Phoenix sees the
//    wall time between ticks (~step length) while the robot sees HAL time; both agree on average.

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>

#include "wpiemu/sdk/core/interface.h"

namespace wpiemu::bridges::vendor {

enum class TimeSyncPolicy {
    Advise,  // default: expose advice while wall-clock-driven vendor firmware sims are live
    Ignore,  // never advise (caller knows better, e.g. no Phoenix closed-loop in the robot code)
};

struct TimeSyncAdvice {
    bool paceToWallClock = false;
    double maxRealtimeFactor = 1.0;  // sim time may advance at most this many times faster than the wall clock
    bool pauseUnsafe = false;        // a long pause lets the firmware sim run ahead of the robot
    std::string reason;
};

class ITimeSyncAdvisor {
    WPIEMU_INTERFACE(ITimeSyncAdvisor)
public:
    virtual TimeSyncAdvice timeSyncAdvice() const = 0;
};

// Keeps simulated time from running ahead of `factor` x wall time. Pure arithmetic over an injectable clock.
class RealtimePacer {
public:
    using NowUs = std::function<std::int64_t()>;  // monotonic microseconds

    explicit RealtimePacer(double factor = 1.0, NowUs now = defaultNow())
        : factor_(factor > 0.0 ? factor : 1.0), now_(std::move(now)) {}

    void setFactor(double factor) { factor_ = factor > 0.0 ? factor : 1.0; }
    double factor() const { return factor_; }

    // Call when the first tick is released (and after a pause/reset): re-anchors the schedule.
    void restart() {
        started_ = false;
        simUs_ = 0;
    }

    // `stepUs` = simulated time the NEXT tick will advance. Returns microseconds to sleep (>= 0) before releasing
    // it so that simulated time never exceeds factor * wall time. Falls behind gracefully: never negative, and
    // re-anchors when more than `maxLagUs` behind so a stall is not "caught up" with a burst of fast ticks.
    std::int64_t microsToWait(std::int64_t stepUs, std::int64_t maxLagUs = 250'000) {
        const std::int64_t now = now_();
        if (!started_) {
            started_ = true;
            anchorWallUs_ = now;
            simUs_ = 0;
        }
        const double targetWallUs = static_cast<double>(simUs_ + stepUs) / factor_;
        const std::int64_t due = anchorWallUs_ + static_cast<std::int64_t>(targetWallUs);
        simUs_ += stepUs;
        if (due >= now) return due - now;
        if (now - due > maxLagUs) {  // stalled: do not burst
            anchorWallUs_ = now - static_cast<std::int64_t>(static_cast<double>(simUs_) / factor_);
        }
        return 0;
    }

private:
    static NowUs defaultNow() {
        return [] {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
        };
    }

    double factor_;
    NowUs now_;
    bool started_ = false;
    std::int64_t anchorWallUs_ = 0;
    std::int64_t simUs_ = 0;
};

}  // namespace wpiemu::bridges::vendor
