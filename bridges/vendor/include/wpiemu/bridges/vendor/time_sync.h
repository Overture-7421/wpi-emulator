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
//  * The advice is a hint: the emulator can still run uncapped (e.g. headless regression tests with
//    Phoenix devices unmapped/looped back). Known residual error: with real-time pacing, Phoenix sees the
//    wall time between ticks (~step length) while the robot sees HAL time; both agree on average.

#include <string>

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

}  // namespace wpiemu::bridges::vendor
