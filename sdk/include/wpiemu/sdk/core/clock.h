#pragma once

#include <chrono>
#include <cstdint>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::sdk {

// Read-only view of the simulation clock. The emulator is the clock master (docs/reference/extension-ipc.md): time only
// advances when the sim loop steps it, in fixed increments. Everything that needs "time" reads it
// here; nothing calls a wall clock (determinism).
class ISimClock {
    WPIEMU_INTERFACE(ISimClock)
public:
    virtual SimTimeUs simTimeUs() const = 0;     // monotonic since session start (0 after reset)
    virtual std::uint64_t tickIndex() const = 0;  // number of completed sim ticks
    virtual SimTimeUs fixedStepUs() const = 0;    // dt of one tick (default 5000 = 200 Hz)

    double simTimeSeconds() const { return usToSeconds(simTimeUs()); }
    double fixedStepSeconds() const { return usToSeconds(fixedStepUs()); }
};

// Monotonic wall clock for pacing, watchdogs and network timing; injectable so those are testable without sleeping.
// Simulation logic never reads it (ISimClock above is the simulation's time).
class IWallClock {
    WPIEMU_INTERFACE(IWallClock)
public:
    virtual std::chrono::steady_clock::time_point now() const = 0;
};

class SteadyWallClock final : public IWallClock {
public:
    std::chrono::steady_clock::time_point now() const override { return std::chrono::steady_clock::now(); }
};

}  // namespace wpiemu::sdk
