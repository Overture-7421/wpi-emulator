#pragma once

#include <cstdint>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::sdk {

// Read-only view of the simulation clock. The emulator is the clock master (ADR 0001): time only
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

}  // namespace wpiemu::sdk
