#pragma once

#include <atomic>
#include <cstdint>

namespace wpiemu::halsim {

// Plain copy of the counters, for tests and diagnostics.
struct RuntimeStats {
    std::uint64_t sessions = 0;          // accepted handshakes (1 + reconnects)
    std::uint64_t ticksCompleted = 0;    // SensorFrames answered with TickResult::Completed
    std::uint64_t framesDropped = 0;     // stale epoch / foreign session
    std::uint64_t protocolErrors = 0;    // out-of-order or duplicated frames
    std::uint64_t stalls = 0;            // robot did not quiesce within the watchdog
    std::uint64_t emulatorLost = 0;      // no traffic from the emulator for a whole watchdog period
    std::uint64_t announces = 0;         // DeviceAnnounce messages sent
};

struct AtomicRuntimeStats {
    std::atomic<std::uint64_t> sessions{0};
    std::atomic<std::uint64_t> ticksCompleted{0};
    std::atomic<std::uint64_t> framesDropped{0};
    std::atomic<std::uint64_t> protocolErrors{0};
    std::atomic<std::uint64_t> stalls{0};
    std::atomic<std::uint64_t> emulatorLost{0};
    std::atomic<std::uint64_t> announces{0};

    RuntimeStats snapshot() const {
        return RuntimeStats{sessions.load(),       ticksCompleted.load(), framesDropped.load(), protocolErrors.load(),
                            stalls.load(),         emulatorLost.load(),   announces.load()};
    }
};

}  // namespace wpiemu::halsim
