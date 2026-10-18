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
    // Vision: all zero while no SensorFrame carries a camera frame.
    std::uint64_t visionFramesApplied = 0;  // VisionFrames handed to the bridges (accepted frames)
    std::uint64_t visionApplyCalls = 0;     // applyVision fan-outs, i.e. ticks with camera frames
    std::uint64_t visionApplyTotalUs = 0;   // wall time spent in those fan-outs ("publish" cost)
    std::uint64_t visionApplyMaxUs = 0;     // the slowest single fan-out
};

struct AtomicRuntimeStats {
    std::atomic<std::uint64_t> sessions{0};
    std::atomic<std::uint64_t> ticksCompleted{0};
    std::atomic<std::uint64_t> framesDropped{0};
    std::atomic<std::uint64_t> protocolErrors{0};
    std::atomic<std::uint64_t> stalls{0};
    std::atomic<std::uint64_t> emulatorLost{0};
    std::atomic<std::uint64_t> announces{0};
    std::atomic<std::uint64_t> visionFramesApplied{0};
    std::atomic<std::uint64_t> visionApplyCalls{0};
    std::atomic<std::uint64_t> visionApplyTotalUs{0};
    std::atomic<std::uint64_t> visionApplyMaxUs{0};

    // One applyVision fan-out that handed `frames` camera frames to the bridges and took `us`.
    void recordVisionApply(std::uint64_t frames, std::uint64_t us) {
        visionFramesApplied.fetch_add(frames, std::memory_order_relaxed);
        visionApplyCalls.fetch_add(1, std::memory_order_relaxed);
        visionApplyTotalUs.fetch_add(us, std::memory_order_relaxed);
        std::uint64_t seen = visionApplyMaxUs.load(std::memory_order_relaxed);
        while (us > seen &&
               !visionApplyMaxUs.compare_exchange_weak(seen, us, std::memory_order_relaxed)) {
        }
    }

    RuntimeStats snapshot() const {
        return RuntimeStats{sessions.load(),         ticksCompleted.load(),
                            framesDropped.load(),    protocolErrors.load(),
                            stalls.load(),           emulatorLost.load(),
                            announces.load(),        visionFramesApplied.load(),
                            visionApplyCalls.load(), visionApplyTotalUs.load(),
                            visionApplyMaxUs.load()};
    }
};

}  // namespace wpiemu::halsim
