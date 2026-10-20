#pragma once

// Write-policy decorator over an ISimDeviceBackend (SOLID: the SDK SimDeviceBridge stays untouched; vendor
// quirks live here). It implements three documented behaviours:
//
//  1. Integrated raw inputs (Phoenix 6 `rawYawInput`, `rawPositionInput`): the CTRE firmware sim INTEGRATES
//     changes of these values into the reported yaw/position, so the emulator must feed a CONTINUOUS value
//     and write it exactly ONCE per tick. Guarded values are therefore coalesced (last write of a batch wins),
//     angle-unwrapped when requested, skipped when unchanged, and verified by read-back.
//  2. Disturbance handling: if the value read back at the next tick differs from what we wrote (Phoenix's own
//     periodic mirror, or another writer such as a robot-side simulation), it is counted, re-asserted
//     (reassertAfterMirror) and, when persistent, reported once as a foreign write (ADR 0002 R4/R9).
//  3. Robot-initiated resets (`setYaw`, `setPosition`) are NOT fought: the firmware keeps them as an offset on
//     top of the integrated raw value, we only ever write absolute continuous physical truth.
//
// Threading: setValue/beginWrites/endWrites run on the extension clock thread; reassertAfterMirror may run on
// the robot thread from a SimPeriodicBefore callback (the clock thread is quiescent then). All state is
// mutex-protected.

#include <cstdint>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/devices/sim_device_bridge.h"

namespace wpiemu::bridges::vendor {

struct GuardedValueSpec {
    std::string deviceNameRegex;  // ECMAScript, full match over the SimDevice name
    std::string valueName;        // HAL value name, e.g. "rawYawInput"
    bool unwrapDegrees = false;   // treat the value as an angle in degrees and make it continuous
};

struct GuardOptions {
    std::vector<GuardedValueSpec> guarded;
    double absTolerance = 1e-4;        // read-back equality: |a-b| <= abs + rel*|a|
    double relTolerance = 1e-6;
    int foreignWriteThreshold = 5;     // consecutive mismatching read-backs before a foreign writer is reported
};

struct InputDisturbance {
    std::string device;
    std::string value;
    std::uint64_t mismatches = 0;        // total read-back mismatches
    std::uint64_t reasserts = 0;         // times a guarded value was restored after the mirror
    bool foreignWriter = false;          // persistent mismatch: someone else keeps writing this input
    double lastWritten = 0.0;
    double lastReadback = 0.0;
};

class GuardedSimDeviceBackend final : public sdk::ISimDeviceBackend {
public:
    GuardedSimDeviceBackend(sdk::ISimDeviceBackend& inner, GuardOptions options);

    void setLogger(sdk::ILogger* logger);

    // ISimDeviceBackend
    std::uint64_t generation() const override { return inner_->generation(); }
    void listDevices(std::vector<sdk::SimDeviceInfo>& out) const override;
    void listValues(sdk::SimDeviceHandle device, std::vector<sdk::SimValueInfo>& out) const override;
    double getValue(sdk::SimValueHandle value) const override { return inner_->getValue(value); }
    void setValue(sdk::SimValueHandle value, double v) override;

    // Brackets one tick of input writes: guarded values are held back and flushed once in endWrites().
    void beginWrites();
    void endWrites();

    // Call right after the CTRE wrapper's own SimPeriodicBefore mirror ran (robot thread): restores every
    // guarded value whose read-back no longer matches what we wrote. Returns how many were restored.
    std::size_t reassertAfterMirror();

    // Forget write history and angle-unwrapping state (emulator Reset / new run epoch).
    void resetHistory();

    std::vector<InputDisturbance> disturbances() const;

private:
    struct Meta {
        std::string device;
        std::string value;
        int guardIndex = -1;
    };
    struct State {
        bool hasWritten = false;
        double lastWritten = 0.0;
        bool hasIncoming = false;
        double lastIncoming = 0.0;
        double continuous = 0.0;   // unwrapped value (guarded angles)
        bool pending = false;
        double pendingValue = 0.0;
        std::uint64_t mismatches = 0;
        std::uint64_t reasserts = 0;
        int streak = 0;            // mismatch detections since the input was last seen intact
        bool reasserted = false;   // a mirror re-assert happened since the last read-back check
        bool foreign = false;
        double lastReadback = 0.0;
    };
    struct Guard {
        GuardedValueSpec spec;
        std::regex deviceRe;
        bool valid = true;
    };

    bool close(double a, double b) const;
    bool readbackMatches(const Meta& meta, double readback, double written) const;
    double unwrapped(State& s, const Guard& g, double incoming) const;
    void commit(sdk::SimValueHandle h, const Meta& meta, State& s, double target, bool force);
    bool verifyReadback(sdk::SimValueHandle h, const Meta& meta, State& s);
    void noteMismatch(const Meta& meta, State& s);
    void report(const Meta& meta, const State& s);

    sdk::ISimDeviceBackend* inner_;
    GuardOptions options_;
    std::vector<Guard> guards_;
    sdk::ILogger* logger_ = nullptr;

    mutable std::mutex mutex_;
    mutable std::unordered_map<sdk::SimDeviceHandle, std::string> deviceNames_;
    mutable std::unordered_map<sdk::SimValueHandle, Meta> meta_;
    std::unordered_map<sdk::SimValueHandle, State> state_;
    std::vector<sdk::SimValueHandle> pending_;
    bool inBatch_ = false;
};

}  // namespace wpiemu::bridges::vendor
