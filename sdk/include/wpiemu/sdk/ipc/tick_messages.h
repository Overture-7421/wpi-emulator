#pragma once

// Per-tick messages: the SensorFrame the emulator sends and the ActuatorFrame that answers it (one tick in flight,
// protocol_rules.h). Plain structs: the codec in ipc/ decides the serialisation. See messages.h for the flow.

#include <cstdint>
#include <vector>

#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/signal.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/vision/camera_frame.h"

namespace wpiemu::sdk::proto {

// app -> ext: inputs the robot will see during this step.
struct SensorFrame {
    std::uint64_t tick = 0;
    SimTimeUs simTimeUs = 0;             // time AFTER this step completes
    std::uint32_t stepUs = static_cast<std::uint32_t>(kDefaultStepUs);   // HALSIM_StepTiming delta
    double batteryVoltageV = kNominalBatteryVoltageV;   // written to the roboRIO VIn + every device's supply voltage
    double batteryCurrentA = 0.0;        // informational (roboRIO current)
    bool full = false;                   // true: contains every Input signal; false: only changed ones
    std::vector<SignalSample> samples;
    // Camera frames due this tick (CameraFrame::publishTimeUs <= simTimeUs), see vision/camera_frame.h.
    std::vector<CameraFrame> vision;
    // identity of the run this frame belongs to (see classifyFrame).
    std::uint64_t sessionId = kUnspecified;
    std::uint64_t runEpoch = kUnspecified;
};

// How a tick ended, reported by the extension in ActuatorFrame::result. Appended values need a `default` path.
enum class TickResult : std::uint8_t {
    Completed = 0,       // HALSIM_StepTiming returned and outputs were collected
    RobotStalled = 1,    // the robot did not quiesce within the watchdog; `samples` are empty/stale
    Stopped = 2,         // the extension honoured Stop (or the watchdog policy) and is leaving lock-step
    ProtocolError = 3,   // the SensorFrame was out of order/duplicated/unexpected; nothing was stepped
};

// ext -> app: outputs the robot produced (observed after the step).
struct ActuatorFrame {
    std::uint64_t tick = 0;
    SimTimeUs simTimeUs = 0;
    std::uint64_t robotLoopCount = 0;    // periodic iterations completed (diagnostics)
    bool full = false;                   // see SensorFrame::full
    std::vector<SignalSample> samples;
    // ---- absent in a frame that does not send them => defaults: Completed, ack == tick, unspecified identity ----
    std::uint64_t sessionId = kUnspecified;   // echo of the SensorFrame being answered
    std::uint64_t runEpoch = kUnspecified;    // echo of the SensorFrame being answered
    std::uint64_t ackTick = kNoAckTick;       // tick of the SensorFrame this answers; unset => `tick`
    TickResult result = TickResult::Completed;

    std::uint64_t ackedTick() const { return ackTick != kNoAckTick ? ackTick : tick; }
};

}  // namespace wpiemu::sdk::proto
