#pragma once

// Simulation control: the SimControl commands the emulator sends and the SimControlAck that answers them.
// Plain structs: the codec in ipc/ decides the serialisation. See messages.h for the flow.

#include <cstdint>
#include <string>

#include "wpiemu/sdk/ipc/protocol_rules.h"

namespace wpiemu::sdk::proto {

enum class SimCommand : std::uint8_t {
    Pause = 0,
    Resume = 1,
    Step = 2,       // advance `stepCount` ticks then stay paused
    Reset = 3,      // reset physics/game state; robot keeps running (bumps runEpoch, MUST be acked)
    Stop = 4,       // end the session (extension resumes timing and exits cleanly; MUST be acked)
    SetTimeScale = 5,
    // One long-lived robot process. A session whose HelloAck said `discoveryOnly` is ARMED: devices are
    // announced and HAL time runs free. BeginLockstep switches it to lock-step; EndLockstep returns it to armed.
    // Both follow the Reset rules (first message of a NEW epoch, MUST carry a commandId and be acked); the
    // extension answers ControlResult::Rejected when the transition is impossible (already there, program not
    // started, real-time clock preference) and ::Unsupported when the peer does not support it.
    BeginLockstep = 6,
    EndLockstep = 7,
};

// Receiver rules:
//  * commandId != 0: the receiver answers with exactly one SimControlAck carrying the same
//  commandId.
//    Reset and Stop MUST carry a non-zero commandId. IDs increase within a connection, across
//    epochs. The last 64 distinct command results (including rejections) are retained and replayed
//    exactly. Older/evicted IDs are rejected, NEVER re-applied. Reconnect starts fresh command
//    history. Stop terminates the connection: retries are only meaningful while that connection is
//    still live.
//  * runEpoch: for every command except Reset, the sender's epoch in force; a stale one is rejected
//    (ControlResult::StaleEpoch). Reset is the FIRST message of the NEW epoch: its runEpoch is the
//    already-bumped value and must be newer than the receiver's. Stop is honoured regardless of
//    epoch.
struct SimControl {
    SimCommand command = SimCommand::Pause;
    std::uint32_t stepCount = 1;
    double timeScale = 1.0;
    // ----
    std::uint64_t commandId = kUnspecified;   // 0 = fire-and-forget (no ack); sender-unique, increasing
    std::uint64_t sessionId = kUnspecified;
    std::uint64_t runEpoch = kUnspecified;
};

enum class ControlResult : std::uint8_t {
    Applied = 0,
    Rejected = 1,      // invalid in the current state (e.g. Step while running)
    StaleEpoch = 2,    // command carried an older runEpoch; ignored
    Unsupported = 3,   // peer does not implement the command
};

// Receiver -> sender acknowledgement of a SimControl with a non-zero commandId.
struct SimControlAck {
    std::uint64_t commandId = kUnspecified;
    SimCommand command = SimCommand::Pause;
    ControlResult result = ControlResult::Applied;
    std::uint64_t sessionId = kUnspecified;
    std::uint64_t runEpoch = kUnspecified;    // epoch in force AFTER the command (the new one for Reset)
    std::uint64_t tick = 0;                   // last tick the receiver completed
    std::string detail;                       // human readable when result != Applied
};

}  // namespace wpiemu::sdk::proto
