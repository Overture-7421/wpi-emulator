#pragma once

// Reference state machines for the lock-step rules (messages.h, docs/ARCHITECTURE.md), shared by
// both ends so they cannot drift apart. Pure bookkeeping: no I/O, no clocks, no threads (call from
// the thread that owns the channel's receive side). Wall-clock watchdog timing stays with the
// caller, which tells these classes when it expired.
//
//   emulator: LockstepWindow   - one outstanding tick, validates ActuatorFrames, drops stale ones
//   extension: LockstepFollower - validates SensorFrames/SimControl, builds acks stamped with identity

#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <utility>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::sdk::proto {

enum class AckVerdict : std::uint8_t {
    Completed,       // tick done, window open again
    Dropped,         // stale epoch / foreign session: ignored, window state unchanged
    Stalled,         // extension reported RobotStalled: stop sending, surface to the UI
    Stopped,         // extension reported Stopped: session is over
    ProtocolError,   // unsolicited / wrong tick / extension ahead of the emulator: close the session
};

// Emulator side. Owns "what is in flight" for the current (sessionId, runEpoch).
class LockstepWindow {
public:
    // New connection or Reset: adopt the identity and discard any outstanding tick.
    void begin(std::uint64_t sessionId, std::uint64_t runEpoch) {
        session_ = sessionId;
        epoch_ = runEpoch;
        outstanding_ = false;
        stalled_ = false;
    }

    std::uint64_t sessionId() const { return session_; }
    std::uint64_t runEpoch() const { return epoch_; }
    bool canSend() const { return !outstanding_ && !stalled_; }
    bool outstanding() const { return outstanding_; }
    std::uint64_t outstandingTick() const { return tick_; }
    bool stalled() const { return stalled_; }
    std::uint64_t droppedFrames() const { return dropped_; }

    // Call right before transmitting `frame`; the frame must carry this window's identity.
    Status onSent(const SensorFrame& frame) {
        if (stalled_) return makeError(ErrorCode::InvalidState, "robot stalled; lock-step suspended");
        if (outstanding_) return makeError(ErrorCode::InvalidState, "lock-step window is one tick: ack tick " + std::to_string(tick_) + " first");
        if (classifyFrame(frame.sessionId, frame.runEpoch, session_, epoch_) != FrameDisposition::Accept)
            return makeError(ErrorCode::InvalidArgument, "SensorFrame identity does not match the window");
        outstanding_ = true;
        tick_ = frame.tick;
        return okStatus();
    }

    AckVerdict onAck(const ActuatorFrame& frame) {
        switch (classifyFrame(frame.sessionId, frame.runEpoch, session_, epoch_)) {
            case FrameDisposition::Accept: break;
            case FrameDisposition::DropStaleEpoch:
            case FrameDisposition::DropForeignSession: ++dropped_; return AckVerdict::Dropped;
            case FrameDisposition::NewerEpoch: return AckVerdict::ProtocolError;
        }
        if (!outstanding_ || frame.ackedTick() != tick_) return AckVerdict::ProtocolError;
        outstanding_ = false;
        switch (frame.result) {
            case TickResult::Completed: return AckVerdict::Completed;
            case TickResult::RobotStalled: stalled_ = true; return AckVerdict::Stalled;
            case TickResult::Stopped: return AckVerdict::Stopped;
            case TickResult::ProtocolError: return AckVerdict::ProtocolError;
        }
        return AckVerdict::ProtocolError;
    }

    // The caller's watchdog expired while a tick was outstanding.
    void onWatchdogExpired() {
        outstanding_ = false;
        stalled_ = true;
    }

private:
    std::uint64_t session_ = kUnspecified;
    std::uint64_t epoch_ = kUnspecified;
    std::uint64_t tick_ = 0;
    std::uint64_t dropped_ = 0;
    bool outstanding_ = false;
    bool stalled_ = false;
};

enum class FollowerAction : std::uint8_t {
    Step,            // run HALSIM_StepTiming for this frame, then answer with makeAck(..., Completed)
    Drop,            // stale / foreign: do nothing, send nothing
    ProtocolError,   // answer with makeAck(..., TickResult::ProtocolError) and do NOT step
};

struct ControlDecision {
    bool apply = false;     // perform the command now
    bool sendAck = false;   // transmit `ack` (false: foreign session, ignored silently)
    SimControlAck ack;
};

// Extension side (the peer that follows the emulator's clock).
class LockstepFollower {
public:
    // Adopt the identity from HelloAck (also used when the emulator reconnects).
    void begin(std::uint64_t sessionId, std::uint64_t runEpoch) {
        session_ = sessionId;
        epoch_ = runEpoch;
        haveTick_ = false;
        lastTick_ = 0;
        commandHighWater_ = kUnspecified;
        commandHistory_.clear();
    }

    std::uint64_t sessionId() const { return session_; }
    std::uint64_t runEpoch() const { return epoch_; }
    std::uint64_t lastCompletedTick() const { return lastTick_; }

    FollowerAction onSensorFrame(const SensorFrame& frame) {
        switch (classifyFrame(frame.sessionId, frame.runEpoch, session_, epoch_)) {
            case FrameDisposition::Accept: break;
            case FrameDisposition::DropStaleEpoch:
            case FrameDisposition::DropForeignSession: return FollowerAction::Drop;
            case FrameDisposition::NewerEpoch:
                // A frame of a newer epoch without the Reset that announces it: adopt, do not guess.
                epoch_ = frame.runEpoch;
                haveTick_ = false;
                break;
        }
        if (haveTick_ && frame.tick <= lastTick_) return FollowerAction::ProtocolError;
        return FollowerAction::Step;
    }

    // Stamps an ActuatorFrame answering `sensor`. Call with Completed only after the step finished.
    ActuatorFrame makeAck(const SensorFrame& sensor, TickResult result) {
        ActuatorFrame f;
        f.tick = sensor.tick;
        f.simTimeUs = sensor.simTimeUs;
        f.sessionId = session_;
        f.runEpoch = epoch_;
        f.ackTick = sensor.tick;
        f.result = result;
        if (result == TickResult::Completed) {
            haveTick_ = true;
            lastTick_ = sensor.tick;
        }
        return f;
    }

    ControlDecision onControl(const SimControl& c) {
        return onControl(c, [] { return std::pair{ControlResult::Applied, std::string{}}; });
    }

    // State-dependent checks run only AFTER identity validation and duplicate lookup. Their
    // rejections participate in the same replay history as applied commands.
    ControlDecision onControl(
        const SimControl& c,
        const std::function<std::pair<ControlResult, std::string>()>& precondition) {
        ControlDecision d;
        const FrameDisposition disp = classifyFrame(c.sessionId, c.runEpoch, session_, epoch_);
        if (disp == FrameDisposition::DropForeignSession) return d;
        d.sendAck = c.commandId != kUnspecified;
        if (d.sendAck) {
            for (const SimControlAck& ack : commandHistory_) {
                if (c.commandId == ack.commandId) {
                    d.ack = ack;
                    return d;
                }
            }
        }

        ControlResult result = ControlResult::Applied;
        std::string detail;
        if (d.sendAck && c.commandId <= commandHighWater_) {
            result = ControlResult::Rejected;
            detail = "commandId is outside the retained retry window; command was not reapplied";
        } else if (static_cast<std::uint8_t>(c.command) >
                   static_cast<std::uint8_t>(SimCommand::EndLockstep)) {
            result = ControlResult::Unsupported;
            detail = "unknown simulation command";
        } else {
            auto check = precondition();
            result = check.first;
            detail = std::move(check.second);
        }
        if (result != ControlResult::Applied) {
            // Do not change the epoch or tick bookkeeping for rejected commands.
        } else if (c.command == SimCommand::Stop) {
            d.apply = true;  // honoured whatever the epoch
        } else if (c.command == SimCommand::Reset || c.command == SimCommand::BeginLockstep ||
                   c.command == SimCommand::EndLockstep) {
            if (disp == FrameDisposition::NewerEpoch || c.runEpoch == kUnspecified || epoch_ == kUnspecified) {
                if (c.runEpoch != kUnspecified) epoch_ = c.runEpoch;
                haveTick_ = false;
                lastTick_ = 0;
                d.apply = true;
            } else {
                result = ControlResult::StaleEpoch;
                detail = "Reset, BeginLockstep and EndLockstep must carry a newer runEpoch";
            }
        } else if (disp == FrameDisposition::DropStaleEpoch) {
            result = ControlResult::StaleEpoch;
            detail = "command issued under an older runEpoch";
        } else if (disp == FrameDisposition::NewerEpoch) {
            result = ControlResult::Rejected;
            detail = "runEpoch is ahead of the one in force (missing Reset)";
        } else {
            d.apply = true;
        }

        d.ack.commandId = c.commandId;
        d.ack.command = c.command;
        d.ack.result = result;
        d.ack.sessionId = session_;
        d.ack.runEpoch = epoch_;
        d.ack.tick = lastTick_;
        d.ack.detail = std::move(detail);
        if (d.sendAck && c.commandId > commandHighWater_) {
            commandHighWater_ = c.commandId;
            commandHistory_.push_back(d.ack);
            if (commandHistory_.size() > 64) commandHistory_.pop_front();
        }
        return d;
    }

private:
    std::uint64_t session_ = kUnspecified;
    std::uint64_t epoch_ = kUnspecified;
    std::uint64_t lastTick_ = 0;
    std::uint64_t commandHighWater_ = kUnspecified;
    std::deque<SimControlAck> commandHistory_;
    bool haveTick_ = false;
};

}  // namespace wpiemu::sdk::proto
