#pragma once

// The robot-process (HALSim extension) end of the protocol, for tests of the emulator side. It owns the message-level
// behaviour of the real extension: Hello, answering SensorFrames through LockstepFollower (identity, ackTick, epoch),
// acknowledging SimControl, the lifecycle signal. It records what it receives.
//
// Nothing runs by itself: pump() handles what has arrived, so a test drives it in line, from a waitUntil pump or from a
// driver thread (the counters are atomic for that). Misbehaviour is configuration, not subclassing: never call hello(),
// send a version the emulator refuses, `reply` / `onFrame` to hold, stall or drop the connection, `ackControls` and
// `controlResult` to leave commands unanswered or refuse them, `leaveOnStop` to end the connection like the extension does.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/lifecycle.h"
#include "wpiemu/sdk/ipc/lockstep.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::sdk::testing {

class FakeExtension {
public:
    // What to do with a SensorFrame the follower accepts.
    enum class Reply {
        Answer,      // acknowledge with Completed
        Stall,       // acknowledge with RobotStalled
        Hold,        // keep it in `pending` unanswered until answer()
        Disconnect,  // close the connection, as when the robot process dies mid-tick
    };
    using FrameHook = std::function<Reply(const proto::SensorFrame&, FakeExtension&)>;
    using WallClock = std::function<std::chrono::steady_clock::time_point()>;

    FakeExtension(std::unique_ptr<proto::ITransport> transport, std::shared_ptr<const proto::IMessageCodec> codec)
        : ch(std::move(transport), std::move(codec)) {}

    bool send(const proto::Message& m) { return ch.send(m).ok(); }

    void hello(std::uint32_t proposedWatchdogMs = 0, std::uint32_t version = proto::kProtocolVersion,
               bool supportsArmed = true, std::int32_t wpilibYear = 2026) {
        proto::Hello h;
        h.protocolVersion = version;
        h.language = proto::RobotLanguage::Java;
        h.wpilibYear = wpilibYear;
        h.wpilibVersion = wpilibYear > 0 ? std::to_string(wpilibYear) + ".2.1" : "";
        h.pid = 4242;
        h.programName = "TestRobot";
        h.proposedWatchdogMs = proposedWatchdogMs;
        h.supportsArmed = supportsArmed;
        send(h);
    }
    // After the robot constructor: every device it built has been announced.
    void programStarted() { send(proto::makeLifecycleLog(proto::kLifecycleProgramStarted)); }

    // Handles every message that has arrived, waiting up to `wait` for the first. A peer that closes ends the
    // connection (`disconnected`); an unknown message type is skipped.
    void pump(std::chrono::milliseconds wait = std::chrono::milliseconds{0}) {
        std::chrono::milliseconds timeout = wait;
        while (!disconnected) {
            proto::Message m;
            const Result<bool> got = ch.receive(m, timeout);
            timeout = std::chrono::milliseconds{0};
            if (!got) {
                if (got.error().code == ErrorCode::Unsupported) continue;
                disconnected = true;
                break;
            }
            if (!got.value()) break;
            if (clock) lastRxAt = clock();
            handle(m);
        }
    }

    // Answers `sf` as the extension would after stepping it; a frame the follower refuses is dropped or reported as
    // a ProtocolError exactly like one arriving through pump().
    void answer(const proto::SensorFrame& sf, proto::TickResult result = proto::TickResult::Completed) {
        if (admit(sf)) respond(sf, result);
    }

    bool handshaken() const { return helloAck.has_value() && helloAck->accepted; }

    // Mirrors the extension's watchdog rule (sim_session.cpp): armed only while not paused and no tick is in flight.
    bool watchdogWouldFire(std::chrono::milliseconds timeout, std::chrono::steady_clock::time_point now) const {
        return !remotePaused && !pending.has_value() && now - lastRxAt > timeout;
    }

    proto::MessageChannel ch;
    proto::LockstepFollower follower;

    Reply reply = Reply::Answer;
    FrameHook onFrame;  // when set, decides per accepted frame instead of `reply`
    bool ackControls = true;
    proto::ControlResult controlResult = proto::ControlResult::Applied;
    bool leaveOnStop = false;  // an applied Stop, or Goodbye, closes the connection after the ack
    std::vector<SignalSample> outputs;  // the samples every answer carries
    WallClock clock;  // stamps lastRxAt on every message; unset leaves it alone

    bool disconnected = false;  // the connection is over, closed by the peer or by this fake
    bool remotePaused = false;
    std::chrono::steady_clock::time_point lastRxAt{};
    std::optional<proto::HelloAck> helloAck;
    std::optional<proto::SensorFrame> pending;  // the unanswered frame of Reply::Hold
    std::vector<proto::SensorFrame> sensorFrames;
    std::vector<proto::SimControl> controls;
    std::vector<proto::DsState> dsStates;
    std::vector<proto::LoopbackSet> loopbacks;
    std::vector<VisionSetup> visionSetups;
    std::vector<std::string> trace;  // "ack", "vision", "frame" in arrival order

    std::atomic<int> stepped{0};        // answers sent, whatever their result
    std::atomic<int> droppedFrames{0};  // SensorFrames of a stale epoch or a foreign session
    std::atomic<int> acceptedDs{0};     // DsState carrying the identity in force
    std::atomic<int> droppedDs{0};      // DsState of another epoch or session

private:
    void handle(const proto::Message& m) {
        if (const auto* ack = std::get_if<proto::HelloAck>(&m)) {
            helloAck = *ack;
            trace.push_back("ack");
            if (ack->accepted) follower.begin(ack->sessionId, ack->runEpoch);
        } else if (const auto* sf = std::get_if<proto::SensorFrame>(&m)) {
            handleFrame(*sf);
        } else if (const auto* vs = std::get_if<VisionSetup>(&m)) {
            visionSetups.push_back(*vs);
            trace.push_back("vision");
        } else if (const auto* sc = std::get_if<proto::SimControl>(&m)) {
            handleControl(*sc);
        } else if (const auto* ds = std::get_if<proto::DsState>(&m)) {
            dsStates.push_back(*ds);
            if (proto::classifyFrame(ds->sessionId, ds->runEpoch, follower.sessionId(), follower.runEpoch()) ==
                proto::FrameDisposition::Accept)
                ++acceptedDs;
            else
                ++droppedDs;
        } else if (const auto* lb = std::get_if<proto::LoopbackSet>(&m)) {
            loopbacks.push_back(*lb);
        } else if (std::holds_alternative<proto::Goodbye>(m) && leaveOnStop) {
            leave();
        }
    }

    void handleFrame(const proto::SensorFrame& sf) {
        sensorFrames.push_back(sf);
        trace.push_back("frame");
        if (!admit(sf)) return;
        switch (onFrame ? onFrame(sf, *this) : reply) {
            case Reply::Answer: respond(sf, proto::TickResult::Completed); break;
            case Reply::Stall: respond(sf, proto::TickResult::RobotStalled); break;
            case Reply::Hold: pending = sf; break;
            case Reply::Disconnect: leave(); break;
        }
    }

    void handleControl(const proto::SimControl& sc) {
        controls.push_back(sc);
        const proto::ControlDecision d = follower.onControl(sc, [&] {
            return std::pair{controlResult, controlResult == proto::ControlResult::Applied
                                                ? std::string{}
                                                : std::string("scripted control result")};
        });
        if (d.apply && sc.command == proto::SimCommand::Pause) remotePaused = true;
        if (d.apply && sc.command == proto::SimCommand::Resume) remotePaused = false;
        if (d.sendAck && ackControls) send(d.ack);
        if (leaveOnStop && d.apply && sc.command == proto::SimCommand::Stop) leave();
    }

    // The follower's verdict on `sf`: true when it is to be stepped. A stale or foreign frame is dropped silently, an
    // out-of-order one is refused with a ProtocolError and not stepped.
    bool admit(const proto::SensorFrame& sf) {
        switch (follower.onSensorFrame(sf)) {
            case proto::FollowerAction::Step: return true;
            case proto::FollowerAction::Drop: ++droppedFrames; return false;
            case proto::FollowerAction::ProtocolError:
                send(follower.makeAck(sf, proto::TickResult::ProtocolError));
                return false;
        }
        return false;
    }

    void respond(const proto::SensorFrame& sf, proto::TickResult result) {
        proto::ActuatorFrame a = follower.makeAck(sf, result);
        a.samples = outputs;
        ++stepped;
        send(a);
    }

    void leave() {
        ch.close();
        disconnected = true;
    }
};

}  // namespace wpiemu::sdk::testing
