#include <doctest/doctest.h>

#include <chrono>
#include <memory>

#include "wpiemu/sdk/ipc/lockstep.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu::sdk;
using namespace std::chrono_literals;

namespace {

proto::SensorFrame sensor(std::uint64_t tick, std::uint64_t session, std::uint64_t epoch) {
    proto::SensorFrame f;
    f.tick = tick;
    f.sessionId = session;
    f.runEpoch = epoch;
    return f;
}

proto::ActuatorFrame actuator(std::uint64_t ackTick, std::uint64_t session, std::uint64_t epoch,
                              proto::TickResult result = proto::TickResult::Completed) {
    proto::ActuatorFrame f;
    f.tick = ackTick;
    f.ackTick = ackTick;
    f.sessionId = session;
    f.runEpoch = epoch;
    f.result = result;
    return f;
}

}  // namespace

TEST_CASE("v2 defaults keep v1 frames meaningful") {
    const proto::ActuatorFrame legacy;  // what a v1 codec produces for the absent trailing fields
    CHECK(legacy.result == proto::TickResult::Completed);
    CHECK(legacy.runEpoch == proto::kUnspecified);
    proto::ActuatorFrame f;
    f.tick = 9;
    CHECK(f.ackedTick() == 9);  // ackTick unset => the frame answers `tick`
    f.ackTick = 7;
    CHECK(f.ackedTick() == 7);

    const proto::HelloAck ack;
    CHECK(ack.runEpoch == proto::kFirstRunEpoch);
    CHECK(ack.watchdogTimeoutMs == proto::kDefaultWatchdogTimeoutMs);
    CHECK(ack.watchdogPolicy == proto::WatchdogPolicy::ResumeRealtime);
    CHECK(proto::kLockstepWindow == 1);
    CHECK(proto::kProtocolVersion == 3);
    CHECK(proto::kMinCompatibleProtocolVersion == 1);
    CHECK_FALSE(proto::Hello{}.supportsArmed);
}

TEST_CASE("watchdog negotiation takes the slower side and clamps") {
    using proto::negotiateWatchdogTimeoutMs;
    CHECK(negotiateWatchdogTimeoutMs(0, 0) == proto::kDefaultWatchdogTimeoutMs);
    CHECK(negotiateWatchdogTimeoutMs(5000, 2000) == 5000);
    CHECK(negotiateWatchdogTimeoutMs(1000, 3000) == 3000);
    CHECK(negotiateWatchdogTimeoutMs(1, 0) == proto::kMinWatchdogTimeoutMs);
    CHECK(negotiateWatchdogTimeoutMs(10'000'000, 0) == proto::kMaxWatchdogTimeoutMs);
}

TEST_CASE("frames with a stale epoch or a foreign session are dropped") {
    using proto::classifyFrame;
    using proto::FrameDisposition;
    CHECK(classifyFrame(5, 3, 5, 3) == FrameDisposition::Accept);
    CHECK(classifyFrame(5, 2, 5, 3) == FrameDisposition::DropStaleEpoch);
    CHECK(classifyFrame(5, 4, 5, 3) == FrameDisposition::NewerEpoch);
    CHECK(classifyFrame(6, 3, 5, 3) == FrameDisposition::DropForeignSession);
    // unspecified identity (v1 peer / in-process caller) is always accepted
    CHECK(classifyFrame(0, 0, 5, 3) == FrameDisposition::Accept);
    CHECK(classifyFrame(5, 3, 0, 0) == FrameDisposition::Accept);
}

TEST_CASE("lock-step window allows exactly one outstanding tick") {
    proto::LockstepWindow w;
    w.begin(7, 1);
    CHECK(w.canSend());
    REQUIRE(w.onSent(sensor(1, 7, 1)).ok());
    CHECK_FALSE(w.canSend());
    CHECK(w.onSent(sensor(2, 7, 1)).error().code == ErrorCode::InvalidState);  // no pipelining

    CHECK(w.onAck(actuator(1, 7, 1)) == proto::AckVerdict::Completed);
    CHECK(w.canSend());
    REQUIRE(w.onSent(sensor(2, 7, 1)).ok());

    // unsolicited / wrong tick is a protocol error and does not free the window
    CHECK(w.onAck(actuator(5, 7, 1)) == proto::AckVerdict::ProtocolError);
    CHECK(w.outstanding());
    // a frame stamped with another identity cannot be sent
    w.begin(7, 2);
    CHECK(w.onSent(sensor(1, 7, 1)).error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("a late ActuatorFrame from before a reset is dropped, not applied") {
    proto::LockstepWindow w;
    w.begin(7, 1);
    REQUIRE(w.onSent(sensor(10, 7, 1)).ok());

    w.begin(7, 2);  // Reset: new epoch, the in-flight tick is abandoned
    REQUIRE(w.canSend());
    REQUIRE(w.onSent(sensor(0, 7, 2)).ok());

    CHECK(w.onAck(actuator(10, 7, 1)) == proto::AckVerdict::Dropped);  // stale epoch
    CHECK(w.droppedFrames() == 1);
    CHECK(w.outstanding());  // still waiting for the real answer to tick 0 of epoch 2
    CHECK(w.onAck(actuator(10, 99, 2)) == proto::AckVerdict::Dropped);  // foreign session
    CHECK(w.onAck(actuator(0, 7, 2)) == proto::AckVerdict::Completed);
    CHECK(w.onAck(actuator(0, 7, 3)) == proto::AckVerdict::ProtocolError);  // extension cannot be ahead
}

TEST_CASE("stall, stop and watchdog end the lock-step window safely") {
    proto::LockstepWindow w;
    w.begin(1, 1);
    REQUIRE(w.onSent(sensor(1, 1, 1)).ok());
    CHECK(w.onAck(actuator(1, 1, 1, proto::TickResult::RobotStalled)) == proto::AckVerdict::Stalled);
    CHECK(w.stalled());
    CHECK_FALSE(w.canSend());
    CHECK(w.onSent(sensor(2, 1, 1)).error().code == ErrorCode::InvalidState);
    w.begin(1, 2);  // reconnect/reset recovers
    CHECK(w.canSend());

    REQUIRE(w.onSent(sensor(1, 1, 2)).ok());
    CHECK(w.onAck(actuator(1, 1, 2, proto::TickResult::Stopped)) == proto::AckVerdict::Stopped);

    w.begin(1, 3);
    REQUIRE(w.onSent(sensor(1, 1, 3)).ok());
    w.onWatchdogExpired();
    CHECK(w.stalled());
    CHECK_FALSE(w.outstanding());
}

TEST_CASE("follower validates sensor frames and stamps its acknowledgements") {
    proto::LockstepFollower f;
    f.begin(7, 1);
    const auto s1 = sensor(1, 7, 1);
    REQUIRE(f.onSensorFrame(s1) == proto::FollowerAction::Step);
    const proto::ActuatorFrame a1 = f.makeAck(s1, proto::TickResult::Completed);
    CHECK(a1.sessionId == 7);
    CHECK(a1.runEpoch == 1);
    CHECK(a1.ackedTick() == 1);
    CHECK(a1.result == proto::TickResult::Completed);
    CHECK(f.lastCompletedTick() == 1);

    CHECK(f.onSensorFrame(s1) == proto::FollowerAction::ProtocolError);          // duplicate
    CHECK(f.onSensorFrame(sensor(2, 7, 0)) == proto::FollowerAction::Step);      // unspecified epoch accepted
    CHECK(f.onSensorFrame(sensor(2, 8, 1)) == proto::FollowerAction::Drop);      // foreign session
    CHECK(f.onSensorFrame(sensor(2, 7, 1)) == proto::FollowerAction::Step);

    // a frame of a newer epoch re-bases the follower (tick numbering restarts per epoch)
    CHECK(f.onSensorFrame(sensor(0, 7, 2)) == proto::FollowerAction::Step);
    CHECK(f.runEpoch() == 2);
    CHECK(f.onSensorFrame(sensor(5, 7, 1)) == proto::FollowerAction::Drop);      // stale epoch
}

TEST_CASE("Reset and Stop are acknowledged commands") {
    proto::LockstepFollower f;
    f.begin(7, 1);

    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    reset.commandId = 11;
    reset.sessionId = 7;
    reset.runEpoch = 2;  // the first message of the new epoch
    proto::ControlDecision d = f.onControl(reset);
    CHECK(d.apply);
    CHECK(d.sendAck);
    CHECK(d.ack.commandId == 11);
    CHECK(d.ack.result == proto::ControlResult::Applied);
    CHECK(d.ack.runEpoch == 2);
    CHECK(f.runEpoch() == 2);

    d = f.onControl(reset);  // retransmission: re-acked, not re-applied
    CHECK_FALSE(d.apply);
    CHECK(d.sendAck);
    CHECK(d.ack.commandId == 11);

    proto::SimControl staleReset = reset;
    staleReset.commandId = 12;
    staleReset.runEpoch = 2;  // not newer than what is in force
    d = f.onControl(staleReset);
    CHECK_FALSE(d.apply);
    CHECK(d.ack.result == proto::ControlResult::StaleEpoch);

    proto::SimControl pause;
    pause.command = proto::SimCommand::Pause;
    pause.commandId = 13;
    pause.sessionId = 7;
    pause.runEpoch = 1;  // issued before the reset
    d = f.onControl(pause);
    CHECK_FALSE(d.apply);
    CHECK(d.ack.result == proto::ControlResult::StaleEpoch);

    pause.runEpoch = 0;  // fire-and-forget callers need no ack
    pause.commandId = 0;
    d = f.onControl(pause);
    CHECK(d.apply);
    CHECK_FALSE(d.sendAck);

    proto::SimControl stop;
    stop.command = proto::SimCommand::Stop;
    stop.commandId = 14;
    stop.sessionId = 7;
    stop.runEpoch = 1;  // Stop is honoured whatever the epoch
    d = f.onControl(stop);
    CHECK(d.apply);
    CHECK(d.ack.result == proto::ControlResult::Applied);

    stop.sessionId = 99;  // another connection's command is ignored silently
    stop.commandId = 15;
    d = f.onControl(stop);
    CHECK_FALSE(d.apply);
    CHECK_FALSE(d.sendAck);
}

TEST_CASE("emulator and extension lock-step across an in-memory channel, with a mid-run reset") {
    auto pair = testing::makeInMemoryTransportPair();
    auto codec = std::make_shared<testing::MemoryMessageCodec>();
    proto::MessageChannel emulator(std::move(pair.a), codec);
    proto::MessageChannel extension(std::move(pair.b), codec);

    proto::LockstepWindow window;
    proto::LockstepFollower follower;
    window.begin(42, 1);
    follower.begin(42, 1);

    auto exchange = [&](std::uint64_t tick) {
        proto::SensorFrame s = sensor(tick, window.sessionId(), window.runEpoch());
        REQUIRE(window.onSent(s).ok());
        REQUIRE(emulator.send(s).ok());

        proto::Message m;
        REQUIRE(extension.receive(m, 100ms).value());
        const auto& got = std::get<proto::SensorFrame>(m);
        REQUIRE(follower.onSensorFrame(got) == proto::FollowerAction::Step);
        REQUIRE(extension.send(follower.makeAck(got, proto::TickResult::Completed)).ok());

        REQUIRE(emulator.receive(m, 100ms).value());
        return window.onAck(std::get<proto::ActuatorFrame>(m));
    };

    CHECK(exchange(0) == proto::AckVerdict::Completed);
    CHECK(exchange(1) == proto::AckVerdict::Completed);

    // Reset: the emulator bumps the epoch and sends the acknowledged command
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    reset.commandId = 1;
    reset.sessionId = 42;
    reset.runEpoch = 2;
    window.begin(42, 2);
    REQUIRE(emulator.send(reset).ok());
    proto::Message m;
    REQUIRE(extension.receive(m, 100ms).value());
    const proto::ControlDecision decision = follower.onControl(std::get<proto::SimControl>(m));
    REQUIRE(decision.apply);
    REQUIRE(extension.send(decision.ack).ok());
    REQUIRE(emulator.receive(m, 100ms).value());
    const auto& ack = std::get<proto::SimControlAck>(m);
    CHECK(ack.commandId == 1);
    CHECK(ack.result == proto::ControlResult::Applied);
    CHECK(ack.runEpoch == 2);

    // an ActuatorFrame the extension had already queued under epoch 1 is dropped by the emulator
    REQUIRE(extension.send(actuator(1, 42, 1)).ok());
    REQUIRE(emulator.receive(m, 100ms).value());
    CHECK(window.onAck(std::get<proto::ActuatorFrame>(m)) == proto::AckVerdict::Dropped);

    CHECK(exchange(0) == proto::AckVerdict::Completed);  // epoch 2 restarts at tick 0
}

TEST_CASE("control retries preserve results across intervening commands and skip preconditions") {
    proto::LockstepFollower follower;
    follower.begin(7, 1);
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    reset.commandId = 1;
    reset.sessionId = 7;
    reset.runEpoch = 2;
    const auto original = follower.onControl(reset);
    REQUIRE(original.apply);
    auto pause = reset;
    pause.command = proto::SimCommand::Pause;
    pause.commandId = 2;
    REQUIRE(follower.onControl(pause).apply);

    int checks = 0;
    const auto precondition = [&] {
        ++checks;
        return std::pair{proto::ControlResult::Rejected, std::string("state changed")};
    };
    const auto retry = follower.onControl(reset, precondition);
    CHECK_FALSE(retry.apply);
    CHECK(retry.sendAck);
    CHECK(retry.ack.commandId == original.ack.commandId);
    CHECK(retry.ack.command == original.ack.command);
    CHECK(retry.ack.result == original.ack.result);
    CHECK(retry.ack.runEpoch == original.ack.runEpoch);
    CHECK(retry.ack.tick == original.ack.tick);
    CHECK(retry.ack.detail == original.ack.detail);
    CHECK(checks == 0);

    auto rejected = pause;
    rejected.commandId = 3;
    const auto firstRejection = follower.onControl(rejected, precondition);
    CHECK(firstRejection.ack.result == proto::ControlResult::Rejected);
    pause.commandId = 4;
    REQUIRE(follower.onControl(pause).apply);
    const auto retryRejection = follower.onControl(rejected);
    CHECK_FALSE(retryRejection.apply);
    CHECK(retryRejection.ack.detail == firstRejection.ack.detail);
    CHECK(retryRejection.ack.result == firstRejection.ack.result);
    rejected.sessionId = 99;
    CHECK_FALSE(follower.onControl(rejected, precondition).sendAck);
    CHECK(checks == 1);
}

TEST_CASE("control retry retention is bounded and evicted IDs never reapply") {
    proto::LockstepFollower follower;
    follower.begin(7, 1);
    proto::SimControl pause;
    pause.sessionId = 7;
    pause.runEpoch = 1;
    for (std::uint64_t id = 1; id <= 65; ++id) {
        pause.commandId = id;
        REQUIRE(follower.onControl(pause).apply);
    }
    pause.commandId = 2;  // still in the 64-result window
    const auto retained = follower.onControl(pause);
    CHECK_FALSE(retained.apply);
    CHECK(retained.ack.result == proto::ControlResult::Applied);
    pause.commandId = 1;
    const auto evicted = follower.onControl(pause);
    CHECK_FALSE(evicted.apply);
    CHECK(evicted.ack.result == proto::ControlResult::Rejected);
    CHECK(evicted.ack.detail.find("retry window") != std::string::npos);
    follower.begin(8, 2);
    pause.sessionId = 8;
    pause.runEpoch = 2;
    CHECK(follower.onControl(pause).apply);  // new connection, fresh ID namespace
}
