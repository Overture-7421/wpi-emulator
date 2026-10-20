#include <doctest/doctest.h>

#include <atomic>
#include <thread>

#include "runner_rig.h"

// SessionRunner, part 1: the connection lifecycle and the lock-step protocol - handshake, ticks, Reset, the watchdog,
// reconnects and Stop. Parts 2 and 3 (test_runner_control.cpp, test_runner_vision.cpp) cover run control and cameras.

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using namespace std::chrono_literals;
using wpiemu::test_runtime::Rig;

TEST_CASE("runner: handshake assigns session, epoch, step and the negotiated watchdog") {
    Rig rig;
    CHECK(rig.runner->state() == RunnerState::WaitingForRobot);
    REQUIRE(rig.slot.latest() != nullptr);  // an initial snapshot exists before any robot connects
    CHECK_FALSE(rig.slot.latest()->robotConnected());

    rig.connect();
    rig.ext->hello(500);
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
    const proto::HelloAck& ack = *rig.ext->helloAck;
    CHECK(ack.accepted);
    CHECK(ack.stepUs == 5000);
    CHECK(ack.sessionId == 1);
    CHECK(ack.runEpoch == proto::kFirstRunEpoch);
    CHECK(ack.watchdogTimeoutMs == 500);
    CHECK(ack.clockMode == proto::ClockMode::Lockstep);
    CHECK_FALSE(ack.discoveryOnly);
    REQUIRE(rig.run([&] { return !rig.ext->loopbacks.empty() && !rig.ext->dsStates.empty(); }));
    CHECK(rig.ext->loopbacks.front().loopback.empty());
    CHECK(rig.ext->dsStates.front().sessionId == ack.sessionId);
    CHECK(rig.ext->dsStates.front().runEpoch == ack.runEpoch);
    CHECK(rig.runner->state() == RunnerState::Running);
    REQUIRE(rig.runner->robotHello().has_value());
    CHECK(rig.runner->robotHello()->pid == 4242);
    CHECK(rig.runner->runEpoch() == 1);
    CHECK(rig.slot.latest()->robotConnected());
}

TEST_CASE("runner: watchdog negotiation clamps and honours the emulator preference") {
    {
        Rig rig;
        rig.connect();
        rig.ext->hello(10);  // below the minimum
        REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
        CHECK(rig.ext->helloAck->watchdogTimeoutMs == proto::kMinWatchdogTimeoutMs);
    }
    {
        Rig rig;
        rig.connect();
        rig.ext->hello(0);
        REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
        CHECK(rig.ext->helloAck->watchdogTimeoutMs == proto::kDefaultWatchdogTimeoutMs);
    }
    {
        RunnerOptions o = Rig::fast();
        o.preferredWatchdogMs = 5000;
        Rig rig(o);
        rig.connect();
        rig.ext->hello(300);
        REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
        CHECK(rig.ext->helloAck->watchdogTimeoutMs == 5000);  // the larger of the two
    }
}

TEST_CASE("runner: a peer with a protocol version below the minimum is refused") {
    Rig rig;
    rig.connect();
    rig.ext->hello(0, 0);
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
    CHECK_FALSE(rig.ext->helloAck->accepted);
    CHECK_FALSE(rig.ext->helloAck->reason.empty());
    CHECK(rig.runner->state() == RunnerState::WaitingForRobot);
    CHECK(rig.runner->ticksSent() == 0);
    CHECK(rig.run([&] { return rig.ext->disconnected; }));  // the link was closed
}

TEST_CASE("runner: lock-step - consecutive ticks, one outstanding, identity on every frame") {
    Rig rig;
    rig.ext = nullptr;
    rig.handshake();
    rig.ext->autoAck = false;
    rig.ext->pending.reset();
    // the first tick was auto-acked during the handshake; from here every tick needs an explicit answer
    const std::size_t base = rig.ext->sensorFrames.size();
    REQUIRE(rig.run([&] { return rig.ext->pending.has_value(); }));
    for (int i = 0; i < 20; ++i) {
        rig.runner->poll(5ms);
        rig.ext->pump();
    }
    CHECK(rig.ext->sensorFrames.size() == base + 1);  // exactly one frame in flight while unacknowledged

    for (int i = 0; i < 5; ++i) {
        REQUIRE(rig.ext->pending.has_value());
        const proto::SensorFrame sf = *rig.ext->pending;
        rig.ext->pending.reset();
        rig.ext->answer(sf);
        REQUIRE(rig.run([&] { return rig.ext->pending.has_value(); }));
    }
    const auto& frames = rig.ext->sensorFrames;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        CHECK(frames[i].tick == i);
        CHECK(frames[i].sessionId == 1);
        CHECK(frames[i].runEpoch == 1);
        CHECK(frames[i].stepUs == 5000);
        CHECK(frames[i].simTimeUs == static_cast<SimTimeUs>((i + 1) * 5000));
    }
    CHECK(frames.front().full);
    CHECK_FALSE(frames[1].full);
}

TEST_CASE("runner: robot outputs reach the mechanisms, sensors come back, announces update the loopback set") {
    Rig rig;
    rig.connect();
    rig.ext->hello();
    rig.ext->announceAll();
    rig.ext->send(wpiemu::test_runtime::talonAnnounce(300, 77));  // a device nobody maps
    rig.ext->outputs = {{DeviceId{105}, 1, 6.0}};                 // intake motorVoltage = 6 V
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 5; }));

    // the unmapped device is in the last LoopbackSet; mapped ones are not
    REQUIRE(rig.run([&] { return !rig.ext->loopbacks.empty() && rig.ext->loopbacks.back().loopback.size() == 1; }));
    CHECK(rig.ext->loopbacks.back().loopback[0] == DeviceId{300});
    CHECK(rig.warnings("unmapped_loopback") == 1);

    // intake encoder (cancoder 106) velocity = 6 V * 10 / 2pi rps appears in sensor frames
    bool seen = false;
    for (const auto& f : rig.ext->sensorFrames)
        for (const auto& s : f.samples)
            if (s.device == DeviceId{106} && s.signal == 1 && std::abs(s.value - 60.0 / kTwoPi) < 1e-9) seen = true;
    CHECK(seen);
}

TEST_CASE("runner: Reset bumps the epoch, resets the sim, restarts tick numbers and ignores late old-epoch frames") {
    Rig rig;
    rig.handshake();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 10; }));
    const auto* intake = dynamic_cast<wpiemu::test_runtime::TestRoller*>(rig.session->mechanisms().find("intake"));
    REQUIRE(intake != nullptr);
    const int resets = intake->resets;

    // hold the extension: the next tick stays unanswered while Reset is issued
    rig.ext->autoAck = false;
    rig.ext->pending.reset();
    REQUIRE(rig.run([&] { return rig.ext->pending.has_value(); }));
    const proto::SensorFrame inFlight = *rig.ext->pending;

    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    rig.runner->control(reset);
    REQUIRE(rig.run([&] { return !rig.ext->controls.empty(); }));
    const proto::SimControl& sent = rig.ext->controls.back();
    CHECK(sent.command == proto::SimCommand::Reset);
    CHECK(sent.commandId != 0);
    CHECK(sent.runEpoch == 2);
    CHECK(sent.sessionId == 1);
    CHECK(rig.runner->runEpoch() == 2);
    CHECK(intake->resets == resets + 1);
    CHECK(rig.session->clock().tickIndex() == 0);

    // the answer to the pre-reset tick arrives late (old epoch): dropped, not applied
    proto::ActuatorFrame late;
    late.tick = inFlight.tick;
    late.ackTick = inFlight.tick;
    late.sessionId = 1;
    late.runEpoch = 1;
    late.samples = {{DeviceId{105}, 1, 12.0}};
    rig.ext->send(late);
    const std::uint64_t droppedBefore = rig.runner->droppedFrames();
    REQUIRE(rig.run([&] { return rig.runner->droppedFrames() > droppedBefore; }));
    CHECK(rig.session->hub().motorOutput({"intake", "motor"}).valid == false);

    // ticks resume in the new epoch from 0, after the extension acknowledged the Reset
    rig.ext->autoAck = true;
    if (rig.ext->pending) rig.ext->answer(*rig.ext->pending);
    rig.ext->pending.reset();
    auto newEpochFrames = [&] {
        std::vector<proto::SensorFrame> frames;
        for (const auto& f : rig.ext->sensorFrames)
            if (f.runEpoch == 2) frames.push_back(f);
        return frames;
    };
    REQUIRE(rig.run([&] { return newEpochFrames().size() >= 3; }));
    const auto frames = newEpochFrames();
    for (std::size_t i = 0; i < 3; ++i) CHECK(frames[i].tick == i);
    CHECK(frames.front().full);
    CHECK_FALSE(frames[1].full);
}

TEST_CASE("runner: a start placement queued before Reset is in force when the sim resets") {
    Rig rig;
    rig.handshake();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 5; }));
    rig.runner->setStartPlacement(RobotStartPose{6.f, 2.f, 0.5f});
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    rig.runner->control(reset);
    REQUIRE(rig.run([&] { return rig.runner->runEpoch() == 2; }));
    const Pose3 pose = rig.session->world().pose(rig.session->chassisBody());
    CHECK(pose.position.x == doctest::Approx(6.f));
    CHECK(pose.position.y == doctest::Approx(2.f));
}

TEST_CASE("runner: no ticks are sent until the extension acknowledges a Reset") {
    Rig rig;
    rig.handshake();
    rig.ext->ackControls = false;
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    rig.runner->control(reset);
    REQUIRE(rig.run([&] { return !rig.ext->controls.empty(); }));
    rig.ext->sensorFrames.clear();
    for (int i = 0; i < 30; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.ext->sensorFrames.empty());  // blocked on the ack

    // the ack never comes: the watchdog gives up and the runner reports a stall instead of hanging
    rig.clock.advance(3s);
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::RobotStalled; }));
}

TEST_CASE("runner: watchdog expiry stalls lock-step, late frames are ignored, Reset recovers") {
    Rig rig;
    rig.handshake(500);
    rig.ext->autoAck = false;
    rig.ext->pending.reset();
    REQUIRE(rig.run([&] { return rig.ext->pending.has_value(); }));
    const proto::SensorFrame outstanding = *rig.ext->pending;

    rig.clock.advance(400ms);
    rig.runner->poll(1ms);
    CHECK(rig.runner->state() == RunnerState::Running);  // not yet
    rig.clock.advance(200ms);
    rig.runner->poll(1ms);
    CHECK(rig.runner->state() == RunnerState::RobotStalled);
    CHECK(rig.logger.count(LogLevel::Error) >= 1);
    CHECK(rig.warnings("robot_stalled") == 1);

    // no further frames, even though plenty of time passes
    const std::size_t frames = rig.ext->sensorFrames.size();
    for (int i = 0; i < 20; ++i) {
        rig.clock.advance(100ms);
        rig.runner->poll(1ms);
        rig.ext->pump();
    }
    CHECK(rig.ext->sensorFrames.size() == frames);

    // the stalled answer finally arrives: dropped, lock-step stays suspended
    rig.ext->answer(outstanding);
    const std::uint64_t dropped = rig.runner->droppedFrames();
    REQUIRE(rig.run([&] { return rig.runner->droppedFrames() > dropped; }));
    CHECK(rig.runner->state() == RunnerState::RobotStalled);

    // Reset opens a new epoch and recovery
    rig.ext->autoAck = true;
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    rig.runner->control(reset);
    REQUIRE(rig.run([&] { return rig.ext->sensorFrames.size() > frames; }));
    CHECK(rig.runner->state() == RunnerState::Running);
    CHECK(rig.ext->sensorFrames.back().runEpoch == 2);
}

TEST_CASE("runner: a RobotStalled answer from the extension suspends lock-step") {
    Rig rig;
    rig.handshake();
    rig.ext->nextResult = proto::TickResult::RobotStalled;
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::RobotStalled; }));
    const std::uint64_t sent = rig.runner->ticksSent();
    for (int i = 0; i < 20; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == sent);
}

TEST_CASE("runner: disconnect clears devices, reconnect starts a new session and epoch from a reset sim") {
    Rig rig;
    rig.handshake();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 8; }));
    CHECK(rig.session->hub().discovered().size() == 7);

    rig.ext->ch.close();
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::WaitingForRobot; }));
    CHECK(rig.session->hub().discovered().empty());
    CHECK(rig.session->hub().boundDevice({"intake", "motor"}).has_value());  // bindings survive
    CHECK_FALSE(rig.slot.latest()->robotConnected());

    rig.link.reset();
    rig.connect();
    rig.ext->hello();
    rig.ext->announceAll();
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value() && rig.runner->ticksSent() > 8 && rig.session->hub().discovered().size() == 7; }));
    CHECK(rig.ext->helloAck->sessionId == 2);
    CHECK(rig.ext->helloAck->runEpoch == 2);
    CHECK(rig.session->hub().discovered().size() == 7);
    REQUIRE_FALSE(rig.ext->sensorFrames.empty());
    CHECK(rig.ext->sensorFrames.front().tick == 0);
    CHECK(rig.ext->sensorFrames.front().runEpoch == 2);
}

TEST_CASE("runner: a new connection replaces the old one") {
    Rig rig;
    rig.handshake();
    auto first = std::move(rig.ext);
    rig.link.reset();
    rig.connect();
    rig.ext->hello();
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
    CHECK(rig.ext->helloAck->sessionId == 2);
    // the first link was closed by the runner
    for (int i = 0; i < 100 && !first->disconnected; ++i) {
        first->pump();
        std::this_thread::sleep_for(5ms);
    }
    CHECK(first->disconnected);
}

TEST_CASE("runner: Stop is acknowledged by the extension and finishes the session") {
    Rig rig;
    rig.handshake();
    proto::SimControl stop;
    stop.command = proto::SimCommand::Stop;
    rig.runner->control(stop);
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::Stopped; }));
    bool sawStop = false;
    for (const auto& c : rig.ext->controls) sawStop |= c.command == proto::SimCommand::Stop && c.commandId != 0;
    CHECK(sawStop);
    const std::uint64_t sent = rig.runner->ticksSent();
    for (int i = 0; i < 10; ++i) rig.runner->poll(1ms);
    CHECK(rig.runner->ticksSent() == sent);
    CHECK_FALSE(rig.slot.latest()->robotConnected());
}

TEST_CASE("runner: Stop gives up on an unresponsive extension, Stop without a robot ends immediately") {
    {
        Rig rig;
        rig.handshake(300);
        rig.ext->ackControls = false;
        proto::SimControl stop;
        stop.command = proto::SimCommand::Stop;
        rig.runner->control(stop);
        for (int i = 0; i < 5; ++i) rig.runner->poll(1ms);
        CHECK(rig.runner->state() != RunnerState::Stopped);
        rig.clock.advance(1s);
        REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::Stopped; }));
    }
    {
        Rig rig;
        proto::SimControl stop;
        stop.command = proto::SimCommand::Stop;
        rig.runner->control(stop);
        rig.runner->poll(1ms);
        CHECK(rig.runner->state() == RunnerState::Stopped);
    }
}

TEST_CASE("runner: robot logs are forwarded, stray and malformed traffic is ignored safely") {
    Rig rig;
    rig.connect();
    rig.ext->send(proto::Goodbye{"early"});          // before Hello
    rig.ext->send(wpiemu::test_runtime::talonAnnounce(1, 1));  // before Hello
    rig.ext->hello();
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
    CHECK(rig.session->hub().discovered().empty());
    CHECK(rig.runner->state() == RunnerState::Running);  // the early Goodbye did not tear the session down

    rig.ext->hello();  // duplicate Hello
    proto::Log log;
    log.level = LogLevel::Error;
    log.source = "bridge.phoenix6";
    log.message = "something odd";
    rig.ext->send(log);
    rig.ext->send(proto::SensorFrame{});  // the robot must never send this
    REQUIRE(rig.run([&] {
        for (const auto& e : rig.logger.entries())
            if (e.category == "robot.bridge.phoenix6" && e.message == "something odd" && e.level == LogLevel::Error) return true;
        return false;
    }));
    CHECK(rig.runner->state() == RunnerState::Running);
    CHECK(rig.ext->helloAck->sessionId == 1);
}

TEST_CASE("runner: an unsolicited ActuatorFrame is a protocol error and closes the connection") {
    Rig rig;
    rig.handshake();
    rig.ext->autoAck = false;
    rig.ext->pending.reset();
    REQUIRE(rig.run([&] { return rig.ext->pending.has_value(); }));
    proto::ActuatorFrame bogus;  // wrong tick for the one in flight, current identity
    bogus.tick = 999;
    bogus.ackTick = 999;
    bogus.sessionId = 1;
    bogus.runEpoch = 1;
    rig.ext->send(bogus);
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::WaitingForRobot; }));
}

TEST_CASE("runner: runs on its own thread against a robot thread, then stops cleanly") {
    RunnerOptions o = Rig::fast();
    Rig rig(o);
    rig.connect();
    rig.runner->start();
    rig.runner->start();  // idempotent

    std::atomic<bool> stopRobot{false};
    std::thread robot([&] {
        rig.ext->hello();
        rig.ext->announceAll();
        rig.ext->outputs = {{DeviceId{105}, 1, 3.0}};
        while (!stopRobot) {
            rig.ext->pump();
            std::this_thread::sleep_for(1ms);
        }
    });
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (rig.runner->ticksSent() < 100 && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(1ms);
    CHECK(rig.runner->ticksSent() >= 100);

    proto::SimControl stop;
    stop.command = proto::SimCommand::Stop;
    rig.runner->control(stop);
    while (rig.runner->state() != RunnerState::Stopped && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(1ms);
    CHECK(rig.runner->state() == RunnerState::Stopped);
    stopRobot = true;
    robot.join();
    rig.runner->stop();  // joins the already-finished thread
    CHECK(rig.slot.latest()->tick() > 0);
}
