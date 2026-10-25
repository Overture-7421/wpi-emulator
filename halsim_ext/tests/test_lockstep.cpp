#include <doctest/doctest.h>

#include <algorithm>

#include "fakes.h"
#include "wpiemu/sdk/testing/fake_emulator.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

namespace {
// Rig with one bridge, one device already announced, and the robot program "started".
struct Started {
    Started() : bridge(rig.addBridge()), pwm(FakeBridge::makeDescriptor("pwm", 3)) {
        bridge.queueDevice(pwm);
        REQUIRE(rig.start());
        rig.clock.started = true;
        REQUIRE(rig.emulator->waitForLifecycle(2000ms));
        id = bridge.idOf(pwm);
    }
    Rig rig;
    FakeBridge& bridge;
    sdk::DeviceDescriptor pwm;
    sdk::DeviceId id;
};
}  // namespace

TEST_CASE("lockstep: devices are announced before program_started, and frames wait for the program") {
    Rig rig;
    FakeBridge& bridge = rig.addBridge();
    bridge.queueDevice(FakeBridge::makeDescriptor("pwm", 3));
    REQUIRE(rig.start());

    // A frame arrives while the robot constructor is still running: it must not be answered or stepped.
    proto::SensorFrame f;
    f.tick = 1;
    f.simTimeUs = 5000;
    f.sessionId = 7;
    f.runEpoch = 1;
    REQUIRE(rig.emulator->send(proto::Message(f)));
    CHECK_FALSE(rig.emulator->take<proto::ActuatorFrame>(150ms).has_value());
    CHECK(rig.clock.steps == 0);
    CHECK(rig.emulator->take<proto::DeviceAnnounce>(1000ms).has_value());  // discovery runs meanwhile

    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const auto ack = rig.emulator->take<proto::ActuatorFrame>(2000ms);
    REQUIRE(ack.has_value());
    CHECK(ack->ackedTick() == 1);
    CHECK(rig.clock.fpgaUs == 5000);

    const auto history = rig.emulator->history();
    const auto announce = std::find(history.begin(), history.end(), static_cast<int>(proto::MessageType::DeviceAnnounce));
    const auto started = std::find(history.begin(), history.end(), sdk::testing::FakeEmulator::kHistoryLifecycle);
    REQUIRE(announce != history.end());
    REQUIRE(started != history.end());
    CHECK(announce < started);
}

TEST_CASE("lockstep: by default nothing is polled while the robot constructor runs") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.pollWhileConstructing = false;  // the production default
    Rig rig(defaultAck(), cfg);
    FakeBridge& bridge = rig.addBridge();
    bridge.queueDevice(FakeBridge::makeDescriptor("pwm", 3));
    REQUIRE(rig.start());
    std::this_thread::sleep_for(150ms);
    CHECK(bridge.polls == 0);
    CHECK_FALSE(rig.emulator->take<proto::DeviceAnnounce>(10ms).has_value());

    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    CHECK(rig.emulator->take<proto::DeviceAnnounce>(1000ms).has_value());  // announced at program start, before the signal
}

TEST_CASE("lockstep: each tick steps HAL time by exactly stepUs and is acknowledged with the session identity") {
    Started t;
    for (int i = 1; i <= 10; ++i) {
        const auto ack = t.rig.emulator->tick();
        REQUIRE(ack.has_value());
        CHECK(ack->result == proto::TickResult::Completed);
        CHECK(ack->ackedTick() == static_cast<std::uint64_t>(i));
        CHECK(ack->sessionId == 7);
        CHECK(ack->runEpoch == proto::kFirstRunEpoch);
        CHECK(ack->simTimeUs == i * 5000);
    }
    CHECK(t.rig.clock.fpgaUs == 50000);
    CHECK(t.rig.clock.steps == 10);
    CHECK(t.rig.runtime->stats().ticksCompleted == 10);

    const auto odd = t.rig.emulator->tick({}, 2500);
    REQUIRE(odd.has_value());
    CHECK(t.rig.clock.stepLog.back() == 2500);
}

TEST_CASE("lockstep: outputs are delta-compressed, the first and every Nth frame are full") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.fullFrameInterval = 4;
    Rig rig(defaultAck(), cfg);
    FakeBridge& bridge = rig.addBridge();
    const auto d = FakeBridge::makeDescriptor("pwm", 1);
    bridge.queueDevice(d);
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const sdk::DeviceId id = bridge.idOf(d);
    bridge.setOutput(id, 0, 0.5);

    auto a1 = rig.emulator->tick();
    REQUIRE(a1);
    CHECK(a1->full);
    REQUIRE(a1->samples.size() == 1);
    CHECK(a1->samples[0].value == 0.5);

    auto a2 = rig.emulator->tick();
    REQUIRE(a2);
    CHECK_FALSE(a2->full);
    CHECK(a2->samples.empty());

    bridge.setOutput(id, 0, -0.25);
    auto a3 = rig.emulator->tick();
    REQUIRE(a3);
    CHECK_FALSE(a3->full);
    REQUIRE(a3->samples.size() == 1);
    CHECK(a3->samples[0].value == -0.25);

    auto a4 = rig.emulator->tick();
    REQUIRE(a4);
    CHECK_FALSE(a4->full);

    auto a5 = rig.emulator->tick();  // N = 4 ticks after the last full frame -> full again
    REQUIRE(a5);
    CHECK(a5->full);
    CHECK(a5->samples.size() == 1);
}

TEST_CASE("lockstep: sensor inputs reach the owning bridge, battery and driver station are applied before the step") {
    Started t;
    std::vector<std::uint64_t> fpgaAtDsApply;
    proto::DsState ds;
    ds.enabled = true;
    ds.mode = proto::RobotMode::Autonomous;
    ds.station = proto::AllianceStation::Blue2;
    ds.joysticks[0].present = true;
    ds.joysticks[0].axes = {0.5f, -1.0f};
    REQUIRE(t.rig.emulator->sendDs(ds));

    const auto ack = t.rig.emulator->tick({{t.id, 1, 3.25}}, 5000, 11.5);
    REQUIRE(ack);

    REQUIRE(t.rig.ds.last().has_value());
    CHECK(t.rig.ds.last()->enabled);
    CHECK(t.rig.ds.last()->mode == proto::RobotMode::Autonomous);
    CHECK(t.rig.ds.last()->joysticks[0].axes.size() == 2);
    const auto battery = t.rig.power.last();
    REQUIRE(battery.has_value());
    CHECK(battery->first == doctest::Approx(11.5));
    const auto inputs = t.bridge.inputs();
    REQUIRE(inputs.size() == 1);
    CHECK(inputs[0].device == t.id);
    CHECK(inputs[0].value == 3.25);

    // The battery is only rewritten when it changes.
    const std::size_t calls = t.rig.power.calls.size();
    REQUIRE(t.rig.emulator->tick({}, 5000, 11.5));
    CHECK(t.rig.power.calls.size() == calls);
}

TEST_CASE("lockstep: a default (attached, disabled) driver station is applied when the program starts") {
    Started t;
    REQUIRE(waitUntil([&] { return t.rig.ds.count() >= 1; }));
    const proto::DsState first = t.rig.ds.applied.front();
    CHECK(first.dsAttached);
    CHECK_FALSE(first.enabled);
}

TEST_CASE("lockstep: unmapped devices are looped back locally and never travel to the emulator") {
    Started t;
    t.bridge.setOutput(t.id, 0, 0.9);
    proto::LoopbackSet set;
    set.loopback = {t.id};
    REQUIRE(t.rig.emulator->send(proto::Message(set)));
    std::this_thread::sleep_for(50ms);

    const auto ack = t.rig.emulator->tick();
    REQUIRE(ack);
    CHECK(ack->samples.empty());
    CHECK(t.bridge.loopbacks() == std::vector<sdk::DeviceId>{t.id});

    // Clearing the set maps the device again: its outputs flow.
    REQUIRE(t.rig.emulator->send(proto::Message(proto::LoopbackSet{})));
    std::this_thread::sleep_for(50ms);
    const auto again = t.rig.emulator->tick();
    REQUIRE(again);
    CHECK(again->samples.size() == 1);
}

TEST_CASE("lockstep: a device created while the robot ran is announced before the samples that mention it") {
    Started t;
    const auto late = FakeBridge::makeDescriptor("dio", 8);
    t.rig.clock.onStep = [&](std::uint64_t) {
        static bool done = false;
        if (!done) {
            done = true;
            t.bridge.queueDevice(late);  // created by robot code during the step
        }
    };
    REQUIRE(t.rig.emulator->tick());
    const sdk::DeviceId lateId = t.bridge.idOf(late);
    REQUIRE(lateId.valid());
    t.bridge.setOutput(lateId, 0, 1.0);
    REQUIRE(t.rig.emulator->tick());

    const auto history = t.rig.emulator->history();
    // announce(pwm), log, ack1, announce(dio), ack2 ...: the dio announce precedes the second ack.
    int announces = 0;
    int actuators = 0;
    bool lateAnnouncedBeforeSecondAck = false;
    for (const int type : history) {
        if (type == static_cast<int>(proto::MessageType::DeviceAnnounce) && ++announces == 2)
            lateAnnouncedBeforeSecondAck = actuators < 2;
        if (type == static_cast<int>(proto::MessageType::ActuatorFrame)) ++actuators;
    }
    CHECK(lateAnnouncedBeforeSecondAck);
}

TEST_CASE("lockstep: retiring a device announces it as retired and forgets its baseline") {
    Started t;
    t.bridge.setOutput(t.id, 0, 1.0);
    REQUIRE(t.rig.emulator->tick());
    t.bridge.queueRetire(t.pwm);
    REQUIRE(t.rig.emulator->tick());
    const auto retired = t.rig.emulator->take<proto::DeviceAnnounce>(1000ms, [](const proto::DeviceAnnounce& a) { return a.retired; });
    REQUIRE(retired.has_value());
    CHECK(retired->id == t.id);
}

TEST_CASE("lockstep: Pause, Resume, Step and SetTimeScale are acknowledged with the identity in force") {
    Started t;
    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Step, 1));  // while running: rejected
    auto stepAck = t.rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(stepAck);
    CHECK(stepAck->result == proto::ControlResult::Rejected);
    CHECK(stepAck->commandId == 1);

    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Pause, 2));
    auto pauseAck = t.rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(pauseAck);
    CHECK(pauseAck->result == proto::ControlResult::Applied);
    CHECK(pauseAck->sessionId == 7);

    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Step, 3));  // while paused: accepted
    auto step2 = t.rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(step2);
    CHECK(step2->result == proto::ControlResult::Applied);

    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Resume, 4));
    auto resumeAck = t.rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(resumeAck);
    CHECK(resumeAck->result == proto::ControlResult::Applied);

    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::SetTimeScale, 5));
    CHECK(t.rig.emulator->take<proto::SimControlAck>(1000ms)->result == proto::ControlResult::Applied);

    // Fire-and-forget commands (commandId 0) are applied without an ack.
    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Pause));
    std::this_thread::sleep_for(50ms);
    CHECK(t.rig.emulator->count<proto::SimControlAck>() == 0);
}

TEST_CASE("lockstep: Reset opens a new epoch, restarts ticks and forces a full frame") {
    Started t;
    t.bridge.setOutput(t.id, 0, 0.5);
    REQUIRE(t.rig.emulator->tick());
    const auto second = t.rig.emulator->tick();
    REQUIRE(second);
    CHECK_FALSE(second->full);

    t.rig.emulator->epoch = 2;
    t.rig.emulator->nextTick = 1;
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    reset.commandId = 9;
    reset.sessionId = 7;
    reset.runEpoch = 2;
    REQUIRE(t.rig.emulator->send(proto::Message(reset)));
    const auto resetAck = t.rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(resetAck);
    CHECK(resetAck->result == proto::ControlResult::Applied);
    CHECK(resetAck->runEpoch == 2);

    const auto first = t.rig.emulator->tick();
    REQUIRE(first);
    CHECK(first->runEpoch == 2);
    CHECK(first->ackedTick() == 1);
    CHECK(first->full);
    CHECK(first->samples.size() == 1);
}

TEST_CASE("lockstep: a repeated command id is re-acknowledged, not re-applied") {
    Started t;
    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Pause, 5));
    REQUIRE(t.rig.emulator->take<proto::SimControlAck>(1000ms));
    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Pause, 5));
    const auto again = t.rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(again);
    CHECK(again->commandId == 5);
    CHECK(again->result == proto::ControlResult::Applied);
}

TEST_CASE("lockstep: Stop is acknowledged, resumes HAL time, says Goodbye and ends the runtime") {
    Started t;
    REQUIRE(t.rig.emulator->tick());
    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Stop, 11));
    const auto ack = t.rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(ack);
    CHECK(ack->command == proto::SimCommand::Stop);
    CHECK(t.rig.emulator->take<proto::Goodbye>(1000ms).has_value());
    CHECK(t.rig.runtime->waitFinished(2000ms));
    CHECK_FALSE(t.rig.clock.paused);
    CHECK(t.rig.clock.resumeCalls >= 1);
}

TEST_CASE("lockstep: stale epochs and foreign sessions are dropped, duplicates are protocol errors") {
    Started t;
    REQUIRE(t.rig.emulator->tick());  // tick 1 completed

    // Newer-epoch-less duplicate: tick 1 again.
    proto::SensorFrame dup;
    dup.tick = 1;
    dup.sessionId = 7;
    dup.runEpoch = 1;
    REQUIRE(t.rig.emulator->send(proto::Message(dup)));
    const auto err = t.rig.emulator->take<proto::ActuatorFrame>(1000ms);
    REQUIRE(err);
    CHECK(err->result == proto::TickResult::ProtocolError);

    // Foreign session: dropped silently.
    proto::SensorFrame foreign;
    foreign.tick = 2;
    foreign.sessionId = 99;
    foreign.runEpoch = 1;
    REQUIRE(t.rig.emulator->send(proto::Message(foreign)));
    CHECK_FALSE(t.rig.emulator->take<proto::ActuatorFrame>(100ms).has_value());
    CHECK(t.rig.runtime->stats().framesDropped == 1);
    CHECK(t.rig.runtime->stats().protocolErrors == 1);
    CHECK(t.rig.clock.steps == 1);

    // A stale driver-station state (older epoch) is dropped, a current one is applied.
    t.rig.emulator->epoch = 2;
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    reset.commandId = 1;
    reset.sessionId = 7;
    reset.runEpoch = 2;
    REQUIRE(t.rig.emulator->send(proto::Message(reset)));
    REQUIRE(t.rig.emulator->take<proto::SimControlAck>(1000ms));
    proto::DsState stale;
    stale.enabled = true;
    stale.sessionId = 7;
    stale.runEpoch = 1;
    REQUIRE(t.rig.emulator->send(proto::Message(stale)));
    t.rig.emulator->nextTick = 1;
    REQUIRE(t.rig.emulator->tick());
    CHECK_FALSE(t.rig.ds.last()->enabled);
}

TEST_CASE("free-running (realtime) mode: frames are answered without stepping HAL time") {
    proto::HelloAck ack = defaultAck();
    ack.clockMode = proto::ClockMode::Realtime;
    Rig rig(ack);
    FakeBridge& bridge = rig.addBridge();
    const auto d = FakeBridge::makeDescriptor("pwm", 2);
    bridge.queueDevice(d);
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    bridge.setOutput(bridge.idOf(d), 0, 0.4);

    const auto a = rig.emulator->tick({{bridge.idOf(d), 1, 2.0}});
    REQUIRE(a);
    CHECK(a->result == proto::TickResult::Completed);
    CHECK(rig.clock.steps == 0);
    CHECK(bridge.inputs().size() == 1);

    // Driver-station updates apply immediately when nothing steps.
    proto::DsState ds;
    ds.enabled = true;
    REQUIRE(rig.emulator->sendDs(ds));
    CHECK(waitUntil([&] { return rig.ds.last() && rig.ds.last()->enabled; }));
}

TEST_CASE("discovery-only: devices are announced, nothing is stepped, no watchdog interferes") {
    proto::HelloAck ack = defaultAck();
    ack.discoveryOnly = true;
    Rig rig(ack);
    FakeBridge& bridge = rig.addBridge();
    bridge.queueDevice(FakeBridge::makeDescriptor("pwm", 5));
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    CHECK(rig.emulator->take<proto::DeviceAnnounce>(1000ms).has_value());

    std::this_thread::sleep_for(600ms);  // longer than the 400 ms watchdog
    CHECK(rig.clock.steps == 0);
    CHECK(rig.runtime->stats().emulatorLost == 0);
    bridge.queueDevice(FakeBridge::makeDescriptor("dio", 6));  // lazily created later
    CHECK(rig.emulator->take<proto::DeviceAnnounce>(1000ms, [](const proto::DeviceAnnounce& a) {
              return a.descriptor.type == "dio";
          }).has_value());
}

TEST_CASE("armed session: BeginLockstep pauses HAL time and steps, EndLockstep returns to free-running") {
    proto::HelloAck ack = defaultAck();
    ack.discoveryOnly = true;
    Rig rig(ack);
    rig.addBridge();
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    rig.emulator->epoch = 2;
    REQUIRE(rig.emulator->sendControl(proto::SimCommand::BeginLockstep, 1));
    const auto begin = rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(begin);
    CHECK(begin->result == proto::ControlResult::Applied);
    CHECK(begin->runEpoch == 2);

    const auto first = rig.emulator->tick();
    REQUIRE(first);
    CHECK(first->result == proto::TickResult::Completed);
    CHECK(first->ackedTick() == 1);
    CHECK(rig.clock.steps == 1);

    rig.emulator->epoch = 3;
    REQUIRE(rig.emulator->sendControl(proto::SimCommand::EndLockstep, 2));
    const auto end = rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(end);
    CHECK(end->result == proto::ControlResult::Applied);

    REQUIRE(rig.emulator->sendControl(proto::SimCommand::EndLockstep, 3));  // already armed
    CHECK(rig.emulator->take<proto::SimControlAck>(1000ms)->result == proto::ControlResult::Rejected);
}

TEST_CASE("lockstep: BeginLockstep while already in lock-step is rejected") {
    Started t;
    t.rig.emulator->epoch = 2;
    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::BeginLockstep, 1));
    const auto ack = t.rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(ack);
    CHECK(ack->result == proto::ControlResult::Rejected);
}

TEST_CASE(
    "armed session: Begin and End retries replay original acks without repeating transitions") {
    auto ack = defaultAck();
    ack.discoveryOnly = true;
    Rig rig(ack);
    auto& bridge = rig.addBridge();
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const auto send = [&](proto::SimCommand command, std::uint64_t id, std::uint64_t epoch) {
        proto::SimControl c;
        c.command = command;
        c.commandId = id;
        c.sessionId = 7;
        c.runEpoch = epoch;
        REQUIRE(rig.emulator->send(proto::Message(c)));
        const auto response = rig.emulator->take<proto::SimControlAck>(1000ms);
        REQUIRE(response);
        return *response;
    };
    const auto sameAck = [](const proto::SimControlAck& a, const proto::SimControlAck& b) {
        CHECK(a.commandId == b.commandId);
        CHECK(a.command == b.command);
        CHECK(a.result == b.result);
        CHECK(a.sessionId == b.sessionId);
        CHECK(a.runEpoch == b.runEpoch);
        CHECK(a.tick == b.tick);
        CHECK(a.detail == b.detail);
    };
    const auto begin = send(proto::SimCommand::BeginLockstep, 1, 2);
    REQUIRE(begin.result == proto::ControlResult::Applied);
    sameAck(begin, send(proto::SimCommand::BeginLockstep, 1, 2));
    REQUIRE(send(proto::SimCommand::Pause, 2, 2).result == proto::ControlResult::Applied);
    sameAck(begin, send(proto::SimCommand::BeginLockstep, 1, 2));
    CHECK(rig.clock.pauseCalls == 1);
    CHECK(bridge.historyResets == 2);  // connection and Begin, not either retry

    const auto end = send(proto::SimCommand::EndLockstep, 3, 3);
    REQUIRE(end.result == proto::ControlResult::Applied);
    sameAck(end, send(proto::SimCommand::EndLockstep, 3, 3));
    REQUIRE(send(proto::SimCommand::Pause, 4, 3).result == proto::ControlResult::Applied);
    sameAck(end, send(proto::SimCommand::EndLockstep, 3, 3));
    CHECK(rig.clock.resumeCalls == 1);

    const auto rejected = send(proto::SimCommand::EndLockstep, 5, 4);
    REQUIRE(rejected.result == proto::ControlResult::Rejected);
    REQUIRE(send(proto::SimCommand::BeginLockstep, 6, 4).result == proto::ControlResult::Applied);
    sameAck(rejected, send(proto::SimCommand::EndLockstep, 5, 4));
    CHECK(rig.clock.paused);
    CHECK(rig.clock.resumeCalls == 1);
    sameAck(begin, send(proto::SimCommand::BeginLockstep, 1, 2));  // retained across epochs

    proto::SimControl foreign;
    foreign.command =
        proto::SimCommand::BeginLockstep;  // would fail the current state precondition
    foreign.commandId = 7;
    foreign.sessionId = 99;
    foreign.runEpoch = 5;
    REQUIRE(rig.emulator->send(proto::Message(foreign)));
    CHECK_FALSE(rig.emulator->take<proto::SimControlAck>(50ms));
}

TEST_CASE("extension log lines are forwarded to the emulator as Log messages") {
    Started t;
    t.rig.runtime->logger().warn("bridge.test", "something odd");
    const auto log = t.rig.emulator->take<proto::Log>(1000ms, [](const proto::Log& l) { return l.source == "bridge.test"; });
    REQUIRE(log.has_value());
    CHECK(log->message == "something odd");
    CHECK(log->level == sdk::LogLevel::Warn);
    CHECK(t.rig.logContains("something odd"));
}
