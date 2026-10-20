#include <doctest/doctest.h>

#include "runner_rig.h"

// SessionRunner, part 2: what the operator, the UI and the game steer through the runner - pause, step and resume, the driver
// station and bindings forwarded to the robot, pacing, and the discovery-only and armed run modes.

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using namespace std::chrono_literals;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::Rig;
using wpiemu::test_runtime::sampleConfig;

TEST_CASE("runner: Pause stops frames, Step advances exactly n ticks, Resume continues") {
    Rig rig;
    rig.handshake();
    proto::SimControl pause;
    pause.command = proto::SimCommand::Pause;
    rig.runner->control(pause);
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::Paused; }));
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == rig.ext->sensorFrames.size(); }));  // settle: last frame answered
    const std::uint64_t paused = rig.runner->ticksSent();
    for (int i = 0; i < 20; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == paused);
    CHECK(rig.slot.latest()->paused());

    proto::SimControl step;
    step.command = proto::SimCommand::Step;
    step.stepCount = 3;
    rig.runner->control(step);
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == paused + 3; }));
    for (int i = 0; i < 20; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == paused + 3);  // still paused afterwards
    CHECK(rig.runner->state() == RunnerState::Paused);

    proto::SimControl resume;
    resume.command = proto::SimCommand::Resume;
    rig.runner->control(resume);
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() > paused + 10; }));
    CHECK(rig.runner->state() == RunnerState::Running);

    // Step while running is ignored
    const std::uint64_t before = rig.runner->ticksSent();
    rig.runner->control(step);
    pause.command = proto::SimCommand::Pause;
    rig.runner->control(pause);
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::Paused; }));
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == rig.ext->sensorFrames.size(); }));
    const std::uint64_t settled = rig.runner->ticksSent();
    CHECK(settled >= before);
    for (int i = 0; i < 10; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == settled);
}

TEST_CASE("runner: starting paused waits for a Step") {
    RunnerOptions o = Rig::fast();
    o.startPaused = true;
    Rig rig(o);
    rig.connect();
    rig.ext->hello();
    for (int i = 0; i < 30; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.ext->helloAck.has_value());
    CHECK(rig.runner->ticksSent() == 0);
    CHECK(rig.runner->state() == RunnerState::Paused);
    proto::SimControl step;
    step.command = proto::SimCommand::Step;
    rig.runner->control(step);  // stepCount defaults to 1
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == 1; }));
}

TEST_CASE("runner: Pause and Resume are forwarded so the extension's watchdog stays disarmed while paused") {
    Rig rig;
    rig.handshake(500);
    CHECK_FALSE(rig.ext->remotePaused);

    proto::SimControl pause;
    pause.command = proto::SimCommand::Pause;
    rig.runner->control(pause);
    REQUIRE(rig.run([&] { return rig.ext->remotePaused; }));
    const proto::SimControl& sent = rig.ext->controls.back();
    CHECK(sent.command == proto::SimCommand::Pause);
    CHECK(sent.commandId != 0);
    CHECK(sent.sessionId == 1);
    CHECK(sent.runEpoch == 1);
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::Paused; }));
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == rig.ext->sensorFrames.size(); }));

    // far longer than the 500 ms watchdog, with no DsState or any other traffic
    rig.ext->lastRxAt = rig.clock.now();
    const std::size_t warnings = rig.logger.count(LogLevel::Warn);
    for (int i = 0; i < 20; ++i) {
        rig.clock.advance(1s);
        rig.runner->poll(1ms);
        rig.ext->pump();
    }
    CHECK(rig.clock.now() - rig.ext->lastRxAt > 500ms);
    CHECK_FALSE(rig.ext->watchdogWouldFire(500ms, rig.clock.now()));
    CHECK(rig.runner->state() == RunnerState::Paused);
    CHECK_FALSE(rig.ext->disconnected);
    CHECK(rig.logger.count(LogLevel::Warn) == warnings);

    proto::SimControl resume;
    resume.command = proto::SimCommand::Resume;
    rig.runner->control(resume);
    REQUIRE(rig.run([&] { return !rig.ext->remotePaused; }));
    CHECK(rig.ext->controls.back().command == proto::SimCommand::Resume);
    CHECK(rig.ext->controls.back().commandId > sent.commandId);
    const std::uint64_t before = rig.runner->ticksSent();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() > before + 5; }));
    CHECK(rig.runner->state() == RunnerState::Running);
}

TEST_CASE("runner: a start-paused run tells the extension it is paused after the handshake, and again after a reconnect") {
    RunnerOptions o = Rig::fast();
    o.startPaused = true;
    Rig rig(o);
    rig.connect();
    rig.ext->hello();
    REQUIRE(rig.run([&] { return rig.ext->remotePaused; }));
    CHECK(rig.ext->controls.back().command == proto::SimCommand::Pause);
    CHECK(rig.ext->controls.back().sessionId == rig.ext->helloAck->sessionId);
    CHECK(rig.runner->ticksSent() == 0);

    rig.link->close();  // the robot restarts: the new extension instance starts unpaused
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::WaitingForRobot; }));
    rig.connect();
    rig.ext->hello();
    REQUIRE(rig.run([&] { return rig.ext->remotePaused; }));
    CHECK(rig.ext->helloAck->sessionId == 2);
    CHECK(rig.ext->controls.back().sessionId == 2);
    CHECK(rig.runner->state() == RunnerState::Paused);
}

TEST_CASE("runner: a Pause issued while no robot is connected is sent once it connects") {
    Rig rig;
    proto::SimControl pause;
    pause.command = proto::SimCommand::Pause;
    rig.runner->control(pause);
    rig.runner->poll(1ms);
    rig.connect();
    rig.ext->hello();
    REQUIRE(rig.run([&] { return rig.ext->remotePaused; }));
    CHECK(rig.runner->ticksSent() == 0);
}

TEST_CASE("runner: DS state is stamped and forwarded, brownout forces the robot disabled") {
    RobotConfig c = sampleConfig();
    c.battery.internalResistanceOhm = 0.5;
    Rig rig(Rig::fast(), c);
    rig.connect();
    rig.ext->hello();
    rig.ext->announceAll();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 2; }));

    proto::DsState ds;
    ds.enabled = true;
    ds.mode = proto::RobotMode::Teleop;
    rig.runner->setDriverStation(ds);
    REQUIRE(rig.run([&] { return !rig.ext->dsStates.empty() && rig.ext->dsStates.back().enabled; }));
    CHECK(rig.ext->dsStates.back().sessionId == 1);
    CHECK(rig.session->driverStation().enabled);

    // the robot floors every motor: 170 A through 0.5 ohm -> brownout -> the DS state is re-sent disabled
    rig.ext->outputs = {{DeviceId{105}, 1, 12.0}, {DeviceId{101}, 1, 12.0}, {DeviceId{102}, 1, 12.0}, {DeviceId{103}, 1, 12.0}, {DeviceId{104}, 1, 12.0}};
    REQUIRE(rig.run([&] { return !rig.ext->dsStates.back().enabled; }));
    CHECK(rig.logger.count(LogLevel::Warn) >= 1);

    // power comes back: the requested (enabled) state is restored
    rig.ext->outputs = {{DeviceId{105}, 1, 0.0}, {DeviceId{101}, 1, 0.0}, {DeviceId{102}, 1, 0.0}, {DeviceId{103}, 1, 0.0}, {DeviceId{104}, 1, 0.0}};
    const bool restored = rig.run([&] { return rig.ext->dsStates.back().enabled; });
    INFO("battery " << rig.session->battery().voltageV() << " V, browned out " << rig.session->battery().brownedOut() << ", current " << rig.session->battery().currentA() << " A, ticks " << rig.runner->ticksSent());
    REQUIRE(restored);
}

TEST_CASE("runner: the game plugin's FMS message overrides DsState::gameSpecificMessage and follows Reset") {
    struct MessageState final : IGameState {
        std::string_view pluginId() const override { return "msg_game"; }
        void reset(SimContext&) override { steps = 0; }
        void step(SimContext&, double) override { ++steps; }
        void publish(SnapshotBuilder&) const override {}
        std::optional<std::string> gameSpecificMessage() const override {
            if (steps < 3) return std::nullopt;
            return std::string("R");
        }
        int steps = 0;
    };
    struct MessageModule final : testing::FakeGameModule {
        MessageModule() : FakeGameModule("msg_game", 3001) {}
        Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override {
            return std::unique_ptr<IGameState>(std::make_unique<MessageState>());
        }
    };

    RobotConfig c = sampleConfig();
    c.gamePluginId = "msg_game";
    Rig rig(Rig::fast(), c, [](Registries& r) { REQUIRE(r.plugins.add(std::make_unique<MessageModule>()).ok()); });
    rig.connect();
    rig.ext->hello();
    rig.ext->announceAll();
    REQUIRE(rig.run([&] { return !rig.ext->dsStates.empty() && rig.ext->dsStates.back().gameSpecificMessage == "R"; }));
    CHECK(rig.ext->dsStates.front().gameSpecificMessage.empty());  // sent at Hello, before the game had one
    CHECK(rig.ext->dsStates.back().sessionId == 1);
    CHECK(rig.session->driverStation().gameSpecificMessage == "R");

    // the UI's own text never beats the game's FMS data
    proto::DsState ds;
    ds.gameSpecificMessage = "manual";
    rig.runner->setDriverStation(ds);
    for (int i = 0; i < 20; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.ext->dsStates.back().gameSpecificMessage == "R");

    // Reset clears the game's message, the new epoch re-latches it
    const std::size_t before = rig.ext->dsStates.size();
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    rig.runner->control(reset);
    REQUIRE(rig.run([&] { return rig.ext->dsStates.size() > before && rig.ext->dsStates.back().runEpoch == 2; }));
    CHECK(rig.ext->dsStates[before].gameSpecificMessage == "manual");
    REQUIRE(rig.run([&] { return rig.ext->dsStates.back().gameSpecificMessage == "R" && rig.ext->dsStates.back().runEpoch == 2; }));
}

TEST_CASE("runner: without a game message the UI's gameSpecificMessage is forwarded unchanged") {
    Rig rig;
    rig.connect();
    rig.ext->hello();
    rig.ext->announceAll();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 2; }));
    proto::DsState ds;
    ds.gameSpecificMessage = "RBG";
    rig.runner->setDriverStation(ds);
    REQUIRE(rig.run([&] { return !rig.ext->dsStates.empty() && rig.ext->dsStates.back().gameSpecificMessage == "RBG"; }));
}

TEST_CASE("runner: live bind/unbind through the controller resends the loopback set") {
    Rig rig;
    rig.connect();
    rig.ext->hello();
    rig.ext->announceAll();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 2; }));
    CHECK(rig.ext->loopbacks.back().loopback.empty());

    rig.runner->unbindSlot({"intake", "motor"});
    REQUIRE(rig.run([&] { return rig.ext->loopbacks.back().loopback.size() == 1; }));
    CHECK(rig.ext->loopbacks.back().loopback[0] == DeviceId{105});

    rig.runner->bindSlot({"intake", "motor"}, DeviceKey{"phoenix6", "talonfx", 10, "rio", -1});
    REQUIRE(rig.run([&] { return rig.ext->loopbacks.back().loopback.empty(); }));

    const std::size_t count = rig.ext->loopbacks.size();
    rig.runner->bindSlot({"intake", "motor"}, DeviceKey{"phoenix6", "talonfx", 10, "rio", -1});  // no change: nothing resent
    rig.runner->bindSlot({"x", "y"}, DeviceKey{"phoenix6", "talonfx", 1, "rio", -1});              // device already bound elsewhere
    for (int i = 0; i < 20; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.ext->loopbacks.size() == count);
    CHECK(rig.logger.count(LogLevel::Warn) >= 1);
}

TEST_CASE("runner: real-time pacing releases one tick per step interval of wall time") {
    RunnerOptions o = Rig::fast();
    o.timeScale = 1.0;
    Rig rig(o);
    rig.connect();
    rig.ext->hello();
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
    for (int i = 0; i < 10; ++i) {
        rig.runner->poll(1ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == 1);  // the first tick is due immediately, the next one 5 ms later
    rig.clock.advance(4ms);
    for (int i = 0; i < 10; ++i) {
        rig.runner->poll(1ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == 1);
    rig.clock.advance(1ms);
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == 2; }));

    // half speed: 10 ms between ticks
    proto::SimControl scale;
    scale.command = proto::SimCommand::SetTimeScale;
    scale.timeScale = 0.5;
    rig.runner->control(scale);
    for (int i = 0; i < 10; ++i) {
        rig.runner->poll(1ms);
        rig.ext->pump();
    }
    const std::uint64_t t0 = rig.runner->ticksSent();
    rig.clock.advance(9ms);
    for (int i = 0; i < 10; ++i) {
        rig.runner->poll(1ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == t0);
    rig.clock.advance(1ms);
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == t0 + 1; }));

    // falling far behind does not cause a burst of catch-up ticks
    rig.clock.advance(10s);
    for (int i = 0; i < 10; ++i) {
        rig.runner->poll(1ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() <= t0 + 2);
}

TEST_CASE("runner: discovery-only runs announce devices but never step") {
    RunnerOptions o = Rig::fast();
    o.discoveryOnly = true;
    Rig rig(o);
    rig.connect();
    rig.ext->hello();
    rig.ext->announceAll();
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value() && rig.session->hub().discovered().size() == 7; }));
    CHECK(rig.ext->helloAck->discoveryOnly);
    for (int i = 0; i < 20; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == 0);
    CHECK(rig.ext->sensorFrames.empty());
    REQUIRE(rig.run([&] { return rig.slot.latest()->devices().size() == 7; }));
}

TEST_CASE("runner: an armed run steps from tick one after BeginLockstep and returns to armed on EndLockstep") {
    RunnerOptions o = Rig::fast();
    o.discoveryOnly = true;
    Rig rig(o);
    rig.connect();
    rig.ext->hello();
    rig.ext->announceAll();
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value() && rig.session->hub().discovered().size() == 7; }));
    CHECK(rig.runner->armed());

    proto::SimControl begin;
    begin.command = proto::SimCommand::BeginLockstep;
    rig.runner->control(begin);
    REQUIRE(rig.run([&] { return !rig.ext->controls.empty(); }));
    CHECK(rig.ext->controls.back().command == proto::SimCommand::BeginLockstep);
    CHECK(rig.ext->controls.back().commandId != 0);
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 1; }));
    CHECK_FALSE(rig.runner->armed());
    CHECK(rig.ext->sensorFrames.front().runEpoch == rig.ext->controls.back().runEpoch);
    rig.ext->outputs = {{DeviceId{105}, 1, 12.0}};
    REQUIRE(rig.run([&] { return rig.session->hub().motorOutput({"intake", "motor"}).valid; }));
    CHECK(rig.session->hub().motorOutput({"intake", "motor"}).voltageV == doctest::Approx(12));

    proto::SimControl end;
    end.command = proto::SimCommand::EndLockstep;
    rig.runner->control(end);
    REQUIRE(rig.run([&] { return rig.runner->armed(); }));
    CHECK(rig.ext->controls.back().command == proto::SimCommand::EndLockstep);
    const std::uint64_t ticks = rig.runner->ticksSent();
    for (int i = 0; i < 20; ++i) {
        rig.runner->poll(2ms);
        rig.ext->pump();
    }
    CHECK(rig.runner->ticksSent() == ticks);
    CHECK_FALSE(rig.session->hub().motorOutput({"intake", "motor"}).valid);
    rig.ext->outputs.clear();
    rig.runner->control(begin);
    REQUIRE(rig.run([&] { return !rig.runner->armed(); }));
    CHECK_FALSE(rig.session->hub().motorOutput({"intake", "motor"}).valid);
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() > ticks; }));
    CHECK(rig.session->hub().motorOutput({"intake", "motor"}).voltageV == doctest::Approx(0));
}

TEST_CASE("runner: peers without explicit Armed support never receive Begin or End") {
    SUBCASE("discovery handshake is refused conservatively") {
        auto options = Rig::fast();
        options.discoveryOnly = true;
        Rig rig(options);
        rig.connect();
        rig.ext->hello(0, 2, false);
        REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
        CHECK_FALSE(rig.ext->helloAck->accepted);
        CHECK(rig.ext->helloAck->reason.find("supportsArmed") != std::string::npos);
        for (const auto command :
             {proto::SimCommand::BeginLockstep, proto::SimCommand::EndLockstep}) {
            proto::SimControl c;
            c.command = command;
            rig.runner->control(c);
        }
        rig.runner->poll(0ms);
        rig.ext->pump();
        CHECK(rig.ext->controls.empty());
        CHECK(rig.runner->ticksSent() == 0);
        CHECK(rig.runner->state() == RunnerState::WaitingForRobot);
    }
    SUBCASE("legacy normal lockstep remains usable but transitions are gated") {
        Rig rig;
        rig.connect();
        rig.ext->hello(0, 2, false);
        REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 1; }));
        CHECK(rig.ext->helloAck->accepted);
        const auto epoch = rig.runner->runEpoch();
        for (const auto command :
             {proto::SimCommand::BeginLockstep, proto::SimCommand::EndLockstep}) {
            proto::SimControl c;
            c.command = command;
            rig.runner->control(c);
        }
        rig.runner->poll(0ms);
        rig.ext->pump();
        CHECK(rig.ext->controls.empty());
        CHECK(rig.runner->runEpoch() == epoch);
        CHECK_FALSE(rig.runner->armed());
    }
}

TEST_CASE("runner: rejected or timed-out Begin leaves Armed without sending ticks") {
    bool timeout = false;
    SUBCASE("rejection") {}
    SUBCASE("timeout") {
        timeout = true;
    }
    auto options = Rig::fast();
    options.discoveryOnly = true;
    Rig rig(options);
    rig.connect();
    rig.ext->hello();
    REQUIRE(rig.run([&] { return rig.ext->helloAck.has_value(); }));
    if (timeout)
        rig.ext->ackControls = false;
    else
        rig.ext->controlResult = proto::ControlResult::Rejected;
    proto::SimControl begin;
    begin.command = proto::SimCommand::BeginLockstep;
    rig.runner->control(begin);
    REQUIRE(rig.run([&] { return !rig.ext->controls.empty(); }));
    if (timeout) {
        rig.clock.advance(61s);
        rig.runner->poll(0ms);
        CHECK(rig.runner->state() == RunnerState::RobotStalled);
    } else {
        REQUIRE(rig.run([&] { return rig.runner->armed(); }));
    }
    CHECK(rig.runner->armed());
    CHECK(rig.runner->ticksSent() == 0);
    CHECK(rig.ext->sensorFrames.empty());
}
