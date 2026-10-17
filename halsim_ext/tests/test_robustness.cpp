#include <doctest/doctest.h>

#include "fakes.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

namespace {
struct Started {
    explicit Started(proto::HelloAck ack = defaultAck(), ExtensionConfig cfg = Rig::defaultConfig()) : rig(std::move(ack), std::move(cfg)) {
        REQUIRE(rig.start());
        rig.clock.started = true;
        REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    }
    Rig rig;
};

struct ConsoleGate {
    void write(std::string_view) {
        std::unique_lock lock(mutex);
        if (!enabled) return;
        entered = true;
        cv.notify_all();
        cv.wait(lock, [&] { return released; });
    }
    void release() {
        std::lock_guard lock(mutex);
        released = true;
        cv.notify_all();
    }
    std::mutex mutex;
    std::condition_variable cv;
    bool enabled = false;
    bool entered = false;
    bool released = false;
};
}  // namespace

TEST_CASE("watchdog: a robot that never finishes the step is reported RobotStalled and the robot is unfrozen") {
    Started t;  // 400 ms watchdog
    REQUIRE(t.rig.emulator->tick());
    t.rig.clock.blockNextStep = true;

    const auto start = std::chrono::steady_clock::now();
    const auto ack = t.rig.emulator->tick({}, 5000, 12.0, 3000ms);
    const auto waited = std::chrono::steady_clock::now() - start;
    REQUIRE(ack.has_value());
    CHECK(ack->result == proto::TickResult::RobotStalled);
    CHECK(ack->ackedTick() == 2);
    CHECK(ack->sessionId == 7);
    CHECK(waited >= 300ms);
    CHECK(waited < 2500ms);
    CHECK(t.rig.clock.resumeCalls >= 1);
    CHECK_FALSE(t.rig.clock.paused);
    CHECK(t.rig.runtime->stats().stalls == 1);
    CHECK(t.rig.process.calls == 0);  // default policy resumes real time, it does not kill the robot

    // Lock-step is over until a Reset: later frames are answered RobotStalled without stepping.
    t.rig.clock.release();
    const auto later = t.rig.emulator->tick();
    REQUIRE(later);
    CHECK(later->result == proto::TickResult::RobotStalled);

    // A Reset re-engages lock-step (HAL time is paused again) and ticks work again.
    const int pausesBefore = t.rig.clock.pauseCalls;
    t.rig.emulator->epoch = 2;
    t.rig.emulator->nextTick = 1;
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    reset.commandId = 3;
    reset.sessionId = 7;
    reset.runEpoch = 2;
    REQUIRE(t.rig.emulator->send(proto::Message(reset)));
    REQUIRE(t.rig.emulator->take<proto::SimControlAck>(1000ms));
    CHECK(t.rig.clock.pauseCalls == pausesBefore + 1);
    const auto recovered = t.rig.emulator->tick();
    REQUIRE(recovered);
    CHECK(recovered->result == proto::TickResult::Completed);
}

TEST_CASE("watchdog: a silent emulator makes the extension resume real-time timing by itself") {
    Started t;
    REQUIRE(t.rig.emulator->tick());
    CHECK(t.rig.clock.paused);
    REQUIRE(waitUntil([&] { return !t.rig.clock.paused; }, 2500ms));
    CHECK(t.rig.runtime->stats().emulatorLost == 1);
    CHECK(t.rig.emulator->take<proto::Log>(1000ms, [](const proto::Log& log) {
        return log.message.find("watchdog") != std::string::npos;
    }));
    // The emulator that comes back late is told the lock-step is over.
    const auto late = t.rig.emulator->tick();
    REQUIRE(late);
    CHECK(late->result == proto::TickResult::Stopped);
}

TEST_CASE("watchdog: an emulator that announced Pause is not considered lost") {
    Started t;
    REQUIRE(t.rig.emulator->tick());
    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Pause, 1));
    REQUIRE(t.rig.emulator->take<proto::SimControlAck>(1000ms));
    std::this_thread::sleep_for(1000ms);  // > 2 x watchdog
    CHECK(t.rig.clock.paused);
    CHECK(t.rig.runtime->stats().emulatorLost == 0);

    // Resume re-arms it.
    REQUIRE(t.rig.emulator->sendControl(proto::SimCommand::Resume, 2));
    REQUIRE(t.rig.emulator->take<proto::SimControlAck>(1000ms));
    CHECK(waitUntil([&] { return !t.rig.clock.paused; }, 2500ms));
}

TEST_CASE("watchdog: the timeout is the negotiated one and WPIEMU_WATCHDOG=off disables it") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.watchdogEnabled = false;
    Started t(defaultAck(), cfg);
    REQUIRE(t.rig.emulator->tick());
    std::this_thread::sleep_for(800ms);
    CHECK(t.rig.clock.paused);
    CHECK(t.rig.runtime->stats().emulatorLost == 0);
}

TEST_CASE("watchdog: Terminate policy ends the robot process instead of resuming time") {
    proto::HelloAck ack = defaultAck();
    ack.watchdogPolicy = proto::WatchdogPolicy::Terminate;
    Started t(ack);
    REQUIRE(t.rig.emulator->tick());
    REQUIRE(waitUntil([&] { return t.rig.process.calls > 0; }, 2500ms));
    CHECK(t.rig.process.lastCode == 0);
    CHECK(t.rig.clock.resumeCalls == 0);
}

TEST_CASE("disconnect: losing the emulator resumes HAL timing immediately and ends the runtime") {
    Started t;
    REQUIRE(t.rig.emulator->tick());
    t.rig.emulator->disconnect();
    REQUIRE(t.rig.runtime->waitFinished(2000ms));
    CHECK_FALSE(t.rig.clock.paused);
    CHECK(t.rig.clock.resumeCalls >= 1);
    CHECK(t.rig.process.calls == 0);
}

TEST_CASE("disconnect: Terminate policy ends the process on a lost connection") {
    proto::HelloAck ack = defaultAck();
    ack.watchdogPolicy = proto::WatchdogPolicy::Terminate;
    Started t(ack);
    t.rig.emulator->disconnect();
    REQUIRE(t.rig.runtime->waitFinished(2000ms));
    CHECK(t.rig.process.calls == 1);
}

TEST_CASE("watchdog: recovery never waits for a blocked console or logger mutex") {
    bool terminate = false;
    bool holdLogger = false;
    SUBCASE("resume without another logger") {}
    SUBCASE("terminate without another logger") {
        terminate = true;
    }
    SUBCASE("resume while another logger owns the mutex") {
        holdLogger = true;
    }
    SUBCASE("terminate while another logger owns the mutex") {
        terminate = holdLogger = true;
    }
    ConsoleGate gate;
    auto ack = defaultAck();
    if (terminate) ack.watchdogPolicy = proto::WatchdogPolicy::Terminate;
    Rig rig(ack);
    rig.console = [&](std::string_view line) { gate.write(line); };
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    rig.clock.blockNextStep = true;
    proto::SensorFrame frame;
    frame.tick = 1;
    frame.sessionId = 7;
    frame.runEpoch = 1;
    REQUIRE(rig.emulator->send(proto::Message(frame)));
    REQUIRE(waitUntil([&] { return rig.clock.blockedSteps == 1; }));
    {
        std::lock_guard lock(gate.mutex);
        gate.enabled = true;
    }
    std::thread writer;
    if (holdLogger) {
        writer = std::thread([&] { rig.runtime->logger().warn("test", "console is blocked"); });
        std::unique_lock lock(gate.mutex);
        CHECK(gate.cv.wait_for(lock, 1000ms, [&] { return gate.entered; }));
    }
    CHECK(waitUntil([&] { return terminate ? rig.process.calls > 0 : rig.clock.resumeCalls > 0; },
                    2500ms));
    // Always release before any failing assertion can unwind borrowed service lifetimes.
    gate.release();
    if (writer.joinable()) writer.join();
    rig.clock.release();
    rig.runtime->stop();
    CHECK(rig.runtime->finished());
}

TEST_CASE("disconnect: recovery precedes diagnostics even while the console owns the logger") {
    bool terminate = false;
    SUBCASE("resume") {}
    SUBCASE("terminate") {
        terminate = true;
    }
    ConsoleGate gate;
    auto ack = defaultAck();
    if (terminate) ack.watchdogPolicy = proto::WatchdogPolicy::Terminate;
    Rig rig(ack);
    rig.console = [&](std::string_view line) { gate.write(line); };
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    {
        std::lock_guard lock(gate.mutex);
        gate.enabled = true;
    }
    std::thread writer([&] { rig.runtime->logger().warn("test", "console is blocked"); });
    {
        std::unique_lock lock(gate.mutex);
        CHECK(gate.cv.wait_for(lock, 1000ms, [&] { return gate.entered; }));
    }
    rig.emulator->disconnect();
    CHECK(waitUntil([&] { return terminate ? rig.process.calls > 0 : rig.clock.resumeCalls > 0; },
                    2000ms));
    gate.release();
    writer.join();
    CHECK(rig.runtime->waitFinished(2000ms));
}

TEST_CASE("stop(): bounded detach does not log through a blocked console") {
    ConsoleGate gate;
    Rig rig;
    rig.console = [&](std::string_view line) { gate.write(line); };
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    {
        std::lock_guard lock(gate.mutex);
        gate.enabled = true;
    }
    std::thread writer([&] { rig.runtime->logger().warn("test", "console is blocked"); });
    {
        std::unique_lock lock(gate.mutex);
        CHECK(gate.cv.wait_for(lock, 1000ms, [&] { return gate.entered; }));
    }
    std::atomic<bool> stopped{false};
    std::thread stopper([&] {
        rig.runtime->stop();
        stopped = true;
    });
    CHECK(waitUntil([&] { return stopped.load(); }, 4500ms));
    CHECK_FALSE(rig.clock.paused);
    gate.release();
    writer.join();
    stopper.join();
    CHECK(rig.runtime->waitFinished(2000ms));
}

TEST_CASE("disconnect before the robot program started still unfreezes the robot") {
    Rig rig;
    REQUIRE(rig.start());
    CHECK(rig.clock.paused);
    rig.emulator->disconnect();
    REQUIRE(rig.runtime->waitFinished(2000ms));
    CHECK_FALSE(rig.clock.paused);
}

TEST_CASE("an emulator Goodbye is treated like a disconnect") {
    Started t;
    REQUIRE(t.rig.emulator->send(proto::Message(proto::Goodbye{"closing"})));
    REQUIRE(t.rig.runtime->waitFinished(2000ms));
    CHECK_FALSE(t.rig.clock.paused);
}

TEST_CASE("stop(): resumes HAL timing, says Goodbye and joins the clock thread") {
    Started t;
    REQUIRE(t.rig.emulator->tick());
    t.rig.runtime->stop();
    CHECK(t.rig.runtime->finished());
    CHECK_FALSE(t.rig.clock.paused);
    CHECK(t.rig.emulator->take<proto::Goodbye>(1000ms).has_value());
    t.rig.runtime->stop();  // idempotent
}

TEST_CASE("stop() while the robot is stuck inside a step unblocks it") {
    Started t;
    t.rig.clock.blockNextStep = true;
    proto::SensorFrame f;
    f.tick = 1;
    f.simTimeUs = 5000;
    f.sessionId = 7;
    f.runEpoch = 1;
    REQUIRE(t.rig.emulator->send(proto::Message(f)));
    REQUIRE(waitUntil([&] { return t.rig.clock.blockedSteps == 1; }));
    t.rig.runtime->stop();
    CHECK(t.rig.runtime->finished());
}

TEST_CASE("a bridge that cannot start does not stop the session") {
    Rig rig;
    FakeBridge& broken = rig.addBridge("broken");
    FakeBridge& good = rig.addBridge("good");
    broken.failStart = true;
    good.queueDevice(FakeBridge::makeDescriptor("pwm", 0));
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    CHECK(rig.emulator->take<proto::DeviceAnnounce>(1000ms).has_value());
    CHECK(rig.logContains("broken"));
    CHECK(rig.emulator->tick().has_value());
}
