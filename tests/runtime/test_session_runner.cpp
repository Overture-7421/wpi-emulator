#include <doctest/doctest.h>

#include <algorithm>
#include <atomic>
#include <deque>
#include <functional>
#include <thread>

#include "runtime_fixtures.h"
#include "wpiemu/core/runtime/session_runner.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using namespace std::chrono_literals;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;
using wpiemu::test_runtime::sampleDevices;

namespace {

class ManualClock final : public IWallClock {
public:
    std::chrono::steady_clock::time_point now() const override { return t_; }
    void advance(std::chrono::milliseconds d) { t_ += d; }

private:
    std::chrono::steady_clock::time_point t_ = std::chrono::steady_clock::time_point{} + 10s;
};

// The robot-process side: speaks the protocol through the same in-memory pair, using the SDK's LockstepFollower.
struct FakeExtension {
    FakeExtension(std::unique_ptr<proto::ITransport> transport, std::shared_ptr<const proto::IMessageCodec> codec)
        : ch(std::move(transport), std::move(codec)) {}

    void send(const proto::Message& m) { REQUIRE(ch.send(m).ok()); }
    void hello(std::uint32_t proposedWatchdogMs = 0,
               std::uint32_t version = proto::kProtocolVersion, bool supportsArmed = true) {
        proto::Hello h;
        h.protocolVersion = version;
        h.language = proto::RobotLanguage::Java;
        h.wpilibYear = 2026;
        h.wpilibVersion = "2026.2.1";
        h.pid = 4242;
        h.programName = "TestRobot";
        h.proposedWatchdogMs = proposedWatchdogMs;
        h.supportsArmed = supportsArmed;
        send(h);
    }
    void announceAll() {
        for (const auto& a : sampleDevices()) send(a);
    }

    void pump() {
        for (;;) {
            proto::Message m;
            const Result<bool> got = ch.receive(m, 0ms);
            if (!got) {
                disconnected = true;
                break;
            }
            if (!got.value()) break;
            lastRxAt = clock != nullptr ? clock->now() : lastRxAt;
            if (const auto* ack = std::get_if<proto::HelloAck>(&m)) {
                helloAck = *ack;
                if (ack->accepted) follower.begin(ack->sessionId, ack->runEpoch);
            } else if (const auto* sf = std::get_if<proto::SensorFrame>(&m)) {
                sensorFrames.push_back(*sf);
                if (autoAck) answer(*sf, nextResult);
                else pending = *sf;
            } else if (const auto* sc = std::get_if<proto::SimControl>(&m)) {
                controls.push_back(*sc);
                const proto::ControlDecision d = follower.onControl(*sc, [&] {
                    return std::pair{controlResult, controlResult == proto::ControlResult::Applied
                                                        ? std::string{}
                                                        : std::string("scripted control result")};
                });
                if (d.apply && sc->command == proto::SimCommand::Pause) remotePaused = true;
                if (d.apply && sc->command == proto::SimCommand::Resume) remotePaused = false;
                if (d.sendAck && ackControls) send(d.ack);
            } else if (const auto* ds = std::get_if<proto::DsState>(&m)) {
                dsStates.push_back(*ds);
            } else if (const auto* lb = std::get_if<proto::LoopbackSet>(&m)) {
                loopbacks.push_back(*lb);
            }
        }
    }
    void answer(const proto::SensorFrame& sf, proto::TickResult result = proto::TickResult::Completed) {
        if (follower.onSensorFrame(sf) != proto::FollowerAction::Step) return;
        proto::ActuatorFrame a = follower.makeAck(sf, result);
        a.samples = outputs;
        send(a);
    }

    // Mirrors the extension's watchdog rule (sim_session.cpp): armed only while not paused and no tick is in flight.
    bool watchdogWouldFire(std::chrono::milliseconds timeout, std::chrono::steady_clock::time_point now) const {
        return !remotePaused && !pending.has_value() && now - lastRxAt > timeout;
    }

    proto::MessageChannel ch;
    proto::LockstepFollower follower;
    const ManualClock* clock = nullptr;
    std::chrono::steady_clock::time_point lastRxAt{};
    bool remotePaused = false;
    bool autoAck = true;
    bool ackControls = true;
    proto::ControlResult controlResult = proto::ControlResult::Applied;
    bool disconnected = false;
    proto::TickResult nextResult = proto::TickResult::Completed;
    std::optional<proto::HelloAck> helloAck;
    std::optional<proto::SensorFrame> pending;
    std::vector<proto::SensorFrame> sensorFrames;
    std::vector<proto::SimControl> controls;
    std::vector<proto::DsState> dsStates;
    std::vector<proto::LoopbackSet> loopbacks;
    std::vector<SignalSample> outputs;
};

struct Rig {
    Registries reg;
    testing::CapturingLogger logger;
    std::unique_ptr<SimulationSession> session;
    MessageInbox inbox;
    LatestSnapshotSlot slot;
    ManualClock clock;
    std::unique_ptr<SessionRunner> runner;
    std::shared_ptr<testing::MemoryMessageCodec> codec = std::make_shared<testing::MemoryMessageCodec>();
    std::shared_ptr<ChannelRobotLink> link;
    std::unique_ptr<FakeExtension> ext;
    std::uint64_t nextLinkId = 1;

    static RunnerOptions fast() {
        RunnerOptions o;
        o.timeScale = 0.0;  // unpaced: ticks as soon as the previous one is acknowledged
        o.snapshotMinInterval = 0ms;
        return o;
    }

    explicit Rig(RunnerOptions options = fast(), RobotConfig config = sampleConfig(),
                 const std::function<void(Registries&)>& prepare = {}) {
        if (prepare) prepare(reg);
        SessionParts p;
        p.world = std::make_unique<testing::FakePhysicsWorld>(config.sim.physics);
        p.config = std::move(config);
        p.mechanisms = &reg.mechanisms;
        p.drivetrains = &reg.drivetrains;
        p.tireModels = &reg.tires;
        p.plugins = &reg.plugins;
        p.logger = &logger;
        auto built = SessionBuilder::build(std::move(p));
        REQUIRE(built.ok());
        session = std::move(built).value();
        runner = std::make_unique<SessionRunner>(*session, inbox, slot, logger, options, &clock);
    }
    ~Rig() {
        runner.reset();
        if (link) link->close();
    }

    void connect() {
        auto pair = testing::makeInMemoryTransportPair();
        link = std::make_shared<ChannelRobotLink>(nextLinkId++, std::move(pair.a), codec, inbox);
        InboundEvent ev;
        ev.kind = InboundEvent::Kind::Connected;
        ev.linkId = link->id();
        ev.link = link;
        inbox.push(std::move(ev));
        link->start();
        ext = std::make_unique<FakeExtension>(std::move(pair.b), codec);
        ext->clock = &clock;
        ext->lastRxAt = clock.now();
    }

    bool run(const std::function<bool()>& done, int maxIterations = 300) {
        for (int i = 0; i < maxIterations; ++i) {
            runner->poll(10ms);
            ext->pump();
            if (done()) return true;
        }
        return false;
    }
    // connect + Hello + announce, until the first tick is acknowledged
    void handshake(std::uint32_t proposedWatchdogMs = 0) {
        connect();
        ext->hello(proposedWatchdogMs);
        ext->announceAll();
        REQUIRE(run([&] { return ext->helloAck.has_value() && runner->ticksSent() >= 1 && session->hub().discovered().size() == 7; }));
    }
    std::size_t warnings(const char* code) const {
        const auto snap = slot.latest();
        std::size_t n = 0;
        for (const auto& w : snap->warnings()) n += w.code == code ? 1u : 0u;
        return n;
    }
};

}  // namespace

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

TEST_CASE("runner: setRobotPose is queued for the sim thread and lands on the next poll, with or without a robot") {
    Rig rig;
    const Pose3 first{Vec3{4.f, -1.f, 0.25f}, Quat::fromYaw(0.9f)};
    rig.runner->setRobotPose(first);
    CHECK(rig.session->world().pose(rig.session->chassisBody()).position.x == 0.f);   // the controller side only enqueues
    rig.runner->poll(0ms);
    CHECK(rig.session->world().pose(rig.session->chassisBody()).position == first.position);
    REQUIRE(rig.slot.latest() != nullptr);
    CHECK(rig.slot.latest()->chassis().pose.position == first.position);              // the UI's snapshot is refreshed
    CHECK(rig.slot.latest()->chassis().pose.rotation.yaw() == doctest::Approx(0.9f).epsilon(1e-5));
    CHECK(rig.runner->ticksSent() == 0);

    // a robot connects and runs: later teleports land between ticks, and the robot code sees the new heading
    rig.handshake();
    rig.runner->setRobotPose(Pose3{Vec3{-2.f, 3.f, 0.25f}, Quat::fromYaw(-0.4f)});
    REQUIRE(rig.run([&] { return rig.slot.latest()->chassis().pose.position.x == -2.f; }));
    std::size_t before = rig.ext->sensorFrames.size();
    REQUIRE(rig.run([&] { return rig.ext->sensorFrames.size() > before + 1; }));
    bool sawYaw = false;
    for (const auto& frame : rig.ext->sensorFrames)
        for (const auto& s : frame.samples)
            if (s.device == DeviceId{107} && s.signal == 0 && std::abs(s.value + 0.4) < 1e-4) sawYaw = true;   // pigeon yaw
    CHECK(sawYaw);
}

TEST_CASE("runner: setRobotPose also works while paused and the next Step starts from the new pose") {
    Rig rig;
    rig.handshake();
    proto::SimControl pause;
    pause.command = proto::SimCommand::Pause;
    rig.runner->control(pause);
    REQUIRE(rig.run([&] { return rig.runner->state() == RunnerState::Paused; }));
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == rig.ext->sensorFrames.size(); }));
    const std::uint64_t paused = rig.runner->ticksSent();

    rig.runner->setRobotPose(Pose3{Vec3{6.f, 2.f, 0.25f}, Quat::fromYaw(1.5f)});
    REQUIRE(rig.run([&] { return rig.slot.latest()->chassis().pose.position.x == 6.f; }));
    CHECK(rig.slot.latest()->paused());
    CHECK(rig.runner->state() == RunnerState::Paused);
    CHECK(rig.runner->ticksSent() == paused);   // no tick is needed to apply it
    CHECK(rig.session->world().linearVelocity(rig.session->chassisBody()) == Vec3{});

    proto::SimControl step;
    step.command = proto::SimCommand::Step;
    step.stepCount = 1;
    rig.runner->control(step);
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() == paused + 1; }));
    CHECK(rig.session->world().pose(rig.session->chassisBody()).position.x == doctest::Approx(6.f));
    CHECK(rig.session->world().pose(rig.session->chassisBody()).rotation.yaw() == doctest::Approx(1.5f).epsilon(1e-4));
}

TEST_CASE("runner: Reset after a teleport returns to the configured start pose") {
    RobotConfig c = sampleConfig();
    c.sim.startPose = StartPose{2.f, 1.f, 0.3f};
    Rig rig(Rig::fast(), c);
    rig.handshake();
    CHECK(rig.session->world().pose(rig.session->chassisBody()).position.x == doctest::Approx(2.f));

    rig.runner->setRobotPose(Pose3{Vec3{9.f, 8.f, 0.25f}, Quat{}});
    REQUIRE(rig.run([&] { return rig.slot.latest()->chassis().pose.position.x == 9.f; }));
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    rig.runner->control(reset);
    REQUIRE(rig.run([&] { return rig.runner->runEpoch() == 2 && rig.slot.latest()->chassis().pose.position.x == doctest::Approx(2.f); }));
    CHECK(rig.session->world().pose(rig.session->chassisBody()).position.y == doctest::Approx(1.f));
    CHECK(rig.session->world().pose(rig.session->chassisBody()).rotation.yaw() == doctest::Approx(0.3f).epsilon(1e-4));
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

TEST_CASE("inbox: multi-producer order per producer, timeouts and wake") {
    MessageInbox inbox;
    InboundEvent out;
    CHECK_FALSE(inbox.tryPop(out));
    const auto t0 = std::chrono::steady_clock::now();
    CHECK_FALSE(inbox.waitPop(out, 30ms));
    CHECK(std::chrono::steady_clock::now() - t0 >= 25ms);

    std::thread waiter([&] {
        InboundEvent e;
        CHECK_FALSE(inbox.waitPop(e, 5s));  // returns early because of wake()
    });
    std::this_thread::sleep_for(20ms);
    const auto w0 = std::chrono::steady_clock::now();
    inbox.wake();
    waiter.join();
    CHECK(std::chrono::steady_clock::now() - w0 < 2s);

    constexpr int kProducers = 4, kEach = 500;
    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p)
        producers.emplace_back([&inbox, p] {
            for (int i = 0; i < kEach; ++i) {
                InboundEvent e;
                e.linkId = static_cast<std::uint64_t>(p);
                e.detail = std::to_string(i);
                inbox.push(std::move(e));
            }
        });
    for (auto& t : producers) t.join();
    CHECK(inbox.size() == kProducers * kEach);
    std::vector<int> next(kProducers, 0);
    int received = 0;
    while (inbox.tryPop(out)) {
        CHECK(std::stoi(out.detail) == next[out.linkId]++);
        ++received;
    }
    CHECK(received == kProducers * kEach);
}

namespace {
// Minimal in-memory ITransportServer: connect() hands the other end of a fresh pair to accept().
class LoopbackServer final : public proto::ITransportServer {
public:
    Status listen() override {
        listening = true;
        return okStatus();
    }
    std::string endpoint() const override { return "loopback:test"; }
    Result<std::unique_ptr<proto::ITransport>> accept(std::chrono::milliseconds timeout) override {
        std::unique_lock lock(m);
        cv.wait_for(lock, timeout, [&] { return !pending.empty() || closed; });
        if (closed) return makeError(ErrorCode::Cancelled, "closed");
        if (pending.empty()) return std::unique_ptr<proto::ITransport>{};
        auto t = std::move(pending.front());
        pending.pop_front();
        return t;
    }
    void close() override {
        {
            std::lock_guard lock(m);
            closed = true;
        }
        cv.notify_all();
    }
    std::unique_ptr<testing::InMemoryTransport> connectClient() {
        auto pair = testing::makeInMemoryTransportPair();
        {
            std::lock_guard lock(m);
            pending.push_back(std::move(pair.a));
        }
        cv.notify_all();
        return std::move(pair.b);
    }
    std::mutex m;
    std::condition_variable cv;
    std::deque<std::unique_ptr<proto::ITransport>> pending;
    bool closed = false;
    bool listening = false;
};
}  // namespace

TEST_CASE("acceptor: accepted transports become Connected events followed by their messages") {
    MessageInbox inbox;
    testing::CapturingLogger logger;
    auto codec = std::make_shared<testing::MemoryMessageCodec>();
    auto server = std::make_unique<LoopbackServer>();
    LoopbackServer* raw = server.get();
    RobotLinkAcceptor acceptor(std::move(server), codec, inbox, &logger);
    REQUIRE(acceptor.start().ok());
    CHECK(raw->listening);
    CHECK(acceptor.endpoint() == "loopback:test");
    CHECK_FALSE(acceptor.start().ok());  // already running

    proto::MessageChannel client(raw->connectClient(), codec);
    InboundEvent ev;
    REQUIRE(inbox.waitPop(ev, 2s));
    CHECK(ev.kind == InboundEvent::Kind::Connected);
    REQUIRE(ev.link != nullptr);
    const std::shared_ptr<IRobotLink> link = ev.link;  // the runner's reference
    CHECK(link->connected());

    proto::Hello hello;
    hello.pid = 7;
    REQUIRE(client.send(hello).ok());
    REQUIRE(inbox.waitPop(ev, 2s));
    CHECK(ev.kind == InboundEvent::Kind::Message);
    CHECK(std::get<proto::Hello>(ev.message).pid == 7);

    // emulator -> robot through the link
    proto::HelloAck reply;
    reply.sessionId = 9;
    REQUIRE(link->send(reply).ok());
    proto::Message received;
    REQUIRE(client.receive(received, 2s).ok());
    CHECK(std::get<proto::HelloAck>(received).sessionId == 9);
    client.close();
    REQUIRE(inbox.waitPop(ev, 2s));
    CHECK(ev.kind == InboundEvent::Kind::Disconnected);
    CHECK_FALSE(link->send(reply).ok());
    acceptor.stop();
    acceptor.stop();  // idempotent
}
