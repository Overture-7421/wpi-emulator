#pragma once

// The SessionRunner test rig, shared by test_session_runner.cpp, test_runner_control.cpp and test_runner_vision.cpp: a manually
// advanced wall clock, the robot-process side of the protocol, and a Rig that wires a session, the runner and an in-memory
// link together.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "runtime_fixtures.h"
#include "wpiemu/core/runtime/session_runner.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

namespace wpiemu::test_runtime {

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using namespace std::chrono_literals;

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
                trace.push_back("ack");
                if (ack->accepted) follower.begin(ack->sessionId, ack->runEpoch);
            } else if (const auto* sf = std::get_if<proto::SensorFrame>(&m)) {
                sensorFrames.push_back(*sf);
                trace.push_back("frame");
                if (autoAck) answer(*sf, nextResult);
                else pending = *sf;
            } else if (const auto* vs = std::get_if<VisionSetup>(&m)) {
                visionSetups.push_back(*vs);
                trace.push_back("vision");
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
    std::vector<VisionSetup> visionSetups;
    std::vector<std::string> trace;  // "ack", "vision", "frame" in arrival order
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

}  // namespace wpiemu::test_runtime
