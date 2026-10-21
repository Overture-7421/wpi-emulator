#pragma once

// The SessionRunner test rig, shared by test_session_runner.cpp, test_runner_control.cpp and test_runner_vision.cpp: a manually
// advanced wall clock, the robot-process side of the protocol, and a Rig that wires a session, the runner and an in-memory
// link together.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include "runtime_fixtures.h"
#include "wpiemu/core/runtime/session_runner.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/fake_extension.h"
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

// The robot-process side, pumped by Rig::run, that announces the devices of sampleConfig().
struct RigExtension : testing::FakeExtension {
    using testing::FakeExtension::FakeExtension;

    void announceAll() {
        for (const auto& a : sampleDevices()) REQUIRE(send(a));
    }
};
using Reply = testing::FakeExtension::Reply;

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
    std::unique_ptr<RigExtension> ext;
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
        ext = std::make_unique<RigExtension>(std::move(pair.b), codec);
        ext->clock = [this] { return clock.now(); };
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
