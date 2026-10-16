// Lock-step integration (ARCHITECTURE section 4.1): LockstepWindow (emulator) and LockstepFollower
// (extension) driven over the real transports and the real binary codec.
#include <doctest/doctest.h>

#include <algorithm>
#include <cstdio>
#include <numeric>

#include "ipc_test_support.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::ipc::testing;
using namespace wpiemu::sdk;
using namespace std::chrono_literals;
using Act = FakeExtension::Act;

namespace {

struct Session {
    Link link;
    std::unique_ptr<EmulatorSide> emulator;
    std::unique_ptr<FakeExtension> extension;

    Session(TransportKind kind, FakeExtension::SensorHook hook = {}, std::uint64_t sessionId = 1, std::uint64_t epoch = 1,
            std::uint32_t watchdogMs = 0)
        : link(kind) {
        auto pair = link.connect();
        emulator = std::make_unique<EmulatorSide>(std::move(pair.emulator));
        extension = std::make_unique<FakeExtension>(std::move(pair.extension), std::move(hook));
        const Status s = emulator->handshake(sessionId, epoch, watchdogMs);
        REQUIRE_MESSAGE(s.ok(), (s.ok() ? std::string() : s.error().message));
        REQUIRE(extension->waitHandshaken(5000ms));
    }
};

}  // namespace

TEST_CASE("lockstep: normal ticking acknowledges every tick, in order, with the right payload") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind);
        for (std::uint64_t tick = 1; tick <= 300; ++tick) {
            proto::ActuatorFrame answer;
            REQUIRE(s.emulator->tick(tick, 2000ms, &answer) == TickOutcome::Completed);
            CHECK(answer.ackedTick() == tick);
            CHECK(answer.sessionId == 1);
            CHECK(answer.runEpoch == 1);
            REQUIRE(answer.samples.size() == 1);
            CHECK(answer.samples[0].value == doctest::Approx(2.0 * static_cast<double>(tick)));
            CHECK(s.emulator->window().canSend());
        }
        CHECK(s.extension->stepped == 300);
        CHECK(s.emulator->window().droppedFrames() == 0);
    });
}

TEST_CASE("lockstep: the window refuses a second outstanding tick") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind, [](const proto::SensorFrame&, FakeExtension&) { return Act::Ignore; });
        CHECK(s.emulator->tick(1, 30ms) == TickOutcome::Timeout);
        CHECK(s.emulator->window().stalled());
        CHECK(s.emulator->tick(2, 30ms) == TickOutcome::SendFailed);  // lock-step suspended
    });
}

TEST_CASE("lockstep: round-trip latency per tick") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind);
        constexpr int kWarmup = 300;
        constexpr int kTicks = 4000;
        std::vector<double> us;
        us.reserve(kTicks);
        for (int i = 1; i <= kWarmup + kTicks; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            REQUIRE(s.emulator->tick(static_cast<std::uint64_t>(i), 2000ms) == TickOutcome::Completed);
            const auto dt = std::chrono::steady_clock::now() - t0;
            if (i > kWarmup) us.push_back(std::chrono::duration<double, std::micro>(dt).count());
        }
        std::sort(us.begin(), us.end());
        const double mean = std::accumulate(us.begin(), us.end(), 0.0) / static_cast<double>(us.size());
        const double median = us[us.size() / 2];
        const double p99 = us[us.size() * 99 / 100];
        std::printf("[ipc latency] %-10s round trip per tick (SensorFrame -> ActuatorFrame, 1 sample, %d ticks): median %.1f us, mean %.1f us, p99 %.1f us, max %.1f us\n",
                    toString(kind), kTicks, median, mean, p99, us.back());
        CHECK(median < 500.0);  // the design target; generous enough for a loaded CI machine
        CHECK(mean < 2000.0);
    });
}

TEST_CASE("lockstep: a late frame from before a Reset carries a stale epoch and is dropped, not mistaken for the current tick") {
    forEachTransportKind([](TransportKind kind) {
        // After the first tick of the new epoch, the extension first emits a stale answer (epoch 1) for the
        // very tick number that is outstanding, then the real answer. Without epoch checks the first would complete the tick.
        std::atomic<bool> inject{false};
        Session s(kind, [&](const proto::SensorFrame& sf, FakeExtension& ext) {
            if (inject) {
                proto::ActuatorFrame stale;
                stale.tick = sf.tick;
                stale.ackTick = sf.tick;
                stale.sessionId = sf.sessionId;
                stale.runEpoch = 1;  // old epoch
                stale.samples.push_back({sdk::DeviceId{1}, 0, -999.0});
                (void)ext.channel().send(stale);
            }
            return Act::Answer;
        });
        for (std::uint64_t t = 1; t <= 3; ++t) REQUIRE(s.emulator->tick(t, 2000ms) == TickOutcome::Completed);

        REQUIRE(s.emulator->reset(100).ok());
        proto::SimControlAck ack;
        REQUIRE(s.emulator->awaitAck(100, 2000ms, &ack));
        CHECK(ack.result == proto::ControlResult::Applied);
        CHECK(ack.runEpoch == 2);
        CHECK(ack.command == proto::SimCommand::Reset);

        inject = true;
        for (std::uint64_t t = 1; t <= 5; ++t) {  // tick numbering restarts per epoch
            proto::ActuatorFrame answer;
            REQUIRE(s.emulator->tick(t, 2000ms, &answer) == TickOutcome::Completed);
            CHECK(answer.runEpoch == 2);
            REQUIRE(answer.samples.size() == 1);
            CHECK(answer.samples[0].value == doctest::Approx(2.0 * static_cast<double>(t)));  // not the -999 impostor
        }
        CHECK(s.emulator->window().droppedFrames() == 5);
    });
}

TEST_CASE("lockstep: Reset while a tick is outstanding - the late answer is dropped and the new epoch proceeds") {
    forEachTransportKind([](TransportKind kind) {
        // The robot is slow on tick 10 of epoch 1; the emulator resets meanwhile.
        Session s(kind, [](const proto::SensorFrame& sf, FakeExtension&) {
            if (sf.tick == 10 && sf.runEpoch == 1) std::this_thread::sleep_for(150ms);
            return Act::Answer;
        });
        for (std::uint64_t t = 1; t <= 9; ++t) REQUIRE(s.emulator->tick(t, 2000ms) == TickOutcome::Completed);

        // Tick 10 goes out, then the user presses Reset before it is acknowledged.
        const proto::SensorFrame frame = s.emulator->makeFrame(10);
        REQUIRE(s.emulator->window().onSent(frame).ok());
        REQUIRE(s.emulator->channel().send(frame).ok());
        REQUIRE(s.emulator->reset(7).ok());  // window.begin() discards the outstanding tick
        CHECK_FALSE(s.emulator->window().outstanding());

        // A DsState queued under the old epoch and sent after the Reset must be ignored by the extension.
        proto::DsState oldDs;
        oldDs.enabled = true;
        oldDs.sessionId = s.emulator->session();
        oldDs.runEpoch = 1;
        REQUIRE(s.emulator->channel().send(oldDs).ok());
        proto::DsState newDs = oldDs;
        newDs.runEpoch = 2;
        REQUIRE(s.emulator->channel().send(newDs).ok());

        proto::SimControlAck ack;
        REQUIRE(s.emulator->awaitAck(7, 3000ms, &ack));  // the late epoch-1 ActuatorFrame passes by on the way
        CHECK(ack.result == proto::ControlResult::Applied);
        CHECK(ack.runEpoch == 2);
        CHECK(s.emulator->window().droppedFrames() == 1);

        // The new epoch ticks from 1 again.
        proto::ActuatorFrame answer;
        REQUIRE(s.emulator->tick(1, 2000ms, &answer) == TickOutcome::Completed);
        CHECK(answer.runEpoch == 2);
        const auto end = std::chrono::steady_clock::now() + 2000ms;
        while (s.extension->droppedDs + s.extension->acceptedDs < 2 && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(1ms);
        CHECK(s.extension->droppedDs == 1);
        CHECK(s.extension->acceptedDs == 1);
    });
}

TEST_CASE("lockstep: a repeated command id is re-acknowledged, never re-applied, a stale Reset is rejected") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind);
        REQUIRE(s.emulator->reset(5).ok());
        proto::SimControlAck first;
        REQUIRE(s.emulator->awaitAck(5, 2000ms, &first));
        CHECK(first.runEpoch == 2);

        // Retransmission of the same command (same id, same epoch).
        REQUIRE(s.emulator->sendControl(proto::SimCommand::Reset, 5, 2).ok());
        proto::Message m;
        auto got = receiveSkippingUnknown(s.emulator->channel(), m, 2000ms);
        REQUIRE(got.ok());
        REQUIRE(got.value());
        const auto& again = std::get<proto::SimControlAck>(m);
        CHECK(again.commandId == 5);
        CHECK(again.result == proto::ControlResult::Applied);
        CHECK(again.runEpoch == 2);

        // A Reset that does not carry a newer epoch is refused.
        REQUIRE(s.emulator->sendControl(proto::SimCommand::Reset, 6, 2).ok());
        got = receiveSkippingUnknown(s.emulator->channel(), m, 2000ms);
        REQUIRE(got.value());
        CHECK(std::get<proto::SimControlAck>(m).result == proto::ControlResult::StaleEpoch);

        // A non-Reset command under an old epoch is rejected, Stop is honoured whatever the epoch.
        REQUIRE(s.emulator->sendControl(proto::SimCommand::Pause, 7, 1).ok());
        got = receiveSkippingUnknown(s.emulator->channel(), m, 2000ms);
        REQUIRE(got.value());
        CHECK(std::get<proto::SimControlAck>(m).result == proto::ControlResult::StaleEpoch);
        REQUIRE(s.emulator->sendControl(proto::SimCommand::Stop, 8, 1).ok());
        got = receiveSkippingUnknown(s.emulator->channel(), m, 2000ms);
        REQUIRE(got.value());
        CHECK(std::get<proto::SimControlAck>(m).result == proto::ControlResult::Applied);
    });
}

TEST_CASE("lockstep: frames of a foreign session are dropped on both sides") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind);
        proto::SensorFrame foreign = s.emulator->makeFrame(1);
        foreign.sessionId = 999;
        REQUIRE(s.emulator->channel().send(foreign).ok());
        const auto end = std::chrono::steady_clock::now() + 2000ms;
        while (s.extension->droppedSensor == 0 && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(1ms);
        CHECK(s.extension->droppedSensor == 1);
        CHECK(s.extension->stepped == 0);
        REQUIRE(s.emulator->tick(1, 2000ms) == TickOutcome::Completed);  // tick 1 was never consumed
    });
}

TEST_CASE("lockstep: a duplicate or out-of-order SensorFrame is answered with ProtocolError and not stepped") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind);
        for (std::uint64_t t = 1; t <= 3; ++t) REQUIRE(s.emulator->tick(t, 2000ms) == TickOutcome::Completed);
        REQUIRE(s.emulator->channel().send(s.emulator->makeFrame(3)).ok());  // duplicate, bypassing the window
        proto::Message m;
        auto got = receiveSkippingUnknown(s.emulator->channel(), m, 2000ms);
        REQUIRE(got.value());
        const auto& af = std::get<proto::ActuatorFrame>(m);
        CHECK(af.result == proto::TickResult::ProtocolError);
        CHECK(af.ackedTick() == 3);
        CHECK(s.extension->stepped == 3);
    });
}

TEST_CASE("lockstep: the robot reports RobotStalled and the window suspends lock-step until reset") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind, [](const proto::SensorFrame& sf, FakeExtension&) { return sf.tick == 4 && sf.runEpoch == 1 ? Act::AnswerStalled : Act::Answer; });
        for (std::uint64_t t = 1; t <= 3; ++t) REQUIRE(s.emulator->tick(t, 2000ms) == TickOutcome::Completed);
        CHECK(s.emulator->tick(4, 2000ms) == TickOutcome::Stalled);
        CHECK(s.emulator->window().stalled());
        CHECK_FALSE(s.emulator->window().canSend());
        CHECK(s.emulator->tick(5, 2000ms) == TickOutcome::SendFailed);

        REQUIRE(s.emulator->reset(1).ok());
        REQUIRE(s.emulator->awaitAck(1, 2000ms));
        CHECK(s.emulator->window().canSend());
        CHECK(s.emulator->tick(1, 2000ms) == TickOutcome::Completed);
    });
}

TEST_CASE("lockstep: robot stall - the watchdog expires, the emulator never blocks past it, a late answer is ignored after Reset") {
    forEachTransportKind([](TransportKind kind) {
        // Tick 3 of epoch 1 hangs for 400 ms (robot code stuck) and is then answered anyway.
        Session s(kind, [](const proto::SensorFrame& sf, FakeExtension&) {
            if (sf.tick == 3 && sf.runEpoch == 1) std::this_thread::sleep_for(400ms);
            return Act::Answer;
        });
        for (std::uint64_t t = 1; t <= 2; ++t) REQUIRE(s.emulator->tick(t, 2000ms) == TickOutcome::Completed);

        const auto t0 = std::chrono::steady_clock::now();
        CHECK(s.emulator->tick(3, 120ms) == TickOutcome::Timeout);  // watchdog
        const auto waited = std::chrono::steady_clock::now() - t0;
        CHECK(waited >= 100ms);
        CHECK(waited < 380ms);  // returned at the watchdog, not when the robot finally answered
        CHECK(s.emulator->window().stalled());
        CHECK_FALSE(s.emulator->window().canSend());

        // The operator resets; the late epoch-1 answer arrives afterwards and must not count.
        REQUIRE(s.emulator->reset(2).ok());
        REQUIRE(s.emulator->awaitAck(2, 3000ms));
        CHECK(s.emulator->window().droppedFrames() == 1);
        CHECK(s.emulator->window().canSend());
        for (std::uint64_t t = 1; t <= 3; ++t) CHECK(s.emulator->tick(t, 2000ms) == TickOutcome::Completed);
    });
}

TEST_CASE("lockstep: the peer disappears in the middle of a tick, the emulator notices and a reconnect starts a fresh epoch") {
    forEachTransportKind([](TransportKind kind) {
        Link link(kind);
        auto first = link.connect();
        EmulatorSide emulator(std::move(first.emulator));
        auto crashing = std::make_unique<FakeExtension>(std::move(first.extension), [](const proto::SensorFrame& sf, FakeExtension&) {
            return sf.tick == 3 ? Act::Disconnect : Act::Answer;  // robot process dies while stepping tick 3
        });
        REQUIRE(emulator.handshake(1, 1).ok());
        REQUIRE(crashing->waitHandshaken(5000ms));
        for (std::uint64_t t = 1; t <= 2; ++t) REQUIRE(emulator.tick(t, 2000ms) == TickOutcome::Completed);

        const auto t0 = std::chrono::steady_clock::now();
        CHECK(emulator.tick(3, 5000ms) == TickOutcome::Disconnected);
        CHECK(std::chrono::steady_clock::now() - t0 < 2500ms);  // detected by the transport, long before the watchdog
        CHECK(emulator.window().stalled());
        CHECK_FALSE(emulator.channel().connected());
        crashing.reset();

        // The robot is restarted: it connects again; the emulator bumps the session and the epoch.
        auto second = link.connect();
        EmulatorSide emulator2(std::move(second.emulator));
        FakeExtension fresh(std::move(second.extension));
        REQUIRE(emulator2.handshake(2, emulator.epoch() + 1).ok());
        REQUIRE(fresh.waitHandshaken(5000ms));
        CHECK(emulator2.window().sessionId() == 2);
        CHECK(emulator2.window().runEpoch() == 2);
        CHECK(fresh.epoch == 2);
        for (std::uint64_t t = 1; t <= 10; ++t) CHECK(emulator2.tick(t, 2000ms) == TickOutcome::Completed);
    });
}

TEST_CASE("lockstep: the emulator disappears mid-tick - the extension side sees Disconnected promptly (it must then unblock the robot itself)") {
    forEachTransportKind([](TransportKind kind) {
        Link link(kind);
        auto pair = link.connect();
        EmulatorSide emulator(std::move(pair.emulator));
        proto::MessageChannel ext(std::move(pair.extension), codec());
        std::atomic<bool> handshakeOk{false};
        std::thread hello([&] { handshakeOk = emulator.handshake(1, 1).ok(); });
        REQUIRE(performClientHandshake(ext, proto::Hello{}, 5000ms).ok());
        hello.join();
        REQUIRE(handshakeOk);

        REQUIRE(ext.send(proto::Goodbye{}).ok());  // keep the pipe busy in both directions
        std::atomic<int> outcome{0};
        std::atomic<std::int64_t> detectedAfterMs{-1};
        const auto t0 = std::chrono::steady_clock::now();
        std::thread watchdogThread([&] {
            proto::Message m;
            auto r = receiveSkippingUnknown(ext, m, 10000ms);  // the extension's IPC thread, waiting for the next SensorFrame
            detectedAfterMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
            outcome = (!r.ok() && r.error().code == ErrorCode::Disconnected) ? 1 : 2;
        });
        std::this_thread::sleep_for(50ms);
        emulator.channel().close();  // the emulator crashed / was killed
        watchdogThread.join();
        CHECK(outcome == 1);
        CHECK(detectedAfterMs < 3000);
        CHECK_FALSE(ext.connected());
    });
}

TEST_CASE("lockstep: Stop is honoured and acknowledged, then the extension leaves lock-step") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind);
        REQUIRE(s.emulator->tick(1, 2000ms) == TickOutcome::Completed);
        REQUIRE(s.emulator->sendControl(proto::SimCommand::Stop, 3, s.emulator->epoch()).ok());
        proto::SimControlAck ack;
        REQUIRE(s.emulator->awaitAck(3, 2000ms, &ack));
        CHECK(ack.result == proto::ControlResult::Applied);
        CHECK(ack.command == proto::SimCommand::Stop);
        CHECK(ack.tick == 1);
        proto::Message m;
        auto r = receiveSkippingUnknown(s.emulator->channel(), m, 2000ms);
        REQUIRE_FALSE(r.ok());  // the extension closed its end
        CHECK(r.error().code == ErrorCode::Disconnected);
    });
}

TEST_CASE("lockstep: discovery traffic (DeviceAnnounce, LoopbackSet) interleaves with ticks without disturbing the window") {
    forEachTransportKind([](TransportKind kind) {
        Session s(kind);
        for (std::uint64_t t = 1; t <= 20; ++t) {
            REQUIRE(s.emulator->channel().send(proto::LoopbackSet{{DeviceId{5}}}).ok());
            REQUIRE(s.emulator->tick(t, 2000ms) == TickOutcome::Completed);
        }
        CHECK(s.emulator->window().droppedFrames() == 0);
    });
}
