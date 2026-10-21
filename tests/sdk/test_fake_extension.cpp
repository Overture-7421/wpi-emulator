#include <doctest/doctest.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/sdk/testing/fake_extension.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu::sdk;
using namespace wpiemu::sdk::testing;
using namespace std::chrono_literals;

namespace {

// The fake extension on one end of an in-memory pipe, the test as the emulator on the other; nothing runs unless pumped.
struct Wire {
    explicit Wire(InMemoryTransportPair pair = makeInMemoryTransportPair())
        : emulator(std::move(pair.a), codec), ext(std::move(pair.b), codec) {}

    std::shared_ptr<MemoryMessageCodec> codec = std::make_shared<MemoryMessageCodec>();
    proto::MessageChannel emulator;
    FakeExtension ext;
};

// The next message the emulator end has waiting, when it is a T.
template <class T>
std::optional<T> take(proto::MessageChannel& emulator) {
    proto::Message m;
    const Result<bool> got = emulator.receive(m, 0ms);
    if (!got || !got.value()) return std::nullopt;
    if (const T* v = std::get_if<T>(&m)) return *v;
    return std::nullopt;
}

proto::SensorFrame sensor(std::uint64_t tick, std::uint64_t session = 7, std::uint64_t epoch = 1) {
    proto::SensorFrame f;
    f.tick = tick;
    f.sessionId = session;
    f.runEpoch = epoch;
    return f;
}

proto::SimControl control(proto::SimCommand command, std::uint64_t commandId, std::uint64_t epoch = 1) {
    proto::SimControl c;
    c.command = command;
    c.commandId = commandId;
    c.sessionId = 7;
    c.runEpoch = epoch;
    return c;
}

void handshake(Wire& w, std::uint64_t session = 7, std::uint64_t epoch = 1) {
    w.ext.hello();
    REQUIRE(take<proto::Hello>(w.emulator).has_value());
    proto::HelloAck ack;
    ack.sessionId = session;
    ack.runEpoch = epoch;
    REQUIRE(w.emulator.send(ack).ok());
    w.ext.pump();
    REQUIRE(w.ext.handshaken());
}

}  // namespace

TEST_CASE("fake extension: it stays silent until told to say hello, and the Hello carries the chosen version") {
    Wire w;
    w.ext.pump();
    CHECK_FALSE(take<proto::Hello>(w.emulator).has_value());

    w.ext.hello(750, 0, false);
    const auto hello = take<proto::Hello>(w.emulator);
    REQUIRE(hello.has_value());
    CHECK(hello->protocolVersion == 0);
    CHECK(hello->proposedWatchdogMs == 750);
    CHECK_FALSE(hello->supportsArmed);

    w.ext.programStarted();
    const auto log = take<proto::Log>(w.emulator);
    REQUIRE(log.has_value());
    CHECK(proto::isLifecycleLog(*log, proto::kLifecycleProgramStarted));
}

TEST_CASE("fake extension: a refused HelloAck gives it no identity, an accepted one is adopted by every answer") {
    {
        Wire w;
        w.ext.hello();
        proto::HelloAck refusal;
        refusal.accepted = false;
        refusal.reason = "too old";
        REQUIRE(w.emulator.send(refusal).ok());
        w.ext.pump();
        REQUIRE(w.ext.helloAck.has_value());
        CHECK_FALSE(w.ext.handshaken());
    }
    Wire w;
    handshake(w, 9, 3);
    w.ext.outputs = {{DeviceId{5}, 1, 2.5}};
    REQUIRE(w.emulator.send(sensor(1, 9, 3)).ok());
    w.ext.pump();
    const auto answer = take<proto::ActuatorFrame>(w.emulator);
    REQUIRE(answer.has_value());
    CHECK(answer->sessionId == 9);
    CHECK(answer->runEpoch == 3);
    CHECK(answer->ackedTick() == 1);
    CHECK(answer->result == proto::TickResult::Completed);
    CHECK(answer->samples == w.ext.outputs);
    CHECK(w.ext.stepped == 1);
}

TEST_CASE("fake extension: stale and foreign frames are dropped silently, an out-of-order one is refused") {
    Wire w;
    handshake(w, 7, 2);
    REQUIRE(w.emulator.send(sensor(1, 7, 2)).ok());
    REQUIRE(w.emulator.send(sensor(1, 7, 2)).ok());  // duplicate tick
    REQUIRE(w.emulator.send(sensor(2, 7, 1)).ok());  // epoch before the one in force
    REQUIRE(w.emulator.send(sensor(2, 99, 2)).ok());  // another session
    w.ext.pump();

    const auto first = take<proto::ActuatorFrame>(w.emulator);
    REQUIRE(first.has_value());
    CHECK(first->result == proto::TickResult::Completed);
    const auto refused = take<proto::ActuatorFrame>(w.emulator);
    REQUIRE(refused.has_value());
    CHECK(refused->result == proto::TickResult::ProtocolError);
    CHECK(refused->ackedTick() == 1);
    CHECK_FALSE(take<proto::ActuatorFrame>(w.emulator).has_value());
    CHECK(w.ext.droppedFrames == 2);
    CHECK(w.ext.stepped == 1);
    CHECK(w.ext.sensorFrames.size() == 4);
}

TEST_CASE("fake extension: Hold keeps the frame until it is answered, Stall and Disconnect misbehave on demand") {
    Wire w;
    handshake(w);
    w.ext.reply = FakeExtension::Reply::Hold;
    REQUIRE(w.emulator.send(sensor(1)).ok());
    w.ext.pump();
    CHECK_FALSE(take<proto::ActuatorFrame>(w.emulator).has_value());
    REQUIRE(w.ext.pending.has_value());
    w.ext.answer(*w.ext.pending);
    const auto late = take<proto::ActuatorFrame>(w.emulator);
    REQUIRE(late.has_value());
    CHECK(late->ackedTick() == 1);

    w.ext.reply = FakeExtension::Reply::Stall;
    REQUIRE(w.emulator.send(sensor(2)).ok());
    w.ext.pump();
    const auto stalled = take<proto::ActuatorFrame>(w.emulator);
    REQUIRE(stalled.has_value());
    CHECK(stalled->result == proto::TickResult::RobotStalled);

    w.ext.reply = FakeExtension::Reply::Disconnect;
    REQUIRE(w.emulator.send(sensor(3)).ok());
    w.ext.pump();
    CHECK(w.ext.disconnected);
    proto::Message m;
    const Result<bool> gone = w.emulator.receive(m, 0ms);
    REQUIRE_FALSE(gone.ok());
    CHECK(gone.error().code == ErrorCode::Disconnected);
}

TEST_CASE("fake extension: onFrame decides per frame and only sees frames the follower accepts") {
    Wire w;
    handshake(w);
    std::vector<std::uint64_t> seen;
    w.ext.onFrame = [&](const proto::SensorFrame& sf, FakeExtension&) {
        seen.push_back(sf.tick);
        return sf.tick == 2 ? FakeExtension::Reply::Hold : FakeExtension::Reply::Answer;
    };
    REQUIRE(w.emulator.send(sensor(1)).ok());
    REQUIRE(w.emulator.send(sensor(5, 99)).ok());  // foreign session
    REQUIRE(w.emulator.send(sensor(2)).ok());
    w.ext.pump();
    CHECK(seen == std::vector<std::uint64_t>{1, 2});
    CHECK(take<proto::ActuatorFrame>(w.emulator).has_value());
    CHECK_FALSE(take<proto::ActuatorFrame>(w.emulator).has_value());
    REQUIRE(w.ext.pending.has_value());
    CHECK(w.ext.pending->tick == 2);
}

TEST_CASE("fake extension: controls are acknowledged unless withheld, refused on request, and Stop can end the connection") {
    Wire w;
    handshake(w);
    REQUIRE(w.emulator.send(control(proto::SimCommand::Pause, 1)).ok());
    w.ext.pump();
    const auto applied = take<proto::SimControlAck>(w.emulator);
    REQUIRE(applied.has_value());
    CHECK(applied->commandId == 1);
    CHECK(applied->result == proto::ControlResult::Applied);
    CHECK(w.ext.remotePaused);

    w.ext.ackControls = false;
    REQUIRE(w.emulator.send(control(proto::SimCommand::Resume, 2)).ok());
    w.ext.pump();
    CHECK_FALSE(take<proto::SimControlAck>(w.emulator).has_value());
    CHECK_FALSE(w.ext.remotePaused);

    w.ext.ackControls = true;
    w.ext.controlResult = proto::ControlResult::Rejected;
    REQUIRE(w.emulator.send(control(proto::SimCommand::Pause, 3)).ok());
    w.ext.pump();
    const auto refused = take<proto::SimControlAck>(w.emulator);
    REQUIRE(refused.has_value());
    CHECK(refused->result == proto::ControlResult::Rejected);
    CHECK_FALSE(refused->detail.empty());
    CHECK_FALSE(w.ext.remotePaused);
    CHECK(w.ext.controls.size() == 3);

    w.ext.controlResult = proto::ControlResult::Applied;
    REQUIRE(w.emulator.send(control(proto::SimCommand::Stop, 4)).ok());
    w.ext.pump();
    CHECK_FALSE(w.ext.disconnected);  // staying connected is the default

    w.ext.leaveOnStop = true;
    REQUIRE(w.emulator.send(control(proto::SimCommand::Stop, 5)).ok());
    w.ext.pump();
    CHECK(w.ext.disconnected);
    CHECK(take<proto::SimControlAck>(w.emulator).has_value());  // both Stops were acknowledged before the close
    CHECK(take<proto::SimControlAck>(w.emulator).has_value());
    proto::Message m;
    CHECK_FALSE(w.emulator.receive(m, 0ms).ok());
}

TEST_CASE("fake extension: Goodbye ends the connection only when it leaves on Stop") {
    Wire w;
    handshake(w);
    REQUIRE(w.emulator.send(proto::Goodbye{"bye"}).ok());
    w.ext.pump();
    CHECK_FALSE(w.ext.disconnected);
    w.ext.leaveOnStop = true;
    REQUIRE(w.emulator.send(proto::Goodbye{"bye"}).ok());
    w.ext.pump();
    CHECK(w.ext.disconnected);
}

TEST_CASE("fake extension: it records DsState, LoopbackSet and VisionSetup and counts DsState by identity") {
    Wire w;
    handshake(w, 7, 2);
    w.ext.reply = FakeExtension::Reply::Hold;
    proto::DsState current;
    current.sessionId = 7;
    current.runEpoch = 2;
    proto::DsState stale = current;
    stale.runEpoch = 1;
    REQUIRE(w.emulator.send(VisionSetup{}).ok());
    REQUIRE(w.emulator.send(current).ok());
    REQUIRE(w.emulator.send(stale).ok());
    REQUIRE(w.emulator.send(proto::LoopbackSet{{DeviceId{5}}}).ok());
    REQUIRE(w.emulator.send(sensor(1, 7, 2)).ok());
    w.ext.pump();
    CHECK(w.ext.dsStates.size() == 2);
    CHECK(w.ext.acceptedDs == 1);
    CHECK(w.ext.droppedDs == 1);
    REQUIRE(w.ext.loopbacks.size() == 1);
    CHECK(w.ext.loopbacks[0].loopback == std::vector<DeviceId>{DeviceId{5}});
    CHECK(w.ext.visionSetups.size() == 1);
    CHECK(w.ext.trace == std::vector<std::string>{"ack", "vision", "frame"});
}

TEST_CASE("fake extension: the watchdog model follows its clock, pause and the frame in flight") {
    Wire w;
    handshake(w);
    auto now = std::chrono::steady_clock::time_point{} + 10s;
    w.ext.clock = [&] { return now; };
    REQUIRE(w.emulator.send(proto::DsState{}).ok());
    w.ext.pump();
    CHECK(w.ext.lastRxAt == now);

    now += 600ms;
    CHECK(w.ext.watchdogWouldFire(500ms, now));
    CHECK_FALSE(w.ext.watchdogWouldFire(700ms, now));

    REQUIRE(w.emulator.send(control(proto::SimCommand::Pause, 1)).ok());
    w.ext.pump();
    now += 600ms;
    CHECK_FALSE(w.ext.watchdogWouldFire(500ms, now));  // paused

    REQUIRE(w.emulator.send(control(proto::SimCommand::Resume, 2)).ok());
    w.ext.reply = FakeExtension::Reply::Hold;
    REQUIRE(w.emulator.send(sensor(1)).ok());
    w.ext.pump();
    now += 600ms;
    CHECK_FALSE(w.ext.watchdogWouldFire(500ms, now));  // a tick is in flight
    w.ext.pending.reset();
    CHECK(w.ext.watchdogWouldFire(500ms, now));
}

namespace {

// Fails the next decode with a chosen error, after consuming the frame like a real codec would.
class FailingCodec final : public proto::IMessageCodec {
public:
    explicit FailingCodec(std::shared_ptr<MemoryMessageCodec> inner) : inner_(std::move(inner)) {}
    std::string_view name() const override { return "failing"; }
    Status encode(const proto::Message& m, std::vector<std::byte>& out) const override { return inner_->encode(m, out); }
    Result<proto::Message> decode(std::span<const std::byte> frame) const override {
        Result<proto::Message> decoded = inner_->decode(frame);
        if (!failNext || !decoded) return decoded;
        const ErrorCode code = *failNext;
        failNext.reset();
        return makeError(code, "scripted decode failure");
    }
    mutable std::optional<ErrorCode> failNext;

private:
    std::shared_ptr<MemoryMessageCodec> inner_;
};

}  // namespace

TEST_CASE("fake extension: a frame of an unknown message type is skipped, any other decode error ends the connection") {
    auto pair = makeInMemoryTransportPair();
    auto inner = std::make_shared<MemoryMessageCodec>();
    auto failing = std::make_shared<FailingCodec>(inner);
    proto::MessageChannel emulator(std::move(pair.a), inner);
    FakeExtension ext(std::move(pair.b), failing);

    failing->failNext = ErrorCode::Unsupported;
    REQUIRE(emulator.send(proto::DsState{}).ok());
    REQUIRE(emulator.send(proto::LoopbackSet{}).ok());
    ext.pump();
    CHECK_FALSE(ext.disconnected);
    CHECK(ext.dsStates.empty());  // the skipped frame
    CHECK(ext.loopbacks.size() == 1);

    failing->failNext = ErrorCode::ProtocolError;
    REQUIRE(emulator.send(proto::LoopbackSet{}).ok());
    ext.pump();
    CHECK(ext.disconnected);
}
