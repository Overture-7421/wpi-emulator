#include <doctest/doctest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/telemetry/sim_controller.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu::sdk;
using namespace std::chrono_literals;

TEST_CASE("message type tags follow the variant order (frozen wire ids)") {
    CHECK(proto::typeOf(proto::Message{proto::Hello{}}) == proto::MessageType::Hello);
    CHECK(proto::typeOf(proto::Message{proto::HelloAck{}}) == proto::MessageType::HelloAck);
    CHECK(proto::typeOf(proto::Message{proto::DeviceAnnounce{}}) == proto::MessageType::DeviceAnnounce);
    CHECK(proto::typeOf(proto::Message{proto::LoopbackSet{}}) == proto::MessageType::LoopbackSet);
    CHECK(proto::typeOf(proto::Message{proto::SensorFrame{}}) == proto::MessageType::SensorFrame);
    CHECK(proto::typeOf(proto::Message{proto::ActuatorFrame{}}) == proto::MessageType::ActuatorFrame);
    CHECK(proto::typeOf(proto::Message{proto::DsState{}}) == proto::MessageType::DsState);
    CHECK(proto::typeOf(proto::Message{proto::SimControl{}}) == proto::MessageType::SimControl);
    CHECK(proto::typeOf(proto::Message{proto::Log{}}) == proto::MessageType::Log);
    CHECK(proto::typeOf(proto::Message{proto::Goodbye{}}) == proto::MessageType::Goodbye);
    CHECK(proto::typeOf(proto::Message{proto::SimControlAck{}}) == proto::MessageType::SimControlAck);
    CHECK(proto::typeOf(proto::Message{VisionSetup{}}) == proto::MessageType::VisionSetup);
    CHECK(proto::kMinCompatibleProtocolVersion <= proto::kProtocolVersion);
}

TEST_CASE("driver station numbers map to an alliance and back") {
    CHECK(proto::allianceOf(proto::AllianceStation::Red1) == Alliance::Red);
    CHECK(proto::allianceOf(proto::AllianceStation::Red3) == Alliance::Red);
    CHECK(proto::allianceOf(proto::AllianceStation::Blue1) == Alliance::Blue);
    CHECK(proto::allianceOf(proto::AllianceStation::Blue3) == Alliance::Blue);
    for (const Alliance a : {Alliance::Red, Alliance::Blue})
        for (int number = 1; number <= 3; ++number) {
            const proto::AllianceStation station = proto::stationOf(a, number);
            CHECK(proto::allianceOf(station) == a);
            CHECK(static_cast<int>(station) % 3 + 1 == number);
        }
    CHECK(proto::stationOf(Alliance::Red, 1) == proto::AllianceStation::Red1);
    CHECK(proto::stationOf(Alliance::Blue, 3) == proto::AllianceStation::Blue3);
    CHECK(proto::stationOf(Alliance::Blue, 0) == proto::AllianceStation::Blue1);   // out of range is clamped
    CHECK(proto::stationOf(Alliance::Red, 9) == proto::AllianceStation::Red3);
    CHECK(otherAlliance(Alliance::Red) == Alliance::Blue);
    CHECK(std::string(allianceKey(Alliance::Blue)) == "blue");
    CHECK(std::string(allianceKey(Alliance::Red)) == "red");
}

TEST_CASE("timeLeft counts down to a deadline and never goes negative") {
    const auto now = std::chrono::steady_clock::now();
    CHECK(proto::timeLeft(now - 5s).count() == 0);
    CHECK(proto::timeLeft(now).count() == 0);
    const auto left = proto::timeLeft(now + 10s);
    CHECK(left > 9s);
    CHECK(left <= 10s);
}

TEST_CASE("handshake + per-tick exchange across an in-memory channel") {
    auto pair = testing::makeInMemoryTransportPair();
    auto codec = std::make_shared<testing::MemoryMessageCodec>();
    proto::MessageChannel emulator(std::move(pair.a), codec);
    proto::MessageChannel extension(std::move(pair.b), codec);

    proto::Hello hello;
    hello.language = proto::RobotLanguage::Java;
    hello.wpilibYear = 2026;
    hello.pid = 4242;
    REQUIRE(extension.send(hello).ok());

    proto::Message m;
    auto got = emulator.receive(m, 100ms);
    REQUIRE(got.ok());
    REQUIRE(got.value());
    const auto* h = std::get_if<proto::Hello>(&m);
    REQUIRE(h != nullptr);
    CHECK(h->pid == 4242);
    CHECK(h->wpilibYear == 2026);

    proto::HelloAck ack;
    ack.stepUs = 5000;
    REQUIRE(emulator.send(ack).ok());

    proto::DeviceAnnounce ann;
    ann.id = DeviceId{1};
    ann.descriptor.family = "phoenix6";
    ann.descriptor.type = "talonfx";
    ann.descriptor.id = 4;
    ann.signalSpecs = {{"motorVoltage", SignalDirection::Output, SignalType::Double, "V"}};
    REQUIRE(extension.send(ann).ok());

    proto::ActuatorFrame af;
    af.tick = 1;
    af.samples = {{DeviceId{1}, 0, 6.0}};
    REQUIRE(extension.send(af).ok());

    // extension side sees the ack; emulator side sees announce then actuators, in order
    got = extension.receive(m, 100ms);
    REQUIRE(got.ok());
    CHECK(std::holds_alternative<proto::HelloAck>(m));
    REQUIRE(emulator.receive(m, 100ms).value());
    CHECK(std::holds_alternative<proto::DeviceAnnounce>(m));
    REQUIRE(emulator.receive(m, 100ms).value());
    const auto* frame = std::get_if<proto::ActuatorFrame>(&m);
    REQUIRE(frame != nullptr);
    CHECK(frame->samples[0].value == 6.0);

    // timeout and disconnect semantics
    auto idle = emulator.receive(m, 5ms);
    REQUIRE(idle.ok());
    CHECK_FALSE(idle.value());
    extension.close();
    CHECK(emulator.receive(m, 50ms).error().code == ErrorCode::Disconnected);
    CHECK_FALSE(emulator.connected());
}

TEST_CASE("snapshots are immutable shared views and the latest slot hands them across threads") {
    SnapshotBuilder builder;
    builder.data().tick = 7;
    builder.data().chassis.pose.position = {1.f, 2.f, 0.f};
    builder.setHud("game.fuel", 5);
    builder.setHud("game.fuel", 6);  // overwrite, not duplicate
    EntityGroupSample fuel;
    fuel.group = "fuel";
    fuel.positions = {{0.f, 0.f, 0.f}, {1.f, 1.f, 0.f}};
    builder.addEntityGroup(fuel);
    MechanismState ms;
    ms.id = "intake";
    builder.addMechanism(ms);
    auto snapshot = builder.build();

    LatestSnapshotSlot slot;
    CHECK(slot.latest() == nullptr);
    std::thread producer([&] { slot.publish(snapshot); });
    producer.join();

    const auto latest = slot.latest();
    REQUIRE(latest != nullptr);
    CHECK(latest->tick() == 7);
    CHECK(latest->chassis().pose.position.y == 2.f);
    CHECK(latest->hud().size() == 1);
    CHECK(latest->hudValue("game.fuel")->asInt() == 6);
    CHECK(latest->hudValue("nope") == nullptr);
    CHECK(latest->entityGroups()[0].positions.size() == 2);
    CHECK(latest->mechanisms()[0].id == "intake");
}

namespace {
class RecordingController final : public ISimController {
public:
    void setDriverStation(const proto::DsState& ds) override { lastDs = ds; }
    void control(const proto::SimControl& c) override { lastControl = c; }
    void bindSlot(const SlotAddress& s, const DeviceKey& d) override { bound = {s, d}; }
    void unbindSlot(const SlotAddress&) override { bound = {}; }
    void setStartPlacement(const std::optional<RobotStartPose>&) override {}
    proto::DsState lastDs;
    proto::SimControl lastControl;
    SlotMapping bound;
};
}  // namespace

TEST_CASE("UI-facing controller interface is mockable") {
    RecordingController c;
    ISimController& iface = c;
    proto::DsState ds;
    ds.enabled = true;
    iface.setDriverStation(ds);
    proto::SimControl step;
    step.command = proto::SimCommand::Step;
    step.stepCount = 3;
    iface.control(step);
    iface.bindSlot({"intake", "motor"}, {"phoenix6", "talonfx", 4, "rio", -1});
    CHECK(c.lastDs.enabled);
    CHECK(c.lastControl.stepCount == 3);
    CHECK(c.bound.slot.owner == "intake");
}
