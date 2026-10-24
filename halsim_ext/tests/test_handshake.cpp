#include <doctest/doctest.h>

#include "fakes.h"
#include "wpiemu/sdk/testing/fake_emulator.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

TEST_CASE("handshake: Hello carries the robot identity and the proposed watchdog") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.proposedWatchdogMs = 750;
    cfg.programName = "frcUserProgram";
    Rig rig(defaultAck(), cfg);
    REQUIRE(rig.start());

    const auto hello = rig.emulator->take<proto::Hello>(1000ms);
    REQUIRE(hello.has_value());
    CHECK(hello->protocolVersion == proto::kProtocolVersion);
    CHECK(hello->pid == 4242);
    CHECK(hello->language == proto::RobotLanguage::Cpp);
    CHECK(hello->wpilibYear == 2026);
    CHECK(hello->wpilibVersion == "2026.2.1");
    CHECK(hello->programName == "frcUserProgram");
    CHECK(hello->proposedWatchdogMs == 750);
    CHECK(hello->supportsArmed);
    REQUIRE(hello->halPortCounts.size() == 1);
    CHECK(hello->halPortCounts[0].first == "pwm");
    CHECK(rig.client.lastEndpoint == "memory:test");
    CHECK(rig.runtime->stats().sessions == 1);
}

TEST_CASE("handshake: lock-step pauses HAL time before the bridges start") {
    Rig rig;
    FakeBridge& bridge = rig.addBridge();
    REQUIRE(rig.start());
    CHECK(rig.clock.paused);
    CHECK(bridge.started);
    CHECK(rig.clock.pauseCalls == 1);
}

TEST_CASE("handshake: realtime, discovery-only and the realtime override never pause HAL time") {
    SUBCASE("HelloAck says Realtime") {
        proto::HelloAck ack = defaultAck();
        ack.clockMode = proto::ClockMode::Realtime;
        Rig rig(ack);
        REQUIRE(rig.start());
        CHECK(rig.clock.pauseCalls == 0);
    }
    SUBCASE("discovery-only run") {
        proto::HelloAck ack = defaultAck();
        ack.discoveryOnly = true;
        Rig rig(ack);
        REQUIRE(rig.start());
        CHECK(rig.clock.pauseCalls == 0);
    }
    SUBCASE("WPIEMU_CLOCK=realtime") {
        ExtensionConfig cfg = Rig::defaultConfig();
        cfg.clock = ClockPreference::Realtime;
        Rig rig(defaultAck(), cfg);
        REQUIRE(rig.start());
        CHECK(rig.clock.pauseCalls == 0);
    }
}

TEST_CASE("handshake: a refused connection leaves the robot untouched") {
    proto::HelloAck ack = defaultAck();
    ack.accepted = false;
    ack.reason = "version";
    Rig rig(ack);
    FakeBridge& bridge = rig.addBridge();
    const sdk::Status s = rig.start();
    REQUIRE_FALSE(s);
    CHECK(s.error().message.find("version") != std::string::npos);
    CHECK(rig.clock.pauseCalls == 0);
    CHECK_FALSE(bridge.started);
}

TEST_CASE("handshake: no HelloAck within the timeout fails cleanly") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.handshakeTimeout = 150ms;
    Rig rig(defaultAck(), cfg);
    auto pair = sdk::testing::makeInMemoryTransportPair();
    sdk::testing::QueuedTransportClient silent;
    silent.push(std::move(pair.b));
    sdk::testing::FakeEmulator mute(std::move(pair.a), rig.codec);

    RuntimeDeps deps{cfg, rig.robot, HalPorts{rig.clock, rig.ds, rig.power, rig.process}, silent, rig.codec, {}, {}, {}};
    ExtensionRuntime runtime(std::move(deps));
    const sdk::Status s = runtime.start();
    REQUIRE_FALSE(s);
    CHECK(s.error().code == sdk::ErrorCode::Timeout);
    CHECK(rig.clock.pauseCalls == 0);
}

TEST_CASE("handshake: nobody listening fails after the connect timeout") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.connectTimeout = 50ms;
    sdk::testing::QueuedTransportClient nobody;
    Rig rig(defaultAck(), cfg);
    RuntimeDeps deps{cfg, rig.robot, HalPorts{rig.clock, rig.ds, rig.power, rig.process}, nobody, rig.codec, {}, {}, {}};
    ExtensionRuntime runtime(std::move(deps));
    const sdk::Status s = runtime.start();
    REQUIRE_FALSE(s);
    CHECK(s.error().code == sdk::ErrorCode::Timeout);
    CHECK(rig.clock.pauseCalls == 0);
}

TEST_CASE("handshake: an emulator Goodbye instead of HelloAck aborts") {
    Rig rig;
    auto pair = sdk::testing::makeInMemoryTransportPair();
    sdk::testing::QueuedTransportClient client;
    client.push(std::move(pair.b));
    sdk::testing::FakeEmulator emu(std::move(pair.a), rig.codec);
    std::thread answer([&] {
        const auto hello = emu.take<proto::Hello>(1000ms);
        if (hello) emu.send(proto::Message(proto::Goodbye{"emulator is shutting down"}));
    });
    ExtensionConfig cfg = Rig::defaultConfig();
    RuntimeDeps deps{cfg, rig.robot, HalPorts{rig.clock, rig.ds, rig.power, rig.process}, client, rig.codec, {}, {}, {}};
    ExtensionRuntime runtime(std::move(deps));
    const sdk::Status s = runtime.start();
    answer.join();
    REQUIRE_FALSE(s);
    CHECK(s.error().code == sdk::ErrorCode::Cancelled);
}

TEST_CASE("handshake: starting twice is rejected") {
    Rig rig;
    REQUIRE(rig.start());
    const sdk::Status again = rig.runtime->start();
    REQUIRE_FALSE(again);
    CHECK(again.error().code == sdk::ErrorCode::InvalidState);
}
