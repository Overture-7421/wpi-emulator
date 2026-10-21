#include <doctest/doctest.h>

#include "fakes.h"
#include "wpiemu/sdk/testing/fake_emulator.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

TEST_CASE("reconnect: after the emulator restarts the extension re-handshakes, re-announces and re-engages lock-step") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.reconnect = true;
    Rig rig(defaultAck(), cfg);
    FakeBridge& bridge = rig.addBridge();
    const auto pwm = FakeBridge::makeDescriptor("pwm", 3);
    bridge.queueDevice(pwm);
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    REQUIRE(rig.emulator->tick());
    CHECK(rig.clock.pauseCalls == 1);
    CHECK(bridge.historyResets == 1);

    // The "new emulator": next pipe, a new session id and a later epoch.
    proto::HelloAck second = defaultAck();
    second.sessionId = 8;
    second.runEpoch = 5;
    std::unique_ptr<sdk::testing::FakeEmulator> first = std::move(rig.emulator);
    rig.connectNext(second);
    first->disconnect();

    // Robot is unfrozen while the emulator is away, then paused again for the new session.
    REQUIRE(waitUntil([&] { return rig.emulator->take<proto::Hello>(10ms).has_value(); }, 3000ms));
    REQUIRE(waitUntil([&] { return rig.runtime->stats().sessions == 2; }, 3000ms));
    CHECK(rig.clock.resumeCalls >= 1);
    CHECK(waitUntil([&] { return rig.clock.pauseCalls == 2; }, 2000ms));

    const auto announce = rig.emulator->take<proto::DeviceAnnounce>(2000ms);
    REQUIRE(announce.has_value());
    CHECK(announce->descriptor.type == "pwm");
    CHECK(announce->id == bridge.idOf(pwm));  // device ids are stable across the reconnect

    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const auto ack = rig.emulator->tick();
    REQUIRE(ack);
    CHECK(ack->sessionId == 8);
    CHECK(ack->runEpoch == 5);
    CHECK(ack->ackedTick() == 1);  // numbering restarts with the new session
    CHECK(ack->full);
    CHECK(bridge.historyResets == 2);
}

TEST_CASE("reconnect gives up when the emulator never returns and stop() still works") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.reconnect = true;
    cfg.connectTimeout = 10ms;
    Rig rig(defaultAck(), cfg);
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    rig.emulator->disconnect();

    REQUIRE(waitUntil([&] { return rig.client.attempts > 3; }, 2000ms));  // retrying
    CHECK_FALSE(rig.runtime->finished());
    CHECK_FALSE(rig.clock.paused);
    rig.runtime->stop();
    CHECK(rig.runtime->finished());
}

TEST_CASE("Terminate policy never reconnects") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.reconnect = true;
    proto::HelloAck ack = defaultAck();
    ack.watchdogPolicy = proto::WatchdogPolicy::Terminate;
    Rig rig(ack, cfg);
    REQUIRE(rig.start());
    rig.emulator->disconnect();
    REQUIRE(rig.runtime->waitFinished(2000ms));
    CHECK(rig.process.calls == 1);
    CHECK(rig.runtime->stats().sessions == 1);
}
