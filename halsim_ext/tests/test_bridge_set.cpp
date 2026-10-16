#include <doctest/doctest.h>

#include "fakes.h"
#include "wpiemu/halsim_ext/bridge_set.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

namespace {
struct Fixture {
    DeviceTable table;
    sdk::NullLogger logger;
    std::atomic<bool> hooks{false};
};

BridgeSet makeSet(Fixture& fx, std::vector<FakeBridge*>& raw, int count) {
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    for (int i = 0; i < count; ++i) {
        auto b = std::make_unique<FakeBridge>("fake." + std::to_string(i));
        raw.push_back(b.get());
        bridges.push_back(std::move(b));
    }
    return BridgeSet(std::move(bridges), fx.table, fx.logger);
}
}  // namespace

TEST_CASE("bridge set routes samples to the bridge that announced the device") {
    Fixture fx;
    std::vector<FakeBridge*> raw;
    BridgeSet set = makeSet(fx, raw, 2);
    set.start(fx.hooks);
    raw[0]->queueDevice(FakeBridge::makeDescriptor("pwm", 1));
    raw[1]->queueDevice(FakeBridge::makeDescriptor("dio", 2));
    set.poll();

    const sdk::DeviceId a = raw[0]->idOf(FakeBridge::makeDescriptor("pwm", 1));
    const sdk::DeviceId b = raw[1]->idOf(FakeBridge::makeDescriptor("dio", 2));
    REQUIRE(a.valid());
    REQUIRE(b.valid());
    CHECK(a != b);

    const std::vector<sdk::SignalSample> samples{{a, 1, 0.5}, {b, 1, 1.0}, {sdk::DeviceId{77}, 0, 9.0}};
    set.applyInputs(samples);
    REQUIRE(raw[0]->inputs().size() == 1);
    CHECK(raw[0]->inputs()[0].value == 0.5);
    REQUIRE(raw[1]->inputs().size() == 1);
    CHECK(raw[1]->inputs()[0].value == 1.0);

    set.applyLoopback(std::vector<sdk::DeviceId>{b});
    CHECK(raw[0]->loopbacks().empty());
    CHECK(raw[1]->loopbacks() == std::vector<sdk::DeviceId>{b});
}

TEST_CASE("bridge set skips retired devices and collects every bridge's outputs") {
    Fixture fx;
    std::vector<FakeBridge*> raw;
    BridgeSet set = makeSet(fx, raw, 2);
    set.start(fx.hooks);
    const auto d0 = FakeBridge::makeDescriptor("pwm", 0);
    const auto d1 = FakeBridge::makeDescriptor("pwm", 1);
    raw[0]->queueDevice(d0);
    raw[1]->queueDevice(d1);
    set.poll();
    raw[0]->setOutput(raw[0]->idOf(d0), 0, 0.25);
    raw[1]->setOutput(raw[1]->idOf(d1), 0, -0.75);

    std::vector<sdk::SignalSample> out;
    set.collectOutputs(out);
    CHECK(out.size() == 2);

    raw[1]->queueRetire(d1);
    set.poll();
    set.applyInputs(std::vector<sdk::SignalSample>{{raw[1]->idOf(d1), 1, 3.0}});
    CHECK(raw[1]->inputs().empty());
}

TEST_CASE("a bridge that fails to start is dropped and never polled, the others continue") {
    Fixture fx;
    std::vector<FakeBridge*> raw;
    BridgeSet set = makeSet(fx, raw, 2);
    raw[0]->failStart = true;
    set.start(fx.hooks);
    CHECK(set.activeCount() == 1);
    CHECK(raw[0]->stops == 1);
    set.poll();
    CHECK(raw[0]->polls == 0);
    CHECK(raw[1]->polls == 1);
}

TEST_CASE("bridge set stop is safe to repeat") {
    Fixture fx;
    std::vector<FakeBridge*> raw;
    {
        BridgeSet set = makeSet(fx, raw, 3);
        set.start(fx.hooks);
        set.stop();
        set.stop();
        CHECK(raw[0]->stops == 2);  // each stop() reaches every bridge; bridges are idempotent by contract
        CHECK(raw[2]->stops == 2);
    }
    CHECK(raw.size() == 3);
}

TEST_CASE("bridge host exposes the session's optional-hooks switch and forwards observeBus") {
    Fixture fx;
    std::vector<FakeBridge*> raw;
    BridgeSet set = makeSet(fx, raw, 1);
    set.start(fx.hooks);
    CHECK_FALSE(raw[0]->host->enableOptionalHooks());
    fx.hooks = true;
    CHECK(raw[0]->host->enableOptionalHooks());

    std::vector<sdk::proto::DeviceAnnounce> sent;
    fx.table.setSink([&](sdk::proto::DeviceAnnounce&& a) { sent.push_back(std::move(a)); });
    const sdk::DeviceId id = raw[0]->host->announce(FakeBridge::makeDescriptor("talonfx", 3), FakeBridge::makeSpecs());
    raw[0]->host->observeBus(id, "canivore");
    REQUIRE(sent.size() == 2);
    CHECK(sent.back().observedBus == "canivore");
}
