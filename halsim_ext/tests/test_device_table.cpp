#include <doctest/doctest.h>

#include "fakes.h"
#include "wpiemu/halsim_ext/device_table.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using wpiemu::halsim::testing::FakeBridge;

TEST_CASE("device table assigns session-unique ids and announces once per key") {
    DeviceTable table;
    std::vector<sdk::proto::DeviceAnnounce> sent;
    table.setSink([&](sdk::proto::DeviceAnnounce&& a) { sent.push_back(std::move(a)); });

    const auto pwm3 = FakeBridge::makeDescriptor("pwm", 3);
    const auto pwm4 = FakeBridge::makeDescriptor("pwm", 4);
    const sdk::DeviceId a = table.announce(0, pwm3, FakeBridge::makeSpecs());
    const sdk::DeviceId b = table.announce(1, pwm4, FakeBridge::makeSpecs());
    CHECK(a.valid());
    CHECK(a != b);
    CHECK(sent.size() == 2);

    // identical re-announce: same id, no message
    CHECK(table.announce(0, pwm3, FakeBridge::makeSpecs()) == a);
    CHECK(sent.size() == 2);

    // changed signals: same id, re-sent
    std::vector<sdk::SignalSpec> more = FakeBridge::makeSpecs();
    more.push_back({"extra", sdk::SignalDirection::Output, sdk::SignalType::Bool, ""});
    CHECK(table.announce(0, pwm3, more) == a);
    CHECK(sent.size() == 3);
    CHECK(sent.back().signalSpecs.size() == 3);

    CHECK(table.ownerOf(a) == std::optional<std::size_t>(0));
    CHECK(table.ownerOf(b) == std::optional<std::size_t>(1));
    CHECK_FALSE(table.ownerOf(sdk::DeviceId{}).has_value());
    CHECK_FALSE(table.ownerOf(sdk::DeviceId{99}).has_value());
    CHECK(table.liveCount() == 2);
}

TEST_CASE("retire marks the device, drops ownership and un-retires on re-announce") {
    DeviceTable table;
    std::vector<sdk::proto::DeviceAnnounce> sent;
    table.setSink([&](sdk::proto::DeviceAnnounce&& a) { sent.push_back(std::move(a)); });

    const auto d = FakeBridge::makeDescriptor("dio", 1);
    const sdk::DeviceId id = table.announce(0, d, FakeBridge::makeSpecs());
    table.retire(id);
    CHECK(sent.back().retired);
    CHECK(sent.back().id == id);
    CHECK_FALSE(table.ownerOf(id).has_value());
    CHECK(table.liveCount() == 0);
    table.retire(id);  // idempotent
    CHECK(sent.size() == 2);

    CHECK(table.announce(0, d, FakeBridge::makeSpecs()) == id);
    CHECK_FALSE(sent.back().retired);
    CHECK(table.ownerOf(id).has_value());
}

TEST_CASE("observeBus re-announces with the observed label once") {
    DeviceTable table;
    std::vector<sdk::proto::DeviceAnnounce> sent;
    table.setSink([&](sdk::proto::DeviceAnnounce&& a) { sent.push_back(std::move(a)); });
    sdk::DeviceDescriptor d = FakeBridge::makeDescriptor("talonfx", 4);
    d.bus = sdk::kBusUnknown;
    const sdk::DeviceId id = table.announce(0, d, FakeBridge::makeSpecs());

    table.observeBus(id, "canivore");
    REQUIRE(sent.size() == 2);
    CHECK(sent.back().observedBus == "canivore");
    table.observeBus(id, "canivore");
    CHECK(sent.size() == 2);
}

TEST_CASE("snapshot returns only live devices, a detached sink records silently") {
    DeviceTable table;  // no sink yet
    const sdk::DeviceId a = table.announce(0, FakeBridge::makeDescriptor("pwm", 0), FakeBridge::makeSpecs());
    const sdk::DeviceId b = table.announce(0, FakeBridge::makeDescriptor("pwm", 1), FakeBridge::makeSpecs());
    table.retire(a);
    const auto snap = table.snapshot();
    REQUIRE(snap.size() == 1);
    CHECK(snap[0].id == b);
    CHECK_FALSE(snap[0].retired);
}
