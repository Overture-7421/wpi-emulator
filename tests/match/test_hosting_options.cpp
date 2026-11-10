// Hosting options, the relay address syntax and the constraint filter of the UI's controller (app/host).
#include <doctest/doctest.h>

#include <vector>

#include "host/constrained_controller.h"
#include "host/hosting.h"
#include "wpiemu/sdk/ipc/control_messages.h"
#include "wpiemu/sdk/match/match_protocol.h"

using namespace wpiemu;
namespace host = wpiemu::app::host;
using sdk::proto::SimCommand;
using sdk::proto::SimControl;

TEST_CASE("hosting options: off is always valid and the defaults are sane") {
    host::HostingOptions o;
    CHECK(o.mode == host::HostingMode::Off);
    CHECK(host::validateHosting(o).ok());
    o.maxPlayers = 99;   // ignored while off
    CHECK(host::validateHosting(o).ok());
    host::HostingOptions direct;
    direct.mode = host::HostingMode::Direct;
    CHECK(direct.maxPlayers == sdk::match::kMaxRobotsPerMatch);
    CHECK(host::validateHosting(direct).ok());
}

TEST_CASE("hosting options: player count bounds and the direct bind address") {
    host::HostingOptions o;
    o.mode = host::HostingMode::Direct;
    o.maxPlayers = 1;
    CHECK_FALSE(host::validateHosting(o).ok());
    o.maxPlayers = sdk::match::kMaxRobotsPerMatch + 1;
    CHECK_FALSE(host::validateHosting(o).ok());
    o.maxPlayers = host::kMinHostingPlayers;
    CHECK(host::validateHosting(o).ok());
    o.bindHost.clear();
    CHECK_FALSE(host::validateHosting(o).ok());
    CHECK(host::normalizedHosting(o).bindHost == "::");

    host::HostingOptions wild;
    wild.maxPlayers = 100;
    CHECK(host::normalizedHosting(wild).maxPlayers == sdk::match::kMaxRobotsPerMatch);
    wild.maxPlayers = 0;
    CHECK(host::normalizedHosting(wild).maxPlayers == host::kMinHostingPlayers);
}

TEST_CASE("hosting options: the relay address accepts host:port and relay:// and rejects the rest") {
    const auto plain = host::parseRelayAddress("relay.example.org:7000");
    REQUIRE(plain.ok());
    CHECK(plain.value().host == "relay.example.org");
    CHECK(plain.value().port == 7000);

    const auto scheme = host::parseRelayAddress("  RELAY://10.1.2.3:9  ");
    REQUIRE(scheme.ok());
    CHECK(scheme.value().host == "10.1.2.3");
    CHECK(scheme.value().port == 9);

    const auto withRoom = host::parseRelayAddress("relay://example.org:1234/ABCD1234");   // the room comes from the invite
    REQUIRE(withRoom.ok());
    CHECK(withRoom.value().port == 1234);

    const auto v6 = host::parseRelayAddress("[::1]:8080");
    REQUIRE(v6.ok());
    CHECK(v6.value().host == "::1");

    CHECK_FALSE(host::parseRelayAddress("").ok());
    CHECK_FALSE(host::parseRelayAddress("example.org").ok());       // the port is required
    CHECK_FALSE(host::parseRelayAddress("example.org:0").ok());
    CHECK_FALSE(host::parseRelayAddress("enet://example.org:1").ok());
    CHECK_FALSE(host::parseRelayAddress("bad host:1").ok());

    host::HostingOptions o;
    o.mode = host::HostingMode::Relay;
    CHECK_FALSE(host::validateHosting(o).ok());   // no relay chosen
    o.relayEndpoint = "relay.example.org:7000";
    CHECK(host::validateHosting(o).ok());
}

TEST_CASE("hosting options: mode keys round trip") {
    for (const auto mode : {host::HostingMode::Off, host::HostingMode::Direct, host::HostingMode::Relay})
        CHECK(host::hostingModeFromKey(host::hostingModeKey(mode)) == mode);
    CHECK(host::hostingModeFromKey("nonsense", host::HostingMode::Direct) == host::HostingMode::Direct);
}

namespace {

class RecordingController final : public sdk::ISimController {
public:
    void setDriverStation(const sdk::proto::DsState&) override {}
    void control(const SimControl& c) override { commands.push_back(c); }
    void bindSlot(const sdk::SlotAddress&, const sdk::DeviceKey&) override {}
    void unbindSlot(const sdk::SlotAddress&) override {}
    void setStartPlacement(const std::optional<sdk::RobotStartPose>&) override {}
    std::vector<SimControl> commands;
};

SimControl scale(double s) {
    SimControl c;
    c.command = SimCommand::SetTimeScale;
    c.timeScale = s;
    return c;
}

SimControl step() {
    SimControl c;
    c.command = SimCommand::Step;
    return c;
}

}  // namespace

TEST_CASE("constrained controller: without peers everything is forwarded") {
    RecordingController inner;
    host::ConstrainedController c(inner);
    c.control(scale(0.5));
    c.control(step());
    REQUIRE(inner.commands.size() == 2);
    CHECK(inner.commands[0].timeScale == 0.5);
    CHECK_FALSE(c.timeScaleLocked());
    CHECK_FALSE(c.stepDisabled());
}

TEST_CASE("constrained controller: with peers the time scale is held at 1x, Step is dropped, and the scale returns afterwards") {
    RecordingController inner;
    host::ConstrainedController c(inner, 2.0);

    c.apply(true, true);   // a client joined: the scale the runner was started with (2x) is forced to 1x
    REQUIRE(inner.commands.size() == 1);
    CHECK(inner.commands[0].command == SimCommand::SetTimeScale);
    CHECK(inner.commands[0].timeScale == 1.0);
    CHECK(c.timeScaleLocked());
    CHECK(c.stepDisabled());

    c.control(scale(4.0));   // the UI asks while locked: remembered, not forwarded
    c.control(step());
    SimControl pause;
    pause.command = SimCommand::Pause;
    c.control(pause);
    REQUIRE(inner.commands.size() == 2);
    CHECK(inner.commands[1].command == SimCommand::Pause);

    c.apply(true, true);   // no change: nothing new is sent
    CHECK(inner.commands.size() == 2);

    c.apply(false, false);   // the last client left: what the user asked for comes back
    REQUIRE(inner.commands.size() == 3);
    CHECK(inner.commands[2].timeScale == 4.0);
    c.control(step());
    CHECK(inner.commands.size() == 4);
}

TEST_CASE("constrained controller: a 1x request never sends redundant commands when the lock changes") {
    RecordingController inner;
    host::ConstrainedController c(inner);
    c.apply(true, true);
    c.apply(false, false);
    CHECK(inner.commands.empty());
}
