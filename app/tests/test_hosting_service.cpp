// The hosting preferences and status the UI binds to (HostingService): persistence, normalisation, the restart hint and kicks.
#include <doctest/doctest.h>

#include <QSignalSpy>

#include "common/hosting_service.h"
#include "ui/key_value_store.h"

using namespace wpiemu::app;
namespace host = wpiemu::app::host;

namespace {

std::shared_ptr<const host::HostingStatus> lobbyWith(std::size_t remotes) {
    host::HostingStatus s;
    s.mode = host::HostingMode::Direct;
    s.phase = host::HostingPhase::Lobby;
    s.invite = "ABCDE-FGHJK-MNPQR-STVWX";
    host::HostingParticipant self;
    self.isHost = true;
    s.roster.push_back(self);
    for (std::size_t i = 0; i < remotes; ++i) {
        host::HostingParticipant p;
        p.id = static_cast<wpiemu::sdk::match::ParticipantId>(i + 1);
        p.name = "Remote " + std::to_string(i + 1);
        s.roster.push_back(p);
    }
    return std::make_shared<const host::HostingStatus>(std::move(s));
}

}  // namespace

TEST_CASE("hosting service: defaults to off with an idle status") {
    MemoryKeyValueStore store;
    HostingService service(store);
    CHECK(service.options() == host::HostingOptions{});
    CHECK(service.validate().ok());
    REQUIRE(service.status() != nullptr);
    CHECK(service.status()->phase == host::HostingPhase::Off);
    CHECK_FALSE(service.restartRequired());
    CHECK_FALSE(service.appliedOptions().has_value());
}

TEST_CASE("hosting service: options persist, are normalised and announce only real changes") {
    MemoryKeyValueStore store;
    HostingService service(store);
    QSignalSpy changed(&service, &HostingService::optionsChanged);

    host::HostingOptions o;
    o.mode = host::HostingMode::Relay;
    o.relayEndpoint = "relay.example.org:7000";
    o.port = 1234;
    o.maxPlayers = 99;   // out of range: clamped, not refused
    o.bindHost = "127.0.0.1";
    service.setOptions(o);
    CHECK(changed.count() == 1);
    CHECK(service.options().maxPlayers == wpiemu::sdk::match::kMaxRobotsPerMatch);
    service.setOptions(service.options());
    CHECK(changed.count() == 1);

    HostingService reloaded(store);
    CHECK(reloaded.options() == service.options());
    CHECK(reloaded.options().mode == host::HostingMode::Relay);
    CHECK(reloaded.options().relayEndpoint == "relay.example.org:7000");
    CHECK(reloaded.options().port == 1234);
    CHECK(reloaded.validate().ok());

    o.relayEndpoint.clear();
    service.setOptions(o);
    CHECK_FALSE(service.validate().ok());   // relay mode without a relay
}

TEST_CASE("hosting service: nothing about the invite is ever stored") {
    MemoryKeyValueStore store;
    HostingService service(store);
    host::HostingOptions o;
    o.mode = host::HostingMode::Direct;
    service.setOptions(o);
    service.publish(lobbyWith(1));
    for (const char* key : {"hosting/invite", "hosting/secret"}) CHECK_FALSE(store.value(QString::fromLatin1(key)).isValid());
    CHECK(store.value(QStringLiteral("hosting/mode")).toString() == QStringLiteral("direct"));
}

TEST_CASE("hosting service: the restart hint compares the running session's options with the preferences") {
    MemoryKeyValueStore store;
    HostingService service(store);
    host::HostingOptions o;
    o.mode = host::HostingMode::Direct;
    service.setOptions(o);
    service.sessionStarted(service.options());
    CHECK_FALSE(service.restartRequired());
    o.port = 4242;
    service.setOptions(o);
    CHECK(service.restartRequired());
    o.port = service.appliedOptions()->port;
    service.setOptions(o);
    CHECK_FALSE(service.restartRequired());
    service.sessionEnded();
    CHECK_FALSE(service.appliedOptions().has_value());
    CHECK(service.status()->phase == host::HostingPhase::Off);
}

TEST_CASE("hosting service: status snapshots are immutable and a change publishes a new one") {
    MemoryKeyValueStore store;
    HostingService service(store);
    QSignalSpy changed(&service, &HostingService::statusChanged);
    const auto first = lobbyWith(1);
    service.publish(first);
    CHECK(changed.count() == 1);
    CHECK(service.status() == first);
    service.publish(lobbyWith(1));   // equal content: no new snapshot, no signal
    CHECK(changed.count() == 1);
    CHECK(service.status() == first);
    service.publish(lobbyWith(2));
    CHECK(changed.count() == 2);
    CHECK(first->roster.size() == 2);   // the old snapshot did not change
    CHECK(service.status()->roster.size() == 3);
    service.publish(nullptr);   // null means idle
    CHECK(service.status()->phase == host::HostingPhase::Off);
}

TEST_CASE("hosting service: kicks go to the backend only for a remote participant of a hosted match") {
    MemoryKeyValueStore store;
    HostingService service(store);
    QSignalSpy kicks(&service, &HostingService::kickRequested);
    service.kick(1);   // not hosting
    CHECK(kicks.count() == 0);
    service.publish(lobbyWith(2));
    service.kick(wpiemu::sdk::match::kHostParticipantId);   // the host cannot be kicked
    CHECK(kicks.count() == 0);
    service.kick(2, QStringLiteral("too slow"));
    REQUIRE(kicks.count() == 1);
    CHECK(kicks.at(0).at(0).toUInt() == 2);
    CHECK(kicks.at(0).at(1).toString() == QStringLiteral("too slow"));
}
