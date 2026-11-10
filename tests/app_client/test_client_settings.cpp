// What the client remembers: defaults, a round trip through the store, normalisation and the invite staying out of it.
#include <doctest/doctest.h>

#include <QDir>
#include <QTemporaryDir>

#include "client/client_settings.h"
#include "wpiemu/sdk/core/environment.h"

using namespace wpiemu;
using namespace wpiemu::app;
using namespace wpiemu::app::client;

TEST_CASE("client settings: an empty store gives the defaults (production relay, no diagnostics)") {
    MemoryKeyValueStore store;
    const ClientPrefs p = ClientSettings(store).load();
    CHECK(p == ClientPrefs{});
    CHECK(p.relay == RelayChoice::Production);
    CHECK(p.directAddress.empty());
    CHECK_FALSE(p.showDiagnostics);
    CHECK(p.drive.deadband == doctest::Approx(0.10f));
}

TEST_CASE("client settings: every preference survives a save and a load") {
    MemoryKeyValueStore store;
    ClientSettings settings(store);
    ClientPrefs p;
    p.displayName = "Iron Owls";
    p.directAddress = "192.168.1.20:47300";
    p.relay = RelayChoice::Custom;
    p.customRelay = "relay.example.org:7000";
    p.robotPath = "C:/robots/robot.json";
    p.drive.deadband = 0.2f;
    p.drive.linearScale = 0.75f;
    p.drive.angularScale = 0.5f;
    p.drive.fieldRelative = true;
    p.showDiagnostics = true;
    settings.save(p);

    const ClientPrefs loaded = ClientSettings(store).load();
    CHECK(loaded == p);
}

TEST_CASE("client settings: the choice persists in an ini file across store instances") {
    QTemporaryDir dir;
    REQUIRE(dir.isValid());
    const QString ini = dir.filePath(QStringLiteral("client.ini"));
    ClientPrefs p;
    p.displayName = "Gearheads";
    p.relay = RelayChoice::Development;
    p.showDiagnostics = true;
    {
        QSettingsKeyValueStore store(ini);
        ClientSettings(store).save(p);
    }
    QSettingsKeyValueStore again(ini);
    const ClientPrefs loaded = ClientSettings(again).load();
    CHECK(loaded.displayName == "Gearheads");
    CHECK(loaded.relay == RelayChoice::Development);
    CHECK(loaded.showDiagnostics);
}

TEST_CASE("client settings: unusable stored values fall back and out-of-range numbers are clamped") {
    MemoryKeyValueStore store;
    store.setValue(QString::fromLatin1(kPrefRelay), QStringLiteral("somewhere-else"));
    store.setValue(QString::fromLatin1(kPrefDeadband), QStringLiteral("not a number"));
    store.setValue(QString::fromLatin1(kPrefLinearScale), 7.0);
    store.setValue(QString::fromLatin1(kPrefAngularScale), -3.0);
    const ClientPrefs p = ClientSettings(store).load();
    CHECK(p.relay == RelayChoice::Production);
    CHECK(p.drive.deadband == doctest::Approx(0.10f));
    CHECK(p.drive.linearScale == doctest::Approx(1.f));
    CHECK(p.drive.angularScale == doctest::Approx(0.05f));
}

TEST_CASE("client settings: relay choice keys are stable and round trip") {
    for (const RelayChoice c : {RelayChoice::Production, RelayChoice::Development, RelayChoice::Custom})
        CHECK(relayChoiceFromKey(relayChoiceKey(c)) == c);
    CHECK(std::string(relayChoiceKey(RelayChoice::Production)) == "production");
    CHECK(std::string(relayChoiceKey(RelayChoice::Development)) == "development");
    CHECK(std::string(relayChoiceKey(RelayChoice::Custom)) == "custom");
    CHECK(relayChoiceFromKey("", RelayChoice::Development) == RelayChoice::Development);
}

TEST_CASE("client settings: nothing the client stores looks like an invite") {
    MemoryKeyValueStore store;
    ClientSettings settings(store);
    ClientPrefs p;
    p.directAddress = "host:47300";
    settings.save(p);
    // The preferences have no invite field at all: the secret of a hosting session is never written to disk.
    const char* keys[] = {kPrefDisplayName, kPrefDirectAddress, kPrefRelay, kPrefCustomRelay, kPrefRobotPath, kPrefDeadband,
                          kPrefLinearScale, kPrefAngularScale, kPrefFieldRelative, kPrefDiagnostics};
    for (const char* key : keys) CHECK(std::string(key).find("invite") == std::string::npos);
}

TEST_CASE("client settings: the relay presets follow the stored DNS suffix") {
    MemoryKeyValueStore store;
    ClientSettings settings(store);
    if (!sdk::readEnvironment(kRelayDnsSuffixEnvironment).has_value() && !settings.relayPresets().configured) {
        store.setValue(QString::fromLatin1(kRelayDnsSuffixSetting), QStringLiteral("-zz9"));
        const RelayPresets presets = settings.relayPresets();
        CHECK(presets.configured);
        CHECK(presets.production == "wpiemu-relay-zz9.southcentralus.azurecontainer.io:7777");
        CHECK(presets.development == "wpiemu-relay-dev-zz9.southcentralus.azurecontainer.io:7777");
    }
}
