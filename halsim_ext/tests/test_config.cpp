#include <doctest/doctest.h>

#include "wpiemu/halsim_ext/config.h"
#include "wpiemu/sdk/testing/environment.h"

using namespace wpiemu::halsim;
using wpiemu::sdk::testing::envOf;

TEST_CASE("config defaults: no endpoint means the extension is inert") {
    std::vector<std::string> warnings;
    const ExtensionConfig c = parseExtensionConfig(envOf({}), &warnings);
    CHECK(c.endpoint.empty());
    CHECK_FALSE(c.disabled);
    CHECK(c.clock == ClockPreference::FollowEmulator);
    CHECK(c.watchdogEnabled);
    CHECK(c.reconnect);
    CHECK_FALSE(c.pollWhileConstructing);
    CHECK(c.connectTimeout.count() == 10000);
    CHECK(warnings.empty());
    CHECK(parseExtensionConfig(envOf({{"WPIEMU_LIVE_DISCOVERY", "1"}})).pollWhileConstructing);
}

TEST_CASE("config reads the ipc-published variable names and the short aliases") {
    CHECK(parseExtensionConfig(envOf({{"WPIEMU_IPC_ENDPOINT", "\\\\.\\pipe\\wpiemu-1"}})).endpoint == "\\\\.\\pipe\\wpiemu-1");
    CHECK(parseExtensionConfig(envOf({{"WPIEMU_ENDPOINT", "/tmp/x.sock"}})).endpoint == "/tmp/x.sock");
    // the ipc name wins over the alias
    CHECK(parseExtensionConfig(envOf({{"WPIEMU_IPC_ENDPOINT", "a"}, {"WPIEMU_ENDPOINT", "b"}})).endpoint == "a");

    const ExtensionConfig c = parseExtensionConfig(envOf({{"WPIEMU_WATCHDOG_MS", "750"}, {"WPIEMU_CONNECT_TIMEOUT_MS", "1234"}}));
    CHECK(c.proposedWatchdogMs == 750);
    CHECK(c.connectTimeout.count() == 1234);
}

TEST_CASE("config parses switches and keeps defaults for malformed values") {
    std::vector<std::string> warnings;
    const ExtensionConfig c = parseExtensionConfig(
        envOf({{"WPIEMU_CLOCK", "realtime"},
               {"WPIEMU_WATCHDOG", "off"},
               {"WPIEMU_RECONNECT", "0"},
               {"WPIEMU_LOG_LEVEL", "debug"},
               {"WPIEMU_DISABLE", "1"},
               {"WPIEMU_PROGRAM_NAME", " my robot "},
               {"WPIEMU_HANDSHAKE_TIMEOUT_MS", "banana"},
               {"WPIEMU_IPC_CONNECT_TIMEOUT_MS", "-5"}}),
        &warnings);
    CHECK(c.clock == ClockPreference::Realtime);
    CHECK_FALSE(c.watchdogEnabled);
    CHECK_FALSE(c.reconnect);
    CHECK(c.logLevel == wpiemu::sdk::LogLevel::Debug);
    CHECK(c.disabled);
    CHECK(c.programName == "my robot");
    CHECK(c.handshakeTimeout.count() == 5000);
    CHECK(c.connectTimeout.count() == 10000);
    CHECK(warnings.size() == 2);
}
