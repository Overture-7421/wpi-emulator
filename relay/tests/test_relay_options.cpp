#include <doctest/doctest.h>

#include <map>

#include "wpiemu/relay/relay_options.h"

using namespace wpiemu::relay;
using wpiemu::sdk::ErrorCode;

namespace {

using Env = std::map<std::string, std::string>;

wpiemu::sdk::Result<RelayOptions> parse(const std::vector<std::string>& args, const Env& env = {}) {
    return parseRelayOptions(args, [&env](const std::string& name) -> std::optional<std::string> {
        const auto it = env.find(name);
        if (it == env.end()) return std::nullopt;
        return it->second;
    });
}

}  // namespace

TEST_CASE("relay options: defaults") {
    const auto parsed = parse({});
    REQUIRE(parsed);
    const RelayOptions& o = parsed.value();
    CHECK(o.command == RelayCommand::Run);
    CHECK(o.port == 7777);
    CHECK(o.bindHost == "::");
    CHECK(o.logFormat == LogFormat::Json);
    CHECK(o.server.table.maxRooms == 100);
    CHECK(o.server.table.maxPeersPerRoom == 6);
    CHECK(o.server.table.roomTtlWithoutHostMs == 30'000);
    CHECK(o.server.table.peerBytesPerSecond == 64 * 1024);
    CHECK(o.server.table.lockoutMaxFailures == 10);
    CHECK(o.server.table.lockoutWindowMs == 60'000);
    CHECK(o.server.table.lockoutDurationMs == 300'000);
    CHECK(o.maxConnections == 100 * 7 + 64);
}

TEST_CASE("relay options: the command line sets values") {
    const auto parsed = parse({"--port", "9000", "--bind=127.0.0.1", "--max-rooms", "5", "--room-ttl-ms", "2000", "--log", "text",
                               "--peer-bytes-per-sec=4096", "--lockout-failures", "3", "--lockout-window-s", "10",
                               "--lockout-duration-s", "20"});
    REQUIRE(parsed);
    const RelayOptions& o = parsed.value();
    CHECK(o.port == 9000);
    CHECK(o.bindHost == "127.0.0.1");
    CHECK(o.server.table.maxRooms == 5);
    CHECK(o.server.table.roomTtlWithoutHostMs == 2000);
    CHECK(o.logFormat == LogFormat::Text);
    CHECK(o.server.table.peerBytesPerSecond == 4096);
    CHECK(o.server.table.lockoutMaxFailures == 3);
    CHECK(o.server.table.lockoutWindowMs == 10'000);
    CHECK(o.server.table.lockoutDurationMs == 20'000);
    CHECK(o.maxConnections == 5 * 7 + 64);
}

TEST_CASE("relay options: the environment wins over the command line") {
    const Env env = {{"WPIEMU_RELAY_PORT", "8123"},          {"WPIEMU_RELAY_BIND", "0.0.0.0"},
                     {"WPIEMU_RELAY_MAX_ROOMS", "12"},        {"WPIEMU_RELAY_ROOM_TTL_MS", "45000"},
                     {"WPIEMU_RELAY_PEER_BYTES_PER_SEC", "32768"}, {"WPIEMU_RELAY_LOG", "text"},
                     {"WPIEMU_RELAY_LOCKOUT_FAILURES", "7"}};
    const auto parsed = parse({"--port", "9000", "--bind", "::1", "--max-rooms", "5", "--log", "json"}, env);
    REQUIRE(parsed);
    const RelayOptions& o = parsed.value();
    CHECK(o.port == 8123);
    CHECK(o.bindHost == "0.0.0.0");
    CHECK(o.server.table.maxRooms == 12);
    CHECK(o.server.table.roomTtlWithoutHostMs == 45'000);
    CHECK(o.server.table.peerBytesPerSecond == 32768);
    CHECK(o.logFormat == LogFormat::Text);
    CHECK(o.server.table.lockoutMaxFailures == 7);
}

TEST_CASE("relay options: an empty environment variable is ignored") {
    const auto parsed = parse({"--port", "9000"}, {{"WPIEMU_RELAY_PORT", ""}});
    REQUIRE(parsed);
    CHECK(parsed.value().port == 9000);
}

TEST_CASE("relay options: invalid values are errors that name the setting") {
    struct Case {
        std::vector<std::string> args;
        Env env;
        std::string mentions;
    };
    const std::vector<Case> cases = {
        {{}, {{"WPIEMU_RELAY_PORT", "0"}}, "WPIEMU_RELAY_PORT"},
        {{}, {{"WPIEMU_RELAY_PORT", "70000"}}, "WPIEMU_RELAY_PORT"},
        {{}, {{"WPIEMU_RELAY_PORT", "77x"}}, "WPIEMU_RELAY_PORT"},
        {{}, {{"WPIEMU_RELAY_PORT", "-1"}}, "WPIEMU_RELAY_PORT"},
        {{}, {{"WPIEMU_RELAY_MAX_ROOMS", "0"}}, "WPIEMU_RELAY_MAX_ROOMS"},
        {{}, {{"WPIEMU_RELAY_MAX_ROOMS", "100000"}}, "WPIEMU_RELAY_MAX_ROOMS"},
        {{}, {{"WPIEMU_RELAY_ROOM_TTL_MS", "5"}}, "WPIEMU_RELAY_ROOM_TTL_MS"},
        {{}, {{"WPIEMU_RELAY_PEER_BYTES_PER_SEC", "abc"}}, "WPIEMU_RELAY_PEER_BYTES_PER_SEC"},
        {{}, {{"WPIEMU_RELAY_LOG", "xml"}}, "WPIEMU_RELAY_LOG"},
        {{}, {{"WPIEMU_RELAY_BIND", "bad host"}}, "WPIEMU_RELAY_BIND"},
        {{"--port", "nope"}, {}, "--port"},
        {{"--max-peers", "7"}, {}, "--max-peers"},
        {{"--port"}, {}, "missing value"},
        {{"--frobnicate", "1"}, {}, "--frobnicate"},
    };
    for (const Case& c : cases) {
        CAPTURE(c.mentions);
        const auto parsed = parse(c.args, c.env);
        REQUIRE_FALSE(parsed);
        CHECK(parsed.error().code == ErrorCode::InvalidArgument);
        CHECK(parsed.error().message.find(c.mentions) != std::string::npos);
    }
}

TEST_CASE("relay options: help and version never fail on the environment") {
    for (const char* flag : {"--help", "-h", "--version"}) {
        const auto parsed = parse({flag}, {{"WPIEMU_RELAY_PORT", "garbage"}});
        REQUIRE(parsed);
        CHECK(parsed.value().command == (std::string(flag) == "--version" ? RelayCommand::Version : RelayCommand::Help));
    }
    const std::string usage = relayUsage();
    for (const char* name : {"WPIEMU_RELAY_PORT", "WPIEMU_RELAY_BIND", "WPIEMU_RELAY_MAX_ROOMS", "WPIEMU_RELAY_ROOM_TTL_MS",
                             "WPIEMU_RELAY_PEER_BYTES_PER_SEC", "WPIEMU_RELAY_LOG", "--selfcheck", "--version"})
        CHECK(usage.find(name) != std::string::npos);
}

TEST_CASE("relay options: selfcheck takes a port and a host and honours the environment port") {
    const auto parsed = parse({"--selfcheck", "--port", "9100", "--host", "relay.example"});
    REQUIRE(parsed);
    CHECK(parsed.value().command == RelayCommand::SelfCheck);
    CHECK(parsed.value().port == 9100);
    CHECK(parsed.value().selfCheckHost == "relay.example");
    CHECK(parsed.value().selfCheckTimeout == std::chrono::milliseconds(2000));

    const auto fromEnv = parse({"--selfcheck"}, {{"WPIEMU_RELAY_PORT", "7788"}});
    REQUIRE(fromEnv);
    CHECK(fromEnv.value().port == 7788);
    CHECK(fromEnv.value().selfCheckHost == "127.0.0.1");
}
