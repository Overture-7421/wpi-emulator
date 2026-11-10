#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "wpiemu/relay/relay_address.h"
#include "wpiemu/relay/relay_log.h"

using namespace wpiemu::relay;

namespace {

struct Captured {
    std::vector<std::string> lines;
    RelayLog log;
    explicit Captured(LogFormat format = LogFormat::Json, LogLevel min = LogLevel::Info)
        : log(format, [this](const std::string& line) { lines.push_back(line); }, [] { return std::string("2026-01-02T03:04:05.006Z"); }, min) {}
};

}  // namespace

TEST_CASE("relay log: json lines carry ts, level, event and typed fields") {
    Captured c;
    c.log.log(LogLevel::Warn, "join_rejected", {{"room", "ABCD2345"}, {"peerId", 3}, {"detail", "no_such_room"}});
    REQUIRE(c.lines.size() == 1);
    CHECK(c.lines[0] ==
          R"({"ts":"2026-01-02T03:04:05.006Z","level":"warn","event":"join_rejected","room":"ABCD2345","peerId":3,"detail":"no_such_room"})");
}

TEST_CASE("relay log: json escapes quotes, backslashes and control characters") {
    Captured c;
    c.log.log(LogLevel::Info, "e", {{"v", std::string("a\"b\\c\nd\x01") + "e"}});
    REQUIRE(c.lines.size() == 1);
    CHECK(c.lines[0].find("\"v\":\"a\\\"b\\\\c\\nd\\u0001e\"") != std::string::npos);
    CHECK(c.lines[0].find('\n') == std::string::npos);
}

TEST_CASE("relay log: the text format quotes only what needs it") {
    Captured c(LogFormat::Text);
    c.log.log(LogLevel::Info, "stats", {{"rooms", 2}, {"detail", "two words"}, {"empty", ""}});
    REQUIRE(c.lines.size() == 1);
    CHECK(c.lines[0] == R"(2026-01-02T03:04:05.006Z info stats rooms=2 detail="two words" empty="")");
}

TEST_CASE("relay log: lines below the minimum level are dropped") {
    Captured c(LogFormat::Json, LogLevel::Warn);
    c.log.log(LogLevel::Info, "quiet");
    c.log.log(LogLevel::Error, "loud");
    REQUIRE(c.lines.size() == 1);
    CHECK(c.lines[0].find("loud") != std::string::npos);
}

TEST_CASE("relay log: a flood is limited and summarized once") {
    Captured c;
    for (int i = 0; i < 100; ++i) c.log.logLimited(LogLevel::Warn, "bad", {{"i", i}}, 5000, 10);
    CHECK(c.lines.size() == 10);
    c.log.flushSuppressed(5500);
    CHECK(c.lines.size() == 10);  // the window has not passed yet
    c.log.flushSuppressed(6100);
    REQUIRE(c.lines.size() == 11);
    CHECK(c.lines.back().find("log_suppressed") != std::string::npos);
    CHECK(c.lines.back().find("\"count\":90") != std::string::npos);
    c.log.flushSuppressed(8000);
    CHECK(c.lines.size() == 11);
    c.log.logLimited(LogLevel::Warn, "bad", {}, 9000, 10);
    CHECK(c.lines.size() == 12);
}

TEST_CASE("relay log: the UTC timestamp has the ISO form") {
    const std::string ts = utcTimestamp();
    REQUIRE(ts.size() == 24);
    CHECK(ts[4] == '-');
    CHECK(ts[10] == 'T');
    CHECK(ts[19] == '.');
    CHECK(ts.back() == 'Z');
}

TEST_CASE("relay address: IP literals from the transport description, truncated for logs") {
    const auto v4 = sourceAddressOf("enet://203.0.113.77:4000");
    REQUIRE(v4.has_value());
    CHECK(v4->key == "203.0.113.77");
    CHECK(v4->display == "203.0.113.x");
    CHECK_FALSE(v4->loopback);

    const auto mapped = sourceAddressOf("enet://[::ffff:203.0.113.77]:4000");
    REQUIRE(mapped.has_value());
    CHECK(mapped->key == "203.0.113.77");

    const auto v6 = sourceAddressOf("enet://[2001:db8:1:2:3:4:5:6]:4000");
    REQUIRE(v6.has_value());
    CHECK(v6->key == "2001:db8:1:2::/64");
    CHECK(v6->display == "2001:db8:1::x");
    CHECK(sourceAddressOf("enet://[2001:db8:1:2:9:9:9:9]:1")->key == v6->key);

    CHECK(sourceAddressOf("enet://127.0.0.1:1")->loopback);
    CHECK(sourceAddressOf("enet://[::1]:1")->loopback);
    CHECK(sourceAddressOf("enet://[::ffff:127.0.0.1]:1")->loopback);
}

TEST_CASE("relay address: anything that is not an IP literal is unknown, never guessed") {
    CHECK_FALSE(sourceAddressOf("enet://?").has_value());
    CHECK_FALSE(sourceAddressOf("").has_value());
    CHECK_FALSE(sourceAddressOf("pipe:\\\\.\\pipe\\x").has_value());
    CHECK_FALSE(sourceAddressOf("enet://relay.example.com:7777").has_value());
    CHECK_FALSE(sourceAddressOf("enet://999.1.1.1:7777").has_value());
    CHECK_FALSE(sourceAddressOf("enet://[2001:db8::g]:7777").has_value());
}
