#include <doctest/doctest.h>

#include <string>

#include "wpiemu/net/endpoint.h"

using wpiemu::net::EndpointScheme;
using wpiemu::net::NetEndpoint;
using wpiemu::sdk::ErrorCode;

TEST_CASE("net endpoint: enet IPv4, hostname and IPv6 forms parse and format") {
    auto v4 = NetEndpoint::parse("enet://192.168.1.20:7777");
    REQUIRE(v4.ok());
    CHECK(v4.value().scheme == EndpointScheme::Enet);
    CHECK(v4.value().host == "192.168.1.20");
    CHECK(v4.value().port == 7777);
    CHECK(v4.value().roomId.empty());
    CHECK(v4.value().format() == "enet://192.168.1.20:7777");

    auto name = NetEndpoint::parse("enet://robot-host.example.com:1");
    REQUIRE(name.ok());
    CHECK(name.value().host == "robot-host.example.com");
    CHECK(name.value().port == 1);

    auto v6 = NetEndpoint::parse("enet://[::1]:7777");
    REQUIRE(v6.ok());
    CHECK(v6.value().host == "::1");
    CHECK(v6.value().format() == "enet://[::1]:7777");

    auto full = NetEndpoint::parse("enet://[2001:db8:0:0:0:0:0:1]:65535");
    REQUIRE(full.ok());
    CHECK(full.value().port == 65535);

    auto mapped = NetEndpoint::parse("enet://[::ffff:192.0.2.1]:9");
    REQUIRE(mapped.ok());
    CHECK(mapped.value().host == "::ffff:192.0.2.1");

    auto ephemeral = NetEndpoint::parse("ENET://0.0.0.0:0");
    REQUIRE(ephemeral.ok());
    CHECK(ephemeral.value().port == 0);
    CHECK(ephemeral.value().format() == "enet://0.0.0.0:0");
}

TEST_CASE("net endpoint: relay endpoints carry a validated room id") {
    auto relay = NetEndpoint::parse("relay://relay.example.com:7778/ABCDEFGH");
    REQUIRE(relay.ok());
    CHECK(relay.value().scheme == EndpointScheme::Relay);
    CHECK(relay.value().host == "relay.example.com");
    CHECK(relay.value().port == 7778);
    CHECK(relay.value().roomId == "ABCDEFGH");
    CHECK(relay.value().format() == "relay://relay.example.com:7778/ABCDEFGH");

    auto v6 = NetEndpoint::parse("relay://[::1]:7778/01234567");
    REQUIRE(v6.ok());
    CHECK(v6.value().format() == "relay://[::1]:7778/01234567");

    for (const char* text : {"relay://h:1/ABCDEFG",        // too short
                             "relay://h:1/ABCDEFGHJ",      // too long
                             "relay://h:1/ABCDEFGI",       // I is not in the alphabet
                             "relay://h:1/abcdefgh",       // lower case
                             "relay://h:1/",               // empty room
                             "relay://h:1",                // no room
                             "relay://h:1/ABCDEFGH/more",  // extra path
                             "relay://h:0/ABCDEFGH"}) {    // port 0
        CAPTURE(text);
        auto bad = NetEndpoint::parse(text);
        REQUIRE_FALSE(bad.ok());
        CHECK(bad.error().code == ErrorCode::InvalidArgument);
    }
}

TEST_CASE("net endpoint: malformed endpoints are rejected") {
    for (const char* text : {"",
                             "enet://",
                             "enet://:7777",
                             "enet://host",
                             "enet://host:",
                             "enet://host:abc",
                             "enet://host:65536",
                             "enet://host:-1",
                             "enet://host:123456",
                             "enet://host:80/path",
                             "enet://host:80/",
                             "enet://ho st:80",
                             "enet://-host:80",
                             "enet://host-:80",
                             "enet://ho..st:80",
                             "enet://host_name:80",
                             "enet://999.1.1.1:80",
                             "enet://1.2.3:80",
                             "enet://1.2.3.4.5:80",
                             "enet://01.2.3.4:80",
                             "enet://::1:7777",
                             "enet://[::1:7777",
                             "enet://[::1]7777",
                             "enet://[::1]",
                             "enet://[]:7777",
                             "enet://[1:2:3]:7777",
                             "enet://[1::2::3]:7777",
                             "enet://[:::]:7777",
                             "enet://[fe80::1%eth0]:7777",
                             "enet://[g::1]:7777",
                             "enet://[1:2:3:4:5:6:7:8:9]:7777",
                             "enet://user@host:80",
                             "http://host:80",
                             "host:80",
                             "://host:80"}) {
        CAPTURE(text);
        auto bad = NetEndpoint::parse(text);
        REQUIRE_FALSE(bad.ok());
        CHECK(bad.error().code == ErrorCode::InvalidArgument);
    }
}

TEST_CASE("net endpoint: format and parse round trip") {
    for (const char* text : {"enet://localhost:1", "enet://[fe80::1]:50000", "enet://10.0.0.1:65535",
                             "relay://r.example.org:443/ZZZZZZZZ"}) {
        CAPTURE(text);
        auto parsed = NetEndpoint::parse(text);
        REQUIRE(parsed.ok());
        auto again = NetEndpoint::parse(parsed.value().format());
        REQUIRE(again.ok());
        CHECK(again.value() == parsed.value());
    }
}

TEST_CASE("net endpoint: a person's relay address becomes the relay endpoint of the room") {
    const auto plain = NetEndpoint::parseRelayAddress("relay.example.org:7000", "ABCD1234");
    REQUIRE(plain.ok());
    CHECK(plain.value().format() == "relay://relay.example.org:7000/ABCD1234");

    const auto scheme = NetEndpoint::parseRelayAddress("  RELAY://10.1.2.3:9/ignored  ", "ABCD1234");
    REQUIRE(scheme.ok());
    CHECK(scheme.value().host == "10.1.2.3");
    CHECK(scheme.value().roomId == "ABCD1234");

    const auto v6 = NetEndpoint::parseRelayAddress("[::1]:8080", "ABCD1234");
    REQUIRE(v6.ok());
    CHECK(v6.value().host == "::1");

    for (const char* bad : {"", "example.org", "example.org:0", "enet://example.org:1", "bad host:1"}) {
        CAPTURE(bad);
        const auto r = NetEndpoint::parseRelayAddress(bad, "ABCD1234");
        CHECK_FALSE(r.ok());
        if (!r.ok()) CHECK_FALSE(r.error().message.empty());
    }
}

TEST_CASE("net endpoint: a person's direct address defaults the port") {
    const auto bare = NetEndpoint::parseDirectAddress("192.168.1.20");
    REQUIRE(bare.ok());
    CHECK(bare.value().format() == "enet://192.168.1.20:47300");

    const auto port = NetEndpoint::parseDirectAddress(" host.example:5000 ");
    REQUIRE(port.ok());
    CHECK(port.value().port == 5000);

    const auto scheme = NetEndpoint::parseDirectAddress("enet://10.0.0.2");
    REQUIRE(scheme.ok());
    CHECK(scheme.value().port == 47300);

    const auto v6 = NetEndpoint::parseDirectAddress("[::1]");
    REQUIRE(v6.ok());
    CHECK(v6.value().format() == "enet://[::1]:47300");
    const auto v6port = NetEndpoint::parseDirectAddress("[::1]:6000");
    REQUIRE(v6port.ok());
    CHECK(v6port.value().port == 6000);

    for (const char* bad : {"", "relay://host:1", "bad host", "host:99999", "[::1"}) {
        CAPTURE(bad);
        CHECK_FALSE(NetEndpoint::parseDirectAddress(bad).ok());
    }
}
