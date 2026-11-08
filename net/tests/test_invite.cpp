#include <doctest/doctest.h>

#include <array>
#include <set>
#include <string>

#include "wpiemu/net/invite.h"
#include "wpiemu/net/os_random.h"
#include "wpiemu/sdk/match/relay_messages.h"

using namespace wpiemu;
using namespace wpiemu::net;

namespace {

std::string hex(std::span<const std::uint8_t> bytes) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    for (const std::uint8_t b : bytes) {
        out.push_back(digits[b >> 4]);
        out.push_back(digits[b & 15]);
    }
    return out;
}

InviteSecret parsed(std::string_view text) {
    auto secret = parseInvite(text);
    REQUIRE(secret.ok());
    return std::move(secret).value();
}

}  // namespace

TEST_CASE("os random fills a buffer and two draws differ") {
    std::array<std::byte, 64> a{};
    std::array<std::byte, 64> b{};
    REQUIRE(fillRandom(a).ok());
    REQUIRE(fillRandom(b).ok());
    CHECK(a != b);
    CHECK(a != std::array<std::byte, 64>{});
    std::array<std::byte, 1000> large{};
    CHECK(fillRandom(large).ok());
    CHECK(fillRandom(std::span<std::byte>{}).ok());
}

TEST_CASE("invite text encodes and decodes round trip") {
    const InviteSecret secret = parsed("ABCDE-FGHJK-MNPQR-STVWX");
    CHECK(inviteText(secret) == "ABCDE-FGHJK-MNPQR-STVWX");
    CHECK(hex(secret.bytes()) == "52d8d73e1194e95b5f19d6f9d0");

    for (int i = 0; i < 50; ++i) {
        auto generated = generateInvite();
        REQUIRE(generated.ok());
        const std::string text = inviteText(generated.value());
        CHECK(text.size() == 23);
        auto again = parseInvite(text);
        REQUIRE(again.ok());
        CHECK(again.value() == generated.value());
        CHECK(inviteText(again.value()) == text);
    }
}

TEST_CASE("invite parser tolerates case, separators and lookalikes") {
    const InviteSecret canonical = parsed("01234-56789-ABCDE-FGHJK");
    CHECK(parsed("0123456789abcdefghjk") == canonical);
    CHECK(parsed("  01234 56789 abcde fghjk  ") == canonical);
    CHECK(parsed("0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-G-H-J-K") == canonical);
    CHECK(parsed("o1234-56789-ABCDE-FGHJK") == canonical);
    CHECK(parsed("O1234-56789-ABCDE-FGHJK") == canonical);
    CHECK(parsed("0I234-56789-ABCDE-FGHJK") == parsed("01234-56789-ABCDE-FGHJK"));
    CHECK(parsed("0l234-56789-ABCDE-FGHJK") == parsed("01234-56789-ABCDE-FGHJK"));
    CHECK(parsed("00000-00000-00000-0000O").isZero());
}

TEST_CASE("invite parser rejects malformed codes with a clear error") {
    for (const char* bad : {"", "ABCDE", "ABCDE-FGHJK-MNPQR-STVW", "ABCDE-FGHJK-MNPQR-STVWXY", "-----"}) {
        auto result = parseInvite(bad);
        REQUIRE_FALSE(result.ok());
        CHECK(result.error().code == sdk::ErrorCode::ParseError);
        CHECK(result.error().message.find("20 characters") != std::string::npos);
    }
    for (const char* bad : {"ABCDE-FGHJK-MNPQR-STVWU", "ABCDE-FGHJK-MNPQR-STV!X", "ABCDE_FGHJK_MNPQR_STVWX"}) {
        auto result = parseInvite(bad);
        REQUIRE_FALSE(result.ok());
        CHECK(result.error().code == sdk::ErrorCode::ParseError);
        CHECK(result.error().message.find("invalid character") != std::string::npos);
    }
}

TEST_CASE("invite derivations match the pinned vectors") {
    const InviteSecret secret = parsed("ABCDE-FGHJK-MNPQR-STVWX");
    CHECK(roomId(secret) == "KKJF3WY0");
    CHECK(hex(hostAuth(secret)) == "b5e94eaae468eafb5f0a080c1a1e17b0da4ce857492dbf4cf4b5e5e0a4cb8872");
    CHECK(hex(channelKey(secret)) == "b41c9894ff5a4e98fcb42cfb08ea1b6b9d76b92a7c7b4f980450a2e3f0cbba35");

    const InviteSecret zero = parsed("00000-00000-00000-00000");
    CHECK(roomId(zero) == "5YCPRXBZ");
    CHECK(hex(hostAuth(zero)) == "dedfd76f2af4847a1b31851b0ffd40766d3960af11ba2fe6f5cf58656570ec9d");
    CHECK(hex(channelKey(zero)) == "78d4346f435f9d590c265a34ccf63857e27769cb5e1956ed0a09abf8a20016fc");
}

TEST_CASE("room id, host auth and channel key are independent derivations in the relay alphabet") {
    std::set<std::string> ids;
    for (int i = 0; i < 100; ++i) {
        auto generated = generateInvite();
        REQUIRE(generated.ok());
        const InviteSecret& secret = generated.value();
        const std::string id = roomId(secret);
        CHECK(sdk::match::relay::isValidRoomId(id));
        for (const char c : id) CHECK(std::string(sdk::match::relay::kRoomIdAlphabet).find(c) != std::string::npos);
        ids.insert(id);
        const auto auth = hostAuth(secret);
        const auto key = channelKey(secret);
        CHECK(auth != key);
        CHECK(std::string(auth.begin(), auth.begin() + 8) != id);
        CHECK(constantTimeEqual(hostAuth(secret), auth));
    }
    CHECK(ids.size() >= 99);
}

TEST_CASE("generated secrets differ, are never zero and keep the trailing bits clear") {
    auto a = generateInvite();
    auto b = generateInvite();
    REQUIRE(a.ok());
    REQUIRE(b.ok());
    CHECK(a.value() != b.value());
    CHECK_FALSE(a.value().isZero());
    CHECK_FALSE(b.value().isZero());
    for (int i = 0; i < 100; ++i) {
        auto s = generateInvite();
        REQUIRE(s.ok());
        CHECK((s.value().bytes().back() & 0x0f) == 0);
    }
}

TEST_CASE("fromBytes rejects set trailing bits") {
    InviteSecret::Bytes bytes{};
    bytes.back() = 0x01;
    CHECK_FALSE(InviteSecret::fromBytes(bytes).ok());
    bytes.back() = 0x10;
    CHECK(InviteSecret::fromBytes(bytes).ok());
}

TEST_CASE("constant time comparison") {
    const std::array<std::uint8_t, 4> a{1, 2, 3, 4};
    std::array<std::uint8_t, 4> b = a;
    CHECK(constantTimeEqual(a, b));
    b[3] = 5;
    CHECK_FALSE(constantTimeEqual(a, b));
    b = a;
    b[0] = 0;
    CHECK_FALSE(constantTimeEqual(a, b));
    CHECK_FALSE(constantTimeEqual(a, std::span<const std::uint8_t>(b).first(3)));
    CHECK(constantTimeEqual({}, {}));
}

TEST_CASE("wipe zeroes the secret and moves leave nothing behind") {
    InviteSecret secret = parsed("ABCDE-FGHJK-MNPQR-STVWX");
    InviteSecret copy = secret;
    CHECK(copy == secret);
    secret.wipe();
    CHECK(secret.isZero());
    CHECK_FALSE(copy.isZero());
    InviteSecret moved = std::move(copy);
    CHECK(copy.isZero());  // NOLINT: a moved-from secret is deliberately wiped
    CHECK(inviteText(moved) == "ABCDE-FGHJK-MNPQR-STVWX");
}
