#include <doctest/doctest.h>

#include <string>
#include <string_view>
#include <vector>

#include "match_samples.h"
#include "wpiemu/ipc/binary_codec.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::proto;
using wpiemu::test_ipc::matchSamples;
using wpiemu::test_ipc::NamedMessage;

// The practice-match and relay messages (MessageType 13..28) pinned to hex captured from the codec when it was written.
// Layout: docs/reference/extension-ipc.md. These bytes are the compatibility contract between a host, its clients and
// the relay of different builds: a change here is a protocol change.
namespace {

struct Golden {
    const char* name;
    const char* hex;
};

const std::vector<Golden>& goldens() {
    static const std::vector<Golden> table = {
        {"Join",
         "010d00010000000b5465616d2032353420c3a90867616d6532303236ea070000"
         "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf197b2263686173736973223a7b226d6173"
         "734b67223a35307d7d"},
        {"JoinAccept",
         "010e000300887766554433221107000000000000000488130000000000001e005a"
         "000000c03f000010c00000003f00000000000000000000803e0000403fa0a1a2a3"
         "a4a5a6a7a8a9aaabacadaeaf01393000000000000048d9ad0300000000020d0000"
         "0004486f73740014000000010c000300035a65640417000f0000"},
        {"JoinReject", "010f00041b726f626f742e6472697665747261696e2e776865656c436f756e74"},
        {"Roster",
         "01100008070605040302010900000000000000030d00000004486f737400140000"
         "00010c00010003416d790115000500000c00020003426f620516000a0000"},
        {"Leave", "01110003627965"},
        {"MatchInputPacket",
         "0112008877665544332211070000000000000003290029000000110400000000000004"
         "000048410004000050c000f800000000ff7f0000c84200802e0000002900"
         "2a000000120400000000000004000048410004000050c000f800000000ff7f0000c84200"
         "802f00000029002b000000130400000000000004000048410004000050c000f800000000"
         "ff7f0000c842008030000000"},
        {"MatchSnapshot",
         "01130088776655443322110700000000000000e110000006120f0000000000300558"
         "26010000006300000002120f0000000000fdff1300020101010000a5420a70686173"
         "652e6175746f022c000000dc05000042f5ffffe80350fb480d300202640038ff2c01"
         "70fe090000000201000000404102000000003f2c000100b80b000084eaffffd00750"
         "fb480d300202640038ff2c0170fe0a0000000201000000404102000000003f012400"
         "0100e0100000020b000000160000000221000000640038ff05002c000000d4fe9001"
         "00000207000000f6420800000080bf"},
        {"MatchSnapshotAck", "01140088776655443322110700000000000000e1100000"},
        {"MatchControl",
         "011500038877665544332211080000000000000002004d00000063c5540000000000"
         "327906000000000090186f7e000000000f72656d6f76656420627920686f7374"},
        {"RegisterRoom",
         "0116000100000008374b33515a39584d01080f161d242b323940474e555c636a7178"
         "7f868d949ba2a9b0b7bec5ccd3da"},
        {"RoomRegistered", "011700010500"},
        {"RoomRejected", "01180002"},
        {"JoinRoom", "01190001000000083031323341424344"},
        {"JoinResult", "011a000005ffff"},
        {"PeerJoined", "011b000400"},
        {"PeerLeft", "011c00040003"},
    };
    return table;
}

std::vector<std::byte> fromHex(std::string_view text) {
    std::vector<std::byte> out;
    int high = -1;
    for (const char c : text) {
        const int v = c <= '9' ? c - '0' : c - 'a' + 10;
        if (high < 0) {
            high = v;
        } else {
            out.push_back(static_cast<std::byte>(high * 16 + v));
            high = -1;
        }
    }
    return out;
}

std::string toHex(const std::vector<std::byte>& bytes) {
    static constexpr char digits[] = "0123456789abcdef";
    std::string s;
    for (const std::byte b : bytes) {
        s += digits[static_cast<unsigned>(b) >> 4];
        s += digits[static_cast<unsigned>(b) & 15u];
    }
    return s;
}

const Golden& goldenOf(std::string_view name) {
    for (const Golden& g : goldens())
        if (name == g.name) return g;
    FAIL("no golden named " << name);
    return goldens().front();
}

}  // namespace

TEST_CASE("match codec golden: every match and relay message has a sample and a golden") {
    const std::vector<NamedMessage> samples = matchSamples();
    CHECK(samples.size() == goldens().size());
    CHECK(samples.size() == std::variant_size_v<Message> - kExtensionMessageCount);
    std::vector<bool> seen(std::variant_size_v<Message>, false);
    for (const NamedMessage& s : samples) {
        CAPTURE(s.name);
        CHECK(isMatchMessage(s.message));
        CHECK_FALSE(seen[s.message.index()]);
        seen[s.message.index()] = true;
        (void)goldenOf(s.name);
    }
    for (std::size_t i = kExtensionMessageCount; i < seen.size(); ++i) CHECK_MESSAGE(seen[i], "message index " << i);
}

TEST_CASE("match codec golden: every message encodes to its pinned bytes") {
    const BinaryMessageCodec codec;
    for (const NamedMessage& s : matchSamples()) {
        CAPTURE(s.name);
        std::vector<std::byte> out;
        REQUIRE(codec.encode(s.message, out).ok());
        CHECK(toHex(out) == toHex(fromHex(goldenOf(s.name).hex)));
    }
}

TEST_CASE("match codec golden: the pinned bytes decode to the sample and compare equal") {
    const BinaryMessageCodec codec;
    for (const NamedMessage& s : matchSamples()) {
        CAPTURE(s.name);
        const Result<Message> decoded = codec.decode(fromHex(goldenOf(s.name).hex));
        REQUIRE(decoded.ok());
        CHECK(decoded.value().index() == s.message.index());
        CHECK(typeOf(decoded.value()) == typeOf(s.message));
        CHECK(wpiemu::test_ipc::sameMatchMessage(decoded.value(), s.message));
    }
}

TEST_CASE("match codec golden: the type byte of each frame is the MessageType value") {
    for (const NamedMessage& s : matchSamples()) {
        CAPTURE(s.name);
        const std::vector<std::byte> frame = fromHex(goldenOf(s.name).hex);
        REQUIRE(frame.size() >= 3);
        CHECK(std::to_integer<int>(frame[0]) == kWireFormatVersion);
        CHECK(std::to_integer<int>(frame[1]) == static_cast<int>(typeOf(s.message)));
        CHECK(std::to_integer<int>(frame[2]) == 0);
    }
}

TEST_CASE("match codec golden: a newer peer's trailing bytes are ignored") {
    const BinaryMessageCodec codec;
    for (const NamedMessage& s : matchSamples()) {
        CAPTURE(s.name);
        std::vector<std::byte> frame = fromHex(goldenOf(s.name).hex);
        frame.push_back(std::byte{0xAB});
        frame.push_back(std::byte{0xCD});
        const Result<Message> decoded = codec.decode(frame);
        REQUIRE(decoded.ok());
        CHECK(wpiemu::test_ipc::sameMatchMessage(decoded.value(), s.message));
    }
}

TEST_CASE("match codec golden: HALSim frames are unchanged by the match types (a Hello still decodes)") {
    const BinaryMessageCodec codec;
    Hello hello;
    hello.wpilibYear = 2026;
    std::vector<std::byte> out;
    REQUIRE(codec.encode(Message{hello}, out).ok());
    const Result<Message> back = codec.decode(out);
    REQUIRE(back.ok());
    CHECK(std::holds_alternative<Hello>(back.value()));
}
