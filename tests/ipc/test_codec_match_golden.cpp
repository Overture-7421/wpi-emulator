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
         "010e000300887766554433221107000000000000000488130000000000001e005a00"
         "0000c03f000010c00000003f00000000000000000000803e0000403fa0a1a2a3a4a5"
         "a6a7a8a9aaabacadaeaf01393000000000000048d9ad030000000002de0000000448"
         "6f7374001400000001cf00bc028a027800ceff19009cffc8000200000a00ecff1e00"
         "004000000201200328005a00ff33000101d4fe0000c201000000e0000064006400fa"
         "00010203040000fa00fa0056ff32002600010006fffa0056ff330026000200fa0006"
         "ff56ff34002600030106ff06ff56ff3500260003009001ceff640002010000000010"
         "67616d655f323032362e7069636b75702c015802960000019cff0000f401000000f0"
         "00001167616d655f323032362e73686f6f746572ffff000000000102000000000000"
         "0000000000000000000000000000dd000300035a65640417000f0000cf00bf028a02"
         "7800ceff19009cffc8000200000a00ecff1e00004000000201200328005a00ff3300"
         "0101d4fe0000c201000000e0000064006400fa00010203040000fa00fa0056ff3200"
         "2600010006fffa0056ff330026000200fa0006ff56ff34002600030106ff06ff56ff"
         "3500260003009001ceff64000201000000001067616d655f323032362e7069636b75"
         "702c015802960000019cff0000f401000000f000001167616d655f323032362e7368"
         "6f6f746572ffff0000000001020000000000000000000000000000000000000000"},
        {"JoinReject", "010f00041b726f626f742e6472697665747261696e2e776865656c436f756e74"},
        {"Roster",
         "0110000807060504030201090000000000000003de00000004486f73740014000000"
         "01cf00bc028a027800ceff19009cffc8000200000a00ecff1e000040000002012003"
         "28005a00ff33000101d4fe0000c201000000e0000064006400fa00010203040000fa"
         "00fa0056ff32002600010006fffa0056ff330026000200fa0006ff56ff3400260003"
         "0106ff06ff56ff3500260003009001ceff64000201000000001067616d655f323032"
         "362e7069636b75702c015802960000019cff0000f401000000f000001167616d655f"
         "323032362e73686f6f746572ffff0000000001020000000000000000000000000000"
         "000000000000dd00010003416d79011500050000cf00bd028a027800ceff19009cff"
         "c8000200000a00ecff1e00004000000201200328005a00ff33000101d4fe0000c201"
         "000000e0000064006400fa00010203040000fa00fa0056ff32002600010006fffa00"
         "56ff330026000200fa0006ff56ff34002600030106ff06ff56ff3500260003009001"
         "ceff64000201000000001067616d655f323032362e7069636b75702c015802960000"
         "019cff0000f401000000f000001167616d655f323032362e73686f6f746572ffff00"
         "00000001020000000000000000000000000000000000000000dd00020003426f6205"
         "16000a0000cf00be028a027800ceff19009cffc8000200000a00ecff1e0000400000"
         "0201200328005a00ff33000101d4fe0000c201000000e0000064006400fa00010203"
         "040000fa00fa0056ff32002600010006fffa0056ff330026000200fa0006ff56ff34"
         "002600030106ff06ff56ff3500260003009001ceff64000201000000001067616d65"
         "5f323032362e7069636b75702c015802960000019cff0000f401000000f000001167"
         "616d655f323032362e73686f6f746572ffff00000000010200000000000000000000"
         "00000000000000000000"},
        {"Leave", "01110003627965"},
        {"MatchInputPacket",
         "0112008877665544332211070000000000000003440029000000110400000000000004"
         "000048410004000050c000f800000000ff7f0000c84200802e00000002012c016afff4"
         "01004000e00201050080ff7f00000080ff7f000044002a000000120400000000000004"
         "000048410004000050c000f800000000ff7f0000c84200802f00000002012c016afff4"
         "01004000e00201050080ff7f00000080ff7f000044002b000000130400000000000004"
         "000048410004000050c000f800000000ff7f0000c84200803000000002012c016afff4"
         "01004000e00201050080ff7f00000080ff7f0000"},
        {"MatchSnapshot",
         "01130088776655443322110700000000000000e110000006120f000000000030055826"
         "010000006300000002120f0000000000fdff1300020101010000a5420a70686173652e"
         "6175746f0247000000dc05000042f5ffffe80350fb480d300202640038ff2c0170fe09"
         "0000000201000000404102000000003f0200fa00b5ff800234120000ffff0f18fcd007"
         "b80b04000500060047000100b80b000084eaffffd00750fb480d300202640038ff2c01"
         "70fe0a0000000201000000404102000000003f0201f401b5ff800234120000ffff0f18"
         "fcd007b80b0400050006000124000100e0100000020b00000016000000022100000064"
         "0038ff05002c000000d4fe900100000207000000f6420800000080bf"},
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

Message goldenSampleOf(std::string_view name) {
    for (const NamedMessage& s : matchSamples())
        if (name == s.name) return s.message;
    FAIL("no sample named " << name);
    return {};
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

TEST_CASE("match codec golden: records written before the zone pose tail decode with no zone poses") {
    const BinaryMessageCodec codec;
    // MatchInputPacket and MatchSnapshot as the codec wrote them before the zone pose lists existed.
    const std::string legacyInput =
        "0112008877665544332211070000000000000003290029000000110400000000000004"
        "000048410004000050c000f800000000ff7f0000c84200802e0000002900"
        "2a000000120400000000000004000048410004000050c000f800000000ff7f0000c84200"
        "802f00000029002b000000130400000000000004000048410004000050c000f800000000"
        "ff7f0000c842008030000000";
    const std::string legacySnapshot =
        "01130088776655443322110700000000000000e110000006120f0000000000300558"
        "26010000006300000002120f0000000000fdff1300020101010000a5420a70686173"
        "652e6175746f022c000000dc05000042f5ffffe80350fb480d300202640038ff2c01"
        "70fe090000000201000000404102000000003f2c000100b80b000084eaffffd00750"
        "fb480d300202640038ff2c0170fe0a0000000201000000404102000000003f012400"
        "0100e0100000020b000000160000000221000000640038ff05002c000000d4fe9001"
        "00000207000000f6420800000080bf";

    const Result<Message> input = codec.decode(fromHex(legacyInput));
    REQUIRE(input.ok());
    auto expectedInput = std::get<match::MatchInputPacket>(goldenSampleOf("MatchInputPacket"));
    for (match::MatchInput& in : expectedInput.inputs) in.zonePoses.clear();
    CHECK(std::get<match::MatchInputPacket>(input.value()) == expectedInput);

    const Result<Message> snapshot = codec.decode(fromHex(legacySnapshot));
    REQUIRE(snapshot.ok());
    auto expectedSnapshot = std::get<match::MatchSnapshot>(goldenSampleOf("MatchSnapshot"));
    for (match::RobotWireState& r : expectedSnapshot.robots) r.zonePoses.clear();
    CHECK(std::get<match::MatchSnapshot>(snapshot.value()) == expectedSnapshot);
}

TEST_CASE("match codec golden: the zone pose tails are the documented 13-byte entries") {
    // Count byte then, per entry: index u8, x y z yaw pitch roll as little-endian i16 (see zone_pose_wire.h).
    match::MatchInputPacket packet;
    packet.inputs.assign(1, match::MatchInput{});
    packet.inputs[0].zonePoses = {{2, 1, -1, 256, -256, 0x7FFF, -0x8000}};
    std::vector<std::byte> out;
    REQUIRE(BinaryMessageCodec().encode(Message{packet}, out).ok());
    const std::string hex = toHex(out);
    CHECK(hex.ends_with("01" "02" "0100" "ffff" "0001" "00ff" "ff7f" "0080"));

    match::MatchSnapshot snapshot;
    snapshot.robots.assign(1, match::RobotWireState{});
    snapshot.robots[0].zonePoses = {{15, 0x0102, 0, 0, 0, 0, 0x0304}};
    out.clear();
    REQUIRE(BinaryMessageCodec().encode(Message{snapshot}, out).ok());
    CHECK(toHex(out).find("01" "0f" "0201" "0000" "0000" "0000" "0000" "0403") != std::string::npos);
}

TEST_CASE("match codec golden: roster entries written before the appearance tail decode with no appearance") {
    const BinaryMessageCodec codec;
    // Roster and JoinAccept as the codec wrote them before RosterEntry::appearance existed.
    const std::string legacyRoster =
        "01100008070605040302010900000000000000030d00000004486f737400140000"
        "00010c00010003416d790115000500000c00020003426f620516000a0000";
    const std::string legacyAccept =
        "010e000300887766554433221107000000000000000488130000000000001e005a"
        "000000c03f000010c00000003f00000000000000000000803e0000403fa0a1a2a3"
        "a4a5a6a7a8a9aaabacadaeaf01393000000000000048d9ad0300000000020d0000"
        "0004486f73740014000000010c000300035a65640417000f0000";

    const Result<Message> roster = codec.decode(fromHex(legacyRoster));
    REQUIRE(roster.ok());
    auto expectedRoster = std::get<match::Roster>(goldenSampleOf("Roster"));
    for (match::RosterEntry& e : expectedRoster.entries) e.appearance = {};
    CHECK(std::get<match::Roster>(roster.value()) == expectedRoster);

    const Result<Message> accept = codec.decode(fromHex(legacyAccept));
    REQUIRE(accept.ok());
    auto expectedAccept = std::get<match::JoinAccept>(goldenSampleOf("JoinAccept"));
    for (match::RosterEntry& e : expectedAccept.roster) e.appearance = {};
    CHECK(std::get<match::JoinAccept>(accept.value()) == expectedAccept);
}

TEST_CASE("match codec golden: the appearance tail is the documented record") {
    // Hand-verified layout (see robot_appearance.h), all little-endian:
    //   record length u16, chassis size x y z u16, centre x y z i16, COM height u16,
    //   primitives: count, each kind u8, pose (index u8, x y z yaw pitch roll i16), extent x y z u16, r g b,
    //   wheels: count, each kind u8, steered u8, hub x y z i16, radius u16, width u16,
    //   zones: count, each pose (13 bytes), kind (length, bytes), size x y z u16, on-mechanism u8.
    match::RosterEntry entry;
    entry.appearance.sizeXMm = 0x0102;
    entry.appearance.sizeYMm = 0x0304;
    entry.appearance.sizeZMm = 5;
    entry.appearance.centreXMm = -1;
    entry.appearance.centreYMm = 2;
    entry.appearance.centreZMm = -3;
    entry.appearance.comHeightMm = 0x0A0B;
    match::AppearancePrimitive prim;
    prim.kind = PrimitiveKind::Cylinder;
    prim.pose = {0, 1, -1, 256, -256, 0x7FFF, -0x8000};
    prim.extentXMm = 0x10;
    prim.extentYMm = 0x20;
    prim.extentZMm = 0x30;
    prim.r = 0xA1;
    prim.g = 0xB2;
    prim.b = 0xC3;
    entry.appearance.primitives = {prim};
    match::AppearanceWheel wheel;
    wheel.kind = WheelKind::Swerve;
    wheel.steered = true;
    wheel.xMm = 0x0100;
    wheel.yMm = -2;
    wheel.zMm = 3;
    wheel.radiusMm = 0x33;
    wheel.widthMm = 0x26;
    entry.appearance.wheels = {wheel};
    match::AppearanceZone zone;
    zone.pose = {0, 0x0102, 0, 0, 0, 0, 0x0304};
    zone.kind = "ab";
    zone.sizeXMm = 1;
    zone.sizeYMm = 2;
    zone.sizeZMm = 3;
    zone.onMechanism = true;
    entry.appearance.zones = {zone};

    match::Roster roster;
    roster.entries = {entry};
    std::vector<std::byte> out;
    REQUIRE(BinaryMessageCodec().encode(Message{roster}, out).ok());
    const std::string expected =
        "4b00"                                              // record length: 14 + 24 + 13 + 24 = 75
        "0201" "0403" "0500" "ffff" "0200" "fdff" "0b0a"    // chassis box, centre, COM height
        "01" "01" "00" "0100" "ffff" "0001" "00ff" "ff7f" "0080" "1000" "2000" "3000" "a1b2c3"  // 1 primitive
        "01" "03" "01" "0001" "feff" "0300" "3300" "2600"   // 1 wheel
        "01" "00" "0201" "0000" "0000" "0000" "0000" "0403" "02" "6162" "0100" "0200" "0300" "01";  // 1 zone
    CHECK(toHex(out).ends_with(expected));
    const Result<Message> back = BinaryMessageCodec().decode(out);
    REQUIRE(back.ok());
    CHECK(std::get<match::Roster>(back.value()) == roster);
}
