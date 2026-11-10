#include <doctest/doctest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "match_samples.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::proto;
using wpiemu::test_ipc::matchSamples;
using wpiemu::test_ipc::NamedMessage;

// Hostile input for the match and relay messages (MessageType 13..29). They share the Writer/Reader of the HALSim
// protocol, so these tests pin that every list is capped by its contract constant and by the bytes left, every string by
// its contract limit, every enum by its maximum, and that no byte sequence crashes, throws or hangs the decoder.
namespace {

using Bytes = std::vector<std::byte>;

constexpr std::uint16_t kFirstMatchType = 13;
constexpr std::uint16_t kLastMatchType = 29;

class Builder {
public:
    Builder& u8(unsigned v) {
        bytes.push_back(static_cast<std::byte>(v & 0xFFu));
        return *this;
    }
    Builder& u16(unsigned v) { return u8(v).u8(v >> 8); }
    Builder& u32(std::uint32_t v) { return u16(v & 0xFFFFu).u16(v >> 16); }
    Builder& u64(std::uint64_t v) { return u32(static_cast<std::uint32_t>(v)).u32(static_cast<std::uint32_t>(v >> 32)); }
    Builder& varint(std::uint64_t v) {
        while (v >= 0x80) {
            u8(static_cast<unsigned>(v | 0x80));
            v >>= 7;
        }
        return u8(static_cast<unsigned>(v));
    }
    Builder& zeros(std::size_t n) {
        bytes.insert(bytes.end(), n, std::byte{0});
        return *this;
    }
    Builder& str(const std::string& s) {
        varint(s.size());
        for (const char c : s) u8(static_cast<unsigned char>(c));
        return *this;
    }
    Builder& add(const Bytes& more) {
        bytes.insert(bytes.end(), more.begin(), more.end());
        return *this;
    }
    // A u16-length-prefixed record around `body`.
    Builder& record(const Bytes& body) { return u16(static_cast<unsigned>(body.size())).add(body); }
    Bytes bytes;
};

Bytes frameOf(std::uint16_t type, const Bytes& body) {
    Builder b;
    b.u8(kWireFormatVersion).u16(type).add(body);
    return b.bytes;
}

Bytes frameOf(MessageType type, const Bytes& body) { return frameOf(static_cast<std::uint16_t>(type), body); }

Bytes encodeOk(const Message& m) {
    const BinaryMessageCodec codec;
    Bytes out;
    const Status status = codec.encode(m, out);
    REQUIRE_MESSAGE(status.ok(), (status.ok() ? "" : status.error().message));
    return out;
}

// A decode must never throw; anything else is the caller's to judge.
Result<Message> decodeNoThrow(const Bytes& frame, const BinaryMessageCodec& codec = BinaryMessageCodec()) {
    try {
        return codec.decode(frame);
    } catch (...) {
        FAIL("the decoder threw");
    }
    return makeError(ErrorCode::Internal, "unreachable");
}

bool isProtocolError(const Result<Message>& r) { return !r.ok() && r.error().code == ErrorCode::ProtocolError; }

bool messageContains(const Result<Message>& r, const char* text) {
    return !r.ok() && r.error().message.find(text) != std::string::npos;
}

void checkRejected(const Bytes& frame, const char* text) {
    const Result<Message> r = decodeNoThrow(frame);
    CHECK(isProtocolError(r));
    CHECK_MESSAGE(messageContains(r, text), (r.ok() ? "decoded" : r.error().message));
}

// What a decoded message must satisfy whatever bytes it came from: lists within their caps, strings within their limits,
// and a canonical re-encoding (encode, decode and encode again is stable).
void checkDecodedInvariants(const Message& m) {
    using namespace wpiemu::sdk::match;
    std::visit(
        [](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, Join>) {
                CHECK(v.displayName.size() <= kMaxNameBytes);
                CHECK(v.gamePluginId.size() <= kMaxNameBytes);
                CHECK(v.robotSpecJson.size() <= kMaxRobotSpecBytes);
            } else if constexpr (std::is_same_v<T, JoinAccept>) {
                CHECK(v.roster.size() <= kMaxRobotsPerMatch);
            } else if constexpr (std::is_same_v<T, Roster>) {
                CHECK(v.entries.size() <= kMaxRobotsPerMatch);
            } else if constexpr (std::is_same_v<T, JoinReject> || std::is_same_v<T, Leave> ||
                                 std::is_same_v<T, MatchControl>) {
                CHECK(v.detail.size() <= kMaxDetailBytes);
            } else if constexpr (std::is_same_v<T, MatchInputPacket>) {
                CHECK(fitsLimits(v));
            } else if constexpr (std::is_same_v<T, MatchSnapshot>) {
                CHECK(fitsLimits(v));
            } else if constexpr (std::is_same_v<T, relay::RegisterRoom> || std::is_same_v<T, relay::JoinRoom>) {
                CHECK(relay::isValidRoomId(v.roomId));
            }
        },
        m);
    const BinaryMessageCodec codec;
    Bytes once;
    REQUIRE(codec.encode(m, once).ok());
    const Result<Message> again = codec.decode(once);
    REQUIRE(again.ok());
    Bytes twice;
    REQUIRE(codec.encode(again.value(), twice).ok());
    CHECK(once == twice);
}

Bytes emptySnapshotHead() {
    Bytes b = encodeOk(match::MatchSnapshot{});
    REQUIRE(b.size() > 6);
    b.resize(b.size() - 3);  // robots, entityDeltas and gameHud counts (all zero)
    return b;
}

Bytes robotBody(std::size_t wheels, std::size_t hud, std::size_t wheelPad, std::size_t hudPad) {
    Builder b;
    b.u16(1).u32(0).u32(0).u16(0).u16(0).u16(0).u16(0);
    b.varint(wheels).zeros(wheelPad);
    b.u32(0);
    b.varint(hud).zeros(hudPad);
    return b.bytes;
}

std::string repeat(char c, std::size_t n) { return std::string(n, c); }

}  // namespace

TEST_CASE("match hostile: every strict prefix of every message is a ProtocolError") {
    for (const NamedMessage& s : matchSamples()) {
        CAPTURE(s.name);
        const Bytes full = encodeOk(s.message);
        for (std::size_t n = 0; n < full.size(); ++n) {
            const Bytes prefix(full.begin(), full.begin() + static_cast<std::ptrdiff_t>(n));
            const Result<Message> r = decodeNoThrow(prefix);
            CAPTURE(n);
            CHECK(isProtocolError(r));
        }
    }
}

TEST_CASE("match hostile: every single-byte mutation decodes to an error or to a valid, canonical message") {
    for (const NamedMessage& s : matchSamples()) {
        CAPTURE(s.name);
        const Bytes original = encodeOk(s.message);
        for (std::size_t at = 0; at < original.size(); ++at) {
            for (unsigned value = 0; value < 256; ++value) {
                if (static_cast<std::byte>(value) == original[at]) continue;
                Bytes mutated = original;
                mutated[at] = static_cast<std::byte>(value);
                const Result<Message> r = decodeNoThrow(mutated);
                if (r.ok()) {
                    // Only the mutations that survive are worth the (slower) invariant checks.
                    if (!isMatchMessage(r.value())) continue;
                    checkDecodedInvariants(r.value());
                } else {
                    CHECK_FALSE(r.error().message.empty());
                }
            }
        }
    }
}

TEST_CASE("match hostile: an empty body is a ProtocolError for every match and relay type with fields") {
    for (std::uint16_t type = kFirstMatchType; type <= kLastMatchType; ++type) {
        CAPTURE(type);
        const bool hasFields = type != static_cast<std::uint16_t>(MessageType::CloseRoom);
        CHECK(isProtocolError(decodeNoThrow(frameOf(type, {}))) == hasFields);
    }
}

TEST_CASE("match hostile: CloseRoom has no fields and is exactly its header") {
    const Bytes encoded = encodeOk(match::relay::CloseRoom{});
    CHECK(encoded == frameOf(MessageType::CloseRoom, {}));
    const Result<Message> decoded = decodeNoThrow(encoded);
    REQUIRE(decoded.ok());
    CHECK(std::holds_alternative<match::relay::CloseRoom>(decoded.value()));
}

TEST_CASE("match hostile: list counts above the cap are refused (with enough bytes behind them)") {
    using namespace wpiemu::sdk::match;
    const Builder ids = Builder().u64(1).u64(1);
    // Roster
    checkRejected(frameOf(MessageType::Roster, Builder().add(ids.bytes).varint(kMaxRobotsPerMatch + 1).zeros(7 * 11).bytes),
                  "limit");
    checkRejected(frameOf(MessageType::Roster, Builder().add(ids.bytes).varint(std::uint64_t{1} << 40).zeros(64).bytes), "limit");
    // JoinAccept roster
    Bytes accept = encodeOk(JoinAccept{});
    accept.pop_back();
    {
        Builder b;
        b.add(accept).varint(kMaxRobotsPerMatch + 1).zeros(7 * 11);
        checkRejected(b.bytes, "limit");
    }
    // MatchInputPacket inputs, and wheels inside one input
    checkRejected(frameOf(MessageType::MatchInputPacket,
                          Builder().add(ids.bytes).varint(kInputRedundancy + 1).zeros(4 * 19).bytes),
                  "limit");
    {
        Builder input;
        input.u32(1).u64(1).varint(kMaxWheelsPerRobot + 1).zeros((kMaxWheelsPerRobot + 1) * 6).u32(0);
        checkRejected(frameOf(MessageType::MatchInputPacket,
                              Builder().add(ids.bytes).varint(1).record(input.bytes).bytes),
                      "limit");
    }
    // MatchSnapshot: robots, entityDeltas, gameHud, wheels and hud inside a robot, removed and upserted inside a delta
    const Bytes head = emptySnapshotHead();
    checkRejected(Builder().add(head).varint(kMaxRobotsPerMatch + 1).zeros(7 * 26).bytes, "limit");
    checkRejected(Builder().add(head).varint(0).varint(kMaxEntityGroups + 1).zeros(9 * 10).bytes, "limit");
    checkRejected(Builder().add(head).varint(0).varint(0).varint(kMaxGameHudValues + 1).zeros(17 * 6).bytes, "limit");
    checkRejected(Builder()
                      .add(head)
                      .varint(1)
                      .record(robotBody(kMaxWheelsPerRobot + 1, 0, (kMaxWheelsPerRobot + 1) * 4, 0))
                      .bytes,
                  "limit");
    checkRejected(Builder()
                      .add(head)
                      .varint(1)
                      .record(robotBody(0, kMaxHudValuesPerRobot + 1, 0, (kMaxHudValuesPerRobot + 1) * 6))
                      .bytes,
                  "limit");
    {
        Builder removed;
        removed.u16(1).u32(0).varint(kMaxEntityDeltaEntries + 1).zeros((kMaxEntityDeltaEntries + 1) * 4).varint(0);
        checkRejected(Builder().add(head).varint(0).varint(1).record(removed.bytes).bytes, "limit");
        Builder upserted;
        upserted.u16(1).u32(0).varint(0).varint(kMaxEntityDeltaEntries + 1).zeros((kMaxEntityDeltaEntries + 1) * 10);
        checkRejected(Builder().add(head).varint(0).varint(1).record(upserted.bytes).bytes, "limit");
    }
}

TEST_CASE("match hostile: list counts the frame cannot hold are refused without allocating them") {
    using namespace wpiemu::sdk::match;
    const Builder ids = Builder().u64(1).u64(1);
    checkRejected(frameOf(MessageType::Roster, Builder().add(ids.bytes).varint(kMaxRobotsPerMatch).zeros(10).bytes), "frame");
    checkRejected(frameOf(MessageType::MatchInputPacket, Builder().add(ids.bytes).varint(kInputRedundancy).zeros(18).bytes),
                  "frame");
    const Bytes head = emptySnapshotHead();
    checkRejected(Builder().add(head).varint(kMaxRobotsPerMatch).zeros(20).bytes, "frame");
    checkRejected(Builder().add(head).varint(0).varint(kMaxEntityGroups).zeros(9).bytes, "frame");
    checkRejected(Builder().add(head).varint(0).varint(0).varint(kMaxGameHudValues).zeros(10).bytes, "frame");
    {
        Builder removed;
        removed.u16(1).u32(0).varint(1000).zeros(10);
        checkRejected(Builder().add(head).varint(0).varint(1).record(removed.bytes).bytes, "frame");
        Builder upserted;
        upserted.u16(1).u32(0).varint(0).varint(1000).zeros(10);
        checkRejected(Builder().add(head).varint(0).varint(1).record(upserted.bytes).bytes, "frame");
    }
    {
        Builder input;
        input.u32(1).u64(1).varint(kMaxWheelsPerRobot).zeros(5);
        checkRejected(frameOf(MessageType::MatchInputPacket, Builder().add(ids.bytes).varint(1).record(input.bytes).bytes),
                      "frame");
    }
    // A varint that never ends, or overflows 64 bits, is malformed before it is a count.
    checkRejected(frameOf(MessageType::Roster, Builder().add(ids.bytes).zeros(0).u8(0xFF).u8(0xFF).u8(0xFF).bytes), "varint");
    Builder overflow;
    overflow.add(ids.bytes);
    for (int i = 0; i < 9; ++i) overflow.u8(0xFF);
    overflow.u8(0x7F);
    checkRejected(frameOf(MessageType::Roster, overflow.bytes), "varint");
    // A record whose length claims more than the frame holds.
    checkRejected(frameOf(MessageType::Roster, Builder().add(ids.bytes).varint(1).u16(5000).zeros(16).bytes), "record");
}

TEST_CASE("match hostile: strings above their contract limit are refused, at the limit they decode") {
    using namespace wpiemu::sdk::match;
    const auto joinBody = [](const std::string& name, const std::string& plugin, const std::string& spec) {
        return frameOf(MessageType::Join, Builder().u32(1).str(name).str(plugin).u32(2026).zeros(kResumeTokenBytes).str(spec).bytes);
    };
    const BinaryMessageCodec codec;
    CHECK(decodeNoThrow(joinBody(repeat('a', kMaxNameBytes), repeat('b', kMaxNameBytes), repeat('c', kMaxRobotSpecBytes))).ok());
    checkRejected(joinBody(repeat('a', kMaxNameBytes + 1), "p", ""), "display name");
    checkRejected(joinBody("n", repeat('b', kMaxNameBytes + 1), ""), "plugin id");
    checkRejected(joinBody("n", "p", repeat('c', kMaxRobotSpecBytes + 1)), "robot spec");

    const auto detailFrame = [](MessageType type, const std::string& detail) {
        switch (type) {
            case MessageType::JoinReject: return frameOf(type, Builder().u8(0).str(detail).bytes);
            case MessageType::Leave: return frameOf(type, Builder().str(detail).bytes);
            default:
                return frameOf(type, Builder().u8(0).u64(0).u64(0).u16(0).u32(0).u64(0).u64(0).u64(0).str(detail).bytes);
        }
    };
    for (const MessageType type : {MessageType::JoinReject, MessageType::Leave, MessageType::MatchControl}) {
        CAPTURE(static_cast<int>(type));
        CHECK(decodeNoThrow(detailFrame(type, repeat('d', kMaxDetailBytes))).ok());
        checkRejected(detailFrame(type, repeat('d', kMaxDetailBytes + 1)), "detail");
    }

    {
        Builder entry;
        entry.u16(1).str(repeat('n', kMaxNameBytes + 1)).u8(0).u16(0).u16(0).u8(0);
        checkRejected(frameOf(MessageType::Roster, Builder().u64(1).u64(1).varint(1).record(entry.bytes).bytes), "roster name");
        MatchSnapshot s;
        s.info.phaseKey = repeat('k', kMaxNameBytes + 1);
        const Bytes body = emptySnapshotHead();
        // The info record starts after the 53 fixed bytes of the snapshot header.
        Builder info;
        info.u8(0).u8(0).u8(0).u8(0).u32(0).str(s.info.phaseKey);
        Builder out;
        out.add(Bytes(body.begin(), body.begin() + 53)).record(info.bytes).varint(0).varint(0).varint(0);
        checkRejected(out.bytes, "phase key");
    }

    // A declared length beyond the bytes left, or beyond the shared 1 MiB string cap.
    checkRejected(frameOf(MessageType::Leave, Builder().varint(1000).zeros(3).bytes), "string");
    checkRejected(frameOf(MessageType::Leave, Builder().varint((std::size_t{1} << 20) + 1).zeros(64).bytes), "string");
    checkRejected(frameOf(MessageType::Leave, Builder().varint(std::uint64_t{1} << 62).bytes), "string");
}

TEST_CASE("match hostile: every enum is bounded by its maximum") {
    struct Case {
        const char* message;
        std::size_t offset;
        std::uint8_t max;
    };
    const Case cases[] = {
        {"JoinAccept", 21, 5},      // station
        {"JoinAccept", 78, 3},      // state
        {"JoinAccept", 105, 5},     // roster entry station
        {"JoinReject", 3, 9},       // reason
        {"Roster", 29, 5},          // entry station
        {"MatchSnapshot", 55, 3},   // info.state
        {"MatchSnapshot", 57, 2},   // info.mode
        {"MatchControl", 3, 6},     // kind
        {"RoomRejected", 3, 3},     // reason
        {"JoinResult", 4, 5},       // reason
        {"PeerLeft", 5, 3},         // reason
    };
    const std::vector<NamedMessage> samples = matchSamples();
    for (const Case& c : cases) {
        CAPTURE(c.message);
        CAPTURE(c.offset);
        Bytes frame;
        for (const NamedMessage& s : samples)
            if (std::string(s.name) == c.message) frame = encodeOk(s.message);
        REQUIRE(c.offset < frame.size());
        frame[c.offset] = static_cast<std::byte>(c.max);
        CHECK(decodeNoThrow(frame).ok());
        frame[c.offset] = static_cast<std::byte>(c.max + 1);
        checkRejected(frame, "invalid");
        frame[c.offset] = static_cast<std::byte>(0xFF);
        checkRejected(frame, "invalid");
    }
}

TEST_CASE("match hostile: a fixed-size array is read in full, so a short one is truncated") {
    using namespace wpiemu::sdk::match;
    const Bytes head = Builder().u32(1).str("n").str("p").u32(2026).bytes;
    CHECK(decodeNoThrow(frameOf(MessageType::Join, Builder().add(head).zeros(kResumeTokenBytes).varint(0).bytes)).ok());
    checkRejected(frameOf(MessageType::Join, Builder().add(head).zeros(kResumeTokenBytes - 1).varint(0).bytes), "truncated");
    checkRejected(frameOf(MessageType::Join, Builder().add(head).zeros(kResumeTokenBytes - 1).bytes), "truncated");

    const std::string id = "7K3QZ9XM";
    CHECK(decodeNoThrow(frameOf(MessageType::RegisterRoom, Builder().u32(1).str(id).zeros(relay::kRelayHostAuthBytes).bytes)).ok());
    checkRejected(frameOf(MessageType::RegisterRoom, Builder().u32(1).str(id).zeros(relay::kRelayHostAuthBytes - 1).bytes),
                  "truncated");
    checkRejected(frameOf(MessageType::RegisterRoom, Builder().u32(1).str(id).bytes), "truncated");
}

TEST_CASE("match hostile: room ids outside the alphabet or of the wrong length are refused") {
    const std::vector<std::string> bad = {"",         "7K3QZ9X",     "7K3QZ9XMM",  "7k3qz9xm",
                                          "7K3QZ9XI", "7K3QZ9XL",    "7K3QZ9XO",   "7K3QZ9XU",
                                          "7K3QZ9X ", "7K3QZ9X-",    std::string("7K3QZ9X\0", 8),
                                          "7K3QZ9X\xC3", "\xE2\x82\xAC" "ABCDE"};
    const std::vector<std::string> good = {"00000000", "ZZZZZZZZ", "0123ABCD", "89JKMNPQ", "RSTVWXYZ"};
    for (const std::string& id : bad) {
        CAPTURE(id);
        checkRejected(frameOf(MessageType::JoinRoom, Builder().u32(1).str(id).bytes), "room id");
        checkRejected(frameOf(MessageType::RegisterRoom, Builder().u32(1).str(id).zeros(32).bytes), "room id");
    }
    for (const std::string& id : good) {
        CAPTURE(id);
        const Result<Message> join = decodeNoThrow(frameOf(MessageType::JoinRoom, Builder().u32(1).str(id).bytes));
        REQUIRE(join.ok());
        CHECK(std::get<match::relay::JoinRoom>(join.value()).roomId == id);
        CHECK(decodeNoThrow(frameOf(MessageType::RegisterRoom, Builder().u32(1).str(id).zeros(32).bytes)).ok());
    }
}

TEST_CASE("match hostile: empty, tiny, oversized and garbage frames never crash") {
    const BinaryMessageCodec codec;
    CHECK(isProtocolError(decodeNoThrow({}, codec)));
    for (std::size_t n = 1; n < 3; ++n) CHECK(isProtocolError(decodeNoThrow(Bytes(n, std::byte{1}), codec)));

    std::mt19937 rng(0x5EED);
    Bytes garbage(std::size_t{1} << 20);
    for (std::byte& b : garbage) b = static_cast<std::byte>(rng() & 0xFFu);
    for (std::uint16_t type = kFirstMatchType; type <= kLastMatchType; ++type) {
        CAPTURE(type);
        for (const std::byte fill : {std::byte{0x00}, std::byte{0xFF}, std::byte{0x80}}) {
            const Bytes body(std::size_t{1} << 20, fill);
            const Result<Message> r = decodeNoThrow(frameOf(type, body), codec);
            if (r.ok()) checkDecodedInvariants(r.value());
        }
        const Result<Message> random = decodeNoThrow(frameOf(type, garbage), codec);
        if (random.ok()) checkDecodedInvariants(random.value());
    }
    // Garbage including the header.
    for (int i = 0; i < 64; ++i) {
        Bytes frame(1 + rng() % 4096);
        for (std::byte& b : frame) b = static_cast<std::byte>(rng() & 0xFFu);
        frame[0] = static_cast<std::byte>(kWireFormatVersion);
        (void)decodeNoThrow(frame, codec);
    }
    // A codec with a small frame cap refuses a longer frame before parsing it.
    const BinaryMessageCodec small(1024);
    CHECK(isProtocolError(decodeNoThrow(frameOf(MessageType::Leave, Bytes(2000, std::byte{0})), small)));
}

TEST_CASE("match hostile: random bodies of every match type are an error or a valid message") {
    std::mt19937 rng(20260);
    for (std::uint16_t type = kFirstMatchType; type <= kLastMatchType; ++type) {
        CAPTURE(type);
        for (int i = 0; i < 3000; ++i) {
            Bytes body(rng() % 400);
            for (std::byte& b : body) b = static_cast<std::byte>(rng() % 4 == 0 ? rng() & 0xFFu : (rng() % 3));
            const Result<Message> r = decodeNoThrow(frameOf(type, body));
            if (r.ok()) {
                REQUIRE(isMatchMessage(r.value()));
                checkDecodedInvariants(r.value());
            }
        }
    }
}

TEST_CASE("match hostile: unknown message types 30 and above are skippable like any future type") {
    const BinaryMessageCodec codec;
    for (const std::uint16_t type : {std::uint16_t{0}, std::uint16_t{30}, std::uint16_t{31}, std::uint16_t{255},
                                     std::uint16_t{256}, std::uint16_t{0x7FFF}, std::uint16_t{0xFFFF}}) {
        CAPTURE(type);
        const Result<Message> r = decodeNoThrow(frameOf(type, Builder().u32(1234).bytes), codec);
        REQUIRE_FALSE(r.ok());
        CHECK(isSkippableDecodeError(r.error()));
    }

    auto pair = testing::makeInMemoryTransportPair();
    const auto shared = std::make_shared<BinaryMessageCodec>();
    MessageChannel sender(std::move(pair.a), shared);
    MessageChannel receiver(std::move(pair.b), shared);
    REQUIRE(sender.transport().send(frameOf(30, Builder().u8(1).bytes)).ok());
    REQUIRE(sender.transport().send(frameOf(500, {})).ok());
    match::relay::PeerLeft left;
    left.peerId = 9;
    left.reason = match::relay::PeerLeftReason::TimedOut;
    REQUIRE(sender.send(left).ok());
    Message got;
    std::size_t skipped = 0;
    const Result<bool> r = receiveSkippingUnknown(receiver, got, std::chrono::milliseconds(200), &skipped);
    REQUIRE(r.ok());
    REQUIRE(r.value());
    CHECK(skipped == 2);
    CHECK(std::get<match::relay::PeerLeft>(got) == left);
}

TEST_CASE("match hostile: messages beyond a wire limit are refused on encode and leave the buffer untouched") {
    using namespace wpiemu::sdk::match;
    const BinaryMessageCodec codec;
    const auto rejected = [&](const Message& m, const char* what) {
        CAPTURE(what);
        Bytes out = {std::byte{1}, std::byte{2}, std::byte{3}};
        const Status status = codec.encode(m, out);
        CHECK_FALSE(status.ok());
        if (!status.ok()) CHECK(status.error().code == ErrorCode::InvalidArgument);
        CHECK(out == Bytes{std::byte{1}, std::byte{2}, std::byte{3}});
    };
    const auto joinWith = [](auto&& edit) {
        Join j;
        edit(j);
        return Message{j};
    };
    rejected(joinWith([](Join& j) { j.displayName = repeat('a', kMaxNameBytes + 1); }), "Join.displayName");
    rejected(joinWith([](Join& j) { j.gamePluginId = repeat('a', kMaxNameBytes + 1); }), "Join.gamePluginId");
    rejected(joinWith([](Join& j) { j.robotSpecJson = repeat('a', kMaxRobotSpecBytes + 1); }), "Join.robotSpecJson");
    JoinReject reject;
    reject.detail = repeat('d', kMaxDetailBytes + 1);
    rejected(reject, "JoinReject.detail");
    Leave leave;
    leave.detail = repeat('d', kMaxDetailBytes + 1);
    rejected(leave, "Leave.detail");
    MatchControl control;
    control.detail = repeat('d', kMaxDetailBytes + 1);
    rejected(control, "MatchControl.detail");

    RosterEntry entry;
    entry.name = repeat('n', kMaxNameBytes + 1);
    Roster nameRoster;
    nameRoster.entries = {entry};
    rejected(nameRoster, "RosterEntry.name");
    Roster tooMany;
    tooMany.entries.assign(kMaxRobotsPerMatch + 1, RosterEntry{});
    rejected(tooMany, "Roster.entries");
    JoinAccept accept;
    accept.roster.assign(kMaxRobotsPerMatch + 1, RosterEntry{});
    rejected(accept, "JoinAccept.roster");

    MatchInputPacket packet;
    packet.inputs.assign(kInputRedundancy + 1, MatchInput{});
    rejected(packet, "MatchInputPacket.inputs");
    MatchInputPacket wheels;
    wheels.inputs.assign(1, MatchInput{});
    wheels.inputs[0].wheels.assign(kMaxWheelsPerRobot + 1, WheelInput{});
    rejected(wheels, "MatchInput.wheels");

    const auto snapshotWith = [](auto&& edit) {
        MatchSnapshot s;
        edit(s);
        return Message{s};
    };
    rejected(snapshotWith([](MatchSnapshot& s) { s.info.phaseKey = repeat('k', kMaxNameBytes + 1); }), "MatchInfo.phaseKey");
    rejected(snapshotWith([](MatchSnapshot& s) { s.robots.assign(kMaxRobotsPerMatch + 1, RobotWireState{}); }), "robots");
    rejected(snapshotWith([](MatchSnapshot& s) { s.entityDeltas.assign(kMaxEntityGroups + 1, EntityDelta{}); }), "entityDeltas");
    rejected(snapshotWith([](MatchSnapshot& s) { s.gameHud.assign(kMaxGameHudValues + 1, HudValue{}); }), "gameHud");
    rejected(snapshotWith([](MatchSnapshot& s) {
                 s.robots.assign(1, RobotWireState{});
                 s.robots[0].wheels.assign(kMaxWheelsPerRobot + 1, WheelWireState{});
             }),
             "robot wheels");
    rejected(snapshotWith([](MatchSnapshot& s) {
                 s.robots.assign(1, RobotWireState{});
                 s.robots[0].hud.assign(kMaxHudValuesPerRobot + 1, HudValue{});
             }),
             "robot hud");
    rejected(snapshotWith([](MatchSnapshot& s) {
                 s.entityDeltas.assign(1, EntityDelta{});
                 s.entityDeltas[0].removed.assign(kMaxEntityDeltaEntries + 1, 1u);
             }),
             "delta removed");
    rejected(snapshotWith([](MatchSnapshot& s) {
                 s.entityDeltas.assign(1, EntityDelta{});
                 s.entityDeltas[0].upserted.assign(kMaxEntityDeltaEntries + 1, EntityState{});
             }),
             "delta upserted");

    for (const char* id : {"", "7K3QZ9X", "7k3qz9xm", "7K3QZ9XI", "7K3QZ9XMM"}) {
        relay::RegisterRoom reg;
        reg.roomId = id;
        rejected(reg, "RegisterRoom.roomId");
        relay::JoinRoom join;
        join.roomId = id;
        rejected(join, "JoinRoom.roomId");
    }
    // The default-constructed messages carry no room id and are rejected too: a room id is required content.
    rejected(relay::RegisterRoom{}, "RegisterRoom default");
}

namespace {

Bytes poseBytes(unsigned index) {
    Builder b;
    b.u8(index).u16(100).u16(0xFF9C).u16(500).u16(0x4000).u16(0).u16(0xFFFF);
    return b.bytes;
}

// A MatchInputPacket frame with one input whose record is `body` (seq, tick, no wheels, flags, then `tail`).
Bytes packetWithInputTail(const Bytes& tail) {
    Builder input;
    input.u32(1).u64(1).varint(0).u32(0).add(tail);
    return frameOf(MessageType::MatchInputPacket, Builder().u64(1).u64(1).varint(1).record(input.bytes).bytes);
}

// A snapshot frame with one robot whose record is a robot without wheels and HUD values followed by `tail`.
Bytes snapshotWithRobotTail(const Bytes& tail) {
    Builder robot;
    robot.add(robotBody(0, 0, 0, 0)).add(tail);
    return Builder().add(emptySnapshotHead()).varint(1).record(robot.bytes).varint(0).varint(0).bytes;
}

Bytes poseList(const std::vector<unsigned>& indices) {
    Builder b;
    b.varint(indices.size());
    for (const unsigned i : indices) b.add(poseBytes(i));
    return b.bytes;
}

}  // namespace

TEST_CASE("match hostile: zone pose lists of an input are bounded, ordered and optional") {
    using namespace wpiemu::sdk::match;
    // Absent (an older writer) and empty both mean no overrides.
    for (const Bytes& tail : {Bytes{}, Bytes{std::byte{0}}}) {
        const Result<Message> r = decodeNoThrow(packetWithInputTail(tail));
        REQUIRE(r.ok());
        CHECK(std::get<MatchInputPacket>(r.value()).inputs.at(0).zonePoses.empty());
    }
    // The full list decodes.
    std::vector<unsigned> all;
    for (unsigned i = 0; i < kMaxZonesPerRobot; ++i) all.push_back(i);
    const Result<Message> full = decodeNoThrow(packetWithInputTail(poseList(all)));
    REQUIRE(full.ok());
    CHECK(std::get<MatchInputPacket>(full.value()).inputs.at(0).zonePoses.size() == kMaxZonesPerRobot);

    // More than the cap, with all the bytes behind the count.
    all.push_back(kMaxZonesPerRobot);
    checkRejected(packetWithInputTail(poseList(all)), "limit");
    checkRejected(packetWithInputTail(Builder().varint(std::uint64_t{1} << 40).zeros(200).bytes), "limit");
    // A count the bytes left cannot hold.
    checkRejected(packetWithInputTail(Builder().varint(3).add(poseBytes(0)).bytes), "frame");
    // Order and range.
    checkRejected(packetWithInputTail(poseList({2, 1})), "zone poses");
    checkRejected(packetWithInputTail(poseList({3, 3})), "zone poses");
    checkRejected(packetWithInputTail(poseList({0, 5, 4})), "zone poses");
    checkRejected(packetWithInputTail(poseList({static_cast<unsigned>(kMaxZonesPerRobot)})), "zone poses");
    checkRejected(packetWithInputTail(poseList({1, 255})), "zone poses");
    // A truncated pose.
    Bytes cut = poseList({0});
    cut.pop_back();
    checkRejected(packetWithInputTail(cut), "frame");
}

TEST_CASE("match hostile: zone pose lists of a robot snapshot are bounded by the snapshot cap") {
    using namespace wpiemu::sdk::match;
    for (const Bytes& tail : {Bytes{}, Bytes{std::byte{0}}}) {
        const Result<Message> r = decodeNoThrow(snapshotWithRobotTail(tail));
        REQUIRE(r.ok());
        CHECK(std::get<MatchSnapshot>(r.value()).robots.at(0).zonePoses.empty());
    }
    const Result<Message> four = decodeNoThrow(snapshotWithRobotTail(poseList({0, 3, 7, 15})));
    REQUIRE(four.ok());
    CHECK(std::get<MatchSnapshot>(four.value()).robots.at(0).zonePoses.size() == kMaxSnapshotZonePosesPerRobot);
    // Above the snapshot cap (though within the input cap), with every byte present.
    checkRejected(snapshotWithRobotTail(poseList({0, 1, 2, 3, 4})), "limit");
    checkRejected(snapshotWithRobotTail(Builder().varint(2).add(poseBytes(0)).bytes), "frame");
    checkRejected(snapshotWithRobotTail(poseList({4, 4})), "zone poses");
    checkRejected(snapshotWithRobotTail(poseList({9, 2})), "zone poses");
    checkRejected(snapshotWithRobotTail(poseList({16})), "zone poses");
}

TEST_CASE("match hostile: zone pose lists beyond their contract are refused on encode") {
    using namespace wpiemu::sdk::match;
    const BinaryMessageCodec codec;
    const auto rejected = [&](const Message& m, const char* what) {
        CAPTURE(what);
        Bytes out = {std::byte{9}};
        const Status status = codec.encode(m, out);
        CHECK_FALSE(status.ok());
        CHECK(out == Bytes{std::byte{9}});
    };
    const auto inputWith = [](std::vector<ZonePoseInput> poses) {
        MatchInputPacket p;
        p.inputs.assign(1, MatchInput{});
        p.inputs[0].zonePoses = std::move(poses);
        return Message{p};
    };
    const auto snapshotWith = [](std::vector<ZonePoseInput> poses) {
        MatchSnapshot s;
        s.robots.assign(1, RobotWireState{});
        s.robots[0].zonePoses = std::move(poses);
        return Message{s};
    };
    std::vector<ZonePoseInput> tooMany;
    for (std::size_t i = 0; i <= kMaxZonesPerRobot; ++i) tooMany.push_back({static_cast<std::uint8_t>(i), 0, 0, 0, 0, 0, 0});
    rejected(inputWith(tooMany), "input list above the cap");
    rejected(inputWith({{3, 0, 0, 0, 0, 0, 0}, {3, 0, 0, 0, 0, 0, 0}}), "input duplicate index");
    rejected(inputWith({{4, 0, 0, 0, 0, 0, 0}, {2, 0, 0, 0, 0, 0, 0}}), "input decreasing index");
    rejected(inputWith({{static_cast<std::uint8_t>(kMaxZonesPerRobot), 0, 0, 0, 0, 0, 0}}), "input index at the cap");
    tooMany.resize(kMaxSnapshotZonePosesPerRobot + 1);
    rejected(snapshotWith(tooMany), "snapshot list above the cap");
    rejected(snapshotWith({{5, 0, 0, 0, 0, 0, 0}, {5, 0, 0, 0, 0, 0, 0}}), "snapshot duplicate index");
    rejected(snapshotWith({{200, 0, 0, 0, 0, 0, 0}}), "snapshot index out of range");
    CHECK_FALSE(fitsLimits(std::get<MatchSnapshot>(snapshotWith(tooMany))));
}
