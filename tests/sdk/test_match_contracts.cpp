#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "wpiemu/sdk/ipc/delivery.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/match/input_messages.h"
#include "wpiemu/sdk/match/join_messages.h"
#include "wpiemu/sdk/match/match_control.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/quantize.h"
#include "wpiemu/sdk/match/relay_messages.h"
#include "wpiemu/sdk/match/snapshot_messages.h"

using namespace wpiemu::sdk;
using namespace wpiemu::sdk::match;
using namespace wpiemu::sdk::proto;

namespace {

constexpr double kEps = 1e-6;  // float rounding slack on top of the documented bounds

constexpr std::size_t altIndex(MessageType t) { return static_cast<std::size_t>(t) - 1; }

// The variant alternative at the position a MessageType value names must be exactly its message struct.
template <class T, MessageType Type>
constexpr bool alternativeIs = std::is_same_v<std::variant_alternative_t<altIndex(Type), Message>, T>;

static_assert(alternativeIs<Hello, MessageType::Hello>);
static_assert(alternativeIs<VisionSetup, MessageType::VisionSetup>);
static_assert(alternativeIs<match::Join, MessageType::Join>);
static_assert(alternativeIs<match::JoinAccept, MessageType::JoinAccept>);
static_assert(alternativeIs<match::JoinReject, MessageType::JoinReject>);
static_assert(alternativeIs<match::Roster, MessageType::Roster>);
static_assert(alternativeIs<match::Leave, MessageType::Leave>);
static_assert(alternativeIs<match::MatchInputPacket, MessageType::MatchInputPacket>);
static_assert(alternativeIs<match::MatchSnapshot, MessageType::MatchSnapshot>);
static_assert(alternativeIs<match::MatchSnapshotAck, MessageType::MatchSnapshotAck>);
static_assert(alternativeIs<match::MatchControl, MessageType::MatchControl>);
static_assert(alternativeIs<match::relay::RegisterRoom, MessageType::RegisterRoom>);
static_assert(alternativeIs<match::relay::RoomRegistered, MessageType::RoomRegistered>);
static_assert(alternativeIs<match::relay::RoomRejected, MessageType::RoomRejected>);
static_assert(alternativeIs<match::relay::JoinRoom, MessageType::JoinRoom>);
static_assert(alternativeIs<match::relay::JoinResult, MessageType::JoinResult>);
static_assert(alternativeIs<match::relay::PeerJoined, MessageType::PeerJoined>);
static_assert(alternativeIs<match::relay::PeerLeft, MessageType::PeerLeft>);

template <class T>
void checkAlternative(MessageType expected, std::uint16_t value) {
    const Message m{T{}};
    CHECK(typeOf(m) == expected);
    CHECK(static_cast<std::uint16_t>(typeOf(m)) == value);
    CHECK(m.index() + 1 == value);
}

template <std::size_t... I>
std::vector<Message> oneOfEach(std::index_sequence<I...>) {
    return {Message{std::in_place_index<I>}...};
}

}  // namespace

TEST_CASE("match contracts: constants are consistent") {
    CHECK(kMatchProtocolVersion >= kMinCompatibleMatchProtocolVersion);
    CHECK(kMaxRobotsPerMatch == 6);
    CHECK(kInputRedundancy == 3);
    CHECK(kMaxWheelsPerRobot == 8);
    CHECK(kMaxZonesPerRobot <= 32);
    CHECK(kMaxNameBytes == 64);
    CHECK(kInputTimeoutUs == 250'000);
    CHECK(kMaxExtrapolationUs == 250'000);
    CHECK(kDefaultInputRateHz <= kMaxInputRateHz);
    CHECK(kSnapshotRateWanHz < kSnapshotRateLanHz);
    CHECK(kHostParticipantId == 0);
    CHECK(kNoParticipant > kMaxRobotsPerMatch);
    CHECK(relay::kRelayRoomIdChars == 8);
    CHECK(relay::kRelayHostAuthBytes == 32);
    CHECK(relay::kRelayBroadcastPeer == 0xFFFF);
    CHECK(relay::kRelayChannelControl == 0);
    CHECK(relay::kRelayChannelGameReliable == 1);
    CHECK(relay::kRelayChannelGameUnreliable == 2);
    CHECK(relay::kRelayMaxPeersPerRoom == 6);
    CHECK(relay::kRelayMaxDatagram > kSnapshotBudgetBytes);
    CHECK(relay::kRelayMaxReliablePacket > kMaxRobotSpecBytes);
}

TEST_CASE("match contracts: every message alternative has its documented type value") {
    CHECK(std::variant_size_v<Message> == kLastMessageTypeValue);
    CHECK(kExtensionMessageCount == 12);
    checkAlternative<VisionSetup>(MessageType::VisionSetup, 12);
    checkAlternative<match::Join>(MessageType::Join, 13);
    checkAlternative<match::JoinAccept>(MessageType::JoinAccept, 14);
    checkAlternative<match::JoinReject>(MessageType::JoinReject, 15);
    checkAlternative<match::Roster>(MessageType::Roster, 16);
    checkAlternative<match::Leave>(MessageType::Leave, 17);
    checkAlternative<match::MatchInputPacket>(MessageType::MatchInputPacket, 18);
    checkAlternative<match::MatchSnapshot>(MessageType::MatchSnapshot, 19);
    checkAlternative<match::MatchSnapshotAck>(MessageType::MatchSnapshotAck, 20);
    checkAlternative<match::MatchControl>(MessageType::MatchControl, 21);
    checkAlternative<match::relay::RegisterRoom>(MessageType::RegisterRoom, 22);
    checkAlternative<match::relay::RoomRegistered>(MessageType::RoomRegistered, 23);
    checkAlternative<match::relay::RoomRejected>(MessageType::RoomRejected, 24);
    checkAlternative<match::relay::JoinRoom>(MessageType::JoinRoom, 25);
    checkAlternative<match::relay::JoinResult>(MessageType::JoinResult, 26);
    checkAlternative<match::relay::PeerJoined>(MessageType::PeerJoined, 27);
    checkAlternative<match::relay::PeerLeft>(MessageType::PeerLeft, 28);
}

TEST_CASE("match contracts: index and type value agree for ALL alternatives and the existing values are unchanged") {
    const std::vector<Message> all = oneOfEach(std::make_index_sequence<std::variant_size_v<Message>>{});
    REQUIRE(all.size() == kLastMessageTypeValue);
    for (std::size_t i = 0; i < all.size(); ++i) {
        CAPTURE(i);
        CHECK(static_cast<std::size_t>(typeOf(all[i])) == i + 1);
        CHECK(isMatchMessage(all[i]) == (i >= kExtensionMessageCount));
    }
    CHECK(static_cast<int>(MessageType::Hello) == 1);
    CHECK(static_cast<int>(MessageType::SimControlAck) == 11);
}

TEST_CASE("match contracts: deliveryOf is unreliable only for inputs, snapshots and acks") {
    const std::vector<Message> all = oneOfEach(std::make_index_sequence<std::variant_size_v<Message>>{});
    for (const Message& m : all) {
        const MessageType t = typeOf(m);
        const bool lossy = t == MessageType::MatchInputPacket || t == MessageType::MatchSnapshot ||
                           t == MessageType::MatchSnapshotAck;
        CAPTURE(static_cast<int>(t));
        CHECK(deliveryOf(m) == (lossy ? Delivery::Unreliable : Delivery::Reliable));
    }
    CHECK(deliveryOf(Message{match::MatchControl{}}) == Delivery::Reliable);
    CHECK(deliveryOf(Message{match::Join{}}) == Delivery::Reliable);
}

TEST_CASE("match contracts: default-constructed messages are empty, valid and equal to themselves") {
    CHECK(match::Join{} == match::Join{});
    CHECK(match::Join{}.protocolVersion == kMatchProtocolVersion);
    CHECK(match::Join{}.robotSpecJson.empty());
    CHECK_FALSE(hasResumeToken(match::Join{}.resumeToken));
    const match::JoinAccept accept;
    CHECK(accept == match::JoinAccept{});
    CHECK(accept.roster.empty());
    CHECK(accept.sessionId == kUnspecified);
    CHECK(accept.runEpoch == kFirstRunEpoch);
    CHECK(match::JoinReject{} == match::JoinReject{});
    CHECK(match::Roster{}.entries.empty());
    CHECK(match::Leave{} == match::Leave{});

    const MatchInput input;
    CHECK(input.inputSeq == kNoInputSeq);
    CHECK(input.wheels.empty());
    CHECK(fitsLimits(input));
    CHECK(fitsLimits(MatchInputPacket{}));
    CHECK(MatchInputPacket{}.inputs.empty());

    const MatchSnapshot snapshot;
    CHECK(fitsLimits(snapshot));
    CHECK(snapshot == MatchSnapshot{});
    CHECK(snapshot.ackInputSeq == kNoInputSeq);
    CHECK(snapshot.ackTick == kNoAckTick);
    CHECK(snapshot.info.state == MatchState::Lobby);
    CHECK(snapshot.info.matchTimeS == -1.f);
    CHECK(snapshot.robots.empty());
    CHECK(snapshot.entityDeltas.empty());
    CHECK(EntityDelta{}.baselineId == kNoBaseline);
    CHECK(MatchSnapshotAck{} == MatchSnapshotAck{});

    CHECK(match::MatchControl{} == match::MatchControl{});
    CHECK(match::MatchControl{}.target == kNoParticipant);
    CHECK(relay::RegisterRoom{} == relay::RegisterRoom{});
    CHECK_FALSE(relay::JoinResult{}.ok);
    CHECK(relay::JoinResult{}.peerId == relay::kRelayBroadcastPeer);
    CHECK(relay::PeerLeft{} == relay::PeerLeft{});
}

TEST_CASE("match contracts: station carries the alliance") {
    const RosterEntry entry;
    CHECK(allianceOf(entry.station) == Alliance::Red);
    CHECK(allianceOf(AllianceStation::Blue2) == Alliance::Blue);
}

TEST_CASE("quantize: angle round trip stays within half a step modulo a full turn") {
    CHECK(kMaxAngleQuantizationErrorRad == doctest::Approx(3.14159265358979 / 65536.0));
    for (double a = -7.0; a <= 7.0; a += 0.00137) {
        const double back = dequantizeAngle(quantizeAngle(a));
        CHECK(std::fabs(angleDifferenceRad(back, a)) <= kMaxAngleQuantizationErrorRad + kEps);
        CHECK(back >= -kPi - kEps);
        CHECK(back < kPi + kEps);
    }
}

TEST_CASE("quantize: angle edge values and wrap at pi") {
    CHECK(quantizeAngle(0.0) == 0);
    CHECK(quantizeAngle(kTwoPi) == 0);
    CHECK(quantizeAngle(-kTwoPi) == 0);
    CHECK(quantizeAngle(kPi) == std::numeric_limits<std::int16_t>::min());
    CHECK(quantizeAngle(-kPi) == std::numeric_limits<std::int16_t>::min());
    CHECK(quantizeAngle(3.0 * kPi) == std::numeric_limits<std::int16_t>::min());
    CHECK(quantizeAngle(kPi / 2.0) == 16384);
    CHECK(quantizeAngle(-kPi / 2.0) == -16384);
    CHECK(quantizeAngle(kPi - 1e-9) == std::numeric_limits<std::int16_t>::min());  // rounds up onto the wrap
    CHECK(quantizeAngle(-1e-12) == 0);
    CHECK(dequantizeAngle(std::numeric_limits<std::int16_t>::min()) == doctest::Approx(-kPi));
    CHECK(dequantizeAngle(std::numeric_limits<std::int16_t>::max()) < kPi);
}

TEST_CASE("quantize: every i16 angle value survives a dequantize and quantize round trip") {
    for (int q = std::numeric_limits<std::int16_t>::min(); q <= std::numeric_limits<std::int16_t>::max(); ++q)
        REQUIRE(quantizeAngle(dequantizeAngle(static_cast<std::int16_t>(q))) == q);
}

TEST_CASE("quantize: non-finite and huge angles map to a defined value") {
    CHECK(quantizeAngle(std::numeric_limits<double>::quiet_NaN()) == 0);
    CHECK(quantizeAngle(std::numeric_limits<double>::infinity()) == 0);
    CHECK(quantizeAngle(-std::numeric_limits<double>::infinity()) == 0);
    const std::int16_t huge = quantizeAngle(1e300);
    CHECK(huge >= std::numeric_limits<std::int16_t>::min());
}

TEST_CASE("quantize: angleDifferenceRad is the shortest signed difference") {
    CHECK(angleDifferenceRad(0.1, -0.1) == doctest::Approx(0.2));
    CHECK(angleDifferenceRad(kPi - 0.1, -kPi + 0.1) == doctest::Approx(-0.2));
    CHECK(angleDifferenceRad(-kPi + 0.1, kPi - 0.1) == doctest::Approx(0.2));
    CHECK(std::fabs(angleDifferenceRad(kTwoPi, 0.0)) < 1e-12);
}

TEST_CASE("quantize: millimetre positions round trip within half a millimetre") {
    for (double m = -40.0; m <= 40.0; m += 0.000731) {
        const double back = dequantizeMilli(quantizeMilliI32(m));
        CHECK(std::fabs(back - m) <= 0.0005 + 1e-12);
    }
    CHECK(quantizeMilliI32(0.0) == 0);
    CHECK(quantizeMilliI32(1.2345) == 1235);
    CHECK(quantizeMilliI32(-1.2345) == -1235);
    CHECK(quantizeMilliI32(8.2296) == 8230);
}

TEST_CASE("quantize: milli values saturate instead of overflowing") {
    CHECK(quantizeMilliI32(1e12) == std::numeric_limits<std::int32_t>::max());
    CHECK(quantizeMilliI32(-1e12) == std::numeric_limits<std::int32_t>::min());
    CHECK(quantizeMilliI32(2147483.647) == std::numeric_limits<std::int32_t>::max());
    CHECK(quantizeMilliI16(1000.0) == std::numeric_limits<std::int16_t>::max());
    CHECK(quantizeMilliI16(-1000.0) == std::numeric_limits<std::int16_t>::min());
    CHECK(quantizeMilliI16(32.767) == std::numeric_limits<std::int16_t>::max());
    CHECK(quantizeMilliI16(32.7675) == std::numeric_limits<std::int16_t>::max());
    CHECK(quantizeCentiI16(1e9) == std::numeric_limits<std::int16_t>::max());
    CHECK(quantizeCentiI16(-1e9) == std::numeric_limits<std::int16_t>::min());
}

TEST_CASE("quantize: milli and centi values map NaN and infinity to zero") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(quantizeMilliI32(nan) == 0);
    CHECK(quantizeMilliI32(inf) == 0);
    CHECK(quantizeMilliI32(-inf) == 0);
    CHECK(quantizeMilliI16(nan) == 0);
    CHECK(quantizeMilliI16(inf) == 0);
    CHECK(quantizeCentiI16(nan) == 0);
    CHECK(quantizeCentiI16(-inf) == 0);
}

TEST_CASE("quantize: velocities round trip within half a unit across the usable range") {
    for (double v = -30.0; v <= 30.0; v += 0.00317) {
        CHECK(std::fabs(dequantizeMilli(quantizeMilliI16(v)) - v) <= 0.0005 + 1e-12);
    }
}

TEST_CASE("quantize: entity centimetres round trip within half a centimetre") {
    for (double m = -300.0; m <= 300.0; m += 0.0173) {
        CHECK(std::fabs(dequantizeCenti(quantizeCentiI16(m)) - m) <= 0.005 + 1e-12);
    }
    CHECK(quantizeCentiI16(8.2296) == 823);
}

TEST_CASE("match contracts: sequence numbers compare across the u32 wrap") {
    CHECK(seqNewer(2, 1));
    CHECK_FALSE(seqNewer(1, 2));
    CHECK_FALSE(seqNewer(5, 5));
    CHECK(seqNewer(0, 0xFFFFFFFFu));
    CHECK(seqNewer(3, 0xFFFFFFF0u));
    CHECK_FALSE(seqNewer(0xFFFFFFFFu, 0));
}

TEST_CASE("match contracts: appendInput keeps the newest kInputRedundancy inputs oldest first") {
    MatchInputPacket packet;
    for (std::uint32_t seq = 1; seq <= 5; ++seq) {
        MatchInput input;
        input.inputSeq = seq;
        appendInput(packet, input);
        CHECK(packet.inputs.size() == std::min<std::size_t>(seq, kInputRedundancy));
        CHECK(packet.inputs.back().inputSeq == seq);
        CHECK(fitsLimits(packet));
    }
    CHECK(packet.inputs.front().inputSeq == 3);
    CHECK(packet.inputs[1].inputSeq == 4);
}

TEST_CASE("match contracts: fitsLimits rejects over-long lists") {
    MatchInput input;
    input.wheels.resize(kMaxWheelsPerRobot);
    CHECK(fitsLimits(input));
    input.wheels.resize(kMaxWheelsPerRobot + 1);
    CHECK_FALSE(fitsLimits(input));

    MatchInputPacket packet;
    packet.inputs.resize(kInputRedundancy + 1);
    CHECK_FALSE(fitsLimits(packet));

    MatchSnapshot snapshot;
    snapshot.robots.resize(kMaxRobotsPerMatch);
    CHECK(fitsLimits(snapshot));
    snapshot.robots.resize(kMaxRobotsPerMatch + 1);
    CHECK_FALSE(fitsLimits(snapshot));
    snapshot.robots.resize(1);
    snapshot.robots[0].wheels.resize(kMaxWheelsPerRobot + 1);
    CHECK_FALSE(fitsLimits(snapshot));
    snapshot.robots[0].wheels.clear();
    snapshot.entityDeltas.resize(1);
    snapshot.entityDeltas[0].upserted.resize(kMaxEntityDeltaEntries + 1);
    CHECK_FALSE(fitsLimits(snapshot));
}

TEST_CASE("match contracts: zone flags and wheel speed helpers") {
    CHECK(zoneFlagBit(0) == 1u);
    CHECK(zoneFlagBit(15) == 0x8000u);
    CHECK(zoneFlagBit(kMaxZonesPerRobot) == 0u);
    CHECK(zoneFlagsMask(0) == 0u);
    CHECK(zoneFlagsMask(3) == 7u);
    CHECK(zoneFlagsMask(kMaxZonesPerRobot) == 0xFFFFu);
    CHECK(zoneFlagsMask(100) == 0xFFFFu);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK(sanitizedWheelSpeed(nan, 50.f) == 0.f);
    CHECK(sanitizedWheelSpeed(std::numeric_limits<float>::infinity(), 50.f) == 0.f);
    CHECK(sanitizedWheelSpeed(80.f, 50.f) == 50.f);
    CHECK(sanitizedWheelSpeed(-80.f, 50.f) == -50.f);
    CHECK(sanitizedWheelSpeed(12.5f, 50.f) == 12.5f);
}

TEST_CASE("match contracts: room ids and resume tokens") {
    CHECK(relay::isValidRoomId("0123ABCD"));
    CHECK(relay::isValidRoomId("ZZZZZZZZ"));
    CHECK_FALSE(relay::isValidRoomId("0123ABC"));
    CHECK_FALSE(relay::isValidRoomId("0123ABCDE"));
    CHECK_FALSE(relay::isValidRoomId("0123ABCI"));  // I, L, O, U are not in the alphabet
    CHECK_FALSE(relay::isValidRoomId("0123abcd"));
    ResumeToken token{};
    CHECK_FALSE(hasResumeToken(token));
    token[kResumeTokenBytes - 1] = 1;
    CHECK(hasResumeToken(token));
}
