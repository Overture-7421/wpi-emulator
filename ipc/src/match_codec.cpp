#include "match_codec.h"

#include <array>
#include <new>
#include <string>
#include <type_traits>
#include <utility>

#include "wpiemu/sdk/core/wire_codec.h"
#include "wpiemu/sdk/core/wire_fields.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::ipc::detail {

using namespace sdk;
namespace match = sdk::match;
namespace relay = sdk::match::relay;

namespace {

// Wire layout: docs/reference/extension-ipc.md. Same conventions as binary_codec.cpp: one `fields(codec, value)` per
// struct for the symmetric Writer/Reader, length-prefixed records for nested structs that may grow, appended tails
// guarded by `c.more()`. Every list has a cap from sdk/match/match_protocol.h and a minimum element size, so a hostile
// count cannot allocate more than the bytes left.

struct MatchCodecDomain {};
using Writer = wire::Writer<MatchCodecDomain>;
using Reader = wire::Reader<MatchCodecDomain>;
using wire::Is;
using wire::ReadState;

// Smallest encoding of each list element (records count their 2-byte length prefix).
constexpr std::size_t kRosterEntryMinBytes = 2 + 2 + 1 + 1 + 2 + 2 + 1;
constexpr std::size_t kWheelInputBytes = 4 + 2;
constexpr std::size_t kZonePoseBytes = match::kWireZonePoseBytes;
// A MatchInput of a writer that predates zonePoses has no tail: the minimum is the record without it.
constexpr std::size_t kMatchInputMinBytes = 2 + 4 + 8 + 1 + 4;
constexpr std::size_t kWheelWireStateBytes = 2 + 2;
constexpr std::size_t kHudValueBytes = 2 + 4;
constexpr std::size_t kRobotWireStateMinBytes = 2 + 2 + 4 + 4 + 2 + 2 + 2 + 2 + 1 + 4 + 1;
constexpr std::size_t kEntityStateBytes = 4 + 2 + 2 + 2;
constexpr std::size_t kRemovedEntityBytes = 4;
constexpr std::size_t kEntityDeltaMinBytes = 2 + 2 + 4 + 1 + 1;

template <class C, class A>
void fixedBytes(C& c, A& bytes) {
    static_assert(std::is_same_v<std::remove_const_t<typename A::value_type>, std::uint8_t>);
    for (auto& b : bytes) c.u8(b);
}

template <class C, class S>
void roomId(C& c, S& id) {
    c.str(id, relay::kRelayRoomIdChars, "room id too long");
    if (!relay::isValidRoomId(id)) c.fail("invalid room id");
}

// ---- shared records ---------------------------------------------------------------------------------------

template <class C, Is<match::RosterEntry> M>
void fields(C& c, M& e) {
    c.record([&] {
        c.u16(e.participantId);
        c.str(e.name, match::kMaxNameBytes, "roster name too long");
        c.enumeration(e.station, 5, "invalid AllianceStation");
        c.u16(e.pingMs);
        c.u16(e.lossPermille);
        c.boolean(e.isHost);
    });
}

template <class C, Is<match::WheelInput> M>
void fields(C& c, M& w) {
    c.f32(w.wheelSpeedRadPerSec);
    c.i16(w.steerAngleQ);
}

template <class C, Is<match::ZonePoseInput> M>
void fields(C& c, M& z) {
    c.u8(z.zoneIndex);
    c.i16(z.xMm);
    c.i16(z.yMm);
    c.i16(z.zMm);
    c.i16(z.yawQ);
    c.i16(z.pitchQ);
    c.i16(z.rollQ);
}

// Appended tail of the records that carry zone poses: absent in an older record (no overrides), always written. Indices
// must be strictly increasing and below kMaxZonesPerRobot; anything else is a malformed frame (and unencodable).
template <class C, class V>
void zonePoseTail(C& c, V& poses, std::size_t cap) {
    if (!c.more()) return;
    c.list(poses, cap, kZonePoseBytes);
    if (!match::zonePosesWellFormed(poses, cap)) c.fail("zone poses not strictly increasing or index out of range");
}

template <class C, Is<match::MatchInput> M>
void fields(C& c, M& in) {
    c.record([&] {
        c.u32(in.inputSeq);
        c.u64(in.targetTick);
        c.list(in.wheels, match::kMaxWheelsPerRobot, kWheelInputBytes);
        c.u32(in.zoneFlags);
        zonePoseTail(c, in.zonePoses, match::kMaxZonesPerRobot);
    });
}

template <class C, Is<match::HudValue> M>
void fields(C& c, M& h) {
    c.u16(h.keyId);
    c.f32(h.value);
}

template <class C, Is<match::MatchInfo> M>
void fields(C& c, M& info) {
    c.record([&] {
        c.enumeration(info.state, 3, "invalid MatchState");
        c.boolean(info.enabled);
        c.enumeration(info.mode, 2, "invalid RobotMode");
        c.boolean(info.eStop);
        c.f32(info.matchTimeS);
        c.str(info.phaseKey, match::kMaxNameBytes, "phase key too long");
    });
}

template <class C, Is<match::WheelWireState> M>
void fields(C& c, M& w) {
    c.i16(w.steerAngleQ);
    c.i16(w.spinAngleQ);
}

template <class C, Is<match::RobotWireState> M>
void fields(C& c, M& r) {
    c.record([&] {
        c.u16(r.participantId);
        c.i32(r.xMm);
        c.i32(r.yMm);
        c.i16(r.yawQ);
        c.i16(r.vxMmps);
        c.i16(r.vyMmps);
        c.i16(r.omegaMradps);
        c.list(r.wheels, match::kMaxWheelsPerRobot, kWheelWireStateBytes);
        c.u32(r.zoneFlags);
        c.list(r.hud, match::kMaxHudValuesPerRobot, kHudValueBytes);
        zonePoseTail(c, r.zonePoses, match::kMaxSnapshotZonePosesPerRobot);
    });
}

template <class C, Is<match::EntityState> M>
void fields(C& c, M& e) {
    c.u32(e.id);
    c.i16(e.xCm);
    c.i16(e.yCm);
    c.i16(e.zCm);
}

template <class C, Is<match::EntityDelta> M>
void fields(C& c, M& d) {
    c.record([&] {
        c.u16(d.groupId);
        c.u32(d.baselineId);
        c.list(d.removed, match::kMaxEntityDeltaEntries, kRemovedEntityBytes);
        c.list(d.upserted, match::kMaxEntityDeltaEntries, kEntityStateBytes);
    });
}

// ---- session setup messages --------------------------------------------------------------------------------

template <class C, Is<match::Join> M>
void fields(C& c, M& m) {
    c.u32(m.protocolVersion);
    c.str(m.displayName, match::kMaxNameBytes, "display name too long");
    c.str(m.gamePluginId, match::kMaxNameBytes, "plugin id too long");
    c.i32(m.wpilibYear);
    fixedBytes(c, m.resumeToken);
    c.str(m.robotSpecJson, match::kMaxRobotSpecBytes, "robot spec too long");
}

template <class C, Is<match::JoinAccept> M>
void fields(C& c, M& m) {
    c.u16(m.participantId);
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    c.enumeration(m.station, 5, "invalid AllianceStation");
    c.i64(m.stepUs);
    c.u16(m.snapshotRateHz);
    c.u16(m.inputRateHz);
    fields(c, m.startPose);
    fixedBytes(c, m.resumeToken);
    c.enumeration(m.state, 3, "invalid MatchState");
    c.u64(m.serverTick);
    c.i64(m.serverSimTimeUs);
    c.list(m.roster, match::kMaxRobotsPerMatch, kRosterEntryMinBytes);
}

template <class C, Is<match::JoinReject> M>
void fields(C& c, M& m) {
    c.enumeration(m.reason, 9, "invalid JoinRejectReason");
    c.str(m.detail, match::kMaxDetailBytes, "detail too long");
}

template <class C, Is<match::Roster> M>
void fields(C& c, M& m) {
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    c.list(m.entries, match::kMaxRobotsPerMatch, kRosterEntryMinBytes);
}

template <class C, Is<match::Leave> M>
void fields(C& c, M& m) {
    c.str(m.detail, match::kMaxDetailBytes, "detail too long");
}

// ---- gameplay messages -------------------------------------------------------------------------------------

template <class C, Is<match::MatchInputPacket> M>
void fields(C& c, M& m) {
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    c.list(m.inputs, match::kInputRedundancy, kMatchInputMinBytes);
}

template <class C, Is<match::MatchSnapshot> M>
void fields(C& c, M& m) {
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    c.u32(m.snapshotSeq);
    c.u64(m.tick);
    c.i64(m.simTimeUs);
    c.u32(m.ackInputSeq);
    c.u64(m.ackTick);
    c.i16(m.inputLeadTicks);
    fields(c, m.info);
    c.list(m.robots, match::kMaxRobotsPerMatch, kRobotWireStateMinBytes);
    c.list(m.entityDeltas, match::kMaxEntityGroups, kEntityDeltaMinBytes);
    c.list(m.gameHud, match::kMaxGameHudValues, kHudValueBytes);
}

template <class C, Is<match::MatchSnapshotAck> M>
void fields(C& c, M& m) {
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    c.u32(m.snapshotSeq);
}

template <class C, Is<match::MatchControl> M>
void fields(C& c, M& m) {
    c.enumeration(m.kind, 6, "invalid MatchControlKind");
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    c.u16(m.target);
    c.u32(m.pingSeq);
    c.i64(m.clientTimeUs);
    c.u64(m.serverTick);
    c.i64(m.serverSimTimeUs);
    c.str(m.detail, match::kMaxDetailBytes, "detail too long");
}

// ---- relay control messages --------------------------------------------------------------------------------

template <class C, Is<relay::RegisterRoom> M>
void fields(C& c, M& m) {
    c.u32(m.protocolVersion);
    roomId(c, m.roomId);
    fixedBytes(c, m.hostAuth);
}

template <class C, Is<relay::RoomRegistered> M>
void fields(C& c, M& m) {
    c.boolean(m.resumed);
    c.u16(m.maxPeers);
}

template <class C, Is<relay::RoomRejected> M>
void fields(C& c, M& m) {
    c.enumeration(m.reason, 3, "invalid RoomRejectReason");
}

template <class C, Is<relay::JoinRoom> M>
void fields(C& c, M& m) {
    c.u32(m.protocolVersion);
    roomId(c, m.roomId);
}

template <class C, Is<relay::JoinResult> M>
void fields(C& c, M& m) {
    c.boolean(m.ok);
    c.enumeration(m.reason, 5, "invalid JoinRoomFailure");
    c.u16(m.peerId);
}

template <class C, Is<relay::PeerJoined> M>
void fields(C& c, M& m) {
    c.u16(m.peerId);
}

template <class C, Is<relay::PeerLeft> M>
void fields(C& c, M& m) {
    c.u16(m.peerId);
    c.enumeration(m.reason, 3, "invalid PeerLeftReason");
}

template <class C, Is<relay::CloseRoom> M>
void fields(C&, M&) {}

template <class T>
Result<proto::Message> decodeBody(std::span<const std::byte> body) {
    ReadState state;
    Reader r(body, &state);
    T value;
    fields(r, value);
    if (!state.ok) return makeError(ErrorCode::ProtocolError, std::string("malformed frame: ") + state.what);
    return proto::Message{std::move(value)};
}

}  // namespace

bool encodeMatchBody(const proto::Message& message, std::vector<std::byte>& out) {
    Writer w(out);
    std::visit(
        [&w](const auto& m) {
            if constexpr (requires { fields(w, m); }) fields(w, m);
        },
        message);
    return !w.bad();
}

Result<proto::Message> decodeMatchBody(proto::MessageType type, std::span<const std::byte> body) {
    using proto::MessageType;
    switch (type) {
        case MessageType::Join: return decodeBody<match::Join>(body);
        case MessageType::JoinAccept: return decodeBody<match::JoinAccept>(body);
        case MessageType::JoinReject: return decodeBody<match::JoinReject>(body);
        case MessageType::Roster: return decodeBody<match::Roster>(body);
        case MessageType::Leave: return decodeBody<match::Leave>(body);
        case MessageType::MatchInputPacket: return decodeBody<match::MatchInputPacket>(body);
        case MessageType::MatchSnapshot: return decodeBody<match::MatchSnapshot>(body);
        case MessageType::MatchSnapshotAck: return decodeBody<match::MatchSnapshotAck>(body);
        case MessageType::MatchControl: return decodeBody<match::MatchControl>(body);
        case MessageType::RegisterRoom: return decodeBody<relay::RegisterRoom>(body);
        case MessageType::RoomRegistered: return decodeBody<relay::RoomRegistered>(body);
        case MessageType::RoomRejected: return decodeBody<relay::RoomRejected>(body);
        case MessageType::JoinRoom: return decodeBody<relay::JoinRoom>(body);
        case MessageType::JoinResult: return decodeBody<relay::JoinResult>(body);
        case MessageType::PeerJoined: return decodeBody<relay::PeerJoined>(body);
        case MessageType::PeerLeft: return decodeBody<relay::PeerLeft>(body);
        case MessageType::CloseRoom: return decodeBody<relay::CloseRoom>(body);
        default: break;
    }
    return makeError(ErrorCode::Unsupported, "unknown message type " + std::to_string(static_cast<unsigned>(type)));
}

}  // namespace wpiemu::ipc::detail
