#pragma once

// One representative instance of every practice-match and relay message (MessageType 13..29), every field set to a
// non-default value so a dropped or reordered field shows. Shared by the golden, hostile-input and delivery tests.

#include <string>
#include <vector>

#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::test_ipc {

struct NamedMessage {
    const char* name;
    sdk::proto::Message message;
};

inline sdk::match::ResumeToken sampleResumeToken() {
    sdk::match::ResumeToken token{};
    for (std::size_t i = 0; i < token.size(); ++i) token[i] = static_cast<std::uint8_t>(0xA0 + i);
    return token;
}

inline sdk::match::RosterEntry sampleRosterEntry(sdk::match::ParticipantId id, const char* name,
                                                 sdk::proto::AllianceStation station, bool host) {
    sdk::match::RosterEntry e;
    e.participantId = id;
    e.name = name;
    e.station = station;
    e.pingMs = static_cast<std::uint16_t>(20 + id);
    e.lossPermille = static_cast<std::uint16_t>(5 * id);
    e.isHost = host;
    return e;
}

inline sdk::match::MatchInput sampleMatchInput(std::uint32_t seq) {
    sdk::match::MatchInput in;
    in.inputSeq = seq;
    in.targetTick = 1000 + seq;
    in.wheels = {{12.5f, 1024}, {-3.25f, -2048}, {0.f, 32767}, {100.f, -32768}};
    in.zoneFlags = 0x5u + seq;
    in.zonePoses = {{1, 300, -150, 500, 0x4000, -0x2000, 0x0102}, {5, -32768, 32767, 0, -32768, 32767, 0}};
    return in;
}

inline sdk::match::MatchSnapshot sampleMatchSnapshot() {
    using namespace sdk::match;
    MatchSnapshot s;
    s.sessionId = 0x1122334455667788ULL;
    s.runEpoch = 7;
    s.snapshotSeq = 4321;
    s.tick = 987654;
    s.simTimeUs = 4938270000LL;
    s.ackInputSeq = 99;
    s.ackTick = 987650;
    s.inputLeadTicks = -3;
    s.info.state = MatchState::Paused;
    s.info.enabled = true;
    s.info.mode = sdk::proto::RobotMode::Autonomous;
    s.info.eStop = true;
    s.info.matchTimeS = 82.5f;
    s.info.phaseKey = "phase.auto";
    for (ParticipantId p = 0; p < 2; ++p) {
        RobotWireState r;
        r.participantId = p;
        r.xMm = 1500 * (p + 1);
        r.yMm = -2750 * (p + 1);
        r.yawQ = static_cast<std::int16_t>(1000 * (p + 1));
        r.vxMmps = -1200;
        r.vyMmps = 3400;
        r.omegaMradps = 560;
        r.wheels = {{100, -200}, {300, -400}};
        r.zoneFlags = 0x9u + p;
        r.hud = {{1, 12.f}, {2, 0.5f}};
        r.zonePoses = {{static_cast<std::uint8_t>(p), static_cast<std::int16_t>(250 * (p + 1)), -75, 640, 0x1234, 0, -1},
                       {15, -1000, 2000, 3000, 4, 5, 6}};
        s.robots.push_back(r);
    }
    EntityDelta d;
    d.groupId = 1;
    d.baselineId = 4320;
    d.removed = {11, 22};
    d.upserted = {{33, 100, -200, 5}, {44, -300, 400, 0}};
    s.entityDeltas.push_back(d);
    s.gameHud = {{7, 123.f}, {8, -1.f}};
    return s;
}

// Equality of two messages of the match protocol (the HALSim messages do not all define operator==).
inline bool sameMatchMessage(const sdk::proto::Message& a, const sdk::proto::Message& b) {
    if (a.index() != b.index()) return false;
    return std::visit(
        [&b](const auto& x) {
            using T = std::decay_t<decltype(x)>;
            if constexpr (requires(const T& l, const T& r) { l == r; }) return x == std::get<T>(b);
            else return false;
        },
        a);
}

inline std::vector<NamedMessage> matchSamples() {
    using namespace sdk::match;
    using namespace sdk::match::relay;
    using sdk::proto::AllianceStation;
    using sdk::proto::Message;
    std::vector<NamedMessage> all;

    Join join;
    join.displayName = "Team 254 \xC3\xA9";
    join.gamePluginId = "game2026";
    join.wpilibYear = 2026;
    join.resumeToken = sampleResumeToken();
    join.robotSpecJson = R"({"chassis":{"massKg":50}})";
    all.push_back({"Join", join});

    JoinAccept accept;
    accept.participantId = 3;
    accept.sessionId = 0x1122334455667788ULL;
    accept.runEpoch = 7;
    accept.station = AllianceStation::Blue2;
    accept.stepUs = 5000;
    accept.snapshotRateHz = 30;
    accept.inputRateHz = 90;
    accept.startPose.position = {1.5f, -2.25f, 0.5f};
    accept.startPose.rotation = {0.f, 0.f, 0.25f, 0.75f};
    accept.resumeToken = sampleResumeToken();
    accept.state = MatchState::Running;
    accept.serverTick = 12345;
    accept.serverSimTimeUs = 61725000;
    accept.roster = {sampleRosterEntry(0, "Host", AllianceStation::Red1, true),
                     sampleRosterEntry(3, "Zed", AllianceStation::Blue2, false)};
    all.push_back({"JoinAccept", accept});

    JoinReject reject;
    reject.reason = JoinRejectReason::BadSpec;
    reject.detail = "robot.drivetrain.wheelCount";
    all.push_back({"JoinReject", reject});

    Roster roster;
    roster.sessionId = 0x0102030405060708ULL;
    roster.runEpoch = 9;
    roster.entries = {sampleRosterEntry(0, "Host", AllianceStation::Red1, true),
                      sampleRosterEntry(1, "Amy", AllianceStation::Red2, false),
                      sampleRosterEntry(2, "Bob", AllianceStation::Blue3, false)};
    all.push_back({"Roster", roster});

    Leave leave;
    leave.detail = "bye";
    all.push_back({"Leave", leave});

    MatchInputPacket packet;
    packet.sessionId = 0x1122334455667788ULL;
    packet.runEpoch = 7;
    packet.inputs = {sampleMatchInput(41), sampleMatchInput(42), sampleMatchInput(43)};
    all.push_back({"MatchInputPacket", packet});

    all.push_back({"MatchSnapshot", sampleMatchSnapshot()});

    MatchSnapshotAck snapshotAck;
    snapshotAck.sessionId = 0x1122334455667788ULL;
    snapshotAck.runEpoch = 7;
    snapshotAck.snapshotSeq = 4321;
    all.push_back({"MatchSnapshotAck", snapshotAck});

    MatchControl control;
    control.kind = MatchControlKind::Kick;
    control.sessionId = 0x1122334455667788ULL;
    control.runEpoch = 8;
    control.target = 2;
    control.pingSeq = 77;
    control.clientTimeUs = 5555555;
    control.serverTick = 424242;
    control.serverSimTimeUs = 2121210000;
    control.detail = "removed by host";
    all.push_back({"MatchControl", control});

    RegisterRoom reg;
    reg.roomId = "7K3QZ9XM";
    for (std::size_t i = 0; i < reg.hostAuth.size(); ++i) reg.hostAuth[i] = static_cast<std::uint8_t>(i * 7 + 1);
    all.push_back({"RegisterRoom", reg});

    RoomRegistered registered;
    registered.resumed = true;
    registered.maxPeers = 5;
    all.push_back({"RoomRegistered", registered});

    RoomRejected rejected;
    rejected.reason = RoomRejectReason::RelayFull;
    all.push_back({"RoomRejected", rejected});

    JoinRoom joinRoom;
    joinRoom.roomId = "0123ABCD";
    all.push_back({"JoinRoom", joinRoom});

    JoinResult result;
    result.ok = false;
    result.reason = JoinRoomFailure::HostAbsent;
    result.peerId = 0xFFFF;
    all.push_back({"JoinResult", result});

    PeerJoined joined;
    joined.peerId = 4;
    all.push_back({"PeerJoined", joined});

    PeerLeft left;
    left.peerId = 4;
    left.reason = PeerLeftReason::Oversized;
    all.push_back({"PeerLeft", left});

    all.push_back({"CloseRoom", CloseRoom{}});
    return all;
}

}  // namespace wpiemu::test_ipc
