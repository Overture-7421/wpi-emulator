#pragma once

// Session setup messages of the match protocol (all reliable): a client's Join, the host's JoinAccept or JoinReject, the
// Roster the host keeps current, and Leave. Plain structs; see match_protocol.h for the run identity and limits.
//
// Handshake order, after the secure transport is up: client -> Join; host -> JoinAccept (or JoinReject, then closes);
// host -> Roster (to everyone, on every change and about once a second for ping and loss figures).

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::sdk::match {

// Rejoin credential issued in JoinAccept. All-zero means "none".
using ResumeToken = std::array<std::uint8_t, kResumeTokenBytes>;
constexpr bool hasResumeToken(const ResumeToken& token) {
    for (const std::uint8_t b : token)
        if (b != 0) return true;
    return false;
}

struct Join {
    std::uint32_t protocolVersion = kMatchProtocolVersion;
    std::string displayName;  // <= kMaxNameBytes
    std::string gamePluginId;  // must equal the host's plugin (WrongPlugin)
    std::int32_t wpilibYear = 0;  // must equal the host's season (WrongSeason)
    ResumeToken resumeToken{};    // from an earlier JoinAccept when rejoining after a reconnect
    // The robot description: JSON of a RobotConfig subset (chassis, drivetrain, zones, plugin sections), <= kMaxRobotSpecBytes.
    // The host decodes it with the existing RobotConfigCodec and ConfigPipeline and then applies the fairness clamps; this
    // header deliberately defines no second spec struct.
    std::string robotSpecJson;

    bool operator==(const Join&) const = default;
};

struct RosterEntry {
    ParticipantId participantId = kHostParticipantId;
    std::string name;
    // Carries the alliance too (proto::allianceOf) so the two can never disagree.
    proto::AllianceStation station = proto::AllianceStation::Red1;
    std::uint16_t pingMs = 0;
    std::uint16_t lossPermille = 0;  // fraction of this participant's packets lost, in 1/1000
    bool isHost = false;

    bool operator==(const RosterEntry&) const = default;
};

struct JoinAccept {
    ParticipantId participantId = kNoParticipant;
    std::uint64_t sessionId = proto::kUnspecified;  // identity of this connection (all later frames carry it)
    std::uint64_t runEpoch = proto::kFirstRunEpoch;
    proto::AllianceStation station = proto::AllianceStation::Red1;  // assigned alliance and station
    std::int64_t stepUs = 5000;  // the host's simulation step (200 Hz)
    std::uint16_t snapshotRateHz = kSnapshotRateLanHz;  // initial; the host adapts it per peer
    std::uint16_t inputRateHz = kDefaultInputRateHz;
    Pose3 startPose;  // the accepted robot's initial pose in the field frame
    ResumeToken resumeToken{};
    MatchState state = MatchState::Lobby;
    std::uint64_t serverTick = 0;       // host tick and sim time at acceptance: seeds the client's clock sync
    std::int64_t serverSimTimeUs = 0;
    std::vector<RosterEntry> roster;    // <= kMaxRobotsPerMatch

    bool operator==(const JoinAccept&) const = default;
};

struct JoinReject {
    JoinRejectReason reason = JoinRejectReason::BadProtocol;
    std::string detail;  // human-readable, <= kMaxDetailBytes (e.g. the first validation issue key for BadSpec)

    bool operator==(const JoinReject&) const = default;
};

struct Roster {
    std::uint64_t sessionId = proto::kUnspecified;
    std::uint64_t runEpoch = proto::kUnspecified;
    std::vector<RosterEntry> entries;  // <= kMaxRobotsPerMatch

    bool operator==(const Roster&) const = default;
};

// Client -> host: the participant is leaving; the host drops the robot. (The host removing someone is MatchControl::Kick.)
struct Leave {
    std::string detail;  // <= kMaxDetailBytes

    bool operator==(const Leave&) const = default;
};

}  // namespace wpiemu::sdk::match
