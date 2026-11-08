#pragma once

// Host <-> client lifecycle commands of a practice match (reliable): reset to a new epoch, pause and resume, remove a
// participant, close the server, and the ping that measures round-trip time and seeds the client's clock sync.
// One struct with a `kind`, so the codec has a single record. See match_protocol.h for the run identity.

#include <cstdint>
#include <string>

#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::sdk::match {

enum class MatchControlKind : std::uint8_t {
    Reset = 0,          // host -> all: the world restarted; `runEpoch` is the NEW epoch, clients drop older frames and re-sync
    Pause = 1,          // host -> all
    Resume = 2,         // host -> all
    Kick = 3,           // host -> victim: `target` is removed and the connection closes; `detail` says why
    ServerClosing = 4,  // host -> all: the room closes, return to the join screen
    PingRequest = 5,    // client -> host
    PingReply = 6,      // host -> client; last value 6
};

struct MatchControl {
    MatchControlKind kind = MatchControlKind::Reset;
    std::uint64_t sessionId = proto::kUnspecified;
    std::uint64_t runEpoch = proto::kUnspecified;
    ParticipantId target = kNoParticipant;  // Kick: the participant removed
    std::uint32_t pingSeq = 0;              // Ping*: echoed by the reply
    // PingRequest: the client's own monotonic microseconds when it sent the request (opaque to the host). PingReply:
    // the same value echoed, so RTT = clientNow - clientTimeUs.
    std::int64_t clientTimeUs = 0;
    // PingReply only: the host's tick and sim time when it answered, for the clock-sync offset (minimum-RTT filter).
    std::uint64_t serverTick = 0;
    std::int64_t serverSimTimeUs = 0;
    std::string detail;  // <= kMaxDetailBytes

    bool operator==(const MatchControl&) const = default;
};

}  // namespace wpiemu::sdk::match
