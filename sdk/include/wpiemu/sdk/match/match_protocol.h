#pragma once

// Constants and small enums of the practice-match protocol between the host app (the game server) and its game clients.
// Plain data only: the codec in ipc/ decides the serialisation. Message headers in this directory:
//   join_messages.h      Join, JoinAccept, JoinReject, Roster, Leave
//   input_messages.h     WheelInput, MatchInput, MatchInputPacket (client -> host)
//   snapshot_messages.h  MatchSnapshot and its parts, MatchSnapshotAck (host <-> client)
//   match_control.h      MatchControl (reset, pause, kick, ping)
//   relay_messages.h     the plaintext relay control messages (they never carry game data)
//   quantize.h           the fixed-point schemes the snapshot and input fields use
//   zone_pose_wire.h     ZonePoseInput (a zone's chassis-frame pose on the wire), used by inputs and snapshots
//   robot_appearance.h   RobotAppearance (a participant's drawable robot and zones), carried by RosterEntry
// All of them are alternatives of proto::Message (ipc/messages.h), encoded by the same BinaryCodec.
//
// Run identity. Match frames reuse proto::classifyFrame (ipc/protocol_rules.h) unchanged: `sessionId` is assigned by the
// host once per accepted client connection (JoinAccept) and `runEpoch` is the host's epoch, bumped by every Reset
// (MatchControl) and every resume. Receivers drop frames of an older epoch or another session; 0 (kUnspecified) is always
// accepted. Frames sent before JoinAccept carry kUnspecified.
//
// Encryption. Everything in this directory except relay_messages.h travels inside the end-to-end encrypted transport; the
// relay sees only the relay messages, and only on its control channel.

#include <cstddef>
#include <cstdint>

namespace wpiemu::sdk::match {

inline constexpr std::uint32_t kMatchProtocolVersion = 1;
inline constexpr std::uint32_t kMinCompatibleMatchProtocolVersion = 1;

// ---- limits (also the decoder's caps: a longer list or string is a malformed frame) ------------------------------

inline constexpr std::size_t kMaxRobotsPerMatch = 6;     // host robot included
inline constexpr std::size_t kInputRedundancy = 3;       // inputs repeated in each MatchInputPacket (oldest first)
inline constexpr std::size_t kMaxWheelsPerRobot = 8;     // same bound the spec clamps use for the wheel count
inline constexpr std::size_t kMaxZonesPerRobot = 16;     // zone flags use the low bits of a u32
inline constexpr std::size_t kMaxAppearancePrimitives = 8;  // visual chassis primitives one robot's appearance lists
inline constexpr std::size_t kMaxSnapshotZonePosesPerRobot = 4;  // moved zones one robot reports in a snapshot
inline constexpr std::size_t kMaxNameBytes = 64;         // display names, plugin ids, roster names
inline constexpr std::size_t kMaxDetailBytes = 256;      // reject/kick/leave free-text details
inline constexpr std::size_t kMaxRobotSpecBytes = 64 * 1024;  // JSON robot description in Join
inline constexpr std::size_t kMaxHudValuesPerRobot = 8;  // per-robot HUD list in a snapshot
inline constexpr std::size_t kMaxGameHudValues = 16;     // game-wide HUD list in a snapshot
inline constexpr std::size_t kMaxEntityGroups = 8;       // entity groups (fuel, ...) per snapshot
inline constexpr std::size_t kMaxEntityDeltaEntries = 1024;  // upserts or removals of one group in one snapshot
inline constexpr std::size_t kResumeTokenBytes = 16;

// Best-effort snapshot size the host aims for per client (the payload before the 24-byte encryption overhead). The host
// trims entity deltas to fit; it is a budget, not a wire limit.
inline constexpr std::size_t kSnapshotBudgetBytes = 1200;

// ---- rates and timing (defaults; JoinAccept states the values the host actually uses) -----------------------------

inline constexpr std::uint16_t kDefaultInputRateHz = 60;
inline constexpr std::uint16_t kMaxInputRateHz = 120;  // the host rate-limits inputs above this
inline constexpr std::uint16_t kSnapshotRateLanHz = 60;
inline constexpr std::uint16_t kSnapshotRateWanHz = 30;
inline constexpr std::int64_t kInputTimeoutUs = 250'000;           // no input this long: the host zeroes the robot's setpoints
inline constexpr std::int64_t kMaxExtrapolationUs = 250'000;       // a client extrapolates a starved remote robot at most this long
inline constexpr std::int64_t kMaxInterpolationDelayUs = 250'000;  // upper clamp of the adaptive interpolation buffer

// ---- identities -------------------------------------------------------------------------------------------------

using ParticipantId = std::uint16_t;
inline constexpr ParticipantId kHostParticipantId = 0;   // the host's own robot
inline constexpr ParticipantId kNoParticipant = 0xFFFF;  // "nobody" (MatchControl::target of kinds that need none)

inline constexpr std::uint32_t kNoInputSeq = 0;  // MatchSnapshot::ackInputSeq before any input arrived; inputs start at 1
inline constexpr std::uint32_t kNoBaseline = 0;  // EntityDelta::baselineId meaning "full state"; snapshotSeq starts at 1

// Serial-number comparison for the wrapping u32 sequence numbers (inputSeq, snapshotSeq): true when `a` is newer than `b`.
constexpr bool seqNewer(std::uint32_t a, std::uint32_t b) { return a != b && static_cast<std::int32_t>(a - b) > 0; }

// ---- enums (wire values are fixed; the decoder bounds each by the last value noted) --------------------------------

// What the host is doing with the world. The world only advances while the host's robot is in lock-step.
enum class MatchState : std::uint8_t {
    Lobby = 0,        // waiting for the host to start
    Running = 1,
    Paused = 2,       // the host paused
    HostStalled = 3,  // the host's robot stopped answering (robot watchdog); last value 3
};

// Why the host refused a Join (or closed a connection).
enum class JoinRejectReason : std::uint8_t {
    BadProtocol = 0,     // Join::protocolVersion outside the host's compatible range
    WrongPlugin = 1,     // the client's game plugin is not the host's
    WrongSeason = 2,     // the client's WPILib year differs from the host's
    RoomFull = 3,
    BadSpec = 4,         // the robot description failed decode, validation or the fairness clamps
    Kicked = 5,          // removed by the host earlier in this session
    Banned = 6,
    HostClosing = 7,
    BadInvite = 8,       // wrong or missing invite secret (surfaced by the secure layer; kept here for the UI)
    BadResumeToken = 9,  // last value 9
};

}  // namespace wpiemu::sdk::match
