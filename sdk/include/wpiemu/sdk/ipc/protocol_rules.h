#pragma once

// Rules both peers of the emulator <-> extension protocol share: versioning, run identity and watchdog
// negotiation. The messages themselves are in messages.h; the flow is in docs/reference/extension-ipc.md.
//
// Versioning: the SDK stays at version 1 until release (docs/CONTRIBUTING.md), so `kProtocolVersion` is 1. Peers exchange it in
// Hello/HelloAck; a peer whose version is < the other's `kMinCompatibleProtocolVersion` is refused. Codecs
// tolerate absent trailing fields (default value) and ignore bytes they do not know.
//
// Lock-step robustness (see docs/ARCHITECTURE.md):
//  * WINDOW: exactly ONE outstanding tick (kLockstepWindow). The emulator sends SensorFrame N only
//    after the ActuatorFrame acknowledging N-1 arrived (or after a Reset/reconnect started a new
//    run epoch). No pipelining, so a frame can never be matched to the wrong tick.
//  * IDENTITY: every frame/command carries `sessionId` (one per accepted connection, assigned by
//    the emulator in HelloAck) and `runEpoch` (monotonic, owned by the emulator, bumped on every
//    Reset and on every reconnect). Receivers DROP frames whose epoch is older than the one in force
//    (classifyFrame). 0 means "unspecified" (a peer that does not send it, or an in-process caller) and is always accepted.
//  * ACK: every ActuatorFrame states how the tick ended (TickResult) and which tick it answers
//    (ackTick). Reset/Stop are explicit commands with a commandId that the extension acknowledges
//    with SimControlAck.
//  * WATCHDOG: both sides use the timeout negotiated in Hello/HelloAck. When it expires, or the
//    transport disconnects, the EXTENSION MUST ITSELF unblock the robot: resume HAL timing
//    (HALSIM_ResumeTiming) or terminate, per HelloAck::watchdogPolicy. It must never wait for the
//    emulator to do it (the emulator may be dead). The emulator, on its side, reports RobotStalled
//    and stops sending SensorFrames; it never blocks the sim thread past the watchdog.

#include <cstdint>

namespace wpiemu::sdk::proto {

inline constexpr std::uint32_t kProtocolVersion = 1;
inline constexpr std::uint32_t kMinCompatibleProtocolVersion = 1;

// sessionId / runEpoch / commandId value meaning "not specified" (peers that do not send it, in-process callers).
inline constexpr std::uint64_t kUnspecified = 0;
inline constexpr std::uint64_t kFirstRunEpoch = 1;
// ActuatorFrame::ackTick value meaning "not set": the frame answers the tick in `tick` (frames that do not send it).
inline constexpr std::uint64_t kNoAckTick = ~std::uint64_t{0};
// Maximum number of SensorFrames in flight (sent, not yet acknowledged by an ActuatorFrame).
inline constexpr std::uint32_t kLockstepWindow = 1;

inline constexpr std::uint32_t kDefaultWatchdogTimeoutMs = 2000;
inline constexpr std::uint32_t kMinWatchdogTimeoutMs = 100;
inline constexpr std::uint32_t kMaxWatchdogTimeoutMs = 60000;

// Watchdog timeout both sides use: the larger of what the extension proposed and what the emulator
// prefers (0 = no preference), clamped to [kMin, kMax]; kDefault when neither has a preference.
constexpr std::uint32_t negotiateWatchdogTimeoutMs(std::uint32_t proposedByExtension,
                                                   std::uint32_t preferredByEmulator) {
    std::uint32_t v = proposedByExtension > preferredByEmulator ? proposedByExtension : preferredByEmulator;
    if (v == 0) v = kDefaultWatchdogTimeoutMs;
    if (v < kMinWatchdogTimeoutMs) v = kMinWatchdogTimeoutMs;
    if (v > kMaxWatchdogTimeoutMs) v = kMaxWatchdogTimeoutMs;
    return v;
}

// What a receiver does with an incoming frame / command, judged against the identity it has in force.
enum class FrameDisposition : std::uint8_t {
    Accept = 0,
    DropStaleEpoch,       // older epoch: late frame from before a Reset/reconnect; discard silently (count it)
    DropForeignSession,   // different sessionId: belongs to another connection; discard
    NewerEpoch,           // newer epoch than in force: the extension adopts it; the emulator treats it as ProtocolError
};

constexpr FrameDisposition classifyFrame(std::uint64_t frameSession, std::uint64_t frameEpoch,
                                         std::uint64_t currentSession, std::uint64_t currentEpoch) {
    if (frameSession != kUnspecified && currentSession != kUnspecified && frameSession != currentSession)
        return FrameDisposition::DropForeignSession;
    if (frameEpoch == kUnspecified || currentEpoch == kUnspecified) return FrameDisposition::Accept;
    if (frameEpoch < currentEpoch) return FrameDisposition::DropStaleEpoch;
    if (frameEpoch > currentEpoch) return FrameDisposition::NewerEpoch;
    return FrameDisposition::Accept;
}

}  // namespace wpiemu::sdk::proto
