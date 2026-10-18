#pragma once

// Wire-protocol message definitions between the emulator (Qt app) and the HALSim extension
// running inside the robot process. Plain structs: NO serialisation format is implied here
// (the codec in ipc/ decides: flatbuffers, protobuf-lite, hand-rolled binary...).
//
// Versioning: the SDK stays at version 1 until release (ADR 0013), so `kProtocolVersion` is 1. Peers exchange it in
// Hello/HelloAck; a peer whose version is < the other's `kMinCompatibleProtocolVersion` is refused. Codecs
// tolerate absent trailing fields (default value) and ignore bytes they do not know.
//
// Flow (ADR 0001 / 0002), emulator = clock master:
//   ext -> app : Hello (+ proposed watchdog)
//   app -> ext : HelloAck (clock mode, step size, sessionId, runEpoch, negotiated watchdog)
//   ext -> app : DeviceAnnounce*  (any time; re-announce on reconnect)
//   app -> ext : LoopbackSet      (on every mapping change)
//   app -> ext : VisionSetup      (after HelloAck on every connection, and again immediately before the first
//                                  SensorFrame of every new run epoch / after a Reset, so the extension recreates
//                                  its camera topics)
//   app -> ext : DsState, SimControl (any time)
//   ext -> app : SimControlAck    (one per SimControl with a non-zero commandId)
//   per tick   : app -> ext SensorFrame (inputs for tick N + step dt)
//                ext steps HALSIM_StepTiming(dt) then
//                ext -> app ActuatorFrame (outputs observed after tick N, ackTick = N, result)
//   ext -> app : Log*  (robot stdout is captured by the launcher, not here)
//   either     : Goodbye
//
// Lock-step robustness (see docs/ARCHITECTURE.md section 4):
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
//
// Threading: transports deliver messages on their own IO thread; consumers hand them to the
// sim thread via a queue. Messages are value types and thread-safe to move between threads.

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/devices/signal.h"
#include "wpiemu/sdk/vision/camera_frame.h"

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

enum class MessageType : std::uint16_t {
    Hello = 1,
    HelloAck = 2,
    DeviceAnnounce = 3,
    LoopbackSet = 4,
    SensorFrame = 5,
    ActuatorFrame = 6,
    DsState = 7,
    SimControl = 8,
    Log = 9,
    Goodbye = 10,
    SimControlAck = 11,
    VisionSetup = 12,
};

enum class RobotLanguage : std::uint8_t { Unknown = 0, Java = 1, Cpp = 2 };
enum class ClockMode : std::uint8_t {
    Lockstep = 0,   // emulator steps HALSIM time; robot loop runs exactly once per its period (default)
    Realtime = 1,   // HALSIM time resumes; frames are best-effort
};
// What the extension does by itself when the watchdog expires or the transport drops.
enum class WatchdogPolicy : std::uint8_t {
    ResumeRealtime = 0,   // HALSIM_ResumeTiming: the robot program keeps running in real time (default)
    Terminate = 1,        // the extension ends the robot process cleanly
};

// ---- handshake --------------------------------------------------------------------------------
struct Hello {
    std::uint32_t protocolVersion = kProtocolVersion;
    RobotLanguage language = RobotLanguage::Unknown;
    std::int32_t wpilibYear = 0;         // e.g. 2026 (from the HAL/WPILib version the robot runs)
    std::string wpilibVersion;           // free-form, e.g. "2026.2.1"
    std::string extensionVersion;
    std::int64_t pid = 0;                // robot process id, so the launcher can kill it (ADR 0001)
    std::string programName;
    std::vector<std::pair<std::string, std::int32_t>> halPortCounts;  // "pwm" -> 20, ...
    std::uint32_t proposedWatchdogMs = 0;  // extension's minimum acceptable watchdog; 0 = no preference
    bool supportsArmed =
        false;  // Begin/EndLockstep supported; absent => unsupported
};

struct HelloAck {
    bool accepted = true;
    std::string reason;                  // when !accepted
    std::uint32_t protocolVersion = kProtocolVersion;
    ClockMode clockMode = ClockMode::Lockstep;
    std::uint32_t stepUs = 5000;         // must divide the robot period (20 ms) evenly
    std::uint64_t sessionId = 0;
    bool discoveryOnly = false;          // armed start: announce only, no physics, until BeginLockstep
    bool enableL3Hooks = false;          // opt-in CTRE hooks (ADR 0002 L3)
    // ---- absent in a HelloAck that does not send them => these defaults ----
    std::uint64_t runEpoch = kFirstRunEpoch;  // epoch in force for this connection (monotonic across reconnects)
    std::uint32_t watchdogTimeoutMs = kDefaultWatchdogTimeoutMs;  // negotiateWatchdogTimeoutMs(...)
    WatchdogPolicy watchdogPolicy = WatchdogPolicy::ResumeRealtime;
};

// ---- discovery ----------------------------------------------------------------------------------
struct DeviceAnnounce {
    DeviceId id;
    DeviceDescriptor descriptor;
    std::vector<SignalSpec> signalSpecs;     // SignalSample::signal indexes into this
    bool retired = false;                // robot freed the device (descriptor still identifies it)
    // CAN bus label OBSERVED at runtime (CTRE L3 hook); empty = not observed. Never
    // derived from a static SimDeviceRule. A re-announce with a new value updates the hub's device.
    std::string observedBus;
};

// Devices the extension must loop back itself (unmapped: output copied to input, no IPC hop).
// Absence from the set => mapped. Sent whenever mappings change; always the FULL set.
struct LoopbackSet {
    std::vector<DeviceId> loopback;
};

// ---- per tick -------------------------------------------------------------------------------------
// app -> ext: inputs the robot will see during this step.
struct SensorFrame {
    std::uint64_t tick = 0;
    SimTimeUs simTimeUs = 0;             // time AFTER this step completes
    std::uint32_t stepUs = 5000;         // HALSIM_StepTiming delta
    double batteryVoltageV = 12.6;       // written to the roboRIO VIn + every device's supply voltage
    double batteryCurrentA = 0.0;        // informational (roboRIO current)
    bool full = false;                   // true: contains every Input signal; false: only changed ones
    std::vector<SignalSample> samples;
    // Camera frames due this tick (CameraFrame::publishTimeUs <= simTimeUs), see vision/camera_frame.h.
    std::vector<CameraFrame> vision;
    // identity of the run this frame belongs to (see classifyFrame).
    std::uint64_t sessionId = kUnspecified;
    std::uint64_t runEpoch = kUnspecified;
};

// How a tick ended, reported by the extension in ActuatorFrame::result. Appended values need a `default` path.
enum class TickResult : std::uint8_t {
    Completed = 0,       // HALSIM_StepTiming returned and outputs were collected
    RobotStalled = 1,    // the robot did not quiesce within the watchdog; `samples` are empty/stale
    Stopped = 2,         // the extension honoured Stop (or the watchdog policy) and is leaving lock-step
    ProtocolError = 3,   // the SensorFrame was out of order/duplicated/unexpected; nothing was stepped
};

// ext -> app: outputs the robot produced (observed after the step).
struct ActuatorFrame {
    std::uint64_t tick = 0;
    SimTimeUs simTimeUs = 0;
    std::uint64_t robotLoopCount = 0;    // periodic iterations completed (diagnostics)
    bool full = false;                   // see SensorFrame::full
    std::vector<SignalSample> samples;
    // ---- absent in a frame that does not send them => defaults: Completed, ack == tick, unspecified identity ----
    std::uint64_t sessionId = kUnspecified;   // echo of the SensorFrame being answered
    std::uint64_t runEpoch = kUnspecified;    // echo of the SensorFrame being answered
    std::uint64_t ackTick = kNoAckTick;       // tick of the SensorFrame this answers; unset => `tick`
    TickResult result = TickResult::Completed;

    std::uint64_t ackedTick() const { return ackTick != kNoAckTick ? ackTick : tick; }
};

// ---- driver station -------------------------------------------------------------------------------
enum class RobotMode : std::uint8_t { Teleop = 0, Autonomous = 1, Test = 2 };
enum class AllianceStation : std::uint8_t { Red1 = 0, Red2, Red3, Blue1, Blue2, Blue3 };

struct JoystickState {
    bool present = false;
    std::vector<float> axes;             // [-1, 1]
    std::vector<std::int16_t> povs;      // degrees, -1 = released
    std::uint32_t buttons = 0;           // bit i = button i+1
    std::uint8_t buttonCount = 0;
};

struct DsState {
    bool dsAttached = true;
    bool fmsAttached = false;
    bool enabled = false;
    bool eStop = false;
    RobotMode mode = RobotMode::Teleop;
    AllianceStation station = AllianceStation::Red1;
    // Seconds REMAINING in the current period (WPILib's DriverStation.getMatchTime() convention, counting down);
    // -1 = no timed match, consumers use their own integrated clock.
    double matchTimeS = -1.0;
    // FMS "game data" (DriverStation.getGameSpecificMessage()). Empty until the game/FMS has produced it. The runtime
    // overlays IGameState::gameSpecificMessage() on top of whatever the UI requested.
    std::string gameSpecificMessage;
    std::array<JoystickState, 6> joysticks{};
    // identity of the run this state belongs to; a DsState queued before a Reset is dropped after it.
    std::uint64_t sessionId = kUnspecified;
    std::uint64_t runEpoch = kUnspecified;
};

// ---- control --------------------------------------------------------------------------------------
enum class SimCommand : std::uint8_t {
    Pause = 0,
    Resume = 1,
    Step = 2,       // advance `stepCount` ticks then stay paused
    Reset = 3,      // reset physics/game state; robot keeps running (bumps runEpoch, MUST be acked)
    Stop = 4,       // end the session (extension resumes timing and exits cleanly; MUST be acked)
    SetTimeScale = 5,
    // One long-lived robot process. A session whose HelloAck said `discoveryOnly` is ARMED: devices are
    // announced and HAL time runs free. BeginLockstep switches it to lock-step; EndLockstep returns it to armed.
    // Both follow the Reset rules (first message of a NEW epoch, MUST carry a commandId and be acked); the
    // extension answers ControlResult::Rejected when the transition is impossible (already there, program not
    // started, real-time clock preference) and ::Unsupported when the peer does not support it.
    BeginLockstep = 6,
    EndLockstep = 7,
};

// Receiver rules:
//  * commandId != 0: the receiver answers with exactly one SimControlAck carrying the same
//  commandId.
//    Reset and Stop MUST carry a non-zero commandId. IDs increase within a connection, across
//    epochs. The last 64 distinct command results (including rejections) are retained and replayed
//    exactly. Older/evicted IDs are rejected, NEVER re-applied. Reconnect starts fresh command
//    history. Stop terminates the connection: retries are only meaningful while that connection is
//    still live.
//  * runEpoch: for every command except Reset, the sender's epoch in force; a stale one is rejected
//    (ControlResult::StaleEpoch). Reset is the FIRST message of the NEW epoch: its runEpoch is the
//    already-bumped value and must be newer than the receiver's. Stop is honoured regardless of
//    epoch.
struct SimControl {
    SimCommand command = SimCommand::Pause;
    std::uint32_t stepCount = 1;
    double timeScale = 1.0;
    // ----
    std::uint64_t commandId = kUnspecified;   // 0 = fire-and-forget (no ack); sender-unique, increasing
    std::uint64_t sessionId = kUnspecified;
    std::uint64_t runEpoch = kUnspecified;
};

enum class ControlResult : std::uint8_t {
    Applied = 0,
    Rejected = 1,      // invalid in the current state (e.g. Step while running)
    StaleEpoch = 2,    // command carried an older runEpoch; ignored
    Unsupported = 3,   // peer does not implement the command
};

// Receiver -> sender acknowledgement of a SimControl with a non-zero commandId.
struct SimControlAck {
    std::uint64_t commandId = kUnspecified;
    SimCommand command = SimCommand::Pause;
    ControlResult result = ControlResult::Applied;
    std::uint64_t sessionId = kUnspecified;
    std::uint64_t runEpoch = kUnspecified;    // epoch in force AFTER the command (the new one for Reset)
    std::uint64_t tick = 0;                   // last tick the receiver completed
    std::string detail;                       // human readable when result != Applied
};

// ---- misc -----------------------------------------------------------------------------------------
struct Log {
    SimTimeUs simTimeUs = 0;
    LogLevel level = LogLevel::Info;
    std::string source;                  // "extension", "bridge.phoenix6", ...
    std::string message;
};

struct Goodbye {
    std::string reason;
};

using Message = std::variant<Hello, HelloAck, DeviceAnnounce, LoopbackSet, SensorFrame,
                             ActuatorFrame, DsState, SimControl, Log, Goodbye, SimControlAck, VisionSetup>;

// Order of alternatives in `Message` == order of MessageType values minus 1; never reorder.
inline MessageType typeOf(const Message& m) {
    return static_cast<MessageType>(m.index() + 1);
}

}  // namespace wpiemu::sdk::proto
