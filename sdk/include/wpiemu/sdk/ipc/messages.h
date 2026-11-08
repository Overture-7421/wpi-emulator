#pragma once

// Wire-protocol message definitions between the emulator (Qt app) and the HALSim extension
// running inside the robot process. Plain structs: NO serialisation format is implied here
// (the codec in ipc/ decides: flatbuffers, protobuf-lite, hand-rolled binary...).
//
// This umbrella header includes the topic headers and defines the `Message` variant over them:
//   protocol_rules.h     versioning, run identity (classifyFrame), lock-step and watchdog rules
//   session_messages.h   Hello/HelloAck, DeviceAnnounce/LoopbackSet, Log, Goodbye
//   tick_messages.h      SensorFrame, ActuatorFrame
//   driver_station.h     DsState and the station numbering
//   control_messages.h   SimControl, SimControlAck
//   match/*.h            practice-match and relay control messages (values 13..28; the robot process ignores them)
// (VisionSetup lives with the camera types in vision/camera_frame.h.) Include a topic header alone when that is all you need.
//
// Flow (docs/reference/extension-ipc.md), emulator = clock master:
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
// Threading: transports deliver messages on their own IO thread; consumers hand them to the
// sim thread via a queue. Messages are value types and thread-safe to move between threads.

#include <cstddef>
#include <cstdint>
#include <variant>

#include "wpiemu/sdk/ipc/control_messages.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/ipc/session_messages.h"
#include "wpiemu/sdk/ipc/tick_messages.h"
#include "wpiemu/sdk/match/input_messages.h"
#include "wpiemu/sdk/match/join_messages.h"
#include "wpiemu/sdk/match/match_control.h"
#include "wpiemu/sdk/match/relay_messages.h"
#include "wpiemu/sdk/match/snapshot_messages.h"
#include "wpiemu/sdk/vision/camera_frame.h"

namespace wpiemu::sdk::proto {

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
    // Practice-match protocol (sdk/match/, appended after the HALSim messages; never reorder or renumber).
    // 13-21 travel inside the end-to-end encrypted match transport, 22-28 are the plaintext relay control messages.
    Join = 13,
    JoinAccept = 14,
    JoinReject = 15,
    Roster = 16,
    Leave = 17,
    MatchInputPacket = 18,    // unreliable
    MatchSnapshot = 19,       // unreliable
    MatchSnapshotAck = 20,    // unreliable
    MatchControl = 21,
    RegisterRoom = 22,
    RoomRegistered = 23,
    RoomRejected = 24,
    JoinRoom = 25,
    JoinResult = 26,
    PeerJoined = 27,
    PeerLeft = 28,
};

using Message = std::variant<Hello, HelloAck, DeviceAnnounce, LoopbackSet, SensorFrame, ActuatorFrame, DsState,
                             SimControl, Log, Goodbye, SimControlAck, VisionSetup,
                             // practice match (values 13..28 above)
                             match::Join, match::JoinAccept, match::JoinReject, match::Roster, match::Leave,
                             match::MatchInputPacket, match::MatchSnapshot, match::MatchSnapshotAck,
                             match::MatchControl, match::relay::RegisterRoom, match::relay::RoomRegistered,
                             match::relay::RoomRejected, match::relay::JoinRoom, match::relay::JoinResult,
                             match::relay::PeerJoined, match::relay::PeerLeft>;

// The HALSim extension protocol is the first kExtensionMessageCount alternatives; the robot process ignores the rest.
inline constexpr std::size_t kExtensionMessageCount = 12;
inline constexpr std::uint16_t kLastMessageTypeValue = static_cast<std::uint16_t>(MessageType::PeerLeft);
static_assert(std::variant_size_v<Message> == kLastMessageTypeValue, "one MessageType value per Message alternative");

// Order of alternatives in `Message` == order of MessageType values minus 1; never reorder.
inline MessageType typeOf(const Message& m) {
    return static_cast<MessageType>(m.index() + 1);
}

// True for the practice-match and relay messages (everything after the HALSim extension protocol).
inline bool isMatchMessage(const Message& m) { return m.index() >= kExtensionMessageCount; }

}  // namespace wpiemu::sdk::proto
