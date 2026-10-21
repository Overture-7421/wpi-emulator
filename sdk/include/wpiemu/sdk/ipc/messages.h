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

#include <cstdint>
#include <variant>

#include "wpiemu/sdk/ipc/control_messages.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/ipc/session_messages.h"
#include "wpiemu/sdk/ipc/tick_messages.h"
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
};

using Message = std::variant<Hello, HelloAck, DeviceAnnounce, LoopbackSet, SensorFrame,
                             ActuatorFrame, DsState, SimControl, Log, Goodbye, SimControlAck, VisionSetup>;

// Order of alternatives in `Message` == order of MessageType values minus 1; never reorder.
inline MessageType typeOf(const Message& m) {
    return static_cast<MessageType>(m.index() + 1);
}

}  // namespace wpiemu::sdk::proto
