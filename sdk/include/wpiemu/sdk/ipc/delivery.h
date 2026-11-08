#pragma once

// Delivery class of a frame: whether a transport may lose or reorder it.
//
//   Reliable    ordered and never dropped (handshakes, rosters, lifecycle and control messages).
//   Unreliable  latest-wins streams (inputs, snapshots): a transport may drop, duplicate or reorder them, so a receiver
//               must tolerate gaps and stale frames (sequence numbers in the message, see protocol_rules.h).
//
// Transports without a lossy channel (pipes, in-memory) treat both the same: every frame is delivered in order.

#include <cstdint>

#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::sdk::proto {

enum class Delivery : std::uint8_t { Reliable, Unreliable };

// The delivery class a message type is sent with. Every HALSim extension message is reliable: it rides a lock-step
// stream where a lost frame would stall the tick. Message types that are sent best-effort (match inputs and
// snapshots) get their `case` returning Unreliable here, next to the type they are added to in messages.h.
inline Delivery deliveryOf(const Message& message) {
    switch (typeOf(message)) {
        case MessageType::Hello:
        case MessageType::HelloAck:
        case MessageType::DeviceAnnounce:
        case MessageType::LoopbackSet:
        case MessageType::SensorFrame:
        case MessageType::ActuatorFrame:
        case MessageType::DsState:
        case MessageType::SimControl:
        case MessageType::Log:
        case MessageType::Goodbye:
        case MessageType::SimControlAck:
        case MessageType::VisionSetup: return Delivery::Reliable;
    }
    return Delivery::Reliable;
}

}  // namespace wpiemu::sdk::proto
