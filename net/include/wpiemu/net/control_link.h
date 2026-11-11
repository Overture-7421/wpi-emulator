#pragma once

// What a net-layer peer transport offers beyond sdk::proto::ITransport, for code that sits BETWEEN peers (the relay) or
// speaks the relay protocol (the relay host/client transports and the probe). A game match never needs it: MatchHost and
// MatchClientSession only see ITransport.
//
//   control channel   a separate reliable channel (relay_messages.h: kRelayChannelControl = 0) for the plaintext relay
//                     control messages. ITransport::receive NEVER returns a control frame, and control frames never mix
//                     with game frames.
//   tagged game       the game channels (reliable = 1, unreliable-sequenced = 2) with the delivery class a frame arrived
//                     on, so a forwarder can relay it on the same class. ITransport::receive and receiveGame read the SAME
//                     inbound game queue: use one of them on a peer, not both.
//
// Send game frames with ITransport::send / sendUnreliable as usual. Threading follows the ITransport contract: one thread
// may call the receive methods while another sends; close() may come from any thread and wakes blocked receives.
// Implemented by the ENet peer transports (enet_transport.h) and by in-memory fakes in tests.

#include <chrono>
#include <cstddef>
#include <span>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/delivery.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::net {

class IControlLink {
    WPIEMU_INTERFACE(IControlLink)
public:
    // Reliable, ordered. A frame above kMaxControlFrameBytes is refused with InvalidArgument; empty frames are refused.
    virtual sdk::Status sendControl(std::span<const std::byte> frame) = 0;
    // Blocks up to `timeout`. True and fills `frame` when a control frame arrived, false on timeout, Disconnected once the
    // peer is gone and everything buffered was read.
    virtual sdk::Result<bool> receiveControl(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) = 0;
    // Like ITransport::receive, plus the delivery class the frame arrived on.
    virtual sdk::Result<bool> receiveGame(std::vector<std::byte>& frame, sdk::proto::Delivery& delivery,
                                          std::chrono::milliseconds timeout) = 0;
};

// Control frames are small protocol messages (RegisterRoom, JoinRoom, ...); a larger frame is a protocol error.
inline constexpr std::size_t kMaxControlFrameBytes = 4096;

// The control link of `transport`, or nullptr when it has none (not a net-layer peer transport).
inline IControlLink* controlLinkOf(sdk::proto::ITransport& transport) { return dynamic_cast<IControlLink*>(&transport); }

}  // namespace wpiemu::net
