#pragma once

// Internal (not installed): what the relay host and client transports share: sending and decoding the plaintext relay
// control messages, the 2-byte peer id envelope, the error mapping of the relay's refusals and a bounded frame inbox.

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/net/control_link.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::net::detail {

// Encodes `message` with the shared binary codec and sends it on the control channel.
sdk::Status sendRelayMessage(IControlLink& link, const sdk::proto::Message& message);
// ProtocolError for anything that does not decode; the caller decides what a stray message means.
sdk::Result<sdk::proto::Message> decodeRelayMessage(std::span<const std::byte> frame);

// Appends `id` (little endian) and `payload` to `out`.
void writeEnvelope(std::vector<std::byte>& out, sdk::match::relay::RelayPeerId id, std::span<const std::byte> payload);
// Splits an enveloped frame; false when it is shorter than the envelope plus one payload byte.
bool readEnvelope(std::span<const std::byte> frame, sdk::match::relay::RelayPeerId& id, std::span<const std::byte>& payload);

sdk::Error joinFailureError(sdk::match::relay::JoinRoomFailure reason);
sdk::Error registerRejectError(sdk::match::relay::RoomRejectReason reason);

// A bounded FIFO of received frames for one consumer. Producers never block. After close(discard = false) the consumer
// first drains what was queued, then sees Disconnected; close(discard = true) drops what was queued.
class FrameInbox {
public:
    enum class Push { Pushed, Full, Closed };

    FrameInbox(std::size_t maxFrames, std::size_t maxBytes) : maxFrames_(maxFrames), maxBytes_(maxBytes) {}

    Push push(std::vector<std::byte>&& frame);
    // True and fills `frame`, false on timeout, Disconnected once closed and empty.
    sdk::Result<bool> pop(std::vector<std::byte>& frame, std::chrono::milliseconds timeout);
    void close(bool discard);
    std::size_t size() const;

private:
    const std::size_t maxFrames_;
    const std::size_t maxBytes_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::vector<std::byte>> frames_;
    std::size_t bytes_ = 0;
    bool closed_ = false;
};

}  // namespace wpiemu::net::detail
