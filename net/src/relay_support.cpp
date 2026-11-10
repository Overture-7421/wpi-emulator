#include "relay_support.h"

#include <algorithm>
#include <utility>

#include "wpiemu/ipc/binary_codec.h"

namespace wpiemu::net::detail {

using sdk::ErrorCode;
namespace relay = sdk::match::relay;

namespace {

const ipc::BinaryMessageCodec& codec() {
    static const ipc::BinaryMessageCodec instance(kMaxControlFrameBytes);
    return instance;
}

}  // namespace

sdk::Status sendRelayMessage(IControlLink& link, const sdk::proto::Message& message) {
    std::vector<std::byte> frame;
    if (sdk::Status encoded = codec().encode(message, frame); !encoded) return encoded;
    return link.sendControl(frame);
}

sdk::Result<sdk::proto::Message> decodeRelayMessage(std::span<const std::byte> frame) { return codec().decode(frame); }

void writeEnvelope(std::vector<std::byte>& out, relay::RelayPeerId id, std::span<const std::byte> payload) {
    out.clear();
    out.reserve(relay::kRelayEnvelopeBytes + payload.size());
    out.push_back(static_cast<std::byte>(id & 0xFF));
    out.push_back(static_cast<std::byte>(id >> 8));
    out.insert(out.end(), payload.begin(), payload.end());
}

bool readEnvelope(std::span<const std::byte> frame, relay::RelayPeerId& id, std::span<const std::byte>& payload) {
    if (frame.size() <= relay::kRelayEnvelopeBytes) return false;
    id = static_cast<relay::RelayPeerId>(static_cast<unsigned>(frame[0]) | (static_cast<unsigned>(frame[1]) << 8));
    payload = frame.subspan(relay::kRelayEnvelopeBytes);
    return true;
}

sdk::Error joinFailureError(relay::JoinRoomFailure reason) {
    switch (reason) {
        case relay::JoinRoomFailure::NoSuchRoom: return {ErrorCode::NotFound, "relay: no such room (check the invite)"};
        case relay::JoinRoomFailure::RoomFull: return {ErrorCode::OutOfRange, "relay: the room is full"};
        case relay::JoinRoomFailure::BadVersion: return {ErrorCode::VersionMismatch, "relay: protocol version not supported"};
        case relay::JoinRoomFailure::LockedOut:
            return {ErrorCode::InvalidState, "relay: too many failed joins from this address, locked out"};
        case relay::JoinRoomFailure::HostAbsent: return {ErrorCode::Disconnected, "relay: the host is not connected to the room"};
        case relay::JoinRoomFailure::None: break;
    }
    return {ErrorCode::ProtocolError, "relay: join refused without a reason"};
}

sdk::Error registerRejectError(relay::RoomRejectReason reason) {
    switch (reason) {
        case relay::RoomRejectReason::RoomTaken: return {ErrorCode::AlreadyExists, "relay: the room id is already taken"};
        case relay::RoomRejectReason::BadAuth: return {ErrorCode::InvalidArgument, "relay: bad host authentication"};
        case relay::RoomRejectReason::RelayFull: return {ErrorCode::OutOfRange, "relay: the relay is full"};
        case relay::RoomRejectReason::BadVersion:
            return {ErrorCode::VersionMismatch, "relay: protocol version not supported"};
    }
    return {ErrorCode::ProtocolError, "relay: registration refused"};
}

FrameInbox::Push FrameInbox::push(std::vector<std::byte>&& frame) {
    std::lock_guard lock(mutex_);
    if (closed_) return Push::Closed;
    if (frames_.size() >= maxFrames_ || bytes_ + frame.size() > maxBytes_) return Push::Full;
    bytes_ += frame.size();
    frames_.push_back(std::move(frame));
    cv_.notify_one();
    return Push::Pushed;
}

sdk::Result<bool> FrameInbox::pop(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    if (frames_.empty() && !closed_ && timeout.count() > 0)
        cv_.wait_for(lock, timeout, [this] { return !frames_.empty() || closed_; });
    if (frames_.empty()) {
        if (closed_) return sdk::makeError(ErrorCode::Disconnected, "relay peer disconnected");
        return false;
    }
    std::vector<std::byte> taken = std::move(frames_.front());
    frames_.pop_front();
    bytes_ -= taken.size();
    lock.unlock();
    frame = std::move(taken);
    return true;
}

void FrameInbox::close(bool discard) {
    std::deque<std::vector<std::byte>> dropped;
    {
        std::lock_guard lock(mutex_);
        closed_ = true;
        if (discard) {
            dropped.swap(frames_);
            bytes_ = 0;
        }
    }
    cv_.notify_all();
}

std::size_t FrameInbox::size() const {
    std::lock_guard lock(mutex_);
    return frames_.size();
}

}  // namespace wpiemu::net::detail
