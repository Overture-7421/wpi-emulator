#include "enet_link.h"

#include <utility>

namespace wpiemu::net::detail {

using sdk::ErrorCode;
using sdk::makeError;

PeerLink::PeerLink(LinkLimits limits, std::string description)
    : limits_(limits), description_(std::move(description)), inbound_(limits.maxQueuedInboundFrames) {}

sdk::Status PeerLink::send(std::span<const std::byte> frame, bool reliable) {
    if (closeRequested_.load() || state_.load() == LinkState::Closed)
        return makeError(ErrorCode::Disconnected, "transport closed: " + description_);
    if (frame.empty())  // ENet's range coder reads past an empty buffer, so no empty packet is ever handed to it
        return makeError(ErrorCode::InvalidArgument, "empty frames are not supported");
    const std::size_t limit = reliable ? limits_.maxReliableFrameBytes : limits_.maxUnreliableFrameBytes;
    if (frame.size() > limit)
        return makeError(ErrorCode::InvalidArgument,
                         std::string(reliable ? "reliable" : "unreliable") + " frame of " + std::to_string(frame.size()) +
                             " bytes exceeds the " + std::to_string(limit) + " byte limit");
    std::lock_guard lock(outMutex_);
    if (outboundBytes_ + frame.size() > limits_.maxQueuedOutboundBytes) {
        if (!reliable) {
            dropped_.fetch_add(1);
            return sdk::okStatus();
        }
        return makeError(ErrorCode::IoError, "send backlog full: " + description_);
    }
    outbound_.push_back({std::vector<std::byte>(frame.begin(), frame.end()), reliable});
    outboundBytes_ += frame.size();
    return sdk::okStatus();
}

sdk::Result<bool> PeerLink::receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) {
    switch (inbound_.pop(frame, timeout)) {
        case sdk::BlockingQueue<std::vector<std::byte>>::PopResult::Item: return true;
        case sdk::BlockingQueue<std::vector<std::byte>>::PopResult::Timeout: return false;
        case sdk::BlockingQueue<std::vector<std::byte>>::PopResult::Closed: break;
    }
    return makeError(ErrorCode::Disconnected, "peer disconnected: " + description_);
}

void PeerLink::requestClose() {
    closeRequested_.store(true);
    (void)inbound_.closeAndDrain();
}

sdk::proto::LinkStats PeerLink::stats() const {
    sdk::proto::LinkStats out;
    out.rttMs = rttMs_.load();
    out.lossPermille = lossPermille_.load();
    out.bytesIn = bytesIn_.load();
    out.bytesOut = bytesOut_.load();
    out.droppedFrames = dropped_.load();
    return out;
}

LinkState PeerLink::waitWhileConnecting(std::chrono::steady_clock::time_point deadline) {
    std::unique_lock lock(stateMutex_);
    stateChanged_.wait_until(lock, deadline, [this] { return state_.load() != LinkState::Connecting; });
    return state_.load();
}

std::deque<OutboundFrame> PeerLink::takeOutbound() {
    std::deque<OutboundFrame> taken;
    std::lock_guard lock(outMutex_);
    taken.swap(outbound_);
    outboundBytes_ = 0;
    return taken;
}

sdk::BlockingQueue<std::vector<std::byte>>::PushResult PeerLink::deliver(std::vector<std::byte>&& frame) {
    return inbound_.push(std::move(frame));
}

void PeerLink::markConnected() {
    {
        std::lock_guard lock(stateMutex_);
        LinkState expected = LinkState::Connecting;
        state_.compare_exchange_strong(expected, LinkState::Connected);
    }
    stateChanged_.notify_all();
}

void PeerLink::markClosed(DisconnectReason reason) {
    reason_.store(static_cast<std::uint32_t>(reason));
    {
        std::lock_guard lock(stateMutex_);
        state_.store(LinkState::Closed);
    }
    stateChanged_.notify_all();
    inbound_.close();
    (void)takeOutbound();
}

void PeerLink::setLinkQuality(std::uint32_t rttMs, std::uint32_t lossPermille) {
    rttMs_.store(rttMs);
    lossPermille_.store(lossPermille);
}

}  // namespace wpiemu::net::detail
