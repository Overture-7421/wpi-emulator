#pragma once

// Internal (not installed): the state shared between one ENet peer, owned by a pump thread, and the ITransport object a
// consumer holds for it. No ENet types appear here. The consumer side (send, receive, requestClose) may be used from any
// thread; the pump side is called only from the pump thread. The link is held by shared_ptr by both sides, so it outlives
// whichever of them goes first.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/blocking_queue.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::net::detail {

enum class LinkState : std::uint8_t { Connecting, Connected, Closed };

// Carried as the ENet disconnect data so the far end can say why it was dropped.
enum class DisconnectReason : std::uint32_t { Normal = 0, ServerFull = 1, Unauthenticated = 2, Backlog = 3 };

struct LinkLimits {
    std::size_t maxReliableFrameBytes = 0;
    std::size_t maxUnreliableFrameBytes = 0;
    std::size_t maxQueuedOutboundBytes = 0;
    std::size_t maxQueuedInboundFrames = 0;
};

struct OutboundFrame {
    std::vector<std::byte> bytes;
    bool reliable = true;
};

class PeerLink {
public:
    PeerLink(LinkLimits limits, std::string description);

    // ---- consumer side
    sdk::Status send(std::span<const std::byte> frame, bool reliable);
    sdk::Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout);
    // Idempotent. Discards frames not yet received, wakes a blocked receive; the pump still flushes frames already queued
    // for sending and then disconnects the peer politely.
    void requestClose();
    bool closeRequested() const { return closeRequested_.load(); }
    bool connected() const { return state_.load() == LinkState::Connected && !closeRequested_.load(); }
    sdk::proto::LinkStats stats() const;
    const std::string& description() const { return description_; }
    // Blocks until the link leaves Connecting or `deadline` passes; returns the state seen.
    LinkState waitWhileConnecting(std::chrono::steady_clock::time_point deadline);
    DisconnectReason remoteReason() const { return static_cast<DisconnectReason>(reason_.load()); }

    // ---- pump side
    std::deque<OutboundFrame> takeOutbound();
    sdk::BlockingQueue<std::vector<std::byte>>::PushResult deliver(std::vector<std::byte>&& frame);
    void markConnected();
    // The peer is gone: buffered inbound frames stay readable, then receive() reports Disconnected.
    void markClosed(DisconnectReason reason);
    void countDropped() { dropped_.fetch_add(1); }
    void countBytesIn(std::uint64_t n) { bytesIn_.fetch_add(n); }
    void countBytesOut(std::uint64_t n) { bytesOut_.fetch_add(n); }
    void setLinkQuality(std::uint32_t rttMs, std::uint32_t lossPermille);

private:
    const LinkLimits limits_;
    const std::string description_;
    sdk::BlockingQueue<std::vector<std::byte>> inbound_;

    std::mutex outMutex_;
    std::deque<OutboundFrame> outbound_;
    std::size_t outboundBytes_ = 0;

    std::atomic<LinkState> state_{LinkState::Connecting};
    std::atomic<bool> closeRequested_{false};
    std::atomic<std::uint32_t> reason_{0};
    std::atomic<std::uint32_t> rttMs_{0};
    std::atomic<std::uint32_t> lossPermille_{0};
    std::atomic<std::uint64_t> bytesIn_{0};
    std::atomic<std::uint64_t> bytesOut_{0};
    std::atomic<std::uint64_t> dropped_{0};

    std::mutex stateMutex_;
    std::condition_variable stateChanged_;
};

}  // namespace wpiemu::net::detail
