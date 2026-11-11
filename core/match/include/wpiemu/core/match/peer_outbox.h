#pragma once

// A bounded outbound queue with its own sender thread for ONE peer of the match host. The sim thread only enqueues
// (never blocks, never encodes beyond what the link does on the sender thread); the sender thread calls
// IMessageLink::send, which may block on a slow peer.
//
// Policy (a slow or stuck peer must never stall the sim thread):
//   * unreliable messages (snapshots, latest-wins) are bounded to `unreliableCapacity`; when full the OLDEST queued
//     unreliable message is dropped and counted;
//   * reliable messages are bounded to `reliableCapacity`; when full enqueue() reports Overflow and the host drops the
//     peer;
//   * a send blocked for longer than `stallTimeout` of the injected wall clock makes stalled() true, and the host drops
//     the peer (closing the link unblocks the sender).

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>

#include "wpiemu/core/runtime/message_link.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/stoppable_thread.h"

namespace wpiemu::core::match {

struct OutboxLimits {
    std::size_t reliableCapacity = 32;
    std::size_t unreliableCapacity = 4;
    std::chrono::milliseconds stallTimeout{3000};
};

class PeerOutbox {
public:
    enum class Result { Queued, QueuedDroppedOldest, Overflow, Closed };

    PeerOutbox(std::shared_ptr<runtime::IMessageLink> link, const sdk::IWallClock& clock, OutboxLimits limits);
    ~PeerOutbox();  // closes the link, then joins the sender
    PeerOutbox(const PeerOutbox&) = delete;
    PeerOutbox& operator=(const PeerOutbox&) = delete;

    // Delivery class follows sdk::proto::deliveryOf. `closeAfter` closes the link once this message was sent.
    Result enqueue(sdk::proto::Message message, bool closeAfter = false);
    bool stalled() const;
    bool failed() const { return failed_.load(); }
    std::size_t pending() const;
    // Waits until everything queued was handed to the link (or the sender died). False on timeout.
    bool waitIdle(std::chrono::milliseconds timeout);
    std::uint64_t droppedUnreliable() const { return droppedUnreliable_.load(); }
    std::uint64_t sent() const { return sent_.load(); }

private:
    struct Item {
        sdk::proto::Message message;
        bool reliable = true;
        bool closeAfter = false;
    };
    void run(const sdk::StopToken& stop);

    std::shared_ptr<runtime::IMessageLink> link_;
    const sdk::IWallClock& clock_;
    OutboxLimits limits_;

    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::condition_variable idle_;
    std::deque<Item> queue_;
    bool sending_ = false;
    bool stopped_ = false;

    std::atomic<std::chrono::steady_clock::rep> sendingSince_{0};
    std::atomic<bool> failed_{false};
    std::atomic<std::uint64_t> droppedUnreliable_{0};
    std::atomic<std::uint64_t> sent_{0};
    sdk::StoppableThread thread_;
};

}  // namespace wpiemu::core::match
