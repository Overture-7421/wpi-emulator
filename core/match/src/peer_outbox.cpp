#include "wpiemu/core/match/peer_outbox.h"

#include <algorithm>

namespace wpiemu::core::match {

PeerOutbox::PeerOutbox(std::shared_ptr<runtime::IMessageLink> link, const sdk::IWallClock& clock, OutboxLimits limits)
    : link_(std::move(link)), clock_(clock), limits_(limits) {
    thread_.start([this](const sdk::StopToken& stop) { run(stop); });
}

PeerOutbox::~PeerOutbox() {
    {
        std::lock_guard lock(mutex_);
        stopped_ = true;
    }
    thread_.requestStop();
    wake_.notify_all();
    link_->close();  // unblocks a send in progress
    thread_.join();
}

PeerOutbox::Result PeerOutbox::enqueue(sdk::proto::Message message, bool closeAfter) {
    Item item;
    item.reliable = sdk::proto::deliveryOf(message) == sdk::proto::Delivery::Reliable;
    item.closeAfter = closeAfter;
    item.message = std::move(message);
    Result result = Result::Queued;
    {
        std::lock_guard lock(mutex_);
        if (stopped_ || failed_.load()) return Result::Closed;
        const auto reliableCount = [&] { return static_cast<std::size_t>(std::count_if(queue_.begin(), queue_.end(), [](const Item& i) { return i.reliable; })); };
        if (item.reliable) {
            if (reliableCount() >= limits_.reliableCapacity) return Result::Overflow;
        } else if (queue_.size() - reliableCount() >= limits_.unreliableCapacity) {
            const auto oldest = std::find_if(queue_.begin(), queue_.end(), [](const Item& i) { return !i.reliable; });
            if (oldest != queue_.end()) {
                queue_.erase(oldest);
                ++droppedUnreliable_;
                result = Result::QueuedDroppedOldest;
            }
        }
        queue_.push_back(std::move(item));
    }
    wake_.notify_one();
    return result;
}

bool PeerOutbox::stalled() const {
    const auto since = sendingSince_.load();
    if (since == 0) return false;
    const auto now = clock_.now().time_since_epoch().count();
    return std::chrono::steady_clock::duration(now - since) > limits_.stallTimeout;
}

std::size_t PeerOutbox::pending() const {
    std::lock_guard lock(mutex_);
    return queue_.size() + (sending_ ? 1 : 0);
}

bool PeerOutbox::waitIdle(std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    return idle_.wait_for(lock, timeout, [this] { return (queue_.empty() && !sending_) || stopped_ || failed_.load(); });
}

void PeerOutbox::run(const sdk::StopToken& stop) {
    for (;;) {
        Item item;
        {
            std::unique_lock lock(mutex_);
            wake_.wait(lock, [&] { return !queue_.empty() || stopped_ || stop.stopRequested(); });
            if (stopped_ || stop.stopRequested()) break;
            item = std::move(queue_.front());
            queue_.pop_front();
            sending_ = true;
        }
        sendingSince_ = std::max<std::chrono::steady_clock::rep>(1, clock_.now().time_since_epoch().count());
        const sdk::Status status = link_->send(item.message);
        sendingSince_ = 0;
        if (!status) failed_ = true;
        else ++sent_;
        if (item.closeAfter) link_->close();
        {
            std::lock_guard lock(mutex_);
            sending_ = false;
        }
        idle_.notify_all();
        if (!status) break;
    }
    {
        std::lock_guard lock(mutex_);
        sending_ = false;
    }
    idle_.notify_all();
}

}  // namespace wpiemu::core::match
