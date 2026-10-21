#pragma once

// A FIFO between threads: producers never block, a consumer waits with a timeout. Optionally bounded, and
// closeable so that blocked consumers and later producers learn the owner is shutting down. Policy stays with the
// owner: what a full queue means, which items to discard on close, what a timeout means.

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>

namespace wpiemu::sdk {

template <class T>
class BlockingQueue {
public:
    enum class PushResult { Pushed, Full, Closed };
    enum class PopResult { Item, Timeout, Closed };

    static constexpr std::size_t kUnbounded = std::numeric_limits<std::size_t>::max();

    explicit BlockingQueue(std::size_t capacity = kUnbounded) : capacity_(capacity) {}
    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;

    // A closed or full queue rejects `item` and leaves it untouched, so the caller still owns what it could not hand over.
    PushResult push(T&& item) {
        std::lock_guard lock(mutex_);
        if (closed_) return PushResult::Closed;
        if (items_.size() >= capacity_) return PushResult::Full;
        items_.push_back(std::move(item));
        cv_.notify_one();
        return PushResult::Pushed;
    }

    // Waits up to `timeout` (zero polls) for an item and moves it into `out`; the previous value of `out` is destroyed
    // after the queue's lock is released. Closed is reported only once the queue is empty, so a consumer first drains
    // what was queued before close(). A wake() that finds a consumer waiting, or the next pop that would wait, ends it
    // with Timeout.
    PopResult pop(T& out, std::chrono::milliseconds timeout) {
        std::optional<T> taken;
        {
            std::unique_lock lock(mutex_);
            if (items_.empty() && timeout.count() > 0)
                cv_.wait_for(lock, timeout, [this] { return !items_.empty() || woken_ || closed_; });
            if (items_.empty()) {
                woken_ = false;
                return closed_ ? PopResult::Closed : PopResult::Timeout;
            }
            taken.emplace(std::move(items_.front()));
            items_.pop_front();
        }
        out = std::move(*taken);
        return PopResult::Item;
    }

    void wake() {
        std::lock_guard lock(mutex_);
        woken_ = true;
        cv_.notify_all();
    }

    // Idempotent. Rejects later pushes and wakes every waiter; queued items stay available to pop().
    void close() {
        std::lock_guard lock(mutex_);
        closed_ = true;
        cv_.notify_all();
    }

    // close() that also takes the queued items, for an owner that must dispose of them (outside any lock of its own).
    std::deque<T> closeAndDrain() {
        std::deque<T> pending;
        std::lock_guard lock(mutex_);
        closed_ = true;
        pending.swap(items_);
        cv_.notify_all();
        return pending;
    }

    bool closed() const {
        std::lock_guard lock(mutex_);
        return closed_;
    }

    std::size_t size() const {
        std::lock_guard lock(mutex_);
        return items_.size();
    }

private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<T> items_;
    bool woken_ = false;
    bool closed_ = false;
};

}  // namespace wpiemu::sdk
