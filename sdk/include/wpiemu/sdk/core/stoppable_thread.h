#pragma once

// A worker thread with a stop request it can poll, joined by its owner. std::jthread is not available on every
// supported standard library.
//
// Declare the member after the state its body uses: members are destroyed in reverse order, so the destructor's join
// then runs while that state is alive. An owner that must wake a blocked body (close a socket, wake a queue) does so
// between requestStop() and join(). start(), join() and stop() belong to the owning thread; requestStop() and
// stopRequested() may be called from any thread, including the body.

#include <atomic>
#include <thread>
#include <utility>

namespace wpiemu::sdk {

class StopToken {
public:
    bool stopRequested() const noexcept { return flag_->load(); }

private:
    friend class StoppableThread;
    explicit StopToken(const std::atomic<bool>& flag) : flag_(&flag) {}
    const std::atomic<bool>* flag_;
};

class StoppableThread {
public:
    StoppableThread() = default;
    ~StoppableThread() { stop(); }
    StoppableThread(const StoppableThread&) = delete;
    StoppableThread& operator=(const StoppableThread&) = delete;

    // Runs body(StopToken) on a new thread; a stop requested earlier is still pending, so the body sees it at once.
    // Does nothing and returns false while an earlier run has not been joined.
    template <class Body>
    bool start(Body&& body) {
        if (thread_.joinable()) return false;
        thread_ = std::thread([this, run = std::forward<Body>(body)]() mutable { run(StopToken(stopRequested_)); });
        return true;
    }

    void requestStop() noexcept { stopRequested_ = true; }
    // For an owner that restarts a joined thread.
    void resetStop() noexcept { stopRequested_ = false; }
    bool stopRequested() const noexcept { return stopRequested_; }

    bool joinable() const noexcept { return thread_.joinable(); }
    // Idempotent; returns once the body has returned.
    void join() {
        if (thread_.joinable()) thread_.join();
    }
    void stop() {
        requestStop();
        join();
    }

private:
    std::atomic<bool> stopRequested_{false};
    std::thread thread_;
};

}  // namespace wpiemu::sdk
