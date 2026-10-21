#pragma once

// Fails a threaded test that deadlocks instead of hanging the whole run: aborts the process when the guard outlives
// its deadline.

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>

namespace wpiemu::sdk::testing {

class DeadlockDeadline {
public:
    explicit DeadlockDeadline(std::chrono::milliseconds deadline = std::chrono::seconds(10))
        : thread_([this, deadline] {
              std::unique_lock lock(mutex_);
              if (!cv_.wait_for(lock, deadline, [this] { return done_; })) std::abort();
          }) {}
    ~DeadlockDeadline() {
        {
            std::lock_guard lock(mutex_);
            done_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }
    DeadlockDeadline(const DeadlockDeadline&) = delete;
    DeadlockDeadline& operator=(const DeadlockDeadline&) = delete;

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    bool done_ = false;
    std::thread thread_;
};

}  // namespace wpiemu::sdk::testing
