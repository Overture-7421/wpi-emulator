#pragma once

// Bounded polling for tests that wait on another thread or process.

#include <chrono>
#include <thread>
#include <utility>

namespace wpiemu::sdk::testing {

// Polls `done` on the steady clock until it returns true or `timeout` elapses, sleeping `pollInterval`
// between polls (not at all when it is zero). `pump` runs after each unsuccessful poll, before the
// sleep, for waits that must drive something themselves, such as an event loop. `done` is evaluated once
// more after the deadline, so a stall just past the boundary cannot fail a condition that already holds;
// the result of the last evaluation is returned.
template <class Done, class Pump>
bool waitUntil(Done&& done, std::chrono::milliseconds timeout, std::chrono::milliseconds pollInterval,
               Pump&& pump) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (done()) return true;
        pump();
        if (pollInterval > std::chrono::milliseconds::zero()) std::this_thread::sleep_for(pollInterval);
    }
    return done();
}

template <class Done>
bool waitUntil(Done&& done, std::chrono::milliseconds timeout,
               std::chrono::milliseconds pollInterval = std::chrono::milliseconds{5}) {
    return waitUntil(std::forward<Done>(done), timeout, pollInterval, [] {});
}

}  // namespace wpiemu::sdk::testing
