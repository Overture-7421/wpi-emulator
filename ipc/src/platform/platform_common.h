#pragma once

// Internal helpers shared by both platform transports. Syscalls, security and retry policy stay in the platform files.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::ipc::platform {

// How often a client retries while the server is not listening yet, and how often close() re-checks that no read/write is
// still running on the stream. Neither is a timeout: the connect deadline is the caller's.
inline constexpr std::chrono::milliseconds kConnectRetryInterval{5};
inline constexpr std::chrono::microseconds kCloseSpinInterval{50};

// Milliseconds left until `deadline`, clamped to what a poll()/WaitForMultipleObjects() timeout can carry (INT32_MAX).
inline long long boundedWaitMs(std::chrono::steady_clock::time_point deadline) {
    return std::min<long long>(sdk::proto::timeLeft(deadline).count(), 0x7FFFFFFF);
}

// Counts the read()/write() calls running on a stream so close() can wait for them to leave before it releases the handle.
class Inflight {
public:
    explicit Inflight(std::atomic<int>& counter) : counter_(counter) { ++counter_; }
    ~Inflight() { --counter_; }
    Inflight(const Inflight&) = delete;
    Inflight& operator=(const Inflight&) = delete;

    static void waitUntilIdle(const std::atomic<int>& counter) {
        while (counter.load() != 0) std::this_thread::sleep_for(kCloseSpinInterval);
    }

private:
    std::atomic<int>& counter_;
};

// `endpoint` parsed, and checked to be of the `expected` kind ("named-pipe", "unix-socket" in the message) for this client.
inline sdk::Result<ConnectionEndpoint> parseEndpointOfKind(const std::string& endpoint, EndpointKind expected, const char* kindName) {
    sdk::Result<ConnectionEndpoint> parsed = ConnectionEndpoint::parse(endpoint);
    if (!parsed) return parsed.error();
    if (parsed.value().kind() != expected)
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, std::string("not a ") + kindName + " endpoint: " + endpoint);
    return parsed;
}

inline sdk::Error connectTimedOut(const std::string& endpoint) {
    return sdk::makeError(sdk::ErrorCode::Timeout, "timed out connecting to " + endpoint);
}

}  // namespace wpiemu::ipc::platform
