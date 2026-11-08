#pragma once

// Shared helpers for the ENet loopback tests: byte/text conversion, deadline-bounded receives, a manual wall clock and a
// connected server/client/peer triple.

#include <doctest/doctest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "wpiemu/net/enet_transport.h"
#include "wpiemu/sdk/core/clock.h"

namespace wpiemu::net::test {

using namespace std::chrono_literals;
using Bytes = std::vector<std::byte>;
using sdk::ErrorCode;
using sdk::proto::ITransport;

inline constexpr auto kLong = 5000ms;

inline Bytes bytesOf(const std::string& text) {
    Bytes out(text.size());
    std::memcpy(out.data(), text.data(), text.size());
    return out;
}

inline std::string textOf(const Bytes& bytes) {
    return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

inline Bytes patterned(std::size_t size, std::uint32_t seed) {
    Bytes out(size);
    std::mt19937 rng(seed);
    for (auto& b : out) b = static_cast<std::byte>(rng() & 0xFF);
    return out;
}

// The next frame, or nullopt on timeout or when the transport reports an error (stored in `error` when given).
inline std::optional<Bytes> receiveFrame(ITransport& transport, std::chrono::milliseconds timeout = kLong,
                                         sdk::Error* error = nullptr) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    Bytes frame;
    for (;;) {
        auto got = transport.receive(frame, std::min<std::chrono::milliseconds>(50ms, sdk::proto::timeLeft(deadline)));
        if (!got) {
            if (error != nullptr) *error = got.error();
            return std::nullopt;
        }
        if (got.value()) return frame;
        if (std::chrono::steady_clock::now() >= deadline) return std::nullopt;
    }
}

inline sdk::Error receiveError(ITransport& transport, std::chrono::milliseconds timeout = kLong) {
    sdk::Error error;
    error.code = ErrorCode::Internal;
    Bytes frame;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto got = transport.receive(frame, 20ms);
        if (!got) return got.error();
    }
    return error;
}

struct ManualClock final : sdk::IWallClock {
    std::atomic<std::int64_t> ns{1'000'000'000'000};
    std::chrono::steady_clock::time_point now() const override {
        return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns.load()));
    }
    void advance(std::chrono::milliseconds by) { ns += std::chrono::duration_cast<std::chrono::nanoseconds>(by).count(); }
};

struct Pair {
    std::unique_ptr<EnetServerTransport> server;
    std::unique_ptr<ITransport> client;  // the connecting side
    std::unique_ptr<ITransport> peer;    // the server's end of the same link
};

inline EnetServerOptions loopbackServer(const std::string& host = "127.0.0.1") {
    EnetServerOptions options;
    options.bindHost = host;
    return options;
}

inline std::unique_ptr<EnetServerTransport> listening(EnetServerOptions options) {
    auto server = std::make_unique<EnetServerTransport>(std::move(options));
    const auto status = server->listen();
    REQUIRE_MESSAGE(status.ok(), (status.ok() ? "" : status.error().message));
    return server;
}

inline std::unique_ptr<ITransport> connectTo(const std::string& endpoint, EnetClientOptions options = {},
                                             std::chrono::milliseconds timeout = kLong) {
    EnetClientTransport client(std::move(options));
    auto connected = client.connect(endpoint, timeout);
    REQUIRE_MESSAGE(connected.ok(), (connected.ok() ? "" : connected.error().message));
    return std::move(connected).value();
}

inline std::unique_ptr<ITransport> acceptOne(EnetServerTransport& server, std::chrono::milliseconds timeout = kLong) {
    auto accepted = server.accept(timeout);
    REQUIRE(accepted.ok());
    return std::move(accepted).value();
}

inline Pair makePair(EnetServerOptions serverOptions = loopbackServer(), EnetClientOptions clientOptions = {}) {
    Pair pair;
    pair.server = listening(std::move(serverOptions));
    pair.client = connectTo(pair.server->endpoint(), std::move(clientOptions));
    pair.peer = acceptOne(*pair.server);
    REQUIRE(pair.peer != nullptr);
    return pair;
}

}  // namespace wpiemu::net::test
