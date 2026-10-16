#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::sdk::testing {

namespace detail {

struct InMemoryConnection {
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<std::vector<std::byte>> incoming[2];
    bool closed[2] = {false, false};
};

class InMemoryTransport final : public proto::ITransport {
public:
    InMemoryTransport(std::shared_ptr<InMemoryConnection> connection, int side)
        : connection_(std::move(connection)), side_(side) {}

    ~InMemoryTransport() override { close(); }

    Status send(std::span<const std::byte> frame) override {
        std::lock_guard lock(connection_->mutex);
        if (connection_->closed[side_] || connection_->closed[1 - side_])
            return makeError(ErrorCode::Disconnected, "in-memory peer is closed");
        connection_->incoming[1 - side_].emplace_back(frame.begin(), frame.end());
        connection_->cv.notify_all();
        return okStatus();
    }

    Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        std::unique_lock lock(connection_->mutex);
        const auto ready = [&] {
            return connection_->closed[side_] || !connection_->incoming[side_].empty() ||
                   connection_->closed[1 - side_];
        };
        if (!ready() && !connection_->cv.wait_for(lock, timeout, ready)) return false;
        if (connection_->closed[side_])
            return makeError(ErrorCode::Disconnected, "in-memory transport is closed");
        auto& queue = connection_->incoming[side_];
        if (!queue.empty()) {
            frame = std::move(queue.front());
            queue.pop_front();
            return true;
        }
        return makeError(ErrorCode::Disconnected, "in-memory peer is closed");
    }

    void close() override {
        {
            std::lock_guard lock(connection_->mutex);
            connection_->closed[side_] = true;
        }
        connection_->cv.notify_all();
    }

    bool connected() const override {
        std::lock_guard lock(connection_->mutex);
        return !connection_->closed[side_] && !connection_->closed[1 - side_];
    }

    std::string describe() const override { return "in-memory"; }

private:
    std::shared_ptr<InMemoryConnection> connection_;
    int side_;
};

struct MemoryCodecState {
    std::mutex mutex;
    std::uint64_t nextId = 1;
    std::map<std::uint64_t, proto::Message> messages;
};

}  // namespace detail

using InMemoryTransport = detail::InMemoryTransport;

struct InMemoryTransportPair {
    std::unique_ptr<InMemoryTransport> a;
    std::unique_ptr<InMemoryTransport> b;
};

inline InMemoryTransportPair makeInMemoryTransportPair() {
    auto connection = std::make_shared<detail::InMemoryConnection>();
    return {std::make_unique<detail::InMemoryTransport>(connection, 0),
            std::make_unique<detail::InMemoryTransport>(std::move(connection), 1)};
}

class MemoryMessageCodec final : public proto::IMessageCodec {
public:
    MemoryMessageCodec() : state_(std::make_shared<detail::MemoryCodecState>()) {}

    std::string_view name() const override { return "in-memory"; }

    Status encode(const proto::Message& message, std::vector<std::byte>& out) const override {
        std::lock_guard lock(state_->mutex);
        const std::uint64_t id = state_->nextId++;
        state_->messages.emplace(id, message);
        out.resize(sizeof(id));
        for (std::size_t i = 0; i < sizeof(id); ++i)
            out[i] = static_cast<std::byte>((id >> (i * 8)) & 0xff);
        return okStatus();
    }

    Result<proto::Message> decode(std::span<const std::byte> frame) const override {
        if (frame.size() != sizeof(std::uint64_t))
            return makeError(ErrorCode::ProtocolError, "invalid in-memory message frame");
        std::uint64_t id = 0;
        for (std::size_t i = 0; i < sizeof(id); ++i)
            id |= static_cast<std::uint64_t>(std::to_integer<unsigned char>(frame[i])) << (i * 8);
        std::lock_guard lock(state_->mutex);
        const auto it = state_->messages.find(id);
        if (it == state_->messages.end())
            return makeError(ErrorCode::ProtocolError, "unknown in-memory message frame");
        proto::Message message = std::move(it->second);
        state_->messages.erase(it);
        return message;
    }

private:
    std::shared_ptr<detail::MemoryCodecState> state_;
};

}  // namespace wpiemu::sdk::testing
