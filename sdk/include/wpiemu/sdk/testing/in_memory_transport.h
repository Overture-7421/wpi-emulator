#pragma once

// In-process transport pair + a codec that "serialises" by parking the message in a shared
// table (the frame is just a ticket id). Lets IPC consumers (extension side, app side) be tested
// against each other with no sockets and no real wire format.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>

#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::sdk::testing {

class MemoryMessageCodec final : public proto::IMessageCodec {
public:
    std::string_view name() const override { return "memory"; }

    Status encode(const proto::Message& message, std::vector<std::byte>& out) const override {
        std::lock_guard lock(store_->mutex);
        const std::uint64_t ticket = store_->next++;
        store_->messages.emplace(ticket, message);
        const auto* p = reinterpret_cast<const std::byte*>(&ticket);
        out.insert(out.end(), p, p + sizeof(ticket));
        return okStatus();
    }
    Result<proto::Message> decode(std::span<const std::byte> frame) const override {
        if (frame.size() != sizeof(std::uint64_t)) return makeError(ErrorCode::ProtocolError, "bad frame size");
        std::uint64_t ticket = 0;
        std::memcpy(&ticket, frame.data(), sizeof(ticket));
        std::lock_guard lock(store_->mutex);
        const auto it = store_->messages.find(ticket);
        if (it == store_->messages.end()) return makeError(ErrorCode::ProtocolError, "unknown ticket");
        proto::Message m = std::move(it->second);
        store_->messages.erase(it);
        return m;
    }

private:
    struct Store {
        std::mutex mutex;
        std::uint64_t next = 1;
        std::map<std::uint64_t, proto::Message> messages;
    };
    std::shared_ptr<Store> store_ = std::make_shared<Store>();
};

class InMemoryTransport final : public proto::ITransport {
public:
    struct Pipe {
        std::mutex mutex;
        std::condition_variable cv;
        std::deque<std::vector<std::byte>> frames;
        std::atomic<bool> closed{false};
    };

    InMemoryTransport(std::shared_ptr<Pipe> in, std::shared_ptr<Pipe> out, std::string label)
        : in_(std::move(in)), out_(std::move(out)), label_(std::move(label)) {}

    Status send(std::span<const std::byte> frame) override {
        std::lock_guard lock(out_->mutex);
        if (out_->closed || in_->closed) return makeError(ErrorCode::Disconnected, "peer closed");
        out_->frames.emplace_back(frame.begin(), frame.end());
        out_->cv.notify_one();
        return okStatus();
    }
    Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        std::unique_lock lock(in_->mutex);
        in_->cv.wait_for(lock, timeout, [&] { return !in_->frames.empty() || in_->closed; });
        if (!in_->frames.empty()) {
            frame = std::move(in_->frames.front());
            in_->frames.pop_front();
            return true;
        }
        if (in_->closed) return makeError(ErrorCode::Disconnected, "closed");
        return false;
    }
    void close() override {
        for (const auto& p : {in_, out_}) {
            std::lock_guard lock(p->mutex);
            p->closed = true;
            p->cv.notify_all();
        }
    }
    bool connected() const override {
        std::lock_guard a(in_->mutex);
        return !in_->closed;
    }
    std::string describe() const override { return "memory:" + label_; }

private:
    std::shared_ptr<Pipe> in_;
    std::shared_ptr<Pipe> out_;
    std::string label_;
};

struct InMemoryTransportPair {
    std::unique_ptr<InMemoryTransport> a;  // e.g. emulator side
    std::unique_ptr<InMemoryTransport> b;  // e.g. extension side
};

inline InMemoryTransportPair makeInMemoryTransportPair() {
    auto ab = std::make_shared<InMemoryTransport::Pipe>();
    auto ba = std::make_shared<InMemoryTransport::Pipe>();
    return {std::make_unique<InMemoryTransport>(ba, ab, "a"), std::make_unique<InMemoryTransport>(ab, ba, "b")};
}

}  // namespace wpiemu::sdk::testing
