#include "wpiemu/ipc/memory_stream.h"

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <mutex>

namespace wpiemu::ipc {

using namespace sdk;

namespace {

struct Lane {
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<std::byte> bytes;
    bool closed = false;
};

class MemoryByteStream final : public IByteStream {
public:
    MemoryByteStream(std::shared_ptr<Lane> in, std::shared_ptr<Lane> out, std::size_t maxReadChunk, std::string label)
        : in_(std::move(in)), out_(std::move(out)), maxReadChunk_(maxReadChunk), label_(std::move(label)) {}

    ~MemoryByteStream() override { close(); }

    Result<std::size_t> read(std::span<std::byte> buffer, std::chrono::milliseconds timeout) override {
        std::unique_lock lock(in_->mutex);
        in_->cv.wait_for(lock, timeout, [&] { return !in_->bytes.empty() || in_->closed; });
        if (in_->bytes.empty()) {
            if (in_->closed) return makeError(ErrorCode::Disconnected, "memory stream closed");
            return std::size_t{0};
        }
        std::size_t n = std::min(buffer.size(), in_->bytes.size());
        if (maxReadChunk_ != 0) n = std::min(n, maxReadChunk_);
        std::copy_n(in_->bytes.begin(), n, buffer.begin());
        in_->bytes.erase(in_->bytes.begin(), in_->bytes.begin() + static_cast<std::ptrdiff_t>(n));
        return n;
    }

    Status write(std::span<const std::byte> data, std::chrono::milliseconds) override {
        std::lock_guard lock(out_->mutex);
        if (out_->closed) return makeError(ErrorCode::Disconnected, "memory stream closed");
        out_->bytes.insert(out_->bytes.end(), data.begin(), data.end());
        out_->cv.notify_all();
        return okStatus();
    }

    void close() override {
        for (const auto& lane : {in_, out_}) {
            std::lock_guard lock(lane->mutex);
            lane->closed = true;
            lane->cv.notify_all();
        }
    }

    bool isOpen() const override {
        std::lock_guard lock(in_->mutex);
        return !in_->closed;
    }

    std::string describe() const override { return "memory:" + label_; }

private:
    std::shared_ptr<Lane> in_;
    std::shared_ptr<Lane> out_;
    std::size_t maxReadChunk_;
    std::string label_;
};

}  // namespace

MemoryStreamPair makeMemoryStreamPair(MemoryStreamOptions options) {
    auto ab = std::make_shared<Lane>();
    auto ba = std::make_shared<Lane>();
    return {std::make_unique<MemoryByteStream>(ba, ab, options.maxReadChunk, "a"),
            std::make_unique<MemoryByteStream>(ab, ba, options.maxReadChunk, "b")};
}

}  // namespace wpiemu::ipc
