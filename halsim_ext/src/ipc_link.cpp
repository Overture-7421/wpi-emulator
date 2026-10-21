#include "wpiemu/halsim_ext/ipc_link.h"

#include <utility>

namespace wpiemu::halsim {

using sdk::ErrorCode;

IpcLink::IpcLink(std::unique_ptr<sdk::proto::MessageChannel> channel) : channel_(std::move(channel)) {}

IpcLink::~IpcLink() {
    close("link destroyed");
    if (reader_.joinable() && reader_.get_id() != std::this_thread::get_id()) reader_.join();
    if (writer_.joinable() && writer_.get_id() != std::this_thread::get_id()) writer_.join();
}

void IpcLink::start() {
    reader_ = std::thread([this] { readerLoop(); });
    writer_ = std::thread([this] { writerLoop(); });
}

bool IpcLink::post(sdk::proto::Message message) {
    {
        std::lock_guard lock(mutex_);
        if (closed_.load(std::memory_order_acquire)) return false;
        outbound_.push_back(std::move(message));
    }
    outboundCv_.notify_one();
    return true;
}

bool IpcLink::postDroppable(sdk::proto::Message message) {
    {
        std::lock_guard lock(mutex_);
        if (closed_.load(std::memory_order_acquire)) return false;
        if (queuedLogs_ >= kMaxQueuedLogs) {
            droppedLogs_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        ++queuedLogs_;
        outbound_.push_back(std::move(message));
    }
    outboundCv_.notify_one();
    return true;
}

IpcLink::PopResult IpcLink::pop(sdk::proto::Message& out, std::chrono::milliseconds timeout) {
    switch (inbound_.pop(out, timeout)) {
        case InboundQueue::PopResult::Item: return PopResult::Message;
        case InboundQueue::PopResult::Closed: return PopResult::Closed;
        case InboundQueue::PopResult::Timeout: break;
    }
    return PopResult::Timeout;
}

void IpcLink::close(std::string reason) {
    {
        std::lock_guard lock(mutex_);
        if (closed_.exchange(true, std::memory_order_acq_rel)) return;
        reason_ = reason.empty() ? "closed" : std::move(reason);
    }
    channel_->close();
    inbound_.close();
    outboundCv_.notify_all();
    drainedCv_.notify_all();
}

bool IpcLink::flush(std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    return drainedCv_.wait_for(lock, timeout, [&] {
        return closed_.load(std::memory_order_acquire) || (outbound_.empty() && !writing_);
    }) && outbound_.empty() && !writing_;
}

std::string IpcLink::closeReason() const {
    std::lock_guard lock(mutex_);
    return reason_;
}

void IpcLink::readerLoop() {
    int decodeErrors = 0;
    while (!closed_.load(std::memory_order_acquire)) {
        sdk::proto::Message message;
        const sdk::Result<bool> got = channel_->receive(message, std::chrono::milliseconds(50));
        if (!got) {
            const sdk::Error& e = got.error();
            if (e.code == ErrorCode::Unsupported) {  // well-formed frame of an unknown type: skip (protocol evolution)
                skipped_.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            if (e.code == ErrorCode::ProtocolError && ++decodeErrors <= kMaxConsecutiveDecodeErrors) {
                skipped_.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            close(e.code == ErrorCode::Disconnected ? "peer disconnected" : "receive failed: " + e.message);
            return;
        }
        if (!got.value()) continue;  // timeout
        decodeErrors = 0;
        switch (inbound_.push(std::move(message))) {
            case InboundQueue::PushResult::Pushed: break;
            case InboundQueue::PushResult::Full:
                close("inbound queue overflow: the clock thread is not keeping up");
                return;
            case InboundQueue::PushResult::Closed: return;
        }
    }
}

void IpcLink::writerLoop() {
    for (;;) {
        sdk::proto::Message message;
        {
            std::unique_lock lock(mutex_);
            outboundCv_.wait(lock, [&] { return !outbound_.empty() || closed_.load(std::memory_order_acquire); });
            if (closed_.load(std::memory_order_acquire)) return;
            message = std::move(outbound_.front());
            outbound_.pop_front();
            if (std::holds_alternative<sdk::proto::Log>(message) && queuedLogs_ > 0) --queuedLogs_;
            writing_ = true;
        }
        const sdk::Status sent = channel_->send(message);
        {
            std::lock_guard lock(mutex_);
            writing_ = false;
        }
        drainedCv_.notify_all();
        if (!sent) {
            close(sent.error().code == ErrorCode::Disconnected ? "peer disconnected" : "send failed: " + sent.error().message);
            return;
        }
    }
}

}  // namespace wpiemu::halsim
