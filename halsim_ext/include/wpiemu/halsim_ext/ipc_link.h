#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "wpiemu/sdk/core/blocking_queue.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::halsim {

// The extension's single IPC endpoint (docs/reference/extension-ipc.md: "one IPC thread owns the socket"). Two private
// threads move messages between the transport and two queues, so HAL callbacks, bridges and the
// clock thread only ever enqueue and never touch the socket:
//   reader : MessageChannel::receive -> inbound queue
//   writer : outbound queue -> MessageChannel::send
// Used after the synchronous Hello/HelloAck exchange (which uses the channel directly).
class IpcLink {
public:
    enum class PopResult { Message, Timeout, Closed };

    explicit IpcLink(std::unique_ptr<sdk::proto::MessageChannel> channel);
    ~IpcLink();
    IpcLink(const IpcLink&) = delete;
    IpcLink& operator=(const IpcLink&) = delete;

    void start();

    // Control path: never dropped. Returns false once the link is closed.
    bool post(sdk::proto::Message message);
    // Log path: dropped (returns false) instead of growing the queue without bound.
    bool postDroppable(sdk::proto::Message message);

    // Next inbound message; `Closed` only after the inbound queue drained.
    PopResult pop(sdk::proto::Message& out, std::chrono::milliseconds timeout);

    bool open() const { return !closed_.load(std::memory_order_acquire); }
    // Idempotent; wakes every waiter. Pending outbound messages are discarded unless flush() ran.
    void close(std::string reason = {});
    // Waits until the outbound queue has been handed to the transport.
    bool flush(std::chrono::milliseconds timeout);
    std::string closeReason() const;

    std::uint64_t skippedFrames() const { return skipped_.load(); }
    std::uint64_t droppedLogs() const { return droppedLogs_.load(); }

private:
    void readerLoop();
    void writerLoop();

    static constexpr std::size_t kMaxInbound = 8192;
    static constexpr std::size_t kMaxQueuedLogs = 512;
    static constexpr int kMaxConsecutiveDecodeErrors = 8;

    using InboundQueue = sdk::BlockingQueue<sdk::proto::Message>;

    std::unique_ptr<sdk::proto::MessageChannel> channel_;
    std::atomic<bool> closed_{false};
    std::atomic<std::uint64_t> skipped_{0};
    std::atomic<std::uint64_t> droppedLogs_{0};

    InboundQueue inbound_{kMaxInbound};

    // Guards the outbound side: its queue, the writer's in-flight send (what flush() waits for) and the close reason.
    mutable std::mutex mutex_;
    std::condition_variable outboundCv_;
    std::condition_variable drainedCv_;
    std::deque<sdk::proto::Message> outbound_;
    std::size_t queuedLogs_ = 0;
    bool writing_ = false;  // writer is inside send()
    std::string reason_;

    std::thread reader_;
    std::thread writer_;
};

}  // namespace wpiemu::halsim
