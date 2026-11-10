#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/blocking_queue.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/stoppable_thread.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::core::runtime {

// The sim thread's view of one connection to a peer: the HALSim extension in the robot process, or (match host) a game client
// (sdk-only: no ipc module). The names say "message", not "robot", because the same machinery serves both.
class IMessageLink {
    WPIEMU_INTERFACE(IMessageLink)
public:
    virtual sdk::Status send(const sdk::proto::Message& message) = 0;  // thread-safe
    virtual void close() = 0;                                          // any thread; idempotent
    virtual bool connected() const = 0;
    virtual std::string describe() const = 0;
    // Health of the underlying transport when it can measure it (nullopt otherwise). Any thread.
    virtual std::optional<sdk::proto::LinkStats> linkStats() const { return std::nullopt; }
    // Frames that failed to decode and were dropped (only links that tolerate malformed frames count them). Any thread.
    virtual std::uint64_t malformedFrames() const { return 0; }
};

// Something that happened on the IPC side, handed to the sim thread.
struct InboundEvent {
    enum class Kind { Message, Connected, Disconnected };
    Kind kind = Kind::Message;
    std::uint64_t linkId = 0;           // events of a replaced link are ignored by the runner
    sdk::proto::Message message;        // Kind::Message
    std::shared_ptr<IMessageLink> link;   // Kind::Connected
    std::string detail;                 // Kind::Disconnected: why
};

// Multi-producer (IPC threads) / single-consumer (sim thread) mailbox: the ONLY way IPC data reaches the hub.
class MessageInbox {
public:
    MessageInbox();
    ~MessageInbox();
    MessageInbox(const MessageInbox&) = delete;
    MessageInbox& operator=(const MessageInbox&) = delete;

    void push(InboundEvent event);
    // Waits up to `timeout` (0 = just poll). False on timeout, after wake(), or once closed.
    bool waitPop(InboundEvent& out, std::chrono::milliseconds timeout);
    bool tryPop(InboundEvent& out) { return waitPop(out, std::chrono::milliseconds(0)); }
    void wake();  // makes a concurrent waitPop return false immediately
    // Rejects future events and closes pending connections. Last link owners are released unlocked.
    void close();
    std::size_t size() const;

private:
    friend class ChannelMessageLink;
    using Queue = sdk::BlockingQueue<InboundEvent>;
    static void push(const std::shared_ptr<Queue>& queue, InboundEvent event);
    std::shared_ptr<Queue> queue_;  // shared so a reader that outlives the inbox still publishes into a live, closed queue
};

// IMessageLink over a sdk MessageChannel (any ITransport + IMessageCodec, e.g. the ipc module's, or the in-memory
// pair of sdk/testing). A reader thread decodes frames into the inbox; any receive/decode error ends the link
// with an InboundEvent::Disconnected. The constructor does not start reading: call start() after announcing the
// link (so `Connected` is queued before the first message).
class ChannelMessageLink final : public IMessageLink {
public:
    ChannelMessageLink(std::uint64_t linkId, std::unique_ptr<sdk::proto::ITransport> transport,
                     std::shared_ptr<const sdk::proto::IMessageCodec> codec, MessageInbox& inbox);
    ~ChannelMessageLink() override;

    // By default the first receive or decode error ends the link. With a limit > 0 a frame the codec rejects
    // (ErrorCode::ProtocolError) is dropped and counted instead, and the link ends once `limit` frames were dropped.
    // Call before start().
    void setMalformedFrameLimit(std::size_t limit) { malformedLimit_ = limit; }
    void start();
    // Sent on the delivery class of the message (sdk::proto::deliveryOf): latest-wins match streams go unreliable.
    sdk::Status send(const sdk::proto::Message& message) override;
    void close() override;
    void shutdown();  // close and join the reader, called only by the owner
    bool connected() const override;
    std::string describe() const override { return description_; }
    std::optional<sdk::proto::LinkStats> linkStats() const override;
    std::uint64_t malformedFrames() const override { return malformed_.load(); }
    std::uint64_t id() const { return linkId_; }

private:
    void readLoop(const sdk::StopToken& stop);

    std::uint64_t linkId_;
    std::size_t malformedLimit_ = 0;
    std::atomic<std::uint64_t> malformed_{0};
    std::string description_;
    std::shared_ptr<MessageInbox::Queue> inbox_;
    sdk::proto::MessageChannel channel_;
    sdk::proto::ITransport* transport_;  // owned by channel_
    std::mutex sendMutex_;
    std::atomic<bool> started_{false};
    sdk::StoppableThread reader_;
};

// Accepts the extension's connection(s) on an ITransportServer and feeds the inbox. One robot at a time is
// served by the runner; a newer connection replaces the older one (robot restart).
class MessageLinkAcceptor {
public:
    MessageLinkAcceptor(std::unique_ptr<sdk::proto::ITransportServer> server, std::shared_ptr<const sdk::proto::IMessageCodec> codec,
                      MessageInbox& inbox, sdk::ILogger* logger = nullptr);
    ~MessageLinkAcceptor();

    // ChannelMessageLink::setMalformedFrameLimit for every link accepted from now on.
    void setMalformedFrameLimit(std::size_t limit) { malformedLimit_ = limit; }
    sdk::Status start();  // listen() + accept thread
    void stop();
    std::string endpoint() const;  // what the robot gets in WPIEMU_IPC (valid after start())

private:
    void acceptLoop(const sdk::StopToken& stop);

    std::unique_ptr<sdk::proto::ITransportServer> server_;
    std::shared_ptr<const sdk::proto::IMessageCodec> codec_;
    MessageInbox& inbox_;
    sdk::ILogger* logger_;
    std::uint64_t nextLinkId_ = 1;
    std::size_t malformedLimit_ = 0;
    std::vector<std::weak_ptr<ChannelMessageLink>> links_;  // accept thread, then stop() after join
    sdk::StoppableThread thread_;
};

}  // namespace wpiemu::core::runtime
