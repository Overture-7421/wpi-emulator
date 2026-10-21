#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/blocking_queue.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/stoppable_thread.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::core::runtime {

// The sim thread's view of one connection to the extension in the robot process (sdk-only: no ipc module).
class IRobotLink {
    WPIEMU_INTERFACE(IRobotLink)
public:
    virtual sdk::Status send(const sdk::proto::Message& message) = 0;  // thread-safe
    virtual void close() = 0;                                          // any thread; idempotent
    virtual bool connected() const = 0;
    virtual std::string describe() const = 0;
};

// Something that happened on the IPC side, handed to the sim thread.
struct InboundEvent {
    enum class Kind { Message, Connected, Disconnected };
    Kind kind = Kind::Message;
    std::uint64_t linkId = 0;           // events of a replaced link are ignored by the runner
    sdk::proto::Message message;        // Kind::Message
    std::shared_ptr<IRobotLink> link;   // Kind::Connected
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
    friend class ChannelRobotLink;
    using Queue = sdk::BlockingQueue<InboundEvent>;
    static void push(const std::shared_ptr<Queue>& queue, InboundEvent event);
    std::shared_ptr<Queue> queue_;  // shared so a reader that outlives the inbox still publishes into a live, closed queue
};

// IRobotLink over a sdk MessageChannel (any ITransport + IMessageCodec, e.g. the ipc module's, or the in-memory
// pair of sdk/testing). A reader thread decodes frames into the inbox; any receive/decode error ends the link
// with an InboundEvent::Disconnected. The constructor does not start reading: call start() after announcing the
// link (so `Connected` is queued before the first message).
class ChannelRobotLink final : public IRobotLink {
public:
    ChannelRobotLink(std::uint64_t linkId, std::unique_ptr<sdk::proto::ITransport> transport,
                     std::shared_ptr<const sdk::proto::IMessageCodec> codec, MessageInbox& inbox);
    ~ChannelRobotLink() override;

    void start();
    sdk::Status send(const sdk::proto::Message& message) override;
    void close() override;
    void shutdown();  // close and join the reader, called only by the owner
    bool connected() const override;
    std::string describe() const override { return description_; }
    std::uint64_t id() const { return linkId_; }

private:
    void readLoop(const sdk::StopToken& stop);

    std::uint64_t linkId_;
    std::string description_;
    std::shared_ptr<MessageInbox::Queue> inbox_;
    sdk::proto::MessageChannel channel_;
    std::mutex sendMutex_;
    std::atomic<bool> started_{false};
    sdk::StoppableThread reader_;
};

// Accepts the extension's connection(s) on an ITransportServer and feeds the inbox. One robot at a time is
// served by the runner; a newer connection replaces the older one (robot restart).
class RobotLinkAcceptor {
public:
    RobotLinkAcceptor(std::unique_ptr<sdk::proto::ITransportServer> server, std::shared_ptr<const sdk::proto::IMessageCodec> codec,
                      MessageInbox& inbox, sdk::ILogger* logger = nullptr);
    ~RobotLinkAcceptor();

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
    std::vector<std::weak_ptr<ChannelRobotLink>> links_;  // accept thread, then stop() after join
    sdk::StoppableThread thread_;
};

}  // namespace wpiemu::core::runtime
