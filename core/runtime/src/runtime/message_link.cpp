#include "wpiemu/core/runtime/message_link.h"

#include <algorithm>
#include <thread>

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

// ---- MessageInbox -----------------------------------------------------------------------------------------

MessageInbox::MessageInbox() : queue_(std::make_shared<Queue>()) {}

MessageInbox::~MessageInbox() { close(); }

void MessageInbox::push(InboundEvent event) {
    push(queue_, std::move(event));
}

void MessageInbox::push(const std::shared_ptr<Queue>& queue, InboundEvent event) {
    if (queue->push(std::move(event)) == Queue::PushResult::Pushed) return;
    // A rejected Connected event can be its link's last owner. Never join its reader under the lock.
    if (event.link) event.link->close();
}

bool MessageInbox::waitPop(InboundEvent& out, std::chrono::milliseconds timeout) {
    return queue_->pop(out, timeout) == Queue::PopResult::Item;
}

void MessageInbox::wake() { queue_->wake(); }

void MessageInbox::close() {
    for (auto& event : queue_->closeAndDrain())
        if (event.link) event.link->close();
    // Destruction may join readers that still publish Disconnected into the closed, live queue.
}

std::size_t MessageInbox::size() const { return queue_->size(); }

// ---- ChannelMessageLink ---------------------------------------------------------------------------------------

ChannelMessageLink::ChannelMessageLink(std::uint64_t linkId, std::unique_ptr<proto::ITransport> transport,
                                   std::shared_ptr<const proto::IMessageCodec> codec, MessageInbox& inbox)
    : linkId_(linkId), description_(transport->describe()), inbox_(inbox.queue_), channel_(std::move(transport), std::move(codec)),
      transport_(&channel_.transport()) {}

ChannelMessageLink::~ChannelMessageLink() { shutdown(); }

void ChannelMessageLink::shutdown() {
    close();
    reader_.join();
}

void ChannelMessageLink::start() {
    if (started_.exchange(true)) return;
    reader_.start([this](const StopToken& stop) { readLoop(stop); });
}

Status ChannelMessageLink::send(const proto::Message& message) {
    std::lock_guard lock(sendMutex_);
    return channel_.send(message, proto::deliveryOf(message));
}

std::optional<proto::LinkStats> ChannelMessageLink::linkStats() const { return transport_->linkStats(); }

void ChannelMessageLink::close() {
    reader_.requestStop();
    channel_.close();
}

bool ChannelMessageLink::connected() const { return !reader_.stopRequested() && channel_.connected(); }

void ChannelMessageLink::readLoop(const StopToken& stop) {
    std::string reason = "closed";
    while (!stop.stopRequested()) {
        proto::Message message;
        const Result<bool> got = channel_.receive(message, std::chrono::milliseconds(100));
        if (!got) {
            const bool malformed = malformedLimit_ > 0 && got.error().code == ErrorCode::ProtocolError;
            if (malformed && ++malformed_ < malformedLimit_) continue;
            reason = malformed ? "too many malformed frames: " + got.error().message : got.error().message;
            break;
        }
        if (!got.value()) continue;
        InboundEvent ev;
        ev.kind = InboundEvent::Kind::Message;
        ev.linkId = linkId_;
        ev.message = std::move(message);
        MessageInbox::push(inbox_, std::move(ev));
    }
    reader_.requestStop();
    InboundEvent ev;
    ev.kind = InboundEvent::Kind::Disconnected;
    ev.linkId = linkId_;
    ev.detail = reason;
    MessageInbox::push(inbox_, std::move(ev));
}

// ---- MessageLinkAcceptor --------------------------------------------------------------------------------------

MessageLinkAcceptor::MessageLinkAcceptor(std::unique_ptr<proto::ITransportServer> server, std::shared_ptr<const proto::IMessageCodec> codec,
                                     MessageInbox& inbox, ILogger* logger)
    : server_(std::move(server)), codec_(std::move(codec)), inbox_(inbox), logger_(logger) {}

MessageLinkAcceptor::~MessageLinkAcceptor() { stop(); }

Status MessageLinkAcceptor::start() {
    if (thread_.joinable()) return makeError(ErrorCode::InvalidState, "acceptor already started");
    if (Status s = server_->listen(); !s) return s;
    thread_.resetStop();
    thread_.start([this](const StopToken& stop) { acceptLoop(stop); });
    return okStatus();
}

void MessageLinkAcceptor::stop() {
    thread_.requestStop();
    if (server_) server_->close();
    thread_.join();
    for (const auto& weak : links_)
        if (auto link = weak.lock()) link->shutdown();
    links_.clear();
}

std::string MessageLinkAcceptor::endpoint() const { return server_->endpoint(); }

void MessageLinkAcceptor::acceptLoop(const StopToken& stop) {
    while (!stop.stopRequested()) {
        Result<std::unique_ptr<proto::ITransport>> accepted = server_->accept(std::chrono::milliseconds(200));
        if (!accepted) {
            if (stop.stopRequested()) break;
            if (logger_ != nullptr) logger_->warn("link", "accept failed: " + accepted.error().message);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (!accepted.value()) continue;  // timeout
        if (stop.stopRequested()) {
            accepted.value()->close();
            break;
        }
        auto link = std::make_shared<ChannelMessageLink>(nextLinkId_++, std::move(accepted).value(), codec_, inbox_);
        link->setMalformedFrameLimit(malformedLimit_);
        std::erase_if(links_, [](const auto& weak) { return weak.expired(); });
        links_.push_back(link);
        if (logger_ != nullptr) logger_->info("link", "robot connected: " + link->describe());
        InboundEvent ev;
        ev.kind = InboundEvent::Kind::Connected;
        ev.linkId = link->id();
        ev.link = link;
        inbox_.push(std::move(ev));
        link->start();
    }
}

}  // namespace wpiemu::core::runtime
