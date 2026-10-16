#include "wpiemu/core/runtime/robot_link.h"

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

// ---- MessageInbox -----------------------------------------------------------------------------------------

void MessageInbox::push(InboundEvent event) {
    {
        std::lock_guard lock(mutex_);
        queue_.push_back(std::move(event));
    }
    cv_.notify_one();
}

bool MessageInbox::waitPop(InboundEvent& out, std::chrono::milliseconds timeout) {
    InboundEvent taken;
    {
        std::unique_lock lock(mutex_);
        if (queue_.empty() && timeout.count() > 0) cv_.wait_for(lock, timeout, [this] { return !queue_.empty() || woken_; });
        if (queue_.empty()) {
            woken_ = false;
            return false;
        }
        taken = std::move(queue_.front());
        queue_.pop_front();
    }
    // `out` may hold the last reference to a link whose destructor joins a thread that pushes here: assign unlocked.
    out = std::move(taken);
    return true;
}

void MessageInbox::wake() {
    {
        std::lock_guard lock(mutex_);
        woken_ = true;
    }
    cv_.notify_all();
}

std::size_t MessageInbox::size() const {
    std::lock_guard lock(mutex_);
    return queue_.size();
}

// ---- ChannelRobotLink ---------------------------------------------------------------------------------------

ChannelRobotLink::ChannelRobotLink(std::uint64_t linkId, std::unique_ptr<proto::ITransport> transport,
                                   std::shared_ptr<const proto::IMessageCodec> codec, MessageInbox& inbox)
    : linkId_(linkId), description_(transport->describe()), inbox_(inbox), channel_(std::move(transport), std::move(codec)) {}

ChannelRobotLink::~ChannelRobotLink() {
    stop_ = true;
    channel_.close();
    if (reader_.joinable()) reader_.join();
}

void ChannelRobotLink::start() {
    if (started_.exchange(true)) return;
    reader_ = std::thread([this] { readLoop(); });
}

Status ChannelRobotLink::send(const proto::Message& message) {
    std::lock_guard lock(sendMutex_);
    return channel_.send(message);
}

void ChannelRobotLink::close() {
    stop_ = true;
    channel_.close();
}

bool ChannelRobotLink::connected() const { return !stop_ && channel_.connected(); }

void ChannelRobotLink::readLoop() {
    std::string reason = "closed";
    while (!stop_) {
        proto::Message message;
        const Result<bool> got = channel_.receive(message, std::chrono::milliseconds(100));
        if (!got) {
            reason = got.error().message;
            break;
        }
        if (!got.value()) continue;
        InboundEvent ev;
        ev.kind = InboundEvent::Kind::Message;
        ev.linkId = linkId_;
        ev.message = std::move(message);
        inbox_.push(std::move(ev));
    }
    stop_ = true;
    InboundEvent ev;
    ev.kind = InboundEvent::Kind::Disconnected;
    ev.linkId = linkId_;
    ev.detail = reason;
    inbox_.push(std::move(ev));
}

// ---- RobotLinkAcceptor --------------------------------------------------------------------------------------

RobotLinkAcceptor::RobotLinkAcceptor(std::unique_ptr<proto::ITransportServer> server, std::shared_ptr<const proto::IMessageCodec> codec,
                                     MessageInbox& inbox, ILogger* logger)
    : server_(std::move(server)), codec_(std::move(codec)), inbox_(inbox), logger_(logger) {}

RobotLinkAcceptor::~RobotLinkAcceptor() { stop(); }

Status RobotLinkAcceptor::start() {
    if (thread_.joinable()) return makeError(ErrorCode::InvalidState, "acceptor already started");
    if (Status s = server_->listen(); !s) return s;
    stop_ = false;
    thread_ = std::thread([this] { acceptLoop(); });
    return okStatus();
}

void RobotLinkAcceptor::stop() {
    stop_ = true;
    if (server_) server_->close();
    if (thread_.joinable()) thread_.join();
}

std::string RobotLinkAcceptor::endpoint() const { return server_->endpoint(); }

void RobotLinkAcceptor::acceptLoop() {
    while (!stop_) {
        Result<std::unique_ptr<proto::ITransport>> accepted = server_->accept(std::chrono::milliseconds(200));
        if (!accepted) {
            if (stop_) break;
            if (logger_ != nullptr) logger_->warn("link", "accept failed: " + accepted.error().message);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (!accepted.value()) continue;  // timeout
        auto link = std::make_shared<ChannelRobotLink>(nextLinkId_++, std::move(accepted).value(), codec_, inbox_);
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
