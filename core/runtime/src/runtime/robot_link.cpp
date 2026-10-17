#include "wpiemu/core/runtime/robot_link.h"

#include <algorithm>

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

// ---- MessageInbox -----------------------------------------------------------------------------------------

struct MessageInbox::State {
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<InboundEvent> queue;
    bool woken = false;
    bool closed = false;
};

MessageInbox::MessageInbox() : state_(std::make_shared<State>()) {}

MessageInbox::~MessageInbox() { close(); }

void MessageInbox::push(InboundEvent event) {
    push(state_, std::move(event));
}

void MessageInbox::push(const std::shared_ptr<State>& state, InboundEvent event) {
    {
        std::lock_guard lock(state->mutex);
        if (!state->closed) {
            state->queue.push_back(std::move(event));
            state->cv.notify_one();
            return;
        }
    }
    // A rejected Connected event can be its link's last owner. Never join its reader under the lock.
    if (event.link) event.link->close();
}

bool MessageInbox::waitPop(InboundEvent& out, std::chrono::milliseconds timeout) {
    InboundEvent taken;
    {
        std::unique_lock lock(state_->mutex);
        if (state_->queue.empty() && timeout.count() > 0)
            state_->cv.wait_for(lock, timeout, [this] { return !state_->queue.empty() || state_->woken || state_->closed; });
        if (state_->queue.empty()) {
            state_->woken = false;
            return false;
        }
        taken = std::move(state_->queue.front());
        state_->queue.pop_front();
    }
    // `out` may hold the last reference to a link whose destructor joins a thread that pushes here: assign unlocked.
    out = std::move(taken);
    return true;
}

void MessageInbox::wake() {
    {
        std::lock_guard lock(state_->mutex);
        state_->woken = true;
    }
    state_->cv.notify_all();
}

void MessageInbox::close() {
    std::deque<InboundEvent> pending;
    {
        std::lock_guard lock(state_->mutex);
        state_->closed = true;
        pending.swap(state_->queue);
    }
    state_->cv.notify_all();
    for (auto& event : pending)
        if (event.link) event.link->close();
    // Destruction may join readers that still publish Disconnected into the closed, live State.
}

std::size_t MessageInbox::size() const {
    std::lock_guard lock(state_->mutex);
    return state_->queue.size();
}

// ---- ChannelRobotLink ---------------------------------------------------------------------------------------

ChannelRobotLink::ChannelRobotLink(std::uint64_t linkId, std::unique_ptr<proto::ITransport> transport,
                                   std::shared_ptr<const proto::IMessageCodec> codec, MessageInbox& inbox)
    : linkId_(linkId), description_(transport->describe()), inbox_(inbox.state_), channel_(std::move(transport), std::move(codec)) {}

ChannelRobotLink::~ChannelRobotLink() { shutdown(); }

void ChannelRobotLink::shutdown() {
    close();
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
        MessageInbox::push(inbox_, std::move(ev));
    }
    stop_ = true;
    InboundEvent ev;
    ev.kind = InboundEvent::Kind::Disconnected;
    ev.linkId = linkId_;
    ev.detail = reason;
    MessageInbox::push(inbox_, std::move(ev));
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
    for (const auto& weak : links_)
        if (auto link = weak.lock()) link->shutdown();
    links_.clear();
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
        if (stop_) {
            accepted.value()->close();
            break;
        }
        auto link = std::make_shared<ChannelRobotLink>(nextLinkId_++, std::move(accepted).value(), codec_, inbox_);
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
