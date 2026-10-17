#include <doctest/doctest.h>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "wpiemu/core/runtime/robot_link.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using namespace std::chrono_literals;

namespace {

// A broken join / mailbox-lock ordering must fail finitely rather than hang the whole test run.
class DeadlockDeadline {
public:
    DeadlockDeadline()
        : thread_([this] {
              std::unique_lock lock(mutex_);
              if (!cv_.wait_for(lock, 10s, [this] { return done_; })) std::abort();
          }) {}
    ~DeadlockDeadline() {
        {
            std::lock_guard lock(mutex_);
            done_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    bool done_ = false;
    std::thread thread_;
};

struct TransportState {
    bool waitFor(const std::function<bool()>& predicate) {
        std::unique_lock lock(mutex);
        return cv.wait_for(lock, 2s, predicate);
    }
    std::mutex mutex;
    std::condition_variable cv;
    bool receiving = false;
    bool closed = false;
    bool holdReader = false;
    bool releaseReader = false;
    bool readerReturned = false;
    bool readerTimedOut = false;
    bool destroyed = false;
};

class LifecycleTransport final : public proto::ITransport {
public:
    explicit LifecycleTransport(std::shared_ptr<TransportState> state) : state_(std::move(state)) {}
    ~LifecycleTransport() override {
        std::lock_guard lock(state_->mutex);
        state_->destroyed = true;
        state_->cv.notify_all();
    }
    Status send(std::span<const std::byte>) override { return okStatus(); }
    Result<bool> receive(std::vector<std::byte>&, std::chrono::milliseconds timeout) override {
        std::unique_lock lock(state_->mutex);
        state_->receiving = true;
        state_->cv.notify_all();
        if (!state_->cv.wait_for(lock, timeout, [this] { return state_->closed; })) return false;
        if (state_->holdReader)
            state_->readerTimedOut =
                !state_->cv.wait_for(lock, 2s, [this] { return state_->releaseReader; });
        state_->readerReturned = true;
        state_->cv.notify_all();
        return makeError(ErrorCode::Disconnected, "closed");
    }
    void close() override {
        std::lock_guard lock(state_->mutex);
        state_->closed = true;
        state_->cv.notify_all();
    }
    bool connected() const override {
        std::lock_guard lock(state_->mutex);
        return !state_->closed;
    }
    std::string describe() const override { return "lifecycle-test"; }

private:
    std::shared_ptr<TransportState> state_;
};

struct ServerState {
    std::mutex mutex;
    std::condition_variable cv;
    bool closed = false;
    bool returnAfterClose = false;
    int accepts = 0;
};

class LifecycleServer final : public proto::ITransportServer {
public:
    LifecycleServer(std::shared_ptr<ServerState> server, std::shared_ptr<TransportState> transport)
        : state_(std::move(server)), transport_(std::move(transport)) {}
    Status listen() override { return okStatus(); }
    std::string endpoint() const override { return "lifecycle-test"; }
    Result<std::unique_ptr<proto::ITransport>> accept(std::chrono::milliseconds timeout) override {
        std::unique_lock lock(state_->mutex);
        ++state_->accepts;
        state_->cv.notify_all();
        if (state_->accepts == 1) {
            if (state_->returnAfterClose)
                state_->cv.wait_for(lock, 2s, [this] { return state_->closed; });
            return std::unique_ptr<proto::ITransport>(
                std::make_unique<LifecycleTransport>(transport_));
        }
        state_->cv.wait_for(lock, timeout, [this] { return state_->closed; });
        return std::unique_ptr<proto::ITransport>{};
    }
    void close() override {
        std::lock_guard lock(state_->mutex);
        state_->closed = true;
        state_->cv.notify_all();
    }

private:
    std::shared_ptr<ServerState> state_;
    std::shared_ptr<TransportState> transport_;
};

}  // namespace

TEST_CASE("inbox: destroying an unconsumed connection joins its notifying reader safely") {
    DeadlockDeadline deadline;
    auto state = std::make_shared<TransportState>();
    state->holdReader = true;
    auto inbox = std::make_unique<MessageInbox>();
    auto link =
        std::make_shared<ChannelRobotLink>(1, std::make_unique<LifecycleTransport>(state),
                                           std::make_shared<testing::MemoryMessageCodec>(), *inbox);
    InboundEvent connected;
    connected.kind = InboundEvent::Kind::Connected;
    connected.link = link;
    inbox->push(std::move(connected));
    link->start();
    link.reset();  // the pending Connected event is the only owner
    REQUIRE(state->waitFor([&] { return state->receiving; }));

    std::mutex mutex;
    std::condition_variable cv;
    bool disposed = false;
    std::thread dispose([owned = std::move(inbox), &mutex, &cv, &disposed]() mutable {
        owned.reset();
        {
            std::lock_guard lock(mutex);
            disposed = true;
        }
        cv.notify_all();
    });
    const bool closed = state->waitFor([&] { return state->closed; });
    {
        std::lock_guard lock(mutex);
        CHECK_FALSE(disposed);  // destruction must still be waiting for the reader
    }
    {
        std::lock_guard lock(state->mutex);
        state->releaseReader = true;
        state->cv.notify_all();
    }
    bool completed;
    {
        std::unique_lock lock(mutex);
        completed = cv.wait_for(lock, 2s, [&] { return disposed; });
    }
    CHECK(closed);
    CHECK(completed);
    dispose.join();  // DeadlockDeadline bounds even a broken implementation
    std::lock_guard lock(state->mutex);
    CHECK(state->destroyed);
    CHECK(state->readerReturned);
    CHECK_FALSE(state->readerTimedOut);
}

TEST_CASE("link: a reader can finish after its mailbox owner is destroyed") {
    DeadlockDeadline deadline;
    auto state = std::make_shared<TransportState>();
    auto inbox = std::make_unique<MessageInbox>();
    auto link =
        std::make_unique<ChannelRobotLink>(1, std::make_unique<LifecycleTransport>(state),
                                           std::make_shared<testing::MemoryMessageCodec>(), *inbox);
    link->start();
    REQUIRE(state->waitFor([&] { return state->receiving; }));
    inbox.reset();
    link->shutdown();  // Disconnected is rejected by the still-live, closed mailbox state
    link.reset();
    std::lock_guard lock(state->mutex);
    CHECK(state->readerReturned);
    CHECK(state->destroyed);
}

TEST_CASE("inbox: close rejects pending and future connection owners outside its lock") {
    DeadlockDeadline deadline;
    MessageInbox inbox;
    inbox.close();
    inbox.close();
    auto state = std::make_shared<TransportState>();
    auto link =
        std::make_shared<ChannelRobotLink>(1, std::make_unique<LifecycleTransport>(state),
                                           std::make_shared<testing::MemoryMessageCodec>(), inbox);
    link->start();
    InboundEvent connected;
    connected.kind = InboundEvent::Kind::Connected;
    connected.link = std::move(link);
    inbox.push(std::move(connected));
    CHECK(inbox.size() == 0);
    InboundEvent event;
    CHECK_FALSE(inbox.waitPop(event, 5s));  // closure wakes / bypasses the wait permanently
    std::lock_guard lock(state->mutex);
    CHECK(state->closed);
    CHECK(state->destroyed);
}

TEST_CASE("acceptor: stop joins readers of unconsumed accepted connections") {
    DeadlockDeadline deadline;
    MessageInbox inbox;
    auto transport = std::make_shared<TransportState>();
    auto server = std::make_shared<ServerState>();
    RobotLinkAcceptor acceptor(std::make_unique<LifecycleServer>(server, transport),
                               std::make_shared<testing::MemoryMessageCodec>(), inbox);
    REQUIRE(acceptor.start().ok());
    REQUIRE(transport->waitFor([&] { return transport->receiving; }));
    {
        std::unique_lock lock(server->mutex);
        REQUIRE(server->cv.wait_for(lock, 2s, [&] { return server->accepts >= 2; }));
    }
    acceptor.stop();
    acceptor.stop();
    {
        std::lock_guard lock(transport->mutex);
        CHECK(transport->closed);
        CHECK(transport->readerReturned);
        CHECK_FALSE(transport->destroyed);  // pending Connected still owns the stopped link
    }
    inbox.close();
    CHECK(inbox.size() == 0);
    std::lock_guard lock(transport->mutex);
    CHECK(transport->destroyed);
}

TEST_CASE("acceptor: a connection returned concurrently with stop is closed without publication") {
    DeadlockDeadline deadline;
    MessageInbox inbox;
    auto transport = std::make_shared<TransportState>();
    auto server = std::make_shared<ServerState>();
    server->returnAfterClose = true;
    RobotLinkAcceptor acceptor(std::make_unique<LifecycleServer>(server, transport),
                               std::make_shared<testing::MemoryMessageCodec>(), inbox);
    REQUIRE(acceptor.start().ok());
    {
        std::unique_lock lock(server->mutex);
        REQUIRE(server->cv.wait_for(lock, 2s, [&] { return server->accepts == 1; }));
    }
    acceptor.stop();
    CHECK(inbox.size() == 0);
    std::lock_guard lock(transport->mutex);
    CHECK(transport->closed);
    CHECK(transport->destroyed);
    CHECK_FALSE(transport->receiving);
}
