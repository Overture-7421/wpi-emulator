#include "wpiemu/ipc/in_process.h"

#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>

#include "wpiemu/ipc/endpoint.h"

namespace wpiemu::ipc {

using namespace sdk;
using namespace sdk::proto;

namespace detail {

struct Listener {
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<std::unique_ptr<ITransport>> pending;
    bool closed = false;
};

struct InProcessState {
    MemoryStreamOptions streamOptions;
    FramedTransportOptions transportOptions;
    std::mutex mutex;
    std::condition_variable cv;  // signalled when a listener appears
    std::map<std::string, std::shared_ptr<Listener>> listeners;
};

}  // namespace detail

namespace {

using detail::InProcessState;
using detail::Listener;

class InProcessServer final : public ITransportServer {
public:
    InProcessServer(std::shared_ptr<InProcessState> state, std::string endpoint)
        : state_(std::move(state)), endpoint_(std::move(endpoint)) {}
    ~InProcessServer() override { close(); }

    Status listen() override {
        if (!ConnectionEndpoint::parse(endpoint_) || endpoint_.compare(0, kInProcessPrefix.size(), kInProcessPrefix) != 0)
            return makeError(ErrorCode::InvalidArgument, "not an in-process endpoint: " + endpoint_);
        std::lock_guard lock(state_->mutex);
        if (listener_) return makeError(ErrorCode::InvalidState, "already listening");
        if (state_->listeners.count(endpoint_) != 0) return makeError(ErrorCode::AlreadyExists, endpoint_ + " is in use");
        listener_ = std::make_shared<Listener>();
        state_->listeners[endpoint_] = listener_;
        state_->cv.notify_all();
        return okStatus();
    }

    std::string endpoint() const override { return endpoint_; }

    Result<std::unique_ptr<ITransport>> accept(std::chrono::milliseconds timeout) override {
        if (!listener_) return makeError(ErrorCode::InvalidState, "listen() has not been called");
        std::unique_lock lock(listener_->mutex);
        listener_->cv.wait_for(lock, timeout, [&] { return !listener_->pending.empty() || listener_->closed; });
        if (!listener_->pending.empty()) {
            std::unique_ptr<ITransport> t = std::move(listener_->pending.front());
            listener_->pending.pop_front();
            return t;
        }
        if (listener_->closed) return makeError(ErrorCode::Cancelled, "server closed");
        return std::unique_ptr<ITransport>{};
    }

    void close() override {
        if (!listener_) return;
        {
            std::lock_guard lock(state_->mutex);
            const auto it = state_->listeners.find(endpoint_);
            if (it != state_->listeners.end() && it->second == listener_) state_->listeners.erase(it);
        }
        std::deque<std::unique_ptr<ITransport>> orphans;
        {
            std::lock_guard lock(listener_->mutex);
            listener_->closed = true;
            orphans.swap(listener_->pending);
            listener_->cv.notify_all();
        }
        for (auto& t : orphans) t->close();
    }

private:
    std::shared_ptr<InProcessState> state_;
    std::string endpoint_;
    std::shared_ptr<Listener> listener_;
};

class InProcessClient final : public ITransportClient {
public:
    explicit InProcessClient(std::shared_ptr<InProcessState> state) : state_(std::move(state)) {}

    Result<std::unique_ptr<ITransport>> connect(const std::string& endpoint, std::chrono::milliseconds timeout) override {
        if (endpoint.compare(0, kInProcessPrefix.size(), kInProcessPrefix) != 0)
            return makeError(ErrorCode::InvalidArgument, "not an in-process endpoint: " + endpoint);
        std::shared_ptr<Listener> listener;
        {
            std::unique_lock lock(state_->mutex);
            if (!state_->cv.wait_for(lock, timeout, [&] { return state_->listeners.count(endpoint) != 0; }))
                return makeError(ErrorCode::Timeout, "nobody is listening on " + endpoint);
            listener = state_->listeners.at(endpoint);
        }
        MemoryStreamPair streams = makeMemoryStreamPair(state_->streamOptions);
        auto mine = std::make_unique<FramedTransport>(std::move(streams.first), state_->transportOptions);
        auto theirs = std::make_unique<FramedTransport>(std::move(streams.second), state_->transportOptions);
        {
            std::lock_guard lock(listener->mutex);
            if (listener->closed) return makeError(ErrorCode::Disconnected, "server closed");
            listener->pending.push_back(std::move(theirs));
            listener->cv.notify_one();
        }
        return std::unique_ptr<ITransport>(std::move(mine));
    }

private:
    std::shared_ptr<InProcessState> state_;
};

}  // namespace

std::shared_ptr<InProcessNetwork> InProcessNetwork::create(MemoryStreamOptions streamOptions,
                                                           FramedTransportOptions transportOptions) {
    auto state = std::make_shared<InProcessState>();
    state->streamOptions = streamOptions;
    state->transportOptions = transportOptions;
    return std::shared_ptr<InProcessNetwork>(new InProcessNetwork(std::move(state)));
}

std::unique_ptr<ITransportServer> InProcessNetwork::createServer(std::string endpoint) {
    return std::make_unique<InProcessServer>(state_, std::move(endpoint));
}

std::unique_ptr<ITransportClient> InProcessNetwork::createClient() { return std::make_unique<InProcessClient>(state_); }

}  // namespace wpiemu::ipc
