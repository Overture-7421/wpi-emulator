#include "wpiemu/ipc/in_process.h"

#include <condition_variable>
#include <map>
#include <mutex>

#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/sdk/core/blocking_queue.h"

namespace wpiemu::ipc {

using namespace sdk;
using namespace sdk::proto;

namespace detail {

// Connections a client made and the server has not accepted yet; closing it refuses further clients.
using Listener = BlockingQueue<std::unique_ptr<ITransport>>;

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
        std::unique_ptr<ITransport> accepted;
        switch (listener_->pop(accepted, timeout)) {
            case Listener::PopResult::Item: return accepted;
            case Listener::PopResult::Closed: return makeError(ErrorCode::Cancelled, "server closed");
            case Listener::PopResult::Timeout: break;
        }
        return std::unique_ptr<ITransport>{};
    }

    void close() override {
        if (!listener_) return;
        {
            std::lock_guard lock(state_->mutex);
            const auto it = state_->listeners.find(endpoint_);
            if (it != state_->listeners.end() && it->second == listener_) state_->listeners.erase(it);
        }
        for (auto& orphan : listener_->closeAndDrain()) orphan->close();
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
        std::unique_ptr<ITransport> theirs = std::make_unique<FramedTransport>(std::move(streams.second), state_->transportOptions);
        if (listener->push(std::move(theirs)) == Listener::PushResult::Closed)
            return makeError(ErrorCode::Disconnected, "server closed");
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
