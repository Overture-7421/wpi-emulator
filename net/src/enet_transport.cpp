#include "wpiemu/net/enet_transport.h"

#include <mutex>
#include <utility>

#include "enet_pump.h"
#include "wpiemu/net/endpoint.h"

namespace wpiemu::net {

using sdk::ErrorCode;
using sdk::makeError;

struct EnetServerTransport::Impl {
    explicit Impl(EnetServerOptions o) : options(std::move(o)) {
        NetEndpoint configured;
        configured.host = options.bindHost.empty() ? "::" : options.bindHost;
        configured.port = options.port;
        endpoint = configured.format();
    }

    std::shared_ptr<detail::EnetHostPump> current() const {
        std::lock_guard lock(mutex);
        return pump;
    }

    const EnetServerOptions options;
    mutable std::mutex mutex;
    std::shared_ptr<detail::EnetHostPump> pump;
    std::string endpoint;
};

EnetServerTransport::EnetServerTransport(EnetServerOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}

EnetServerTransport::~EnetServerTransport() { close(); }

sdk::Status EnetServerTransport::listen() {
    std::lock_guard lock(impl_->mutex);
    if (impl_->pump) return makeError(ErrorCode::InvalidState, "already listening on " + impl_->endpoint);
    auto created = detail::EnetHostPump::createServer(impl_->options);
    if (!created) return created.error();
    impl_->pump = std::move(created).value();
    impl_->endpoint = impl_->pump->endpoint();
    impl_->pump->start();
    return sdk::okStatus();
}

std::string EnetServerTransport::endpoint() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->endpoint;
}

sdk::Result<std::unique_ptr<sdk::proto::ITransport>> EnetServerTransport::accept(std::chrono::milliseconds timeout) {
    const auto pump = impl_->current();
    if (!pump) return makeError(ErrorCode::InvalidState, "accept() before listen()");
    return pump->accept(timeout);
}

void EnetServerTransport::close() {
    const auto pump = impl_->current();
    if (pump) pump->stop();
}

std::size_t EnetServerTransport::peerCount() const {
    const auto pump = impl_->current();
    return pump ? pump->peerCount() : 0;
}

EnetWireStats EnetServerTransport::wireStats() const {
    const auto pump = impl_->current();
    return pump ? pump->wireStats() : EnetWireStats{};
}

EnetClientTransport::EnetClientTransport(EnetClientOptions options) : options_(std::move(options)) {}

EnetClientTransport::~EnetClientTransport() = default;

sdk::Result<std::unique_ptr<sdk::proto::ITransport>> EnetClientTransport::connect(const std::string& endpoint,
                                                                                  std::chrono::milliseconds timeout) {
    auto parsed = NetEndpoint::parse(endpoint);
    if (!parsed) return parsed.error();
    if (parsed.value().scheme != EndpointScheme::Enet)
        return makeError(ErrorCode::InvalidArgument, "EnetClientTransport connects to enet:// endpoints, got " + endpoint);
    if (parsed.value().port == 0) return makeError(ErrorCode::InvalidArgument, "cannot connect to port 0: " + endpoint);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto created = detail::EnetHostPump::createClient(options_, parsed.value().host, parsed.value().port);
    if (!created) return created.error();
    auto [pump, link] = std::move(created).value();

    switch (link->waitWhileConnecting(deadline)) {
        case detail::LinkState::Connected:
            return std::unique_ptr<sdk::proto::ITransport>(std::make_unique<detail::LinkTransport>(std::move(link), std::move(pump)));
        case detail::LinkState::Closed:
            pump->stop();
            return makeError(ErrorCode::Disconnected, "connection to " + endpoint + " failed");
        case detail::LinkState::Connecting: break;
    }
    link->requestClose();
    pump->stop();
    return makeError(ErrorCode::Timeout, "no connection to " + endpoint + " within " + std::to_string(timeout.count()) + " ms");
}

}  // namespace wpiemu::net
