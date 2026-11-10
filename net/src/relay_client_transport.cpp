#include <algorithm>
#include <atomic>
#include <thread>
#include <utility>
#include <variant>

#include "relay_support.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/relay_transport.h"

namespace wpiemu::net {

using sdk::ErrorCode;
using sdk::makeError;
namespace relay = sdk::match::relay;

namespace {

// The transport of a joined client: the ENet link to the relay with a thread that watches the control channel.
class RelayClientPeer final : public sdk::proto::ITransport {
public:
    RelayClientPeer(std::unique_ptr<sdk::proto::ITransport> link, IControlLink& control, std::string description)
        : link_(std::move(link)), control_(control), description_(std::move(description)) {
        watcher_ = std::thread([this] { watch(); });
    }

    ~RelayClientPeer() override {
        link_->close();
        watcher_.join();
    }

    sdk::Status send(std::span<const std::byte> frame) override {
        if (ended_.load()) return makeError(ErrorCode::Disconnected, "relay peer closed: " + description_);
        return link_->send(frame);
    }
    sdk::Status sendUnreliable(std::span<const std::byte> frame) override {
        if (ended_.load()) return makeError(ErrorCode::Disconnected, "relay peer closed: " + description_);
        return link_->sendUnreliable(frame);
    }
    sdk::Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        return link_->receive(frame, timeout);
    }
    void close() override {
        ended_.store(true);
        link_->close();
    }
    bool connected() const override { return !ended_.load() && link_->connected(); }
    std::string describe() const override { return description_; }
    std::optional<sdk::proto::LinkStats> linkStats() const override { return link_->linkStats(); }

private:
    void watch() {
        std::vector<std::byte> frame;
        for (;;) {
            auto got = control_.receiveControl(frame, std::chrono::milliseconds(100));
            if (!got) break;
            if (!got.value()) continue;
            auto message = detail::decodeRelayMessage(frame);
            if (message && std::holds_alternative<relay::PeerLeft>(message.value())) {
                ended_.store(true);
                link_->close();
                break;
            }
        }
    }

    std::unique_ptr<sdk::proto::ITransport> link_;
    IControlLink& control_;
    const std::string description_;
    std::atomic<bool> ended_{false};
    std::thread watcher_;
};

}  // namespace

RelayClientTransport::RelayClientTransport(RelayClientOptions options) : options_(std::move(options)) {}

RelayClientTransport::~RelayClientTransport() = default;

sdk::Result<std::unique_ptr<sdk::proto::ITransport>> RelayClientTransport::connect(const std::string& endpoint,
                                                                                  std::chrono::milliseconds timeout) {
    auto parsed = NetEndpoint::parse(endpoint);
    if (!parsed) return parsed.error();
    const NetEndpoint& target = parsed.value();
    if (target.scheme != EndpointScheme::Relay)
        return makeError(ErrorCode::InvalidArgument, "RelayClientTransport connects to relay:// endpoints, got " + endpoint);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    NetEndpoint relayEndpoint;
    relayEndpoint.scheme = EndpointScheme::Enet;
    relayEndpoint.host = target.host;
    relayEndpoint.port = target.port;
    EnetClientTransport enet(options_.link);
    auto connected = enet.connect(relayEndpoint.format(), timeout);
    if (!connected) return connected.error();
    std::unique_ptr<sdk::proto::ITransport> link = std::move(connected).value();
    IControlLink* control = controlLinkOf(*link);
    if (control == nullptr) return makeError(ErrorCode::Internal, "the relay link has no control channel");

    relay::JoinRoom join;
    join.protocolVersion = options_.protocolVersion;
    join.roomId = target.roomId;
    if (sdk::Status sent = detail::sendRelayMessage(*control, join); !sent) return sent.error();

    std::vector<std::byte> frame;
    for (;;) {
        const auto left = sdk::proto::timeLeft(deadline);
        if (left.count() == 0)
            return makeError(ErrorCode::Timeout, "the relay did not answer the join within " + std::to_string(timeout.count()) + " ms");
        auto got = control->receiveControl(frame, std::min(left, std::chrono::milliseconds(50)));
        if (!got) return makeError(ErrorCode::Disconnected, "the relay closed the connection while joining " + endpoint);
        if (!got.value()) continue;
        auto message = detail::decodeRelayMessage(frame);
        if (!message) return makeError(ErrorCode::ProtocolError, "unreadable relay answer: " + message.error().message);
        const auto* result = std::get_if<relay::JoinResult>(&message.value());
        if (result == nullptr) return makeError(ErrorCode::ProtocolError, "unexpected relay message while joining");
        if (!result->ok) return detail::joinFailureError(result->reason);
        if (result->peerId == relay::kRelayHostPeer || result->peerId == relay::kRelayBroadcastPeer)
            return makeError(ErrorCode::ProtocolError, "the relay assigned an invalid peer id");
        const std::string description = target.format() + "(peer " + std::to_string(result->peerId) + ")";
        return std::unique_ptr<sdk::proto::ITransport>(
            std::make_unique<RelayClientPeer>(std::move(link), *control, description));
    }
}

}  // namespace wpiemu::net
