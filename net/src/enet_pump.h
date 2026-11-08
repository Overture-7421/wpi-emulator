#pragma once

// Internal (not installed): the pump that owns ONE ENet host and its thread, shared by the server and client transports.
// ENet headers stay in the .cpp. See enet_transport.h for the threading contract and the channel map.

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

#include "enet_link.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::net::detail {

// A transport for one link. Destroying it closes the link and releases `owner`, which for a client is the pump (so the
// pump thread is joined).
class LinkTransport final : public sdk::proto::ITransport {
public:
    explicit LinkTransport(std::shared_ptr<PeerLink> link, std::shared_ptr<void> owner = {});
    ~LinkTransport() override;

    sdk::Status send(std::span<const std::byte> frame) override { return link_->send(frame, true); }
    sdk::Status sendUnreliable(std::span<const std::byte> frame) override { return link_->send(frame, false); }
    sdk::Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        return link_->receive(frame, timeout);
    }
    void close() override { link_->requestClose(); }
    bool connected() const override { return link_->connected(); }
    std::string describe() const override { return link_->description(); }
    std::optional<sdk::proto::LinkStats> linkStats() const override { return link_->stats(); }

private:
    std::shared_ptr<PeerLink> link_;
    std::shared_ptr<void> owner_;
};

class EnetHostPump {
public:
    struct Client {
        std::shared_ptr<EnetHostPump> pump;
        std::shared_ptr<PeerLink> link;
    };

    // Binds a host (the thread is not started until start()). IoError when the address cannot be bound.
    static sdk::Result<std::shared_ptr<EnetHostPump>> createServer(const EnetServerOptions& options);
    // Creates a host with one outgoing connection and starts its thread; the caller waits on `link`.
    static sdk::Result<Client> createClient(const EnetLinkOptions& options, const std::string& host, std::uint16_t port);

    ~EnetHostPump();
    EnetHostPump(const EnetHostPump&) = delete;
    EnetHostPump& operator=(const EnetHostPump&) = delete;

    void start();
    // Disconnects every peer politely (bounded by the linger), joins the thread and releases the host. Idempotent.
    void stop();

    sdk::Result<std::unique_ptr<sdk::proto::ITransport>> accept(std::chrono::milliseconds timeout);
    const std::string& endpoint() const;
    std::size_t peerCount() const;
    EnetWireStats wireStats() const;

private:
    struct Impl;
    explicit EnetHostPump(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::net::detail
