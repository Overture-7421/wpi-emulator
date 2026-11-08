#include "enet_pump.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <enet6/enet.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "wpiemu/net/endpoint.h"
#include "wpiemu/sdk/core/stoppable_thread.h"

namespace wpiemu::net::detail {
namespace {

using sdk::ErrorCode;
using sdk::makeError;
using Clock = std::chrono::steady_clock;

constexpr enet_uint32 kPollMs = 1;
constexpr std::size_t kSpareHostPeers = 1;  // lets a peer over the limit connect, so it can be refused at once
constexpr std::size_t kMaxWaitingDataBytes = 1024 * 1024;

std::mutex g_runtimeMutex;
std::size_t g_runtimeUsers = 0;

// One enet_initialize for as long as any host exists. Counted under a lock so a late release cannot deinitialize ENet
// after a concurrent acquire initialized it.
std::shared_ptr<void> acquireEnetRuntime() {
    static int token = 0;
    std::lock_guard lock(g_runtimeMutex);
    if (g_runtimeUsers == 0 && enet_initialize() != 0) return nullptr;
    ++g_runtimeUsers;
    return std::shared_ptr<void>(&token, [](void*) {
        std::lock_guard guard(g_runtimeMutex);
        if (--g_runtimeUsers == 0) enet_deinitialize();
    });
}

struct TokenBucket {
    double capacity = 0;
    double rate = 0;
    double tokens = 0;
    Clock::time_point last{};
    bool enabled = false;

    void init(double perSecond, double burst, Clock::time_point now) {
        enabled = perSecond > 0;
        rate = perSecond;
        capacity = std::max(perSecond, burst);
        tokens = capacity;
        last = now;
    }
    void refill(Clock::time_point now) {
        if (!enabled) return;
        const double elapsed = std::chrono::duration<double>(now - last).count();
        if (elapsed > 0) tokens = std::min(capacity, tokens + elapsed * rate);
        last = now;
    }
};

std::string describePeer(ENetAddress address) {
    const std::uint16_t port = address.port;
    address.port = 0;  // with a port set ENet appends it (and brackets IPv6)
    char ip[ENET_ADDRESS_MAX_LENGTH] = {};
    if (enet_address_get_host_ip(&address, ip, sizeof ip) != 0) return "enet://?";
    NetEndpoint endpoint;
    endpoint.host = ip;
    endpoint.port = port;
    return endpoint.format();
}

// A literal, else a host name (IPv4 preferred: a dual-stack server accepts both).
bool resolveAddress(const std::string& host, std::uint16_t port, ENetAddress& out) {
    ENetAddress address{};
    if (enet_address_set_host_ip(&address, host.c_str()) != 0) {
        address = {};
        address.port = port;
        if (enet_address_set_host(&address, ENET_ADDRESS_TYPE_IPV4, host.c_str()) != 0 ||
            address.type != ENET_ADDRESS_TYPE_IPV4) {
            address = {};
            address.port = port;
            if (enet_address_set_host(&address, ENET_ADDRESS_TYPE_ANY, host.c_str()) != 0) return false;
        }
    }
    address.port = port;
    out = address;
    return true;
}

LinkLimits limitsOf(const EnetLinkOptions& options) {
    LinkLimits limits;
    limits.maxReliableFrameBytes = options.maxReliableFrameBytes;
    limits.maxUnreliableFrameBytes = options.maxUnreliableFrameBytes;
    limits.maxQueuedOutboundBytes = options.maxQueuedOutboundBytes;
    limits.maxQueuedInboundFrames = std::max<std::size_t>(options.maxQueuedInboundFrames, 1);
    limits.maxQueuedControlFrames = std::max<std::size_t>(options.maxQueuedControlFrames, 1);
    return limits;
}

ENetHost* createHost(ENetAddressType type, const ENetAddress* bindAddress, std::size_t peers,
                     const EnetLinkOptions& options) {
    ENetHost* host = enet_host_create(type, bindAddress, peers, kEnetChannelCount, options.incomingBandwidth,
                                      options.outgoingBandwidth);
    if (host == nullptr) return nullptr;
    host->maximumPacketSize =
        std::max({options.maxReliableFrameBytes, options.maxUnreliableFrameBytes, kMaxControlFrameBytes});
    host->maximumWaitingData = kMaxWaitingDataBytes;
    if (options.compression && enet_host_compress_with_range_coder(host) != 0) {
        enet_host_destroy(host);
        return nullptr;
    }
    return host;
}

struct PumpPeer {
    ENetPeer* peer = nullptr;
    std::shared_ptr<PeerLink> link;
    TokenBucket frames;
    TokenBucket bytes;
    TokenBucket controlFrames;
    TokenBucket controlBytes;
    Clock::time_point connectedAt{};
    bool seenFrame = false;
    bool disconnecting = false;
};

}  // namespace

LinkTransport::LinkTransport(std::shared_ptr<PeerLink> link, std::shared_ptr<void> owner)
    : link_(std::move(link)), owner_(std::move(owner)) {}

LinkTransport::~LinkTransport() { link_->requestClose(); }

struct EnetHostPump::Impl {
    std::shared_ptr<void> runtime;  // first member: released last, after the host is destroyed
    EnetServerOptions options;
    bool serverMode = true;
    std::shared_ptr<const sdk::IWallClock> clock;
    LinkLimits limits;
    ENetHost* host = nullptr;
    std::string endpoint;
    std::unordered_map<ENetPeer*, PumpPeer> peers;  // pump thread only once started
    sdk::BlockingQueue<std::unique_ptr<sdk::proto::ITransport>> accepted;
    std::atomic<std::size_t> peerCount{0};
    std::atomic<std::uint64_t> wireSent{0};
    std::atomic<std::uint64_t> wireReceived{0};
    bool started = false;
    sdk::StoppableThread thread;

    ~Impl() {
        thread.stop();
        if (host != nullptr) enet_host_destroy(host);
    }

    void configurePeer(ENetPeer* peer) const {
        const auto idle = static_cast<enet_uint32>(std::max<std::int64_t>(options.idleTimeout.count(), 1));
        enet_peer_timeout(peer, ENET_PEER_TIMEOUT_LIMIT, idle, idle);
        enet_peer_ping_interval(peer, static_cast<enet_uint32>(std::max<std::int64_t>(options.pingInterval.count(), 1)));
    }

    PumpPeer makePeer(ENetPeer* peer, std::shared_ptr<PeerLink> link) const {
        PumpPeer pp;
        pp.peer = peer;
        pp.link = std::move(link);
        pp.connectedAt = clock->now();
        pp.frames.init(options.maxFramesPerSecond, options.maxFramesPerSecond, pp.connectedAt);
        pp.bytes.init(options.maxBytesPerSecond, static_cast<double>(options.maxReliableFrameBytes), pp.connectedAt);
        pp.controlFrames.init(options.maxControlFramesPerSecond, options.maxControlFramesPerSecond, pp.connectedAt);
        pp.controlBytes.init(options.maxControlBytesPerSecond, static_cast<double>(kMaxControlFrameBytes), pp.connectedAt);
        return pp;
    }

    void refresh() { peerCount.store(peers.size()); }

    // The peer is dropped now, without waiting for the far end.
    void dropNow(std::unordered_map<ENetPeer*, PumpPeer>::iterator it, DisconnectReason reason) {
        enet_peer_disconnect_now(it->second.peer, static_cast<enet_uint32>(reason));
        it->second.link->markClosed(DisconnectReason::Normal);
        peers.erase(it);
        refresh();
    }

    void onConnect(ENetEvent& ev) {
        if (!serverMode) {
            auto it = peers.find(ev.peer);
            if (it != peers.end()) it->second.link->markConnected();
            return;
        }
        if (peers.size() >= options.maxPeers) {
            enet_peer_disconnect_now(ev.peer, static_cast<enet_uint32>(DisconnectReason::ServerFull));
            return;
        }
        configurePeer(ev.peer);
        auto link = std::make_shared<PeerLink>(limits, describePeer(ev.peer->address));
        link->markConnected();
        peers.emplace(ev.peer, makePeer(ev.peer, link));
        refresh();
        auto transport = std::make_unique<LinkTransport>(std::move(link));
        (void)accepted.push(std::move(transport));  // closed or full: the transport's destructor closes the link
    }

    void onDisconnect(ENetEvent& ev) {
        auto it = peers.find(ev.peer);
        if (it == peers.end()) return;
        it->second.link->markClosed(static_cast<DisconnectReason>(ev.data));
        peers.erase(it);
        refresh();
    }

    void onReceive(ENetEvent& ev) {
        std::unique_ptr<ENetPacket, decltype(&enet_packet_destroy)> packet(ev.packet, enet_packet_destroy);
        auto it = peers.find(ev.peer);
        if (it == peers.end()) return;
        PumpPeer& pp = it->second;
        PeerLink& link = *pp.link;
        pp.seenFrame = true;
        const std::size_t size = packet->dataLength;
        const bool control = ev.channelID == kEnetChannelControl;
        const bool reliable = ev.channelID == kEnetChannelReliable;
        if (!control && !reliable && ev.channelID != kEnetChannelUnreliable) {
            link.countDropped();
            return;
        }
        const std::size_t limit = control ? kMaxControlFrameBytes
                                          : (reliable ? options.maxReliableFrameBytes : options.maxUnreliableFrameBytes);
        if (size == 0 || size > limit || !admit(control ? pp.controlFrames : pp.frames, control ? pp.controlBytes : pp.bytes, size)) {
            link.countDropped();
            return;
        }
        link.countBytesIn(size);
        const auto* data = reinterpret_cast<const std::byte*>(packet->data);
        std::vector<std::byte> bytes(data, data + size);
        if (control) {
            if (link.deliverControl(std::move(bytes)) == sdk::BlockingQueue<std::vector<std::byte>>::PushResult::Full)
                dropNow(it, DisconnectReason::Backlog);
            return;
        }
        InboundFrame frame{std::move(bytes), reliable ? sdk::proto::Delivery::Reliable : sdk::proto::Delivery::Unreliable};
        if (link.deliver(std::move(frame)) == PeerLink::GamePush::Full) {
            if (reliable) dropNow(it, DisconnectReason::Backlog);
            else link.countDropped();
        }
    }

    // Takes one frame's worth from both buckets, or nothing when either is short.
    bool admit(TokenBucket& frames, TokenBucket& bytes, std::size_t size) const {
        if (!frames.enabled && !bytes.enabled) return true;
        const auto now = clock->now();
        frames.refill(now);
        bytes.refill(now);
        if ((frames.enabled && frames.tokens < 1.0) || (bytes.enabled && bytes.tokens < static_cast<double>(size))) return false;
        if (frames.enabled) frames.tokens -= 1.0;
        if (bytes.enabled) bytes.tokens -= static_cast<double>(size);
        return true;
    }

    void sendQueued(PumpPeer& pp) {
        std::deque<OutboundFrame> frames = pp.link->takeOutbound();
        for (OutboundFrame& frame : frames) {
            const bool unreliable = frame.channel == FrameChannel::GameUnreliable;
            const enet_uint32 flags = unreliable ? ENET_PACKET_FLAG_UNRELIABLE_FRAGMENT : ENET_PACKET_FLAG_RELIABLE;
            ENetPacket* packet = enet_packet_create(frame.bytes.data(), frame.bytes.size(), flags);
            if (packet == nullptr) {
                pp.link->countDropped();
                continue;
            }
            const std::uint8_t channel = frame.channel == FrameChannel::Control    ? kEnetChannelControl
                                         : frame.channel == FrameChannel::GameReliable ? kEnetChannelReliable
                                                                                       : kEnetChannelUnreliable;
            if (enet_peer_send(pp.peer, channel, packet) != 0) {
                enet_packet_destroy(packet);
                pp.link->countDropped();
                continue;
            }
            pp.link->countBytesOut(frame.bytes.size());
        }
    }

    void servicePeers(bool stopping) {
        std::vector<std::pair<ENetPeer*, bool>> gone;  // peer, whether to disconnect it for being silent
        for (auto& [peer, pp] : peers) {
            // Read before draining: a frame queued before the close request is then always flushed before the disconnect.
            const bool closing = stopping || pp.link->closeRequested();
            sendQueued(pp);
            if (!pp.disconnecting && closing) {
                pp.disconnecting = true;
                enet_peer_disconnect_later(peer, static_cast<enet_uint32>(DisconnectReason::Normal));
            }
            if (serverMode && !stopping && !pp.seenFrame && options.unauthenticatedTimeout.count() > 0 &&
                clock->now() - pp.connectedAt >= options.unauthenticatedTimeout) {
                gone.emplace_back(peer, true);
                continue;
            }
            if (enet_peer_get_state(peer) == ENET_PEER_STATE_DISCONNECTED) {
                gone.emplace_back(peer, false);
                continue;
            }
            pp.link->setLinkQuality(
                peer->roundTripTime,
                static_cast<std::uint32_t>(static_cast<std::uint64_t>(peer->packetLoss) * 1000 / ENET_PEER_PACKET_LOSS_SCALE));
        }
        for (const auto& [peer, silent] : gone) {
            auto it = peers.find(peer);
            if (it == peers.end()) continue;
            if (silent) {
                dropNow(it, DisconnectReason::Unauthenticated);
            } else {
                it->second.link->markClosed(DisconnectReason::Normal);
                peers.erase(it);
                refresh();
            }
        }
    }

    void accountWire() {
        wireSent.fetch_add(host->totalSentData);
        wireReceived.fetch_add(host->totalReceivedData);
        host->totalSentData = 0;
        host->totalReceivedData = 0;
    }

    void run(const sdk::StopToken& stop) {
        bool lingering = false;
        Clock::time_point lingerEnd{};
        for (;;) {
            const bool stopping = stop.stopRequested();
            if (stopping && !lingering) {
                lingering = true;
                lingerEnd = Clock::now() + options.disconnectLinger;
            }
            servicePeers(stopping);
            if (stopping && (peers.empty() || Clock::now() >= lingerEnd)) break;
            if (!serverMode && peers.empty()) break;

            ENetEvent ev;
            int result = enet_host_service(host, &ev, kPollMs);
            if (result < 0) std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
            while (result > 0) {
                switch (ev.type) {
                    case ENET_EVENT_TYPE_CONNECT: onConnect(ev); break;
                    case ENET_EVENT_TYPE_RECEIVE: onReceive(ev); break;
                    case ENET_EVENT_TYPE_DISCONNECT:
                    case ENET_EVENT_TYPE_DISCONNECT_TIMEOUT: onDisconnect(ev); break;
                    default: break;
                }
                result = enet_host_service(host, &ev, 0);
            }
            accountWire();
        }
        enet_host_flush(host);
        accountWire();
        for (auto& [peer, pp] : peers) pp.link->markClosed(DisconnectReason::Normal);
        peers.clear();
        refresh();
        enet_host_destroy(host);
        host = nullptr;
    }
};

EnetHostPump::EnetHostPump(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EnetHostPump::~EnetHostPump() { stop(); }

sdk::Result<std::shared_ptr<EnetHostPump>> EnetHostPump::createServer(const EnetServerOptions& options) {
    auto runtime = acquireEnetRuntime();
    if (runtime == nullptr) return makeError(ErrorCode::IoError, "ENet failed to initialize");

    ENetAddressType type = ENET_ADDRESS_TYPE_ANY;
    ENetAddress address{};
    enet_address_build_any(&address, ENET_ADDRESS_TYPE_IPV6);
    address.port = options.port;
    const bool dualStack = options.bindHost.empty() || options.bindHost == "::";
    if (!dualStack) {
        if (!resolveAddress(options.bindHost, options.port, address))
            return makeError(ErrorCode::IoError, "cannot resolve bind address '" + options.bindHost + "'");
        type = address.type;
    }

    const std::size_t maxPeers = std::max<std::size_t>(options.maxPeers, 1);
    ENetHost* host = createHost(type, &address, maxPeers + kSpareHostPeers, options);
    if (host == nullptr && dualStack) {  // no IPv6 on this machine
        enet_address_build_any(&address, ENET_ADDRESS_TYPE_IPV4);
        address.port = options.port;
        type = ENET_ADDRESS_TYPE_IPV4;
        host = createHost(type, &address, maxPeers + kSpareHostPeers, options);
    }
    if (host == nullptr)
        return makeError(ErrorCode::IoError, "cannot bind enet://" + options.bindHost + ":" + std::to_string(options.port));

    auto impl = std::make_unique<Impl>();
    impl->runtime = std::move(runtime);
    impl->options = options;
    impl->options.maxPeers = maxPeers;
    impl->clock = options.clock ? options.clock : std::make_shared<sdk::SteadyWallClock>();
    impl->limits = limitsOf(options);
    impl->host = host;
    ENetAddress bound{};
    if (enet_socket_get_address(host->socket, &bound) == 0) impl->endpoint = describePeer(bound);
    else impl->endpoint = describePeer(address);
    return std::shared_ptr<EnetHostPump>(new EnetHostPump(std::move(impl)));
}

sdk::Result<EnetHostPump::Client> EnetHostPump::createClient(const EnetLinkOptions& options, const std::string& host,
                                                             std::uint16_t port) {
    auto runtime = acquireEnetRuntime();
    if (runtime == nullptr) return makeError(ErrorCode::IoError, "ENet failed to initialize");

    ENetAddress target{};
    if (!resolveAddress(host, port, target))
        return makeError(ErrorCode::IoError, "cannot resolve host '" + host + "'");
    ENetHost* enetHost = createHost(target.type, nullptr, 1, options);
    if (enetHost == nullptr) return makeError(ErrorCode::IoError, "cannot create a client socket for " + host);

    auto impl = std::make_unique<Impl>();
    impl->runtime = std::move(runtime);
    static_cast<EnetLinkOptions&>(impl->options) = options;
    impl->options.maxPeers = 1;
    impl->options.maxFramesPerSecond = 0;
    impl->options.maxBytesPerSecond = 0;
    impl->options.maxControlFramesPerSecond = 0;
    impl->options.maxControlBytesPerSecond = 0;
    impl->options.unauthenticatedTimeout = std::chrono::milliseconds{0};
    impl->serverMode = false;
    impl->clock = options.clock ? options.clock : std::make_shared<sdk::SteadyWallClock>();
    impl->limits = limitsOf(options);
    impl->host = enetHost;

    ENetPeer* peer = enet_host_connect(enetHost, &target, kEnetChannelCount, 0);
    if (peer == nullptr) return makeError(ErrorCode::IoError, "cannot start a connection to " + host);
    impl->configurePeer(peer);
    auto link = std::make_shared<PeerLink>(impl->limits, describePeer(target));
    impl->peers.emplace(peer, impl->makePeer(peer, link));
    impl->refresh();

    std::shared_ptr<EnetHostPump> pump(new EnetHostPump(std::move(impl)));
    pump->start();
    return Client{std::move(pump), std::move(link)};
}

void EnetHostPump::start() {
    if (impl_->started) return;
    impl_->started = true;
    Impl* impl = impl_.get();
    impl->thread.start([impl](sdk::StopToken stop) { impl->run(stop); });
}

void EnetHostPump::stop() {
    if (!impl_) return;
    impl_->thread.stop();
    (void)impl_->accepted.closeAndDrain();
}

sdk::Result<std::unique_ptr<sdk::proto::ITransport>> EnetHostPump::accept(std::chrono::milliseconds timeout) {
    std::unique_ptr<sdk::proto::ITransport> next;
    switch (impl_->accepted.pop(next, timeout)) {
        case sdk::BlockingQueue<std::unique_ptr<sdk::proto::ITransport>>::PopResult::Item: return next;
        case sdk::BlockingQueue<std::unique_ptr<sdk::proto::ITransport>>::PopResult::Timeout:
            return std::unique_ptr<sdk::proto::ITransport>{};
        case sdk::BlockingQueue<std::unique_ptr<sdk::proto::ITransport>>::PopResult::Closed: break;
    }
    return makeError(ErrorCode::Disconnected, "server closed");
}

const std::string& EnetHostPump::endpoint() const { return impl_->endpoint; }

std::size_t EnetHostPump::peerCount() const { return impl_->peerCount.load(); }

EnetWireStats EnetHostPump::wireStats() const {
    EnetWireStats stats;
    stats.datagramBytesSent = impl_->wireSent.load();
    stats.datagramBytesReceived = impl_->wireReceived.load();
    return stats;
}

}  // namespace wpiemu::net::detail
