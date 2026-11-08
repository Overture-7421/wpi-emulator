#pragma once

// ENet transports for direct connect (enet://host:port): one EnetServerTransport accepts peers, each peer is an
// sdk::proto::ITransport; EnetClientTransport connects to one. The same adapter code serves the relay transports later.
//
// Threading. ENet is not thread-safe, so every ENet host is owned by ONE pump thread that the adapter starts (listen() or
// connect()) and joins (close() or destruction). All ENet calls happen on it. The ITransport objects talk to the pump only
// through thread-safe queues, so one thread may send() while another receive()s, close() works from any thread, and a
// per-peer transport may outlive the server that produced it (it then reports Disconnected). The pump polls about once a
// millisecond, which bounds the latency it adds.
//
// Channels (the numbering of the relay, sdk/match/relay_messages.h; channel 0 is the relay's control channel and carries
// nothing here):
//   1  reliable, ordered      send();            frames up to maxReliableFrameBytes (ENet fragments large ones)
//   2  unreliable, sequenced  sendUnreliable();  frames up to maxUnreliableFrameBytes; ENet's unreliable fragmentation
//                                                carries frames larger than one packet, and losing any fragment drops the
//                                                frame. An unreliable frame above the limit is REJECTED with
//                                                InvalidArgument, never silently upgraded to reliable (the relay drops a peer
//                                                that sends an oversized datagram, so direct connect behaves the same).
//
// Disconnects. A peer that disconnects or times out makes receive() return the frames it already delivered and then
// ErrorCode::Disconnected. close() and destruction disconnect politely: frames already handed to send() are flushed first.
//
// Frames are 1 byte or more: an empty frame is rejected with InvalidArgument (ENet's range coder reads past an empty
// packet buffer) and an empty frame received is dropped and counted.
//
// Hostile peers. Anything arriving on a channel other than 1 or 2, any frame over its size limit and any frame over the
// per-peer rate cap is dropped and counted (LinkStats::droppedFrames); a peer that sends no frame at all within
// unauthenticatedTimeout is disconnected; a peer whose unread backlog exceeds maxQueuedInboundFrames is disconnected.
// Datagrams that are not valid ENet are ignored by ENet. Encryption is not done here: wrap each peer in SecureTransport.
//
// ENet headers never appear in this header.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::net {

inline constexpr std::uint8_t kEnetChannelReliable = sdk::match::relay::kRelayChannelGameReliable;
inline constexpr std::uint8_t kEnetChannelUnreliable = sdk::match::relay::kRelayChannelGameUnreliable;
inline constexpr std::uint8_t kEnetChannelCount = sdk::match::relay::kRelayChannelCount;

// Settings common to both ends. Compression must be set the same on both ends (a mismatch makes the link unusable).
struct EnetLinkOptions {
    // Largest frame send() accepts and the receiver accepts (<= sdk::match::relay::kRelayMaxReliablePacket).
    std::size_t maxReliableFrameBytes = sdk::match::relay::kRelayMaxReliablePacket;
    // Largest frame sendUnreliable() accepts (<= sdk::match::relay::kRelayMaxDatagram).
    std::size_t maxUnreliableFrameBytes = sdk::match::relay::kRelayMaxDatagram;
    // Frames accepted by send() but not yet handed to ENet. Past it a reliable send fails with IoError and an unreliable
    // one is dropped and counted.
    std::size_t maxQueuedOutboundBytes = 8 * 1024 * 1024;
    // Received frames not yet read by receive(). Past it an unreliable frame is dropped and counted, and a reliable frame
    // (which cannot be dropped without corrupting the stream) disconnects the peer.
    std::size_t maxQueuedInboundFrames = 4096;
    // ENet bandwidth hints in bytes per second; 0 means unlimited.
    std::uint32_t incomingBandwidth = 0;
    std::uint32_t outgoingBandwidth = 0;
    // A peer that acknowledges nothing for this long is dropped.
    std::chrono::milliseconds idleTimeout{10000};
    // ENet keep-alive ping period; it also refreshes the round-trip estimate when no reliable traffic flows.
    std::chrono::milliseconds pingInterval{250};
    // On close or destruction, how long to wait for the polite disconnect to be acknowledged.
    std::chrono::milliseconds disconnectLinger{300};
    // ENet's range-coder compression.
    bool compression = true;
    // Time source for the rate cap and the unauthenticated timeout; null means the steady clock.
    std::shared_ptr<const sdk::IWallClock> clock;
};

struct EnetServerOptions : EnetLinkOptions {
    // Address to bind: an IPv4 or IPv6 literal or a host name. The default "::" listens on IPv6 and IPv4 together
    // (dual stack) and falls back to IPv4 only when the machine has no IPv6; "0.0.0.0" is IPv4 only.
    std::string bindHost = "::";
    // 0 asks the OS for a free port; endpoint() reports the one chosen.
    std::uint16_t port = 0;
    // Peers connected at once. A further peer connects and is dropped immediately.
    std::size_t maxPeers = 16;
    // The join gate's transport half: a peer that has sent no frame within this time is disconnected. 0 disables it.
    std::chrono::milliseconds unauthenticatedTimeout{2000};
    // Per-peer receive cap (token buckets holding one second's worth of credit). Frames over either cap are dropped and
    // counted. 0 disables the cap. The byte bucket never holds less than maxReliableFrameBytes, so a maximum frame fits.
    std::uint32_t maxFramesPerSecond = 2000;
    std::uint32_t maxBytesPerSecond = 1024 * 1024;
};

struct EnetClientOptions : EnetLinkOptions {};

// What ENet moved on the wire (UDP payload bytes, after compression) since the host started; for diagnostics and the
// compression measurement.
struct EnetWireStats {
    std::uint64_t datagramBytesSent = 0;
    std::uint64_t datagramBytesReceived = 0;
};

// Listens for ENet peers; accept() yields one ITransport per connected peer.
class EnetServerTransport final : public sdk::proto::ITransportServer {
public:
    explicit EnetServerTransport(EnetServerOptions options = {});
    ~EnetServerTransport() override;
    EnetServerTransport(const EnetServerTransport&) = delete;
    EnetServerTransport& operator=(const EnetServerTransport&) = delete;

    // Binds and starts the pump thread. IoError when the address cannot be bound; InvalidState when already listening.
    sdk::Status listen() override;
    // "enet://host:port" with the bound address and port after listen() (port 0 resolved), the configured one before.
    std::string endpoint() const override;
    // The next connected peer, nullptr on timeout, Disconnected once the server is closed.
    sdk::Result<std::unique_ptr<sdk::proto::ITransport>> accept(std::chrono::milliseconds timeout) override;
    // Disconnects every peer (their transports then report Disconnected), stops and joins the pump thread. Idempotent.
    void close() override;

    // Peers currently connected (including ones not yet accepted).
    std::size_t peerCount() const;
    EnetWireStats wireStats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Connects to an enet:// endpoint (an IPv4 or IPv6 literal or a host name, which prefers IPv4).
class EnetClientTransport final : public sdk::proto::ITransportClient {
public:
    explicit EnetClientTransport(EnetClientOptions options = {});
    ~EnetClientTransport() override;
    EnetClientTransport(const EnetClientTransport&) = delete;
    EnetClientTransport& operator=(const EnetClientTransport&) = delete;

    // InvalidArgument for a bad endpoint or a relay:// one, Timeout when no connection is made within `timeout`,
    // Disconnected when the server refuses (a full server accepts then drops the connection, so a refusal can also show up
    // as Disconnected from the first receive). The returned transport owns its own pump thread; destroying it disconnects
    // politely and joins the thread.
    sdk::Result<std::unique_ptr<sdk::proto::ITransport>> connect(const std::string& endpoint,
                                                                 std::chrono::milliseconds timeout) override;

private:
    EnetClientOptions options_;
};

}  // namespace wpiemu::net
