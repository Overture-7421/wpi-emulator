#pragma once

// The relay transports (docs: "Relay (Azure)"): they make the relay invisible to the game layer. MatchHost sees an
// ITransportServer whose accept() yields one ITransport per client, MatchClientSession an ITransportClient, exactly as
// with direct connect (enet_transport.h). Encryption is NOT done here: wrap each peer transport in SecureTransport, the
// same code as for direct connect; the relay only ever sees opaque game bytes.
//
// Wire use of the relay link (sdk/match/relay_messages.h). One ENet link to the relay per host and per client:
//   channel 0 (control)  plaintext RegisterRoom/JoinRoom and the relay's answers and PeerJoined/PeerLeft. No game frame ever
//                        travels on it.
//   channel 1 / 2        game frames, reliable / unreliable, with the delivery class kept end to end. A client sends them
//                        unprefixed; the host's frames carry the 2-byte little-endian peer id of the target in front, and the
//                        host receives each client's frames prefixed with the sender's id. The virtual peers add and strip
//                        that envelope, so a payload is one frame up to
//                        kRelayMaxReliablePacket - kRelayEnvelopeBytes reliable and kRelayMaxDatagram - kRelayEnvelopeBytes
//                        unreliable from the host (a client may use the full limits).
//
// Threading follows the ITransport contract (one thread may send() while another receive()s; close() from any thread
// wakes blocked calls). Each transport owns its threads and joins them on close() or destruction; ENet itself is only
// touched by the pump thread of the link (enet_pump.h). Per-peer transports may outlive the host transport that produced
// them and then report Disconnected.
//
// ENet types never appear in this header.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "wpiemu/net/enet_transport.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::net {

// ---- host --------------------------------------------------------------------------------------------------------------

struct RelayHostOptions {
    // The relay: a host name or IP literal (IPv6 without brackets) and port.
    std::string relayHost;
    std::uint16_t relayPort = 0;
    // The room the host registers: both come from the invite secret (net::roomId, net::hostAuth); relayHostOptionsFor()
    // fills them. Keeping the secret itself out of these options keeps the transport independent of the encryption layer.
    std::string roomId;
    sdk::match::relay::HostAuth hostAuth{};
    std::uint32_t protocolVersion = sdk::match::relay::kRelayProtocolVersion;

    // ENet settings of the link to the relay. The frame size limits are set by the transport (the relay protocol's limits
    // plus the envelope) whatever is given here; compression must match the relay (default on, like the relay).
    EnetLinkOptions link;

    // Bounds of one attempt: reaching the relay, then waiting for RoomRegistered / RoomRejected.
    std::chrono::milliseconds connectTimeout{5000};
    std::chrono::milliseconds registerTimeout{5000};

    // When the relay link is lost, every virtual peer reports Disconnected. With autoReregister the transport reconnects and
    // registers the SAME room with the same hostAuth, with exponential backoff (reconnectBackoffInitial, doubled up to
    // reconnectBackoffMax) until it succeeds or reconnectGiveUp has passed since the loss (the relay keeps a room without a
    // host for kRelayRoomTtlWithoutHostMs, so the default is twice that). A registration the relay REFUSES for good
    // (RoomTaken, BadAuth, BadVersion) also stops it. After a resume (RoomRegistered.resumed) the relay announces the
    // clients that are still there with PeerJoined, and accept() yields them as NEW virtual peers (same ids, a fresh
    // transport: the encryption handshake runs again). Without autoReregister the first loss is final.
    bool autoReregister = true;
    // A clean close() (or destruction) of a registered host sends CloseRoom, so the relay frees the room and drops its clients
    // at once. Turn it off to leave the room for its kRelayRoomTtlWithoutHostMs grace period, as a crashed host would.
    bool closeRoomOnClose = true;
    std::chrono::milliseconds reconnectBackoffInitial{250};
    std::chrono::milliseconds reconnectBackoffMax{4000};
    std::chrono::milliseconds reconnectGiveUp{2 * sdk::match::relay::kRelayRoomTtlWithoutHostMs};

    // Virtual peers that exist at once (the relay allows kRelayMaxPeersPerRoom). Further PeerJoined are ignored.
    std::size_t maxPeers = 16;
    // Received frames and bytes buffered per virtual peer, not yet read. A frame that does not fit is dropped and counted
    // when unreliable; a reliable one cannot be dropped without corrupting the stream, so that peer is closed.
    std::size_t maxQueuedFramesPerPeer = 4096;
    std::size_t maxQueuedBytesPerPeer = 4 * 1024 * 1024;
};

// Options that register the room of `secret` with the relay at relayHost:relayPort.
RelayHostOptions relayHostOptionsFor(const std::string& relayHost, std::uint16_t relayPort, const InviteSecret& secret);

// Registers a room with the relay and yields one virtual peer per client the relay announces (PeerJoined).
//
// Error mapping of listen(): InvalidArgument for a bad room id or relay address, Timeout when the relay is not reached or
// does not answer, AlreadyExists (RoomTaken: pick a new invite secret), InvalidArgument (BadAuth), OutOfRange (RelayFull),
// VersionMismatch (BadVersion); IoError/Disconnected for a link that fails.
//
// accept(): nullptr on timeout (also while the relay link is being re-established), an error with the reason once the
// transport is dead for good (close(), a lost relay without autoReregister, a refused or given-up re-registration).
// Peers announced but not yet accepted are returned first.
//
// Frames for a peer id that has no virtual peer yet are held briefly (a few frames, about 2 s) because the control and game
// channels are separate ENet streams and a client's first frame can overtake its PeerJoined; frames for ids that are
// unknown or left recently are dropped and counted. PeerJoined for an id that is already live replaces it: the old peer
// reports Disconnected and the new one is accepted (the relay only reuses an id after the old peer left).
//
// A virtual peer's close() is local: it discards its buffered frames and stops delivering, it does not remove the client
// from the relay (the relay has no per-client removal message; the client leaves when it closes its own link). Frames of that client are
// then dropped until it leaves.
//
// Not exposed through ITransport: broadcast. The relay accepts kRelayBroadcastPeer in the envelope to fan one frame out to
// every client; using it for snapshots would save upload bandwidth and is a possible future optimisation (it needs a
// broadcast call on the host transport, since each peer's SecureTransport encrypts with its own key today).
class RelayHostTransport final : public sdk::proto::ITransportServer {
public:
    explicit RelayHostTransport(RelayHostOptions options);
    ~RelayHostTransport() override;
    RelayHostTransport(const RelayHostTransport&) = delete;
    RelayHostTransport& operator=(const RelayHostTransport&) = delete;

    // Connects to the relay and registers the room; blocks up to connectTimeout + registerTimeout. InvalidState when
    // already listening. After a failure listen() may be called again.
    sdk::Status listen() override;
    // "relay://host:port/ROOMID" (what clients connect to once they also hold the invite).
    std::string endpoint() const override;
    sdk::Result<std::unique_ptr<sdk::proto::ITransport>> accept(std::chrono::milliseconds timeout) override;
    // Sends CloseRoom (see RelayHostOptions::closeRoomOnClose), disconnects every virtual peer, closes the relay link and
    // joins the threads. Idempotent.
    void close() override;

    // True while the room is registered on a live relay link.
    bool registered() const;
    // Virtual peers that exist now (not yet left or closed).
    std::size_t peerCount() const;
    // Successful re-registrations after a lost relay link.
    std::size_t reregisterCount() const;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

// ---- client ------------------------------------------------------------------------------------------------------------

struct RelayClientOptions {
    // ENet settings of the link to the relay (defaults match the relay).
    EnetClientOptions link;
    std::uint32_t protocolVersion = sdk::match::relay::kRelayProtocolVersion;
};

// Connects to relay://host:port/ROOMID: reaches the relay, joins the room and returns a transport that carries the game
// frames to the host. The whole call is bounded by `timeout`.
//
// Errors: InvalidArgument for a bad or enet:// endpoint, Timeout when the relay is not reached or does not answer in time,
// NotFound (no such room), OutOfRange (room full), VersionMismatch (bad version), InvalidState (locked out),
// Disconnected (the host is not connected to the room, or the relay hung up); the message says which.
//
// The returned transport sends frames unprefixed (reliable with send(), unreliable with sendUnreliable()) and receives the
// host's frames. It watches the control channel: a PeerLeft (the relay ending this client) makes it report Disconnected,
// as does the relay link going away. describe() is "relay://host:port/ROOMID(peer N)".
class RelayClientTransport final : public sdk::proto::ITransportClient {
public:
    explicit RelayClientTransport(RelayClientOptions options = {});
    ~RelayClientTransport() override;
    RelayClientTransport(const RelayClientTransport&) = delete;
    RelayClientTransport& operator=(const RelayClientTransport&) = delete;

    sdk::Result<std::unique_ptr<sdk::proto::ITransport>> connect(const std::string& endpoint,
                                                                 std::chrono::milliseconds timeout) override;

private:
    RelayClientOptions options_;
};

}  // namespace wpiemu::net
