#pragma once

// TEST ONLY: a small relay that speaks the relay protocol (sdk/match/relay_messages.h) on the real ENet loopback, so the
// relay transports can be tested without the real relay server. It follows the room rules of the real relay as far as the
// transports depend on them: RegisterRoom / RoomRegistered / RoomRejected (RoomTaken while the host is live, BadAuth while
// it is away, RelayFull, BadVersion), re-registering the same hostAuth resumes the room (RoomRegistered{resumed} and a
// PeerJoined for every client still there), JoinRoom / JoinResult (NoSuchRoom, HostAbsent, RoomFull, BadVersion), PeerJoined
// / PeerLeft to the host (PeerLeft only while the host is present), the 2-byte envelope on game frames, and the delivery
// class kept per frame. It never parses a game frame, and it records every byte it sees so a test can assert that a marker
// never crossed it in the clear. The class adds test controls: dropping a connection, ending a client, expiring a room,
// injecting frames or control messages at the host, and overriding the answer to RegisterRoom.
//
// One thread polls every connection with zero timeouts under one mutex; the controls take the same mutex.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/control_link.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/sdk/ipc/delivery.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::net::test {

namespace relay = sdk::match::relay;

struct FakeRelayOptions {
    std::uint16_t port = 0;  // 0: an ephemeral port
    std::size_t maxPeersPerRoom = relay::kRelayMaxPeersPerRoom;
    std::size_t maxRooms = relay::kRelayMaxRooms;
    std::uint32_t protocolVersion = relay::kRelayProtocolVersion;
};

struct FakeRelayTraffic {
    std::uint64_t toHostReliable = 0;
    std::uint64_t toHostUnreliable = 0;
    std::uint64_t toClientReliable = 0;
    std::uint64_t toClientUnreliable = 0;
    std::uint64_t malformedControl = 0;     // control frames that are not relay messages
    std::uint64_t unexpectedControl = 0;  // relay messages a client or host may not send; 0 unless a transport misuses the channel
    std::uint64_t droppedNoHost = 0;
    std::uint64_t registrations = 0;
    std::uint64_t joins = 0;
};

class FakeRelay {
public:
    explicit FakeRelay(FakeRelayOptions options = {}) : options_(options) {
        EnetServerOptions server;
        server.bindHost = "127.0.0.1";
        server.port = options.port;
        server.maxPeers = 32;
        server.unauthenticatedTimeout = std::chrono::milliseconds(0);
        server.maxFramesPerSecond = 0;
        server.maxBytesPerSecond = 0;
        server.maxControlFramesPerSecond = 0;
        server.maxControlBytesPerSecond = 0;
        // The relay forwards a client's frame with the envelope in front.
        server.maxReliableFrameBytes = relay::kRelayMaxReliablePacket + relay::kRelayEnvelopeBytes;
        server.maxUnreliableFrameBytes = relay::kRelayMaxDatagram + relay::kRelayEnvelopeBytes;
        server_ = std::make_unique<EnetServerTransport>(server);
        const auto status = server_->listen();
        if (!status.ok()) {
            listenError_ = status.error().message;
            return;
        }
        const std::string endpoint = server_->endpoint();
        port_ = static_cast<std::uint16_t>(std::stoi(endpoint.substr(endpoint.rfind(':') + 1)));
        thread_ = std::thread([this] { run(); });
    }

    ~FakeRelay() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
        std::lock_guard lock(mutex_);
        conns_.clear();
        server_->close();
    }
    FakeRelay(const FakeRelay&) = delete;
    FakeRelay& operator=(const FakeRelay&) = delete;

    // Empty when the relay is listening.
    const std::string& listenError() const { return listenError_; }
    std::uint16_t port() const { return port_; }
    std::string host() const { return "127.0.0.1"; }
    std::string endpoint(const std::string& roomId) const {
        return "relay://127.0.0.1:" + std::to_string(port_) + "/" + roomId;
    }

    // ---- inspection
    bool hasRoom(const std::string& roomId) const {
        std::lock_guard lock(mutex_);
        return rooms_.count(roomId) != 0;
    }
    bool hostPresent(const std::string& roomId) const {
        std::lock_guard lock(mutex_);
        const auto room = rooms_.find(roomId);
        return room != rooms_.end() && room->second.host.has_value();
    }
    std::vector<relay::RelayPeerId> clientIds(const std::string& roomId) const {
        std::lock_guard lock(mutex_);
        std::vector<relay::RelayPeerId> ids;
        if (const auto room = rooms_.find(roomId); room != rooms_.end())
            for (const auto& entry : room->second.clients) ids.push_back(entry.first);
        return ids;
    }
    FakeRelayTraffic traffic() const {
        std::lock_guard lock(mutex_);
        return traffic_;
    }
    bool sawBytes(const std::string& marker) const {
        std::lock_guard lock(mutex_);
        return observed_.find(marker) != std::string::npos;
    }

    // ---- controls
    // Answer every RegisterRoom with this rejection (nullopt: normal rules).
    void overrideRegisterRejection(std::optional<relay::RoomRejectReason> reason) {
        std::lock_guard lock(mutex_);
        registerOverride_ = reason;
    }
    // Disconnect the host's connection; the room and its clients stay (the relay keeps a room for a while).
    void dropHost(const std::string& roomId) {
        std::lock_guard lock(mutex_);
        dropHostLocked(roomId);
    }
    // Drop the host and a client in one step: the client leaves while the host is away, so nobody is told.
    void dropHostAndClient(const std::string& roomId, relay::RelayPeerId peer) {
        std::lock_guard lock(mutex_);
        dropHostLocked(roomId);
        dropClientLocked(roomId, peer);
    }
    void dropClient(const std::string& roomId, relay::RelayPeerId peer) {
        std::lock_guard lock(mutex_);
        dropClientLocked(roomId, peer);
    }
    // Tell a client it is over (PeerLeft on its control channel) without disconnecting it.
    void endClient(const std::string& roomId, relay::RelayPeerId peer) {
        std::lock_guard lock(mutex_);
        if (Conn* conn = clientConn(roomId, peer)) sendMessage(*conn, relay::PeerLeft{peer, relay::PeerLeftReason::Disconnected});
    }
    // The room's TTL ran out: the room closes and every connection of it is closed.
    void expireRoom(const std::string& roomId) {
        std::lock_guard lock(mutex_);
        const auto room = rooms_.find(roomId);
        if (room == rooms_.end()) return;
        if (room->second.host) closeConn(*room->second.host);
        for (const auto& entry : room->second.clients) closeConn(entry.second);
        rooms_.erase(room);
    }
    // Inject a raw (enveloped) game frame as if a client had sent it, or a control message, at the host.
    void injectGameToHost(const std::string& roomId, relay::RelayPeerId peer, const std::vector<std::byte>& payload,
                          sdk::proto::Delivery delivery = sdk::proto::Delivery::Reliable) {
        std::lock_guard lock(mutex_);
        const auto room = rooms_.find(roomId);
        if (room == rooms_.end() || !room->second.host) return;
        forwardToHost(conns_.at(*room->second.host), peer, payload, delivery);
    }
    void injectControlToHost(const std::string& roomId, const sdk::proto::Message& message) {
        std::lock_guard lock(mutex_);
        const auto room = rooms_.find(roomId);
        if (room == rooms_.end() || !room->second.host) return;
        sendMessage(*conns_.at(*room->second.host), message);
    }

private:
    enum class Role { Unregistered, Host, Client };

    struct Conn {
        std::uint64_t id = 0;
        std::unique_ptr<sdk::proto::ITransport> link;
        IControlLink* control = nullptr;
        Role role = Role::Unregistered;
        std::string room;
        relay::RelayPeerId peer = 0;
        bool dead = false;
    };

    struct Room {
        relay::HostAuth auth{};
        std::optional<std::uint64_t> host;
        std::map<relay::RelayPeerId, std::uint64_t> clients;
    };

    using Bytes = std::vector<std::byte>;

    void run() {
        while (!stop_.load()) {
            bool busy = false;
            {
                std::lock_guard lock(mutex_);
                auto accepted = server_->accept(std::chrono::milliseconds(0));
                if (accepted.ok() && accepted.value()) {
                    auto conn = std::make_unique<Conn>();
                    conn->id = ++nextConn_;
                    conn->link = std::move(accepted).value();
                    conn->control = controlLinkOf(*conn->link);
                    conns_[conn->id] = std::move(conn);
                    busy = true;
                }
                for (auto& entry : conns_) busy = poll(*entry.second) || busy;
                for (auto it = conns_.begin(); it != conns_.end();) {
                    if (it->second->dead) {
                        leave(*it->second);
                        it = conns_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            if (!busy) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    bool poll(Conn& conn) {
        if (conn.dead) return false;
        bool busy = false;
        Bytes frame;
        for (int i = 0; i < 64 && !conn.dead; ++i) {
            auto got = conn.control->receiveControl(frame, std::chrono::milliseconds(0));
            if (!got) {
                conn.dead = true;
                break;
            }
            if (!got.value()) break;
            busy = true;
            observe(frame);
            onControl(conn, frame);
        }
        sdk::proto::Delivery delivery = sdk::proto::Delivery::Reliable;
        for (int i = 0; i < 256 && !conn.dead; ++i) {
            auto got = conn.control->receiveGame(frame, delivery, std::chrono::milliseconds(0));
            if (!got) {
                conn.dead = true;
                break;
            }
            if (!got.value()) break;
            busy = true;
            observe(frame);
            onGame(conn, frame, delivery);
        }
        return busy;
    }

    void observe(const Bytes& frame) {
        if (observed_.size() < (32u << 20)) observed_.append(reinterpret_cast<const char*>(frame.data()), frame.size());
    }

    void sendMessage(Conn& conn, const sdk::proto::Message& message) {
        Bytes frame;
        if (ipc::BinaryMessageCodec().encode(message, frame)) (void)conn.control->sendControl(frame);
    }

    void closeConn(std::uint64_t id) {
        const auto found = conns_.find(id);
        if (found == conns_.end()) return;
        found->second->link->close();
        found->second->dead = true;
    }

    Conn* clientConn(const std::string& roomId, relay::RelayPeerId peer) {
        const auto room = rooms_.find(roomId);
        if (room == rooms_.end()) return nullptr;
        const auto client = room->second.clients.find(peer);
        if (client == room->second.clients.end()) return nullptr;
        const auto conn = conns_.find(client->second);
        return conn == conns_.end() ? nullptr : conn->second.get();
    }

    void dropHostLocked(const std::string& roomId) {
        const auto room = rooms_.find(roomId);
        if (room == rooms_.end() || !room->second.host) return;
        const auto conn = conns_.find(*room->second.host);
        if (conn != conns_.end()) detachAndClose(*conn->second);
    }

    void dropClientLocked(const std::string& roomId, relay::RelayPeerId peer) {
        if (Conn* conn = clientConn(roomId, peer)) detachAndClose(*conn);
    }

    // Removes the connection from its room at once (like the relay does on a disconnect event, so a registration that
    // follows already sees the new state) and closes it.
    void detachAndClose(Conn& conn) {
        leave(conn);
        conn.room.clear();
        conn.role = Role::Unregistered;
        conn.link->close();
        conn.dead = true;
    }

    void onControl(Conn& conn, const Bytes& frame) {
        auto decoded = ipc::BinaryMessageCodec().decode(frame);
        if (!decoded) {
            ++traffic_.malformedControl;
            return;
        }
        if (const auto* reg = std::get_if<relay::RegisterRoom>(&decoded.value())) {
            onRegister(conn, *reg);
        } else if (const auto* join = std::get_if<relay::JoinRoom>(&decoded.value())) {
            onJoin(conn, *join);
        } else {
            ++traffic_.unexpectedControl;
        }
    }

    void onRegister(Conn& conn, const relay::RegisterRoom& reg) {
        if (conn.role != Role::Unregistered) {
            conn.dead = true;
            return;
        }
        if (registerOverride_) {
            sendMessage(conn, relay::RoomRejected{*registerOverride_});
            return;
        }
        if (reg.protocolVersion != options_.protocolVersion) {
            sendMessage(conn, relay::RoomRejected{relay::RoomRejectReason::BadVersion});
            return;
        }
        if (!relay::isValidRoomId(reg.roomId)) {
            conn.dead = true;
            return;
        }
        auto found = rooms_.find(reg.roomId);
        if (found == rooms_.end()) {
            if (rooms_.size() >= options_.maxRooms) {
                sendMessage(conn, relay::RoomRejected{relay::RoomRejectReason::RelayFull});
                return;
            }
            Room room;
            room.auth = reg.hostAuth;
            room.host = conn.id;
            rooms_[reg.roomId] = room;
            conn.role = Role::Host;
            conn.room = reg.roomId;
            ++traffic_.registrations;
            sendMessage(conn, relay::RoomRegistered{false, static_cast<std::uint16_t>(options_.maxPeersPerRoom)});
            return;
        }
        Room& room = found->second;
        if (room.auth != reg.hostAuth) {
            sendMessage(conn, relay::RoomRejected{room.host ? relay::RoomRejectReason::RoomTaken : relay::RoomRejectReason::BadAuth});
            return;
        }
        if (room.host) closeConn(*room.host);  // takeover of a host the relay still believes live
        room.host = conn.id;
        conn.role = Role::Host;
        conn.room = reg.roomId;
        ++traffic_.registrations;
        sendMessage(conn, relay::RoomRegistered{true, static_cast<std::uint16_t>(options_.maxPeersPerRoom)});
        for (const auto& entry : room.clients) sendMessage(conn, relay::PeerJoined{entry.first});
    }

    void onJoin(Conn& conn, const relay::JoinRoom& join) {
        if (conn.role != Role::Unregistered) {
            conn.dead = true;
            return;
        }
        const auto reject = [&](relay::JoinRoomFailure why) {
            sendMessage(conn, relay::JoinResult{false, why, relay::kRelayBroadcastPeer});
        };
        if (join.protocolVersion != options_.protocolVersion) return reject(relay::JoinRoomFailure::BadVersion);
        const auto found = rooms_.find(join.roomId);
        if (found == rooms_.end()) return reject(relay::JoinRoomFailure::NoSuchRoom);
        Room& room = found->second;
        if (!room.host) return reject(relay::JoinRoomFailure::HostAbsent);
        if (room.clients.size() >= options_.maxPeersPerRoom) return reject(relay::JoinRoomFailure::RoomFull);
        relay::RelayPeerId peer = 1;
        while (room.clients.count(peer) != 0) ++peer;
        room.clients[peer] = conn.id;
        conn.role = Role::Client;
        conn.room = join.roomId;
        conn.peer = peer;
        ++traffic_.joins;
        sendMessage(conn, relay::JoinResult{true, relay::JoinRoomFailure::None, peer});
        sendMessage(*conns_.at(*room.host), relay::PeerJoined{peer});
    }

    void forwardToHost(const std::unique_ptr<Conn>& host, relay::RelayPeerId from, const Bytes& payload,
                       sdk::proto::Delivery delivery) {
        Bytes out;
        out.push_back(static_cast<std::byte>(from & 0xFF));
        out.push_back(static_cast<std::byte>(from >> 8));
        out.insert(out.end(), payload.begin(), payload.end());
        if (delivery == sdk::proto::Delivery::Unreliable) {
            ++traffic_.toHostUnreliable;
            (void)host->link->sendUnreliable(out);
        } else {
            ++traffic_.toHostReliable;
            (void)host->link->send(out);
        }
    }

    void sendGame(Conn& to, const std::byte* data, std::size_t size, sdk::proto::Delivery delivery) {
        const std::span<const std::byte> frame(data, size);
        if (delivery == sdk::proto::Delivery::Unreliable) {
            ++traffic_.toClientUnreliable;
            (void)to.link->sendUnreliable(frame);
        } else {
            ++traffic_.toClientReliable;
            (void)to.link->send(frame);
        }
    }

    void onGame(Conn& conn, const Bytes& frame, sdk::proto::Delivery delivery) {
        const auto found = rooms_.find(conn.room);
        if (conn.role == Role::Unregistered || found == rooms_.end()) return;
        Room& room = found->second;
        if (conn.role == Role::Client) {
            if (!room.host) {
                ++traffic_.droppedNoHost;
                return;
            }
            forwardToHost(conns_.at(*room.host), conn.peer, frame, delivery);
            return;
        }
        if (frame.size() <= relay::kRelayEnvelopeBytes) return;
        const auto target = static_cast<relay::RelayPeerId>(static_cast<unsigned>(frame[0]) | (static_cast<unsigned>(frame[1]) << 8));
        const std::byte* payload = frame.data() + relay::kRelayEnvelopeBytes;
        const std::size_t size = frame.size() - relay::kRelayEnvelopeBytes;
        if (target == relay::kRelayBroadcastPeer) {
            for (const auto& entry : room.clients) sendGame(*conns_.at(entry.second), payload, size, delivery);
        } else if (const auto client = room.clients.find(target); client != room.clients.end()) {
            sendGame(*conns_.at(client->second), payload, size, delivery);
        }
    }

    void leave(Conn& conn) {
        const auto found = rooms_.find(conn.room);
        if (found == rooms_.end()) return;
        Room& room = found->second;
        if (conn.role == Role::Host) {
            if (room.host == conn.id) room.host.reset();
        } else if (conn.role == Role::Client) {
            room.clients.erase(conn.peer);
            if (room.host)
                if (const auto host = conns_.find(*room.host); host != conns_.end())
                    sendMessage(*host->second, relay::PeerLeft{conn.peer, relay::PeerLeftReason::Disconnected});
        }
    }

    const FakeRelayOptions options_;
    std::unique_ptr<EnetServerTransport> server_;
    std::uint16_t port_ = 0;
    std::string listenError_;
    std::thread thread_;
    std::atomic<bool> stop_{false};

    mutable std::mutex mutex_;
    std::map<std::uint64_t, std::unique_ptr<Conn>> conns_;
    std::map<std::string, Room> rooms_;
    std::uint64_t nextConn_ = 0;
    std::optional<relay::RoomRejectReason> registerOverride_;
    FakeRelayTraffic traffic_;
    std::string observed_;
};

}  // namespace wpiemu::net::test
