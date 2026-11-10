#include "wpiemu/relay/room_table.h"

#include <algorithm>
#include <unordered_map>
#include <utility>

#include "wpiemu/net/invite.h"

namespace wpiemu::relay {

namespace rl = sdk::match::relay;
using Clock = std::chrono::steady_clock;
using sdk::proto::Delivery;

namespace {

constexpr std::int64_t kMicro = 1'000'000;

// Appends an effect built in place. push_back(Alternative{...}) builds a temporary variant that GCC 14 at -O3 wrongly
// reports as maybe-uninitialized / free-nonheap-object when the vector reallocates; constructing in place avoids it.
template <class T, class... Args>
void addEffect(Effects& effects, Args&&... args) {
    effects.emplace_back(std::in_place_type<T>, std::forward<Args>(args)...);
}

// Credit in micro-bytes so that refills stay integer and deterministic: elapsed microseconds times bytes per second.
struct TokenBucket {
    bool enabled = false;
    std::int64_t tokens = 0;
    std::int64_t capacity = 0;
    std::int64_t rate = 0;
    Clock::time_point last{};

    void configure(std::uint32_t bytesPerSecond, std::uint32_t burstMs, std::size_t minCapacityBytes, Clock::time_point now) {
        enabled = bytesPerSecond > 0;
        rate = bytesPerSecond;
        const std::int64_t burst = static_cast<std::int64_t>(bytesPerSecond) * burstMs * 1000;
        capacity = std::max(burst, static_cast<std::int64_t>(minCapacityBytes) * kMicro);
        tokens = capacity;
        last = now;
    }

    bool take(std::size_t bytes, Clock::time_point now) {
        if (!enabled) return true;
        const std::int64_t elapsedUs = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count();
        if (elapsedUs > 0) {
            const std::int64_t fillUs = capacity / rate + 1;
            tokens = std::min(capacity, tokens + std::min(elapsedUs, fillUs) * rate);
            last = now;
        }
        const std::int64_t cost = static_cast<std::int64_t>(bytes) * kMicro;
        if (tokens < cost) return false;
        tokens -= cost;
        return true;
    }
};

struct Room {
    std::string id;
    rl::HostAuth auth{};
    std::optional<ConnId> host;
    Clock::time_point hostGoneAt{};
    std::map<rl::RelayPeerId, ConnId> clients;  // ordered: deterministic broadcast and expiry order
    TokenBucket bucket;
};

struct Conn {
    std::string room;
    Role role = Role::None;
    rl::RelayPeerId peerId = rl::kRelayBroadcastPeer;
    TokenBucket bucket;
    Clock::time_point strikeWindowStart{};
    std::uint32_t strikes = 0;
};

struct SourceRecord {
    Clock::time_point windowStart{};
    Clock::time_point lockedUntil{};
    Clock::time_point lastSeen{};
    std::uint32_t failures = 0;
};

std::chrono::milliseconds ms(std::uint32_t value) { return std::chrono::milliseconds(value); }

rl::PeerLeftReason peerLeftReasonOf(DropReason reason) {
    return reason == DropReason::Oversized ? rl::PeerLeftReason::Oversized : rl::PeerLeftReason::RateLimited;
}

const char* registerRejectName(rl::RoomRejectReason reason) {
    switch (reason) {
        case rl::RoomRejectReason::RoomTaken: return "room_taken";
        case rl::RoomRejectReason::BadAuth: return "bad_auth";
        case rl::RoomRejectReason::RelayFull: return "relay_full";
        case rl::RoomRejectReason::BadVersion: return "bad_version";
    }
    return "unknown";
}

const char* joinFailureName(rl::JoinRoomFailure reason) {
    switch (reason) {
        case rl::JoinRoomFailure::None: return "none";
        case rl::JoinRoomFailure::NoSuchRoom: return "no_such_room";
        case rl::JoinRoomFailure::RoomFull: return "room_full";
        case rl::JoinRoomFailure::BadVersion: return "bad_version";
        case rl::JoinRoomFailure::LockedOut: return "locked_out";
        case rl::JoinRoomFailure::HostAbsent: return "host_absent";
    }
    return "unknown";
}

bool versionSupported(std::uint32_t version) {
    return version >= rl::kMinCompatibleRelayProtocolVersion && version <= rl::kRelayProtocolVersion;
}

}  // namespace

struct RoomTable::Impl {
    Impl(RoomTableConfig c, std::shared_ptr<const sdk::IWallClock> k) : config(std::move(c)), clock(std::move(k)) {
        config.maxPeersPerRoom = std::clamp<std::size_t>(config.maxPeersPerRoom, 1, 0xFFFE);
        if (config.hostBytesPerSecond == 0) config.hostBytesPerSecond = config.peerBytesPerSecond * static_cast<std::uint32_t>(config.maxPeersPerRoom);
        if (config.roomBytesPerSecond == 0) config.roomBytesPerSecond = 2 * config.peerBytesPerSecond * static_cast<std::uint32_t>(config.maxPeersPerRoom);
        if (config.peerBytesPerSecond == 0) {
            config.hostBytesPerSecond = 0;
            config.roomBytesPerSecond = 0;
        }
    }

    RoomTableConfig config;
    std::shared_ptr<const sdk::IWallClock> clock;
    std::map<std::string, Room> rooms;
    std::unordered_map<ConnId, Conn> conns;
    std::map<std::string, SourceRecord> sources;
    Clock::time_point nextSourceSweep{};
    RelayStats counters;
    std::vector<RoomEvent> events;

    Clock::time_point now() const { return clock->now(); }

    void event(RoomEvent::Kind kind, ConnId conn, const std::string& room, rl::RelayPeerId peer, std::string detail) {
        if (events.size() >= config.maxPendingEvents) return;
        events.push_back(RoomEvent{kind, conn, rl::isValidRoomId(room) ? room : std::string(), peer, std::move(detail)});
    }

    // ---- lockout ----
    bool lockedOut(const std::string& source) const {
        if (source.empty() || config.lockoutMaxFailures == 0) return false;
        const auto it = sources.find(source);
        return it != sources.end() && it->second.lockedUntil > clock->now();
    }

    void noteFailure(const std::string& source, ConnId conn) {
        if (source.empty() || config.lockoutMaxFailures == 0) return;
        const Clock::time_point t = now();
        auto it = sources.find(source);
        if (it == sources.end()) {
            if (sources.size() >= config.maxTrackedSources && !makeRoomForSource(t)) {
                ++counters.lockoutTableFull;
                return;
            }
            it = sources.emplace(source, SourceRecord{t, {}, t, 0}).first;
        }
        SourceRecord& record = it->second;
        record.lastSeen = t;
        if (t - record.windowStart >= ms(config.lockoutWindowMs)) {
            record.windowStart = t;
            record.failures = 0;
        }
        if (++record.failures >= config.lockoutMaxFailures) {
            record.lockedUntil = t + ms(config.lockoutDurationMs);
            record.failures = 0;
            record.windowStart = t;
            ++counters.lockouts;
            event(RoomEvent::Kind::LockedOut, conn, {}, 0, {});
        }
    }

    // Frees one slot: expired records first, else the least recently seen record that is not locked out.
    bool makeRoomForSource(Clock::time_point t) {
        sweepSources(t, true);
        if (sources.size() < config.maxTrackedSources) return true;
        auto victim = sources.end();
        for (auto it = sources.begin(); it != sources.end(); ++it) {
            if (it->second.lockedUntil > t) continue;
            if (victim == sources.end() || it->second.lastSeen < victim->second.lastSeen) victim = it;
        }
        if (victim == sources.end()) return false;
        sources.erase(victim);
        return true;
    }

    void sweepSources(Clock::time_point t, bool force) {
        if (!force && t < nextSourceSweep) return;
        nextSourceSweep = t + std::chrono::seconds(10);
        for (auto it = sources.begin(); it != sources.end();) {
            const SourceRecord& r = it->second;
            if (r.lockedUntil <= t && t - r.windowStart >= ms(config.lockoutWindowMs)) it = sources.erase(it);
            else ++it;
        }
    }

    // ---- membership ----
    Effects violation(ConnId conn) {
        Effects effects = removeConn(conn, rl::PeerLeftReason::Disconnected);
        addEffect<CloseConnection>(effects, conn, DisconnectReason::ProtocolViolation);
        return effects;
    }

    Effects removeConn(ConnId conn, rl::PeerLeftReason reason) {
        Effects effects;
        const auto it = conns.find(conn);
        if (it == conns.end()) return effects;
        const Conn c = it->second;
        conns.erase(it);
        const auto roomIt = rooms.find(c.room);
        if (roomIt == rooms.end()) return effects;
        Room& room = roomIt->second;
        if (c.role == Role::Client) {
            room.clients.erase(c.peerId);
            event(RoomEvent::Kind::PeerLeft, conn, room.id, c.peerId, {});
            if (room.host) addEffect<SendControl>(effects, *room.host, rl::PeerLeft{c.peerId, reason});
        } else if (c.role == Role::Host) {
            room.host.reset();
            room.hostGoneAt = now();
            event(RoomEvent::Kind::HostLeft, conn, room.id, 0, std::to_string(room.clients.size()));
        }
        return effects;
    }

    Effects rejectRegister(ConnId conn, rl::RoomRejectReason reason, const std::string& source, bool countsAsFailure, const std::string& roomId) {
        ++counters.registersRejected;
        if (countsAsFailure) noteFailure(source, conn);
        event(RoomEvent::Kind::RegisterRejected, conn, roomId, 0, registerRejectName(reason));
        Effects effects;
        addEffect<SendControl>(effects, conn, rl::RoomRejected{reason});
        return effects;
    }

    Effects rejectJoin(ConnId conn, rl::JoinRoomFailure reason, const std::string& source, bool countsAsFailure, const std::string& roomId) {
        ++counters.joinsRejected;
        if (countsAsFailure) noteFailure(source, conn);
        event(RoomEvent::Kind::JoinRejected, conn, roomId, 0, joinFailureName(reason));
        Effects effects;
        addEffect<SendControl>(effects, conn, rl::JoinResult{false, reason, rl::kRelayBroadcastPeer});
        return effects;
    }

    void newConn(ConnId conn, const std::string& roomId, Role role, rl::RelayPeerId peerId) {
        Conn c;
        c.room = roomId;
        c.role = role;
        c.peerId = peerId;
        const std::uint32_t rate = role == Role::Host ? config.hostBytesPerSecond : config.peerBytesPerSecond;
        c.bucket.configure(rate, config.burstMs, config.maxReliableBytes, now());
        conns[conn] = std::move(c);
    }

    // ---- operations ----
    Effects registerRoom(ConnId conn, const std::string& source, const rl::RegisterRoom& request) {
        if (conns.count(conn) != 0) return violation(conn);
        if (lockedOut(source)) {
            ++counters.lockedOutAttempts;
            ++counters.registersRejected;
            return {CloseConnection{conn, DisconnectReason::LockedOut}};
        }
        if (!versionSupported(request.protocolVersion))
            return rejectRegister(conn, rl::RoomRejectReason::BadVersion, source, false, request.roomId);
        if (!rl::isValidRoomId(request.roomId)) {
            noteFailure(source, conn);
            ++counters.registersRejected;
            return {CloseConnection{conn, DisconnectReason::ProtocolViolation}};
        }

        const auto found = rooms.find(request.roomId);
        if (found == rooms.end()) {
            if (rooms.size() >= config.maxRooms)
                return rejectRegister(conn, rl::RoomRejectReason::RelayFull, source, false, request.roomId);
            Room& room = rooms[request.roomId];
            room.id = request.roomId;
            room.auth = request.hostAuth;
            room.host = conn;
            room.bucket.configure(config.roomBytesPerSecond, config.burstMs, config.maxReliableBytes * config.maxPeersPerRoom, now());
            newConn(conn, room.id, Role::Host, rl::kRelayHostPeer);
            ++counters.roomsCreated;
            event(RoomEvent::Kind::RoomCreated, conn, room.id, 0, {});
            return {SendControl{conn, rl::RoomRegistered{false, static_cast<std::uint16_t>(config.maxPeersPerRoom)}}};
        }

        Room& room = found->second;
        const bool sameAuth = net::constantTimeEqual(room.auth, request.hostAuth);
        if (!sameAuth) {
            const rl::RoomRejectReason reason = room.host ? rl::RoomRejectReason::RoomTaken : rl::RoomRejectReason::BadAuth;
            return rejectRegister(conn, reason, source, true, room.id);
        }

        Effects effects;
        const bool takeover = room.host.has_value();
        if (takeover) {
            const ConnId old = *room.host;
            conns.erase(old);
            room.host.reset();
            addEffect<CloseConnection>(effects, old, DisconnectReason::HostReplaced);
        }
        room.host = conn;
        newConn(conn, room.id, Role::Host, rl::kRelayHostPeer);
        ++counters.roomsResumed;
        event(RoomEvent::Kind::RoomResumed, conn, room.id, 0, takeover ? "takeover" : "after_absence");
        addEffect<SendControl>(effects, conn, rl::RoomRegistered{true, static_cast<std::uint16_t>(config.maxPeersPerRoom)});
        for (const auto& entry : room.clients) addEffect<SendControl>(effects, conn, rl::PeerJoined{entry.first});
        return effects;
    }

    Effects joinRoom(ConnId conn, const std::string& source, const rl::JoinRoom& request) {
        if (conns.count(conn) != 0) return violation(conn);
        if (lockedOut(source)) {
            ++counters.lockedOutAttempts;
            return rejectJoin(conn, rl::JoinRoomFailure::LockedOut, source, false, request.roomId);
        }
        if (!versionSupported(request.protocolVersion))
            return rejectJoin(conn, rl::JoinRoomFailure::BadVersion, source, false, request.roomId);

        const auto found = rl::isValidRoomId(request.roomId) ? rooms.find(request.roomId) : rooms.end();
        if (found == rooms.end()) return rejectJoin(conn, rl::JoinRoomFailure::NoSuchRoom, source, true, request.roomId);
        Room& room = found->second;
        if (!room.host) return rejectJoin(conn, rl::JoinRoomFailure::HostAbsent, source, false, room.id);
        if (room.clients.size() >= config.maxPeersPerRoom)
            return rejectJoin(conn, rl::JoinRoomFailure::RoomFull, source, false, room.id);

        rl::RelayPeerId peerId = 1;
        while (room.clients.count(peerId) != 0) ++peerId;
        room.clients[peerId] = conn;
        newConn(conn, room.id, Role::Client, peerId);
        ++counters.peersJoined;
        event(RoomEvent::Kind::PeerJoined, conn, room.id, peerId, {});
        Effects effects;
        addEffect<SendControl>(effects, conn, rl::JoinResult{true, rl::JoinRoomFailure::None, peerId});
        addEffect<SendControl>(effects, *room.host, rl::PeerJoined{peerId});
        return effects;
    }

    RouteResult drop(DropReason reason) {
        ++counters.framesDropped[static_cast<std::size_t>(reason)];
        RouteResult result;
        result.dropped = reason;
        return result;
    }

    // A size or rate violation: dropped, and the peer is disconnected once it keeps doing it.
    RouteResult strike(ConnId id, Conn& c, DropReason reason) {
        RouteResult result = drop(reason);
        const Clock::time_point t = now();
        if (t - c.strikeWindowStart >= std::chrono::seconds(1)) {
            c.strikeWindowStart = t;
            c.strikes = 0;
        }
        if (config.abuseDropsPerSecond != 0 && ++c.strikes > config.abuseDropsPerSecond) {
            const std::string room = c.room;
            const rl::RelayPeerId peer = c.peerId;
            ++counters.peersKicked;
            event(RoomEvent::Kind::PeerKicked, id, room, peer, std::string(dropReasonName(reason)));
            result.effects = removeConn(id, peerLeftReasonOf(reason));
            addEffect<CloseConnection>(result.effects, id, reason == DropReason::Oversized ? DisconnectReason::Oversized : DisconnectReason::RateLimited);
        }
        return result;
    }

    RouteResult route(ConnId id, Delivery delivery, std::span<const std::byte> frame) {
        const auto it = conns.find(id);
        if (it == conns.end()) return drop(DropReason::Unregistered);
        Conn& c = it->second;
        Room& room = rooms.at(c.room);
        const bool fromClient = c.role == Role::Client;

        const std::size_t limit = delivery == Delivery::Unreliable ? config.maxDatagramBytes : config.maxReliableBytes;
        const std::size_t envelope = fromClient ? rl::kRelayEnvelopeBytes : 0;
        if (frame.size() + envelope > limit) return strike(id, c, DropReason::Oversized);
        if (!c.bucket.take(frame.size(), now())) return strike(id, c, DropReason::RateLimited);
        counters.bytesReceived += frame.size();

        RouteResult result;
        if (fromClient) {
            if (!room.host) return drop(DropReason::HostAbsent);
            if (!room.bucket.take(frame.size() + envelope, now())) return drop(DropReason::RoomByteCap);
            SendGame out;
            out.to = *room.host;
            out.delivery = delivery;
            // Sized up front and filled by index: reserve() followed by push_back() makes GCC 14 -O3 warn about a free of a non-heap pointer.
            out.frame.resize(frame.size() + envelope);
            out.frame[0] = static_cast<std::byte>(c.peerId & 0xFF);
            out.frame[1] = static_cast<std::byte>(c.peerId >> 8);
            std::copy(frame.begin(), frame.end(), out.frame.begin() + rl::kRelayEnvelopeBytes);
            ++counters.framesForwarded;
            counters.bytesForwarded += out.frame.size();
            result.effects.push_back(std::move(out));
            return result;
        }

        if (frame.size() <= rl::kRelayEnvelopeBytes) return drop(DropReason::TooShort);
        const rl::RelayPeerId target = static_cast<rl::RelayPeerId>(static_cast<std::uint8_t>(frame[0]) |
                                                                    (static_cast<std::uint8_t>(frame[1]) << 8));
        const std::span<const std::byte> body = frame.subspan(rl::kRelayEnvelopeBytes);
        std::vector<ConnId> recipients;
        if (target == rl::kRelayBroadcastPeer) {
            if (room.clients.empty()) return drop(DropReason::NoRecipients);
            for (const auto& entry : room.clients) recipients.push_back(entry.second);
        } else {
            const auto client = room.clients.find(target);
            if (client == room.clients.end()) return drop(DropReason::UnknownPeer);
            recipients.push_back(client->second);
        }
        if (!room.bucket.take(body.size() * recipients.size(), now())) return drop(DropReason::RoomByteCap);
        for (const ConnId to : recipients) {
            SendGame out;
            out.to = to;
            out.delivery = delivery;
            out.frame.assign(body.begin(), body.end());
            ++counters.framesForwarded;
            counters.bytesForwarded += out.frame.size();
            result.effects.push_back(std::move(out));
        }
        return result;
    }

    Effects tick() {
        Effects effects;
        const Clock::time_point t = now();
        std::vector<std::string> expired;
        for (const auto& entry : rooms)
            if (!entry.second.host && t - entry.second.hostGoneAt >= ms(config.roomTtlWithoutHostMs)) expired.push_back(entry.first);
        for (const std::string& id : expired) {
            const auto it = rooms.find(id);
            Room& room = it->second;
            for (const auto& client : room.clients) {
                conns.erase(client.second);
                addEffect<CloseConnection>(effects, client.second, DisconnectReason::RoomExpired);
            }
            ++counters.roomsExpired;
            event(RoomEvent::Kind::RoomExpired, 0, id, 0, std::to_string(room.clients.size()));
            rooms.erase(it);
        }
        sweepSources(t, false);
        return effects;
    }
};

RoomTable::RoomTable(RoomTableConfig config, std::shared_ptr<const sdk::IWallClock> clock)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(clock))) {}
RoomTable::~RoomTable() = default;

Effects RoomTable::registerRoom(ConnId conn, const std::string& source, const rl::RegisterRoom& request) {
    return impl_->registerRoom(conn, source, request);
}
Effects RoomTable::joinRoom(ConnId conn, const std::string& source, const rl::JoinRoom& request) {
    return impl_->joinRoom(conn, source, request);
}
Effects RoomTable::onDisconnect(ConnId conn, rl::PeerLeftReason reason) { return impl_->removeConn(conn, reason); }
RouteResult RoomTable::route(ConnId conn, Delivery delivery, std::span<const std::byte> frame) {
    return impl_->route(conn, delivery, frame);
}
Effects RoomTable::tick() { return impl_->tick(); }

Role RoomTable::roleOf(ConnId conn) const {
    const auto it = impl_->conns.find(conn);
    return it == impl_->conns.end() ? Role::None : it->second.role;
}
rl::RelayPeerId RoomTable::peerIdOf(ConnId conn) const {
    const auto it = impl_->conns.find(conn);
    return it == impl_->conns.end() || it->second.role != Role::Client ? rl::kRelayBroadcastPeer : it->second.peerId;
}
std::string RoomTable::roomOf(ConnId conn) const {
    const auto it = impl_->conns.find(conn);
    return it == impl_->conns.end() ? std::string() : it->second.room;
}
bool RoomTable::hasRoom(const std::string& roomId) const { return impl_->rooms.count(roomId) != 0; }
bool RoomTable::hostPresent(const std::string& roomId) const {
    const auto it = impl_->rooms.find(roomId);
    return it != impl_->rooms.end() && it->second.host.has_value();
}
std::size_t RoomTable::clientCount(const std::string& roomId) const {
    const auto it = impl_->rooms.find(roomId);
    return it == impl_->rooms.end() ? 0 : it->second.clients.size();
}
bool RoomTable::isLockedOut(const std::string& source) const { return impl_->lockedOut(source); }
std::size_t RoomTable::connectionCount() const { return impl_->conns.size(); }

RelayStats RoomTable::stats() const {
    RelayStats stats = impl_->counters;
    stats.rooms = impl_->rooms.size();
    stats.trackedSources = impl_->sources.size();
    for (const auto& entry : impl_->rooms) {
        if (!entry.second.host) ++stats.roomsWithoutHost;
        stats.peers += entry.second.clients.size();
    }
    return stats;
}

std::vector<RoomEvent> RoomTable::drainEvents() { return std::exchange(impl_->events, {}); }

}  // namespace wpiemu::relay
