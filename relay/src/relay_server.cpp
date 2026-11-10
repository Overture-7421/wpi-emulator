#include "wpiemu/relay/relay_server.h"

#include <map>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/control_link.h"
#include "wpiemu/relay/relay_address.h"

namespace wpiemu::relay {

namespace rl = sdk::match::relay;
using Clock = std::chrono::steady_clock;
using sdk::proto::Delivery;

namespace {

rl::PeerLeftReason peerLeftReasonOf(DisconnectReason reason) {
    return reason == DisconnectReason::TransportError ? rl::PeerLeftReason::TimedOut : rl::PeerLeftReason::Disconnected;
}

}  // namespace

struct RelayServer::Impl {
    struct Conn {
        ConnId id = 0;
        std::unique_ptr<sdk::proto::ITransport> transport;
        net::IControlLink* control = nullptr;
        std::string lockKey;  // empty: unknown or exempt
        std::string display = "unknown";
        Clock::time_point acceptedAt{};
        std::size_t rejectedAttempts = 0;
        bool dead = false;
        DisconnectReason reason = DisconnectReason::PeerGone;
    };

    Impl(std::unique_ptr<sdk::proto::ITransportServer> s, RelayServerConfig c, std::shared_ptr<const sdk::IWallClock> k,
         std::shared_ptr<RelayLog> l)
        : server(std::move(s)),
          config(std::move(c)),
          clock(std::move(k)),
          log(std::move(l)),
          table(config.table, clock),
          codec(net::kMaxControlFrameBytes),
          epoch(clock->now()) {
        if (!log) log = std::make_shared<RelayLog>();
        nextTick = epoch;
        nextStats = epoch + config.statsInterval;
        publish();
    }

    std::unique_ptr<sdk::proto::ITransportServer> server;
    RelayServerConfig config;
    std::shared_ptr<const sdk::IWallClock> clock;
    std::shared_ptr<RelayLog> log;
    RoomTable table;
    ipc::BinaryMessageCodec codec;
    Clock::time_point epoch;

    std::map<ConnId, std::unique_ptr<Conn>> conns;
    ConnId nextId = 1;
    bool started = false;
    bool serverClosed = false;
    bool shutDown = false;
    std::atomic<bool> stopRequested{false};
    Clock::time_point nextTick;
    Clock::time_point nextStats;
    RelayStats counters;  // the server's part of the stats

    mutable std::mutex statsMutex;
    RelayStats published;

    std::vector<std::byte> controlBuffer;
    std::vector<std::byte> gameBuffer;
    std::vector<std::byte> encodeBuffer;

    std::int64_t nowMs() const { return std::chrono::duration_cast<std::chrono::milliseconds>(clock->now() - epoch).count(); }

    RelayStats snapshot() const {
        RelayStats s = table.stats();
        s.connections = conns.size();
        s.connectionsAccepted = counters.connectionsAccepted;
        s.controlViolations = counters.controlViolations;
        s.unregisteredTimeouts = counters.unregisteredTimeouts;
        s.unknownSources = counters.unknownSources;
        s.sendFailures = counters.sendFailures;
        return s;
    }

    void publish() {
        RelayStats s = snapshot();
        std::lock_guard lock(statsMutex);
        published = s;
    }

    // ---- logging ----
    std::string displayOf(ConnId id) const {
        const auto it = conns.find(id);
        return it == conns.end() ? std::string("?") : it->second->display;
    }

    void logEvents() {
        for (const RoomEvent& e : table.drainEvents()) {
            const LogLevel level = (e.kind == RoomEvent::Kind::JoinRejected || e.kind == RoomEvent::Kind::RegisterRejected ||
                                    e.kind == RoomEvent::Kind::LockedOut || e.kind == RoomEvent::Kind::PeerKicked)
                                       ? LogLevel::Warn
                                       : LogLevel::Info;
            log->logLimited(level, roomEventName(e.kind),
                            {{"room", e.roomId}, {"peerId", e.peerId}, {"peer", displayOf(e.conn)}, {"detail", e.detail}}, nowMs(), 50);
        }
    }

    // ---- connection lifecycle ----
    void markDead(Conn& c, DisconnectReason reason) {
        if (c.dead) return;
        c.dead = true;
        c.reason = reason;
        c.transport->close();
    }

    void violation(Conn& c, const char* what) {
        ++counters.controlViolations;
        log->logLimited(LogLevel::Warn, "control_violation", {{"peer", c.display}, {"what", what}}, nowMs(), 50);
        markDead(c, DisconnectReason::ProtocolViolation);
    }

    bool acceptPending() {
        bool worked = false;
        for (std::size_t i = 0; i < config.maxAcceptsPerPoll; ++i) {
            sdk::Result<std::unique_ptr<sdk::proto::ITransport>> accepted = server->accept(std::chrono::milliseconds(0));
            if (!accepted) {
                if (accepted.error().code == sdk::ErrorCode::Disconnected) serverClosed = true;
                else log->logLimited(LogLevel::Error, "accept_failed", {{"error", accepted.error().message}}, nowMs(), 5);
                return worked;
            }
            if (accepted.value() == nullptr) return worked;
            worked = true;
            admit(std::move(accepted).value());
        }
        return worked;
    }

    void admit(std::unique_ptr<sdk::proto::ITransport> transport) {
        auto c = std::make_unique<Conn>();
        c->id = nextId++;
        c->control = net::controlLinkOf(*transport);
        c->acceptedAt = clock->now();
        c->transport = std::move(transport);
        ++counters.connectionsAccepted;
        if (c->control == nullptr) {
            log->logLimited(LogLevel::Error, "conn_unsupported", {{"detail", "transport has no control channel"}}, nowMs(), 5);
            c->dead = true;
            c->transport->close();
        } else if (const auto address = sourceAddressOf(c->transport->describe())) {
            c->display = address->display;
            if (!(address->loopback && config.exemptLoopbackFromLockout)) c->lockKey = address->key;
        } else {
            ++counters.unknownSources;
            log->logLimited(LogLevel::Warn, "source_unknown", {{"detail", "the transport did not report an IP address"}}, nowMs(), 5);
        }
        const ConnId id = c->id;
        conns.emplace(id, std::move(c));
    }

    bool service(Conn& c) {
        if (c.dead) return false;
        bool worked = false;
        bool gone = false;
        for (std::size_t i = 0; i < config.maxFramesPerConnectionPerPoll; ++i) {
            const sdk::Result<bool> got = c.control->receiveControl(controlBuffer, std::chrono::milliseconds(0));
            if (!got) {
                gone = true;
                break;
            }
            if (!got.value()) break;
            worked = true;
            handleControl(c, controlBuffer);
            if (c.dead) return true;
        }
        for (std::size_t i = 0; i < config.maxFramesPerConnectionPerPoll; ++i) {
            Delivery delivery = Delivery::Reliable;
            const sdk::Result<bool> got = c.control->receiveGame(gameBuffer, delivery, std::chrono::milliseconds(0));
            if (!got) {
                gone = true;
                break;
            }
            if (!got.value()) break;
            worked = true;
            handleGame(c, delivery, gameBuffer);
            if (c.dead) return true;
        }
        if (gone) {
            markDead(c, DisconnectReason::PeerGone);
            worked = true;
        }
        return worked;
    }

    void handleControl(Conn& c, const std::vector<std::byte>& frame) {
        if (frame.size() > net::kMaxControlFrameBytes) return violation(c, "oversize_control_frame");
        sdk::Result<sdk::proto::Message> decoded = codec.decode(frame);
        if (!decoded) return violation(c, "undecodable_control_frame");
        const sdk::proto::Message& message = decoded.value();
        if (table.roleOf(c.id) != Role::None) {
            if (table.roleOf(c.id) != Role::Host || !std::holds_alternative<rl::CloseRoom>(message))
                return violation(c, "control_frame_after_registration");
            // The host's last game frames (ServerClosing) were sent before CloseRoom but wait on the other channel: forward them first.
            for (std::size_t i = 0; i < config.maxFramesPerConnectionPerPoll; ++i) {
                Delivery delivery = Delivery::Reliable;
                const sdk::Result<bool> pending = c.control->receiveGame(gameBuffer, delivery, std::chrono::milliseconds(0));
                if (!pending || !pending.value()) break;
                handleGame(c, delivery, gameBuffer);
            }
            Effects closing = table.closeRoom(c.id);
            execute(closing);
            logEvents();
            return;
        }

        Effects effects;
        if (const auto* reg = std::get_if<rl::RegisterRoom>(&message)) effects = table.registerRoom(c.id, c.lockKey, *reg);
        else if (const auto* join = std::get_if<rl::JoinRoom>(&message)) effects = table.joinRoom(c.id, c.lockKey, *join);
        else return violation(c, "unexpected_control_message");

        execute(effects);
        logEvents();
        if (!c.dead && table.roleOf(c.id) == Role::None && ++c.rejectedAttempts >= config.maxAttemptsPerConnection)
            markDead(c, DisconnectReason::TooManyAttempts);
    }

    void handleGame(Conn& c, Delivery delivery, const std::vector<std::byte>& frame) {
        RouteResult result = table.route(c.id, delivery, frame);
        execute(result.effects);
        if (result.dropped && !result.effects.empty()) logEvents();
    }

    // ---- effects ----
    void execute(Effects& effects) {
        for (Effect& effect : effects) {
            if (auto* send = std::get_if<SendControl>(&effect)) sendControl(*send);
            else if (auto* game = std::get_if<SendGame>(&effect)) sendGame(*game);
            else if (const auto* close = std::get_if<CloseConnection>(&effect)) {
                const auto it = conns.find(close->to);
                if (it != conns.end()) markDead(*it->second, close->reason);
            }
        }
    }

    void sendControl(const SendControl& send) {
        const auto it = conns.find(send.to);
        if (it == conns.end() || it->second->dead) return;
        Conn& c = *it->second;
        encodeBuffer.clear();
        if (!codec.encode(send.message, encodeBuffer)) {
            ++counters.sendFailures;
            return;
        }
        if (const sdk::Status status = c.control->sendControl(encodeBuffer); !status) {
            ++counters.sendFailures;
            markDead(c, status.error().code == sdk::ErrorCode::Disconnected ? DisconnectReason::PeerGone : DisconnectReason::TransportError);
        }
    }

    void sendGame(const SendGame& send) {
        const auto it = conns.find(send.to);
        if (it == conns.end() || it->second->dead) return;
        Conn& c = *it->second;
        const sdk::Status status = send.delivery == Delivery::Unreliable ? c.transport->sendUnreliable(send.frame) : c.transport->send(send.frame);
        if (status) return;
        ++counters.sendFailures;
        // A lost reliable frame breaks the peer's ordered stream, so that peer goes; a lost datagram is just a lost datagram.
        if (send.delivery == Delivery::Reliable || status.error().code == sdk::ErrorCode::Disconnected)
            markDead(c, status.error().code == sdk::ErrorCode::Disconnected ? DisconnectReason::PeerGone : DisconnectReason::TransportError);
    }

    // Removes dead connections from the table (telling the host) and from the server, until none are left.
    bool reap() {
        bool any = false;
        for (;;) {
            std::vector<ConnId> dead;
            for (const auto& entry : conns)
                if (entry.second->dead) dead.push_back(entry.first);
            if (dead.empty()) break;
            any = true;
            for (const ConnId id : dead) {
                const auto it = conns.find(id);
                Effects effects = table.onDisconnect(id, peerLeftReasonOf(it->second->reason));
                execute(effects);
                logEvents();
                conns.erase(it);
            }
        }
        return any;
    }

    bool tickIfDue() {
        const Clock::time_point t = clock->now();
        if (t < nextTick) return false;
        nextTick = t + config.tickInterval;
        bool worked = false;

        Effects effects = table.tick();
        if (!effects.empty()) worked = true;
        execute(effects);
        for (const auto& entry : conns) {
            Conn& c = *entry.second;
            if (!c.dead && table.roleOf(c.id) == Role::None && t - c.acceptedAt >= config.unregisteredTimeout) {
                ++counters.unregisteredTimeouts;
                markDead(c, DisconnectReason::UnregisteredTimeout);
                worked = true;
            }
        }
        logEvents();
        log->flushSuppressed(nowMs());
        publish();
        if (config.statsInterval.count() > 0 && t >= nextStats) {
            nextStats = t + config.statsInterval;
            logStats();
        }
        return worked;
    }

    void logStats() {
        const RelayStats s = snapshot();
        log->log(LogLevel::Info, "stats",
                 {{"rooms", s.rooms},
                  {"roomsWithoutHost", s.roomsWithoutHost},
                  {"peers", s.peers},
                  {"connections", s.connections},
                  {"framesForwarded", s.framesForwarded},
                  {"bytesForwarded", s.bytesForwarded},
                  {"bytesReceived", s.bytesReceived},
                  {"framesDropped", s.framesDroppedTotal()},
                  {"droppedOversized", s.dropped(DropReason::Oversized)},
                  {"droppedRateLimited", s.dropped(DropReason::RateLimited)},
                  {"droppedRoomByteCap", s.dropped(DropReason::RoomByteCap)},
                  {"droppedUnregistered", s.dropped(DropReason::Unregistered)},
                  {"droppedOther", s.framesDroppedTotal() - s.dropped(DropReason::Oversized) - s.dropped(DropReason::RateLimited) -
                                       s.dropped(DropReason::RoomByteCap) - s.dropped(DropReason::Unregistered)},
                  {"joinsRejected", s.joinsRejected},
                  {"registersRejected", s.registersRejected},
                  {"lockouts", s.lockouts},
                  {"lockedOutAttempts", s.lockedOutAttempts},
                  {"controlViolations", s.controlViolations},
                  {"peersKicked", s.peersKicked},
                  {"roomsCreated", s.roomsCreated},
                  {"roomsExpired", s.roomsExpired},
                  {"roomsClosed", s.roomsClosed},
                  {"pingsAnswered", s.pingsAnswered},
                  {"trackedSources", s.trackedSources}});
    }

    bool poll() {
        if (!started) return false;
        bool worked = acceptPending();
        for (const auto& entry : conns) worked = service(*entry.second) || worked;
        worked = reap() || worked;
        worked = tickIfDue() || worked;
        worked = reap() || worked;
        if (worked) publish();
        return worked;
    }

    void shutdownAll() {
        if (shutDown) return;
        shutDown = true;
        log->log(LogLevel::Info, "stopping", {{"rooms", static_cast<std::uint64_t>(table.stats().rooms)}, {"connections", static_cast<std::uint64_t>(conns.size())}});
        for (const auto& entry : conns) markDead(*entry.second, DisconnectReason::Shutdown);
        conns.clear();
        server->close();
        publish();
    }
};

RelayServer::RelayServer(std::unique_ptr<sdk::proto::ITransportServer> server, RelayServerConfig config,
                         std::shared_ptr<const sdk::IWallClock> clock, std::shared_ptr<RelayLog> log)
    : impl_(std::make_unique<Impl>(std::move(server), std::move(config), std::move(clock), std::move(log))) {}

RelayServer::~RelayServer() { shutdown(); }

sdk::Status RelayServer::start() {
    if (impl_->started) return sdk::makeError(sdk::ErrorCode::InvalidState, "relay already started");
    if (const sdk::Status status = impl_->server->listen(); !status) return status;
    impl_->started = true;
    impl_->log->log(LogLevel::Info, "listening", {{"endpoint", impl_->server->endpoint()}});
    return sdk::okStatus();
}

bool RelayServer::poll() { return impl_->poll(); }

sdk::Status RelayServer::run() {
    if (!impl_->started) {
        if (const sdk::Status status = start(); !status) return status;
    }
    while (!impl_->stopRequested.load() && !impl_->serverClosed) {
        if (!impl_->poll()) std::this_thread::sleep_for(impl_->config.idleSleep);
    }
    impl_->shutdownAll();
    return sdk::okStatus();
}

void RelayServer::stop() { impl_->stopRequested.store(true); }

void RelayServer::shutdown() { impl_->shutdownAll(); }

std::string RelayServer::endpoint() const { return impl_->server->endpoint(); }

RelayStats RelayServer::stats() const {
    std::lock_guard lock(impl_->statsMutex);
    return impl_->published;
}

}  // namespace wpiemu::relay
