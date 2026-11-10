#include "wpiemu/core/match/match_host.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <set>

#include "wpiemu/core/match/entity_delta.h"
#include "wpiemu/core/match/hud_keys.h"
#include "wpiemu/core/match/robot_appearance.h"
#include "wpiemu/core/match/snapshot_budget.h"
#include "wpiemu/core/match/spec_intake.h"
#include "wpiemu/core/match/wire_frame.h"
#include "wpiemu/core/runtime/message_link.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"

namespace wpiemu::core::match {

namespace {

namespace wire = sdk::match;
namespace proto = sdk::proto;
using Clock = std::chrono::steady_clock;

constexpr std::uint64_t kAddedLinkIdBase = std::uint64_t{1} << 62;   // link ids of addConnection (the acceptor counts from 1)
constexpr std::size_t kMaxLoggedClamps = 8;

std::string truncated(std::string s, std::size_t max) {
    if (s.size() > max) s.resize(max);
    return s;
}

// Display names are shown to everyone and logged: cap the length on a UTF-8 boundary and drop control characters.
std::string sanitizeName(std::string name) {
    for (char& c : name)
        if (static_cast<unsigned char>(c) < 0x20 || c == 0x7F) c = '?';
    if (name.size() > wire::kMaxNameBytes) {
        std::size_t cut = wire::kMaxNameBytes;
        while (cut > 0 && (static_cast<unsigned char>(name[cut]) & 0xC0) == 0x80) --cut;
        name.resize(cut);
    }
    return name;
}

// Refills at 'rate' per second up to 'burst'; take() spends one token.
struct TokenBucket {
    double tokens = 0.0;
    Clock::time_point last{};

    void reset(double burst, Clock::time_point t) {
        tokens = burst;
        last = t;
    }
    bool take(double rate, double burst, Clock::time_point t) {
        const double elapsed = std::chrono::duration<double>(t - last).count();
        if (elapsed > 0.0) {
            tokens = std::min(burst, tokens + elapsed * rate);
            last = t;
        }
        if (tokens < 1.0) return false;
        tokens -= 1.0;
        return true;
    }
};

struct Participant {
    wire::ParticipantId id = wire::kNoParticipant;
    std::string name;
    proto::AllianceStation station = proto::AllianceStation::Red1;
    wire::ResumeToken token{};
    wire::RobotAppearance appearance;      // built once from the accepted spec; sent in every roster
    std::uint64_t linkId = 0;              // the connection serving it; 0 = dropped, resumable
    Clock::time_point detachedAt{};
    std::uint64_t sessionId = proto::kUnspecified;

    // inputs
    bool seenInput = false;
    std::uint32_t lastInputSeq = wire::kNoInputSeq;
    TokenBucket inputBucket;
    TokenBucket pingBucket;
    std::uint64_t inputsSubmitted = 0;
    std::uint64_t inputsRateLimited = 0;

    // snapshots
    std::uint32_t snapshotSeq = 0;
    std::int64_t lastSlot = -1;
    std::optional<Clock::time_point> lastIdleSend;
    EntitySizeEstimate entitySize;
    std::uint64_t snapshotsSent = 0;
    EntityDeltaEncoder encoder;
    SnapshotRateSelector rate;
};

struct Connection {
    std::uint64_t linkId = 0;
    std::shared_ptr<runtime::IMessageLink> link;
    std::unique_ptr<PeerOutbox> outbox;
    Clock::time_point connectedAt{};
    Clock::time_point lastRx{};
    Clock::time_point closingSince{};
    wire::ParticipantId participant = wire::kNoParticipant;
    std::size_t strikes = 0;
    std::uint64_t malformedSeen = 0;
    bool joinQueued = false;   // its Join waits for a later poll (joinsPerPoll)
    bool closing = false;   // a final message is on its way; everything the peer sends is ignored
    bool doomed = false;    // removed at the next sweep
    std::string doomReason;
};

struct Command {
    enum class Kind { Kick, Close } kind = Kind::Kick;
    wire::ParticipantId target = wire::kNoParticipant;
    std::string detail;
};

}  // namespace

// ---- Impl ---------------------------------------------------------------------------------------------------------

struct MatchHost::Impl {
    Impl(MatchHostConfig cfg, IMatchHostSession& s, MatchHostServices svc)
        : config(std::move(cfg)), session(s), services(std::move(svc)), clock(*services.clock),
          log(services.logger != nullptr ? *services.logger : nullLogger),
          hudKeys(HudKeyTable::fromDescriptors(s.hudDescriptors(), config.extraHudKeys)) {
        epoch.store(proto::kFirstRunEpoch);
        std::array<std::byte, 8> nonce{};
        if (services.randomFill && services.randomFill(nonce)) {
            for (const std::byte b : nonce) sessionBase = (sessionBase << 8) | std::to_integer<std::uint64_t>(b);
        }
    }

    MatchHostConfig config;
    IMatchHostSession& session;
    MatchHostServices services;
    const sdk::IWallClock& clock;
    sdk::NullLogger nullLogger;
    sdk::ILogger& log;
    HudKeyTable hudKeys;

    runtime::MessageInbox inbox;
    std::unique_ptr<runtime::MessageLinkAcceptor> acceptor;
    std::atomic<std::uint64_t> nextAddedLinkId{kAddedLinkIdBase};

    std::mutex commandMutex;
    std::vector<Command> commands;

    std::atomic<std::uint64_t> epoch{proto::kFirstRunEpoch};
    std::atomic<std::size_t> joined{0};
    std::atomic<std::size_t> attached{0};

    // ---- sim-thread state ----
    MatchHostStats stats;
    std::map<std::uint64_t, Connection> conns;
    std::map<wire::ParticipantId, Participant> parts;
    std::deque<std::pair<std::uint64_t, wire::Join>> pendingJoins;
    std::set<wire::ResumeToken> kickedTokens;
    std::uint32_t nextParticipantId = 1;
    std::uint64_t sessionCounter = 0;
    std::uint64_t sessionBase = 0x57A7E1D5C0FFEE11ull;
    bool haveObservation = false;
    std::uint64_t runnerEpoch = 0;
    wire::MatchState state = wire::MatchState::Lobby;
    bool closed = false;
    bool rosterDirty = false;
    std::optional<Clock::time_point> lastRoster;
    std::size_t joinsThisPoll = 0;
    std::vector<std::byte> scratch;

    // ---- helpers ----
    Clock::time_point now() const { return clock.now(); }

    std::uint64_t newSessionId() {
        for (;;) {
            const std::uint64_t id = sessionBase ^ (++sessionCounter * 0x9E3779B97F4A7C15ull);
            if (id != proto::kUnspecified) return id;
        }
    }

    void updateAtomics() {
        joined.store(parts.size());
        std::size_t n = 0;
        for (const auto& [id, p] : parts)
            if (p.linkId != 0) ++n;
        attached.store(n);
    }

    Participant* participantOf(Connection& c) {
        if (c.participant == wire::kNoParticipant) return nullptr;
        const auto it = parts.find(c.participant);
        return it == parts.end() ? nullptr : &it->second;
    }

    Connection* connectionOf(const Participant& p) {
        if (p.linkId == 0) return nullptr;
        const auto it = conns.find(p.linkId);
        return it == conns.end() ? nullptr : &it->second;
    }

    void doom(Connection& c, std::string reason) {
        if (c.doomed) return;
        c.doomed = true;
        c.doomReason = std::move(reason);
    }

    // Counts a malformed or unexpected message from `c`; the third strike drops the connection.
    void strike(Connection& c, const std::string& what) {
        ++stats.protocolViolations;
        ++c.strikes;
        log.warn("match", "protocol violation from " + c.link->describe() + ": " + what);
        if (c.strikes >= config.maxStrikes) doom(c, "too many protocol violations");
    }

    bool send(Connection& c, proto::Message message, bool closeAfter = false) {
        if (c.doomed) return false;
        switch (c.outbox->enqueue(std::move(message), closeAfter)) {
            case PeerOutbox::Result::Queued: return true;
            case PeerOutbox::Result::QueuedDroppedOldest: ++stats.snapshotsDroppedSlow; return true;
            case PeerOutbox::Result::Overflow:
                ++stats.slowPeersDropped;
                doom(c, "outbound queue overflow: the peer is too slow");
                return false;
            case PeerOutbox::Result::Closed: doom(c, "send failed"); return false;
        }
        return false;
    }

    wire::RosterEntry hostEntry() const {
        wire::RosterEntry e;
        e.participantId = wire::kHostParticipantId;
        e.name = truncated(session.hostName(), wire::kMaxNameBytes);
        e.station = session.driverStation().station;
        e.isHost = true;
        e.appearance = session.hostAppearance();
        return e;
    }

    std::vector<wire::RosterEntry> rosterEntries() {
        std::vector<wire::RosterEntry> entries;
        entries.push_back(hostEntry());
        for (auto& [id, p] : parts) {
            wire::RosterEntry e;
            e.participantId = id;
            e.name = p.name;
            e.station = p.station;
            e.appearance = p.appearance;
            if (Connection* c = connectionOf(p)) {
                if (const auto ls = c->link->linkStats()) {
                    e.pingMs = static_cast<std::uint16_t>(std::min<std::uint32_t>(ls->rttMs, 0xFFFF));
                    e.lossPermille = static_cast<std::uint16_t>(std::min<std::uint32_t>(ls->lossPermille, 1000));
                }
            } else {
                e.lossPermille = 1000;   // dropped: resumable, not receiving
            }
            entries.push_back(std::move(e));
        }
        return entries;
    }

    void broadcastRoster(Clock::time_point t) {
        const std::vector<wire::RosterEntry> entries = rosterEntries();
        for (auto& [id, p] : parts) {
            Connection* c = connectionOf(p);
            if (c == nullptr) continue;
            wire::Roster r;
            r.sessionId = p.sessionId;
            r.runEpoch = epoch.load();
            r.entries = entries;
            send(*c, std::move(r));
        }
        lastRoster = t;
        rosterDirty = false;
    }

    // ---- connections ----
    void onConnected(runtime::InboundEvent& ev, Clock::time_point t) {
        if (closed || conns.size() >= config.maxConnections) {
            ++stats.connectionsRefused;
            log.warn("match", "connection refused (" + std::string(closed ? "closing" : "too many connections") + "): " + ev.link->describe());
            ev.link->close();
            return;
        }
        Connection c;
        c.linkId = ev.linkId;
        c.link = ev.link;
        c.outbox = std::make_unique<PeerOutbox>(ev.link, clock, config.outbox);
        c.connectedAt = c.lastRx = t;
        ++stats.connections;
        conns.emplace(ev.linkId, std::move(c));
    }

    // Frames the link dropped as malformed since the last look (the link ends itself at maxStrikes).
    void countMalformed(Connection& c) {
        const std::uint64_t malformed = c.link->malformedFrames();
        stats.malformedFrames += malformed - c.malformedSeen;
        c.malformedSeen = malformed;
    }

    void onDisconnected(std::uint64_t linkId, const std::string& why) {
        const auto it = conns.find(linkId);
        if (it == conns.end()) return;
        countMalformed(it->second);
        doom(it->second, why.empty() ? "disconnected" : why);
    }

    // Drops doomed connections; a participant that held one becomes resumable.
    void sweep(Clock::time_point t) {
        for (auto it = conns.begin(); it != conns.end();) {
            Connection& c = it->second;
            if (!c.doomed) {
                ++it;
                continue;
            }
            if (Participant* p = participantOf(c)) {
                if (p->linkId == c.linkId) {
                    p->linkId = 0;
                    p->detachedAt = t;
                    rosterDirty = true;
                    log.info("match", "participant " + std::to_string(p->id) + " (" + p->name + ") disconnected: " + c.doomReason +
                                          "; its robot stays for " + std::to_string(config.resumeWindow.count()) + " ms");
                }
            } else {
                log.info("match", "connection closed: " + c.link->describe() + ": " + c.doomReason);
            }
            it = conns.erase(it);
        }
        updateAtomics();
    }

    // ---- removal ----
    void removeParticipant(wire::ParticipantId id) {
        const auto it = parts.find(id);
        if (it == parts.end()) return;
        if (Connection* c = connectionOf(it->second)) c->participant = wire::kNoParticipant;
        if (const sdk::Status s = session.removeRemoteRobot(id); !s)
            log.warn("match", "removing robot " + std::to_string(id) + " failed: " + s.error().message);
        parts.erase(it);
        rosterDirty = true;
        updateAtomics();
    }

    // ---- join ----
    void rejectJoin(Connection& c, wire::JoinRejectReason reason, std::string detail) {
        ++stats.joinsRejected;
        log.info("match", "join refused from " + c.link->describe() + " (reason " + std::to_string(static_cast<int>(reason)) + "): " + detail);
        wire::JoinReject r;
        r.reason = reason;
        r.detail = truncated(std::move(detail), wire::kMaxDetailBytes);
        c.closing = true;
        c.closingSince = now();
        send(c, std::move(r), true);
    }

    std::optional<proto::AllianceStation> chooseStation() const {
        std::array<bool, 6> used{};
        int red = 0, blue = 0;
        const auto mark = [&](proto::AllianceStation s) {
            used[static_cast<std::size_t>(s)] = true;
            (proto::allianceOf(s) == sdk::Alliance::Red ? red : blue)++;
        };
        mark(session.driverStation().station);
        for (const auto& [id, p] : parts) mark(p.station);
        const std::array<sdk::Alliance, 2> order = red <= blue ? std::array{sdk::Alliance::Red, sdk::Alliance::Blue}
                                                               : std::array{sdk::Alliance::Blue, sdk::Alliance::Red};
        for (const sdk::Alliance a : order)
            for (int n = 1; n <= 3; ++n)
                if (!used[static_cast<std::size_t>(proto::stationOf(a, n))]) return proto::stationOf(a, n);
        return std::nullopt;
    }

    std::optional<wire::ResumeToken> newToken() {
        wire::ResumeToken token{};
        if (!services.randomFill) return std::nullopt;
        std::array<std::byte, wire::kResumeTokenBytes> bytes{};
        for (int attempt = 0; attempt < 4; ++attempt) {
            if (!services.randomFill(bytes)) return std::nullopt;
            for (std::size_t i = 0; i < token.size(); ++i) token[i] = std::to_integer<std::uint8_t>(bytes[i]);
            if (wire::hasResumeToken(token)) return token;
        }
        return std::nullopt;
    }

    Participant* findByToken(const wire::ResumeToken& token) {
        if (!wire::hasResumeToken(token)) return nullptr;
        for (auto& [id, p] : parts)
            if (p.token == token) return &p;
        return nullptr;
    }

    wire::JoinAccept makeAccept(Participant& p) {
        wire::JoinAccept a;
        a.participantId = p.id;
        a.sessionId = p.sessionId;
        a.runEpoch = epoch.load();
        a.station = p.station;
        a.stepUs = session.stepUs();
        a.snapshotRateHz = p.rate.hz();
        a.inputRateHz = config.inputRateHz;
        a.resumeToken = p.token;
        a.state = state;
        a.serverTick = session.tickIndex();
        a.serverSimTimeUs = session.simTimeUs();
        if (const auto info = session.remoteRobotInfo(p.id)) {
            a.startPose.position = {info->placement.xM, info->placement.yM, 0.f};
            a.startPose.rotation = sdk::Quat::fromYaw(info->placement.headingRad);
        }
        a.roster = rosterEntries();
        return a;
    }

    void resetPeerStream(Participant& p) {
        p.encoder.reset();
        p.lastSlot = -1;
        p.lastIdleSend.reset();
    }

    void onJoin(Connection& c, wire::Join&& join, Clock::time_point t) {
        if (c.participant != wire::kNoParticipant) {
            strike(c, "Join after the connection already joined");
            return;
        }
        if (closed) return rejectJoin(c, wire::JoinRejectReason::HostClosing, "the host is closing the room");
        if (join.protocolVersion < wire::kMinCompatibleMatchProtocolVersion || join.protocolVersion > wire::kMatchProtocolVersion)
            return rejectJoin(c, wire::JoinRejectReason::BadProtocol,
                              "host speaks match protocol " + std::to_string(wire::kMatchProtocolVersion) + ", client " +
                                  std::to_string(join.protocolVersion));
        if (join.gamePluginId != session.gamePluginId())
            return rejectJoin(c, wire::JoinRejectReason::WrongPlugin, "the host plays '" + truncated(session.gamePluginId(), wire::kMaxNameBytes) + "'");
        if (join.wpilibYear != session.wpilibYear())
            return rejectJoin(c, wire::JoinRejectReason::WrongSeason, "the host plays the " + std::to_string(session.wpilibYear()) + " season");

        if (Participant* resumed = findResumable(join.resumeToken, t)) return resume(c, *resumed, t);
        if (wire::hasResumeToken(join.resumeToken) && kickedTokens.count(join.resumeToken) != 0)
            return rejectJoin(c, wire::JoinRejectReason::Kicked, "removed by the host");

        if (parts.size() + 1 >= wire::kMaxRobotsPerMatch) return rejectJoin(c, wire::JoinRejectReason::RoomFull, "the match is full");
        if (services.specPipeline == nullptr) return rejectJoin(c, wire::JoinRejectReason::BadSpec, "the host cannot read robot descriptions");
        const std::optional<wire::ResumeToken> token = newToken();
        if (!token || nextParticipantId >= wire::kNoParticipant)
            return rejectJoin(c, wire::JoinRejectReason::HostClosing, "the host cannot admit more participants");
        const std::optional<proto::AllianceStation> station = chooseStation();
        if (!station) return rejectJoin(c, wire::JoinRejectReason::RoomFull, "no alliance station is free");

        sdk::Result<AcceptedSpec> spec = acceptRemoteSpec(join.robotSpecJson, *services.specPipeline);
        if (!spec) return rejectJoin(c, wire::JoinRejectReason::BadSpec, spec.error().message);
        for (std::size_t i = 0; i < spec.value().clamped.size() && i < kMaxLoggedClamps; ++i)
            log.info("match", "clamped robot spec of '" + sanitizeName(join.displayName) + "': " + spec.value().clamped[i]);

        const auto id = static_cast<wire::ParticipantId>(nextParticipantId);
        wire::RobotAppearance appearance = appearanceOf(spec.value().config);
        runtime::RemoteRobotRequest request;
        request.participantId = id;
        request.name = join.displayName.empty() ? "Player " + std::to_string(id) : sanitizeName(join.displayName);
        request.station = *station;
        request.spec = std::move(spec.value().config);
        if (const sdk::Status added = session.addRemoteRobot(request); !added) {
            const bool full = added.error().code == sdk::ErrorCode::OutOfRange || added.error().code == sdk::ErrorCode::AlreadyExists;
            return rejectJoin(c, full ? wire::JoinRejectReason::RoomFull : wire::JoinRejectReason::BadSpec, added.error().message);
        }
        ++nextParticipantId;

        Participant p;
        p.id = id;
        p.name = request.name;
        p.station = *station;
        p.token = *token;
        p.appearance = std::move(appearance);
        p.linkId = c.linkId;
        p.sessionId = newSessionId();
        p.inputBucket.reset(config.inputBurst, t);
        p.pingBucket.reset(config.pingBurst, t);
        p.rate = SnapshotRateSelector(config.rate);
        c.participant = id;
        Participant& stored = parts.emplace(id, std::move(p)).first->second;
        ++stats.joinsAccepted;
        updateAtomics();
        log.info("match", "participant " + std::to_string(id) + " (" + stored.name + ") joined at station " +
                              std::to_string(static_cast<int>(stored.station)) + " from " + c.link->describe());
        send(c, makeAccept(stored));
        rosterDirty = true;
    }

    Participant* findResumable(const wire::ResumeToken& token, Clock::time_point t) {
        Participant* p = findByToken(token);
        if (p == nullptr) return nullptr;
        if (p->linkId == 0 && t - p->detachedAt > config.resumeWindow) return nullptr;
        return p;
    }

    void resume(Connection& c, Participant& p, Clock::time_point t) {
        const std::optional<wire::ResumeToken> token = newToken();
        if (!token) return rejectJoin(c, wire::JoinRejectReason::HostClosing, "the host cannot issue a resume token");
        if (Connection* old = connectionOf(p)) {
            old->participant = wire::kNoParticipant;
            doom(*old, "replaced by a resumed connection");
        }
        p.linkId = c.linkId;
        p.token = *token;
        p.sessionId = newSessionId();
        p.inputBucket.reset(config.inputBurst, t);
        p.pingBucket.reset(config.pingBurst, t);
        resetPeerStream(p);
        c.participant = p.id;
        ++stats.resumes;
        updateAtomics();
        log.info("match", "participant " + std::to_string(p.id) + " (" + p.name + ") resumed from " + c.link->describe());
        send(c, makeAccept(p));
        rosterDirty = true;
    }

    // ---- inputs, acks, pings ----
    // False when the frame must be ignored (stale epoch or another connection's frame); NewerEpoch is a violation.
    bool acceptFrame(Connection& c, const Participant& p, std::uint64_t frameSession, std::uint64_t frameEpoch) {
        switch (proto::classifyFrame(frameSession, frameEpoch, p.sessionId, epoch.load())) {
            case proto::FrameDisposition::Accept: return true;
            case proto::FrameDisposition::DropStaleEpoch: ++stats.framesStaleEpoch; return false;
            case proto::FrameDisposition::DropForeignSession: ++stats.framesForeignSession; return false;
            case proto::FrameDisposition::NewerEpoch: strike(c, "frame of an epoch the host has not announced"); return false;
        }
        return false;
    }

    void onInput(Connection& c, Participant& p, wire::MatchInputPacket& packet, Clock::time_point t) {
        ++stats.inputPackets;
        if (!wire::fitsLimits(packet)) return strike(c, "input packet beyond the protocol limits");
        if (!acceptFrame(c, p, packet.sessionId, packet.runEpoch)) return;
        for (const wire::MatchInput& input : packet.inputs) {
            if (input.inputSeq == wire::kNoInputSeq) {
                ++stats.inputsRejected;
                continue;
            }
            if (p.seenInput && !wire::seqNewer(input.inputSeq, p.lastInputSeq)) {
                ++stats.inputsRedundant;
                continue;
            }
            if (!p.inputBucket.take(config.maxInputsPerSecond, config.inputBurst, t)) {
                ++stats.inputsRateLimited;
                ++p.inputsRateLimited;
                continue;
            }
            p.seenInput = true;
            p.lastInputSeq = input.inputSeq;
            const sdk::Result<runtime::RemoteInputOutcome> outcome = session.submitRemoteInput(p.id, input);
            if (outcome) {
                ++stats.inputsSubmitted;
                ++p.inputsSubmitted;
            } else {
                ++stats.inputsRejected;
            }
        }
    }

    void onAck(Connection& c, Participant& p, const wire::MatchSnapshotAck& ack) {
        if (!acceptFrame(c, p, ack.sessionId, ack.runEpoch)) return;
        if (p.encoder.onAck(ack.snapshotSeq)) ++stats.acks;
        else ++stats.acksIgnored;
    }

    void onControl(Connection& c, Participant& p, const wire::MatchControl& ctl, Clock::time_point t) {
        if (ctl.kind != wire::MatchControlKind::PingRequest) return strike(c, "a client sent a host control message");
        // Pings are answered whatever the epoch (a client re-syncing after a Reset needs them); only another
        // connection's session is ignored.
        if (ctl.sessionId != proto::kUnspecified && ctl.sessionId != p.sessionId) {
            ++stats.framesForeignSession;
            return;
        }
        if (!p.pingBucket.take(config.maxPingsPerSecond, config.pingBurst, t)) {
            ++stats.pingsRateLimited;
            return;
        }
        ++stats.pings;
        wire::MatchControl reply;
        reply.kind = wire::MatchControlKind::PingReply;
        reply.sessionId = p.sessionId;
        reply.runEpoch = epoch.load();
        reply.pingSeq = ctl.pingSeq;
        reply.clientTimeUs = ctl.clientTimeUs;
        reply.serverTick = session.tickIndex();
        reply.serverSimTimeUs = session.simTimeUs();
        send(c, std::move(reply));
    }

    void onLeave(Connection& c, Participant& p, const wire::Leave& leave) {
        ++stats.leaves;
        log.info("match", "participant " + std::to_string(p.id) + " (" + p.name + ") left: " + truncated(leave.detail, wire::kMaxDetailBytes));
        removeParticipant(p.id);
        doom(c, "left");
    }

    void onMessage(Connection& c, proto::Message&& message, Clock::time_point t) {
        if (c.doomed || c.closing) return;
        c.lastRx = t;
        if (auto* join = std::get_if<wire::Join>(&message)) {
            if (c.participant != wire::kNoParticipant || c.joinQueued) return strike(c, "Join after the connection already joined");
            if (joinsThisPoll >= config.joinsPerPoll) {
                c.joinQueued = true;
                pendingJoins.emplace_back(c.linkId, std::move(*join));
            } else {
                ++joinsThisPoll;
                onJoin(c, std::move(*join), t);
            }
            return;
        }
        Participant* p = participantOf(c);
        if (p == nullptr) return strike(c, "message before Join");
        if (auto* in = std::get_if<wire::MatchInputPacket>(&message)) onInput(c, *p, *in, t);
        else if (auto* ack = std::get_if<wire::MatchSnapshotAck>(&message)) onAck(c, *p, *ack);
        else if (auto* ctl = std::get_if<wire::MatchControl>(&message)) onControl(c, *p, *ctl, t);
        else if (auto* leave = std::get_if<wire::Leave>(&message)) onLeave(c, *p, *leave);
        else strike(c, "unexpected message type");
    }

    void drainInbox(Clock::time_point t) {
        for (std::size_t n = pendingJoins.size(); n > 0 && joinsThisPoll < config.joinsPerPoll; --n) {
            auto [linkId, join] = std::move(pendingJoins.front());
            pendingJoins.pop_front();
            const auto it = conns.find(linkId);
            if (it == conns.end() || it->second.doomed || it->second.closing) continue;
            ++joinsThisPoll;
            it->second.joinQueued = false;
            onJoin(it->second, std::move(join), t);
        }
        runtime::InboundEvent ev;
        while (inbox.tryPop(ev)) {
            switch (ev.kind) {
                case runtime::InboundEvent::Kind::Connected: onConnected(ev, t); break;
                case runtime::InboundEvent::Kind::Disconnected: onDisconnected(ev.linkId, ev.detail); break;
                case runtime::InboundEvent::Kind::Message: {
                    const auto it = conns.find(ev.linkId);
                    if (it != conns.end()) onMessage(it->second, std::move(ev.message), t);
                    break;
                }
            }
            ev = runtime::InboundEvent{};
        }
    }

    // ---- observation: epoch and state changes ----
    void broadcastControl(wire::MatchControlKind kind, const std::string& detail = {}) {
        for (auto& [id, p] : parts) {
            Connection* c = connectionOf(p);
            if (c == nullptr) continue;
            wire::MatchControl ctl;
            ctl.kind = kind;
            ctl.sessionId = p.sessionId;
            ctl.runEpoch = epoch.load();
            ctl.detail = truncated(detail, wire::kMaxDetailBytes);
            send(*c, std::move(ctl));
        }
    }

    void observe(const runtime::RunnerObservation& obs) {
        if (!haveObservation) {
            haveObservation = true;
            runnerEpoch = obs.runEpoch;
        } else if (obs.runEpoch != runnerEpoch) {
            runnerEpoch = obs.runEpoch;
            epoch.store(epoch.load() + 1);
            for (auto& [id, p] : parts) {
                resetPeerStream(p);
                p.seenInput = false;
                p.lastInputSeq = wire::kNoInputSeq;
            }
            log.info("match", "world restarted: run epoch " + std::to_string(epoch.load()));
            broadcastControl(wire::MatchControlKind::Reset);
        }
        if (obs.matchState != state) {
            const wire::MatchState previous = state;
            state = obs.matchState;
            if (state == wire::MatchState::Paused) broadcastControl(wire::MatchControlKind::Pause);
            else if (previous == wire::MatchState::Paused && state == wire::MatchState::Running) broadcastControl(wire::MatchControlKind::Resume);
            for (auto& [id, p] : parts) p.lastIdleSend.reset();   // tell everyone the new state at once
        }
    }

    // ---- commands ----
    void kickNow(wire::ParticipantId id, const std::string& detail) {
        const auto it = parts.find(id);
        if (it == parts.end()) return;
        Participant& p = it->second;
        ++stats.kicks;
        kickedTokens.insert(p.token);
        if (Connection* c = connectionOf(p)) {
            wire::MatchControl ctl;
            ctl.kind = wire::MatchControlKind::Kick;
            ctl.sessionId = p.sessionId;
            ctl.runEpoch = epoch.load();
            ctl.target = id;
            ctl.detail = truncated(detail, wire::kMaxDetailBytes);
            c->closing = true;
            c->closingSince = now();
            send(*c, std::move(ctl), true);
        }
        log.info("match", "participant " + std::to_string(id) + " (" + p.name + ") was kicked: " + detail);
        removeParticipant(id);
    }

    void closeAll(const std::string& detail) {
        if (closed) return;
        closed = true;
        for (auto& [id, c] : conns) {
            if (c.doomed) continue;
            Participant* p = participantOf(c);
            if (p != nullptr) {
                wire::MatchControl ctl;
                ctl.kind = wire::MatchControlKind::ServerClosing;
                ctl.sessionId = p->sessionId;
                ctl.runEpoch = epoch.load();
                ctl.detail = truncated(detail, wire::kMaxDetailBytes);
                c.closing = true;
                c.closingSince = now();
                send(c, std::move(ctl), true);
            } else if (!c.closing) {
                wire::JoinReject r;
                r.reason = wire::JoinRejectReason::HostClosing;
                r.detail = truncated(detail, wire::kMaxDetailBytes);
                c.closing = true;
                c.closingSince = now();
                send(c, std::move(r), true);
            }
        }
    }

    void removeAllRobots() {
        std::vector<wire::ParticipantId> ids;
        for (const auto& [id, p] : parts) ids.push_back(id);
        for (const wire::ParticipantId id : ids) removeParticipant(id);
    }

    bool outboxesIdle(std::chrono::milliseconds timeout) {
        const auto deadline = Clock::now() + timeout;
        bool idle = true;
        for (auto& [id, c] : conns) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
            idle = c.outbox->waitIdle(std::max(left, std::chrono::milliseconds{0})) && idle;
        }
        return idle;
    }

    void processCommands() {
        std::vector<Command> batch;
        {
            std::lock_guard lock(commandMutex);
            batch.swap(commands);
        }
        for (Command& cmd : batch) {
            if (cmd.kind == Command::Kind::Kick) kickNow(cmd.target, cmd.detail);
            else {
                closeAll(cmd.detail);
                removeAllRobots();
            }
        }
    }

    // ---- timeouts ----
    void runTimeouts(Clock::time_point t) {
        for (auto& [id, c] : conns) {
            if (c.doomed) continue;
            countMalformed(c);
            if (c.closing) {
                if (t - c.closingSince > config.joinTimeout) doom(c, "close timed out");
            } else if (c.participant == wire::kNoParticipant) {
                if (!c.joinQueued && t - c.connectedAt > config.joinTimeout) {
                    ++stats.joinTimeouts;
                    doom(c, "no Join within the join timeout");
                }
            } else if (t - c.lastRx > config.peerTimeout) {
                ++stats.peerTimeouts;
                doom(c, "no packets within the peer timeout");
            }
            if (!c.doomed && (c.outbox->failed() || c.outbox->stalled())) {
                ++stats.slowPeersDropped;
                doom(c, c.outbox->failed() ? "send failed" : "a send is stuck: the peer is too slow");
            }
        }
        std::vector<wire::ParticipantId> expired;
        for (const auto& [id, p] : parts)
            if (p.linkId == 0 && t - p.detachedAt > config.resumeWindow) expired.push_back(id);
        for (const wire::ParticipantId id : expired) {
            log.info("match", "participant " + std::to_string(id) + " did not return within the resume window: robot removed");
            removeParticipant(id);
        }
    }

    // ---- snapshots ----
    wire::MatchInfo makeInfo(const sdk::ISimSnapshot& snap) const {
        wire::MatchInfo info;
        info.state = state;
        const proto::DsState ds = session.driverStation();
        info.enabled = ds.enabled;
        info.mode = ds.mode;
        info.eStop = ds.eStop;
        info.matchTimeS = static_cast<float>(ds.matchTimeS);
        if (!config.phaseHudKey.empty())
            if (const sdk::Value* v = snap.hudValue(config.phaseHudKey); v != nullptr && v->isString())
                info.phaseKey = truncated(v->asString(), wire::kMaxNameBytes);
        return info;
    }

    const WireFrame& ensureFrame(std::optional<WireFrame>& frame) {
        if (!frame) {
            const std::shared_ptr<const sdk::ISimSnapshot> snap = session.buildSnapshot(state == wire::MatchState::Paused, state != wire::MatchState::Lobby);
            frame = buildWireFrame(*snap, hudKeys, makeInfo(*snap));
            stats.hudKeysUnknown += frame->hudKeysUnknown;
        }
        return *frame;
    }

    std::size_t encodedSize(const wire::MatchSnapshot& snapshot) {
        scratch.clear();
        if (!services.codec->encode(snapshot, scratch)) return config.snapshotBudgetBytes + 1;
        return scratch.size();
    }

    // Completes the shared frame for one client and queues it. Robots, info and HUD always go; entity changes fill the
    // rest of the budget (see entity_delta.h for the order and the baseline rules).
    void sendSnapshot(Connection& c, Participant& p, const WireFrame& frame) {
        proto::Message message = wire::MatchSnapshot{};
        wire::MatchSnapshot& s = std::get<wire::MatchSnapshot>(message);
        s.sessionId = p.sessionId;
        s.runEpoch = epoch.load();
        if (++p.snapshotSeq == 0) p.snapshotSeq = 1;
        s.snapshotSeq = p.snapshotSeq;
        s.tick = frame.tick;
        s.simTimeUs = frame.simTimeUs;
        if (const auto info = session.remoteRobotInfo(p.id)) {
            s.ackInputSeq = info->stats.lastAppliedSeq;
            s.ackTick = info->stats.ackTick;
            s.inputLeadTicks = static_cast<std::int16_t>(std::clamp<std::int64_t>(info->stats.inputLeadTicks, INT16_MIN, INT16_MAX));
        }
        s.info = frame.info;
        s.robots = frame.robots;
        s.gameHud = frame.gameHud;

        const EntityPlan plan = p.encoder.plan(frame.groups, frame.anchors);
        const SnapshotFit fit = fitSnapshot(s, plan, config.snapshotBudgetBytes, [this](const wire::MatchSnapshot& m) { return encodedSize(m); }, p.entitySize);
        if (fit.coreOverBudget) ++stats.snapshotsOverBudget;
        if (fit.entityChanges < plan.changes.size()) {
            ++stats.snapshotsTruncated;
            stats.entityChangesDeferred += plan.changes.size() - fit.entityChanges;
        }
        stats.maxSnapshotBytes = std::max(stats.maxSnapshotBytes, fit.encodedBytes);
        p.encoder.commit(plan, fit.entityChanges, s.snapshotSeq);
        ++p.snapshotsSent;
        ++stats.snapshotsSent;
        send(c, std::move(message));
    }

    void adaptRate(Connection& c, Participant& p, Clock::time_point t) {
        if (!p.rate.sampleDue(t)) return;
        p.rate.update(c.link->linkStats(), t);
    }

    void tickSnapshots(Clock::time_point t) {
        const std::int64_t simTime = session.simTimeUs();
        std::optional<WireFrame> frame;
        for (auto& [id, p] : parts) {
            Connection* c = connectionOf(p);
            if (c == nullptr || c->doomed || c->closing) continue;
            adaptRate(*c, p, t);
            const std::int64_t interval = std::max<std::int64_t>(1, std::llround(1e6 / std::max<std::uint16_t>(1, p.rate.hz())));
            const std::int64_t slot = simTime / interval;
            if (slot == p.lastSlot) continue;
            p.lastSlot = slot;
            sendSnapshot(*c, p, ensureFrame(frame));
        }
    }

    void idleSnapshots(Clock::time_point t) {
        std::optional<WireFrame> frame;
        for (auto& [id, p] : parts) {
            Connection* c = connectionOf(p);
            if (c == nullptr || c->doomed || c->closing) continue;
            if (p.lastIdleSend && t - *p.lastIdleSend < config.idleSnapshotInterval) continue;
            p.lastIdleSend = t;
            adaptRate(*c, p, t);
            sendSnapshot(*c, p, ensureFrame(frame));
        }
    }
};

// ---- MatchHost ----------------------------------------------------------------------------------------------------

MatchHost::MatchHost(MatchHostConfig config, IMatchHostSession& session, MatchHostServices services)
    : impl_(std::make_unique<Impl>(std::move(config), session, std::move(services))) {}

MatchHost::~MatchHost() {
    Impl& m = *impl_;
    if (m.acceptor) m.acceptor->stop();
    if (!m.closed) m.closeAll("the server is shutting down");   // no session calls: the robots are the composition root's to remove
    m.outboxesIdle(m.config.closeFlushTimeout);
    m.conns.clear();
}

sdk::Status MatchHost::listen(std::unique_ptr<proto::ITransportServer> server) {
    Impl& m = *impl_;
    if (m.acceptor) return sdk::makeError(sdk::ErrorCode::InvalidState, "the match host is already listening");
    m.acceptor = std::make_unique<runtime::MessageLinkAcceptor>(std::move(server), m.services.codec, m.inbox, m.services.logger);
    m.acceptor->setMalformedFrameLimit(m.config.maxStrikes);
    return m.acceptor->start();
}

std::string MatchHost::endpoint() const { return impl_->acceptor ? impl_->acceptor->endpoint() : std::string(); }

void MatchHost::addConnection(std::unique_ptr<proto::ITransport> transport) {
    Impl& m = *impl_;
    auto link = std::make_shared<runtime::ChannelMessageLink>(m.nextAddedLinkId.fetch_add(1), std::move(transport), m.services.codec, m.inbox);
    link->setMalformedFrameLimit(m.config.maxStrikes);
    runtime::InboundEvent ev;
    ev.kind = runtime::InboundEvent::Kind::Connected;
    ev.linkId = link->id();
    ev.link = link;
    m.inbox.push(std::move(ev));
    link->start();
}

void MatchHost::kick(wire::ParticipantId participant, std::string detail) {
    std::lock_guard lock(impl_->commandMutex);
    Command c;
    c.kind = Command::Kind::Kick;
    c.target = participant;
    c.detail = std::move(detail);
    impl_->commands.push_back(std::move(c));
}

void MatchHost::close(std::string detail) {
    std::lock_guard lock(impl_->commandMutex);
    Command c;
    c.kind = Command::Kind::Close;
    c.detail = std::move(detail);
    impl_->commands.push_back(std::move(c));
}

void MatchHost::closeNow(std::string detail) {
    Impl& m = *impl_;
    m.closeAll(detail);
    m.removeAllRobots();
    m.outboxesIdle(m.config.closeFlushTimeout);
    m.sweep(m.now());
}

void MatchHost::onPoll(const runtime::RunnerObservation& observation) {
    Impl& m = *impl_;
    const Clock::time_point t = m.now();
    m.joinsThisPoll = 0;
    m.observe(observation);
    m.processCommands();
    m.drainInbox(t);
    m.runTimeouts(t);
    if (m.state != wire::MatchState::Running && !m.closed) m.idleSnapshots(t);
    if (m.rosterDirty || !m.lastRoster || t - *m.lastRoster >= m.config.rosterInterval) m.broadcastRoster(t);
    m.sweep(t);
}

void MatchHost::onTickEnd(const runtime::RunnerObservation& observation) {
    Impl& m = *impl_;
    m.observe(observation);
    if (m.closed) return;
    m.tickSnapshots(m.now());
    m.sweep(m.now());
}

HostConstraints MatchHost::constraints() const {
    HostConstraints c;
    c.peersConnected = impl_->joined.load() > 0;
    c.lockTimeScale = c.peersConnected;
    c.stepDisallowed = c.peersConnected;
    return c;
}

std::size_t MatchHost::joinedCount() const { return impl_->joined.load(); }
std::uint64_t MatchHost::runEpoch() const { return impl_->epoch.load(); }

bool MatchHost::waitOutboundIdle(std::chrono::milliseconds timeout) const { return impl_->outboxesIdle(timeout); }

std::vector<ParticipantInfo> MatchHost::participants() const {
    std::vector<ParticipantInfo> out;
    for (const auto& [id, p] : impl_->parts) {
        ParticipantInfo i;
        i.id = id;
        i.name = p.name;
        i.station = p.station;
        i.connected = p.linkId != 0;
        i.snapshotRateHz = p.rate.hz();
        i.inputsSubmitted = p.inputsSubmitted;
        i.inputsRateLimited = p.inputsRateLimited;
        i.snapshotsSent = p.snapshotsSent;
        i.baselineId = p.encoder.baselineId();
        out.push_back(std::move(i));
    }
    return out;
}

MatchHostStats MatchHost::stats() const { return impl_->stats; }

}  // namespace wpiemu::core::match
