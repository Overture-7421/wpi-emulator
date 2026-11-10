#include "host/match_hosting.h"

#include <algorithm>
#include <optional>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/net/os_random.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/net/secure_transport.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::app::host {

namespace {

namespace wire = sdk::match;

constexpr int kInviteAttempts = 4;                 // the relay refuses a room id that is taken: draw a new invite
constexpr std::chrono::milliseconds kAcceptSlice{100};
constexpr std::size_t kUnjoinedConnectionSlack = 3;

HostingPhase phaseOf(wire::MatchState state) {
    switch (state) {
        case wire::MatchState::Lobby: return HostingPhase::Lobby;
        case wire::MatchState::Running: return HostingPhase::Live;
        case wire::MatchState::Paused: return HostingPhase::Paused;
        case wire::MatchState::HostStalled: return HostingPhase::HostStalled;
    }
    return HostingPhase::Lobby;
}

bool sameConstraints(const core::match::HostConstraints& a, const core::match::HostConstraints& b) {
    return a.peersConnected == b.peersConnected && a.lockTimeScale == b.lockTimeScale && a.stepDisallowed == b.stepDisallowed;
}

}  // namespace

// ---- Impl --------------------------------------------------------------------------------------------------------------------

struct MatchHosting::Impl {
    enum class Stage { Starting, Listening, Failed };

    struct Handshake {
        std::thread thread;
        std::atomic<bool> done{false};
    };

    Impl(HostingOptions opts, MatchHostingServices svc, net::InviteSecret invite)
        : options(std::move(opts)), services(std::move(svc)), log(services.logger != nullptr ? *services.logger : nullLogger),
          secret(std::move(invite)) {
        inviteText = net::inviteText(secret);
    }

    HostingOptions options;
    MatchHostingServices services;
    sdk::NullLogger nullLogger;
    sdk::ILogger& log;
    std::shared_ptr<sdk::IWallClock> clock;
    std::unique_ptr<core::match::MatchHost> host;

    // Written only while Starting (a taken relay room draws a new one); read by handshakes once Listening.
    net::InviteSecret secret;
    std::string inviteText;

    // The hosting thread and its handshakes.
    std::thread thread;
    std::atomic<bool> stopping{false};
    std::atomic<bool> started{false};
    std::atomic<bool> shutDown{false};
    std::mutex serverMutex;
    std::shared_ptr<sdk::proto::ITransportServer> server;
    bool serverListening = false;
    std::list<std::unique_ptr<Handshake>> handshakes;   // owned and reaped by the hosting thread, joined by stopThreads
    std::atomic<std::uint64_t> handshakesFailed{0};

    // Published state. `mutex` guards everything below; the sim thread refreshes `view`, any thread may publish.
    mutable std::mutex mutex;
    Stage stage = Stage::Starting;
    std::string detail;
    std::string endpoint;
    std::uint16_t port = 0;
    struct SimView {
        wire::MatchState matchState = wire::MatchState::Lobby;
        std::uint64_t runEpoch = 0;
        std::vector<HostingParticipant> roster;
        core::match::HostConstraints constraints;
        core::match::MatchHostStats stats;
    } view;
    std::shared_ptr<const HostingStatus> current;

    // Sim-thread bookkeeping.
    std::optional<std::chrono::steady_clock::time_point> lastRefresh;
    std::size_t lastJoined = static_cast<std::size_t>(-1);
    core::match::HostConstraints lastConstraints;

    // ---- status ----
    HostingStatus compose() const {   // `mutex` held
        HostingStatus s;
        s.mode = options.mode;
        s.maxPlayers = options.maxPlayers;
        s.invite = inviteText;
        s.endpoint = endpoint;
        s.port = port;
        s.detail = detail;
        s.runEpoch = view.runEpoch;
        s.roster = view.roster;
        s.timeScaleLocked = view.constraints.lockTimeScale;
        s.stepDisabled = view.constraints.stepDisallowed;
        s.joinsAccepted = view.stats.joinsAccepted;
        s.joinsRejected = view.stats.joinsRejected;
        s.handshakesFailed = handshakesFailed.load();
        s.kicks = view.stats.kicks;
        switch (stage) {
            case Stage::Starting: s.phase = HostingPhase::Starting; break;
            case Stage::Failed: s.phase = HostingPhase::Error; break;
            case Stage::Listening: s.phase = phaseOf(view.matchState); break;
        }
        return s;
    }

    void publishLocked() {
        HostingStatus next = compose();
        if (current && *current == next) return;
        current = std::make_shared<const HostingStatus>(std::move(next));
    }

    void publish() {
        std::lock_guard lock(mutex);
        publishLocked();
    }

    void setStage(Stage next, std::string text) {
        std::lock_guard lock(mutex);
        stage = next;
        detail = std::move(text);
        publishLocked();
    }

    // ---- the listener ----
    std::shared_ptr<sdk::proto::ITransportServer> makeServer() const {
        const std::size_t clientSlots = options.maxPlayers - 1 + kUnjoinedConnectionSlack;
        if (options.mode == HostingMode::Direct) {
            net::EnetServerOptions o;
            o.bindHost = options.bindHost;
            o.port = options.port;
            o.maxPeers = clientSlots;
            return std::make_shared<net::EnetServerTransport>(o);
        }
        const sdk::Result<RelayAddress> relay = parseRelayAddress(options.relayEndpoint);
        net::RelayHostOptions o = net::relayHostOptionsFor(relay.value().host, relay.value().port, secret);
        o.maxPeers = clientSlots;
        return std::make_shared<net::RelayHostTransport>(std::move(o));
    }

    void renewInvite() {
        auto fresh = services.randomFill ? net::generateInvite(services.randomFill, wire::relay::kRelayPingRoomId) : net::generateInvite();
        if (!fresh) return;
        std::lock_guard lock(mutex);
        secret = std::move(fresh).value();
        inviteText = net::inviteText(secret);
    }

    void closeServer() {
        std::shared_ptr<sdk::proto::ITransportServer> s;
        {
            std::lock_guard lock(serverMutex);
            s = server;
        }
        if (s) s->close();
    }

    void reapHandshakes(bool all) {
        for (auto it = handshakes.begin(); it != handshakes.end();) {
            if (all || (*it)->done.load()) {
                if ((*it)->thread.joinable()) (*it)->thread.join();
                it = handshakes.erase(it);
            } else {
                ++it;
            }
        }
    }

    void startHandshake(std::unique_ptr<sdk::proto::ITransport> raw) {
        reapHandshakes(false);
        if (handshakes.size() >= kMaxPendingHandshakes) {
            ++handshakesFailed;
            log.warn("hosting", "too many connections are still handshaking; one was refused");
            raw->close();
            return;
        }
        auto entry = std::make_unique<Handshake>();
        Handshake* h = entry.get();
        h->thread = std::thread([this, h, peer = std::move(raw)]() mutable {
            // Every connection proves it holds the invite before MatchHost sees any frame from it.
            auto secured = net::SecureTransport::acceptAsHost(std::move(peer), secret, services.handshakeTimeout);
            if (!secured) {
                ++handshakesFailed;
                log.warn("hosting", "a connection failed the secure handshake: " + secured.error().message);
            } else if (!stopping.load()) {
                host->addConnection(std::move(secured).value());
            }
            h->done.store(true);
        });
        handshakes.push_back(std::move(entry));
    }

    void run() {
        std::shared_ptr<sdk::proto::ITransportServer> listener;
        sdk::Status listening = sdk::okStatus();
        for (int attempt = 0; attempt < kInviteAttempts && !stopping.load(); ++attempt) {
            listener = makeServer();
            {
                std::lock_guard lock(serverMutex);
                server = listener;
            }
            listening = listener->listen();
            if (listening) break;
            const bool taken = options.mode == HostingMode::Relay && listening.error().code == sdk::ErrorCode::AlreadyExists;
            if (!taken || attempt + 1 == kInviteAttempts) break;
            log.info("hosting", "the relay room is taken; drawing a new invite");
            renewInvite();
        }
        if (stopping.load()) return;
        if (!listening) {
            log.error("hosting", "cannot host: " + listening.error().message);
            setStage(Stage::Failed, "Cannot host: " + listening.error().message);
            return;
        }
        {
            std::lock_guard lock(serverMutex);
            serverListening = true;
        }
        const std::string text = listener->endpoint();
        {
            std::lock_guard lock(mutex);
            endpoint = text;
            if (options.mode == HostingMode::Direct) {
                const auto parsed = net::NetEndpoint::parse(text);
                port = parsed ? parsed.value().port : options.port;
            }
            stage = Stage::Listening;
            detail.clear();
            publishLocked();
        }
        log.info("hosting", "hosting a practice match on " + text);

        while (!stopping.load()) {
            auto accepted = listener->accept(kAcceptSlice);
            if (!accepted) {
                if (!stopping.load()) {
                    log.error("hosting", "the listener stopped: " + accepted.error().message);
                    setStage(Stage::Failed, "The connection point was lost: " + accepted.error().message);
                }
                break;
            }
            if (accepted.value() == nullptr) {
                reapHandshakes(false);
                continue;
            }
            startHandshake(std::move(accepted).value());
        }
    }

    // Stops the hosting thread and every handshake. Does not touch the session.
    void stopThreads() {
        stopping.store(true);
        {
            std::lock_guard lock(serverMutex);
            // A listen() still in progress (the relay registration) is cancelled by closing; a listener that is up is closed last.
            if (server && !serverListening) server->close();
        }
        if (thread.joinable()) thread.join();
        reapHandshakes(true);
    }

    // ---- sim thread ----
    void refresh(const core::runtime::RunnerObservation& obs) {
        const core::match::HostConstraints constraints = host->constraints();
        if (!sameConstraints(constraints, lastConstraints)) {
            lastConstraints = constraints;
            if (services.onConstraints) services.onConstraints(constraints);
        }
        const auto now = clock->now();
        const std::size_t joined = host->joinedCount();
        bool due = !lastRefresh || now - *lastRefresh >= kStatusRefresh || joined != lastJoined;
        {
            std::lock_guard lock(mutex);
            due = due || obs.matchState != view.matchState;
        }
        if (!due) return;
        lastRefresh = now;
        lastJoined = joined;

        std::vector<HostingParticipant> roster;
        HostingParticipant self;
        self.id = wire::kHostParticipantId;
        self.name = services.session->hostName();
        self.station = services.session->driverStation().station;
        self.isHost = true;
        roster.push_back(std::move(self));
        for (const core::match::ParticipantInfo& p : host->participants()) {
            HostingParticipant r;
            r.id = p.id;
            r.name = p.name;
            r.station = p.station;
            r.connected = p.connected;
            r.pingMs = p.rttMs;
            r.lossPermille = p.lossPermille;
            r.snapshotRateHz = p.snapshotRateHz;
            roster.push_back(std::move(r));
        }
        std::lock_guard lock(mutex);
        view.matchState = obs.matchState;
        view.runEpoch = host->runEpoch();
        view.roster = std::move(roster);
        view.constraints = constraints;
        view.stats = host->stats();
        publishLocked();
    }
};

// ---- MatchHosting -------------------------------------------------------------------------------------------------------------

sdk::Result<std::unique_ptr<MatchHosting>> MatchHosting::create(HostingOptions options, MatchHostingServices services) {
    if (options.mode == HostingMode::Off) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "hosting is off");
    if (services.session == nullptr || services.pipeline == nullptr)
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, "hosting needs a session and a config pipeline");
    if (const sdk::Status valid = validateHosting(options); !valid) return valid.error();

    auto invite = services.randomFill ? net::generateInvite(services.randomFill, wire::relay::kRelayPingRoomId) : net::generateInvite();
    if (!invite) return sdk::makeError(invite.error().code, "cannot create an invite: " + invite.error().message);

    if (!services.clock) services.clock = std::make_shared<sdk::SteadyWallClock>();
    if (!services.randomFill) services.randomFill = [](std::span<std::byte> out) { return net::fillRandom(out); };

    core::match::MatchHostConfig config = services.matchConfig;
    config.maxPlayers = options.maxPlayers;
    config.maxConnections = options.maxPlayers - 1 + kUnjoinedConnectionSlack;
    config.phaseHudKey = options.phaseHudKey;

    core::match::MatchHostServices hostServices;
    hostServices.codec = std::make_shared<ipc::BinaryMessageCodec>();
    hostServices.clock = services.clock;
    hostServices.logger = services.logger;
    hostServices.specPipeline = services.pipeline;
    hostServices.randomFill = services.randomFill;

    auto impl = std::make_unique<Impl>(options, std::move(services), std::move(invite).value());
    impl->clock = impl->services.clock;
    impl->host = std::make_unique<core::match::MatchHost>(std::move(config), *impl->services.session, std::move(hostServices));
    {
        std::lock_guard lock(impl->mutex);
        HostingParticipant self;
        self.id = wire::kHostParticipantId;
        self.name = impl->services.session->hostName();
        self.station = impl->services.session->driverStation().station;
        self.isHost = true;
        impl->view.roster.push_back(std::move(self));
        impl->publishLocked();
    }
    return std::unique_ptr<MatchHosting>(new MatchHosting(std::move(impl), std::move(options)));
}

MatchHosting::MatchHosting(std::unique_ptr<Impl> impl, HostingOptions options) : options_(std::move(options)), impl_(std::move(impl)) {}

MatchHosting::~MatchHosting() {
    if (impl_->shutDown.exchange(true)) return;
    impl_->stopThreads();   // the session may be gone already: no ServerClosing and no robot removal here
    impl_->closeServer();
}

void MatchHosting::start() {
    Impl& m = *impl_;
    if (m.started.exchange(true) || m.shutDown.load()) return;
    m.thread = std::thread([&m] { m.run(); });
}

std::shared_ptr<const HostingStatus> MatchHosting::status() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->current;
}

core::match::HostConstraints MatchHosting::constraints() const { return impl_->host->constraints(); }

void MatchHosting::kick(sdk::match::ParticipantId participant, std::string detail) { impl_->host->kick(participant, std::move(detail)); }

void MatchHosting::onPoll(const core::runtime::RunnerObservation& observation) {
    if (impl_->shutDown.load()) return;
    impl_->host->onPoll(observation);
    impl_->refresh(observation);
}

void MatchHosting::onTickEnd(const core::runtime::RunnerObservation& observation) {
    if (impl_->shutDown.load()) return;
    impl_->host->onTickEnd(observation);
}

void MatchHosting::shutdown() {
    Impl& m = *impl_;
    if (m.shutDown.exchange(true)) return;
    m.stopThreads();
    m.host->closeNow("the host closed the match");
    m.closeServer();
    if (m.lastConstraints.peersConnected && m.services.onConstraints) m.services.onConstraints({});
    std::lock_guard lock(m.mutex);
    HostingStatus off;
    off.phase = HostingPhase::Off;
    off.maxPlayers = m.options.maxPlayers;
    m.current = std::make_shared<const HostingStatus>(std::move(off));
}

}  // namespace wpiemu::app::host
