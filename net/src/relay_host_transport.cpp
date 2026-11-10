#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <variant>

#include "enet_pump.h"
#include "relay_support.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/sdk/core/interface.h"

namespace wpiemu::net {

using sdk::ErrorCode;
using sdk::makeError;
using sdk::proto::Delivery;
namespace relay = sdk::match::relay;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

namespace {

constexpr milliseconds kPollSlice{50};
constexpr milliseconds kReadSlice{100};
// Frames that overtook their PeerJoined, and ids that left recently (their late frames must not reach a new peer).
constexpr milliseconds kStashTtl{2000};
constexpr std::size_t kStashMaxIds = 8;
constexpr std::size_t kStashMaxFramesPerId = 16;
constexpr std::size_t kStashMaxFrames = 64;

// What a virtual peer needs from the host transport that produced it.
class RelayUplink {
    WPIEMU_INTERFACE(RelayUplink)
public:
    // Sends an already enveloped frame to the relay; Disconnected while there is no live link.
    virtual sdk::Status sendToRelay(std::span<const std::byte> frame, Delivery delivery) = 0;
    virtual std::optional<sdk::proto::LinkStats> relayStats() const = 0;
};

struct PeerState {
    PeerState(relay::RelayPeerId peerId, std::size_t maxFrames, std::size_t maxBytes, std::string text,
              std::weak_ptr<RelayUplink> uplinkRef)
        : id(peerId), inbox(maxFrames, maxBytes), description(std::move(text)), uplink(std::move(uplinkRef)) {}

    // Local close: nothing buffered is delivered. Remote close (the client left, the relay link was lost): what was buffered
    // is delivered first, then Disconnected.
    void closeLocal() {
        open.store(false);
        inbox.close(true);
    }
    void closeRemote() {
        open.store(false);
        inbox.close(false);
    }

    const relay::RelayPeerId id;
    detail::FrameInbox inbox;
    const std::string description;
    const std::weak_ptr<RelayUplink> uplink;
    std::atomic<bool> open{true};
    std::atomic<std::uint64_t> bytesIn{0};
    std::atomic<std::uint64_t> bytesOut{0};
    std::atomic<std::uint64_t> dropped{0};
};

class VirtualPeer final : public sdk::proto::ITransport {
public:
    explicit VirtualPeer(std::shared_ptr<PeerState> state) : state_(std::move(state)) {}
    ~VirtualPeer() override { state_->closeLocal(); }

    sdk::Status send(std::span<const std::byte> frame) override { return sendFrame(frame, Delivery::Reliable); }
    sdk::Status sendUnreliable(std::span<const std::byte> frame) override { return sendFrame(frame, Delivery::Unreliable); }
    sdk::Result<bool> receive(std::vector<std::byte>& frame, milliseconds timeout) override {
        return state_->inbox.pop(frame, timeout);
    }
    void close() override { state_->closeLocal(); }
    bool connected() const override { return state_->open.load(); }
    std::string describe() const override { return state_->description; }
    std::optional<sdk::proto::LinkStats> linkStats() const override {
        // Round trip and loss are those of the host's relay link; the counters are this peer's own.
        sdk::proto::LinkStats stats;
        if (const auto uplink = state_->uplink.lock())
            if (const auto relayStats = uplink->relayStats()) stats = *relayStats;
        stats.bytesIn = state_->bytesIn.load();
        stats.bytesOut = state_->bytesOut.load();
        stats.droppedFrames = state_->dropped.load();
        return stats;
    }

private:
    sdk::Status sendFrame(std::span<const std::byte> frame, Delivery delivery) {
        if (!state_->open.load()) return makeError(ErrorCode::Disconnected, "relay peer closed: " + state_->description);
        if (frame.empty()) return makeError(ErrorCode::InvalidArgument, "empty frames are not supported");
        const std::size_t limit = (delivery == Delivery::Unreliable ? relay::kRelayMaxDatagram : relay::kRelayMaxReliablePacket) -
                                  relay::kRelayEnvelopeBytes;
        if (frame.size() > limit)
            return makeError(ErrorCode::InvalidArgument, "frame of " + std::to_string(frame.size()) +
                                                             " bytes exceeds the relay limit of " + std::to_string(limit));
        const auto uplink = state_->uplink.lock();
        if (!uplink) return makeError(ErrorCode::Disconnected, "relay host transport is gone: " + state_->description);
        detail::writeEnvelope(scratch_, state_->id, frame);
        sdk::Status sent = uplink->sendToRelay(scratch_, delivery);
        if (sent) state_->bytesOut.fetch_add(frame.size());
        return sent;
    }

    std::shared_ptr<PeerState> state_;
    std::vector<std::byte> scratch_;
};

struct Session {
    std::shared_ptr<sdk::proto::ITransport> link;
    IControlLink* control = nullptr;
};

struct Attempt {
    Session session;
    sdk::Error error;
    bool ok = false;
    bool permanent = false;  // retrying cannot help (the relay refused the room for good, or we were stopped)
};

}  // namespace

RelayHostOptions relayHostOptionsFor(const std::string& relayHost, std::uint16_t relayPort, const InviteSecret& secret) {
    RelayHostOptions options;
    options.relayHost = relayHost;
    options.relayPort = relayPort;
    options.roomId = roomId(secret);
    options.hostAuth = hostAuth(secret);
    return options;
}

struct RelayHostTransport::Impl final : RelayUplink, std::enable_shared_from_this<Impl> {
    explicit Impl(RelayHostOptions o) : options(std::move(o)) {
        NetEndpoint ep;
        ep.scheme = EndpointScheme::Relay;
        ep.host = options.relayHost;
        ep.port = options.relayPort;
        ep.roomId = options.roomId;
        endpointText = ep.format();
        options.link.maxReliableFrameBytes = relay::kRelayMaxReliablePacket + relay::kRelayEnvelopeBytes;
        options.link.maxUnreliableFrameBytes = relay::kRelayMaxDatagram + relay::kRelayEnvelopeBytes;
    }
    ~Impl() override { shutdown(); }

    // ---- RelayUplink
    sdk::Status sendToRelay(std::span<const std::byte> frame, Delivery delivery) override {
        std::shared_ptr<sdk::proto::ITransport> link;
        {
            std::lock_guard lock(linkMutex);
            link = currentLink;
        }
        if (!link) return makeError(ErrorCode::Disconnected, "the relay link is down");
        return delivery == Delivery::Unreliable ? link->sendUnreliable(frame) : link->send(frame);
    }
    std::optional<sdk::proto::LinkStats> relayStats() const override {
        std::shared_ptr<sdk::proto::ITransport> link;
        {
            std::lock_guard lock(linkMutex);
            link = currentLink;
        }
        return link ? link->linkStats() : std::nullopt;
    }

    // ---- lifecycle
    sdk::Status listen() {
        std::lock_guard lifecycle(lifecycleMutex);
        if (stop.load()) return makeError(ErrorCode::InvalidState, "the relay host transport is closed");
        if (listening) return makeError(ErrorCode::InvalidState, "already listening on " + endpointText);
        if (!relay::isValidRoomId(options.roomId)) return makeError(ErrorCode::InvalidArgument, "bad room id: " + options.roomId);
        if (options.relayHost.empty() || options.relayPort == 0)
            return makeError(ErrorCode::InvalidArgument, "a relay host and port are required");
        Attempt first = establish();
        if (!first.ok) return first.error;
        listening = true;
        setState(State::Registered);
        supervisor = std::thread([this, session = std::move(first.session)]() mutable { supervise(std::move(session)); });
        return sdk::okStatus();
    }

    sdk::Result<std::unique_ptr<sdk::proto::ITransport>> accept(milliseconds timeout) {
        std::unique_lock lock(mutex);
        if (state == State::Idle) return makeError(ErrorCode::InvalidState, "accept() before listen()");
        cv.wait_for(lock, timeout, [this] { return !pending.empty() || state == State::Dead; });
        if (!pending.empty()) {
            std::shared_ptr<PeerState> peer = std::move(pending.front());
            pending.pop_front();
            return std::unique_ptr<sdk::proto::ITransport>(std::make_unique<VirtualPeer>(std::move(peer)));
        }
        if (state == State::Dead) return deadError;
        return std::unique_ptr<sdk::proto::ITransport>();
    }

    void shutdown() {
        {
            std::lock_guard lock(mutex);
            stop.store(true);
            if (state != State::Dead) {
                state = State::Dead;
                deadError = makeError(ErrorCode::Disconnected, "relay host transport closed");
            }
        }
        cv.notify_all();
        std::shared_ptr<sdk::proto::ITransport> link;
        {
            std::lock_guard lock(linkMutex);
            link = currentLink;
        }
        if (link) link->close();
        link.reset();
        std::lock_guard lifecycle(lifecycleMutex);
        if (supervisor.joinable()) supervisor.join();
        dropAllPeers();
    }

    bool registered() const {
        std::lock_guard lock(mutex);
        return state == State::Registered;
    }
    std::size_t peerCount() const {
        std::lock_guard lock(mutex);
        return static_cast<std::size_t>(std::count_if(peers.begin(), peers.end(), [](const auto& entry) {
            return entry.second->open.load();
        }));
    }

    enum class State { Idle, Registered, Reconnecting, Dead };

    // ---- connecting and registering
    Attempt establish() {
        Attempt attempt;
        const auto deadline = Clock::now() + options.connectTimeout;
        auto created = detail::EnetHostPump::createClient(options.link, options.relayHost, options.relayPort);
        if (!created) {
            attempt.error = created.error();
            attempt.permanent = created.error().code == ErrorCode::InvalidArgument;
            return attempt;
        }
        auto [pump, peerLink] = std::move(created).value();
        detail::LinkState linkState = detail::LinkState::Connecting;
        while (!stop.load()) {
            linkState = peerLink->waitWhileConnecting(std::min(deadline, Clock::now() + kPollSlice));
            if (linkState != detail::LinkState::Connecting || Clock::now() >= deadline) break;
        }
        if (stop.load()) {
            peerLink->requestClose();
            pump->stop();
            attempt.error = makeError(ErrorCode::Cancelled, "stopped while connecting to the relay");
            attempt.permanent = true;
            return attempt;
        }
        if (linkState != detail::LinkState::Connected) {
            peerLink->requestClose();
            pump->stop();
            attempt.error = linkState == detail::LinkState::Closed
                                ? makeError(ErrorCode::Disconnected, "the relay refused the connection to " + endpointText)
                                : makeError(ErrorCode::Timeout, "no connection to the relay at " + endpointText + " within " +
                                                                    std::to_string(options.connectTimeout.count()) + " ms");
            return attempt;
        }
        std::shared_ptr<sdk::proto::ITransport> link = std::make_shared<detail::LinkTransport>(std::move(peerLink), std::move(pump));
        IControlLink* control = controlLinkOf(*link);

        relay::RegisterRoom registration;
        registration.protocolVersion = options.protocolVersion;
        registration.roomId = options.roomId;
        registration.hostAuth = options.hostAuth;
        if (sdk::Status sent = detail::sendRelayMessage(*control, registration); !sent) {
            attempt.error = sent.error();
            return attempt;
        }

        const auto registerDeadline = Clock::now() + options.registerTimeout;
        std::vector<std::byte> frame;
        for (;;) {
            if (stop.load()) {
                attempt.error = makeError(ErrorCode::Cancelled, "stopped while registering the room");
                attempt.permanent = true;
                return attempt;
            }
            const auto left = sdk::proto::timeLeft(registerDeadline);
            if (left.count() == 0) {
                attempt.error = makeError(ErrorCode::Timeout, "the relay did not answer the registration within " +
                                                                  std::to_string(options.registerTimeout.count()) + " ms");
                return attempt;
            }
            auto got = control->receiveControl(frame, std::min(left, kPollSlice));
            if (!got) {
                attempt.error = makeError(ErrorCode::Disconnected, "the relay closed the connection while registering");
                return attempt;
            }
            if (!got.value()) continue;
            auto message = detail::decodeRelayMessage(frame);
            if (!message) {
                attempt.error = makeError(ErrorCode::ProtocolError, "unreadable relay answer: " + message.error().message);
                attempt.permanent = true;
                return attempt;
            }
            if (std::holds_alternative<relay::RoomRegistered>(message.value())) break;
            if (const auto* rejected = std::get_if<relay::RoomRejected>(&message.value())) {
                attempt.error = detail::registerRejectError(rejected->reason);
                attempt.permanent = rejected->reason != relay::RoomRejectReason::RelayFull;
                return attempt;
            }
            attempt.error = makeError(ErrorCode::ProtocolError, "unexpected relay message while registering");
            attempt.permanent = true;
            return attempt;
        }
        attempt.session = Session{std::move(link), control};
        attempt.ok = true;
        return attempt;
    }

    // ---- the supervisor thread: runs the live session, then re-registers
    void supervise(Session session) {
        for (;;) {
            runSession(session);
            session = {};
            if (stop.load()) return;
            if (!options.autoReregister) {
                die(makeError(ErrorCode::Disconnected, "the relay connection was lost"));
                return;
            }
            setState(State::Reconnecting);
            const auto lostAt = Clock::now();
            milliseconds backoff = options.reconnectBackoffInitial;
            Attempt attempt = establish();
            while (!attempt.ok) {
                if (stop.load()) return;
                if (attempt.permanent) {
                    die(attempt.error);
                    return;
                }
                if (Clock::now() - lostAt >= options.reconnectGiveUp) {
                    die(makeError(ErrorCode::Timeout, "gave up re-registering the room: " + attempt.error.message));
                    return;
                }
                {
                    std::unique_lock lock(mutex);
                    cv.wait_for(lock, backoff, [this] { return stop.load(); });
                }
                if (stop.load()) return;
                backoff = std::min(backoff * 2, options.reconnectBackoffMax);
                attempt = establish();
            }
            reregisters.fetch_add(1);
            session = std::move(attempt.session);
        }
    }

    void runSession(Session& session) {
        {
            std::lock_guard lock(linkMutex);
            currentLink = session.link;
        }
        setState(State::Registered);
        std::atomic<bool> over{false};
        std::thread gameReader([&] { gameLoop(*session.link, over); });
        controlLoop(*session.control, over);
        over.store(true);
        {
            std::lock_guard lock(linkMutex);
            currentLink.reset();
        }
        session.link->close();
        gameReader.join();
        dropAllPeers();
    }

    void controlLoop(IControlLink& control, const std::atomic<bool>& over) {
        std::vector<std::byte> frame;
        while (!stop.load() && !over.load()) {
            auto got = control.receiveControl(frame, kReadSlice);
            if (!got) return;
            if (!got.value()) continue;
            auto message = detail::decodeRelayMessage(frame);
            if (!message) {
                ignoredControl.fetch_add(1);
                continue;
            }
            if (const auto* joined = std::get_if<relay::PeerJoined>(&message.value()))
                onPeerJoined(joined->peerId);
            else if (const auto* left = std::get_if<relay::PeerLeft>(&message.value()))
                onPeerLeft(left->peerId);
            else
                ignoredControl.fetch_add(1);
        }
    }

    void gameLoop(sdk::proto::ITransport& link, std::atomic<bool>& over) {
        IControlLink& control = *controlLinkOf(link);
        std::vector<std::byte> frame;
        Delivery delivery = Delivery::Reliable;
        while (!stop.load() && !over.load()) {
            auto got = control.receiveGame(frame, delivery, kReadSlice);
            if (!got) break;
            if (got.value()) onGameFrame(frame, delivery);
        }
        over.store(true);
    }

    // ---- relay events
    void onPeerJoined(relay::RelayPeerId id) {
        if (id == relay::kRelayHostPeer || id == relay::kRelayBroadcastPeer) {
            ignoredControl.fetch_add(1);
            return;
        }
        std::shared_ptr<PeerState> replaced;
        {
            std::lock_guard lock(mutex);
            if (state == State::Dead) return;
            expireStash();
            const auto existing = peers.find(id);
            if (existing != peers.end()) {
                replaced = std::move(existing->second);
                peers.erase(existing);
            } else if (peers.size() >= options.maxPeers) {
                ignoredControl.fetch_add(1);
                return;
            }
            recentlyLeft.erase(id);
            auto peer = std::make_shared<PeerState>(id, options.maxQueuedFramesPerPeer, options.maxQueuedBytesPerPeer,
                                                    endpointText + "(peer " + std::to_string(id) + ")",
                                                    std::weak_ptr<RelayUplink>(weak_from_this()));
            if (const auto held = stash.find(id); held != stash.end()) {
                for (auto& frame : held->second.frames) {
                    const std::size_t size = frame.size();
                    if (peer->inbox.push(std::move(frame)) == detail::FrameInbox::Push::Pushed) peer->bytesIn.fetch_add(size);
                }
                stashedFrames -= held->second.frames.size();
                stash.erase(held);
            }
            peers[id] = peer;
            pending.push_back(std::move(peer));
        }
        cv.notify_all();
        if (replaced) replaced->closeRemote();
    }

    void onPeerLeft(relay::RelayPeerId id) {
        std::shared_ptr<PeerState> peer;
        {
            std::lock_guard lock(mutex);
            if (const auto found = peers.find(id); found != peers.end()) {
                peer = std::move(found->second);
                peers.erase(found);
            }
            if (const auto held = stash.find(id); held != stash.end()) {
                stashedFrames -= held->second.frames.size();
                stash.erase(held);
            }
            recentlyLeft[id] = Clock::now();
        }
        if (peer) peer->closeRemote();
    }

    void onGameFrame(std::vector<std::byte>& frame, Delivery delivery) {
        relay::RelayPeerId id = 0;
        std::span<const std::byte> payload;
        if (!detail::readEnvelope(frame, id, payload)) {
            droppedFrames.fetch_add(1);
            return;
        }
        frame.erase(frame.begin(), frame.begin() + static_cast<std::ptrdiff_t>(relay::kRelayEnvelopeBytes));
        std::lock_guard lock(mutex);
        const auto found = peers.find(id);
        if (found == peers.end()) {
            stashUnknown(id, std::move(frame));
            return;
        }
        PeerState& peer = *found->second;
        if (!peer.open.load()) {
            peer.dropped.fetch_add(1);
            return;
        }
        const std::size_t size = frame.size();
        switch (peer.inbox.push(std::move(frame))) {
            case detail::FrameInbox::Push::Pushed: peer.bytesIn.fetch_add(size); break;
            case detail::FrameInbox::Push::Full:
                peer.dropped.fetch_add(1);
                if (delivery == Delivery::Reliable) peer.closeLocal();  // a reliable stream with a hole is unusable
                break;
            case detail::FrameInbox::Push::Closed: peer.dropped.fetch_add(1); break;
        }
    }

    // Caller holds `mutex`.
    void stashUnknown(relay::RelayPeerId id, std::vector<std::byte>&& frame) {
        expireStash();
        if (recentlyLeft.count(id) != 0) {
            droppedFrames.fetch_add(1);
            return;
        }
        auto held = stash.find(id);
        if (held == stash.end()) {
            if (stash.size() >= kStashMaxIds) {
                droppedFrames.fetch_add(1);
                return;
            }
            held = stash.emplace(id, Stashed{Clock::now(), {}}).first;
        }
        if (held->second.frames.size() >= kStashMaxFramesPerId || stashedFrames >= kStashMaxFrames) {
            droppedFrames.fetch_add(1);
            return;
        }
        held->second.frames.push_back(std::move(frame));
        ++stashedFrames;
    }

    // Caller holds `mutex`.
    void expireStash() {
        const auto now = Clock::now();
        for (auto it = stash.begin(); it != stash.end();) {
            if (now - it->second.since >= kStashTtl) {
                stashedFrames -= it->second.frames.size();
                droppedFrames.fetch_add(it->second.frames.size());
                it = stash.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = recentlyLeft.begin(); it != recentlyLeft.end();) {
            if (now - it->second >= kStashTtl)
                it = recentlyLeft.erase(it);
            else
                ++it;
        }
    }

    // The relay link is gone (or we are closing): every virtual peer reports Disconnected after its buffered frames.
    void dropAllPeers() {
        std::map<relay::RelayPeerId, std::shared_ptr<PeerState>> gone;
        {
            std::lock_guard lock(mutex);
            gone.swap(peers);
            for (const auto& peer : pending) gone.emplace(peer->id, peer);
            pending.clear();
            stash.clear();
            stashedFrames = 0;
            recentlyLeft.clear();
        }
        for (auto& entry : gone) entry.second->closeRemote();
    }

    void setState(State next) {
        {
            std::lock_guard lock(mutex);
            if (state == State::Dead) return;
            state = next;
        }
        cv.notify_all();
    }

    void die(sdk::Error error) {
        {
            std::lock_guard lock(mutex);
            if (state != State::Dead) {
                state = State::Dead;
                deadError = std::move(error);
            }
        }
        cv.notify_all();
    }

    struct Stashed {
        Clock::time_point since;
        std::deque<std::vector<std::byte>> frames;
    };

    RelayHostOptions options;
    std::string endpointText;

    mutable std::mutex mutex;
    std::condition_variable cv;
    State state = State::Idle;
    sdk::Error deadError;
    std::map<relay::RelayPeerId, std::shared_ptr<PeerState>> peers;
    std::deque<std::shared_ptr<PeerState>> pending;
    std::map<relay::RelayPeerId, Stashed> stash;
    std::size_t stashedFrames = 0;
    std::map<relay::RelayPeerId, Clock::time_point> recentlyLeft;

    mutable std::mutex linkMutex;
    std::shared_ptr<sdk::proto::ITransport> currentLink;

    std::mutex lifecycleMutex;
    bool listening = false;
    std::thread supervisor;
    std::atomic<bool> stop{false};
    std::atomic<std::size_t> reregisters{0};
    std::atomic<std::uint64_t> ignoredControl{0};
    std::atomic<std::uint64_t> droppedFrames{0};
};

RelayHostTransport::RelayHostTransport(RelayHostOptions options) : impl_(std::make_shared<Impl>(std::move(options))) {}

RelayHostTransport::~RelayHostTransport() {
    impl_->shutdown();
    impl_.reset();
}

sdk::Status RelayHostTransport::listen() { return impl_->listen(); }

std::string RelayHostTransport::endpoint() const { return impl_->endpointText; }

sdk::Result<std::unique_ptr<sdk::proto::ITransport>> RelayHostTransport::accept(std::chrono::milliseconds timeout) {
    return impl_->accept(timeout);
}

void RelayHostTransport::close() { impl_->shutdown(); }

bool RelayHostTransport::registered() const { return impl_->registered(); }

std::size_t RelayHostTransport::peerCount() const { return impl_->peerCount(); }

std::size_t RelayHostTransport::reregisterCount() const { return impl_->reregisters.load(); }

}  // namespace wpiemu::net
