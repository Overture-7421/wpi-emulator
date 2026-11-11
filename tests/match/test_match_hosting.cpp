// The composed practice-match host (app/host MatchHosting): invite, listener, SecureTransport on every peer, MatchHost over a
// real SimulationSession and real Box3D robots, through real sockets (loopback ENet and an in-process relay).
#include <doctest/doctest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "../../relay/tests/relay_stack_support.h"
#include "../runtime/remote_robot_rig.h"
#include "host/constrained_controller.h"
#include "host/match_hosting.h"
#include "host/tests/hosting_test_client.h"
#include "match_test_support.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/net/secure_transport.h"

namespace {

using namespace std::chrono_literals;
using namespace wpiemu;
namespace host = wpiemu::app::host;
namespace wire = wpiemu::sdk::match;
namespace proto = wpiemu::sdk::proto;
namespace rt = wpiemu::core::runtime;
using wpiemu::sdk::testing::waitUntil;

// The sim thread of a real runner: poll, tick, tick end. Everything that touches the session happens here.
class SimLoop {
public:
    SimLoop(remote_test::Rig& rig, host::MatchHosting& hosting) : rig_(rig), hosting_(hosting) {}
    ~SimLoop() { stop(); }
    void start() {
        thread_ = std::thread([this] {
            while (!stop_.load()) {
                rt::RunnerObservation obs{static_cast<wire::MatchState>(state_.load()), 1};
                hosting_.onPoll(obs);
                rig_.session->tick(rig_.frame, false);
                hosting_.onTickEnd(obs);
                std::this_thread::sleep_for(5ms);
            }
        });
    }
    void stop() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }
    void setState(wire::MatchState state) { state_.store(static_cast<int>(state)); }

private:
    remote_test::Rig& rig_;
    host::MatchHosting& hosting_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<int> state_{static_cast<int>(wire::MatchState::Running)};
};

using Client = host::testing::HostingTestClient;

std::string spec() { return match_test::specJson(match_test::swerveSpec()); }

// Everything one hosting needs around a real session, with the log captured.
struct HostingRig {
    remote_test::Rig rig;
    match_test::PipelineKit kit;
    sdk::testing::CapturingLogger log;
    core::match::SimulationMatchSession adapter{*rig.session};
    std::mutex constraintsMutex;
    std::vector<core::match::HostConstraints> constraints;
    std::unique_ptr<host::MatchHosting> hosting;
    std::string inviteText;   // as first shown: the status no longer has it after shutdown

    explicit HostingRig(host::HostingOptions options) {
        host::MatchHostingServices services;
        services.session = &adapter;
        services.pipeline = &kit.pipeline;
        services.logger = &log;
        services.onConstraints = [this](const core::match::HostConstraints& c) {
            std::lock_guard lock(constraintsMutex);
            constraints.push_back(c);
        };
        auto created = host::MatchHosting::create(std::move(options), std::move(services));
        REQUIRE_MESSAGE(created.ok(), (created.ok() ? "" : created.error().message));
        hosting = std::move(created).value();
        inviteText = hosting->status()->invite;
    }
    std::shared_ptr<const host::HostingStatus> status() const { return hosting->status(); }
    net::InviteSecret invite() const {
        auto parsed = net::parseInvite(status()->invite);
        REQUIRE(parsed.ok());
        return std::move(parsed).value();
    }
    std::size_t constraintChanges() {
        std::lock_guard lock(constraintsMutex);
        return constraints.size();
    }
};

host::HostingOptions directOptions() {
    host::HostingOptions o;
    o.mode = host::HostingMode::Direct;
    o.bindHost = "127.0.0.1";
    o.port = 0;
    return o;
}

std::string withoutDashes(std::string text) {
    text.erase(std::remove(text.begin(), text.end(), '-'), text.end());
    return text;
}

// The secret may appear in the status only as the invite field, and nowhere in the log.
void checkSecretStaysPrivate(const HostingRig& h) {
    const std::string& invite = h.inviteText;
    REQUIRE_FALSE(invite.empty());
    const std::string bare = withoutDashes(invite);
    const auto leaks = [&](const std::string& text) { return text.find(invite) != std::string::npos || text.find(bare) != std::string::npos; };
    const auto status = h.status();
    CHECK_FALSE(leaks(status->detail));
    CHECK_FALSE(leaks(status->endpoint));
    for (const host::HostingParticipant& p : status->roster) CHECK_FALSE(leaks(p.name));
    for (const auto& entry : h.log.entries()) CHECK_FALSE(leaks(entry.message));
}

}  // namespace

TEST_CASE("hosting: create refuses an off mode, bad options and missing services") {
    remote_test::Rig rig;
    match_test::PipelineKit kit;
    core::match::SimulationMatchSession adapter(*rig.session);
    host::MatchHostingServices services;
    services.session = &adapter;
    services.pipeline = &kit.pipeline;

    CHECK_FALSE(host::MatchHosting::create(host::HostingOptions{}, services).ok());
    host::HostingOptions relay;
    relay.mode = host::HostingMode::Relay;   // no relay address
    CHECK_FALSE(host::MatchHosting::create(relay, services).ok());
    host::MatchHostingServices empty;
    CHECK_FALSE(host::MatchHosting::create(directOptions(), empty).ok());
    CHECK(host::MatchHosting::create(directOptions(), services).ok());
}

TEST_CASE("hosting: the status before start has the invite, the host robot and no endpoint") {
    HostingRig h(directOptions());
    const auto status = h.status();
    CHECK(status->mode == host::HostingMode::Direct);
    CHECK(status->phase == host::HostingPhase::Starting);
    CHECK(status->endpoint.empty());
    CHECK(status->invite.size() == net::InviteSecret::kTextChars + 3);
    CHECK(net::parseInvite(status->invite).ok());
    REQUIRE(status->roster.size() == 1);
    CHECK(status->roster[0].isHost);
    CHECK(status->maxPlayers == wire::kMaxRobotsPerMatch);
    CHECK_FALSE(status->accepting());
}

TEST_CASE("hosting: a direct match over loopback joins, drives, snapshots, kicks and shuts down cleanly") {
    HostingRig h(directOptions());
    SimLoop loop(h.rig, *h.hosting);
    h.hosting->start();
    loop.start();
    REQUIRE(waitUntil([&] { return h.status()->accepting() && h.status()->phase == host::HostingPhase::Live; }, 5s));
    const auto up = h.status();
    CHECK(up->phase == host::HostingPhase::Live);
    CHECK(up->endpoint.rfind("enet://", 0) == 0);
    CHECK(up->port != 0);
    const std::string endpoint = "enet://127.0.0.1:" + std::to_string(up->port);

    auto peer = Client::connect(endpoint, h.invite());
    REQUIRE(peer != nullptr);
    REQUIRE(peer->join("Alice", spec()));
    REQUIRE(peer->accept.has_value());
    const wire::ParticipantId id = peer->accept->participantId;
    CHECK(id != wire::kHostParticipantId);
    CHECK(peer->accept->roster.size() == 2);

    // The roster in the status follows (the host first, then the client with its link figures).
    REQUIRE(waitUntil([&] { return h.status()->roster.size() == 2; }, 3s));
    CHECK(h.status()->roster[0].isHost);
    CHECK(h.status()->roster[1].id == id);
    CHECK(h.status()->roster[1].name == "Alice");
    CHECK(h.status()->roster[1].connected);
    CHECK(h.status()->joinsAccepted == 1);
    // A client holds the match: 1x time scale and no single stepping.
    REQUIRE(waitUntil([&] { return h.status()->timeScaleLocked && h.status()->stepDisabled; }, 3s));
    CHECK(h.hosting->constraints().lockTimeScale);

    // Inputs reach the real remote robot: it drives, snapshots show it moving and acknowledge the input.
    std::optional<wire::RobotWireState> start;
    const auto deadline = std::chrono::steady_clock::now() + 6s;
    double moved = 0.0;
    while (std::chrono::steady_clock::now() < deadline && moved < 400.0) {
        peer->drive(40.f);
        peer->pump(16ms);
        if (const auto robot = peer->myRobot()) {
            if (!start) start = robot;
            moved = std::hypot(double(robot->xMm - start->xMm), double(robot->yMm - start->yMm));
        }
    }
    CHECK(moved >= 400.0);
    REQUIRE(peer->lastSnapshot.has_value());
    CHECK(peer->lastSnapshot->robots.size() == 2);   // the host's robot and this client's
    CHECK(peer->lastSnapshot->sessionId == peer->accept->sessionId);
    CHECK(peer->lastSnapshot->ackInputSeq != wire::kNoInputSeq);
    CHECK(peer->lastSnapshot->ackTick != proto::kNoAckTick);
    CHECK(peer->lastSnapshot->info.state == wire::MatchState::Running);
    CHECK(peer->count<wire::Roster>() >= 1);

    // The host pauses: the lobby/paused state reaches the status and the clients.
    loop.setState(wire::MatchState::Paused);
    REQUIRE(waitUntil([&] { return h.status()->phase == host::HostingPhase::Paused; }, 3s));
    REQUIRE(peer->waitFor([&] { return peer->lastSnapshot && peer->lastSnapshot->info.state == wire::MatchState::Paused; }));
    loop.setState(wire::MatchState::HostStalled);
    REQUIRE(waitUntil([&] { return h.status()->phase == host::HostingPhase::HostStalled; }, 3s));
    loop.setState(wire::MatchState::Running);
    REQUIRE(waitUntil([&] { return h.status()->phase == host::HostingPhase::Live; }, 3s));

    checkSecretStaysPrivate(h);

    // Kick: the client is told and disconnected, its robot leaves the roster and the constraints lapse.
    h.hosting->kick(id, "kicked by the test");
    REQUIRE(peer->waitFor([&] { return peer->gotControl(wire::MatchControlKind::Kick) || peer->closed; }));
    CHECK(peer->gotControl(wire::MatchControlKind::Kick));
    REQUIRE(waitUntil([&] { return h.status()->roster.size() == 1; }, 3s));
    CHECK(h.status()->kicks == 1);
    REQUIRE(waitUntil([&] { return !h.status()->timeScaleLocked; }, 3s));
    {
        std::lock_guard lock(h.constraintsMutex);
        REQUIRE(h.constraints.size() >= 2);
        CHECK(h.constraints.front().lockTimeScale);
        CHECK_FALSE(h.constraints.back().lockTimeScale);
    }

    // A second client is still welcome, then the host shuts down: the client is told and the port is free again.
    auto second = Client::connect(endpoint, h.invite());
    REQUIRE(second != nullptr);
    REQUIRE(second->join("Bob", spec()));
    loop.stop();   // from here this thread plays the sim thread
    h.hosting->shutdown();
    CHECK(h.status()->phase == host::HostingPhase::Off);
    CHECK(h.rig.session->remoteRobotIds().empty());   // the clients' robots left the world
    CHECK(second->waitFor([&] { return second->gotControl(wire::MatchControlKind::ServerClosing) || second->closed; }));
    net::EnetServerOptions again;
    again.bindHost = "127.0.0.1";
    again.port = up->port;
    net::EnetServerTransport rebound(again);
    CHECK(rebound.listen().ok());
    checkSecretStaysPrivate(h);
}

TEST_CASE("hosting: connections that do not hold the invite never reach the match") {
    HostingRig h(directOptions());
    SimLoop loop(h.rig, *h.hosting);
    h.hosting->start();
    loop.start();
    REQUIRE(waitUntil([&] { return h.status()->accepting(); }, 5s));
    const std::string endpoint = "enet://127.0.0.1:" + std::to_string(h.status()->port);

    // Another invite: the handshake fails before any game frame is parsed.
    auto stranger = net::generateInvite();
    REQUIRE(stranger.ok());
    std::string error;
    CHECK(Client::connect(endpoint, stranger.value(), &error) == nullptr);
    CHECK_FALSE(error.empty());

    // No handshake at all: a plaintext Join is not understood, never answered and never joins.
    auto plain = Client::connectPlain(endpoint);
    REQUIRE(plain != nullptr);
    wire::Join join;
    join.displayName = "Mallory";
    join.robotSpecJson = spec();
    plain->send(join);
    plain->pump(500ms);
    CHECK(plain->count<wire::JoinAccept>() == 0);

    REQUIRE(waitUntil([&] { return h.status()->handshakesFailed >= 2; }, 5s));
    CHECK(h.status()->joinsAccepted == 0);
    CHECK(h.status()->roster.size() == 1);

    // The real invite still works afterwards.
    auto good = Client::connect(endpoint, h.invite());
    REQUIRE(good != nullptr);
    CHECK(good->join("Alice", spec()));
    loop.stop();
    CHECK(h.rig.session->remoteRobotIds().size() == 1);   // only the client that held the invite
    h.hosting->shutdown();
    checkSecretStaysPrivate(h);
}

TEST_CASE("hosting: the player limit counts the host and a full match rejects the next join") {
    host::HostingOptions o = directOptions();
    o.maxPlayers = 2;
    HostingRig h(o);
    SimLoop loop(h.rig, *h.hosting);
    h.hosting->start();
    loop.start();
    REQUIRE(waitUntil([&] { return h.status()->accepting(); }, 5s));
    const std::string endpoint = "enet://127.0.0.1:" + std::to_string(h.status()->port);

    auto first = Client::connect(endpoint, h.invite());
    REQUIRE(first != nullptr);
    REQUIRE(first->join("Alice", spec()));
    auto second = Client::connect(endpoint, h.invite());
    REQUIRE(second != nullptr);
    CHECK_FALSE(second->join("Bob", spec()));
    REQUIRE(second->count<wire::JoinReject>() == 1);
    for (const proto::Message& m : second->log)
        if (const auto* r = std::get_if<wire::JoinReject>(&m)) CHECK(r->reason == wire::JoinRejectReason::RoomFull);
    loop.stop();
    h.hosting->shutdown();
}

TEST_CASE("hosting: a port that is taken is reported as an error status, not a crash") {
    net::EnetServerOptions taken;
    taken.bindHost = "127.0.0.1";
    taken.port = 0;
    net::EnetServerTransport blocker(taken);
    REQUIRE(blocker.listen().ok());
    const auto bound = net::NetEndpoint::parse(blocker.endpoint());
    REQUIRE(bound.ok());

    host::HostingOptions o = directOptions();
    o.port = bound.value().port;
    HostingRig h(o);
    h.hosting->start();
    REQUIRE(waitUntil([&] { return h.status()->phase == host::HostingPhase::Error; }, 5s));
    CHECK_FALSE(h.status()->detail.empty());
    CHECK_FALSE(h.status()->accepting());
    h.hosting->shutdown();
    CHECK(h.status()->phase == host::HostingPhase::Off);
}

TEST_CASE("hosting: destroying a hosting that never started or whose session is gone is safe") {
    {
        HostingRig h(directOptions());   // never started
    }
    {
        HostingRig h(directOptions());
        h.hosting->start();
        REQUIRE(waitUntil([&] { return h.status()->accepting(); }, 5s));
        // no shutdown(): the destructor stops the threads and closes the listener without touching the session
    }
}

TEST_CASE("hosting relay: the room is registered, joined end to end, kept opaque to the relay and closed with the host") {
    relaystack::StackRelay relay;
    REQUIRE(relay.started());
    host::HostingOptions o;
    o.mode = host::HostingMode::Relay;
    o.relayEndpoint = "127.0.0.1:" + std::to_string(relay.port());
    HostingRig h(o);
    SimLoop loop(h.rig, *h.hosting);
    h.hosting->start();
    loop.start();
    REQUIRE(waitUntil([&] { return h.status()->accepting(); }, 8s));
    const auto up = h.status();
    const std::string room = net::roomId(h.invite());
    CHECK(up->endpoint == relay.endpoint(room));
    CHECK(relay.stats().rooms == 1);

    auto peer = Client::connect(relay.endpoint(room), h.invite());
    REQUIRE(peer != nullptr);
    const std::string marker = "PlainTextMarkerNameXyz";
    REQUIRE(peer->join(marker, spec()));
    REQUIRE(waitUntil([&] { return h.status()->roster.size() == 2; }, 3s));
    CHECK(h.status()->roster[1].name == marker);

    std::optional<wire::RobotWireState> start;
    const auto deadline = std::chrono::steady_clock::now() + 6s;
    double moved = 0.0;
    while (std::chrono::steady_clock::now() < deadline && moved < 400.0) {
        peer->drive(40.f);
        peer->pump(16ms);
        if (const auto robot = peer->myRobot()) {
            if (!start) start = robot;
            moved = std::hypot(double(robot->xMm - start->xMm), double(robot->yMm - start->yMm));
        }
    }
    CHECK(moved >= 400.0);

    // The relay sees opaque bytes: neither the robot name, nor the invite, nor its dash-free form.
    CHECK(relay.capture->gameFrames() > 0);
    CHECK_FALSE(relay.capture->gameContains(relaystack::bytesOf(marker)));
    CHECK_FALSE(relay.capture->anyContains(relaystack::bytesOf(h.status()->invite)));
    CHECK_FALSE(relay.capture->anyContains(relaystack::bytesOf(withoutDashes(h.status()->invite))));
    checkSecretStaysPrivate(h);

    loop.stop();
    h.hosting->shutdown();
    CHECK(h.status()->phase == host::HostingPhase::Off);
    CHECK(peer->waitFor([&] { return peer->gotControl(wire::MatchControlKind::ServerClosing) || peer->closed; }));
    // CloseRoom freed the room at once.
    CHECK(relaystack::waitFor([&] { return relay.stats().rooms == 0; }, 5s));
    CHECK(relay.stats().roomsClosed == 1);
}

TEST_CASE("hosting relay: an unreachable relay is an error status and shutdown does not hang") {
    host::HostingOptions o;
    o.mode = host::HostingMode::Relay;
    o.relayEndpoint = "127.0.0.1:1";   // nothing listens there
    HostingRig h(o);
    h.hosting->start();
    const auto begun = std::chrono::steady_clock::now();
    REQUIRE(waitUntil([&] { return h.status()->phase == host::HostingPhase::Error; }, 15s));
    CHECK_FALSE(h.status()->detail.empty());
    h.hosting->shutdown();
    CHECK(std::chrono::steady_clock::now() - begun < 20s);
}

TEST_CASE("hosting relay: stopping while the relay registration is in progress cancels it") {
    host::HostingOptions o;
    o.mode = host::HostingMode::Relay;
    o.relayEndpoint = "10.255.255.1:9";   // not routable: the connect attempt just waits
    HostingRig h(o);
    h.hosting->start();
    std::this_thread::sleep_for(200ms);
    CHECK(h.status()->phase == host::HostingPhase::Starting);
    const auto begun = std::chrono::steady_clock::now();
    h.hosting->shutdown();
    CHECK(std::chrono::steady_clock::now() - begun < 3s);
}

