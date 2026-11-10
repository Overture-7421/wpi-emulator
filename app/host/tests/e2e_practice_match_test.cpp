// End-to-end test of a practice match (opt-in, label `e2e`, ctest name e2e.practice_match):
//
//   copy of samples/swerve_robot -> real RobotSession (gradle simulateJava + halsim_wpiemu, Box3D world, 2026 plugin) hosting a
//   match in RELAY mode -> production-configured relay on loopback (or the one named by WPIEMU_E2E_RELAY) -> end-to-end encrypted
//   SecureTransport -> scripted players (MatchClient over the net transports).
//
// One robot launch carries every phase, because Gradle and the robot start dominate the run time:
//   1  the relay probe against the relay
//   2  the host starts hosting; a player joins; each side sees the other's robot in its snapshots
//   3  the host's WPILib robot and the player drive; the player's chassis collides with the host's robot
//   4  a player over an impaired network (ImpairedTransport uplink with loss and latency, delayed downlink)
//   5  attacks while the match runs: wrong invite, plaintext peer, tampered and replayed records, oversize frames, a flood
//   6  link loss and resume, kick and a fresh join
//   7  stop: ServerClosing to the players, CloseRoom frees the room, the robot process and the relay port go away
//
//   $env:WPIEMU_E2E = "1"; ctest -C Release -R e2e.practice_match --output-on-failure
//   $env:WPIEMU_E2E_PROJECT = "C:\path\to\robot"   another robot with the swerve sample's CAN layout
//   $env:WPIEMU_E2E_KEEP = "1"                     keep the temp copy for inspection
//   $env:WPIEMU_E2E_VERBOSE = "1"                  echo the robot console
//   $env:WPIEMU_E2E_RELAY_PORT = "47777"           bind the in-process relay to a fixed UDP port (so clumsy/netem can target it)
//   $env:WPIEMU_E2E_RELAY = "host:port"            use an already running relay (for example the deployed dev relay) instead of
//                                                  the in-process one; the checks that read the relay's counters are skipped
// tools/e2e_netem.ps1 runs it under packet loss and latency (see docs/guides/practice-match-validation.md).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include "../../../relay/tests/relay_stack_support.h"
#include "host/robot_session.h"
#include "hosting_test_client.h"
#include "swerve_sample.h"
#include "test_support.h"
#include "wpiemu/core/launcher/project_inspector.h"
#include "wpiemu/core/match/match_client.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/net/secure_transport.h"
#include "wpiemu/relay/relay_probe.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/testing/impaired_transport.h"
#include "wpiemu/sdk/testing/wait.h"

namespace fs = std::filesystem;
using namespace wpiemu;
using namespace wpiemu::app::host::testing;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
namespace mc = wpiemu::core::match;
namespace wire = wpiemu::sdk::match;
namespace proto = wpiemu::sdk::proto;
namespace wr = wpiemu::relay;

#ifndef WPIEMU_E2E_EXTENSION
#define WPIEMU_E2E_EXTENSION ""
#endif
#ifndef WPIEMU_E2E_DEFAULT_PROJECT
#define WPIEMU_E2E_DEFAULT_PROJECT ""
#endif

namespace {

using Wrap = HostingTestClient::Wrap;

fs::path projectUnderTest() {
    return !env("WPIEMU_E2E_PROJECT").empty() ? fs::path(env("WPIEMU_E2E_PROJECT")) : fs::path(WPIEMU_E2E_DEFAULT_PROJECT);
}

// ---- the relay -------------------------------------------------------------------------------------------------------------

// The production-configured in-process relay, or an already running one named by WPIEMU_E2E_RELAY.
struct RelayUnderTest {
    std::unique_ptr<relaystack::StackRelay> local;
    std::string host = "127.0.0.1";
    std::uint16_t port = 0;

    RelayUnderTest() {
        if (const std::string external = env("WPIEMU_E2E_RELAY"); !external.empty()) {
            const auto parsed = net::NetEndpoint::parse("relay://" + external);
            REQUIRE_MESSAGE(parsed.ok(), "WPIEMU_E2E_RELAY must be host:port");
            host = parsed.value().host;
            port = parsed.value().port;
            return;
        }
        const std::string fixed = env("WPIEMU_E2E_RELAY_PORT");
        local = std::make_unique<relaystack::StackRelay>(relaystack::StackRelay::Tweak{},
                                                         fixed.empty() ? std::uint16_t{0} : static_cast<std::uint16_t>(std::stoi(fixed)));
        REQUIRE_MESSAGE(local->started(), local->startError());
        port = local->port();
    }
    bool inProcess() const { return local != nullptr; }
    std::string address() const { return host + ":" + std::to_string(port); }
    std::string endpoint(const std::string& room) const { return "relay://" + address() + "/" + room; }
    // Counters of the in-process relay; zeros for an external one (callers guard with inProcess()).
    wr::RelayStats stats() const { return local ? local->stats() : wr::RelayStats{}; }
};

// ---- transports of the scripted players -----------------------------------------------------------------------------------

// Sits between the raw transport and the encryption handshake: it sees (and can damage) exactly what the relay sees.
class AttackProxy final : public proto::ITransport {
public:
    struct Controls {
        std::atomic<int> flip{0};        // the next N outgoing frames get one byte flipped
        std::atomic<int> duplicate{0};   // the next N outgoing frames are sent twice
        std::atomic<bool> cut{false};    // the link breaks: it closes and every call fails
        std::atomic<std::uint64_t> flipped{0};
        std::atomic<std::uint64_t> duplicated{0};
    };

    AttackProxy(std::unique_ptr<proto::ITransport> inner, std::shared_ptr<Controls> controls)
        : inner_(std::move(inner)), controls_(std::move(controls)) {}

    sdk::Status send(std::span<const std::byte> frame) override { return forward(frame, false); }
    sdk::Status sendUnreliable(std::span<const std::byte> frame) override { return forward(frame, true); }
    sdk::Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        if (controls_->cut) return lost().error();
        return inner_->receive(frame, timeout);
    }
    void close() override { inner_->close(); }
    bool connected() const override { return !controls_->cut && inner_->connected(); }
    std::string describe() const override { return "attack(" + inner_->describe() + ")"; }
    std::optional<proto::LinkStats> linkStats() const override { return inner_->linkStats(); }

private:
    static sdk::Status lost() { return sdk::makeError(sdk::ErrorCode::Disconnected, "the link was cut"); }
    sdk::Status deliver(std::span<const std::byte> frame, bool unreliable) { return unreliable ? inner_->sendUnreliable(frame) : inner_->send(frame); }

    sdk::Status forward(std::span<const std::byte> frame, bool unreliable) {
        if (controls_->cut) {
            inner_->close();
            return lost();
        }
        if (controls_->flip.load() > 0 && !frame.empty()) {
            --controls_->flip;
            std::vector<std::byte> damaged(frame.begin(), frame.end());
            damaged[damaged.size() / 2] ^= std::byte{0x40};
            ++controls_->flipped;
            return deliver(damaged, unreliable);
        }
        const sdk::Status sent = deliver(frame, unreliable);
        if (sent.ok() && controls_->duplicate.load() > 0) {
            --controls_->duplicate;
            ++controls_->duplicated;
            return deliver(frame, unreliable);
        }
        return sent;
    }

    std::unique_ptr<proto::ITransport> inner_;
    std::shared_ptr<Controls> controls_;
};

// Delays what is received (latency plus jitter) without ever reordering it, so reliable frames keep their guarantees.
class DelayedReceive final : public proto::ITransport {
public:
    DelayedReceive(std::unique_ptr<proto::ITransport> inner, std::chrono::milliseconds delay, std::chrono::milliseconds jitter, unsigned seed)
        : inner_(std::move(inner)), delay_(delay), jitter_(jitter), rng_(seed) {}

    sdk::Status send(std::span<const std::byte> frame) override { return inner_->send(frame); }
    sdk::Status sendUnreliable(std::span<const std::byte> frame) override { return inner_->sendUnreliable(frame); }
    sdk::Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        const auto deadline = Clock::now() + timeout;
        for (;;) {
            const auto now = Clock::now();
            if (!queue_.empty() && queue_.front().due <= now) {
                frame = std::move(queue_.front().bytes);
                queue_.pop_front();
                return true;
            }
            if (queue_.empty() && failure_) return *failure_;
            if (now >= deadline) return false;
            const auto wake = queue_.empty() ? deadline : std::min(deadline, queue_.front().due);
            pullWithin(std::clamp(std::chrono::duration_cast<std::chrono::milliseconds>(wake - now), 1ms, 5ms));
        }
    }
    void close() override { inner_->close(); }
    bool connected() const override { return inner_->connected(); }
    std::string describe() const override { return "delayed(" + inner_->describe() + ")"; }
    std::optional<proto::LinkStats> linkStats() const override { return inner_->linkStats(); }

private:
    struct Held {
        Clock::time_point due;
        std::vector<std::byte> bytes;
    };

    // Takes one frame from the inner transport when it arrives within `wait`. True when it queued a frame.
    bool pullWithin(std::chrono::milliseconds wait) {
        if (failure_) return false;
        std::vector<std::byte> bytes;
        auto got = inner_->receive(bytes, wait);
        if (!got) {
            failure_ = got.error();
            return false;
        }
        if (!got.value()) return false;
        const auto jitter = std::chrono::milliseconds(jitter_.count() > 0 ? static_cast<long long>(rng_() % static_cast<unsigned>(jitter_.count() + 1)) : 0);
        Clock::time_point due = Clock::now() + delay_ + jitter;
        if (!queue_.empty()) due = std::max(due, queue_.back().due);
        queue_.push_back({due, std::move(bytes)});
        return true;
    }

    std::unique_ptr<proto::ITransport> inner_;
    std::chrono::milliseconds delay_;
    std::chrono::milliseconds jitter_;
    std::mt19937 rng_;
    std::deque<Held> queue_;
    std::optional<sdk::Error> failure_;
};

sdk::SteadyWallClock& wallClock() {
    static sdk::SteadyWallClock clock;
    return clock;
}

// What the in-process impaired network does to a player: uplink 40 ms +- 20 ms and 6 % loss of the unreliable frames,
// downlink 30 ms + up to 20 ms (ImpairedTransport and DelayedReceive wrap the real relay transport).
Wrap impairedNetwork(unsigned seed) {
    return [seed](std::unique_ptr<proto::ITransport> raw) -> std::unique_ptr<proto::ITransport> {
        sdk::testing::ImpairmentProfile profile;
        profile.latency = 40ms;
        profile.jitter = 20ms;
        profile.lossProbability = 0.06;
        auto delayed = std::make_unique<DelayedReceive>(std::move(raw), 30ms, 20ms, seed);
        return std::make_unique<sdk::testing::ImpairedTransport>(std::move(delayed), wallClock(), profile, seed);
    };
}

sdk::Result<std::unique_ptr<proto::ITransport>> dial(const std::string& endpoint, const net::InviteSecret& secret, const Wrap& wrap = {}) {
    net::RelayClientTransport relay;
    auto connected = relay.connect(endpoint, 5s);
    if (!connected) return connected.error();
    std::unique_ptr<proto::ITransport> transport = std::move(connected).value();
    if (wrap) transport = wrap(std::move(transport));
    return net::SecureTransport::connectAsClient(std::move(transport), secret, 3s);
}

// ---- scripted players ------------------------------------------------------------------------------------------------------

class Player;

// Every player of a test is updated by whoever waits: a MatchClient needs update() often (>= 120 Hz).
struct Players {
    std::vector<Player*> active;
    template <class Done>
    bool until(Done&& done, std::chrono::milliseconds limit, std::chrono::milliseconds slice = 4ms);
    void forAtLeast(std::chrono::milliseconds duration) {
        until([] { return false; }, duration);
    }
};

class Player {
public:
    Player(Players& players, std::string name, const std::string& pluginId) : players_(players), name_(std::move(name)), pluginId_(pluginId) {
        players_.active.push_back(this);
        mc::MatchClientConfig config;
        config.displayName = name_;
        config.joinTimeout = 8s;
        mc::MatchClientServices services;
        services.codec = std::make_shared<ipc::BinaryMessageCodec>();
        services.clock = std::shared_ptr<sdk::IWallClock>(std::shared_ptr<void>(), &wallClock());
        services.specPipeline = &registries_.pipeline;
        services.plugins = &registries_.plugins;
        services.drivetrains = &registries_.drivetrains;
        services.tireModels = &registries_.tireModels;
        client_ = std::make_unique<mc::MatchClient>(config, std::move(services));
    }
    ~Player() {
        client_.reset();
        players_.active.erase(std::remove(players_.active.begin(), players_.active.end(), this), players_.active.end());
    }
    Player(const Player&) = delete;
    Player& operator=(const Player&) = delete;

    // The robot a player brings: the sample's swerve chassis without the mappings and mechanisms only the host's robot has.
    sdk::RobotConfig spec() const {
        sdk::RobotConfig c = swerveSampleMappedConfig();
        c.name = name_;
        c.drivetrain.mappings.clear();
        c.robotMappings.clear();
        c.mechanisms.clear();
        c.gamePluginId = pluginId_;
        return c;
    }
    std::string specJson() const { return registries_.json.dump(registries_.codec.toValue(spec()), false); }

    // Dials and joins; true once the host accepted the robot (Lobby or Live).
    bool join(const std::string& endpoint, const net::InviteSecret& secret, const Wrap& wrap = {}) {
        endpoint_ = endpoint;
        secret_ = secret;
        wrap_ = wrap;
        auto link = dial(endpoint, secret, wrap);
        if (!link) {
            lastError = link.error().message;
            return false;
        }
        const sdk::Status started = client_->connect(std::move(link).value(), spec());
        if (!started) {
            lastError = started.error().message;
            return false;
        }
        return players_.until([&] { return joined(); }, 10s) && joined();
    }
    // Rejoins over a new link with the resume token of the last JoinAccept.
    bool resume() {
        auto link = dial(endpoint_, *secret_, wrap_);
        if (!link) {
            lastError = link.error().message;
            return false;
        }
        const sdk::Status started = client_->resume(std::move(link).value());
        if (!started) {
            lastError = started.error().message;
            return false;
        }
        return players_.until([&] { return joined(); }, 10s) && joined();
    }

    bool joined() const {
        const mc::ConnectionState s = client_->status().state;
        return s == mc::ConnectionState::Lobby || s == mc::ConnectionState::Live;
    }
    mc::MatchClient& client() { return *client_; }
    std::shared_ptr<const mc::MatchClientView> view() const { return client_->view(); }
    mc::ConnectionStatus status() const { return client_->status(); }
    wire::ParticipantId id() const { return view() ? view()->own.id : wire::kNoParticipant; }
    sdk::Vec3 position() const { return view() ? view()->own.pose.position : sdk::Vec3{}; }
    std::optional<sdk::Vec3> hostPosition() const {
        const auto v = view();
        if (v)
            for (const mc::RemoteRobotView& r : v->remotes)
                if (r.isHost) return r.pose.position;
        return std::nullopt;
    }
    void update() { client_->update(); }
    std::string why() const { return lastError + " " + client_->status().detail; }

    std::string lastError;

private:
    Players& players_;
    std::string name_;
    std::string pluginId_;
    TestRegistries registries_;
    std::unique_ptr<mc::MatchClient> client_;
    std::string endpoint_;
    std::optional<net::InviteSecret> secret_;
    Wrap wrap_;
};

template <class Done>
bool Players::until(Done&& done, std::chrono::milliseconds limit, std::chrono::milliseconds slice) {
    const auto deadline = Clock::now() + limit;
    for (;;) {
        for (Player* p : active) p->update();
        if (done()) return true;
        if (Clock::now() >= deadline) return false;
        std::this_thread::sleep_for(slice);
    }
}

double planar(const sdk::Vec3& a, const sdk::Vec3& b) { return std::hypot(double(a.x) - double(b.x), double(a.y) - double(b.y)); }

// ---- the match under test --------------------------------------------------------------------------------------------------

const sdk::RobotSample* remoteIn(const sdk::ISimSnapshot& snap, wire::ParticipantId id) {
    for (const sdk::RobotSample& r : snap.remoteRobots())
        if (r.participantId == id) return &r;
    return nullptr;
}

std::size_t pingReplies(const HostingTestClient& c, std::uint32_t seq) {
    std::size_t n = 0;
    for (const proto::Message& m : c.log)
        if (const auto* control = std::get_if<wire::MatchControl>(&m);
            control != nullptr && control->kind == wire::MatchControlKind::PingReply && control->pingSeq == seq)
            ++n;
    return n;
}

wire::MatchControl pingFrom(const HostingTestClient& c, std::uint32_t seq) {
    wire::MatchControl m;
    m.kind = wire::MatchControlKind::PingRequest;
    m.sessionId = c.accept->sessionId;
    m.runEpoch = c.accept->runEpoch;
    m.pingSeq = seq;
    m.clientTimeUs = 1;
    return m;
}

wire::MatchInputPacket quietInputs(const HostingTestClient& c, std::uint32_t firstSeq) {
    wire::MatchInputPacket packet;
    packet.sessionId = c.accept->sessionId;
    packet.runEpoch = c.accept->runEpoch;
    for (std::uint32_t i = 0; i < wire::kInputRedundancy; ++i) {
        wire::MatchInput input;
        input.inputSeq = firstSeq + i;
        input.wheels.assign(4, wire::WheelInput{0.f, 0});
        packet.inputs.push_back(std::move(input));
    }
    return packet;
}

}  // namespace

TEST_CASE("e2e: practice match with the swerve sample robot through the relay, encrypted") {
    const Clock::time_point t0 = Clock::now();
    const fs::path original = projectUnderTest();
    const fs::path extension = WPIEMU_E2E_EXTENSION;
    const bool verbose = !env("WPIEMU_E2E_VERBOSE").empty();
    auto step = [](const char* what) { std::printf("e2e: %s\n", what); };

    RelayUnderTest relay;
    std::printf("e2e: relay %s (%s)\n", relay.address().c_str(), relay.inProcess() ? "in-process, production configuration" : "external");

    // ---------------------------------------------------------------- phase 1: the probe
    step("phase 1 - relay probe");
    {
        wr::ProbeOptions probe;
        probe.relayHost = relay.host;
        probe.relayPort = relay.port;
        probe.count = 10;
        probe.payloadBytes = 8 * 1024;
        probe.timeout = 15s;
        const wr::ProbeResult result = wr::runProbe(probe);
        INFO(wr::formatProbeText(result));
        CHECK(result.ok);
    }
    const std::uint64_t roomsClosedBefore = relay.stats().roomsClosed;

    // ---------------------------------------------------------------- phase 2: the host starts hosting
    step("phase 2 - host the match through the relay");
    Scratch scratch("wpiemu-match-e2e-");
    const fs::path project = scratch.root / "project";
    copyProject(original, project);
    TestRegistries registries;

    app::host::Options options;
    options.projectDir = project;
    options.extensionLibrary = extension;
    options.scratchDir = scratch.root / "scratch";
    options.handshakeTimeout = 8min;
    options.extraEnv[sdk::env_names::kLogLevel] = "info";
    options.mode = sdk::SessionMode::Run;
    options.timeScale = 1.0;
    options.hosting.mode = app::host::HostingMode::Relay;
    options.hosting.relayEndpoint = relay.address();
    sdk::proto::DsState ds;
    ds.enabled = false;   // the match is joined first and enabled afterwards
    ds.mode = sdk::proto::RobotMode::Teleop;
    ds.station = sdk::proto::AllianceStation::Blue1;
    ds.joysticks[0].present = true;
    ds.joysticks[0].axes = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    ds.joysticks[0].povs = {-1};
    ds.joysticks[0].buttonCount = 12;
    options.driverStation = ds;

    std::vector<std::string> console;
    auto started = app::host::RobotSession::start(registries.view(), swerveSampleMappedConfig(), options);
    REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
    app::host::RobotSession& session = *started.value();
    REQUIRE_FALSE(session.coreOnly());
    const std::string pluginId = session.gameModule()->metadata().id;

    sdk::SessionStatus status;
    const bool programUp = sdk::testing::waitUntil(
        [&] {
            drainConsole(session, console, verbose);
            status = session.status();
            return status.programStarted || !status.active();
        },
        480s, 100ms);
    if (!status.programStarted) {
        dumpConsole(console);
        FAIL("the robot program did not start: " << status.detail);
    }
    REQUIRE(programUp);
    REQUIRE(sdk::testing::waitUntil([&] { return session.hostingStatus()->accepting(); }, 20s, 50ms));
    const auto hosted = session.hostingStatus();
    CHECK(hosted->mode == app::host::HostingMode::Relay);
    REQUIRE_FALSE(hosted->invite.empty());
    auto parsedInvite = net::parseInvite(hosted->invite);
    REQUIRE(parsedInvite.ok());
    const net::InviteSecret invite = parsedInvite.value();
    const std::string room = net::roomId(invite);
    const std::string endpoint = relay.endpoint(room);
    CHECK(hosted->endpoint == endpoint);
    if (relay.inProcess()) CHECK(relay.stats().rooms == 1);
    std::int64_t robotPid = 0;
    if (const auto hello = session.runner().robotHello()) robotPid = hello->pid;
    CHECK(robotPid > 0);

    Players players;
    Player alice(players, "Alice", pluginId);
    REQUIRE_MESSAGE(alice.join(endpoint, invite), alice.why());
    const wire::ParticipantId aliceId = alice.id();
    CHECK(aliceId != wire::kHostParticipantId);
    REQUIRE(sdk::testing::waitUntil([&] { return session.hostingStatus()->roster.size() == 2; }, 5s, 20ms));
    CHECK(session.hostingStatus()->roster[1].name == "Alice");
    CHECK(session.hostingStatus()->timeScaleLocked);

    // Both robots are in each other's snapshots, at the places the other side says they are.
    REQUIRE(players.until([&] { return alice.hostPosition().has_value() && alice.view()->hasOwn; }, 10s));
    std::shared_ptr<const sdk::ISimSnapshot> snap;
    REQUIRE(sdk::testing::waitUntil(
        [&] {
            snap = session.latestSnapshot();
            return snap && remoteIn(*snap, aliceId) != nullptr;
        },
        10s, 20ms, [&] { players.until([] { return true; }, 0ms); }));
    const sdk::RobotSample* aliceOnHost = remoteIn(*snap, aliceId);
    REQUIRE(aliceOnHost != nullptr);
    CHECK(aliceOnHost->name == "Alice");
    CHECK(aliceOnHost->wheels.size() == 4);
    players.forAtLeast(1500ms);   // the interpolation buffer fills
    snap = session.latestSnapshot();
    REQUIRE(snap != nullptr);
    {
        const auto seenByAlice = alice.hostPosition();
        REQUIRE(seenByAlice.has_value());
        CHECK(planar(*seenByAlice, snap->chassis().pose.position) < 0.5);
        const sdk::RobotSample* mine = remoteIn(*snap, aliceId);
        REQUIRE(mine != nullptr);
        CHECK(planar(alice.position(), mine->body.pose.position) < 0.5);
        std::printf("e2e:   host robot (%.2f, %.2f) as Alice sees it (%.2f, %.2f); Alice (%.2f, %.2f) on the host (%.2f, %.2f)\n",
                    snap->chassis().pose.position.x, snap->chassis().pose.position.y, seenByAlice->x, seenByAlice->y, alice.position().x,
                    alice.position().y, mine->body.pose.position.x, mine->body.pose.position.y);
    }

    // ---------------------------------------------------------------- phase 3: both robots drive and collide
    step("phase 3 - the host robot and Alice drive; Alice runs into the host robot");
    const sdk::Vec3 hostStart = snap->chassis().pose.position;
    const sdk::Vec3 aliceStart = alice.position();
    ds.enabled = true;
    ds.joysticks[0].axes = {0.f, -0.35f, 0.f, 0.f, 0.f, 0.f};
    session.controller().setDriverStation(ds);

    double closest = 1e9;
    double aliceTravel = 0.0;
    bool contact = false;
    Clock::time_point contactAt{};
    const bool collided = players.until(
        [&] {
            const auto host = alice.hostPosition();
            if (!host || !alice.view()->own.enabled) return false;
            const double dx = double(host->x) - double(alice.position().x);
            const double dy = double(host->y) - double(alice.position().y);
            const double d = std::max(std::hypot(dx, dy), 1e-3);
            const double speed = 2.5;
            (void)alice.client().driveFieldRelative({static_cast<float>(speed * dx / d), static_cast<float>(speed * dy / d), 0.f});
            const auto truth = session.latestSnapshot();
            if (truth) {
                if (const sdk::RobotSample* mine = remoteIn(*truth, aliceId)) {
                    const double apart = planar(truth->chassis().pose.position, mine->body.pose.position);
                    closest = std::min(closest, apart);
                    aliceTravel = std::max(aliceTravel, planar(mine->body.pose.position, aliceStart));
                    if (!contact && apart < 1.0) {
                        contact = true;
                        contactAt = Clock::now();
                    }
                }
            }
            return contact && Clock::now() - contactAt > 3s;   // keep pushing for a while
        },
        60s);
    snap = session.latestSnapshot();
    REQUIRE(snap != nullptr);
    std::printf("e2e:   host moved %.2f m, Alice travelled %.2f m, closest centre distance %.2f m\n",
                planar(snap->chassis().pose.position, hostStart), aliceTravel, closest);
    CHECK(collided);
    CHECK(planar(snap->chassis().pose.position, hostStart) > 0.3);   // the WPILib robot drove (its own teleop command)
    CHECK(aliceTravel > 3.0);                                         // the remote robot drove on its client's setpoints
    CHECK(closest > 0.6);   // two 0.78 m chassis cannot overlap: the remote chassis collides with the host's robot
    CHECK(closest < 1.0);
    // Alice and the host agree where Alice is.
    if (const sdk::RobotSample* mine = remoteIn(*snap, aliceId)) CHECK(planar(alice.position(), mine->body.pose.position) < 1.0);
    // Stop both robots (host stick to zero, client setpoints to zero).
    (void)alice.client().driveRobotRelative({});
    ds.joysticks[0].axes = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    session.controller().setDriverStation(ds);
    players.forAtLeast(500ms);

    // ---------------------------------------------------------------- phase 4: a player on an impaired network
    step("phase 4 - Bob joins over an impaired network (ImpairedTransport uplink, delayed downlink)");
    {
        Player bob(players, "Bob", pluginId);
        REQUIRE_MESSAGE(bob.join(endpoint, invite, impairedNetwork(7)), bob.why());
        const wire::ParticipantId bobId = bob.id();
        REQUIRE(players.until([&] { return bob.view()->hasOwn && bob.view()->diagnostics.snapshotsReceived > 10; }, 10s));
        CHECK(session.hostingStatus()->roster.size() == 3);
        const auto before = session.latestSnapshot();
        const sdk::RobotSample* start = before ? remoteIn(*before, bobId) : nullptr;
        REQUIRE(start != nullptr);
        const sdk::Vec3 bobStart = start->body.pose.position;
        (void)bob.client().driveRobotRelative({1.5f, 0.f, 0.f});
        players.forAtLeast(4s);
        const auto after = session.latestSnapshot();
        const sdk::RobotSample* end = after ? remoteIn(*after, bobId) : nullptr;
        REQUIRE(end != nullptr);
        const double moved = planar(end->body.pose.position, bobStart);
        std::printf("e2e:   Bob moved %.2f m in 4 s over the impaired link; rtt %.0f ms, loss %.0f permille, %llu snapshots\n", moved,
                    bob.view()->diagnostics.rttMs, bob.view()->diagnostics.lossPermille,
                    static_cast<unsigned long long>(bob.view()->diagnostics.snapshotsReceived));
        CHECK(moved > 0.5);
        CHECK(bob.status().state == mc::ConnectionState::Live);
        CHECK(bob.view()->diagnostics.rttMs > 40.0);   // the delay is really on the path
        (void)bob.client().driveRobotRelative({});
        bob.client().leave("done");
        players.forAtLeast(300ms);
        CHECK(sdk::testing::waitUntil([&] { return session.hostingStatus()->roster.size() == 2; }, 5s, 20ms, [&] { players.until([] { return true; }, 0ms); }));
    }

    // ---------------------------------------------------------------- phase 5: attacks on the running match
    step("phase 5 - attacks: wrong invite, plaintext, tampering, replay, oversize frames, flooding");
    const auto simTimeNow = [&] {
        const auto s = session.latestSnapshot();
        return s ? s->simTimeUs() : 0;
    };
    const auto hostStillRunning = [&](const char* after) {
        const std::int64_t before = simTimeNow();
        players.forAtLeast(600ms);
        INFO("after " << after);
        CHECK(simTimeNow() > before);
        CHECK(session.hostingStatus()->accepting());
        CHECK(session.status().active());
        CHECK(alice.joined());
    };
    const std::uint64_t joinsBefore = session.hostingStatus()->joinsAccepted;

    {
        // wrong invite: the right room, the wrong secret
        const std::uint64_t failedBefore = session.hostingStatus()->handshakesFailed;
        auto stranger = net::generateInvite();
        REQUIRE(stranger.ok());
        std::string error;
        auto intruder = HostingTestClient::connect(endpoint, stranger.value(), &error);
        CHECK(intruder == nullptr);
        CHECK_FALSE(error.empty());
        CHECK(sdk::testing::waitUntil([&] { return session.hostingStatus()->handshakesFailed > failedBefore; }, 5s, 20ms,
                                      [&] { players.until([] { return true; }, 0ms); }));
        // an invite of another room never reaches this host at all
        std::string unknown;
        CHECK(HostingTestClient::connect(relay.endpoint(net::roomId(stranger.value())), stranger.value(), &unknown) == nullptr);
        CHECK_FALSE(unknown.empty());
        hostStillRunning("a wrong invite");
    }
    {
        // a peer that skips the handshake and speaks the game protocol in the clear
        const std::uint64_t failedBefore = session.hostingStatus()->handshakesFailed;
        std::string error;
        auto plain = HostingTestClient::connectPlain(endpoint, &error);
        REQUIRE_MESSAGE(plain != nullptr, error);
        Player fake(players, "Plain", pluginId);
        const bool joined = plain->join("Plain", fake.specJson(), pluginId, session.gameModule()->metadata().year);
        CHECK_FALSE(joined);
        CHECK(sdk::testing::waitUntil([&] { return session.hostingStatus()->handshakesFailed > failedBefore; }, 8s, 20ms,
                                      [&] { players.until([] { return true; }, 0ms); }));
        CHECK(session.hostingStatus()->roster.size() == 2);
        hostStillRunning("a plaintext peer");
    }
    {
        // tampered and replayed records of a joined peer are dropped; the peer and the match carry on
        auto controls = std::make_shared<AttackProxy::Controls>();
        std::string error;
        auto attacker = HostingTestClient::connect(
            endpoint, invite, &error, [controls](std::unique_ptr<proto::ITransport> raw) -> std::unique_ptr<proto::ITransport> {
                return std::make_unique<AttackProxy>(std::move(raw), controls);
            });
        REQUIRE_MESSAGE(attacker != nullptr, error);
        Player maker(players, "Mallory", pluginId);
        REQUIRE(attacker->join("Mallory", maker.specJson(), pluginId, session.gameModule()->metadata().year));
        REQUIRE(sdk::testing::waitUntil([&] { return session.hostingStatus()->roster.size() == 3; }, 5s, 20ms,
                                        [&] { players.until([] { return true; }, 0ms); }));

        controls->flip = 1;
        REQUIRE(attacker->send(pingFrom(*attacker, 1)));
        attacker->pump(700ms);
        CHECK(controls->flipped.load() == 1);
        CHECK(pingReplies(*attacker, 1) == 0);   // the tampered record never reached the match

        REQUIRE(attacker->send(pingFrom(*attacker, 2)));
        CHECK(attacker->waitFor([&] { return pingReplies(*attacker, 2) == 1; }, 3s));   // the next honest record is answered

        controls->duplicate = 1;
        REQUIRE(attacker->send(pingFrom(*attacker, 3)));
        attacker->pump(700ms);
        CHECK(controls->duplicated.load() == 1);
        CHECK(pingReplies(*attacker, 3) == 1);   // the replayed copy was dropped: exactly one answer

        controls->flip = 20;   // a run of damaged input packets
        for (std::uint32_t i = 0; i < 20; ++i) attacker->send(quietInputs(*attacker, 1 + i * 3));
        attacker->pump(300ms);
        CHECK(controls->flipped.load() == 21);
        CHECK(attacker->closed == false);
        REQUIRE(attacker->send(pingFrom(*attacker, 4)));
        CHECK(attacker->waitFor([&] { return pingReplies(*attacker, 4) == 1; }, 3s));
        CHECK(session.hostingStatus()->roster.size() == 3);   // still a participant: dropping is not punishing
        hostStillRunning("tampered and replayed records");

        // oversize frames: above the transport limit the sender refuses; above the spec limit the host refuses the join
        std::string error2;
        auto oversize = HostingTestClient::connect(endpoint, invite, &error2);
        REQUIRE_MESSAGE(oversize != nullptr, error2);
        wire::Join huge;
        huge.displayName = "Huge";
        huge.gamePluginId = pluginId;
        huge.wpilibYear = session.gameModule()->metadata().year;
        huge.robotSpecJson.assign(200 * 1024, 'x');
        CHECK_FALSE(oversize->send(huge));
        huge.robotSpecJson.assign(wire::kMaxRobotSpecBytes + 1024, 'x');
        (void)oversize->send(huge);
        oversize->pump(1500ms);
        CHECK_FALSE(oversize->accept.has_value());
        CHECK(session.hostingStatus()->roster.size() == 3);   // only Mallory besides Alice and the host
        hostStillRunning("oversize frames");
        oversize.reset();

        // flooding: Mallory floods the relay with input packets while Alice keeps playing
        const std::uint64_t alicePacketsBefore = alice.view()->diagnostics.snapshotsReceived;
        const std::int64_t simBefore = simTimeNow();
        std::atomic<bool> flooding{true};
        std::thread flood([&] {
            for (std::uint32_t i = 0; i < 4000 && flooding.load(); ++i) (void)attacker->send(quietInputs(*attacker, 100 + i * 3));
            flooding = false;
        });
        players.until([&] { return !flooding.load(); }, 30s);
        flood.join();
        players.forAtLeast(1500ms);
        if (relay.inProcess()) {
            CHECK(relay.stats().dropped(wr::DropReason::RateLimited) > 0);
            std::printf("e2e:   relay dropped %llu frames for the rate limit, kicked %llu peers\n",
                        static_cast<unsigned long long>(relay.stats().dropped(wr::DropReason::RateLimited)),
                        static_cast<unsigned long long>(relay.stats().peersKicked));
        }
        CHECK(alice.view()->diagnostics.snapshotsReceived > alicePacketsBefore + 30);   // Alice was served throughout
        CHECK(simTimeNow() > simBefore + 1'000'000);                                      // and the host's world kept running
        CHECK(alice.status().state == mc::ConnectionState::Live);
        hostStillRunning("a flood");
        attacker.reset();
    }
    // The two honest participants were never disturbed and nothing was joined by the attackers beyond Mallory.
    CHECK(session.hostingStatus()->joinsAccepted <= joinsBefore + 1);

    // ---------------------------------------------------------------- phase 6: link loss, resume, kick
    step("phase 6 - link loss and resume, kick and a fresh join");
    {
        // Carol joins through a proxy so the test can cut her link.
        auto controls = std::make_shared<AttackProxy::Controls>();
        Wrap viaProxy = [controls](std::unique_ptr<proto::ITransport> raw) -> std::unique_ptr<proto::ITransport> {
            return std::make_unique<AttackProxy>(std::move(raw), controls);
        };
        Player carol(players, "Carol", pluginId);
        REQUIRE_MESSAGE(carol.join(endpoint, invite, viaProxy), carol.why());
        const wire::ParticipantId carolId = carol.id();
        (void)carol.client().driveRobotRelative({1.0f, 0.f, 0.f});
        players.forAtLeast(1500ms);
        (void)carol.client().driveRobotRelative({});
        players.forAtLeast(800ms);
        const auto before = session.latestSnapshot();
        REQUIRE(before != nullptr);
        const sdk::RobotSample* carolBefore = remoteIn(*before, carolId);
        REQUIRE(carolBefore != nullptr);
        const sdk::Vec3 placed = carolBefore->body.pose.position;

        controls->cut = true;
        CHECK(players.until([&] { return !carol.joined(); }, 10s));
        // the host keeps her robot, shown as not connected
        CHECK(sdk::testing::waitUntil(
            [&] {
                for (const auto& p : session.hostingStatus()->roster)
                    if (p.id == carolId && !p.connected) return true;
                return false;
            },
            15s, 50ms, [&] { players.until([] { return true; }, 0ms); }));
        const auto kept = session.latestSnapshot();
        REQUIRE(kept != nullptr);
        CHECK(remoteIn(*kept, carolId) != nullptr);

        Player& same = carol;
        controls->cut = false;
        REQUIRE_MESSAGE(same.resume(), same.why());
        CHECK(same.id() == carolId);   // the same participant, not a new one
        players.forAtLeast(600ms);
        const auto resumed = session.latestSnapshot();
        REQUIRE(resumed != nullptr);
        const sdk::RobotSample* carolAfter = remoteIn(*resumed, carolId);
        REQUIRE(carolAfter != nullptr);
        CHECK(planar(carolAfter->body.pose.position, placed) < 0.5);   // her robot waited where it was
        for (const auto& p : session.hostingStatus()->roster)
            if (p.id == carolId) CHECK(p.connected);

        // kick
        const std::uint64_t kicksBefore = session.hostingStatus()->kicks;
        session.kickParticipant(carolId, "e2e kick");
        CHECK(players.until([&] { return carol.status().state == mc::ConnectionState::Disconnected; }, 10s));
        CHECK(carol.status().reason == mc::DisconnectReason::Kicked);
        CHECK(sdk::testing::waitUntil([&] { return session.hostingStatus()->roster.size() == 2; }, 5s, 20ms,
                                      [&] { players.until([] { return true; }, 0ms); }));
        CHECK(session.hostingStatus()->kicks == kicksBefore + 1);
        CHECK(sdk::testing::waitUntil(
            [&] {
                const auto s = session.latestSnapshot();
                return s && remoteIn(*s, carolId) == nullptr;
            },
            5s, 20ms, [&] { players.until([] { return true; }, 0ms); }));
        // the old resume token is dead ...
        CHECK_FALSE(carol.resume());
        // ... and a fresh join is welcome
        Player carolAgain(players, "Carol2", pluginId);
        REQUIRE_MESSAGE(carolAgain.join(endpoint, invite), carolAgain.why());
        CHECK(carolAgain.id() != carolId);
        carolAgain.client().leave("done");
        players.forAtLeast(300ms);
    }

    // ---------------------------------------------------------------- phase 7: stop frees the room, the robot and the port
    step("phase 7 - stop: the room closes, the clients are told, the robot and the relay port go away");
    CHECK(alice.joined());
    session.requestStop();
    CHECK(players.until([&] { return session.finished(); }, 60s));
    session.waitUntilFinished();
    CHECK(players.until([&] { return alice.status().state == mc::ConnectionState::Disconnected; }, 10s));
    CHECK(alice.status().reason == mc::DisconnectReason::ServerClosing);
    CHECK(session.hostingStatus()->phase == app::host::HostingPhase::Off);
    sdk::testing::waitUntil([&] { return !processAlive(robotPid); }, 10s, 100ms);
    CHECK_FALSE(processAlive(robotPid));
    if (relay.inProcess()) {
        CHECK(relaystack::waitFor([&] { return relay.stats().rooms == 0; }, 5s));
        CHECK(relay.stats().roomsClosed == roomsClosedBefore + 1);   // CloseRoom, not the room's TTL
    }
    // the room is gone: the invite no longer reaches anyone
    std::string gone;
    CHECK(HostingTestClient::connect(endpoint, invite, &gone) == nullptr);
    CHECK_FALSE(gone.empty());
    if (relay.inProcess()) {
        const std::uint16_t relayPort = relay.port;
        CHECK(relay.local->stop());
        CHECK(udpPortIsFree("127.0.0.1", relayPort));   // nothing keeps the relay's port
    }
    std::printf("e2e: practice match total %.1f s\n", std::chrono::duration<double>(Clock::now() - t0).count());
}

int main(int argc, char** argv) {
    auto skip = [](const std::string& why) {
        std::printf("SKIPPED e2e: %s\n", why.c_str());
        return kSkipExitCode;
    };
    if (env("WPIEMU_E2E").empty()) return skip("set WPIEMU_E2E=1 to run the practice match end-to-end test (needs WPILib 2026, Gradle and a robot project)");
    const fs::path project = projectUnderTest();
    std::error_code ec;
    if (project.empty() || !fs::is_directory(project, ec)) return skip("robot project not found: '" + project.string() + "' (set WPIEMU_E2E_PROJECT)");
    const bool windows = fs::path::preferred_separator == '\\';
    if (!fs::exists(project / (windows ? "gradlew.bat" : "gradlew"), ec)) return skip("the project has no Gradle wrapper: " + project.string());
    if (!fs::exists(fs::path(WPIEMU_E2E_EXTENSION), ec)) return skip(std::string("the HALSim extension is not built: ") + WPIEMU_E2E_EXTENSION);
    if (!core::launcher::findWpilibInstall(2026, {}) && env("JAVA_HOME").empty())
        return skip("no WPILib 2026 installation (C:\\Users\\Public\\wpilib\\2026) and no JAVA_HOME");
    doctest::Context context(argc, argv);
    return context.run();
}
