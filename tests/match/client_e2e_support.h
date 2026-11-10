#pragma once

// The end-to-end world of the game-client tests: the REAL MatchHost over the REAL SimulationSession (Box3D, the 2026 field and
// game state) with real MatchClients over impaired in-memory links, all on one manual clock. A step is 5 ms: the clock moves,
// frames that became due are released, the host polls, ticks and sends, and every client updates. Nothing sleeps; the host's
// reader and sender threads are real, so the world waits for them at the two points where order matters.

#include "client_test_support.h"

#include <chrono>
#include <deque>
#include <thread>

#include "wpiemu/plugins/game_2026/plugin.h"

namespace client_test {

namespace g26 = wpiemu::plugins::game_2026;

// ---- statistics -------------------------------------------------------------------------------------------------------

struct Stat {
    std::size_t n = 0;
    double p50 = 0, p95 = 0, p99 = 0, max = 0, mean = 0;
};

inline Stat statOf(std::vector<double> v) {
    Stat s;
    s.n = v.size();
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    const auto at = [&](double q) { return v[static_cast<std::size_t>(std::ceil(q * static_cast<double>(v.size()))) - 1]; };
    s.p50 = at(0.5);
    s.p95 = at(0.95);
    s.p99 = at(0.99);
    s.max = v.back();
    double sum = 0;
    for (const double x : v) sum += x;
    s.mean = sum / static_cast<double>(v.size());
    return s;
}

// ---- network profiles -------------------------------------------------------------------------------------------------

struct NetProfile {
    std::string name;
    testing::ImpairmentProfile up;     // client -> host
    testing::ImpairmentProfile down;   // host -> client
    std::uint32_t rttMs = 2;           // what the host's transport reports (it drives the snapshot rate)
    std::uint32_t lossPermille = 0;
};

inline testing::ImpairmentProfile impairment(int latencyUs, int jitterUs, double loss = 0.0, double reorder = 0.0) {
    testing::ImpairmentProfile p;
    p.latency = std::chrono::microseconds(latencyUs);
    p.jitter = std::chrono::microseconds(jitterUs);
    p.lossProbability = loss;
    p.reorderProbability = reorder;
    p.reorderDelay = std::chrono::microseconds(40'000);
    return p;
}

inline NetProfile lanProfile() {
    NetProfile n;
    n.name = "LAN 60 Hz";
    n.up = n.down = impairment(1000, 500);
    n.rttMs = 2;
    return n;
}

// One-way 20 ms plus 0..20 ms of jitter: 60 ms round trip on average, 3 % loss, snapshots at 30 Hz.
inline NetProfile wanProfile() {
    NetProfile n;
    n.name = "WAN 30 Hz 60 ms RTT 20 ms jitter 3% loss";
    n.up = n.down = impairment(20'000, 20'000, 0.03);
    n.rttMs = 60;
    n.lossPermille = 30;
    return n;
}

// One-way 60 ms plus 0..30 ms: 150 ms round trip, 8 % loss and reordering.
inline NetProfile badWanProfile() {
    NetProfile n;
    n.name = "bad WAN 150 ms RTT 8% loss reordering";
    n.up = n.down = impairment(60'000, 30'000, 0.08, 0.05);
    n.rttMs = 150;
    n.lossPermille = 80;
    return n;
}

inline NetProfile asymmetricProfile(int upUs, int downUs) {
    NetProfile n;
    n.name = "asymmetric " + std::to_string(upUs / 1000) + "/" + std::to_string(downUs / 1000) + " ms";
    n.up = impairment(upUs, 2000);
    n.down = impairment(downUs, 2000);
    n.rttMs = static_cast<std::uint32_t>((upUs + downUs) / 1000);
    return n;
}

// ---- the world --------------------------------------------------------------------------------------------------------

struct TruthPose {
    double x = 0, y = 0, yaw = 0;
    bool valid = false;
};

class E2eWorld;

class E2eClient {
public:
    E2eClient(E2eWorld& world, std::string name, const NetProfile& net, std::uint64_t seed);

    // Opens a link (one more every time the client reconnects).
    Result<std::unique_ptr<proto::ITransport>> makeLink();
    Status join(RobotConfig spec);
    void cutLink() {
        if (probe) probe->cutNow = true;
    }
    // Releases the frames that are due on this client's link and waits until the host's reader has taken them.
    void release();

    wire::ParticipantId id() const { return client->view() ? client->view()->own.id : wire::kNoParticipant; }
    mc::ConnectionState state() const { return client->status().state; }

    E2eWorld& world;
    std::string name;
    NetProfile net;
    std::uint64_t seed;
    ClientKit kit;
    std::shared_ptr<CutTransport::Probe> probe;
    std::shared_ptr<std::atomic<std::uint64_t>> hostReceived;
    std::vector<double> updateUs;   // wall-clock cost of every MatchClient::update()
    std::unique_ptr<mc::MatchClient> client;
    std::size_t links = 0;
};

class E2eWorld {
public:
    struct Options {
        int neutralFuel = 0;
        int depotFuel = 0;
        int subSteps = 1;               // client updates per 5 ms step
        std::size_t maxPlayers = 6;
    };

    explicit E2eWorld(Options options = {})
        : options_(options), rig(makeRigOptions(options)), adapter(*rig.session), h(adapter, MatchHostConfig{}) {
        REQUIRE(g26::registerPlugin(h.kit.plugins).ok());
    }

    static g26session::Options makeRigOptions(const Options& o) {
        g26session::Options r;
        r.neutralFuel = o.neutralFuel;
        r.depotFuel = o.depotFuel;
        return r;
    }

    // A swerve robot of the 2026 game with the given zones (none by default).
    static RobotConfig robotSpec(const char* name, bool fuelZones = false) {
        RobotConfig spec = fuelZones ? g26session::Rig::fuelRobot() : swerveSpec(name);
        spec.name = name;
        spec.gamePluginId = std::string(g26::pluginId());
        return spec;
    }

    E2eClient& addClient(const std::string& name, const NetProfile& net, RobotConfig spec) {
        clients.push_back(std::make_unique<E2eClient>(*this, name, net, 100 + clients.size()));
        E2eClient& c = *clients.back();
        const Status s = c.join(std::move(spec));
        REQUIRE_MESSAGE(s.ok(), (s.ok() ? "" : s.error().message));
        for (int i = 0; i < 2000 && c.state() != mc::ConnectionState::Lobby && c.state() != mc::ConnectionState::Live; ++i) step();
        REQUIRE_MESSAGE((c.state() == mc::ConnectionState::Lobby || c.state() == mc::ConnectionState::Live), c.client->status().detail);
        return c;
    }

    bool hostRunning() const { return h.obs.matchState == wire::MatchState::Running; }

    // One 5 ms step.
    void step() {
        const int sub = std::max(1, options_.subSteps);
        for (int i = 0; i < sub; ++i) {
            h.clock.advance(std::chrono::microseconds(5000 / sub));
            for (const auto& c : clients) c->release();
            h.poll();
            if (i == sub - 1 && hostRunning()) {
                rig.session->tick(rig.frame, false);
                recordTruth();
                h.tickEnd();
            }
            REQUIRE(h.host->waitOutboundIdle(3s));
            for (const auto& c : clients) {
                const auto t0 = std::chrono::steady_clock::now();
                c->client->update();
                c->updateUs.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
            }
            if (onSubStep) onSubStep(*this);
        }
        ++steps;
    }
    void run(double seconds) {
        const int n = static_cast<int>(std::lround(seconds / 0.005));
        for (int i = 0; i < n; ++i) step();
    }

    std::chrono::steady_clock::time_point now() const { return h.clock.now(); }
    double seconds() const { return static_cast<double>(steps) * 0.005; }

    // The host's robot `id` after `tick` completed ticks (the last 1200 ticks are kept).
    TruthPose truthAt(std::uint64_t tick, wire::ParticipantId id) const {
        for (auto it = truth_.rbegin(); it != truth_.rend(); ++it)
            if (it->first == tick) return id < it->second.size() ? it->second[id] : TruthPose{};
        return {};
    }
    std::uint64_t hostTick() const { return rig.session->clock().tickIndex(); }

    Options options_;
    g26session::Rig rig;
    core::match::SimulationMatchSession adapter;
    Harness h;
    std::vector<std::unique_ptr<E2eClient>> clients;
    std::function<void(E2eWorld&)> onSubStep;
    std::uint64_t steps = 0;

private:
    void recordTruth() {
        std::vector<TruthPose> poses(wire::kMaxRobotsPerMatch);
        for (wire::ParticipantId id = 0; id < poses.size(); ++id) {
            const RobotView* view = rig.session->robots().find(id);
            if (view == nullptr) continue;
            const Pose3 p = rig.session->world().pose(view->body);
            poses[id] = {p.position.x, p.position.y, p.rotation.yaw(), true};
        }
        truth_.emplace_back(rig.session->clock().tickIndex(), std::move(poses));
        while (truth_.size() > 1200) truth_.pop_front();
    }

    std::deque<std::pair<std::uint64_t, std::vector<TruthPose>>> truth_;
};

inline E2eClient::E2eClient(E2eWorld& w, std::string n, const NetProfile& profile, std::uint64_t s)
    : world(w), name(std::move(n)), net(profile), seed(s), kit(w.h.clock) {
    REQUIRE(g26::registerPlugin(kit.kit.plugins).ok());
    kit.descriptors = w.adapter.hudDescriptors();
    mc::MatchClientConfig config;
    config.displayName = name;
    mc::MatchClientServices services = kit.services();
    services.reconnect = [this] { return makeLink(); };
    client = std::make_unique<mc::MatchClient>(config, std::move(services));
}

inline Result<std::unique_ptr<proto::ITransport>> E2eClient::makeLink() {
    // `a` is the host's end: what it sends is the profile's downstream.
    testing::ImpairedTransportPair pair = testing::makeImpairedPair(world.h.clock, net.down, net.up, seed + 1000 * (++links));
    auto scripted = std::make_unique<ScriptedTransport>(std::move(pair.a));
    scripted->hasStats = true;
    scripted->rttMs = net.rttMs;
    scripted->lossPermille = net.lossPermille;
    hostReceived = std::make_shared<std::atomic<std::uint64_t>>(0);
    probe = std::make_shared<CutTransport::Probe>();
    world.h.host->addConnection(std::make_unique<CountingTransport>(std::move(scripted), hostReceived));
    return std::unique_ptr<proto::ITransport>(new CutTransport(std::move(pair.b), probe));
}

inline Status E2eClient::join(RobotConfig spec) {
    Result<std::unique_ptr<proto::ITransport>> link = makeLink();
    if (!link) return link.error();
    return client->connect(std::move(link).value(), spec);
}

inline void E2eClient::release() {
    testing::ImpairedTransport* end = probe ? probe->end.load() : nullptr;
    if (end == nullptr) return;
    end->pump();
    const std::uint64_t want = end->stats().delivered;
    // The host's reader thread takes the frames that were just handed to its end; wait for it so every run orders the same.
    for (int spin = 0; spin < 20000 && hostReceived->load() < want; ++spin) std::this_thread::yield();
}

}  // namespace client_test
