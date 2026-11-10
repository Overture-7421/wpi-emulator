#pragma once

// Fixtures of the match host tests: a fake IMatchHostSession, the real config pipeline kit, an in-memory (impaired)
// client, and a harness that plays the sim thread (calls onPoll / onTickEnd) on a manual wall clock.

#include <doctest/doctest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/match/hud_keys.h"
#include "wpiemu/core/match/match_host.h"
#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/sdk/game/default_field.h"
#include "wpiemu/sdk/match/quantize.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/impaired_transport.h"
#include "wpiemu/sdk/testing/wait.h"

namespace match_test {

using namespace wpiemu;
using namespace wpiemu::sdk;
using namespace wpiemu::core::match;
using namespace std::chrono_literals;
namespace wire = wpiemu::sdk::match;
namespace rt = wpiemu::core::runtime;

// ---- robot specs -----------------------------------------------------------------------------------------------------

inline WheelSpec makeWheel(const char* name, float x, float y) {
    WheelSpec w;
    w.name = name;
    w.kind = WheelKind::Swerve;
    w.positionM = {x, y, 0.f};
    w.steered = true;
    w.driveCurrentLimitA = 60.f;
    return w;
}

inline RobotConfig swerveSpec(const char* name = "Swerve") {
    RobotConfig c;
    c.name = name;
    c.chassis.sizeM = {0.7f, 0.7f, 0.12f};
    c.chassis.massKg = 50.f;
    c.chassis.comHeightM = 0.2f;
    c.chassis.groundClearanceM = 0.03f;
    c.drivetrain.spec.kind = drivetrain_kind::kSwerve;
    c.drivetrain.spec.wheels = {makeWheel("fl", 0.25f, 0.25f), makeWheel("fr", 0.25f, -0.25f), makeWheel("rl", -0.25f, 0.25f),
                                makeWheel("rr", -0.25f, -0.25f)};
    return c;
}

inline std::string specJson(const RobotConfig& config) {
    const core::config::RobotConfigCodec codec;
    return core::config::JsonValueCodec().dump(codec.toValue(config), false);
}

// The real config pipeline over the physics registries (what the app composes).
struct PipelineKit {
    DrivetrainRegistry drivetrains;
    TireModelRegistry tires;
    MechanismFactory mechanisms;
    GamePluginRegistry plugins;
    core::config::RobotConfigCodec codec;
    core::config::ConfigValidator validator;
    core::config::PluginRegistrar registrar;
    core::config::ConfigPipeline pipeline;

    PipelineKit()
        : validator(&tires, {}), registrar(mechanisms), pipeline(codec, validator, registrar, drivetrains, plugins) {
        REQUIRE(core::physics::registerPhysicsComponents(drivetrains, tires).ok());
    }
};

// Counter based and deterministic: NOT random, only for tests.
inline std::function<Status(std::span<std::byte>)> testRandom(std::uint64_t seed = 1) {
    auto state = std::make_shared<std::uint64_t>(seed);
    return [state](std::span<std::byte> out) -> Status {
        for (std::byte& b : out) {
            *state = *state * 6364136223846793005ull + 1442695040888963407ull;
            b = static_cast<std::byte>((*state >> 33) | 1);   // never all zero
        }
        return okStatus();
    };
}

// ---- the fake session ---------------------------------------------------------------------------------------------------

class FakeSession : public IMatchHostSession {
public:
    struct Robot {
        rt::RemoteRobotRequest request;
        rt::RemoteInputStats stats;
        std::vector<wire::MatchInput> inputs;
        Pose3 pose;
        Vec3 velocity;
    };

    std::string plugin = "fake_game";
    int year = 2099;
    std::string host = "Host";
    proto::DsState ds;
    std::int64_t step = 5000;
    std::uint64_t tick = 0;
    std::int64_t simTime = 0;
    std::map<wire::ParticipantId, Robot> robots;
    std::vector<EntityGroupSample> groups;
    std::vector<sdk::HudValue> hud;
    std::vector<sdk::HudValue> robotHud;   // the HUD of every remote robot
    std::vector<HudWidgetDescriptor> descriptors;
    BodySample hostBody;
    std::vector<WheelSample> hostWheels;
    std::optional<Error> addFailure;
    std::vector<wire::ParticipantId> removed;
    int builds = 0;

    FakeSession() { ds.station = proto::AllianceStation::Red1; }

    void advance(int ticks = 1) {
        tick += static_cast<std::uint64_t>(ticks);
        simTime += static_cast<std::int64_t>(ticks) * step;
    }

    std::string gamePluginId() const override { return plugin; }
    int wpilibYear() const override { return year; }
    std::string hostName() const override { return host; }
    std::vector<HudWidgetDescriptor> hudDescriptors() const override { return descriptors; }
    std::int64_t stepUs() const override { return step; }
    std::uint64_t tickIndex() const override { return tick; }
    std::int64_t simTimeUs() const override { return simTime; }
    proto::DsState driverStation() const override { return ds; }

    Status addRemoteRobot(const rt::RemoteRobotRequest& request) override {
        if (addFailure) return *addFailure;
        if (robots.size() >= rt::kMaxRemoteRobots) return makeError(ErrorCode::OutOfRange, "full");
        for (const auto& [id, r] : robots)
            if (r.request.station == request.station || id == request.participantId) return makeError(ErrorCode::AlreadyExists, "taken");
        Robot r;
        r.request = request;
        r.pose.position = {static_cast<float>(request.participantId), 1.f, 0.f};
        robots.emplace(request.participantId, std::move(r));
        return okStatus();
    }
    Status removeRemoteRobot(wire::ParticipantId id) override {
        removed.push_back(id);
        return robots.erase(id) != 0 ? okStatus() : makeError(ErrorCode::NotFound, "unknown");
    }
    Result<rt::RemoteInputOutcome> submitRemoteInput(wire::ParticipantId id, const wire::MatchInput& input) override {
        const auto it = robots.find(id);
        if (it == robots.end()) return makeError(ErrorCode::NotFound, "unknown");
        Robot& r = it->second;
        if (input.inputSeq == wire::kNoInputSeq) return makeError(ErrorCode::InvalidArgument, "sequence 0");
        ++r.stats.received;
        r.inputs.push_back(input);
        r.stats.lastAppliedSeq = input.inputSeq;
        r.stats.ackTick = tick;
        r.stats.inputLeadTicks = static_cast<std::int64_t>(input.targetTick) - static_cast<std::int64_t>(tick);
        return rt::RemoteInputOutcome::Queued;
    }
    std::optional<rt::RemoteRobotInfo> remoteRobotInfo(wire::ParticipantId id) const override {
        const auto it = robots.find(id);
        if (it == robots.end()) return std::nullopt;
        rt::RemoteRobotInfo info;
        info.participantId = id;
        info.name = it->second.request.name;
        info.station = it->second.request.station;
        info.placement = {static_cast<float>(id), 1.f, 0.f};
        info.stats = it->second.stats;
        return info;
    }
    std::shared_ptr<const ISimSnapshot> buildSnapshot(bool paused, bool robotConnected) override {
        ++builds;
        SimSnapshotData d;
        d.tick = tick;
        d.simTimeUs = simTime;
        d.paused = paused;
        d.robotConnected = robotConnected;
        d.chassis = hostBody;
        d.wheels = hostWheels;
        d.driverStation = ds;
        d.entityGroups = groups;
        d.hud = hud;
        for (const auto& [id, r] : robots) {
            RobotSample s;
            s.participantId = id;
            s.name = r.request.name;
            s.station = r.request.station;
            s.hud = robotHud;
            s.body.pose = r.pose;
            s.body.linearVelocity = r.velocity;
            for (std::size_t w = 0; w < r.request.spec.drivetrain.spec.wheels.size(); ++w) {
                WheelSample ws;
                ws.steerAngleRad = 0.1 * static_cast<double>(w);
                ws.spinAngleRad = 0.01 * static_cast<double>(tick);
                s.wheels.push_back(ws);
            }
            d.remoteRobots.push_back(std::move(s));
        }
        return std::make_shared<const SimSnapshot>(std::move(d));
    }
};

// ---- harness: the sim thread ---------------------------------------------------------------------------------------------

struct Harness {
    testing::ManualWallClock clock;
    testing::CapturingLogger log;
    PipelineKit kit;
    std::shared_ptr<ipc::BinaryMessageCodec> codec = std::make_shared<ipc::BinaryMessageCodec>();
    std::unique_ptr<MatchHost> host;
    rt::RunnerObservation obs{wire::MatchState::Running, 1};

    explicit Harness(IMatchHostSession& session, MatchHostConfig config = {}, std::uint64_t seed = 1) {
        MatchHostServices services;
        services.codec = codec;
        services.clock = std::shared_ptr<IWallClock>(std::shared_ptr<void>(), &clock);
        services.logger = &log;
        services.specPipeline = &kit.pipeline;
        services.randomFill = testRandom(seed);
        host = std::make_unique<MatchHost>(std::move(config), session, std::move(services));
    }

    void poll() { host->onPoll(obs); }
    void tickEnd() { host->onTickEnd(obs); }
    template <class Done>
    bool pumpUntil(Done&& done, std::chrono::milliseconds timeout = 3000ms) {
        return testing::waitUntil(std::forward<Done>(done), timeout, 1ms, [this] { poll(); });
    }
    // Everything the host queued has been handed to the transports.
    void flush() { REQUIRE(host->waitOutboundIdle(3000ms)); }
};

// ---- an in-memory client ------------------------------------------------------------------------------------------------

class TestClient {
public:
    using HostEndWrapper = std::function<std::unique_ptr<proto::ITransport>(std::unique_ptr<proto::ITransport>)>;

    // `profile` impairs both directions (unreliable frames only for loss). `wrapHostEnd` decorates the host's end.
    TestClient(Harness& h, const testing::ImpairmentProfile& profile = {}, std::uint64_t seed = 7,
               const HostEndWrapper& wrapHostEnd = {})
        : h_(h) {
        pair_ = testing::makeImpairedPair(h.clock, profile, seed);
        channel_ = std::make_unique<proto::MessageChannel>(std::move(pair_.b), h.codec);
        std::unique_ptr<proto::ITransport> hostEnd = std::move(pair_.a);
        if (wrapHostEnd) hostEnd = wrapHostEnd(std::move(hostEnd));
        h.host->addConnection(std::move(hostEnd));
    }

    Status send(const proto::Message& m) { return channel_->send(m, proto::deliveryOf(m)); }
    Status sendRaw(std::span<const std::byte> bytes) { return channel_->transport().send(bytes); }

    // Receives everything that is ready into the log (does not wait).
    std::size_t drain() {
        std::size_t n = 0;
        proto::Message m;
        for (;;) {
            Result<bool> got = channel_->receive(m, 0ms);
            if (!got) {
                disconnected_ = true;
                break;
            }
            if (!got.value()) break;
            log.push_back(std::move(m));
            ++n;
        }
        return n;
    }

    template <class T>
    std::vector<T> all() const {
        std::vector<T> out;
        for (const proto::Message& m : log)
            if (const T* t = std::get_if<T>(&m)) out.push_back(*t);
        return out;
    }
    template <class T>
    std::optional<T> last() const {
        for (auto it = log.rbegin(); it != log.rend(); ++it)
            if (const T* t = std::get_if<T>(&*it)) return *t;
        return std::nullopt;
    }
    template <class T>
    std::size_t count() const {
        return all<T>().size();
    }

    // Pumps the host until `done()` holds; drains the client in between.
    template <class Done>
    bool waitFor(Done&& done, std::chrono::milliseconds timeout = 3000ms) {
        return testing::waitUntil([&] { drain(); return done(); }, timeout, 1ms, [this] { h_.poll(); });
    }

    wire::Join joinMessage(const std::string& name, const RobotConfig& spec, const FakeSession& s) const {
        wire::Join j;
        j.displayName = name;
        j.gamePluginId = s.plugin;
        j.wpilibYear = s.year;
        j.robotSpecJson = specJson(spec);
        return j;
    }

    bool disconnected() {
        drain();
        return disconnected_;
    }
    void close() { channel_->close(); }
    proto::ITransport& transport() { return channel_->transport(); }

    std::vector<proto::Message> log;
    std::optional<wire::JoinAccept> accept;
    std::uint32_t inputSeq = 0;

    // Joins and waits for the JoinAccept (or a JoinReject).
    bool join(const wire::Join& j) {
        REQUIRE(send(j).ok());
        const bool got = waitFor([&] { return count<wire::JoinAccept>() + count<wire::JoinReject>() > 0; });
        accept = last<wire::JoinAccept>();
        return got && accept.has_value();
    }

    wire::MatchInputPacket packetFor(std::uint32_t firstSeq, std::size_t count, std::uint64_t targetTick, std::size_t wheels = 4,
                                     float speed = 1.f) const {
        wire::MatchInputPacket p;
        p.sessionId = accept ? accept->sessionId : 0;
        p.runEpoch = accept ? accept->runEpoch : 0;
        for (std::size_t i = 0; i < count; ++i) {
            wire::MatchInput in;
            in.inputSeq = firstSeq + static_cast<std::uint32_t>(i);
            in.targetTick = targetTick;
            in.wheels.assign(wheels, wire::WheelInput{speed, 0});
            p.inputs.push_back(std::move(in));
        }
        return p;
    }

private:
    Harness& h_;
    testing::ImpairedTransportPair pair_;
    std::unique_ptr<proto::MessageChannel> channel_;
    bool disconnected_ = false;
};

// A transport decorator for the host's end of a connection: reports configurable link stats, and can block its sends like a
// stuck socket.
class ScriptedTransport final : public proto::ITransport {
public:
    explicit ScriptedTransport(std::unique_ptr<proto::ITransport> inner) : inner_(std::move(inner)) {}

    Status send(std::span<const std::byte> frame) override { return forward(frame, false); }
    Status sendUnreliable(std::span<const std::byte> frame) override { return forward(frame, true); }
    Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override { return inner_->receive(frame, timeout); }
    void close() override { inner_->close(); }
    bool connected() const override { return inner_->connected(); }
    std::string describe() const override { return "scripted"; }
    std::optional<proto::LinkStats> linkStats() const override {
        if (!hasStats.load()) return std::nullopt;
        proto::LinkStats s;
        s.rttMs = rttMs.load();
        s.lossPermille = lossPermille.load();
        return s;
    }

    std::atomic<bool> hasStats{false};
    std::atomic<std::uint32_t> rttMs{5};
    std::atomic<std::uint32_t> lossPermille{0};
    std::atomic<bool> blockSends{false};
    std::atomic<int> sends{0};
    std::atomic<int> unreliableSends{0};

private:
    Status forward(std::span<const std::byte> frame, bool unreliable) {
        while (blockSends.load() && inner_->connected()) std::this_thread::sleep_for(1ms);
        ++sends;
        if (unreliable) ++unreliableSends;
        return unreliable ? inner_->sendUnreliable(frame) : inner_->send(frame);
    }
    std::unique_ptr<proto::ITransport> inner_;
};

// A client whose host end is a ScriptedTransport (owned by the host connection; the pointer stays valid while it lives).
struct ScriptedClient {
    ScriptedTransport* transport = nullptr;
    std::unique_ptr<TestClient> client;
    ScriptedClient(Harness& h, const testing::ImpairmentProfile& profile = {}, std::uint64_t seed = 7) {
        client = std::make_unique<TestClient>(h, profile, seed, [this](std::unique_ptr<proto::ITransport> inner) {
            auto t = std::make_unique<ScriptedTransport>(std::move(inner));
            transport = t.get();
            return std::unique_ptr<proto::ITransport>(std::move(t));
        });
    }
};

}  // namespace match_test
