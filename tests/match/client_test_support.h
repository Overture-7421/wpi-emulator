#pragma once

// Fixtures of the game-client tests: a scripted host that speaks the match protocol directly (for the client's join, state and
// failure handling), a kit of everything a MatchClient is built from, and the end-to-end world (the REAL MatchHost over the
// REAL SimulationSession with the 2026 plugin and clients over impaired in-memory links).

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "match_test_support.h"
#include "../game_2026/multirobot_session_rig.h"
#include "wpiemu/core/match/match_client.h"
#include "wpiemu/sdk/core/environment.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

namespace client_test {

using namespace match_test;
namespace mc = wpiemu::core::match;

inline bool envFlag(const char* name) {
    const std::optional<std::string> v = sdk::readEnvironment(name);
    return v.has_value() && !v->empty() && (*v)[0] == '1';
}
inline bool reportRequested() { return envFlag("WPIEMU_STRESS_REPORT"); }
inline bool fullRunRequested() { return envFlag("WPIEMU_STRESS"); }

// ---- transports -------------------------------------------------------------------------------------------------------

// Hands the client's end of a link to the client and can cut it: the end closes (the peer sees the loss) and every call fails.
class CutTransport final : public proto::ITransport {
public:
    struct Probe {
        std::atomic<testing::ImpairedTransport*> end{nullptr};
        std::atomic<bool> cutNow{false};
    };

    CutTransport(std::unique_ptr<testing::ImpairedTransport> inner, std::shared_ptr<Probe> probe)
        : inner_(std::move(inner)), probe_(std::move(probe)) {
        probe_->end = inner_.get();
    }
    ~CutTransport() override { probe_->end = nullptr; }

    Status send(std::span<const std::byte> frame) override { return gate() ? inner_->send(frame) : lost(); }
    Status sendUnreliable(std::span<const std::byte> frame) override { return gate() ? inner_->sendUnreliable(frame) : lost(); }
    Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        if (!gate()) return lost().error();
        return inner_->receive(frame, timeout);
    }
    void close() override { inner_->close(); }
    bool connected() const override { return !probe_->cutNow && inner_->connected(); }
    std::string describe() const override { return "cut(" + inner_->describe() + ")"; }

private:
    bool gate() {
        if (!probe_->cutNow) return true;
        inner_->close();
        return false;
    }
    static Status lost() { return makeError(ErrorCode::Disconnected, "the link was cut"); }

    std::unique_ptr<testing::ImpairedTransport> inner_;
    std::shared_ptr<Probe> probe_;
};

// ---- what a client is built from --------------------------------------------------------------------------------------

// Counts the frames a reader thread took from a link (the end-to-end world waits on it so every run orders alike).
class CountingTransport final : public proto::ITransport {
public:
    CountingTransport(std::unique_ptr<proto::ITransport> inner, std::shared_ptr<std::atomic<std::uint64_t>> received)
        : inner_(std::move(inner)), received_(std::move(received)) {}

    Status send(std::span<const std::byte> frame) override { return inner_->send(frame); }
    Status sendUnreliable(std::span<const std::byte> frame) override { return inner_->sendUnreliable(frame); }
    Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override {
        Result<bool> got = inner_->receive(frame, timeout);
        if (got && got.value()) ++*received_;
        return got;
    }
    void close() override { inner_->close(); }
    bool connected() const override { return inner_->connected(); }
    std::string describe() const override { return inner_->describe(); }
    std::optional<proto::LinkStats> linkStats() const override { return inner_->linkStats(); }

private:
    std::unique_ptr<proto::ITransport> inner_;
    std::shared_ptr<std::atomic<std::uint64_t>> received_;
};

struct ClientKit {
    testing::ManualWallClock* clock = nullptr;
    PipelineKit kit;
    std::shared_ptr<ipc::BinaryMessageCodec> codec = std::make_shared<ipc::BinaryMessageCodec>();
    std::vector<std::string> hudExtraKeys;
    std::vector<HudWidgetDescriptor> descriptors;

    explicit ClientKit(testing::ManualWallClock& c) : clock(&c) {}

    mc::MatchClientServices services(bool physics = true) {
        mc::MatchClientServices s;
        s.codec = codec;
        s.clock = std::shared_ptr<IWallClock>(std::shared_ptr<void>(), clock);
        s.specPipeline = &kit.pipeline;
        if (physics) {
            s.plugins = &kit.plugins;
            s.drivetrains = &kit.drivetrains;
            s.tireModels = &kit.tires;
        }
        s.hudKeys = mc::HudKeyTable::fromDescriptors(descriptors, hudExtraKeys);
        return s;
    }
};

// ---- a scripted host --------------------------------------------------------------------------------------------------

class ScriptedHost {
public:
    explicit ScriptedHost(std::shared_ptr<ipc::BinaryMessageCodec> codec) {
        pair_ = testing::makeInMemoryTransportPair();
        channel_ = std::make_unique<proto::MessageChannel>(std::move(pair_.a), std::move(codec));
    }

    std::unique_ptr<proto::ITransport> clientEnd() { return std::move(pair_.b); }

    Status send(const proto::Message& m) { return channel_->send(m, proto::deliveryOf(m)); }
    void drain() {
        proto::Message m;
        for (;;) {
            const Result<bool> got = channel_->receive(m, 0ms);
            if (!got || !got.value()) break;
            log.push_back(std::move(m));
        }
    }
    template <class T>
    std::vector<T> all() {
        drain();
        std::vector<T> out;
        for (const proto::Message& m : log)
            if (const T* t = std::get_if<T>(&m)) out.push_back(*t);
        return out;
    }
    template <class T>
    std::optional<T> last() {
        const std::vector<T> v = all<T>();
        return v.empty() ? std::nullopt : std::optional<T>(v.back());
    }
    void close() { channel_->close(); }

    // ---- canned messages ----
    wire::RosterEntry entry(wire::ParticipantId id, const char* name, proto::AllianceStation station, bool host = false) const {
        wire::RosterEntry e;
        e.participantId = id;
        e.name = name;
        e.station = station;
        e.isHost = host;
        return e;
    }

    void accept(wire::MatchState state = wire::MatchState::Lobby, wire::ParticipantId id = 1, std::uint64_t tick = 0) {
        self = id;
        wire::JoinAccept a;
        a.participantId = id;
        a.sessionId = session;
        a.runEpoch = epoch;
        a.station = proto::AllianceStation::Red1;
        a.state = state;
        a.serverTick = tick;
        a.serverSimTimeUs = static_cast<std::int64_t>(tick) * 5000;
        a.startPose = {{14.f, 4.f, 0.f}, sdk::Quat::fromYaw(3.14159f)};
        for (std::size_t i = 0; i < token.size(); ++i) token[i] = static_cast<std::uint8_t>(0x10 + i + epoch);
        a.resumeToken = token;
        a.roster = {entry(0, "Host", proto::AllianceStation::Blue1, true), entry(id, "Me", proto::AllianceStation::Red1)};
        REQUIRE(send(a).ok());
    }

    wire::MatchSnapshot snapshot(std::uint64_t tick, wire::MatchState state, bool enabled = true) {
        wire::MatchSnapshot s;
        s.sessionId = session;
        s.runEpoch = epoch;
        s.snapshotSeq = ++seq;
        s.tick = tick;
        s.simTimeUs = static_cast<std::int64_t>(tick) * 5000;
        s.info.state = state;
        s.info.enabled = enabled;
        for (const wire::ParticipantId id : {wire::ParticipantId{0}, self}) {
            wire::RobotWireState r;
            r.participantId = id;
            // The own robot stands where JoinAccept put it; the host robot (0) somewhere else.
            r.xMm = id == self ? 14000 : 2000;
            r.yMm = 4000;
            r.yawQ = id == self ? wire::quantizeAngle(3.14159) : 0;
            r.wheels.assign(4, {});
            s.robots.push_back(r);
        }
        return s;
    }

    void control(wire::MatchControlKind kind, std::string detail = {}, wire::ParticipantId target = wire::kNoParticipant) {
        wire::MatchControl c;
        c.kind = kind;
        c.sessionId = session;
        c.runEpoch = epoch;
        c.target = target;
        c.detail = std::move(detail);
        REQUIRE(send(c).ok());
    }

    // Answers every ping the client sent so far, as a host whose sim time is `simTimeUs`.
    void answerPings(std::int64_t simTimeUs, std::uint64_t tick) {
        for (const wire::MatchControl& c : all<wire::MatchControl>()) {
            if (c.kind != wire::MatchControlKind::PingRequest || answered.count(c.pingSeq) != 0) continue;
            answered.insert(c.pingSeq);
            wire::MatchControl reply;
            reply.kind = wire::MatchControlKind::PingReply;
            reply.sessionId = session;
            reply.runEpoch = epoch;
            reply.pingSeq = c.pingSeq;
            reply.clientTimeUs = c.clientTimeUs;
            reply.serverTick = tick;
            reply.serverSimTimeUs = simTimeUs;
            REQUIRE(send(reply).ok());
        }
    }

    std::uint64_t session = 0xABCDEF;
    std::uint64_t epoch = 1;
    std::uint32_t seq = 0;
    wire::ParticipantId self = 1;
    wire::ResumeToken token{};
    std::vector<proto::Message> log;
    std::set<std::uint32_t> answered;

private:
    testing::InMemoryTransportPair pair_;
    std::unique_ptr<proto::MessageChannel> channel_;
};

}  // namespace client_test
