#include "wpiemu/core/match/match_client.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <variant>

#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/core/match/client_timing.h"
#include "wpiemu/core/match/entity_memory.h"
#include "wpiemu/core/match/robot_track.h"
#include "wpiemu/core/match/spec_intake.h"
#include "wpiemu/core/runtime/physics_wiring.h"
#include "wpiemu/sdk/game/zone_pose.h"
#include "wpiemu/sdk/match/entity_prediction.h"
#include "wpiemu/sdk/match/input_messages.h"
#include "wpiemu/sdk/match/join_messages.h"
#include "wpiemu/sdk/match/match_control.h"
#include "wpiemu/sdk/match/snapshot_messages.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"
#include "wpiemu/sdk/physics/robot_tags.h"

namespace wpiemu::core::match {

namespace {

namespace wire = sdk::match;
namespace proto = sdk::proto;
using std::chrono::microseconds;

constexpr int kMaxMessagesPerUpdate = 512;
constexpr std::size_t kPingMemory = 16;
constexpr std::size_t kErrorWindow = 128;

double wrapPi(double a) { return wire::angleDifferenceRad(a, 0.0); }

// Render time (microseconds of host sim time): rate 1 +- 20 %, a gap beyond one second is a jump.
TimelineTracker newRenderTracker() { return TimelineTracker(TimelineTracker::Config{200'000.0, 0.2, 1'000'000.0}); }
// Prediction timeline (host ticks): rate 1 +- 5 %, a gap beyond 200 ticks (one second) is a jump.
TimelineTracker newPredTracker() { return TimelineTracker(TimelineTracker::Config{100.0, 0.05, 200.0}); }

sdk::Pose3 nlerpPose(const sdk::Pose3& a, const sdk::Pose3& b, double w) {
    const float t = static_cast<float>(w);
    sdk::Pose3 out;
    out.position = a.position + (b.position - a.position) * t;
    sdk::Quat qb = b.rotation;
    if (a.rotation.x * qb.x + a.rotation.y * qb.y + a.rotation.z * qb.z + a.rotation.w * qb.w < 0.f) qb = {-qb.x, -qb.y, -qb.z, -qb.w};
    out.rotation = sdk::Quat{a.rotation.x + (qb.x - a.rotation.x) * t, a.rotation.y + (qb.y - a.rotation.y) * t,
                             a.rotation.z + (qb.z - a.rotation.z) * t, a.rotation.w + (qb.w - a.rotation.w) * t}.normalized();
    return out;
}

const wire::ZonePoseInput* overrideOf(const std::vector<wire::ZonePoseInput>& list, std::size_t zone) {
    for (const wire::ZonePoseInput& z : list)
        if (z.zoneIndex == zone) return &z;
    return nullptr;
}

bool sameSetpoints(std::span<const sdk::WheelSetpoint> a, std::span<const sdk::WheelSetpoint> b) {
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), [](const sdk::WheelSetpoint& x, const sdk::WheelSetpoint& y) {
        return x.wheelSpeedRadPerSec == y.wheelSpeedRadPerSec && x.steerAngleRad == y.steerAngleRad;
    });
}

}  // namespace

struct MatchClient::Impl {
    using Clock = std::chrono::steady_clock;

    struct Command {
        std::vector<sdk::WheelSetpoint> setpoints;
        std::uint32_t zoneFlags = 0;
        std::vector<wire::ZonePoseInput> zonePoses;
        bool dirty = false;
    };

    // ---- construction ----
    MatchClientConfig config;
    MatchClientServices services;
    Clock::time_point origin;

    // ---- published (mutex) ----
    mutable std::mutex mutex;
    Command command;
    std::optional<sdk::DriveKinematics> kinematics;
    double headingRad = 0.0;
    std::shared_ptr<const MatchClientView> published;
    ConnectionStatus publishedStatus;
    std::shared_ptr<const sdk::FieldDescription> fieldDesc;
    std::shared_ptr<const sdk::RobotConfig> accepted;

    // ---- connection ----
    std::unique_ptr<sdk::proto::MessageChannel> channel;
    ConnectionStatus status;
    bool joined = false;
    bool haveJoin = false;
    wire::Join join;
    std::vector<std::string> specClamps;
    std::vector<float> maxWheelSpeed;
    std::vector<bool> steered;
    std::size_t zoneCount = 0;
    std::int64_t joinSentUs = 0;
    std::int64_t lastRxUs = 0;
    bool reconnecting = false;
    std::uint32_t reconnectAttempt = 0;
    std::int64_t nextReconnectUs = 0;
    std::int64_t backoffUs = 0;

    // ---- identity ----
    wire::ParticipantId myId = wire::kNoParticipant;
    std::uint64_t sessionId = proto::kUnspecified;
    std::uint64_t epoch = proto::kFirstRunEpoch;
    proto::AllianceStation station = proto::AllianceStation::Red1;
    std::int64_t stepUs = 5000;
    std::uint16_t inputRateHz = wire::kDefaultInputRateHz;
    wire::ResumeToken token{};
    sdk::Pose3 startPose;
    std::vector<wire::RosterEntry> roster;
    std::map<wire::ParticipantId, std::shared_ptr<const wire::RobotAppearance>> looks;

    // ---- host state ----
    wire::MatchInfo info;
    bool simStalled = false;
    wire::MatchState effective = wire::MatchState::Lobby;
    bool haveSnapshot = false;
    std::uint32_t lastSeq = 0;
    std::uint64_t lastTick = 0;
    std::int64_t lastSimUs = 0;
    std::int64_t lastSimAdvanceUs = 0;
    std::int64_t lastSnapshotArrivalUs = 0;
    int hostReportedLead = 0;
    std::map<wire::ParticipantId, RobotTrack> tracks;
    std::map<wire::ParticipantId, std::vector<wire::HudValue>> robotHud;
    std::vector<wire::HudValue> gameHud;
    EntityMemory memory;
    std::uint32_t ackSeq = 0;
    bool ackDirty = false;

    // ---- timing ----
    ClockSync clock;
    SnapshotCadence cadence;
    TimelineTracker renderTracker = newRenderTracker();
    TimelineTracker predTracker = newPredTracker();
    std::int64_t delayUs = 0;
    std::int64_t renderSimUs = 0;
    std::int64_t lastUpdateUs = 0;
    std::uint32_t pingSeq = 0;
    std::map<std::uint32_t, std::int64_t> pingSent;
    std::int64_t nextPingUs = 0;
    std::size_t startupPingsLeft = 0;
    double rttMs = 0.0;
    double rttMinMs = 0.0;

    // ---- prediction ----
    std::unique_ptr<OwnPredictor> predictor;
    bool haveOwnAuth = false;
    OwnAuthoritative ownAuth;
    std::uint64_t ownAuthTick = 0;
    std::uint32_t ownAckSeq = wire::kNoInputSeq;
    std::uint32_t inputSeq = 0;
    std::int64_t lastInputTick = -1000;
    wire::MatchInputPacket packet;
    bool packetDirty = false;
    int leadBias = 0;
    int leadTicks = 3;
    std::deque<int> margins;
    std::deque<double> errors;
    ReconcileResult lastReconcile;
    double correctionM = 0.0;
    double frac = 0.0;

    // ---- stats ----
    ClientDiagnostics stats;
    std::uint64_t revision = 0;

    Impl(MatchClientConfig c, MatchClientServices s)
        : config(std::move(c)), services(std::move(s)), origin(services.clock->now()), clock(config.clock) {
        status.state = ConnectionState::Disconnected;
    }

    // ---- small helpers ----
    std::int64_t nowUs() const {
        return std::chrono::duration_cast<microseconds>(services.clock->now() - origin).count();
    }
    void logInfo(const std::string& m) const {
        if (services.logger != nullptr) services.logger->info("match-client", m);
    }
    void logWarn(const std::string& m) const {
        if (services.logger != nullptr) services.logger->warn("match-client", m);
    }
    bool enabledNow() const { return info.enabled && !info.eStop && effective == wire::MatchState::Running; }

    void send(const proto::Message& m) {
        if (!channel) return;
        const sdk::Status s = channel->send(m, proto::deliveryOf(m));
        if (!s.ok()) transportLost(DisconnectReason::TransportLost, s.error().message);
    }

    // ---- status ----
    void setStatus(ConnectionStatus s) {
        status = std::move(s);
        std::lock_guard lock(mutex);
        publishedStatus = status;
    }

    void refreshStatus() {
        if (status.state == ConnectionState::Disconnected) return;
        ConnectionStatus next = status;
        if (!joined) {
            next.state = ConnectionState::Connecting;
        } else {
            next.reconnecting = false;
            next.reconnectAttempt = 0;
            switch (effective) {
                case wire::MatchState::Lobby: next.state = ConnectionState::Lobby; break;
                case wire::MatchState::Running: next.state = ConnectionState::Live; break;
                case wire::MatchState::Paused: next.state = ConnectionState::Paused; break;
                case wire::MatchState::HostStalled: next.state = ConnectionState::HostStalled; break;
            }
            next.reason = DisconnectReason::None;
            next.detail.clear();
            next.rejectReason.reset();
        }
        if (!(next == status)) setStatus(std::move(next));
    }

    void disconnect(DisconnectReason reason, std::string detail, std::optional<wire::JoinRejectReason> reject = std::nullopt) {
        if (channel) channel->close();
        channel.reset();
        joined = false;
        reconnecting = false;
        ConnectionStatus s;
        s.state = ConnectionState::Disconnected;
        s.reason = reason;
        s.detail = std::move(detail);
        s.rejectReason = reject;
        setStatus(std::move(s));
        logInfo("disconnected: " + status.detail);
    }

    // The link broke (or the host went silent): retry through services.reconnect when we have a session to resume.
    void transportLost(DisconnectReason reason, const std::string& detail) {
        if (status.state == ConnectionState::Disconnected) return;
        if (channel) channel->close();
        channel.reset();
        const bool canResume = haveJoin && wire::hasResumeToken(token) && services.reconnect != nullptr;
        if (!canResume) return disconnect(reason, detail);
        joined = false;
        reconnecting = true;
        reconnectAttempt = 0;
        backoffUs = std::chrono::duration_cast<microseconds>(config.reconnectInitialBackoff).count();
        nextReconnectUs = nowUs();
        ConnectionStatus s;
        s.state = ConnectionState::Connecting;
        s.reason = reason;
        s.detail = detail;
        s.reconnecting = true;
        setStatus(std::move(s));
        logWarn("link lost, reconnecting: " + detail);
    }

    // ---- prediction setup ----
    void buildPredictor() {
        predictor.reset();
        {
            std::lock_guard lock(mutex);
            fieldDesc.reset();
        }
        if (services.drivetrains == nullptr || services.tireModels == nullptr || !accepted) return;
        sdk::RobotStartPose placement;
        placement.xM = startPose.position.x;
        placement.yM = startPose.position.y;
        placement.headingRad = startPose.rotation.yaw();
        PredictionWorldArgs args{*accepted, services.plugins, *services.drivetrains, *services.tireModels, services.worldFactory, placement,
                                 sdk::makeRobotTag(myId)};
        sdk::Result<std::unique_ptr<PredictionWorld>> world = PredictionWorld::create(args);
        if (!world) {
            logWarn("no prediction world, drawing the own robot from snapshots: " + world.error().message);
            return;
        }
        auto desc = std::make_shared<const sdk::FieldDescription>(world.value()->field());
        OwnPredictorConfig pc = config.prediction;
        pc.stepUs = stepUs;
        predictor = std::make_unique<OwnPredictor>(std::move(world).value(), pc);
        if (config.predictRemoteContacts)
            predictor->setTickHook([this](PredictionWorld& w, std::uint64_t tick) { placeRemotes(w, tick); });
        std::lock_guard lock(mutex);
        fieldDesc = std::move(desc);
    }

    // The other robots' kinematic bodies for the tick about to run: each dead-reckoned from its newest snapshot to that
    // tick's sim time.
    void placeRemotes(PredictionWorld& w, std::uint64_t tick) {
        const std::int64_t tickSimUs = static_cast<std::int64_t>(tick) * stepUs;
        for (const wire::RosterEntry& e : roster) {
            if (e.participantId == myId) continue;
            const auto look = looks.find(e.participantId);
            const auto track = tracks.find(e.participantId);
            if (look == looks.end() || !look->second || look->second->empty() || track == tracks.end()) continue;
            const RemoteSample* s = track->second.latest();
            if (s == nullptr) continue;
            const double dt = static_cast<double>(std::clamp<std::int64_t>(tickSimUs - s->simTimeUs, 0, config.maxExtrapolationUs)) * 1e-6;
            const float comHeight = wire::appearanceMetres(look->second->comHeightMm);
            const sdk::Pose3 pose{{static_cast<float>(s->x + s->vx * dt), static_cast<float>(s->y + s->vy * dt), comHeight},
                                  sdk::Quat::fromYaw(static_cast<float>(s->yaw + s->omega * dt))};
            w.setRemote(e.participantId, *look->second, pose, {static_cast<float>(s->vx), static_cast<float>(s->vy), 0.f},
                        static_cast<float>(s->omega));
        }
    }

    // ---- join ----
    sdk::Status prepareJoin(const sdk::RobotConfig& robot) {
        const config::RobotConfigCodec codec;
        const std::string json = config::JsonValueCodec().dump(codec.toValue(robot), false);
        if (services.specPipeline == nullptr) return sdk::makeError(sdk::ErrorCode::InvalidState, "no config pipeline to check the robot description");
        sdk::Result<AcceptedSpec> spec = acceptRemoteSpec(json, *services.specPipeline);
        if (!spec) {
            ConnectionStatus s;
            s.state = ConnectionState::Disconnected;
            s.reason = DisconnectReason::BadSpec;
            s.detail = spec.error().message;
            setStatus(std::move(s));
            return spec.error();
        }
        join = wire::Join{};
        join.displayName = config.displayName.substr(0, wire::kMaxNameBytes);
        join.gamePluginId = robot.gamePluginId;
        join.wpilibYear = config.wpilibYear;
        if (join.wpilibYear == 0 && services.plugins != nullptr)
            if (const sdk::IGameModule* m = services.plugins->find(robot.gamePluginId)) join.wpilibYear = m->metadata().year;
        join.robotSpecJson = json;
        specClamps = spec.value().clamped;

        auto cfg = std::make_shared<sdk::RobotConfig>(std::move(spec.value().config));
        const auto& wheels = cfg->drivetrain.spec.wheels;
        maxWheelSpeed.clear();
        steered.clear();
        for (const sdk::WheelSpec& w : wheels) {
            maxWheelSpeed.push_back(sdk::maxWheelSpeedRadPerSec(w));
            steered.push_back(sdk::isSteered(w));
        }
        zoneCount = cfg->zones.size();
        std::lock_guard lock(mutex);
        accepted = cfg;
        command = Command{};
        command.setpoints.assign(wheels.size(), sdk::WheelSetpoint{});
        sdk::Result<sdk::DriveKinematics> kin = sdk::DriveKinematics::create(cfg->drivetrain.spec);
        kinematics.reset();
        if (kin) kinematics.emplace(std::move(kin).value());
        return sdk::okStatus();
    }

    sdk::Status startJoin(std::unique_ptr<sdk::proto::ITransport> transport) {
        channel = std::make_unique<sdk::proto::MessageChannel>(std::move(transport), services.codec);
        joined = false;
        joinSentUs = nowUs();
        lastRxUs = joinSentUs;
        ConnectionStatus s;
        s.state = ConnectionState::Connecting;
        s.reconnecting = reconnecting;
        s.reconnectAttempt = reconnectAttempt;
        setStatus(std::move(s));
        wire::Join message = join;
        message.resumeToken = token;
        const sdk::Status sent = channel->send(message, proto::deliveryOf(proto::Message{message}));
        if (!sent.ok()) {
            channel.reset();
            return sent;
        }
        haveJoin = true;
        return sdk::okStatus();
    }

    void onAccept(const wire::JoinAccept& a, std::int64_t now) {
        const bool sameParticipant = predictor != nullptr && a.participantId == myId && myId != wire::kNoParticipant;
        const bool freshParticipant = a.participantId != myId;
        myId = a.participantId;
        sessionId = a.sessionId;
        epoch = a.runEpoch;
        station = a.station;
        stepUs = std::max<std::int64_t>(1, a.stepUs);
        inputRateHz = a.inputRateHz == 0 ? wire::kDefaultInputRateHz : a.inputRateHz;
        token = a.resumeToken;
        startPose = a.startPose;
        setRoster(a.roster);
        info.state = a.state;
        info.enabled = false;

        // A new stream: nothing of the old connection's timing, tracks or entities applies.
        clock.reset();
        cadence.reset();
        tracks.clear();
        robotHud.clear();
        gameHud.clear();
        memory.clear();
        haveSnapshot = false;
        haveOwnAuth = false;
        ackDirty = false;
        simStalled = false;
        margins.clear();
        renderTracker = newRenderTracker();
        predTracker = newPredTracker();
        delayUs = 0;
        lastSimAdvanceUs = now;
        lastSnapshotArrivalUs = now;
        pingSent.clear();
        startupPingsLeft = config.startupPings;
        nextPingUs = now;
        packet = wire::MatchInputPacket{};
        packetDirty = false;
        lastInputTick = -1000;
        if (freshParticipant) inputSeq = 0;
        if (!sameParticipant || freshParticipant) buildPredictor();
        else if (predictor) {
            predictor->stop();
            predictor->world().clearRemotes();
        }
        if (freshParticipant) leadBias = 0;
        if (a.state == wire::MatchState::Running) clock.seed(now, a.serverSimTimeUs, 0);
        joined = true;
        lastRxUs = now;
        reconnecting = false;
        reconnectAttempt = 0;
        effective = a.state;
        ConnectionStatus s;
        s.state = ConnectionState::Lobby;
        setStatus(std::move(s));
        refreshStatus();
        logInfo("joined as participant " + std::to_string(myId));
    }

    void setRoster(const std::vector<wire::RosterEntry>& entries) {
        roster = entries;
        std::map<wire::ParticipantId, std::shared_ptr<const wire::RobotAppearance>> next;
        for (const wire::RosterEntry& e : entries) {
            if (e.participantId == myId) station = e.station;
            if (e.appearance.empty()) continue;
            const auto old = looks.find(e.participantId);
            if (old != looks.end() && old->second && *old->second == e.appearance) next[e.participantId] = old->second;
            else next[e.participantId] = std::make_shared<const wire::RobotAppearance>(e.appearance);
        }
        for (const auto& [id, look] : looks)
            if (next.find(id) == next.end() && predictor) predictor->world().removeRemote(id);
        looks = std::move(next);
        for (auto it = tracks.begin(); it != tracks.end();) {
            const bool listed = std::any_of(entries.begin(), entries.end(), [&](const wire::RosterEntry& e) { return e.participantId == it->first; });
            it = listed || !joined ? std::next(it) : tracks.erase(it);
        }
    }

    // ---- state machine of the host's world ----
    void updateEffectiveState(std::int64_t now) {
        simStalled = joined && info.state == wire::MatchState::Running && now - lastSimAdvanceUs > std::chrono::duration_cast<microseconds>(config.stallInferenceTimeout).count() &&
                     now - lastSnapshotArrivalUs < std::chrono::duration_cast<microseconds>(config.stallInferenceTimeout).count();
        const wire::MatchState next = simStalled ? wire::MatchState::HostStalled : info.state;
        if (next == effective) return;
        const wire::MatchState previous = effective;
        effective = next;
        if (previous != wire::MatchState::Running && next == wire::MatchState::Running) onRunStart(now);
        refreshStatus();
    }

    // The host's sim time stood still and now moves again: the clock model is void.
    void onRunStart(std::int64_t now) {
        clock.reset();
        if (haveSnapshot) clock.seed(now, lastSimUs, static_cast<std::int64_t>(rttMs * 500.0));
        startupPingsLeft = config.startupPings;
        nextPingUs = now;
        margins.clear();
        renderTracker = newRenderTracker();
        lastSimAdvanceUs = now;
    }

    void adoptEpoch(std::uint64_t newEpoch, std::int64_t now) {
        epoch = newEpoch;
        clock.reset();
        cadence.reset();
        tracks.clear();
        memory.clear();
        robotHud.clear();
        haveOwnAuth = false;
        ownAckSeq = wire::kNoInputSeq;
        margins.clear();
        packet = wire::MatchInputPacket{};
        packetDirty = false;
        lastInputTick = -1000;
        renderTracker = newRenderTracker();
        lastSimUs = 0;
        lastSimAdvanceUs = now;
        if (predictor) {
            predictor->stop();
            predictor->clearInputs();
            predictor->world().clearRemotes();
        }
        startupPingsLeft = config.startupPings;
        nextPingUs = now;
        ++stats.resyncs;
        if (info.state == wire::MatchState::Running && haveSnapshot) clock.seed(now, lastSimUs, 0);
    }

    // ---- messages ----
    bool acceptFrame(std::uint64_t frameSession, std::uint64_t frameEpoch, std::int64_t now, bool epochMayAdvance) {
        switch (proto::classifyFrame(frameSession, frameEpoch, sessionId, epoch)) {
            case proto::FrameDisposition::Accept: return true;
            case proto::FrameDisposition::DropForeignSession:
            case proto::FrameDisposition::DropStaleEpoch: ++stats.snapshotsDropped; return false;
            case proto::FrameDisposition::NewerEpoch:
                if (!epochMayAdvance) {
                    ++stats.snapshotsDropped;
                    return false;
                }
                adoptEpoch(frameEpoch, now);
                return true;
        }
        return false;
    }

    void onSnapshot(const wire::MatchSnapshot& s, std::int64_t now) {
        if (!joined) return;
        if (!acceptFrame(s.sessionId, s.runEpoch, now, true)) return;
        if (haveSnapshot && !wire::seqNewer(s.snapshotSeq, lastSeq)) {
            ++stats.snapshotsStale;
            return;
        }
        ++stats.snapshotsReceived;
        lastRxUs = now;
        lastSnapshotArrivalUs = now;
        cadence.onSnapshot(now, s.simTimeUs, s.snapshotSeq);
        if (!haveSnapshot || s.simTimeUs > lastSimUs || s.info.state != info.state) lastSimAdvanceUs = now;
        const bool firstSnapshot = !haveSnapshot;
        haveSnapshot = true;
        lastSeq = s.snapshotSeq;
        lastTick = s.tick;
        lastSimUs = s.simTimeUs;
        info = s.info;
        gameHud = s.gameHud;
        updateEffectiveState(now);
        if (firstSnapshot && effective == wire::MatchState::Running && !clock.synced())
            clock.seed(now, s.simTimeUs, static_cast<std::int64_t>(rttMs * 500.0));

        for (const wire::RobotWireState& r : s.robots) {
            tracks[r.participantId].push(sampleFromWire(r, s.simTimeUs, now));
            robotHud[r.participantId] = r.hud;
            if (r.participantId != myId) continue;
            ownAuth = authoritativeFromWire(r);
            ownAuthTick = s.tick;
            haveOwnAuth = true;
        }
        ownAckSeq = s.ackInputSeq;
        hostReportedLead = s.inputLeadTicks;
        if (s.inputLeadTicks < 0) ++stats.lateInputs;

        if (predictor && predictor->started() && haveOwnAuth) reconcile(s);
        collectMargin(s.inputLeadTicks);

        const EntityMemory::Result applied = memory.apply(s);
        if (applied == EntityMemory::Result::Applied) {
            ackSeq = s.snapshotSeq;
            ackDirty = true;
        } else if (applied == EntityMemory::Result::UnknownBaseline) {
            ++stats.entityBaselineMisses;
        }
    }

    void reconcile(const wire::MatchSnapshot& s) {
        lastReconcile = predictor->reconcile(ownAuth, s.tick, s.ackInputSeq, enabledNow());
        switch (lastReconcile.kind) {
            case ReconcileKind::InBand:
            case ReconcileKind::Corrected:
            case ReconcileKind::Snapped:
                errors.push_back(lastReconcile.posErrM);
                while (errors.size() > kErrorWindow) errors.pop_front();
                break;
            default: break;
        }
        if (lastReconcile.kind == ReconcileKind::Corrected || lastReconcile.kind == ReconcileKind::Snapped) {
            ++stats.reconciliations;
            stats.replayedTicks += lastReconcile.replayedTicks;
            if (lastReconcile.kind == ReconcileKind::Snapped) ++stats.snaps;
        }
        // The host is ahead of the prediction (or its history is gone): start the timeline over from this snapshot.
        if (lastReconcile.kind == ReconcileKind::Ahead || lastReconcile.kind == ReconcileKind::TooOld) predictor->stop();
    }

    // The margin by which the host received the newest input before its tick, judged over a window; the lead follows it.
    void collectMargin(int margin) {
        if (!predictor || !predictor->started() || effective != wire::MatchState::Running) return;
        if (std::fabs(predTracker.value() - targetTicks(nowUs())) > 2.0) return;
        margins.push_back(margin);
        if (margins.size() < config.leadWindow) return;
        const int lowest = *std::min_element(margins.begin(), margins.end());
        if (lowest < config.leadMarginLowTicks) {
            leadBias += config.leadMarginTicks - lowest;
            margins.clear();
        } else if (lowest > config.leadMarginHighTicks) {
            leadBias -= std::max(1, (lowest - config.leadMarginTicks) / 2);
            margins.clear();
        } else {
            margins.pop_front();
        }
    }

    void onControl(const wire::MatchControl& c, std::int64_t now) {
        switch (c.kind) {
            case wire::MatchControlKind::PingReply: {
                if (c.sessionId != proto::kUnspecified && c.sessionId != sessionId) return;
                const auto it = pingSent.find(c.pingSeq);
                if (it == pingSent.end()) return;
                const std::int64_t sentUs = it->second;
                pingSent.erase(it);
                ++stats.pingsReceived;
                const double rtt = static_cast<double>(now - sentUs) / 1000.0;
                rttMs = rttMs == 0.0 ? rtt : rttMs + (rtt - rttMs) / 8.0;
                rttMinMs = rttMinMs == 0.0 ? rtt : std::min(rttMinMs, rtt);
                if (c.runEpoch != proto::kUnspecified && c.runEpoch > epoch) adoptEpoch(c.runEpoch, now);
                if (c.runEpoch != proto::kUnspecified && c.runEpoch < epoch) return;
                if (effective == wire::MatchState::Running) clock.addPing(sentUs, now, c.serverSimTimeUs);
                return;
            }
            case wire::MatchControlKind::Reset:
                if (c.sessionId != proto::kUnspecified && c.sessionId != sessionId) return;
                if (c.runEpoch > epoch) adoptEpoch(c.runEpoch, now);
                return;
            case wire::MatchControlKind::Pause:
                if (c.sessionId != proto::kUnspecified && c.sessionId != sessionId) return;
                info.state = wire::MatchState::Paused;
                updateEffectiveState(now);
                return;
            case wire::MatchControlKind::Resume:
                if (c.sessionId != proto::kUnspecified && c.sessionId != sessionId) return;
                if (info.state == wire::MatchState::Paused) info.state = wire::MatchState::Running;
                lastSimAdvanceUs = now;
                updateEffectiveState(now);
                return;
            case wire::MatchControlKind::Kick:
                if (c.target == myId || c.target == wire::kNoParticipant) disconnect(DisconnectReason::Kicked, c.detail);
                return;
            case wire::MatchControlKind::ServerClosing: disconnect(DisconnectReason::ServerClosing, c.detail); return;
            case wire::MatchControlKind::PingRequest: return;
        }
    }

    void dispatch(const proto::Message& m, std::int64_t now) {
        if (const auto* accept = std::get_if<wire::JoinAccept>(&m)) {
            onAccept(*accept, now);
            return;
        }
        if (const auto* reject = std::get_if<wire::JoinReject>(&m)) {
            if (!joined) disconnect(DisconnectReason::JoinRejected, reject->detail, reject->reason);
            return;
        }
        if (!joined) return;
        lastRxUs = now;
        if (const auto* snap = std::get_if<wire::MatchSnapshot>(&m)) {
            onSnapshot(*snap, now);
        } else if (const auto* control = std::get_if<wire::MatchControl>(&m)) {
            onControl(*control, now);
        } else if (const auto* r = std::get_if<wire::Roster>(&m)) {
            if (acceptFrame(r->sessionId, r->runEpoch, now, false)) setRoster(r->entries);
        }
    }

    void receiveAll(std::int64_t now) {
        for (int i = 0; i < kMaxMessagesPerUpdate && channel; ++i) {
            proto::Message m;
            const sdk::Result<bool> got = channel->receive(m, std::chrono::milliseconds(0));
            if (!got) {
                transportLost(DisconnectReason::TransportLost, got.error().message);
                return;
            }
            if (!got.value()) return;
            dispatch(m, now);
        }
    }

    // ---- timers ----
    void timers(std::int64_t now) {
        if (!channel) {
            if (reconnecting && now >= nextReconnectUs) tryReconnect(now);
            return;
        }
        if (!joined) {
            if (now - joinSentUs > std::chrono::duration_cast<microseconds>(config.joinTimeout).count()) {
                if (reconnecting) transportLost(DisconnectReason::JoinTimeout, "no answer to the rejoin");
                else disconnect(DisconnectReason::JoinTimeout, "the host did not answer the Join");
            }
            return;
        }
        if (now - lastRxUs > std::chrono::duration_cast<microseconds>(config.hostSilenceTimeout).count()) {
            transportLost(DisconnectReason::HostSilent, "the host sent nothing for " + std::to_string(config.hostSilenceTimeout.count()) + " ms");
            return;
        }
        if (now >= nextPingUs) {
            wire::MatchControl ping;
            ping.kind = wire::MatchControlKind::PingRequest;
            ping.sessionId = sessionId;
            ping.runEpoch = epoch;
            ping.pingSeq = ++pingSeq;
            ping.clientTimeUs = now;
            pingSent[ping.pingSeq] = now;
            while (pingSent.size() > kPingMemory) pingSent.erase(pingSent.begin());
            ++stats.pingsSent;
            const bool fast = startupPingsLeft > 0;
            if (fast) --startupPingsLeft;
            nextPingUs = now + std::chrono::duration_cast<microseconds>(fast ? config.startupPingInterval : config.pingInterval).count();
            send(ping);
        }
    }

    void tryReconnect(std::int64_t now) {
        sdk::Result<std::unique_ptr<sdk::proto::ITransport>> link = services.reconnect();
        if (link && link.value()) {
            ++reconnectAttempt;
            if (startJoin(std::move(link).value()).ok()) return;
        } else {
            ++reconnectAttempt;
        }
        if (reconnectAttempt >= config.maxReconnectAttempts) {
            reconnecting = false;
            haveJoin = true;
            disconnect(DisconnectReason::TransportLost, "could not reconnect after " + std::to_string(reconnectAttempt) + " attempts");
            return;
        }
        nextReconnectUs = now + backoffUs;
        backoffUs = std::min(backoffUs * 2, std::chrono::duration_cast<microseconds>(config.reconnectMaxBackoff).count());
        ConnectionStatus s = status;
        s.state = ConnectionState::Connecting;
        s.reconnecting = true;
        s.reconnectAttempt = reconnectAttempt;
        setStatus(std::move(s));
    }

    // ---- the prediction timeline ----
    double targetTicks(std::int64_t now) const {
        return static_cast<double>(clock.serverSimAt(now)) / static_cast<double>(stepUs) + static_cast<double>(leadTicks);
    }

    void updateLead() {
        const double owdUs = static_cast<double>(clock.minRttUs()) * 0.5;
        const double wantedUs = owdUs + 2.0 * static_cast<double>(clock.jitterUs());
        const int base = static_cast<int>(std::ceil(wantedUs / static_cast<double>(stepUs))) + config.leadMarginTicks;
        leadTicks = std::clamp(base + leadBias, config.minLeadTicks, config.maxLeadTicks);
    }

    std::optional<PredictedInput> makeInput(std::uint64_t tick) {
        const std::int64_t keepalive = std::max<std::int64_t>(1, std::llround(1e6 / static_cast<double>(stepUs) / static_cast<double>(std::max<std::uint16_t>(1, inputRateHz))));
        const std::int64_t since = static_cast<std::int64_t>(tick) - lastInputTick;
        Command cmd;
        {
            std::lock_guard lock(mutex);
            cmd = command;
        }
        const bool due = (cmd.dirty && since >= config.minInputSpacingTicks) || since >= keepalive;
        if (!due) return std::nullopt;
        if (cmd.setpoints.size() != maxWheelSpeed.size()) return std::nullopt;
        if (cmd.dirty) {
            std::lock_guard lock(mutex);
            if (sameSetpoints(command.setpoints, cmd.setpoints) && command.zoneFlags == cmd.zoneFlags && command.zonePoses == cmd.zonePoses) command.dirty = false;
        }
        lastInputTick = static_cast<std::int64_t>(tick);
        if (++inputSeq == wire::kNoInputSeq) ++inputSeq;

        PredictedInput in;
        in.seq = inputSeq;
        in.targetTick = tick;
        in.zoneFlags = cmd.zoneFlags & wire::zoneFlagsMask(zoneCount);
        in.zonePoses = cmd.zonePoses;
        wire::MatchInput wireInput;
        wireInput.inputSeq = inputSeq;
        wireInput.targetTick = tick;
        wireInput.zoneFlags = in.zoneFlags;
        wireInput.zonePoses = cmd.zonePoses;
        for (std::size_t i = 0; i < cmd.setpoints.size(); ++i) {
            const float speed = wire::sanitizedWheelSpeed(cmd.setpoints[i].wheelSpeedRadPerSec, maxWheelSpeed[i]);
            const std::int16_t q = wire::quantizeAngle(cmd.setpoints[i].steerAngleRad);
            wireInput.wheels.push_back({speed, q});
            // What the host will apply: the sanitized speed and the dequantized angle of a steered wheel.
            in.setpoints.push_back({speed, steered[i] ? wire::dequantizeAngle(q) : 0.f});
        }
        wire::appendInput(packet, std::move(wireInput));
        packetDirty = true;
        ++stats.inputsSent;
        return in;
    }

    void startPrediction(double target) {
        const std::uint64_t toTick = static_cast<std::uint64_t>(std::max(static_cast<double>(ownAuthTick), std::ceil(target)));
        const std::uint64_t cap = ownAuthTick + static_cast<std::uint64_t>(2 * config.maxLeadTicks + 40);
        predictor->start(ownAuth, ownAuthTick, std::min(toTick, cap), enabledNow());
        predTracker.reset(target);
        lastInputTick = static_cast<std::int64_t>(predictor->tick()) - 1000;
        ++stats.resyncs;
    }

    void advanceTimelines(std::int64_t now) {
        const std::int64_t dtUs = std::clamp<std::int64_t>(now - lastUpdateUs, 0, 250'000);
        lastUpdateUs = now;
        const bool running = effective == wire::MatchState::Running;

        // Remote robots and entities: drawn at the host sim time minus the interpolation delay.
        const std::int64_t targetDelay = std::min(cadence.targetDelayUs(config.maxInterpolationDelayUs), config.maxInterpolationDelayUs);
        delayUs += std::clamp<std::int64_t>(targetDelay - delayUs, -dtUs / 10, dtUs / 10);
        if (delayUs == 0) delayUs = targetDelay;
        if (running && clock.synced()) {
            const double target = static_cast<double>(clock.serverSimAt(now) - delayUs);
            renderTracker.advance(static_cast<double>(dtUs), target);
            renderSimUs = static_cast<std::int64_t>(renderTracker.value());
            renderSimUs = std::min(renderSimUs, lastSimUs + config.maxExtrapolationUs);
        } else if (haveSnapshot) {
            renderSimUs = lastSimUs;
        }

        // The own robot.
        if (predictor == nullptr) return;
        predictor->decayCorrection(dtUs);
        if (!running || !clock.hasPings() || !haveOwnAuth) return;
        updateLead();
        const double target = targetTicks(now);
        if (!predictor->started()) {
            startPrediction(target);
            frac = 1.0;
            return;
        }
        const bool snapped = predTracker.advance(static_cast<double>(dtUs) / static_cast<double>(stepUs), target);
        const double p = predTracker.value();
        if (snapped || p > static_cast<double>(predictor->tick()) + config.maxCatchUpTicks) {
            startPrediction(target);
            frac = 1.0;
            return;
        }
        const std::uint64_t want = static_cast<std::uint64_t>(std::max(0.0, std::floor(p))) + 1;
        const bool enabled = enabledNow();
        while (predictor->tick() < want) {
            std::optional<PredictedInput> in = makeInput(predictor->tick());
            predictor->stepTick(in ? &*in : nullptr, enabled);
        }
        frac = std::clamp(p - static_cast<double>(predictor->tick() - 1), 0.0, 1.0);
    }

    void flushNetwork() {
        if (!joined || !channel) return;
        if (packetDirty) {
            packet.sessionId = sessionId;
            packet.runEpoch = epoch;
            packetDirty = false;
            send(packet);
        }
        if (ackDirty) {
            ackDirty = false;
            wire::MatchSnapshotAck ack;
            ack.sessionId = sessionId;
            ack.runEpoch = epoch;
            ack.snapshotSeq = ackSeq;
            send(ack);
        }
    }

    // ---- view ----
    std::vector<HudEntry> hudEntries(const std::vector<wire::HudValue>& values) const {
        std::vector<HudEntry> out;
        out.reserve(values.size());
        for (const wire::HudValue& v : values) {
            HudEntry e;
            e.keyId = v.keyId;
            if (const auto key = services.hudKeys.keyOf(v.keyId)) e.key = std::string(*key);
            e.value = v.value;
            out.push_back(std::move(e));
        }
        return out;
    }

    std::vector<WheelView> wheelViews(const std::vector<RobotAt::Wheel>& wheels) const {
        std::vector<WheelView> out;
        out.reserve(wheels.size());
        for (const RobotAt::Wheel& w : wheels) out.push_back({static_cast<float>(wrapPi(w.steerRad)), static_cast<float>(wrapPi(w.spinRad))});
        return out;
    }

    std::vector<sdk::Pose3> blendedZonePoses(const std::vector<sdk::Pose3>& rest, const RobotAt& at) const {
        std::vector<sdk::Pose3> out = rest;
        for (std::size_t i = 0; i < out.size(); ++i) {
            const wire::ZonePoseInput* a = at.from != nullptr ? overrideOf(at.from->zonePoses, i) : nullptr;
            const wire::ZonePoseInput* b = at.to != nullptr ? overrideOf(at.to->zonePoses, i) : nullptr;
            if (a == nullptr && b == nullptr) continue;
            const sdk::Pose3 pa = a != nullptr ? wire::toPose3(*a) : rest[i];
            const sdk::Pose3 pb = at.to == nullptr ? pa : (b != nullptr ? wire::toPose3(*b) : rest[i]);
            out[i] = nlerpPose(pa, pb, at.to == nullptr ? 0.0 : at.blend);
        }
        return out;
    }

    std::vector<sdk::Pose3> ownRestPoses() const {
        std::vector<sdk::Pose3> out;
        if (!accepted) return out;
        for (const sdk::ZoneConfig& z : accepted->zones) out.push_back(sdk::remoteZoneSpecPose(z));
        return out;
    }

    std::shared_ptr<const wire::RobotAppearance> lookOf(wire::ParticipantId id) const {
        const auto it = looks.find(id);
        return it == looks.end() ? nullptr : it->second;
    }

    const wire::RosterEntry* rosterOf(wire::ParticipantId id) const {
        for (const wire::RosterEntry& e : roster)
            if (e.participantId == id) return &e;
        return nullptr;
    }

    void fillOwn(MatchClientView& v, std::int64_t now) {
        OwnRobotView& own = v.own;
        own.id = myId;
        const wire::RosterEntry* entry = rosterOf(myId);
        own.name = entry != nullptr ? entry->name : join.displayName;
        own.station = station;
        own.appearance = lookOf(myId);
        own.enabled = enabledNow();
        const auto hud = robotHud.find(myId);
        if (hud != robotHud.end()) own.hud = hudEntries(hud->second);

        std::vector<sdk::Pose3> rest = ownRestPoses();
        std::uint32_t flags = 0;
        std::vector<sdk::Pose3> zones = rest;
        {
            std::lock_guard lock(mutex);
            flags = command.zoneFlags & wire::zoneFlagsMask(zoneCount);
            for (const wire::ZonePoseInput& z : command.zonePoses)
                if (z.zoneIndex < zones.size() && accepted) zones[z.zoneIndex] = sdk::clampRemoteZonePose(accepted->chassis, wire::toPose3(z)).pose;
        }

        if (predictor != nullptr && predictor->started()) {
            const OwnShown shown = predictor->shown(frac);
            own.predicted = true;
            own.pose = shown.pose;
            own.linearVelocity = shown.linearVelocity;
            own.angularVelocity = shown.angularVelocity;
            for (const OwnShown::Wheel& w : shown.wheels) own.wheels.push_back({static_cast<float>(wrapPi(w.steerRad)), static_cast<float>(wrapPi(w.spinRad))});
            own.zoneFlags = own.enabled ? flags : 0;
            own.zonePoses = std::move(zones);
            own.correctionM = predictor->correctionMagnitudeM();
            v.hasOwn = true;
            return;
        }
        const auto track = tracks.find(myId);
        if (track == tracks.end()) {
            if (!joined) return;
            own.pose = startPose;
            v.hasOwn = true;
            own.zonePoses = std::move(zones);
            return;
        }
        const std::optional<RobotAt> at = track->second.at(renderSimUs, config.maxExtrapolationUs);
        if (!at) return;
        const float comHeight = own.appearance ? wire::appearanceMetres(own.appearance->comHeightMm) : 0.f;
        own.pose = {{static_cast<float>(at->x), static_cast<float>(at->y), comHeight}, sdk::Quat::fromYaw(static_cast<float>(at->yaw))};
        own.linearVelocity = {static_cast<float>(at->vx), static_cast<float>(at->vy), 0.f};
        own.angularVelocity = {0.f, 0.f, static_cast<float>(at->omega)};
        own.wheels = wheelViews(at->wheels);
        own.zoneFlags = own.enabled ? flags : 0;
        own.zonePoses = std::move(zones);
        v.hasOwn = true;
        (void)now;
    }

    void fillRemotes(MatchClientView& v) {
        for (const auto& [id, track] : tracks) {
            if (id == myId) continue;
            const std::optional<RobotAt> at = track.at(renderSimUs, config.maxExtrapolationUs);
            if (!at) continue;
            RemoteRobotView r;
            r.id = id;
            r.isHost = id == wire::kHostParticipantId;
            if (const wire::RosterEntry* e = rosterOf(id)) {
                r.name = e->name;
                r.station = e->station;
            }
            r.appearance = lookOf(id);
            const float comHeight = r.appearance ? wire::appearanceMetres(r.appearance->comHeightMm) : 0.f;
            r.pose = {{static_cast<float>(at->x), static_cast<float>(at->y), comHeight}, sdk::Quat::fromYaw(static_cast<float>(at->yaw))};
            r.linearVelocity = {static_cast<float>(at->vx), static_cast<float>(at->vy), 0.f};
            r.angularVelocity = {0.f, 0.f, static_cast<float>(at->omega)};
            r.wheels = wheelViews(at->wheels);
            r.zoneFlags = at->zoneFlags;
            if (r.appearance) r.zonePoses = blendedZonePoses(wire::appearanceZoneRestPoses(*r.appearance), *at);
            const auto hud = robotHud.find(id);
            if (hud != robotHud.end()) r.hud = hudEntries(hud->second);
            r.extrapolating = at->extrapolating;
            r.starved = at->starved;
            v.remotes.push_back(std::move(r));
        }
    }

    void fillEntities(MatchClientView& v) {
        if (!memory.seen()) return;
        const std::optional<std::uint32_t> seq = memory.newestAtOrBefore(renderSimUs);
        if (!seq) return;
        const std::optional<std::uint32_t> before = memory.previousOf(*seq);
        for (const std::uint16_t groupId : memory.groupIds(*seq)) {
            EntityGroupView group;
            group.groupId = groupId;
            const EntityMemory::Group& now = memory.groupOf(*seq, groupId);
            const EntityMemory::Group* old = before ? &memory.groupOf(*before, groupId) : nullptr;
            group.entities.reserve(now.size());
            for (const auto& [id, held] : now) {
                const EntityMemory::Held* previous = nullptr;
                if (old != nullptr) {
                    const auto it = old->find(id);
                    if (it != old->end() && it->second.stampUs < held.stampUs) previous = &it->second;
                }
                const wire::EntityPositionCm p = wire::renderEntityCm(previous != nullptr ? &previous->state : nullptr,
                                                                      previous != nullptr ? previous->stampUs : 0, held.state, held.stampUs, renderSimUs);
                group.entities.push_back({id, static_cast<float>(p.x * 0.01), static_cast<float>(p.y * 0.01), static_cast<float>(p.z * 0.01)});
            }
            v.entities.push_back(std::move(group));
        }
    }

    double percentile95(std::deque<double> values) const {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        return values[static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(values.size()))) - 1];
    }

    void publishView(std::int64_t now) {
        auto v = std::make_shared<MatchClientView>();
        v->revision = ++revision;
        v->connection = status;
        v->sessionId = sessionId;
        v->runEpoch = epoch;
        v->renderSimTimeUs = renderSimUs;
        v->roster = roster;
        v->specClamps = specClamps;
        if (haveJoin) {
            fillOwn(*v, now);
            fillRemotes(*v);
            fillEntities(*v);
        }
        v->hud.state = effective;
        v->hud.enabled = info.enabled;
        v->hud.mode = info.mode;
        v->hud.eStop = info.eStop;
        v->hud.matchTimeS = info.matchTimeS;
        v->hud.phaseKey = info.phaseKey;
        v->hud.game = hudEntries(gameHud);

        ClientDiagnostics d = stats;
        d.rttMs = rttMs;
        d.rttMinMs = rttMinMs;
        d.jitterMs = static_cast<double>(clock.jitterUs()) / 1000.0;
        d.snapshotJitterMs = static_cast<double>(cadence.jitterUs()) / 1000.0;
        d.lossPermille = cadence.lossPermille();
        d.snapshotRateHz = cadence.rateHz();
        d.snapshotAgeMs = haveSnapshot ? static_cast<double>(now - lastSnapshotArrivalUs) / 1000.0 : 0.0;
        d.bufferDelayMs = static_cast<double>(delayUs) / 1000.0;
        d.clockDriftPpm = clock.driftPpm();
        d.clockAsymmetryBoundMs = static_cast<double>(clock.asymmetryBoundUs()) / 1000.0;
        d.clockSynced = clock.synced();
        d.predictionErrorM = lastReconcile.posErrM;
        d.predictionErrorYawRad = lastReconcile.yawErrRad;
        d.predictionErrorP95M = percentile95(errors);
        if (predictor && predictor->started() && clock.synced())
            d.predictionAheadMs = static_cast<double>(static_cast<std::int64_t>(predictor->tick()) * stepUs - clock.serverSimAt(now)) / 1000.0;
        d.inputLeadTicks = leadTicks;
        d.hostReportedLeadTicks = hostReportedLead;
        v->diagnostics = d;

        std::lock_guard lock(mutex);
        published = std::move(v);
        if (predictor && predictor->started()) headingRad = predictor->newest().pose.rotation.yaw();
    }

    void update() {
        const std::int64_t now = nowUs();
        receiveAll(now);
        if (channel || reconnecting) timers(now);
        if (joined) {
            updateEffectiveState(now);
            advanceTimelines(now);
            flushNetwork();
        }
        publishView(now);
    }
};

// ---- MatchClient -------------------------------------------------------------------------------------------------

MatchClient::MatchClient(MatchClientConfig config, MatchClientServices services)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(services))) {}

MatchClient::~MatchClient() = default;

sdk::Status MatchClient::connect(std::unique_ptr<sdk::proto::ITransport> transport, const sdk::RobotConfig& robot) {
    Impl& m = *impl_;
    if (m.status.state != ConnectionState::Disconnected || m.channel)
        return sdk::makeError(sdk::ErrorCode::InvalidState, "the client is already connected");
    if (!transport) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "no transport");
    m.haveJoin = false;
    m.token = {};
    m.myId = wire::kNoParticipant;
    if (const sdk::Status s = m.prepareJoin(robot); !s.ok()) return s;
    return m.startJoin(std::move(transport));
}

sdk::Status MatchClient::resume(std::unique_ptr<sdk::proto::ITransport> transport) {
    Impl& m = *impl_;
    if (m.status.state != ConnectionState::Disconnected || m.channel)
        return sdk::makeError(sdk::ErrorCode::InvalidState, "the client is already connected");
    if (!m.haveJoin) return sdk::makeError(sdk::ErrorCode::InvalidState, "nothing to resume: connect() first");
    if (!transport) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "no transport");
    m.reconnecting = true;
    return m.startJoin(std::move(transport));
}

void MatchClient::leave(std::string detail) {
    Impl& m = *impl_;
    if (m.channel && m.joined) {
        wire::Leave l;
        l.detail = detail.substr(0, wire::kMaxDetailBytes);
        (void)m.channel->send(l, sdk::proto::Delivery::Reliable);
    }
    m.disconnect(DisconnectReason::Left, std::move(detail));
    m.haveJoin = false;
    m.token = {};
}

void MatchClient::update() { impl_->update(); }

sdk::Status MatchClient::setWheelSetpoints(std::span<const sdk::WheelSetpoint> setpoints) {
    Impl& m = *impl_;
    std::lock_guard lock(m.mutex);
    if (setpoints.size() != m.command.setpoints.size())
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, "expected " + std::to_string(m.command.setpoints.size()) + " wheel setpoints");
    for (const sdk::WheelSetpoint& sp : setpoints)
        if (!std::isfinite(sp.wheelSpeedRadPerSec) || !std::isfinite(sp.steerAngleRad))
            return sdk::makeError(sdk::ErrorCode::InvalidArgument, "wheel setpoints must be finite");
    const bool changed = !sameSetpoints(setpoints, m.command.setpoints);
    if (changed) {
        m.command.setpoints.assign(setpoints.begin(), setpoints.end());
        m.command.dirty = true;
    }
    return sdk::okStatus();
}

sdk::Status MatchClient::driveRobotRelative(const sdk::ChassisSpeeds& speeds) {
    Impl& m = *impl_;
    std::vector<sdk::WheelSetpoint> out;
    {
        std::lock_guard lock(m.mutex);
        if (!m.kinematics) return sdk::makeError(sdk::ErrorCode::Unsupported, "this drivetrain has no built-in kinematics: set wheel setpoints directly");
        sdk::Result<std::vector<sdk::WheelSetpoint>> sp = m.kinematics->update(speeds);
        if (!sp) return sp.error();
        out = std::move(sp).value();
    }
    return setWheelSetpoints(out);
}

sdk::Status MatchClient::driveFieldRelative(const sdk::ChassisSpeeds& field) {
    double heading = 0.0;
    {
        std::lock_guard lock(impl_->mutex);
        heading = impl_->headingRad;
    }
    const float c = static_cast<float>(std::cos(heading)), s = static_cast<float>(std::sin(heading));
    sdk::ChassisSpeeds robot;
    robot.vx = c * field.vx + s * field.vy;
    robot.vy = -s * field.vx + c * field.vy;
    robot.omega = field.omega;
    return driveRobotRelative(robot);
}

void MatchClient::setZoneFlags(std::uint32_t flags) {
    std::lock_guard lock(impl_->mutex);
    if (impl_->command.zoneFlags == flags) return;
    impl_->command.zoneFlags = flags;
    impl_->command.dirty = true;
}

void MatchClient::setZoneActive(std::size_t zone, bool active) {
    const std::uint32_t bit = wire::zoneFlagBit(zone);
    std::lock_guard lock(impl_->mutex);
    const std::uint32_t next = active ? (impl_->command.zoneFlags | bit) : (impl_->command.zoneFlags & ~bit);
    if (next == impl_->command.zoneFlags) return;
    impl_->command.zoneFlags = next;
    impl_->command.dirty = true;
}

sdk::Status MatchClient::setZonePose(std::size_t zone, const sdk::Pose3& pose) {
    std::lock_guard lock(impl_->mutex);
    wire::MatchInput scratch;
    scratch.zonePoses = impl_->command.zonePoses;
    if (!wire::setZonePose(scratch, zone, pose)) return sdk::makeError(sdk::ErrorCode::OutOfRange, "zone index beyond the protocol limit");
    if (scratch.zonePoses == impl_->command.zonePoses) return sdk::okStatus();
    impl_->command.zonePoses = std::move(scratch.zonePoses);
    impl_->command.dirty = true;
    return sdk::okStatus();
}

void MatchClient::clearZonePose(std::size_t zone) {
    std::lock_guard lock(impl_->mutex);
    auto& list = impl_->command.zonePoses;
    const auto it = std::remove_if(list.begin(), list.end(), [zone](const wire::ZonePoseInput& z) { return z.zoneIndex == zone; });
    if (it == list.end()) return;
    list.erase(it, list.end());
    impl_->command.dirty = true;
}

std::shared_ptr<const MatchClientView> MatchClient::view() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->published;
}

ConnectionStatus MatchClient::status() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->publishedStatus;
}

std::shared_ptr<const sdk::FieldDescription> MatchClient::fieldDescription() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->fieldDesc;
}

std::shared_ptr<const sdk::RobotConfig> MatchClient::acceptedSpec() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->accepted;
}

std::size_t MatchClient::wheelCount() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->command.setpoints.size();
}

}  // namespace wpiemu::core::match
