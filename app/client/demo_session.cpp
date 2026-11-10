#include "client/demo_session.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

#include "wpiemu/core/match/prediction_world.h"
#include "wpiemu/core/match/robot_appearance.h"
#include "wpiemu/sdk/physics/robot_tags.h"

namespace wpiemu::app::client {
namespace {

namespace mc = core::match;
namespace wire = sdk::match;
constexpr float kPi = 3.14159265358979323846f;

struct Scripted {
    wire::ParticipantId id;
    const char* name;
    sdk::proto::AllianceStation station;
    float cx, cy, radius, phase, speedMps;
    float sizeX;
};

constexpr Scripted kOthers[] = {
    {wire::kHostParticipantId, "Host", sdk::proto::AllianceStation::Blue1, 5.2f, 3.6f, 2.0f, 0.3f, 1.8f, 0.80f},
    {2, "Red Rockets", sdk::proto::AllianceStation::Red2, 11.0f, 5.2f, 2.6f, 2.1f, 2.1f, 0.72f},
    {3, "Gearheads", sdk::proto::AllianceStation::Blue3, 8.4f, 1.9f, 1.7f, 4.0f, 1.4f, 0.68f},
};

sdk::WheelSpec wheelAt(const char* name, float x, float y) {
    sdk::WheelSpec w;
    w.name = name;
    w.kind = sdk::WheelKind::Swerve;
    w.positionM = {x, y, 0.f};
    w.steered = true;
    w.driveCurrentLimitA = 60.f;
    return w;
}

sdk::RobotConfig swerveRobot(const std::string& name, float size) {
    sdk::RobotConfig c;
    c.name = name;
    c.chassis.sizeM = {size, size, 0.15f};
    c.chassis.massKg = 52.f;
    c.chassis.comHeightM = 0.2f;
    c.chassis.groundClearanceM = 0.03f;
    c.drivetrain.spec.kind = sdk::drivetrain_kind::kSwerve;
    const float h = 0.5f * size - 0.08f;
    c.drivetrain.spec.wheels = {wheelAt("fl", h, h), wheelAt("fr", h, -h), wheelAt("rl", -h, h), wheelAt("rr", -h, -h)};
    return c;
}

float wrapPi(float a) {
    while (a > kPi) a -= 2.f * kPi;
    while (a < -kPi) a += 2.f * kPi;
    return a;
}

void collectKeys(const sdk::HudWidgetDescriptor& d, std::vector<std::pair<std::string, float>>& out) {
    const auto add = [&](const std::string& key, float value) {
        if (key.empty()) return;
        for (auto& [k, v] : out)
            if (k == key) return;
        out.push_back({key, value});
    };
    const bool meter = d.kind == sdk::HudWidgetKind::Meter || d.kind == sdk::HudWidgetKind::Gauge;
    if (d.kind != sdk::HudWidgetKind::Scoreboard) add(d.valueKey, meter ? static_cast<float>(0.6 * d.maxValue) : 12.f);
    add(d.maxKey, static_cast<float>(d.maxValue));
    float score = 24.f;
    for (const sdk::HudScoreSide& s : d.sides) {
        add(s.scoreKey, score);
        score += 9.f;
    }
}

}  // namespace

std::optional<DemoScenario> parseDemoScenario(std::string_view name) {
    static constexpr std::pair<std::string_view, DemoScenario> table[] = {
        {"join", DemoScenario::Join},       {"lobby", DemoScenario::Lobby},   {"live", DemoScenario::Live},
        {"paused", DemoScenario::Paused},   {"stalled", DemoScenario::HostStalled},
        {"reconnecting", DemoScenario::Reconnecting}, {"lost", DemoScenario::Lost}, {"kicked", DemoScenario::Kicked},
        {"closed", DemoScenario::Closed}};
    for (const auto& [key, value] : table)
        if (key == name) return value;
    return std::nullopt;
}

const char* demoScenarioNames() { return "join, lobby, live, paused, stalled, reconnecting, lost, kicked, closed"; }

sdk::RobotConfig demoRobotConfig(const ClientServices& services) {
    sdk::RobotConfig config = swerveRobot("Demo Bot", 0.76f);
    const auto plugins = services.plugins().list();
    if (plugins.empty()) return config;
    const sdk::IGameModule* game = plugins.front();
    config.gamePluginId = game->metadata().id;
    const auto kinds = game->contributeZoneKinds();
    const sdk::Vec3 positions[] = {{0.48f, 0.f, 0.22f}, {0.2f, 0.f, 0.6f}};
    for (std::size_t i = 0; i < kinds.size() && i < 2; ++i) {
        sdk::ZoneConfig zone;
        zone.id = "zone" + std::to_string(i);
        zone.kind = kinds[i].kind;
        zone.local.position = positions[i];
        zone.sizeM = {0.30f, 0.60f, 0.40f};
        config.zones.push_back(std::move(zone));
    }
    return config;
}

struct DemoClientSession::Impl {
    Impl(const ClientServices& s, DemoScenario sc) : services(s), scenario(sc) {
        spec = std::make_shared<sdk::RobotConfig>(demoRobotConfig(services));
        pluginId = spec->gamePluginId;
        ownAppearance = std::make_shared<const wire::RobotAppearance>(mc::appearanceOf(*spec));
        for (const Scripted& o : kOthers) {
            const sdk::RobotConfig other = swerveRobot(o.name, o.sizeX);
            appearances.push_back(std::make_shared<const wire::RobotAppearance>(mc::appearanceOf(other)));
        }
        restPoses = wire::appearanceZoneRestPoses(*ownAppearance);
        zonePoses = restPoses;
        sdk::RobotStartPose placement;
        placement.xM = ownX;
        placement.yM = ownY;
        placement.headingRad = ownYaw;
        mc::PredictionWorldArgs args{*spec, &services.plugins(), services.drivetrains(), services.tireModels(), {}, placement,
                                     sdk::makeRobotTag(1)};
        if (auto world = mc::PredictionWorld::create(args)) {
            field = std::make_shared<const sdk::FieldDescription>(world.value()->field());
            fieldLength = field->lengthM > 1.f ? field->lengthM : fieldLength;
            fieldWidth = field->widthM > 1.f ? field->widthM : fieldWidth;
        }
        ownX = fieldLength - 2.2f;
        ownY = 0.5f * fieldWidth;
        joined = scenario != DemoScenario::Join;
        start = std::chrono::steady_clock::now();
        last = start;
        if (const auto plugins = services.plugins().list(); !plugins.empty()) hudDescriptors = plugins.front()->contributeHud();
    }

    bool moving() const { return scenario == DemoScenario::Live; }

    void advance() {
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::min(0.1f, std::chrono::duration<float>(now - last).count());
        last = now;
        if (!moving()) return;
        t += dt;
        sdk::ChassisSpeeds v = command;
        if (!commanded) v = {1.4f, 0.f, 0.55f};   // nobody is driving: circle so the demo is alive
        const float c = std::cos(ownYaw), s = std::sin(ownYaw);
        ownX += (c * v.vx - s * v.vy) * dt;
        ownY += (s * v.vx + c * v.vy) * dt;
        ownYaw = wrapPi(ownYaw + v.omega * dt);
        ownX = std::clamp(ownX, 0.5f, fieldLength - 0.5f);
        ownY = std::clamp(ownY, 0.5f, fieldWidth - 0.5f);
        const float speed = std::hypot(v.vx, v.vy);
        spin = wrapPi(spin + speed / 0.0508f * dt);
        if (speed > 0.05f) steer = std::atan2(v.vy, v.vx);
    }

    std::shared_ptr<const mc::MatchClientView> build() {
        advance();
        auto v = std::make_shared<mc::MatchClientView>();
        v->revision = ++revision;
        v->renderSimTimeUs = static_cast<std::int64_t>(t * 1e6);
        v->sessionId = 7;
        v->runEpoch = 1;
        v->connection = status();
        v->hasOwn = joined;
        const bool live = scenario == DemoScenario::Live;

        mc::OwnRobotView& own = v->own;
        own.id = 1;
        own.name = "You";
        own.station = sdk::proto::AllianceStation::Red1;
        own.appearance = ownAppearance;
        own.pose = {{ownX, ownY, 0.2f}, sdk::Quat::fromYaw(ownYaw)};
        const sdk::ChassisSpeeds v0 = commanded ? command : sdk::ChassisSpeeds{1.4f, 0.f, 0.55f};
        own.linearVelocity = moving() ? sdk::Vec3{std::cos(ownYaw) * v0.vx - std::sin(ownYaw) * v0.vy,
                                                  std::sin(ownYaw) * v0.vx + std::cos(ownYaw) * v0.vy, 0.f}
                                      : sdk::Vec3{};
        own.angularVelocity = {0.f, 0.f, moving() ? v0.omega : 0.f};
        own.wheels.assign(4, mc::WheelView{steer, spin});
        own.zoneFlags = flags;
        own.zonePoses = zonePoses;
        own.predicted = true;
        own.enabled = live;

        for (std::size_t i = 0; i < std::size(kOthers); ++i) {
            const Scripted& o = kOthers[i];
            mc::RemoteRobotView r;
            r.id = o.id;
            r.name = o.name;
            r.station = o.station;
            r.isHost = o.id == wire::kHostParticipantId;
            r.appearance = appearances[i];
            const float a = o.phase + t * o.speedMps / o.radius;
            r.pose = {{o.cx + o.radius * std::cos(a), o.cy + o.radius * std::sin(a), 0.2f}, sdk::Quat::fromYaw(a + kPi / 2.f)};
            r.linearVelocity = {-o.speedMps * std::sin(a), o.speedMps * std::cos(a), 0.f};
            const float wheelSpin = wrapPi(t * o.speedMps / 0.0508f);
            r.wheels.assign(4, mc::WheelView{0.f, wheelSpin});
            r.zoneFlags = (i == 1 && live && std::fmod(t, 4.f) < 2.f) ? 1u : 0u;
            v->remotes.push_back(std::move(r));
        }

        mc::EntityGroupView fuel;
        fuel.groupId = 0;
        std::uint32_t id = 1;
        for (int ix = 0; ix < 12; ++ix)
            for (int iy = 0; iy < 5; ++iy)
                fuel.entities.push_back({id++, 0.5f * fieldLength - 1.4f + 0.25f * static_cast<float>(ix),
                                         0.5f * fieldWidth - 0.5f + 0.25f * static_cast<float>(iy), 0.075f});
        v->entities.push_back(std::move(fuel));

        v->hud.state = scenario == DemoScenario::Lobby    ? wire::MatchState::Lobby
                       : scenario == DemoScenario::Paused ? wire::MatchState::Paused
                       : scenario == DemoScenario::HostStalled ? wire::MatchState::HostStalled
                                                               : wire::MatchState::Running;
        v->hud.enabled = live;
        v->hud.mode = sdk::proto::RobotMode::Teleop;
        v->hud.matchTimeS = scenario == DemoScenario::Lobby ? -1.f : 135.f - std::fmod(t + 20.f, 135.f);
        v->hud.phaseKey = "TELEOP";
        std::vector<std::pair<std::string, float>> keys;
        for (const sdk::HudWidgetDescriptor& d : hudDescriptors) collectKeys(d, keys);
        std::uint16_t keyId = 1;
        for (const auto& [key, value] : keys) v->hud.game.push_back({keyId++, key, value});

        mc::ClientDiagnostics& d = v->diagnostics;
        const double wobble = std::sin(t * 1.7);
        d.rttMs = 41.0 + 4.0 * wobble;
        d.rttMinMs = 33.0;
        d.jitterMs = 3.2 + 0.8 * wobble;
        d.snapshotJitterMs = 2.1;
        d.lossPermille = 8.0;
        d.snapshotRateHz = 30.0;
        d.snapshotAgeMs = 14.0 + 6.0 * std::fabs(wobble);
        d.bufferDelayMs = 78.0;
        d.clockSynced = true;
        d.predictionErrorM = 0.012 + 0.006 * std::fabs(wobble);
        d.predictionErrorP95M = 0.041;
        d.predictionErrorYawRad = 0.004;
        d.predictionAheadMs = 62.0;
        d.inputLeadTicks = 14;
        d.snapshotsReceived = revision * 3;
        d.reconciliations = revision / 2;
        d.snaps = 1;
        v->specClamps.clear();
        return v;
    }

    mc::ConnectionStatus status() const {
        mc::ConnectionStatus s;
        switch (scenario) {
            case DemoScenario::Join: break;
            case DemoScenario::Lobby: s.state = mc::ConnectionState::Lobby; break;
            case DemoScenario::Live: s.state = mc::ConnectionState::Live; break;
            case DemoScenario::Paused: s.state = mc::ConnectionState::Paused; break;
            case DemoScenario::HostStalled: s.state = mc::ConnectionState::HostStalled; break;
            case DemoScenario::Reconnecting:
                s.state = mc::ConnectionState::Connecting;
                s.reconnecting = true;
                s.reconnectAttempt = 2;
                break;
            case DemoScenario::Lost:
                s.state = mc::ConnectionState::Disconnected;
                s.reason = mc::DisconnectReason::TransportLost;
                s.detail = "The connection to the host was lost.";
                break;
            case DemoScenario::Kicked:
                s.state = mc::ConnectionState::Disconnected;
                s.reason = mc::DisconnectReason::Kicked;
                s.detail = "Removed by the host.";
                break;
            case DemoScenario::Closed:
                s.state = mc::ConnectionState::Disconnected;
                s.reason = mc::DisconnectReason::ServerClosing;
                break;
        }
        return s;
    }

    const ClientServices& services;
    DemoScenario scenario;
    std::mutex mutex;
    std::shared_ptr<sdk::RobotConfig> spec;
    std::string pluginId;
    std::shared_ptr<const wire::RobotAppearance> ownAppearance;
    std::vector<std::shared_ptr<const wire::RobotAppearance>> appearances;
    std::vector<sdk::Pose3> restPoses, zonePoses;
    std::shared_ptr<const sdk::FieldDescription> field;
    std::vector<sdk::HudWidgetDescriptor> hudDescriptors;
    float fieldLength = 16.54f, fieldWidth = 8.07f;
    float ownX = 14.f, ownY = 4.f, ownYaw = kPi, spin = 0.f, steer = 0.f;
    sdk::ChassisSpeeds command;
    bool commanded = false;
    std::uint32_t flags = 0;
    float t = 0.f;
    std::uint64_t revision = 0;
    bool joined = false;
    std::chrono::steady_clock::time_point start, last;
};

DemoClientSession::DemoClientSession(const ClientServices& services, DemoScenario scenario)
    : impl_(std::make_unique<Impl>(services, scenario)) {}

DemoClientSession::~DemoClientSession() = default;

void DemoClientSession::join(JoinPlan, sdk::RobotConfig) {
    std::lock_guard lock(impl_->mutex);
    impl_->joined = true;
    if (impl_->scenario == DemoScenario::Join) impl_->scenario = DemoScenario::Live;
}

void DemoClientSession::leave() {
    std::lock_guard lock(impl_->mutex);
    impl_->joined = false;
    impl_->scenario = DemoScenario::Join;
}

void DemoClientSession::reconnect() {
    std::lock_guard lock(impl_->mutex);
    impl_->scenario = DemoScenario::Live;
}

SessionState DemoClientSession::state() const {
    std::lock_guard lock(impl_->mutex);
    SessionState s;
    if (!impl_->joined) return s;
    s.phase = LinkPhase::Active;
    s.everJoined = true;
    s.status = impl_->status();
    return s;
}

std::shared_ptr<const mc::MatchClientView> DemoClientSession::view() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->joined ? impl_->build() : nullptr;
}

std::shared_ptr<const sdk::RobotConfig> DemoClientSession::acceptedSpec() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->joined ? impl_->spec : nullptr;
}

std::shared_ptr<const sdk::FieldDescription> DemoClientSession::fieldDescription() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->field;
}

std::string DemoClientSession::gamePluginId() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->pluginId;
}

sdk::Status DemoClientSession::driveRobotRelative(const sdk::ChassisSpeeds& speeds) {
    std::lock_guard lock(impl_->mutex);
    impl_->command = speeds;
    impl_->commanded = impl_->commanded || std::hypot(speeds.vx, speeds.vy) > 0.01f || std::fabs(speeds.omega) > 0.01f;
    return sdk::okStatus();
}

sdk::Status DemoClientSession::driveFieldRelative(const sdk::ChassisSpeeds& fieldSpeeds) {
    std::lock_guard lock(impl_->mutex);
    const float c = std::cos(impl_->ownYaw), s = std::sin(impl_->ownYaw);
    sdk::ChassisSpeeds robot{c * fieldSpeeds.vx + s * fieldSpeeds.vy, -s * fieldSpeeds.vx + c * fieldSpeeds.vy, fieldSpeeds.omega};
    impl_->command = robot;
    impl_->commanded = impl_->commanded || std::hypot(robot.vx, robot.vy) > 0.01f || std::fabs(robot.omega) > 0.01f;
    return sdk::okStatus();
}

void DemoClientSession::setZoneFlags(std::uint32_t flags) {
    std::lock_guard lock(impl_->mutex);
    impl_->flags = flags;
}

sdk::Status DemoClientSession::setZonePose(std::size_t zone, const sdk::Pose3& chassisPose) {
    std::lock_guard lock(impl_->mutex);
    if (zone >= impl_->zonePoses.size()) return sdk::makeError(sdk::ErrorCode::OutOfRange, "no such zone");
    impl_->zonePoses[zone] = chassisPose;
    return sdk::okStatus();
}

void DemoClientSession::clearZonePose(std::size_t zone) {
    std::lock_guard lock(impl_->mutex);
    if (zone < impl_->zonePoses.size()) impl_->zonePoses[zone] = impl_->restPoses[zone];
}

}  // namespace wpiemu::app::client
