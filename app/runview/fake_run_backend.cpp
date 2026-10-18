#include "fake_run_backend.h"

#include <QDateTime>

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::runview {
namespace {

constexpr double kDt = 0.005;
constexpr float kFieldL = 16.46f;
constexpr float kFieldW = 8.23f;
constexpr int kCapacity = 30;
constexpr float kBallR = 0.075f;
constexpr float kHubX[2] = {4.6f, 11.86f};
constexpr float kHubY = 4.115f;

const sdk::Quat kCylToY = sdk::Quat::fromAxisAngle({1.f, 0.f, 0.f}, -1.5707963f);

sdk::PrimitiveSpec box(std::string name, sdk::Vec3 pos, sdk::Vec3 size, std::string color) {
    sdk::PrimitiveSpec p;
    p.name = std::move(name);
    p.local.position = pos;
    p.sizeM = size;
    p.color = std::move(color);
    return p;
}

sdk::PrimitiveSpec cylinder(std::string name, sdk::Vec3 pos, sdk::Quat rot, float radius, float height, std::string color) {
    sdk::PrimitiveSpec p;
    p.name = std::move(name);
    p.kind = sdk::PrimitiveKind::Cylinder;
    p.local = sdk::Pose3(pos, rot);
    p.radiusM = radius;
    p.heightM = height;
    p.color = std::move(color);
    return p;
}

float approach(float value, float target, float maxDelta) { return value + std::clamp(target - value, -maxDelta, maxDelta); }

}  // namespace

sdk::RobotConfig FakeRunBackend::makeDemoConfig() {
    sdk::RobotConfig c;
    c.name = "Demo robot";
    c.chassis.sizeM = {0.76f, 0.76f, 0.15f};
    c.chassis.comHeightM = 0.2f;
    c.chassis.groundClearanceM = 0.05f;
    auto& prims = c.chassis.primitives;
    prims.push_back(box("frame", {0.f, 0.f, -0.11f}, {0.76f, 0.76f, 0.08f}, "#4b5563"));
    prims.push_back(box("bumper.front", {0.40f, 0.f, -0.10f}, {0.05f, 0.86f, 0.12f}, "#c0392b"));
    prims.push_back(box("bumper.back", {-0.40f, 0.f, -0.10f}, {0.05f, 0.86f, 0.12f}, "#c0392b"));
    prims.push_back(box("bumper.left", {0.f, 0.40f, -0.10f}, {0.76f, 0.05f, 0.12f}, "#c0392b"));
    prims.push_back(box("bumper.right", {0.f, -0.40f, -0.10f}, {0.76f, 0.05f, 0.12f}, "#c0392b"));
    prims.push_back(box("battery", {-0.18f, 0.f, -0.045f}, {0.18f, 0.17f, 0.07f}, "#1f2937"));

    const char* names[4] = {"fl", "fr", "bl", "br"};
    const float xs[4] = {0.3f, 0.3f, -0.3f, -0.3f};
    const float ys[4] = {0.3f, -0.3f, 0.3f, -0.3f};
    c.drivetrain.spec.kind = sdk::drivetrain_kind::kSwerve;
    for (int i = 0; i < 4; ++i) {
        sdk::WheelSpec w;
        w.name = names[i];
        w.kind = sdk::WheelKind::Swerve;
        w.positionM = {xs[i], ys[i], w.radiusM};
        w.steered = true;
        c.drivetrain.spec.wheels.push_back(w);
    }

    sdk::MechanismConfig intake;
    intake.id = "intake";
    intake.type = "mechanism";
    intake.label = "Intake";
    intake.mount.position = {0.38f, 0.f, -0.06f};
    intake.geometry.push_back(box("arm.bar", {0.17f, 0.f, 0.f}, {0.34f, 0.5f, 0.03f}, "#f59e0b"));
    intake.geometry.push_back(cylinder("arm.roller", {0.34f, 0.f, 0.f}, kCylToY, 0.04f, 0.5f, "#facc15"));
    c.mechanisms.push_back(std::move(intake));

    sdk::MechanismConfig turret;
    turret.id = "turret";
    turret.type = "mechanism";
    turret.label = "Turret";
    turret.mount.position = {-0.1f, 0.f, 0.f};
    turret.geometry.push_back(cylinder("base", {0.f, 0.f, 0.03f}, {}, 0.17f, 0.06f, "#6b7280"));
    turret.geometry.push_back(cylinder("yaw.cap", {0.f, 0.f, 0.1f}, {}, 0.12f, 0.1f, "#9ca3af"));
    turret.geometry.push_back(box("yaw.barrel", {0.2f, 0.f, 0.12f}, {0.36f, 0.1f, 0.1f}, "#d1d5db"));
    c.mechanisms.push_back(std::move(turret));

    sdk::MechanismConfig climber;
    climber.id = "climber";
    climber.type = "mechanism";
    climber.label = "Climber";
    climber.mount.position = {-0.3f, 0.f, 0.f};
    climber.geometry.push_back(box("rail", {0.f, 0.f, 0.25f}, {0.04f, 0.04f, 0.5f}, "#6b7280"));
    climber.geometry.push_back(box("carriage.plate", {0.05f, 0.f, 0.06f}, {0.06f, 0.3f, 0.08f}, "#34d399"));
    c.mechanisms.push_back(std::move(climber));

    auto& section = c.pluginSection("demo_game");
    sdk::Box3 intakeZone;
    intakeZone.pose.position = {0.78f, 0.f, -0.08f};
    intakeZone.sizeM = {0.5f, 0.7f, 0.2f};
    sdk::Box3 shooterZone;
    shooterZone.pose.position = {-0.1f, 0.f, 0.35f};
    shooterZone.sizeM = {0.5f, 0.5f, 0.5f};
    section.data["intake_zone"] = sdk::toValue(intakeZone);
    section.data["shooter_zone"] = sdk::toValue(shooterZone);
    return c;
}

sdk::FieldDescription FakeRunBackend::makeDemoField() {
    sdk::FieldDescription f;
    f.name = "Demo field";
    f.lengthM = kFieldL;
    f.widthM = kFieldW;
    auto add = [&](const char* name, sdk::FieldVisual::Kind kind, sdk::Vec3 pos, sdk::Vec3 size, const char* color) {
        sdk::FieldVisual v;
        v.name = name;
        v.kind = kind;
        v.pose.position = pos;
        v.sizeM = size;
        v.color = color;
        f.visuals.push_back(std::move(v));
    };
    using K = sdk::FieldVisual::Kind;
    add("zone.blue", K::Box, {1.5f, kFieldW / 2, 0.001f}, {3.0f, kFieldW, 0.002f}, "#1e3a8a");
    add("zone.red", K::Box, {kFieldL - 1.5f, kFieldW / 2, 0.001f}, {3.0f, kFieldW, 0.002f}, "#7f1d1d");
    add("line.center", K::Box, {kFieldL / 2, kFieldW / 2, 0.002f}, {0.06f, kFieldW, 0.002f}, "#e2e8f0");
    add("wall.south", K::Box, {kFieldL / 2, -0.05f, 0.3f}, {kFieldL + 0.2f, 0.1f, 0.6f}, "#475569");
    add("wall.north", K::Box, {kFieldL / 2, kFieldW + 0.05f, 0.3f}, {kFieldL + 0.2f, 0.1f, 0.6f}, "#475569");
    add("wall.blue", K::Box, {-0.05f, kFieldW / 2, 0.45f}, {0.1f, kFieldW, 0.9f}, "#334155");
    add("wall.red", K::Box, {kFieldL + 0.05f, kFieldW / 2, 0.45f}, {0.1f, kFieldW, 0.9f}, "#334155");
    add("hub.blue", K::Cylinder, {kHubX[0], kHubY, 0.9f}, {1.2f, 1.2f, 1.8f}, "#2563eb");
    add("hub.red", K::Cylinder, {kHubX[1], kHubY, 0.9f}, {1.2f, 1.2f, 1.8f}, "#dc2626");
    add("bump.a", K::Box, {kFieldL / 2 - 2.2f, 1.6f, 0.08f}, {1.4f, 1.4f, 0.16f}, "#64748b");
    add("bump.b", K::Box, {kFieldL / 2 + 2.2f, kFieldW - 1.6f, 0.08f}, {1.4f, 1.4f, 0.16f}, "#64748b");
    return f;
}

FakeRunBackend::FakeRunBackend(Options options, QObject* parent) : IRunBackend(parent), options_(options), timer_(this) {
    registerRunViewMetaTypes();
    scene_.config = std::make_shared<const sdk::RobotConfig>(makeDemoConfig());
    scene_.field = std::make_shared<const sdk::FieldDescription>(makeDemoField());
    scene_.gameName = QStringLiteral("Demo game");
    if (options_.withGameContent) {
        using sdk::HudAnchor;
        using sdk::HudWidgetKind;
        auto hud = [](const char* id, const char* title, HudWidgetKind kind, const char* key, HudAnchor anchor, int order) {
            sdk::HudWidgetDescriptor d;
            d.id = id;
            d.title = title;
            d.kind = kind;
            d.valueKey = key;
            d.anchor = anchor;
            d.order = order;
            return d;
        };
        auto fuel = hud("demo_game.fuel", "Fuel", HudWidgetKind::Meter, "demo.fuel.count", HudAnchor::BottomCenter, 0);
        fuel.maxKey = "demo.fuel.capacity";
        fuel.color = "#facc15";
        auto timer = hud("demo_game.match", "Match", HudWidgetKind::Timer, "demo.match_time", HudAnchor::TopCenter, 0);
        auto hub = hud("demo_game.hub", "Hub", HudWidgetKind::Badge, "demo.hub.active", HudAnchor::TopCenter, 1);
        auto score = hud("demo_game.score", "Score", HudWidgetKind::Counter, "demo.score", HudAnchor::TopRight, 0);
        score.unit = "pts";
        auto message = hud("demo_game.message", "Game data", HudWidgetKind::Text, "demo.message", HudAnchor::TopRight, 1);
        scene_.hud = {fuel, timer, hub, score, message};

        sdk::EditorPanelDescriptor zones;
        zones.id = "demo_game.zones";
        zones.title = "Zones";
        zones.scope = sdk::PanelScope::PluginConfig;
        zones.overlays.push_back({"intake_zone", "Intake zone", "#22c55e", true});
        zones.overlays.push_back({"shooter_zone", "Shooter zone", "#f43f5e", true});
        scene_.panels.push_back(std::move(zones));
    }
    ds_.dsAttached = true;
    resetWorld();

    timer_.setInterval(16);
    connect(&timer_, &QTimer::timeout, this, &FakeRunBackend::onTimer);
    if (options_.autoRun) timer_.start();
}

void FakeRunBackend::scatterBalls() {
    balls_.clear();
    if (!options_.withGameContent) return;
    std::uniform_real_distribution<float> rx(5.2f, 11.2f), ry(0.9f, 7.3f);
    while (static_cast<int>(balls_.size()) < options_.ballCount) {
        const sdk::Vec3 p{rx(rng_), ry(rng_), kBallR};
        bool nearHub = false;
        for (float hx : kHubX) nearHub = nearHub || std::hypot(p.x - hx, p.y - kHubY) < 0.9f;
        if (!nearHub) balls_.push_back({p, {}, false});
    }
}

void FakeRunBackend::resetWorld() {
    pos_ = {2.0f, 4.1f, 0.2f};
    yaw_ = 0.f;
    vel_ = {};
    omega_ = 0.f;
    armAngle_ = turretYaw_ = flywheel_ = carriage_ = 0.f;
    fuelHeld_ = options_.withGameContent ? 12 : 0;
    score_ = 0;
    shotTimer_ = 0.0;
    tick_ = 0;
    simUs_ = 0;
    rng_.seed(1);
    scatterBalls();
}

void FakeRunBackend::setStatus(RobotProcessState process, LinkState link, int exitCode, const QString& detail) {
    const bool live = process == RobotProcessState::Running && link == LinkState::Connected;
    if (!live) {
        programStarted_ = false;
        simulating_ = false;
    }
    const RunStatus next{process, link, exitCode, detail, programStarted_, simulating_};
    if (next == status_) return;
    status_ = next;
    emit statusChanged();
}

void FakeRunBackend::refreshFlags() { setStatus(status_.process, status_.link, status_.exitCode, status_.detail); }

void FakeRunBackend::beginTransition(int delayMs, std::function<void()> fn) {
    if (delayMs <= 0) {
        fn();
        return;
    }
    const int generation = transitionGeneration_;
    QTimer::singleShot(delayMs, this, [this, generation, fn = std::move(fn)] {
        if (generation == transitionGeneration_) fn();
    });
}

void FakeRunBackend::emitLog(sdk::LogLevel level, LogStream stream, const QString& source, const QString& text) {
    emit logEntry({QDateTime::currentMSecsSinceEpoch(), level, stream, source, text});
}

void FakeRunBackend::launchRobot() {
    if (status_.process == RobotProcessState::Starting || status_.process == RobotProcessState::Running) return;
    ++startCount_;
    ++transitionGeneration_;
    slot_.publish(nullptr);
    setStatus(RobotProcessState::Starting, LinkState::Disconnected);
    emitLog(sdk::LogLevel::Info, LogStream::Emulator, QStringLiteral("emulator"), QStringLiteral("Starting robot: gradlew simulateJava"));
    emitLog(sdk::LogLevel::Info, LogStream::Stdout, QStringLiteral("gradle"), QStringLiteral("> Task :compileJava UP-TO-DATE"));
    emitLog(sdk::LogLevel::Info, LogStream::Stdout, QStringLiteral("gradle"), QStringLiteral("> Task :simulateJava"));
    beginTransition(options_.startupDelayMs, [this] {
        setStatus(RobotProcessState::Running, LinkState::Connecting);
        emitLog(sdk::LogLevel::Info, LogStream::Stdout, QStringLiteral("robot"), QStringLiteral("********** Robot program starting **********"));
        beginTransition(options_.startupDelayMs, [this] {
            resetWorld();
            setStatus(RobotProcessState::Running, LinkState::Connected);
            programStarted_ = true;
            refreshFlags();
            emitLog(sdk::LogLevel::Info, LogStream::Emulator, QStringLiteral("extension"), QStringLiteral("Extension connected"));
            publish();
            beginIfReady();
        });
    });
}

void FakeRunBackend::startSimulation() {
    beginWhenReady_ = true;
    if (status_.process != RobotProcessState::Starting && status_.process != RobotProcessState::Running) launchRobot();
    beginIfReady();
}

void FakeRunBackend::beginIfReady() {
    if (!beginWhenReady_ || !programStarted_) return;
    beginWhenReady_ = false;
    if (simulating_) return;
    resetWorld();
    timeAccum_ = 0.0;
    simulating_ = true;
    refreshFlags();
    emitLog(sdk::LogLevel::Info, LogStream::Emulator, QStringLiteral("emulator"), QStringLiteral("Simulation started (lock-step, t = 0)"));
    publish();
}

void FakeRunBackend::stopSimulation() {
    beginWhenReady_ = false;
    if (!simulating_) return;
    ++stopCount_;
    simulating_ = false;
    refreshFlags();
    emitLog(sdk::LogLevel::Info, LogStream::Emulator, QStringLiteral("emulator"), QStringLiteral("Simulation stopped; the robot keeps running"));
}

void FakeRunBackend::restartRobot() {
    ++transitionGeneration_;
    const bool resume = simulating_ || beginWhenReady_;
    if (status_.process == RobotProcessState::Starting || status_.process == RobotProcessState::Running) {
        slot_.publish(nullptr);
        setStatus(RobotProcessState::Exited, LinkState::Disconnected, 0);
    }
    launchRobot();
    beginWhenReady_ = resume;
}

void FakeRunBackend::resetSimulation() {
    if (status_.process != RobotProcessState::Running) return;
    ++resetCount_;
    resetWorld();
    emitLog(sdk::LogLevel::Info, LogStream::Emulator, QStringLiteral("emulator"), QStringLiteral("Simulation reset (epoch advanced)"));
    publish();
}

void FakeRunBackend::setPaused(bool paused) {
    paused_ = paused;
    pendingSteps_ = 0;
    if (status_.process == RobotProcessState::Running) publish();
}

void FakeRunBackend::stepTicks(int ticks) {
    ++stepRequests_;
    if (paused_) pendingSteps_ += std::max(1, ticks);
}

void FakeRunBackend::setTimeScale(double scale) { timeScale_ = std::clamp(scale, 0.05, 16.0); }

void FakeRunBackend::setDriverStation(const sdk::proto::DsState& state) {
    if (state.enabled != ds_.enabled)
        emitLog(sdk::LogLevel::Info, LogStream::Stdout, QStringLiteral("robot"),
                state.enabled ? QStringLiteral("Robot enabled") : QStringLiteral("Robot disabled"));
    ds_ = state;
}

void FakeRunBackend::simulateCrash(int exitCode, const QString& detail) {
    ++transitionGeneration_;
    slot_.publish(nullptr);
    emitLog(sdk::LogLevel::Error, LogStream::Stderr, QStringLiteral("robot"),
            QStringLiteral("Exception in thread \"main\" java.lang.NullPointerException"));
    emitLog(sdk::LogLevel::Error, LogStream::Stderr, QStringLiteral("robot"), QStringLiteral("    at frc.robot.Robot.robotPeriodic(Robot.java:42)"));
    setStatus(RobotProcessState::Crashed, LinkState::Disconnected, exitCode, detail);
}

void FakeRunBackend::simulateHandshakeTimeout() {
    ++transitionGeneration_;
    setStatus(RobotProcessState::Running, LinkState::HandshakeTimeout);
}

void FakeRunBackend::simulateProtocolMismatch(const QString& detail) {
    ++transitionGeneration_;
    setStatus(RobotProcessState::Running, LinkState::ProtocolMismatch, 0, detail);
}

void FakeRunBackend::simulateStall() { setStatus(RobotProcessState::Running, LinkState::Stalled); }

void FakeRunBackend::simulateFailedToStart(const QString& detail) {
    ++transitionGeneration_;
    setStatus(RobotProcessState::FailedToStart, LinkState::Disconnected, 1, detail);
}

void FakeRunBackend::setLinkSettled() {
    ++transitionGeneration_;
    resetWorld();
    setStatus(RobotProcessState::Running, LinkState::Connected);
    programStarted_ = true;
    simulating_ = true;
    refreshFlags();
    publish();
}

void FakeRunBackend::onTimer() {
    if (!simulating_ || status_.process != RobotProcessState::Running || status_.link != LinkState::Connected) return;
    int steps = 0;
    if (paused_) {
        steps = pendingSteps_;
        pendingSteps_ = 0;
    } else {
        timeAccum_ += 0.016 * timeScale_;
        steps = static_cast<int>(timeAccum_ / kDt);
        timeAccum_ -= steps * kDt;
    }
    if (steps <= 0) return;
    for (int i = 0; i < steps; ++i) stepWorld();
    publish();
}

void FakeRunBackend::tickOnce() {
    if (!simulating_ || status_.process != RobotProcessState::Running || status_.link != LinkState::Connected) return;
    stepWorld();
    publish();
}

void FakeRunBackend::stepWorld() {
    const float dt = static_cast<float>(kDt);
    ++tick_;
    simUs_ += 5000;

    const auto& js = ds_.joysticks[0];
    const bool enabled = ds_.enabled && !ds_.eStop;
    auto axis = [&](std::size_t i) { return i < js.axes.size() ? js.axes[i] : 0.f; };
    const bool buttonA = enabled && (js.buttons & 1u) != 0;
    const bool buttonB = enabled && (js.buttons & 2u) != 0;

    float targetVx = 0.f, targetVy = 0.f, targetW = 0.f;
    if (enabled && ds_.mode == sdk::proto::RobotMode::Autonomous) {
        targetVx = 1.6f;
        targetW = 0.7f;
    } else if (enabled) {
        targetVx = -axis(1) * 3.5f;
        targetVy = -axis(0) * 3.5f;
        targetW = -axis(4) * 6.f;
    }
    vel_.x = approach(vel_.x, targetVx, 9.f * dt);
    vel_.y = approach(vel_.y, targetVy, 9.f * dt);
    omega_ = approach(omega_, targetW, 20.f * dt);

    yaw_ = static_cast<float>(std::remainder(static_cast<double>(yaw_ + omega_ * dt), sdk::kTwoPi));
    const float c = std::cos(yaw_), s = std::sin(yaw_);
    pos_.x += (vel_.x * c - vel_.y * s) * dt;
    pos_.y += (vel_.x * s + vel_.y * c) * dt;
    const float margin = 0.55f;
    if (pos_.x < margin || pos_.x > kFieldL - margin) { pos_.x = std::clamp(pos_.x, margin, kFieldL - margin); vel_.x = 0.f; }
    if (pos_.y < margin || pos_.y > kFieldW - margin) { pos_.y = std::clamp(pos_.y, margin, kFieldW - margin); vel_.y = 0.f; }

    static constexpr float kWx[4] = {0.3f, 0.3f, -0.3f, -0.3f};
    static constexpr float kWy[4] = {0.3f, -0.3f, 0.3f, -0.3f};
    for (int i = 0; i < 4; ++i) {
        const float wx = vel_.x - omega_ * kWy[i];
        const float wy = vel_.y + omega_ * kWx[i];
        const float speed = std::hypot(wx, wy);
        if (speed > 0.05f) steer_[i] = std::atan2(wy, wx);
        spin_[i] += (speed * (std::cos(steer_[i]) * wx + std::sin(steer_[i]) * wy >= 0.f ? 1.f : -1.f)) / 0.0508f * dt;
    }

    armAngle_ = approach(armAngle_, buttonA ? 1.45f : 0.f, 3.f * dt);
    carriage_ = approach(carriage_, enabled ? std::clamp(axis(3), 0.f, 1.f) * 0.5f : carriage_, 0.4f * dt);
    turretYaw_ = enabled ? 0.9f * std::sin(static_cast<float>(sdk::usToSeconds(simUs_)) * 0.8f) : turretYaw_;
    flywheel_ = approach(flywheel_, buttonB ? 320.f : 0.f, 240.f * dt);

    if (options_.withGameContent) {
        // Intake: absorb balls inside the zone while deployed.
        if (armAngle_ > 1.2f && fuelHeld_ < kCapacity) {
            for (std::size_t i = 0; i < balls_.size() && fuelHeld_ < kCapacity; ++i) {
                if (balls_[i].flying) continue;
                const sdk::Vec3 rel = sdk::Pose3{{pos_.x, pos_.y, 0.f}, sdk::Quat::fromYaw(yaw_)}.inverse().transformPoint(balls_[i].pos);
                if (rel.x > 0.53f && rel.x < 1.03f && std::fabs(rel.y) < 0.35f) {
                    balls_[i] = balls_.back();
                    balls_.pop_back();
                    ++fuelHeld_;
                    --i;
                }
            }
        }
        // Shooter: fire a ball every 0.2 s while the flywheel is at speed.
        if (flywheel_ > 250.f && fuelHeld_ > 0) {
            shotTimer_ += kDt;
            if (shotTimer_ >= 0.2) {
                shotTimer_ = 0.0;
                --fuelHeld_;
                const float heading = yaw_ + turretYaw_;
                const sdk::Vec3 muzzle = sdk::Pose3{{pos_.x, pos_.y, 0.f}, sdk::Quat::fromYaw(yaw_)}.transformPoint({-0.1f + 0.4f * std::cos(turretYaw_), 0.4f * std::sin(turretYaw_), 0.f});
                Ball b;
                b.pos = {muzzle.x, muzzle.y, 0.75f};
                const float speedXY = 6.f;
                b.vel = {std::cos(heading) * speedXY + (vel_.x * c - vel_.y * s), std::sin(heading) * speedXY + (vel_.x * s + vel_.y * c), 5.f};
                b.flying = true;
                balls_.push_back(b);
            }
        } else {
            shotTimer_ = 0.0;
        }
        // Projectiles.
        for (std::size_t i = 0; i < balls_.size(); ++i) {
            Ball& b = balls_[i];
            if (!b.flying) continue;
            b.vel.z -= 9.81f * dt;
            b.pos += b.vel * dt;
            for (float hx : kHubX) {
                if (std::hypot(b.pos.x - hx, b.pos.y - kHubY) < 0.6f && b.pos.z < 1.9f && b.pos.z > 1.7f && b.vel.z < 0.f) {
                    ++score_;
                    balls_[i] = balls_.back();
                    balls_.pop_back();
                    --i;
                    break;
                }
            }
            if (i < balls_.size() && balls_[i].flying && balls_[i].pos.z <= kBallR) {
                balls_[i].pos.z = kBallR;
                balls_[i].vel = {};
                balls_[i].flying = false;
            }
        }
    }

    logTimer_ += kDt;
    if (logTimer_ >= 3.0) {
        logTimer_ = 0.0;
        if (enabled) {
            emitLog(sdk::LogLevel::Info, LogStream::Stdout, QStringLiteral("robot"),
                    QStringLiteral("Drive: vx=%1 vy=%2").arg(static_cast<double>(vel_.x), 0, 'f', 2).arg(static_cast<double>(vel_.y), 0, 'f', 2));
        } else {
            emitLog(sdk::LogLevel::Warn, LogStream::Stdout, QStringLiteral("robot"), QStringLiteral("Warning at frc.robot.Robot: robot is disabled"));
        }
    }
}

void FakeRunBackend::publish() {
    sdk::SnapshotBuilder builder;
    sdk::SimSnapshotData& d = builder.data();
    d.tick = tick_;
    d.simTimeUs = simUs_;
    d.paused = paused_;
    d.robotConnected = status_.link == LinkState::Connected;
    d.chassis.name = "chassis";
    d.chassis.pose = {pos_, sdk::Quat::fromYaw(yaw_)};
    d.chassis.linearVelocity = {vel_.x * std::cos(yaw_) - vel_.y * std::sin(yaw_), vel_.x * std::sin(yaw_) + vel_.y * std::cos(yaw_), 0.f};
    d.chassis.angularVelocity = {0.f, 0.f, omega_};

    static constexpr const char* kNames[4] = {"fl", "fr", "bl", "br"};
    static constexpr float kWx[4] = {0.3f, 0.3f, -0.3f, -0.3f};
    static constexpr float kWy[4] = {0.3f, -0.3f, 0.3f, -0.3f};
    for (int i = 0; i < 4; ++i) {
        sdk::WheelSample w;
        w.name = kNames[i];
        w.contactPoint = d.chassis.pose.transformPoint({kWx[i], kWy[i], -0.2f});
        w.contactPoint.z = 0.f;
        w.steerAngleRad = steer_[i];
        w.spinAngleRad = spin_[i];
        w.normalLoadN = 120.f;
        d.wheels.push_back(std::move(w));
    }

    auto armPose = [&] { return sdk::Pose3{{}, sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, -armAngle_)}; };
    sdk::MechanismState intake;
    intake.id = "intake";
    intake.type = "mechanism";
    intake.active = armAngle_ > 1.2f;
    intake.links.push_back({"arm", armPose()});
    intake.values = {{"deploy angle", static_cast<double>(armAngle_), "rad"}, {"roller", intake.active ? 420.0 : 0.0, "rad/s"}};
    d.mechanisms.push_back(std::move(intake));

    sdk::MechanismState turret;
    turret.id = "turret";
    turret.type = "mechanism";
    turret.active = flywheel_ > 250.f;
    turret.links.push_back({"yaw", sdk::Pose3{{}, sdk::Quat::fromYaw(turretYaw_)}});
    turret.values = {{"yaw", static_cast<double>(turretYaw_), "rad"}, {"flywheel", static_cast<double>(flywheel_), "rad/s"}};
    d.mechanisms.push_back(std::move(turret));

    sdk::MechanismState climber;
    climber.id = "climber";
    climber.type = "mechanism";
    climber.active = carriage_ > 0.01f;
    climber.links.push_back({"carriage", sdk::Pose3{{0.f, 0.f, carriage_}, {}}});
    climber.values = {{"height", static_cast<double>(carriage_), "m"}};
    d.mechanisms.push_back(std::move(climber));

    if (options_.withGameContent) {
        sdk::EntityGroupSample fuel;
        fuel.group = "fuel";
        fuel.radiusM = kBallR;
        fuel.color = "#ffd000";
        fuel.positions.reserve(balls_.size());
        for (const auto& b : balls_) fuel.positions.push_back(b.pos);
        d.entityGroups.push_back(std::move(fuel));
        d.hud.push_back({"demo.fuel.count", fuelHeld_});
        d.hud.push_back({"demo.fuel.capacity", kCapacity});
        d.hud.push_back({"demo.hub.active", static_cast<int>(sdk::usToSeconds(simUs_) / 10.0) % 2 == 0});
        d.hud.push_back({"demo.score", score_});
        if (ds_.matchTimeS >= 0.0) d.hud.push_back({"demo.match_time", ds_.matchTimeS});
        d.hud.push_back({"demo.message", ds_.gameSpecificMessage.empty() ? std::string("—") : ds_.gameSpecificMessage});
    }

    d.batteryVoltageV = 12.6 - 0.9 * (std::hypot(static_cast<double>(vel_.x), static_cast<double>(vel_.y)) / 3.5) - (flywheel_ > 1.f ? 0.5 : 0.0);
    d.driverStation = ds_;
    d.physicsStats.bodies = 2 + static_cast<int>(balls_.size());
    d.physicsStats.awakeBodies = 2;
    d.physicsStats.contacts = 4;
    d.physicsStats.lastStepMs = 0.35;
    slot_.publish(builder.build());
}

}  // namespace wpiemu::app::runview
