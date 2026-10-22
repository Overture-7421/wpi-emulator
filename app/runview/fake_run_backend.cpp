#include "fake_run_backend.h"

#include <QDateTime>

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/physics/physics_types.h"
#include "wpiemu/sdk/physics/wheel_layout.h"
#include "wpiemu/sdk/testing/demo_game.h"

namespace wpiemu::app::runview {

namespace demo = sdk::testing::demo_game;

namespace {

constexpr double kDt = sdk::kDefaultStepSeconds;
constexpr int kCapacity = 30;
constexpr float kBallR = 0.075f;

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

    c.drivetrain.spec.kind = sdk::drivetrain_kind::kSwerve;
    c.drivetrain.spec.wheels = sdk::swerveWheels({}, 0.3f, 0.3f);

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

    sdk::SensorConfig hopper;
    hopper.id = "hopper_sensor";
    hopper.label = "Hopper sensor";
    hopper.kind = sdk::SensorKind::Distance;
    hopper.source = demo::kFuelCount;
    hopper.properties["threshold"] = 6.0;
    hopper.properties["nearM"] = 0.1;
    hopper.properties["farM"] = 1.0;
    c.sensors.push_back(std::move(hopper));

    sdk::SensorConfig feed;
    feed.id = "feed_sensor";
    feed.label = "Feed sensor";
    feed.kind = sdk::SensorKind::BeamBreak;
    feed.source = demo::kFuelCount;
    feed.properties["threshold"] = 1.0;
    c.sensors.push_back(std::move(feed));

    sdk::ZoneConfig pickup;
    pickup.id = "pickup";
    pickup.kind = demo::kPickupZone;
    pickup.label = "Intake zone";
    pickup.attachment = {"intake", sdk::kRootLink};
    pickup.local.position = {0.4f, 0.f, -0.02f};
    pickup.sizeM = {0.5f, 0.7f, 0.2f};
    c.zones.push_back(std::move(pickup));

    sdk::ZoneConfig shooter;
    shooter.id = "shooter";
    shooter.kind = demo::kShooterZone;
    shooter.label = "Shooter zone";
    shooter.attachment = {"turret", "yaw"};
    shooter.local.position = {0.38f, 0.f, 0.12f};
    c.zones.push_back(std::move(shooter));

    sdk::CameraConfig rear;
    rear.id = "cam_rear";
    rear.label = "Rear Limelight";
    rear.name = "rear";
    rear.kind = sdk::CameraKind::Limelight;
    rear.local = {{-0.38f, 0.f, 0.1f},
                  sdk::Quat::fromYaw(static_cast<float>(sdk::kPi)) * sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, -0.3f)};
    rear.maxRangeM = 5.0;
    c.cameras.push_back(std::move(rear));

    sdk::CameraConfig turretCam;
    turretCam.id = "cam_turret";
    turretCam.label = "Turret camera";
    turretCam.name = "turret";
    turretCam.kind = sdk::CameraKind::PhotonVision;
    turretCam.attachment = {"turret", "yaw"};
    turretCam.local = {{0.1f, 0.f, 0.22f}, sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, -0.15f)};
    turretCam.hFovDeg = 70.0;
    turretCam.vFovDeg = 45.0;
    turretCam.maxRangeM = 6.0;
    c.cameras.push_back(std::move(turretCam));
    c.startPositions = {{"blue_left", "Blue left", sdk::Alliance::Blue, {2.5f, 6.5f, 0.f}},
                        {"blue_right", "Blue right", sdk::Alliance::Blue, {2.5f, 1.7f, 0.f}},
                        {"red_left", "Red left", sdk::Alliance::Red, {demo::kFieldLengthM - 2.5f, 1.7f, sdk::kPiF}}};
    return c;
}

FakeRunBackend::FakeRunBackend(Options options, QObject* parent) : IRuntimeHost(parent), options_(options), timer_(this) {
    scene_.config = std::make_shared<const sdk::RobotConfig>(makeDemoConfig());
    scene_.field = std::make_shared<const sdk::FieldDescription>(demo::field());
    scene_.gameName = QString::fromUtf8(demo::kDisplayName);
    scene_.tags = demo::aprilTags();
    if (options_.withGameContent) {
        scene_.hud = demo::hud();
        scene_.zoneKinds = demo::zoneKinds();
        scene_.gameValues = demo::gameValues();
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
        for (float hx : demo::kHubXM) nearHub = nearHub || std::hypot(p.x - hx, p.y - demo::kHubYM) < 0.9f;
        if (!nearHub) balls_.push_back({p, {}, false});
    }
}

void FakeRunBackend::resetWorld() {
    pos_ = {2.0f, 4.1f, 0.2f};
    yaw_ = 0.f;
    if (const sdk::StartPosition* p = startSelection_.find(scene_.config.get())) {
        pos_.x = p->pose.xM;
        pos_.y = p->pose.yM;
        yaw_ = p->pose.headingRad;
    }
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

void FakeRunBackend::setStatus(sdk::ProcessState process, sdk::LinkState link, int exitCode, const QString& detail) {
    if (!sdk::isLive(process, link)) {
        programStarted_ = false;
        simulating_ = false;
    }
    HostStatus next;
    next.process = process;
    next.link = link;
    next.exitCode = exitCode;
    next.detail = detail;
    next.programStarted = programStarted_;
    next.simulating = simulating_;
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

void FakeRunBackend::emitLog(sdk::LogLevel level, sdk::LogStream stream, const QString& source, const QString& text) {
    emit logLine({QDateTime::currentMSecsSinceEpoch(), level, stream, source, text});
}

void FakeRunBackend::launch() {
    if (status_.process == sdk::ProcessState::Starting || status_.process == sdk::ProcessState::Running) return;
    ++startCount_;
    ++transitionGeneration_;
    slot_.publish(nullptr);
    setStatus(sdk::ProcessState::Starting, sdk::LinkState::Disconnected);
    emitLog(sdk::LogLevel::Info, sdk::LogStream::Emulator, QStringLiteral("emulator"), QStringLiteral("Starting robot: gradlew simulateJava"));
    emitLog(sdk::LogLevel::Info, sdk::LogStream::Stdout, QStringLiteral("gradle"), QStringLiteral("> Task :compileJava UP-TO-DATE"));
    emitLog(sdk::LogLevel::Info, sdk::LogStream::Stdout, QStringLiteral("gradle"), QStringLiteral("> Task :simulateJava"));
    beginTransition(options_.startupDelayMs, [this] {
        setStatus(sdk::ProcessState::Running, sdk::LinkState::Connecting);
        emitLog(sdk::LogLevel::Info, sdk::LogStream::Stdout, QStringLiteral("robot"), QStringLiteral("********** Robot program starting **********"));
        beginTransition(options_.startupDelayMs, [this] {
            resetWorld();
            setStatus(sdk::ProcessState::Running, sdk::LinkState::Connected);
            programStarted_ = true;
            refreshFlags();
            emitLog(sdk::LogLevel::Info, sdk::LogStream::Emulator, QStringLiteral("extension"), QStringLiteral("Extension connected"));
            publish();
            beginIfReady();
        });
    });
}

void FakeRunBackend::startSimulation() {
    beginIntent_.request();
    if (status_.process != sdk::ProcessState::Starting && status_.process != sdk::ProcessState::Running) launch();
    beginIfReady();
}

void FakeRunBackend::beginIfReady() {
    if (!beginIntent_.takeWhenReady(programStarted_) || simulating_) return;
    resetWorld();
    timeAccum_ = 0.0;
    simulating_ = true;
    refreshFlags();
    emitLog(sdk::LogLevel::Info, sdk::LogStream::Emulator, QStringLiteral("emulator"), QStringLiteral("Simulation started (lock-step, t = 0)"));
    publish();
}

void FakeRunBackend::stopSimulation() {
    beginIntent_.cancel();
    if (!simulating_) return;
    simulating_ = false;
    refreshFlags();
    emitLog(sdk::LogLevel::Info, sdk::LogStream::Emulator, QStringLiteral("emulator"), QStringLiteral("Simulation stopped; the robot keeps running"));
}

void FakeRunBackend::stop() {
    ++transitionGeneration_;
    beginIntent_.cancel();
    if (status_.process == sdk::ProcessState::Starting || status_.process == sdk::ProcessState::Running) {
        slot_.publish(nullptr);
        setStatus(sdk::ProcessState::Exited, sdk::LinkState::Disconnected, 0);
    }
}

void FakeRunBackend::restart() {
    const bool resume = simulating_ || beginIntent_.pending();
    stop();
    launch();
    if (resume) beginIntent_.request();
}

void FakeRunBackend::resetSimulation() {
    if (status_.process != sdk::ProcessState::Running) return;
    ++resetCount_;
    resetWorld();
    emitLog(sdk::LogLevel::Info, sdk::LogStream::Emulator, QStringLiteral("emulator"), QStringLiteral("Simulation reset (epoch advanced)"));
    publish();
}

void FakeRunBackend::setStartPosition(const QString& id) {
    if (startSelection_.select(scene_.config.get(), id)) emit startPositionChanged();
}

void FakeRunBackend::setPaused(bool paused) {
    paused_ = paused;
    pendingSteps_ = 0;
    if (status_.process == sdk::ProcessState::Running) publish();
}

void FakeRunBackend::stepTicks(int ticks) {
    if (paused_) pendingSteps_ += std::max(1, ticks);
}

void FakeRunBackend::setTimeScale(double scale) { timeScale_ = std::clamp(scale, 0.05, 16.0); }

void FakeRunBackend::setDriverStation(const sdk::proto::DsState& state) {
    if (state.enabled != ds_.enabled)
        emitLog(sdk::LogLevel::Info, sdk::LogStream::Stdout, QStringLiteral("robot"),
                state.enabled ? QStringLiteral("Robot enabled") : QStringLiteral("Robot disabled"));
    ds_ = state;
}

void FakeRunBackend::simulateCrash(int exitCode, const QString& detail) {
    ++transitionGeneration_;
    slot_.publish(nullptr);
    emitLog(sdk::LogLevel::Error, sdk::LogStream::Stderr, QStringLiteral("robot"),
            QStringLiteral("Exception in thread \"main\" java.lang.NullPointerException"));
    emitLog(sdk::LogLevel::Error, sdk::LogStream::Stderr, QStringLiteral("robot"), QStringLiteral("    at frc.robot.Robot.robotPeriodic(Robot.java:42)"));
    setStatus(sdk::ProcessState::Crashed, sdk::LinkState::Disconnected, exitCode, detail);
}

void FakeRunBackend::simulateHandshakeTimeout() {
    ++transitionGeneration_;
    setStatus(sdk::ProcessState::Running, sdk::LinkState::HandshakeTimeout);
}

void FakeRunBackend::simulateProtocolMismatch(const QString& detail) {
    ++transitionGeneration_;
    setStatus(sdk::ProcessState::Running, sdk::LinkState::ProtocolMismatch, 0, detail);
}

void FakeRunBackend::simulateStall() { setStatus(sdk::ProcessState::Running, sdk::LinkState::Stalled); }

void FakeRunBackend::simulateFailedToStart(const QString& detail) {
    ++transitionGeneration_;
    setStatus(sdk::ProcessState::FailedToStart, sdk::LinkState::Disconnected, 1, detail);
}

void FakeRunBackend::setLinkSettled() {
    ++transitionGeneration_;
    resetWorld();
    setStatus(sdk::ProcessState::Running, sdk::LinkState::Connected);
    programStarted_ = true;
    simulating_ = true;
    refreshFlags();
    publish();
}

void FakeRunBackend::setProjectDefaults(const QString& projectPath, sdk::proto::AllianceStation station) {
    scene_.projectPath = projectPath;
    scene_.defaultStation = station;
    emit sceneChanged();
}

void FakeRunBackend::onTimer() {
    if (!simulating_ || !status_.live()) return;
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
    if (!simulating_ || !status_.live()) return;
    stepWorld();
    publish();
}

void FakeRunBackend::stepWorld() {
    const float dt = static_cast<float>(kDt);
    ++tick_;
    simUs_ += sdk::kDefaultStepUs;

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

    yaw_ = static_cast<float>(sdk::wrapAngleRad(static_cast<double>(yaw_ + omega_ * dt)));
    const float c = std::cos(yaw_), s = std::sin(yaw_);
    pos_.x += (vel_.x * c - vel_.y * s) * dt;
    pos_.y += (vel_.x * s + vel_.y * c) * dt;
    const float margin = 0.55f;
    if (pos_.x < margin || pos_.x > demo::kFieldLengthM - margin) { pos_.x = std::clamp(pos_.x, margin, demo::kFieldLengthM - margin); vel_.x = 0.f; }
    if (pos_.y < margin || pos_.y > demo::kFieldWidthM - margin) { pos_.y = std::clamp(pos_.y, margin, demo::kFieldWidthM - margin); vel_.y = 0.f; }

    const std::vector<sdk::WheelSpec>& wheels = scene_.config->drivetrain.spec.wheels;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const float wx = vel_.x - omega_ * wheels[i].positionM.y;
        const float wy = vel_.y + omega_ * wheels[i].positionM.x;
        const float speed = std::hypot(wx, wy);
        if (speed > 0.05f) steer_[i] = std::atan2(wy, wx);
        spin_[i] += (speed * (std::cos(steer_[i]) * wx + std::sin(steer_[i]) * wy >= 0.f ? 1.f : -1.f)) / static_cast<float>(sdk::kDefaultWheelRadiusM) * dt;
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
            b.vel.z -= sdk::kDefaultGravityMps2 * dt;
            b.pos += b.vel * dt;
            for (float hx : demo::kHubXM) {
                if (std::hypot(b.pos.x - hx, b.pos.y - demo::kHubYM) < 0.6f && b.pos.z < 1.9f && b.pos.z > 1.7f && b.vel.z < 0.f) {
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
            emitLog(sdk::LogLevel::Info, sdk::LogStream::Stdout, QStringLiteral("robot"),
                    QStringLiteral("Drive: vx=%1 vy=%2").arg(static_cast<double>(vel_.x), 0, 'f', 2).arg(static_cast<double>(vel_.y), 0, 'f', 2));
        } else {
            emitLog(sdk::LogLevel::Warn, sdk::LogStream::Stdout, QStringLiteral("robot"), QStringLiteral("Warning at frc.robot.Robot: robot is disabled"));
        }
    }
}

void FakeRunBackend::publish() {
    sdk::SnapshotBuilder builder;
    sdk::SimSnapshotData& d = builder.data();
    d.tick = tick_;
    d.simTimeUs = simUs_;
    d.paused = paused_;
    d.robotConnected = status_.link == sdk::LinkState::Connected;
    d.chassis.name = "chassis";
    d.chassis.pose = {pos_, sdk::Quat::fromYaw(yaw_)};
    d.chassis.linearVelocity = {vel_.x * std::cos(yaw_) - vel_.y * std::sin(yaw_), vel_.x * std::sin(yaw_) + vel_.y * std::cos(yaw_), 0.f};
    d.chassis.angularVelocity = {0.f, 0.f, omega_};

    const std::vector<sdk::WheelSpec>& wheels = scene_.config->drivetrain.spec.wheels;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        sdk::WheelSample w;
        w.name = wheels[i].name;
        w.contactPoint = d.chassis.pose.transformPoint({wheels[i].positionM.x, wheels[i].positionM.y, -0.2f});
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

    // Cameras: the same simple model as the real simulation (range, field of view, facing), on the live link poses.
    const sdk::Pose3 turretFrame = d.chassis.pose * sdk::Pose3{{-0.1f, 0.f, 0.f}, {}} * sdk::Pose3{{}, sdk::Quat::fromYaw(turretYaw_)};
    for (const auto& cam : scene_.config->cameras) {
        const sdk::Pose3 frame = cam.attachment.onChassis() ? d.chassis.pose : turretFrame;
        sdk::CameraState state;
        state.id = cam.id;
        state.worldPose = frame * cam.local;
        const sdk::Pose3 inverse = state.worldPose.inverse();
        const float tanH = std::tan(static_cast<float>(sdk::degToRad(cam.hFovDeg)) * 0.5f);
        const float tanV = std::tan(static_cast<float>(sdk::degToRad(cam.vFovDeg)) * 0.5f);
        const float minFacing = std::cos(static_cast<float>(sdk::degToRad(cam.maxIncidenceDeg)));
        for (const auto& tag : scene_.tags.tags) {
            const sdk::Vec3 local = inverse.transformPoint(tag.pose.position);
            if (local.x <= 0.05f || sdk::length(local) > static_cast<float>(cam.maxRangeM)) continue;
            if (std::fabs(local.y) > local.x * tanH || std::fabs(local.z) > local.x * tanV) continue;
            const sdk::Vec3 toCamera = sdk::normalized(state.worldPose.position - tag.pose.position);
            if (sdk::dot(tag.pose.rotation.rotate({1.f, 0.f, 0.f}), toCamera) < minFacing) continue;
            state.visibleTagIds.push_back(tag.id);
        }
        state.tagsVisible = !state.visibleTagIds.empty();
        d.cameras.push_back(std::move(state));
    }

    if (options_.withGameContent) {
        d.zones.push_back({"pickup", armAngle_ > 1.2f});
        d.zones.push_back({"shooter", flywheel_ > 250.f});
        d.sensors.push_back({"hopper_sensor", static_cast<double>(fuelHeld_), fuelHeld_ >= 6 ? 0.1 : 1.0});
        d.sensors.push_back({"feed_sensor", static_cast<double>(fuelHeld_), fuelHeld_ >= 1 ? 1.0 : 0.0});
        sdk::EntityGroupSample fuel;
        fuel.group = "fuel";
        fuel.radiusM = kBallR;
        fuel.color = "#ffd000";
        fuel.positions.reserve(balls_.size());
        for (const auto& b : balls_) fuel.positions.push_back(b.pos);
        d.entityGroups.push_back(std::move(fuel));
        d.hud.push_back({demo::kFuelCount, fuelHeld_});
        d.hud.push_back({demo::kFuelCapacity, kCapacity});
        d.hud.push_back({demo::kHubActive, static_cast<int>(sdk::usToSeconds(simUs_) / 10.0) % 2 == 0});
        d.hud.push_back({demo::kScoreBlue, score_});
        d.hud.push_back({demo::kScoreRed, score_ / 2 + 3});
        if (ds_.matchTimeS >= 0.0) {
            d.hud.push_back({demo::kMatchTime, ds_.matchTimeS});
            d.hud.push_back({demo::kPhase, std::string(ds_.matchTimeS > 20.0 ? "SHIFT 1" : "AUTO")});
            d.hud.push_back({demo::kHubBlueStatus, std::string("Active 0:18")});
            d.hud.push_back({demo::kHubRedStatus, std::string("Inactive")});
        }
        d.hud.push_back({demo::kMessage, ds_.gameSpecificMessage.empty() ? std::string("—") : ds_.gameSpecificMessage});
    }
    // Like a game plugin, the scripted game asks for the end-game whistle when the teleop clock reaches 0:30.
    if (ds_.enabled && ds_.mode == sdk::proto::RobotMode::Teleop && ds_.matchTimeS >= 0.0 && lastMatchTimeS_ > 30.0 &&
        ds_.matchTimeS <= 30.0)
        emit soundCue(QString::fromUtf8(sdk::sound_cue::kEndGame.data(), static_cast<qsizetype>(sdk::sound_cue::kEndGame.size())));
    lastMatchTimeS_ = ds_.matchTimeS;

    d.batteryVoltageV = sdk::kNominalBatteryVoltageV - 0.9 * (std::hypot(static_cast<double>(vel_.x), static_cast<double>(vel_.y)) / 3.5) - (flywheel_ > 1.f ? 0.5 : 0.0);
    d.driverStation = ds_;
    d.physicsStats.bodies = 2 + static_cast<int>(balls_.size());
    d.physicsStats.awakeBodies = 2;
    d.physicsStats.contacts = 4;
    d.physicsStats.lastStepMs = 0.35;
    slot_.publish(builder.build());
    emit snapshotPublished();
}

}  // namespace wpiemu::app::runview
