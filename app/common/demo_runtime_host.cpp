#include "common/demo_runtime_host.h"

#include <QDateTime>

#include <algorithm>
#include <cmath>

namespace wpiemu::app {

namespace {

constexpr int kSimIntervalMs = 33;

sdk::DiscoveredDevice makeDevice(std::uint32_t runtimeId, const char* type, const char* vendor, int id,
                                 sdk::CapabilitySet caps, const char* family) {
    sdk::DiscoveredDevice d;
    d.id = sdk::DeviceId(runtimeId);
    d.descriptor.family = family;
    d.descriptor.type = type;
    d.descriptor.vendor = vendor;
    d.descriptor.id = id;
    d.descriptor.bus = family == std::string("phoenix6") ? sdk::kBusUnknown : "rio";
    d.descriptor.capabilities = caps;
    return d;
}

sdk::CapabilitySet talonCaps() {
    return sdk::Capability::MotorOutput | sdk::Capability::RotorFeedback | sdk::Capability::PositionSensor |
           sdk::Capability::VelocitySensor | sdk::Capability::LimitInputs | sdk::Capability::SupplyVoltage;
}

}  // namespace

DemoRuntimeHost::DemoRuntimeHost(QObject* parent) : IRuntimeHost(parent) {
    qRegisterMetaType<wpiemu::app::HostLogLine>();
    startupTimer_.setSingleShot(true);
    connect(&startupTimer_, &QTimer::timeout, this, &DemoRuntimeHost::advanceStartup);
    simTimer_.setInterval(kSimIntervalMs);
    connect(&simTimer_, &QTimer::timeout, this, &DemoRuntimeHost::tickSimulation);
}

std::vector<sdk::DiscoveredDevice> DemoRuntimeHost::demoDevicesFor(const sdk::RobotConfig& config) {
    std::vector<sdk::DiscoveredDevice> out;
    std::uint32_t next = 1;
    int driveId = 1;
    int steerId = 11;
    int encoderId = 21;
    for (const auto& w : config.drivetrain.spec.wheels) {
        out.push_back(makeDevice(next++, "talonfx", "ctre", driveId++, talonCaps(), "phoenix6"));
        if (w.steered) {
            out.push_back(makeDevice(next++, "talonfx", "ctre", steerId++, talonCaps(), "phoenix6"));
            out.push_back(makeDevice(next++, "cancoder", "ctre", encoderId++,
                                     sdk::Capability::PositionSensor | sdk::Capability::VelocitySensor, "phoenix6"));
        }
    }
    out.push_back(makeDevice(next++, "pigeon2", "ctre", 0, sdk::CapabilitySet(sdk::Capability::Imu), "phoenix6"));
    out.push_back(makeDevice(next++, "talonfx", "ctre", 30, talonCaps(), "phoenix6"));
    out.push_back(makeDevice(next++, "dio", "wpilib", 0, sdk::CapabilitySet(sdk::Capability::DigitalInput), "hal"));
    return out;
}

void DemoRuntimeHost::setProject(const QString& projectPath, std::shared_ptr<const sdk::RobotConfig> config,
                                 const sdk::IGameModule* game) {
    // A different project invalidates the run; edits of the same project are applied live.
    if (projectPath != projectPath_) {
        beginIntent_.projectReplaced();
        if (status_.active()) stop();
    }
    const bool sameProject = projectPath == projectPath_;
    projectPath_ = projectPath;
    scene_ = {};
    scene_.config = std::move(config);
    if (game != nullptr) {
        scene_.hud = game->contributeHud();
        scene_.panels = game->contributeEditorPanels();
        scene_.zoneKinds = game->contributeZoneKinds();
        scene_.gameName = QString::fromStdString(game->metadata().displayName);
    }
    plannedDevices_ = scene_.config ? demoDevicesFor(*scene_.config) : std::vector<sdk::DiscoveredDevice>{};
    if (!sameProject) {
        announced_.clear();
        resetSimulation();
    }
    emit sceneChanged();
}

void DemoRuntimeHost::setStatus(HostProcessState process, HostLinkState link, const QString& detail) {
    HostStatus next = status_;
    next.process = process;
    next.link = link;
    next.detail = detail;
    if (next == status_) return;
    status_ = next;
    emit statusChanged();
}

void DemoRuntimeHost::log(sdk::LogLevel level, const QString& source, const QString& text) {
    HostLogLine line;
    line.timestampMs = QDateTime::currentMSecsSinceEpoch();
    line.level = level;
    line.stream = HostLogStream::Emulator;
    line.source = source;
    line.text = text;
    emit logLine(line);
}

void DemoRuntimeHost::launch() {
    if (status_.active()) return;
    if (!scene_.config) {
        setStatus(HostProcessState::FailedToStart, HostLinkState::Disconnected, tr("No robot configuration loaded."));
        return;
    }
    status_.exitCode = 0;
    status_.programStarted = false;
    status_.simulating = false;
    startupStage_ = 0;
    announced_.clear();
    ++status_.runGeneration;
    setStatus(HostProcessState::Starting, HostLinkState::Disconnected);
    log(sdk::LogLevel::Info, QStringLiteral("demo"), tr("Demo: launching robot"));
    startupTimer_.start(startupMs_);
}

void DemoRuntimeHost::startSimulation() {
    beginIntent_.request();
    if (!status_.active()) launch();
    beginIfReady();
}

void DemoRuntimeHost::beginIfReady() {
    if (!beginIntent_.takeWhenReady(status_.programStarted)) return;
    resetSimulation();
    status_.simulating = true;
    setStatus(status_.process, status_.link, status_.detail);
    emit statusChanged();
    publish();
}

void DemoRuntimeHost::stopSimulation() {
    beginIntent_.cancel();
    if (!status_.simulating) return;
    status_.simulating = false;
    emit statusChanged();
    publish();
}

void DemoRuntimeHost::advanceStartup() {
    switch (startupStage_++) {
        case 0:
            setStatus(HostProcessState::Running, HostLinkState::Connecting);
            startupTimer_.start(startupMs_);
            break;
        case 1:
            status_.programStarted = true;
            setStatus(HostProcessState::Running, HostLinkState::Connected);
            log(sdk::LogLevel::Info, QStringLiteral("extension"), tr("Demo: HALSim extension connected"));
            simTimer_.start();
            startupTimer_.start(announceMs_);
            beginIfReady();
            break;
        default: announceNextDevice(); break;
    }
}

void DemoRuntimeHost::announceNextDevice() {
    if (announced_.size() >= plannedDevices_.size()) {
        log(sdk::LogLevel::Info, QStringLiteral("discovery"), tr("Demo: %1 devices announced").arg(announced_.size()));
        return;
    }
    sdk::DiscoveredDevice d = plannedDevices_[announced_.size()];
    if (scene_.config) {
        for (const auto& m : scene_.config->allMappings())
            if (m.device.physical() == d.descriptor.physicalId()) d.mapped = true;
    }
    announced_.push_back(d);
    log(sdk::LogLevel::Info, QStringLiteral("discovery"),
        tr("Demo: found %1 %2 (id %3)").arg(QString::fromStdString(d.descriptor.vendor),
                                            QString::fromStdString(d.descriptor.type)).arg(d.descriptor.id));
    publish();
    startupTimer_.start(announceMs_);
}

void DemoRuntimeHost::stop() {
    beginIntent_.cancel();
    if (!status_.active()) return;
    startupTimer_.stop();
    simTimer_.stop();
    status_.simulating = false;
    status_.programStarted = false;
    setStatus(HostProcessState::Exited, HostLinkState::Disconnected);
    log(sdk::LogLevel::Info, QStringLiteral("demo"), tr("Demo: robot stopped"));
    publish();
}

void DemoRuntimeHost::restart() {
    const bool resume = status_.simulating || beginIntent_.pending();
    stop();
    resetSimulation();
    launch();
    if (resume) beginIntent_.request();
}

void DemoRuntimeHost::resetSimulation() {
    x_ = y_ = yaw_ = 0.0;
    vx_ = vy_ = omega_ = 0.0;
    wheelSpin_ = 0.0;
    tick_ = 0;
    publish();
}

void DemoRuntimeHost::setPaused(bool paused) {
    paused_ = paused;
    publish();
}

void DemoRuntimeHost::stepTicks(int ticks) {
    if (!paused_) return;
    for (int i = 0; i < std::max(1, ticks); ++i) integrate(0.005);
    publish();
}

void DemoRuntimeHost::setTimeScale(double scale) { timeScale_ = std::clamp(scale, 0.05, 8.0); }

void DemoRuntimeHost::setDriverStation(const sdk::proto::DsState& state) {
    ds_ = state;
    publish();
}

void DemoRuntimeHost::tickSimulation() {
    if (paused_ || !status_.simulating) return;
    integrate(kSimIntervalMs / 1000.0 * timeScale_);
    publish();
}

void DemoRuntimeHost::integrate(double dt) {
    double forward = 0.0, strafe = 0.0, turn = 0.0;
    if (ds_.enabled && !ds_.eStop) {
        const auto& js = ds_.joysticks[0];
        if (js.present && js.axes.size() >= 3) {
            forward = -static_cast<double>(js.axes[1]);
            strafe = -static_cast<double>(js.axes[0]);
            turn = -static_cast<double>(js.axes[js.axes.size() > 4 ? 4 : 2]);
        } else {
            forward = 0.5;   // nobody is holding a stick: cruise in a slow circle
            turn = 0.35;
        }
    }
    constexpr double kMaxSpeed = 3.0, kMaxOmega = 3.5, kAccel = 8.0;
    const double targetVx = forward * kMaxSpeed, targetVy = strafe * kMaxSpeed, targetW = turn * kMaxOmega;
    const auto approach = [&](double cur, double target) {
        const double step = kAccel * dt;
        return cur + std::clamp(target - cur, -step, step);
    };
    vx_ = approach(vx_, targetVx);
    vy_ = approach(vy_, targetVy);
    omega_ = approach(omega_, targetW);
    x_ += (vx_ * std::cos(yaw_) - vy_ * std::sin(yaw_)) * dt;
    y_ += (vx_ * std::sin(yaw_) + vy_ * std::cos(yaw_)) * dt;
    yaw_ += omega_ * dt;
    wheelSpin_ += vx_ * dt / 0.05;
    ++tick_;
}

void DemoRuntimeHost::publish() {
    sdk::SimSnapshotData data;
    data.tick = tick_;
    data.simTimeUs = static_cast<sdk::SimTimeUs>(tick_) * 5000;
    data.paused = paused_;
    data.robotConnected = status_.link == HostLinkState::Connected;
    data.driverStation = ds_;
    data.devices = announced_;

    const float chassisZ = scene_.config ? scene_.config->chassis.comHeightM : 0.2f;
    data.chassis.name = "chassis";
    data.chassis.pose = sdk::Pose3({static_cast<float>(x_), static_cast<float>(y_), chassisZ}, sdk::Quat::fromYaw(static_cast<float>(yaw_)));
    data.chassis.linearVelocity = {static_cast<float>(vx_), static_cast<float>(vy_), 0.f};
    data.chassis.angularVelocity = {0.f, 0.f, static_cast<float>(omega_)};
    if (scene_.config) {
        for (const auto& w : scene_.config->drivetrain.spec.wheels) {
            sdk::WheelSample s;
            s.name = w.name;
            s.contactPoint = data.chassis.pose.transformPoint({w.positionM.x, w.positionM.y, -chassisZ});
            s.spinAngleRad = wheelSpin_;
            s.normalLoadN = scene_.config->chassis.massKg * 9.81f / static_cast<float>(std::max<std::size_t>(1, scene_.config->drivetrain.spec.wheels.size()));
            data.wheels.push_back(std::move(s));
        }
    }
    const double load = std::min(1.0, std::hypot(vx_, vy_) / 3.0);
    data.batteryVoltageV = ds_.enabled ? 12.6 - 1.8 * load : 12.6;
    slot_.publish(std::make_shared<const sdk::SimSnapshot>(std::move(data)));
    emit snapshotPublished();
}

}  // namespace wpiemu::app
