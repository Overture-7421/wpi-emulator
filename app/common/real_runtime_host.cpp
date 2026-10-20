#include "common/real_runtime_host.h"

#include <QCoreApplication>
#include <QDateTime>

#include <algorithm>
#include <cstdlib>

#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/sdk/core/environment.h"
#include "wpiemu/sdk/core/strings.h"

#ifndef WPIEMU_BUILD_EXTENSION_PATH
#define WPIEMU_BUILD_EXTENSION_PATH ""
#endif

namespace wpiemu::app {

namespace {

std::string envString(const char* name) { return sdk::readEnvironment(name).value_or(std::string()); }

bool envFlag(const char* name) { return sdk::parseBool(envString(name)).value_or(false); }

QString fromStd(const std::string& s) { return QString::fromStdString(s); }

}  // namespace

RealRuntimeHost::RealRuntimeHost(host::Registries registries, RealRuntimeOptions options, QObject* parent)
    : IProjectRuntimeHost(parent), registries_(registries), options_(std::move(options)) {
    timer_.setInterval(std::max(5, options_.pollIntervalMs));
    connect(&timer_, &QTimer::timeout, this, &RealRuntimeHost::pump);
}

RealRuntimeHost::~RealRuntimeHost() {
    timer_.stop();
    session_.reset();   // synchronous: the robot process tree is killed before the app exits
}

std::filesystem::path RealRuntimeHost::extensionPath() const {
    if (!options_.extensionPath.empty()) return options_.extensionPath;
    const std::filesystem::path appDir = options_.appDir.empty() ? std::filesystem::path(QCoreApplication::applicationDirPath().toStdU16String())
                                                                 : options_.appDir;
    const std::filesystem::path buildTree(WPIEMU_BUILD_EXTENSION_PATH);
    // The packaged-layout contract of docs/releasing.md (WPIEMU_EXTENSION, WPIEMU_EXTENSION_DIR, install layouts, build tree).
    return core::launcher::locateHalsimExtension(appDir, &sdk::readEnvironment, buildTree);
}

std::shared_ptr<const sdk::ISimSnapshot> RealRuntimeHost::latestSnapshot() const {
    if (session_) {
        if (auto live = session_->latestSnapshot()) return live;
    }
    return lastSnapshot_;
}

void RealRuntimeHost::setStatus(const HostStatus& status) {
    HostStatus next = status;
    next.runGeneration = runGeneration_;
    if (next == status_) return;
    status_ = next;
    emit statusChanged();
}

void RealRuntimeHost::emitLog(sdk::LogLevel level, const QString& source, const QString& text) {
    emit logLine(emulatorLogLine(level, source, text));
}

void RealRuntimeHost::failToStart(const QString& detail) {
    HostStatus s;
    s.process = sdk::ProcessState::FailedToStart;
    s.link = sdk::LinkState::Disconnected;
    s.detail = detail;
    emitLog(sdk::LogLevel::Error, QStringLiteral("runtime"), detail);
    setStatus(s);
}

void RealRuntimeHost::releaseSession() {
    if (!session_) return;
    timer_.stop();
    session_->requestStop();
    session_->waitUntilFinished();
    setStatus(toHostStatus(session_->status()));   // terminal: Exited, or what ended the run (Crashed, FailedToStart)
    for (const sdk::SessionLogEntry& e : session_->takeLogs(5000)) emit logLine(toHostLogLine(e));
    lastSnapshot_ = session_->latestSnapshot();
    session_.reset();
}

void RealRuntimeHost::setProject(const HostProject& project) {
    const QString& projectPath = project.path;
    const sdk::IGameModule* game = project.game;
    const bool sameProject = projectPath == projectPath_;
    if (!sameProject) beginIntent_.projectReplaced();
    // A different project invalidates a run; edits of the same project apply to the next run.
    if (!sameProject && session_) releaseSession();
    projectPath_ = projectPath;
    config_ = project.config;
    // The selected start position belongs to the project: drop it on a switch or when the entry is gone, follow edits otherwise.
    if (!startPositionId_.isEmpty() && (!sameProject || resolveStartPlacement() == std::nullopt)) {
        startPositionId_.clear();
        emit startPositionChanged();
    }
    if (session_ && !startPositionId_.isEmpty()) session_->controller().setStartPlacement(resolveStartPlacement());
    scene_ = {};
    scene_.config = project.config;
    scene_.projectPath = projectPath;
    scene_.defaultStation = project.defaultStation;
    gamePluginId_.clear();
    if (game != nullptr) {
        gamePluginId_ = game->metadata().id;
        scene_.hud = game->contributeHud();
        scene_.panels = game->contributeEditorPanels();
        scene_.zoneKinds = game->contributeZoneKinds();
        scene_.gameValues = game->contributeGameValues();
        scene_.tags = game->aprilTagLayout();
        scene_.gameName = QString::fromStdString(game->metadata().displayName);
        if (fieldPluginId_ != gamePluginId_ || !field_) {
            field_ = registries_.plugins != nullptr ? host::describeField(registries_.plugins->find(gamePluginId_)) : nullptr;
            fieldPluginId_ = gamePluginId_;
        }
        scene_.field = field_;
    }
    if (!sameProject) {
        lastSnapshot_.reset();
        if (!session_) setStatus(HostStatus{});
    }
    emit sceneChanged();
}

void RealRuntimeHost::launch() {
    if (session_ && status_.active()) return;   // one robot instance serves detection and the run
    releaseSession();
    ++runGeneration_;
    if (!config_ || projectPath_.isEmpty()) {
        failToStart(tr("No robot configuration loaded."));
        return;
    }

    host::Options o;
    o.mode = sdk::SessionMode::Armed;
    o.projectDir = std::filesystem::path(projectPath_.toStdU16String());
    o.extensionLibrary = extensionPath();
    if (o.extensionLibrary.empty()) {
        failToStart(tr("The HALSim extension (%1) was not found next to the application. Reinstall the emulator or set WPIEMU_EXTENSION.")
                        .arg(fromStd(core::launcher::halsimExtensionFileName())));
        return;
    }
    o.scratchDir = options_.scratchDir;
    o.handshakeTimeout = options_.handshakeTimeout;
    o.launchMode = envString("WPIEMU_LAUNCH_MODE") == "direct" ? core::launcher::LaunchMode::ExternalDirect : core::launcher::LaunchMode::GradleInitScript;
    o.gradleOffline = envFlag("WPIEMU_GRADLE_OFFLINE");
    o.preStartPolling = envFlag("WPIEMU_PRESTART_POLLING");
    o.driverStation = ds_;
    o.startPose = resolveStartPlacement();
    o.timeScale = timeScale_;
    if (options_.toolchainOverrides) {
        const RealRuntimeOptions::ToolchainOverrides toolchain = options_.toolchainOverrides();
        o.javaHome = toolchain.javaHome;
        o.wpilibHome = toolchain.wpilibHome;
    }
    if (options_.tweak) options_.tweak(o);

    emitLog(sdk::LogLevel::Info, QStringLiteral("runtime"), tr("Launching the robot with the HALSim extension"));
    auto started = host::RobotSession::start(registries_, *config_, std::move(o));
    if (!started) {
        failToStart(fromStd(started.error().message));
        return;
    }
    session_ = std::move(started).value();
    launchedConfig_ = config_;
    lastSnapshot_.reset();
    setStatus(toHostStatus(session_->status()));
    emit snapshotPublished();
    timer_.start();
}

void RealRuntimeHost::startSimulation() {
    beginIntent_.request();
    // The armed robot holds the model it was launched with: relaunch when the project was edited since.
    if (session_ && status_.active() && config_ != launchedConfig_) releaseSession();
    if (!session_ || !status_.active()) launch();
    sendBeginIfReady();
}

void RealRuntimeHost::sendBeginIfReady() {
    if (!beginIntent_.takeWhenReady(session_ != nullptr && status_.programStarted) || status_.simulating) return;
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::BeginLockstep;
    session_->controller().control(c);
    if (timeScale_ != 1.0) setTimeScale(timeScale_);
    if (paused_) setPaused(true);
}

void RealRuntimeHost::stopSimulation() {
    beginIntent_.cancel();
    if (!session_ || !status_.simulating) return;
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::EndLockstep;
    session_->controller().control(c);
}

void RealRuntimeHost::stop() {
    beginIntent_.cancel();
    if (!session_) return;
    if (status_.active() || !session_->finished()) session_->requestStop();
    pump();
}

void RealRuntimeHost::restart() {
    const bool resume = status_.simulating || beginIntent_.pending();
    releaseSession();
    launch();
    if (resume) beginIntent_.request();
}

void RealRuntimeHost::resetSimulation() {
    if (!session_) return;
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::Reset;
    session_->controller().control(c);
}

std::optional<sdk::RobotStartPose> RealRuntimeHost::resolveStartPlacement() const {
    if (startPositionId_.isEmpty() || !config_) return std::nullopt;
    if (const sdk::StartPosition* p = config_->findStartPosition(startPositionId_.toStdString())) return p->pose;
    return std::nullopt;
}

void RealRuntimeHost::setStartPosition(const QString& id) {
    const QString next = config_ && config_->findStartPosition(id.toStdString()) != nullptr ? id : QString();
    const bool changed = next != startPositionId_;
    startPositionId_ = next;
    if (session_) session_->controller().setStartPlacement(resolveStartPlacement());
    if (changed) emit startPositionChanged();
}

void RealRuntimeHost::setPaused(bool paused) {
    paused_ = paused;
    if (!session_) return;
    sdk::proto::SimControl c;
    c.command = paused ? sdk::proto::SimCommand::Pause : sdk::proto::SimCommand::Resume;
    session_->controller().control(c);
}

void RealRuntimeHost::stepTicks(int ticks) {
    if (!session_ || ticks <= 0) return;
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::Step;
    c.stepCount = static_cast<std::uint32_t>(ticks);
    session_->controller().control(c);
}

void RealRuntimeHost::setTimeScale(double scale) {
    timeScale_ = std::clamp(scale, 0.05, 8.0);
    if (!session_) return;
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::SetTimeScale;
    c.timeScale = timeScale_;
    session_->controller().control(c);
}

void RealRuntimeHost::setDriverStation(const sdk::proto::DsState& state) {
    ds_ = state;
    if (session_) session_->controller().setDriverStation(state);
}

void RealRuntimeHost::pump() {
    if (!session_) {
        timer_.stop();
        return;
    }
    for (const sdk::SessionLogEntry& e : session_->takeLogs(500)) emit logLine(toHostLogLine(e));
    const sdk::SessionStatus s = session_->status();
    setStatus(toHostStatus(s));
    sendBeginIfReady();
    if (auto snapshot = session_->latestSnapshot(); snapshot && snapshot != lastSnapshot_) {
        lastSnapshot_ = std::move(snapshot);
        emit snapshotPublished();
    }
    if (!s.active() && session_->finished()) {
        releaseSession();   // drains the last log lines and keeps the final snapshot
        emit snapshotPublished();
    }
}

}  // namespace wpiemu::app
