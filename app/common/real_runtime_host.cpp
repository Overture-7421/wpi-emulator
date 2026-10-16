#include "common/real_runtime_host.h"

#include <QCoreApplication>
#include <QDateTime>

#include <algorithm>
#include <cstdlib>

#include "wpiemu/core/launcher/extension_locator.h"

#ifndef WPIEMU_BUILD_EXTENSION_PATH
#define WPIEMU_BUILD_EXTENSION_PATH ""
#endif

namespace wpiemu::app {

namespace {

std::string envString(const char* name) { return qEnvironmentVariable(name).toStdString(); }

bool envFlag(const char* name) {
    const std::string v = envString(name);
    return v == "1" || v == "true" || v == "on" || v == "yes";
}

QString fromStd(const std::string& s) { return QString::fromStdString(s); }

HostLogStream mapStream(host::LogStream s) {
    switch (s) {
        case host::LogStream::Stdout: return HostLogStream::Stdout;
        case host::LogStream::Stderr: return HostLogStream::Stderr;
        case host::LogStream::Emulator: break;
    }
    return HostLogStream::Emulator;
}

}  // namespace

RealRuntimeHost::RealRuntimeHost(host::Registries registries, RealRuntimeOptions options, QObject* parent)
    : IRuntimeHost(parent), registries_(registries), options_(std::move(options)) {
    qRegisterMetaType<wpiemu::app::HostLogLine>();
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
    // The packaged-layout contract of docs/packaging.md (WPIEMU_EXTENSION, WPIEMU_EXTENSION_DIR, install layouts, build tree).
    const auto getenv = [](std::string_view name) -> std::optional<std::string> {
        const QByteArray key(name.data(), static_cast<qsizetype>(name.size()));
        if (!qEnvironmentVariableIsSet(key.constData())) return std::nullopt;
        return qEnvironmentVariable(key.constData()).toStdString();
    };
    return core::launcher::locateHalsimExtension(appDir, getenv, buildTree);
}

std::shared_ptr<const sdk::ISimSnapshot> RealRuntimeHost::latestSnapshot() const {
    if (session_) {
        if (auto live = session_->latestSnapshot()) return live;
    }
    return lastSnapshot_;
}

HostStatus RealRuntimeHost::map(const host::SessionStatus& s) {
    HostStatus out;
    out.programStarted = s.programStarted;
    out.simulating = s.simulating;
    out.exitCode = s.exitCode;
    out.detail = fromStd(s.detail);
    switch (s.process) {
        case host::ProcessState::Idle: out.process = HostProcessState::Idle; break;
        case host::ProcessState::Starting: out.process = HostProcessState::Starting; break;
        case host::ProcessState::Running: out.process = HostProcessState::Running; break;
        case host::ProcessState::Stopping: out.process = HostProcessState::Stopping; break;
        case host::ProcessState::Exited: out.process = HostProcessState::Exited; break;
        case host::ProcessState::Crashed: out.process = HostProcessState::Crashed; break;
        case host::ProcessState::FailedToStart: out.process = HostProcessState::FailedToStart; break;
    }
    switch (s.link) {
        case host::LinkState::Disconnected: out.link = HostLinkState::Disconnected; break;
        case host::LinkState::Connecting: out.link = HostLinkState::Connecting; break;
        case host::LinkState::Connected: out.link = HostLinkState::Connected; break;
        case host::LinkState::HandshakeTimeout: out.link = HostLinkState::HandshakeTimeout; break;
        case host::LinkState::ProtocolMismatch: out.link = HostLinkState::ProtocolMismatch; break;
        case host::LinkState::Stalled: out.link = HostLinkState::Stalled; break;
    }
    return out;
}

void RealRuntimeHost::setStatus(const HostStatus& status) {
    if (status == status_) return;
    status_ = status;
    emit statusChanged();
}

void RealRuntimeHost::emitLog(sdk::LogLevel level, const QString& source, const QString& text) {
    HostLogLine line;
    line.timestampMs = QDateTime::currentMSecsSinceEpoch();
    line.level = level;
    line.stream = HostLogStream::Emulator;
    line.source = source;
    line.text = text;
    emit logLine(line);
}

void RealRuntimeHost::failToStart(const QString& detail) {
    HostStatus s;
    s.process = HostProcessState::FailedToStart;
    s.link = HostLinkState::Disconnected;
    s.detail = detail;
    emitLog(sdk::LogLevel::Error, QStringLiteral("runtime"), detail);
    setStatus(s);
}

void RealRuntimeHost::releaseSession() {
    if (!session_) return;
    timer_.stop();
    session_->requestStop();
    session_->waitUntilFinished();
    setStatus(map(session_->status()));   // terminal: Exited, or what ended the run (Crashed, FailedToStart)
    for (const host::LogEntry& e : session_->takeLogs(5000)) {
        HostLogLine line;
        line.timestampMs = e.timestampMs;
        line.level = e.level;
        line.stream = mapStream(e.stream);
        line.source = fromStd(e.source);
        line.text = fromStd(e.text);
        emit logLine(line);
    }
    lastSnapshot_ = session_->latestSnapshot();
    session_.reset();
}

void RealRuntimeHost::setProject(const QString& projectPath, std::shared_ptr<const sdk::RobotConfig> config, const sdk::IGameModule* game) {
    const bool sameProject = projectPath == projectPath_;
    // A different project invalidates a run; edits of the same project apply to the next run.
    if (!sameProject && session_) releaseSession();
    projectPath_ = projectPath;
    config_ = config;
    scene_ = {};
    scene_.config = std::move(config);
    gamePluginId_.clear();
    if (game != nullptr) {
        gamePluginId_ = game->metadata().id;
        scene_.hud = game->contributeHud();
        scene_.panels = game->contributeEditorPanels();
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
    if (!config_ || projectPath_.isEmpty()) {
        failToStart(tr("No robot configuration loaded."));
        return;
    }

    host::Options o;
    o.mode = host::Mode::Armed;
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
    o.timeScale = timeScale_;
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
    setStatus(map(session_->status()));
    emit snapshotPublished();
    timer_.start();
}

void RealRuntimeHost::startSimulation() {
    beginWhenReady_ = true;
    // The armed robot holds the model it was launched with: relaunch when the project was edited since.
    if (session_ && status_.active() && config_ != launchedConfig_) releaseSession();
    if (!session_ || !status_.active()) launch();
    sendBeginIfReady();
}

void RealRuntimeHost::sendBeginIfReady() {
    if (!beginWhenReady_ || !session_ || !status_.programStarted) return;
    if (status_.simulating) {
        beginWhenReady_ = false;
        return;
    }
    beginWhenReady_ = false;
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::BeginLockstep;
    session_->controller().control(c);
    if (timeScale_ != 1.0) setTimeScale(timeScale_);
    if (paused_) setPaused(true);
}

void RealRuntimeHost::stopSimulation() {
    beginWhenReady_ = false;
    if (!session_ || !status_.simulating) return;
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::EndLockstep;
    session_->controller().control(c);
}

void RealRuntimeHost::stop() {
    beginWhenReady_ = false;
    if (!session_) return;
    if (status_.active() || !session_->finished()) session_->requestStop();
    pump();
}

void RealRuntimeHost::restart() {
    const bool resume = status_.simulating || beginWhenReady_;
    releaseSession();
    launch();
    beginWhenReady_ = resume;
}

void RealRuntimeHost::resetSimulation() {
    if (!session_) return;
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::Reset;
    session_->controller().control(c);
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
    for (const host::LogEntry& e : session_->takeLogs(500)) {
        HostLogLine line;
        line.timestampMs = e.timestampMs;
        line.level = e.level;
        line.stream = mapStream(e.stream);
        line.source = fromStd(e.source);
        line.text = fromStd(e.text);
        emit logLine(line);
    }
    const host::SessionStatus s = session_->status();
    setStatus(map(s));
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
