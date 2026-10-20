#include "shell/run_backend_adapter.h"

namespace wpiemu::app {

namespace {

runview::RobotProcessState mapProcess(sdk::ProcessState s) {
    switch (s) {
        case sdk::ProcessState::Idle: return runview::RobotProcessState::Idle;
        case sdk::ProcessState::Starting: return runview::RobotProcessState::Starting;
        case sdk::ProcessState::Running: return runview::RobotProcessState::Running;
        case sdk::ProcessState::Stopping: return runview::RobotProcessState::Stopping;
        case sdk::ProcessState::Exited: return runview::RobotProcessState::Exited;
        case sdk::ProcessState::Crashed: return runview::RobotProcessState::Crashed;
        case sdk::ProcessState::FailedToStart: return runview::RobotProcessState::FailedToStart;
    }
    return runview::RobotProcessState::Idle;
}

runview::LinkState mapLink(sdk::LinkState s) {
    switch (s) {
        case sdk::LinkState::Disconnected: return runview::LinkState::Disconnected;
        case sdk::LinkState::Connecting: return runview::LinkState::Connecting;
        case sdk::LinkState::Connected: return runview::LinkState::Connected;
        case sdk::LinkState::HandshakeTimeout: return runview::LinkState::HandshakeTimeout;
        case sdk::LinkState::ProtocolMismatch: return runview::LinkState::ProtocolMismatch;
        case sdk::LinkState::Stalled: return runview::LinkState::Stalled;
    }
    return runview::LinkState::Disconnected;
}

runview::LogStream mapStream(sdk::LogStream s) {
    switch (s) {
        case sdk::LogStream::Stdout: return runview::LogStream::Stdout;
        case sdk::LogStream::Stderr: return runview::LogStream::Stderr;
        case sdk::LogStream::Emulator: break;
    }
    return runview::LogStream::Emulator;
}

}  // namespace

RunBackendAdapter::RunBackendAdapter(AppServices& services, QObject* parent) : runview::IRunBackend(parent), services_(services) {
    runview::registerRunViewMetaTypes();
    connect(&services_.runtime(), &IRuntimeHost::statusChanged, this, &runview::IRunBackend::statusChanged);
    connect(&services_.runtime(), &IRuntimeHost::sceneChanged, this, &runview::IRunBackend::sceneInfoChanged);
    connect(&services_.runtime(), &IRuntimeHost::startPositionChanged, this, &runview::IRunBackend::startPositionChanged);
    connect(&services_.runtime(), &IRuntimeHost::logLine, this, &RunBackendAdapter::onLog);
}

runview::RunStatus RunBackendAdapter::status() const {
    const HostStatus s = services_.runtime().status();
    runview::RunStatus out;
    out.process = mapProcess(s.process);
    out.link = mapLink(s.link);
    out.exitCode = s.exitCode;
    out.detail = s.detail;
    out.programStarted = s.programStarted;
    out.simulating = s.simulating;
    return out;
}

runview::RunSceneInfo RunBackendAdapter::sceneInfo() const {
    const HostScene scene = services_.runtime().scene();
    runview::RunSceneInfo out;
    out.config = scene.config;
    out.field = scene.field;
    out.hud = scene.hud;
    out.panels = scene.panels;
    out.zoneKinds = scene.zoneKinds;
    out.gameValues = scene.gameValues;
    out.tags = scene.tags;
    out.gameName = scene.gameName;
    return out;
}

void RunBackendAdapter::launchRobot() { services_.runtime().launch(); }

void RunBackendAdapter::startSimulation() {
    if (!dsSent_) {
        // Until the DS panel says otherwise the emulated DS sits at the station chosen in the wizard.
        const ProjectPrefs& prefs = services_.session().prefs();
        sdk::proto::DsState ds;
        ds.station = sdk::proto::stationOf(prefs.alliance, prefs.station);
        services_.runtime().setDriverStation(ds);
    }
    services_.runtime().startSimulation();
}

void RunBackendAdapter::stopSimulation() { services_.runtime().stopSimulation(); }
void RunBackendAdapter::restartRobot() { services_.runtime().restart(); }
void RunBackendAdapter::resetSimulation() { services_.runtime().resetSimulation(); }
void RunBackendAdapter::setStartPosition(const QString& id) { services_.runtime().setStartPosition(id); }
void RunBackendAdapter::setPaused(bool paused) { services_.runtime().setPaused(paused); }
void RunBackendAdapter::stepTicks(int ticks) { services_.runtime().stepTicks(ticks); }
void RunBackendAdapter::setTimeScale(double scale) { services_.runtime().setTimeScale(scale); }

void RunBackendAdapter::setDriverStation(const sdk::proto::DsState& state) {
    dsSent_ = true;
    services_.runtime().setDriverStation(state);
}

void RunBackendAdapter::onLog(const HostLogLine& line) {
    runview::RunLogEntry entry;
    entry.timestampMs = line.timestampMs;
    entry.level = line.level;
    entry.stream = mapStream(line.stream);
    entry.source = line.source;
    entry.text = line.text;
    emit logEntry(entry);
}

}  // namespace wpiemu::app
