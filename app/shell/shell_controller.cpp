#include "shell/shell_controller.h"

#include <QDir>
#include <QFileInfo>

namespace wpiemu::app {

ShellController::ShellController(AppServices& services, QObject* parent) : QObject(parent), services_(services) {
    connect(&services_.session(), &ProjectSession::projectChanged, this, [this] {
        syncRuntime();
        emit projectChanged();
    });
    connect(&services_.session(), &ProjectSession::configChanged, this, [this] {
        syncRuntime();
        emit projectChanged();
    });
    connect(&services_.runtime(), &IRuntimeHost::statusChanged, this, &ShellController::runtimeChanged);
}

QString ShellController::projectName() const {
    return hasProject() ? QFileInfo(services_.session().path()).fileName() : QString();
}

QString ShellController::windowTitle() const {
    return hasProject() ? tr("%1 — %2 — WPI Emulator").arg(projectName(), gameLabel()) : tr("WPI Emulator");
}

QString ShellController::gameLabel() const {
    if (!hasProject()) return {};
    if (const sdk::IGameModule* module = services_.session().gameModule())
        return QString::fromStdString(module->metadata().displayName);
    return tr("Core-only");
}

QString ShellController::runtimeLabel() const {
    const HostStatus s = services_.runtime().status();
    switch (s.process) {
        case sdk::ProcessState::Idle: return tr("Robot not running");
        case sdk::ProcessState::Starting: return tr("Starting robot…");
        case sdk::ProcessState::Running:
            if (s.link == sdk::LinkState::Connected) {
                if (s.simulating) return tr("Simulating");
                return s.programStarted ? tr("Robot ready") : tr("Starting robot program…");
            }
            return tr("Waiting for the robot to connect…");
        case sdk::ProcessState::Stopping: return tr("Stopping robot…");
        case sdk::ProcessState::Exited: return tr("Robot stopped");
        case sdk::ProcessState::Crashed: return tr("Robot crashed");
        case sdk::ProcessState::FailedToStart: return tr("Robot failed to start");
    }
    return {};
}

void ShellController::setMode(AppMode mode) {
    if (mode == AppMode::Run && !canRun()) return;
    if (mode == mode_) return;
    mode_ = mode;
    emit modeChanged(mode_);
    if (mode_ == AppMode::Run) services_.runtime().startSimulation();   // launches first if needed; Begin is sent once armed
    else launchRobot();
}

void ShellController::syncRuntime() {    ProjectSession& session = services_.session();
    if (const sdk::IGameModule* module = session.gameModule()) {
        // Plugin mechanism templates must be registered before anything validates or builds against them.
        if (auto* mutableModule = services_.plugins().find(module->metadata().id)) {
            if (auto status = services_.pluginRegistrar().ensureRegistered(*mutableModule); !status)
                emit errorOccurred(tr("Game module problem: %1").arg(QString::fromStdString(status.error().message)));
        }
    }
    services_.runtime().setProject(session.path(), session.config(), session.gameModule());
}

OpenOutcome ShellController::openProject(const QString& path) {
    ProjectSession& session = services_.session();
    if (auto flushed = session.flushPendingChanges(); !flushed) {
        emit errorOccurred(QString::fromStdString(flushed.error().message));
        return OpenOutcome::Blocked;
    }
    if (auto opened = session.open(path); !opened) {
        emit errorOccurred(QString::fromStdString(opened.error().message));
        return OpenOutcome::Invalid;
    }
    if (!session.loadError().isEmpty())
        emit notice(tr("The saved robot setup could not be read (%1). Please set the robot up again.").arg(session.loadError()));
    if (session.needsSetup()) return OpenOutcome::NeedsSetup;
    mode_ = AppMode::Editor;
    emit modeChanged(mode_);
    launchRobot();
    return OpenOutcome::Opened;
}

bool ShellController::restoreLastProject() {
    if (auto flushed = services_.session().flushPendingChanges(); !flushed) return false;
    if (!services_.settings().data().reopenLastProject) return false;
    services_.recentProjects().pruneMissing();
    const QString last = services_.recentProjects().lastProject();
    if (last.isEmpty()) return false;
    ProjectSession& session = services_.session();
    if (auto opened = session.open(last); !opened) {
        services_.recentProjects().remove(last);
        return false;
    }
    if (session.needsSetup()) {
        // The user did not finish setting this project up last time: let them continue from the welcome page.
        (void)session.close();
        return false;
    }
    mode_ = AppMode::Editor;
    emit modeChanged(mode_);
    launchRobot();
    return true;
}

void ShellController::closeProject() {
    if (auto closed = services_.session().close(); !closed) {
        emit errorOccurred(QString::fromStdString(closed.error().message));
        return;
    }
    services_.runtime().stop();
    mode_ = AppMode::Editor;
    emit modeChanged(mode_);
}

void ShellController::setupCompleted() {
    mode_ = AppMode::Editor;
    emit modeChanged(mode_);
    launchRobot();
}

void ShellController::launchRobot() {
    if (canRun()) services_.runtime().launch();
}

// Starts the simulation on the (already launched) robot; the Run tab is shown while it runs.
void ShellController::startSimulation() {
    if (!canRun()) {
        emit errorOccurred(tr("Set up the robot first."));
        return;
    }
    if (mode_ != AppMode::Run) setMode(AppMode::Run);   // entering the Run tab starts the simulation
    else services_.runtime().startSimulation();
}

void ShellController::stopSimulation() { services_.runtime().stopSimulation(); }

void ShellController::stopRobot() { services_.runtime().stop(); }

void ShellController::rescanDevices() {
    if (!canRun()) {
        emit errorOccurred(tr("Set up the robot first."));
        return;
    }
    services_.runtime().restart();
}

}  // namespace wpiemu::app
