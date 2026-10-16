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
    return hasProject() ? tr("%1 — WPI Emulator").arg(projectName()) : tr("WPI Emulator");
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
        case HostProcessState::Idle: return tr("Robot not running");
        case HostProcessState::Starting: return tr("Starting robot…");
        case HostProcessState::Running:
            if (s.link == HostLinkState::Connected)
                return s.mode == RuntimeMode::Discovery ? tr("Detecting devices…") : tr("Robot connected");
            return tr("Waiting for the robot to connect…");
        case HostProcessState::Stopping: return tr("Stopping robot…");
        case HostProcessState::Exited: return tr("Robot stopped");
        case HostProcessState::Crashed: return tr("Robot crashed");
        case HostProcessState::FailedToStart: return tr("Robot failed to start");
    }
    return {};
}

void ShellController::setMode(AppMode mode) {
    if (mode == AppMode::Run && !canRun()) return;
    if (mode == mode_) return;
    mode_ = mode;
    emit modeChanged(mode_);
}

void ShellController::syncRuntime() {
    ProjectSession& session = services_.session();
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
    if (auto opened = session.open(path); !opened) {
        emit errorOccurred(QString::fromStdString(opened.error().message));
        return OpenOutcome::Invalid;
    }
    if (!session.loadError().isEmpty())
        emit notice(tr("The saved robot setup could not be read (%1). Please set the robot up again.").arg(session.loadError()));
    if (session.needsSetup()) return OpenOutcome::NeedsSetup;
    mode_ = AppMode::Editor;
    emit modeChanged(mode_);
    return OpenOutcome::Opened;
}

bool ShellController::restoreLastProject() {
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
        session.close();
        return false;
    }
    mode_ = AppMode::Editor;
    emit modeChanged(mode_);
    return true;
}

void ShellController::closeProject() {
    services_.runtime().stop();
    services_.session().close();
    mode_ = AppMode::Editor;
    emit modeChanged(mode_);
}

void ShellController::setupCompleted(bool detectDevices) {
    mode_ = AppMode::Editor;
    emit modeChanged(mode_);
    if (detectDevices) startDiscovery();
}

void ShellController::startDiscovery() {
    if (!canRun()) {
        emit errorOccurred(tr("Set up the robot first."));
        return;
    }
    setMode(AppMode::Editor);
    services_.runtime().start(RuntimeMode::Discovery);
}

void ShellController::startRobot() {
    if (!canRun()) {
        emit errorOccurred(tr("Set up the robot first."));
        return;
    }
    setMode(AppMode::Run);
    services_.runtime().start(RuntimeMode::Run);
}

void ShellController::stopRobot() { services_.runtime().stop(); }

}  // namespace wpiemu::app
