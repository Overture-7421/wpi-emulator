#pragma once

#include <QObject>
#include <QString>

#include "common/app_services.h"

namespace wpiemu::app {

enum class AppMode { Editor, Run };

enum class OpenOutcome {
    Opened,       // project open with a robot model: go to the editor
    NeedsSetup,   // WPILib project without a (readable) model: run the wizard
    Invalid,      // not a WPILib project: the wizard's first step explains why
};

// Application-level behaviour behind the main window (MVVM view-model): which project is open,
// which mode is showing, and keeping the runtime in step with the project. No widgets.
class ShellController : public QObject {
    Q_OBJECT
public:
    explicit ShellController(AppServices& services, QObject* parent = nullptr);

    AppMode mode() const { return mode_; }
    bool hasProject() const { return services_.session().hasProject(); }
    bool canRun() const { return services_.session().hasConfig(); }
    QString projectName() const;
    QString windowTitle() const;
    // Short text for the status bar's game chip ("REBUILT (2026)" / "Core-only" / empty without a project).
    QString gameLabel() const;
    bool gameModuleAvailable() const { return services_.session().gameModule() != nullptr; }
    QString runtimeLabel() const;

    void setMode(AppMode mode);

    // Opens `path` (a project folder, or a file inside one).
    OpenOutcome openProject(const QString& path);
    // Reopens the previous project at start-up when enabled; silent when it is gone.
    bool restoreLastProject();
    void closeProject();

    // Called after the wizard saved a model: switches to the editor and optionally detects devices.
    void setupCompleted(bool detectDevices);
    void startDiscovery();
    void startRobot();
    void stopRobot();

signals:
    void modeChanged(wpiemu::app::AppMode mode);
    void projectChanged();
    void runtimeChanged();
    void notice(const QString& message);
    void errorOccurred(const QString& message);

private:
    void syncRuntime();

    AppServices& services_;
    AppMode mode_ = AppMode::Editor;
};

}  // namespace wpiemu::app
