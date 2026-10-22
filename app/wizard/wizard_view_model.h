#pragma once

#include <QObject>
#include <QString>

#include <memory>
#include <vector>

#include "common/app_services.h"
#include "common/robot_basics.h"
#include "common/toolchain_checker.h"

class QThread;

namespace wpiemu::app {

enum class WizardStep { Project = 0, Toolchain = 1, Robot = 2, Summary = 3 };
inline constexpr int kWizardStepCount = 4;

struct WizardResult {
    bool ok = false;
    QString message;          // problem description when !ok, warning text when saved with a warning
    SaveOutcome outcome;
    bool detectDevices = false;
};

// Runs ToolchainChecker off the UI thread. Each run() supersedes the previous one: a stale result is
// dropped, so a slow check can never overwrite a newer one.
class ToolchainRunner : public QObject {
    Q_OBJECT
public:
    explicit ToolchainRunner(const ToolchainChecker& checker, QObject* parent = nullptr);
    ~ToolchainRunner() override;

    void run(const ToolchainInput& input);
    bool running() const { return running_; }

signals:
    void started();
    void finished(const wpiemu::app::ToolchainReport& report);

private:
    void deliver(quint64 generation, const ToolchainReport& report);

    const ToolchainChecker& checker_;
    quint64 generation_ = 0;
    bool running_ = false;
    std::vector<QThread*> threads_;
};

// All wizard state and rules; the pages are thin views over it (MVVM). Nothing touches disk until
// finish(): cancelling is always safe.
class WizardViewModel : public QObject {
    Q_OBJECT
public:
    explicit WizardViewModel(AppServices& services, QObject* parent = nullptr);
    ~WizardViewModel() override;

    // ---- navigation ----
    WizardStep step() const { return step_; }
    bool canGoBack() const { return step_ != WizardStep::Project; }
    bool canGoNext() const;
    bool isLastStep() const { return step_ == WizardStep::Summary; }
    void goTo(WizardStep step);
    void next();
    void back();
    // Highest step the user may jump to from the sidebar.
    WizardStep furthestReachable() const;
    bool stepComplete(WizardStep step) const;

    // ---- step 1: project ----
    const QString& projectPath() const { return projectPath_; }
    const ProjectInspection& inspection() const { return inspection_; }
    bool projectValid() const { return inspection_.usable(); }
    bool hasExistingConfig() const { return existingConfig_; }
    // Accepts a folder or a file inside the project; resolves to the project root and inspects it.
    void setProjectPath(const QString& path);

    // ---- step 2: toolchain ----
    const ToolchainReport& toolchain() const { return toolchain_; }
    bool toolchainChecked() const { return toolchainChecked_; }
    bool toolchainRunning() const { return runner_.running(); }
    void recheckToolchain();

    // The emulator's own Java (ToolchainProvisioner), offered by a failing Java row. Nothing here starts on its own.
    int javaMajor() const;                    // the Java this project's season builds with (0 = the pins do not know the season)
    bool javaDownloadAvailable() const;       // a pinned download exists for that Java on this computer
    qint64 javaDownloadBytes() const;
    bool javaBusy() const;
    int javaPercent() const;                  // 0..100 while downloading, else -1
    QString javaFolder() const;               // where the emulator keeps its own Java
    QString javaError() const;                // why the last download failed; empty otherwise
    void getJava();                           // downloads it; the toolchain is re-checked when it is installed
    void cancelJava();
    const RobotBasics& basics() const { return basics_; }
    void setBasics(const RobotBasics& basics);
    // Switches the drivetrain type, loading that type's defaults but keeping name/alliance/units choices.
    void setDrivetrainKind(const std::string& kind);
    const std::vector<sdk::Issue>& basicsIssues() const { return issues_; }
    bool basicsValid() const { return !hasErrors(issues_); }
    UnitFormatter units() const;

    // ---- step 4: summary ----
    bool detectDevicesAfter() const { return detectAfter_; }
    void setDetectDevicesAfter(bool detect);
    // The robot can be launched for discovery (toolchain good). Discovery is offered, not forced.
    bool discoveryPossible() const;
    sdk::RobotConfig previewConfig() const;

    // Persists the model (app-data + <project>/.wpiemu/robot.json) and makes the project current.
    WizardResult finish();

signals:
    void stepChanged(wpiemu::app::WizardStep step);
    void projectChanged();
    void toolchainChanged();
    void javaChanged();
    void basicsChanged();
    void summaryChanged();

private:
    void applyProject(ProjectInspection inspection);
    void loadExistingBasics();
    void revalidateBasics();
    void startToolchainCheck();
    ToolchainInput toolchainInput() const;
    RobotBuildContext buildContext() const;

    AppServices& services_;
    ToolchainRunner runner_;

    WizardStep step_ = WizardStep::Project;
    QString projectPath_;
    ProjectInspection inspection_;
    bool existingConfig_ = false;
    ToolchainReport toolchain_;
    bool toolchainChecked_ = false;
    RobotBasics basics_;
    std::vector<sdk::Issue> issues_;
    bool detectAfter_ = true;
    bool basicsTouched_ = false;
};

}  // namespace wpiemu::app

Q_DECLARE_METATYPE(wpiemu::app::ToolchainReport)
