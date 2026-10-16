#pragma once

#include <QObject>
#include <QString>

#include <memory>
#include <optional>

#include "common/config_repository.h"
#include "common/project_inspector.h"
#include "common/project_prefs.h"
#include "common/recent_projects.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::app {

// The project the user is working on: where it is, what we know about it and its robot model.
// Loading, saving and "recent" bookkeeping live here so that no widget has to know about files.
class ProjectSession : public QObject {
    Q_OBJECT
public:
    ProjectSession(IProjectInspector& inspector, ConfigRepository& repository, RecentProjects& recent,
                   ProjectPrefsStore& prefs, const sdk::GamePluginRegistry& plugins, QObject* parent = nullptr);

    bool hasProject() const { return !path_.isEmpty(); }
    const QString& path() const { return path_; }
    const ProjectInspection& inspection() const { return inspection_; }

    bool hasConfig() const { return config_ != nullptr; }
    // A project is open but no (readable) robot model exists yet: the setup wizard is due.
    bool needsSetup() const { return hasProject() && !hasConfig(); }
    std::shared_ptr<const sdk::RobotConfig> config() const { return config_; }
    const QString& loadError() const { return loadError_; }
    // Validation findings of the stored model at load time (the editor shows live ones itself).
    const std::vector<sdk::Issue>& loadIssues() const { return loadIssues_; }
    const ProjectPrefs& prefs() const { return prefs_; }
    // Game module selected for this project (config.gamePluginId, else by season); nullptr = core-only.
    const sdk::IGameModule* gameModule() const;

    // Inspects the folder, loads its stored robot model when there is one and records it as recent.
    sdk::Status open(const QString& path);
    void close();

    // Persists the model (app-data + <project>/.wpiemu/robot.json) and makes it the current one.
    SaveOutcome commit(const sdk::RobotConfig& config, const ProjectPrefs& prefs);
    // Re-reads the project's files (e.g. after vendordeps changed).
    void refreshInspection();

signals:
    void projectChanged();   // opened / closed
    void configChanged();    // model replaced or saved
    void saveFailed(const QString& message);

private:
    IProjectInspector& inspector_;
    ConfigRepository& repository_;
    RecentProjects& recent_;
    ProjectPrefsStore& prefsStore_;
    const sdk::GamePluginRegistry& plugins_;

    QString path_;
    ProjectInspection inspection_;
    std::shared_ptr<const sdk::RobotConfig> config_;
    QString loadError_;
    std::vector<sdk::Issue> loadIssues_;
    ProjectPrefs prefs_;
};

}  // namespace wpiemu::app
