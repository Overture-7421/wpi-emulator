#pragma once

#include <QByteArray>
#include <QFileSystemWatcher>
#include <QObject>
#include <QString>
#include <QTimer>

#include <memory>
#include <functional>
#include <optional>

#include "common/config_repository.h"
#include "common/project_inspector.h"
#include "common/project_prefs.h"
#include "common/recent_projects.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::app {

// The project the user is working on: where it is, what we know about it and its robot model.
// Loading, saving and "recent" bookkeeping live here so that no widget has to know about files.
//
// The robot model is watched on disk: when <project>/.wpiemu/robot.json changes outside the app (a git pull,
// a teammate's file copied in, a hand edit) it is loaded again, the same way opening the project loads it.
// The app's own saves are recognised by content and never reload. With unsaved edits open, nothing is
// replaced: externalChangeConflict() asks, and the answer is reloadFromDisk() or keepLocalChanges().
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
    const ProjectPrefs& prefs() const { return prefs_; }
    // Game module selected for this project (config.gamePluginId, else by season); nullptr = core-only.
    const sdk::IGameModule* gameModule() const;

    // Inspects the folder, loads its stored robot model when there is one and records it as recent.
    sdk::Status open(const QString& path);
    sdk::Status close();
    // Composition installs the optional editor's result-aware flush. It runs before identity/model replacement.
    void setPendingSaveHandler(std::function<sdk::Status()> handler) { pendingSaveHandler_ = std::move(handler); }
    sdk::Status flushPendingChanges();
    // Composition installs the editor's "has unsaved edits" check, so an outside change never silently replaces them.
    void setUnsavedChangesQuery(std::function<bool()> query) { unsavedChangesQuery_ = std::move(query); }

    // Loads the robot model from disk now, replacing the one open and any unsaved edits.
    sdk::Status reloadFromDisk();
    // Keeps the model open over a file changed on disk; the next save writes it over that file.
    void keepLocalChanges();

    // Persists the model to <project>/.wpiemu/robot.json and makes it the current one.
    SaveOutcome commit(const sdk::RobotConfig& config, const ProjectPrefs& prefs);
    // Re-reads the project's files (e.g. after vendordeps changed).
    void refreshInspection();

signals:
    void projectChanged();   // opened / closed
    void configChanged();    // model replaced or saved
    void saveFailed(const QString& message);
    // robot.json changed on disk while the editor holds unsaved edits: ask, then reloadFromDisk() or keepLocalChanges().
    void externalChangeConflict();
    // robot.json changed on disk but was not loaded (unreadable or deleted); the model open stays open.
    void externalChangeIgnored(const QString& message);

private:
    void watchConfigFile();
    void checkConfigFile();
    QByteArray readConfigFile() const;

    IProjectInspector& inspector_;
    ConfigRepository& repository_;
    RecentProjects& recent_;
    ProjectPrefsStore& prefsStore_;
    const sdk::GamePluginRegistry& plugins_;

    QString path_;
    ProjectInspection inspection_;
    std::shared_ptr<const sdk::RobotConfig> config_;
    QString loadError_;
    ProjectPrefs prefs_;
    std::function<sdk::Status()> pendingSaveHandler_;
    std::function<bool()> unsavedChangesQuery_;

    QFileSystemWatcher watcher_;
    QTimer settleTimer_;       // a pull or an atomic save is several file events; act once they settle
    QByteArray knownContent_;  // robot.json as last loaded or saved here; any other content came from outside
    bool conflictOpen_ = false;
};

}  // namespace wpiemu::app
