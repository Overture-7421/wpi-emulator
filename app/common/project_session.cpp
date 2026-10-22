#include "common/project_session.h"

#include <QFile>
#include <QFileInfo>

#include "common/project_path.h"

namespace wpiemu::app {

ProjectSession::ProjectSession(IProjectInspector& inspector, ConfigRepository& repository, RecentProjects& recent,
                               ProjectPrefsStore& prefs, const sdk::GamePluginRegistry& plugins, QObject* parent)
    : QObject(parent), inspector_(inspector), repository_(repository), recent_(recent), prefsStore_(prefs), plugins_(plugins) {
    settleTimer_.setSingleShot(true);
    settleTimer_.setInterval(300);
    connect(&settleTimer_, &QTimer::timeout, this, &ProjectSession::checkConfigFile);
    connect(&watcher_, &QFileSystemWatcher::fileChanged, &settleTimer_, qOverload<>(&QTimer::start));
    connect(&watcher_, &QFileSystemWatcher::directoryChanged, &settleTimer_, qOverload<>(&QTimer::start));
}

const sdk::IGameModule* ProjectSession::gameModule() const {
    if (!hasProject()) return nullptr;
    if (config_ && !config_->gamePluginId.empty()) return plugins_.find(config_->gamePluginId);
    const int year = config_ && config_->project.wpilibYear > 0 ? config_->project.wpilibYear : inspection_.wpilibYear;
    return year > 0 ? plugins_.findByYear(year) : nullptr;
}

sdk::Status ProjectSession::open(const QString& rawPath) {
    const QString root = GradleProjectInspector::resolveProjectRoot(rawPath);
    ProjectInspection inspection = inspector_.inspect(root);
    if (!inspection.usable()) {
        const QString why = inspection.notes.empty() ? QStringLiteral("not a WPILib project") : inspection.notes.front().text;
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, why.toStdString());
    }

    if (auto flushed = flushPendingChanges(); !flushed) return flushed;
    path_ = normalizedProjectPath(root);
    inspection_ = std::move(inspection);
    prefs_ = prefsStore_.load(path_);
    config_.reset();
    loadError_.clear();
    if (repository_.exists(path_)) {
        auto loaded = repository_.load(path_);
        if (loaded) {
            config_ =  std::make_shared<const sdk::RobotConfig>(std::move(loaded.value().config));
        } else {
            loadError_ = QString::fromStdString(loaded.error().message);
        }
    }
    knownContent_ = readConfigFile();
    conflictOpen_ = false;
    watchConfigFile();
    recent_.touch(path_);
    emit projectChanged();
    emit configChanged();
    return sdk::okStatus();
}

sdk::Status ProjectSession::flushPendingChanges() {
    return pendingSaveHandler_ ? pendingSaveHandler_() : sdk::okStatus();
}

sdk::Status ProjectSession::close() {
    if (!hasProject()) return sdk::okStatus();
    if (auto flushed = flushPendingChanges(); !flushed) return flushed;
    path_.clear();
    inspection_ = {};
    config_.reset();
    loadError_.clear();
    knownContent_.clear();
    conflictOpen_ = false;
    settleTimer_.stop();
    watchConfigFile();
    emit projectChanged();
    emit configChanged();
    return sdk::okStatus();
}

SaveOutcome ProjectSession::commit(const sdk::RobotConfig& config, const ProjectPrefs& prefs) {
    SaveOutcome outcome = repository_.save(path_, config);
    prefs_ = prefs;
    prefsStore_.save(path_, prefs);
    if (!outcome.ok()) {
        emit saveFailed(outcome.error);
        return outcome;
    }
    config_ = std::make_shared<const sdk::RobotConfig>(config);
    loadError_.clear();
    knownContent_ = readConfigFile();   // what we just wrote: the watcher's events for it change nothing
    conflictOpen_ = false;
    watchConfigFile();
    emit configChanged();
    return outcome;
}

sdk::Status ProjectSession::reloadFromDisk() {
    conflictOpen_ = false;
    if (!hasProject()) return sdk::okStatus();
    const QByteArray content = readConfigFile();
    auto loaded = repository_.load(path_);
    knownContent_ = content;   // either way, this content has been dealt with: do not ask about it again
    if (!loaded) {
        emit externalChangeIgnored(
            tr("The robot setup changed on disk but could not be read: %1. The setup that was open stays open; "
               "saving replaces the file and keeps the unreadable one beside it as robot.json.unreadable.bak.")
                .arg(QString::fromStdString(loaded.error().message)));
        return loaded.error();
    }
    config_ = std::make_shared<const sdk::RobotConfig>(std::move(loaded.value().config));
    loadError_.clear();
    emit configChanged();
    return sdk::okStatus();
}

void ProjectSession::keepLocalChanges() {
    conflictOpen_ = false;
    knownContent_ = readConfigFile();
}

QByteArray ProjectSession::readConfigFile() const {
    if (!hasProject()) return {};
    QFile file(repository_.configPath(path_));
    if (!file.open(QIODevice::ReadOnly)) return {};
    return file.readAll();
}

void ProjectSession::watchConfigFile() {
    if (const QStringList watched = watcher_.files() + watcher_.directories(); !watched.isEmpty()) watcher_.removePaths(watched);
    if (!hasProject()) return;
    // The file and its folder: an atomic save or a checkout replaces the file, which ends a watch on the file
    // alone. Without a .wpiemu folder yet, the project folder, so a pull that brings one in is seen.
    const QString file = repository_.configPath(path_);
    const QString folder = QFileInfo(file).absolutePath();
    if (QFileInfo::exists(file)) watcher_.addPath(file);
    watcher_.addPath(QFileInfo::exists(folder) ? folder : path_);
}

void ProjectSession::checkConfigFile() {
    watchConfigFile();
    // Before setup has produced a model (or after the stored one failed to load) the wizard owns the file.
    if (!hasConfig()) return;
    if (!QFileInfo::exists(repository_.configPath(path_))) {
        if (knownContent_.isEmpty()) return;
        knownContent_.clear();
        emit externalChangeIgnored(tr("The robot setup file %1 was deleted outside the emulator. The setup stays open; "
                                      "saving writes the file again.")
                                       .arg(repository_.configPath(path_)));
        return;
    }
    const QByteArray content = readConfigFile();
    if (content == knownContent_) return;   // our own save, or touched without a change
    if (unsavedChangesQuery_ && unsavedChangesQuery_()) {
        if (!conflictOpen_) {
            conflictOpen_ = true;
            emit externalChangeConflict();
        }
        return;
    }
    (void)reloadFromDisk();
}

void ProjectSession::refreshInspection() {
    if (!hasProject()) return;
    inspection_ = inspector_.inspect(path_);
    emit projectChanged();
}

}  // namespace wpiemu::app
