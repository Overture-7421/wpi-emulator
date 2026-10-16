#include "common/project_session.h"

#include <QDir>
#include <QFileInfo>

namespace wpiemu::app {

ProjectSession::ProjectSession(IProjectInspector& inspector, ConfigRepository& repository, RecentProjects& recent,
                               ProjectPrefsStore& prefs, const sdk::GamePluginRegistry& plugins, QObject* parent)
    : QObject(parent), inspector_(inspector), repository_(repository), recent_(recent), prefsStore_(prefs), plugins_(plugins) {}

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

    path_ = QDir::cleanPath(QFileInfo(root).absoluteFilePath());
    inspection_ = std::move(inspection);
    prefs_ = prefsStore_.load(path_);
    config_.reset();
    loadError_.clear();
    loadIssues_.clear();
    if (repository_.exists(path_)) {
        auto loaded = repository_.load(path_);
        if (loaded) {
            loadIssues_ = loaded.value().issues;
            config_ = std::make_shared<const sdk::RobotConfig>(std::move(loaded.value().config));
        } else {
            loadError_ = QString::fromStdString(loaded.error().message);
        }
    }
    recent_.touch(path_);
    emit projectChanged();
    emit configChanged();
    return sdk::okStatus();
}

void ProjectSession::close() {
    if (!hasProject()) return;
    path_.clear();
    inspection_ = {};
    config_.reset();
    loadError_.clear();
    loadIssues_.clear();
    emit projectChanged();
    emit configChanged();
}

SaveOutcome ProjectSession::commit(const sdk::RobotConfig& config, const ProjectPrefs& prefs) {
    SaveOutcome outcome = repository_.save(path_, config);
    prefs_ = prefs;
    prefsStore_.save(path_, prefs);
    if (!outcome.ok()) {
        emit saveFailed(outcome.appDataError);
        return outcome;
    }
    config_ = std::make_shared<const sdk::RobotConfig>(config);
    loadError_.clear();
    loadIssues_.clear();
    emit configChanged();
    return outcome;
}

void ProjectSession::refreshInspection() {
    if (!hasProject()) return;
    inspection_ = inspector_.inspect(path_);
    emit projectChanged();
}

}  // namespace wpiemu::app
