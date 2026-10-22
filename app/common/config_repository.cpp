#include "common/config_repository.h"

#include <QFileInfo>

namespace wpiemu::app {

namespace {

std::filesystem::path toPath(const QString& s) { return std::filesystem::path(s.toStdU16String()); }

QString fromPath(const std::filesystem::path& p) { return QString::fromStdU16String(p.generic_u16string()); }

}  // namespace

ConfigRepository::ConfigRepository(std::filesystem::path appDataRoot, const sdk::IValueTextCodec& text,
                                   const core::config::RobotConfigCodec& codec, const core::config::ConfigPipeline& pipeline)
    : repo_(std::move(appDataRoot), text, codec), pipeline_(pipeline) {}

QString ConfigRepository::configPath(const QString& projectPath) const {
    return fromPath(repo_.projectStore(toPath(projectPath)).path());
}

bool ConfigRepository::exists(const QString& projectPath) const {
    return QFileInfo::exists(configPath(projectPath)) ||
           QFileInfo::exists(fromPath(repo_.legacyAppDataStore(toPath(projectPath)).path()));
}

sdk::Result<core::config::LoadedConfig> ConfigRepository::load(const QString& projectPath) const {
    auto opened = repo_.open(toPath(projectPath));
    if (!opened) return opened.error();
    return pipeline_.process(std::move(opened.value()));
}

SaveOutcome ConfigRepository::save(const QString& projectPath, const sdk::RobotConfig& config) const {
    SaveOutcome out;
    out.path = configPath(projectPath);
    if (auto s = repo_.save(toPath(projectPath), config); s) out.saved = true;
    else out.error = QString::fromStdString(s.error().message);
    return out;
}

}  // namespace wpiemu::app
