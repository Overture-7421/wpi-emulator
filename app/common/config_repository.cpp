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

QString ConfigRepository::appDataPath(const QString& projectPath) const {
    return fromPath(repo_.appDataStore(toPath(projectPath)).path());
}

QString ConfigRepository::exportPath(const QString& projectPath) const {
    return fromPath(repo_.projectStore(toPath(projectPath)).path());
}

bool ConfigRepository::exists(const QString& projectPath) const {
    return QFileInfo::exists(appDataPath(projectPath)) || QFileInfo::exists(exportPath(projectPath));
}

sdk::Result<core::config::LoadedConfig> ConfigRepository::load(const QString& projectPath) const {
    core::config::DecodeInfo decode;
    auto opened = repo_.open(toPath(projectPath), nullptr, &decode);
    if (!opened) return opened.error();
    return pipeline_.process(std::move(opened.value()), decode);
}

SaveOutcome ConfigRepository::save(const QString& projectPath, const sdk::RobotConfig& config) const {
    SaveOutcome out;
    out.appDataPath = appDataPath(projectPath);
    out.exportPath = exportPath(projectPath);
    if (auto s = repo_.save(toPath(projectPath), config, false); s) out.appDataSaved = true;
    else out.appDataError = QString::fromStdString(s.error().message);
    if (out.appDataSaved) {
        if (auto s = repo_.exportToProject(toPath(projectPath)); s) out.exportSaved = true;
        else out.exportError = QString::fromStdString(s.error().message);
    }
    return out;
}

}  // namespace wpiemu::app
