#pragma once

#include <QString>

#include <filesystem>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/config_store.h"

namespace wpiemu::app {

struct SaveOutcome {
    bool appDataSaved = false;
    bool exportSaved = false;
    QString appDataPath;
    QString exportPath;
    QString appDataError;
    QString exportError;

    // The app-data copy is authoritative; a read-only project folder only produces a warning.
    bool ok() const { return appDataSaved; }
    bool hasWarning() const { return appDataSaved && !exportSaved; }
};

// Qt-facing facade over core's ProjectConfigRepository (app-data copy + <project>/.wpiemu/robot.json)
// and ConfigPipeline (migration -> defaults -> validation). All path handling is delegated to core so
// the editor, the wizard and the launcher agree on locations.
class ConfigRepository {
public:
    ConfigRepository(std::filesystem::path appDataRoot, const sdk::IValueTextCodec& text,
                     const core::config::RobotConfigCodec& codec, const core::config::ConfigPipeline& pipeline);

    QString appDataPath(const QString& projectPath) const;
    QString exportPath(const QString& projectPath) const;

    bool exists(const QString& projectPath) const;
    // Opens the stored model (app-data copy first, then the project export) and runs the load pipeline.
    sdk::Result<core::config::LoadedConfig> load(const QString& projectPath) const;
    // Writes the app-data copy, then exports to the project folder (failure of the latter is a warning).
    SaveOutcome save(const QString& projectPath, const sdk::RobotConfig& config) const;

private:
    core::config::ProjectConfigRepository repo_;
    const core::config::ConfigPipeline& pipeline_;
};

}  // namespace wpiemu::app
