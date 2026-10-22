#pragma once

#include <QString>

#include <filesystem>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/config_store.h"

namespace wpiemu::app {

struct SaveOutcome {
    bool saved = false;
    QString path;    // <project>/.wpiemu/robot.json
    QString error;

    bool ok() const { return saved; }
};

// Qt-facing facade over core's ProjectConfigRepository (<project>/.wpiemu/robot.json, the only copy of the robot
// setup) and ConfigPipeline (defaults -> validation). All path handling is delegated to core so the editor, the
// wizard and the launcher agree on locations.
class ConfigRepository {
public:
    ConfigRepository(std::filesystem::path appDataRoot, const sdk::IValueTextCodec& text,
                     const core::config::RobotConfigCodec& codec, const core::config::ConfigPipeline& pipeline);

    QString configPath(const QString& projectPath) const;

    // A robot setup is in the project, or an older version left one in app data that opening moves in.
    bool exists(const QString& projectPath) const;
    // Opens the project's robot setup and runs the load pipeline.
    sdk::Result<core::config::LoadedConfig> load(const QString& projectPath) const;
    // Writes the project's robot setup.
    SaveOutcome save(const QString& projectPath, const sdk::RobotConfig& config) const;

private:
    core::config::ProjectConfigRepository repo_;
    const core::config::ConfigPipeline& pipeline_;
};

}  // namespace wpiemu::app
