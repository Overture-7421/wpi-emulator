#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/sdk/config/config_io.h"
#include "wpiemu/sdk/core/environment.h"
#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::core::config {

// ---- platform locations (no Qt) --------------------------------------------------------------------

enum class Platform { Windows, MacOS, Linux };
Platform currentPlatform();

// Per-user application data root of the emulator:
//   Windows  %APPDATA%\WpiEmulator            (fallback %USERPROFILE%\AppData\Roaming)
//   macOS    $HOME/Library/Application Support/WpiEmulator
//   Linux    $XDG_DATA_HOME/wpi-emulator      (fallback $HOME/.local/share)
// `env` and `platform` are parameters so every branch is testable on one machine. NotFound when the
// environment offers no home directory.
sdk::Result<std::filesystem::path> appDataRoot(Platform platform, const sdk::EnvLookup& env);

// Key of a project directory: "<sanitised folder name>-<16 hex>", the FNV-1a 64 hash of its canonical path
// (lower-cased on Windows). Stable across runs; distinct projects with the same folder name never collide.
std::string projectKeyForPath(std::string_view canonicalPath, bool caseInsensitive);
std::string projectKey(const std::filesystem::path& projectDir);  // canonicalises first

std::filesystem::path appDataConfigPath(const std::filesystem::path& appDataRoot, const std::filesystem::path& projectDir);
std::filesystem::path projectConfigPath(const std::filesystem::path& projectDir);  // <project>/.wpiemu/robot.json

// ---- file helpers ---------------------------------------------------------------------------------

// Writes `content` to a temp file next to `target` (flushed to disk), then renames it over `target`; the
// target is either the old or the new content, never a torn mix. Creates missing parent directories.
sdk::Status atomicWriteFile(const std::filesystem::path& target, std::string_view content);
sdk::Result<std::string> readTextFile(const std::filesystem::path& file);  // NotFound when absent

// ---- stores -----------------------------------------------------------------------------------------

// One robot.json on disk. load(): NotFound when absent; a file with another schemaVersion is refused (no migration).
class FileConfigStore final : public sdk::IRobotConfigStore {
public:
    FileConfigStore(std::filesystem::path file, std::string label, const sdk::IValueTextCodec& text, const RobotConfigCodec& codec,
                    sdk::ILogger* logger = nullptr);

    std::string describe() const override { return label_ + " (" + file_.string() + ")"; }
    sdk::Result<sdk::RobotConfig> load() override;
    sdk::Status save(const sdk::RobotConfig& config) override;

    const std::filesystem::path& path() const { return file_; }
    bool exists() const;
    // Copies the current file to "<file>.<tag>.bak" ("-2", "-3"... when taken). Returns the backup path
    // (importFromProject keeps the app-data copy this way before replacing it).
    sdk::Result<std::filesystem::path> backup(std::string_view tag) const;

private:
    std::filesystem::path file_;
    std::string label_;
    const sdk::IValueTextCodec& text_;
    const RobotConfigCodec& codec_;
    sdk::ILogger* logger_;
};

enum class ConfigSource { AppData, Project };

// The two stores of one project behind one object: the per-user working copy in app data (keyed by the project
// path hash) and the shareable `<project>/.wpiemu/robot.json` export.
//  * open(): app-data copy when present, else the project export (imported into app data), else NotFound.
//  * save(): app-data copy (always) and, when `alsoExport`, the project export.
class ProjectConfigRepository {
public:
    ProjectConfigRepository(std::filesystem::path appDataRoot, const sdk::IValueTextCodec& text, const RobotConfigCodec& codec,
                            sdk::ILogger* logger = nullptr)
        : root_(std::move(appDataRoot)), text_(text), codec_(codec), logger_(logger) {}

    FileConfigStore appDataStore(const std::filesystem::path& projectDir) const;
    FileConfigStore projectStore(const std::filesystem::path& projectDir) const;

    sdk::Result<sdk::RobotConfig> open(const std::filesystem::path& projectDir, ConfigSource* usedSource = nullptr) const;
    sdk::Status save(const std::filesystem::path& projectDir, const sdk::RobotConfig& config, bool alsoExport = false) const;
    sdk::Status exportToProject(const std::filesystem::path& projectDir) const;     // app-data copy -> project
    sdk::Result<sdk::RobotConfig> importFromProject(const std::filesystem::path& projectDir) const;  // project -> app-data copy

private:
    std::filesystem::path root_;
    const sdk::IValueTextCodec& text_;
    const RobotConfigCodec& codec_;
    sdk::ILogger* logger_;
};

}  // namespace wpiemu::core::config
