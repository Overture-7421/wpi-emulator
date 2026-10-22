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

// Where versions before the project file kept a project's robot setup. Read only to move it into the project.
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
    // (the repository keeps an unreadable or migrated file this way).
    sdk::Result<std::filesystem::path> backup(std::string_view tag) const;

private:
    std::filesystem::path file_;
    std::string label_;
    const sdk::IValueTextCodec& text_;
    const RobotConfigCodec& codec_;
    sdk::ILogger* logger_;
};

enum class ConfigSource { Project, MigratedFromAppData };

// The robot setup of one project. It lives in the project, at `<project>/.wpiemu/robot.json`, the way PathPlanner
// keeps its files in deploy/pathplanner: what is in version control is what runs, and a pull takes effect the next
// time the project is opened. There is no second copy.
//  * open(): the project file, else NotFound. An unreadable file is an error, never replaced by anything else.
//    A project with no file but a setup left in app data by an older version gets that setup moved into the
//    project, once; the app-data file is kept beside itself as "robot.json.migrated.bak".
//  * save(): writes the project file. An existing file that does not load is first kept as
//    "robot.json.unreadable.bak", so finishing setup after a load error never destroys someone's work.
class ProjectConfigRepository {
public:
    // `appDataRoot` is only read to move an older version's setup into the project.
    ProjectConfigRepository(std::filesystem::path appDataRoot, const sdk::IValueTextCodec& text, const RobotConfigCodec& codec,
                            sdk::ILogger* logger = nullptr)
        : root_(std::move(appDataRoot)), text_(text), codec_(codec), logger_(logger) {}

    FileConfigStore projectStore(const std::filesystem::path& projectDir) const;
    FileConfigStore legacyAppDataStore(const std::filesystem::path& projectDir) const;

    sdk::Result<sdk::RobotConfig> open(const std::filesystem::path& projectDir, ConfigSource* usedSource = nullptr) const;
    sdk::Status save(const std::filesystem::path& projectDir, const sdk::RobotConfig& config) const;

private:
    std::filesystem::path root_;
    const sdk::IValueTextCodec& text_;
    const RobotConfigCodec& codec_;
    sdk::ILogger* logger_;
};

}  // namespace wpiemu::core::config
