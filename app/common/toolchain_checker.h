#pragma once

#include <QCoreApplication>
#include <QString>

#include <optional>
#include <vector>

#include "common/system_probe.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::app {

enum class CheckStatus { Ok, Warning, Missing, NotApplicable };

namespace check_id {
inline constexpr const char* kJdk = "jdk";
inline constexpr const char* kGradle = "gradle";
inline constexpr const char* kWpilib = "wpilib";
inline constexpr const char* kCppToolchain = "cpp";
inline constexpr const char* kGameModule = "game";
}  // namespace check_id

struct ToolchainCheck {
    QString id;
    QString title;
    CheckStatus status = CheckStatus::Warning;
    QString detail;          // what was found / why it failed
    QString fixHint;         // how to fix it (empty when Ok)
    bool requiredToRun = false;   // the robot cannot be launched without it
};

struct ToolchainInput {
    QString projectPath;
    QString language;          // "java" | "cpp" | ""
    int wpilibYear = 0;
    QString wpilibHomeOverride;
    QString jdkHomeOverride;
};

struct ToolchainReport {
    std::vector<ToolchainCheck> checks;
    // Game plugin selected for the project's season; empty id = core-only mode.
    QString gamePluginId;
    QString gamePluginName;

    const ToolchainCheck* find(const QString& id) const {
        for (const auto& c : checks)
            if (c.id == id) return &c;
        return nullptr;
    }
    // Every check the robot launch depends on passes.
    bool canRunRobot() const {
        for (const auto& c : checks)
            if (c.requiredToRun && c.status != CheckStatus::Ok && c.status != CheckStatus::NotApplicable) return false;
        return true;
    }
    bool coreOnly() const { return gamePluginId.isEmpty(); }
};

// Pure logic over ISystemProbe + the plugin registry: no widgets, easily tested with fakes.
class ToolchainChecker {
    Q_DECLARE_TR_FUNCTIONS(ToolchainChecker)
public:
    ToolchainChecker(const ISystemProbe& probe, const sdk::GamePluginRegistry& plugins)
        : probe_(probe), plugins_(plugins) {}

    ToolchainReport run(const ToolchainInput& input) const;

    ToolchainCheck checkJdk(const ToolchainInput& input) const;
    ToolchainCheck checkGradleWrapper(const ToolchainInput& input) const;
    ToolchainCheck checkWpilibInstall(const ToolchainInput& input) const;
    ToolchainCheck checkCppToolchain(const ToolchainInput& input) const;
    ToolchainCheck checkGameModule(const ToolchainInput& input, ToolchainReport& report) const;

    // Existing WPILib install folder for the season (override, then the default location), else empty.
    QString findWpilibHome(const ToolchainInput& input) const;
    // Where the installer puts WPILib by default for `year` (may not exist).
    QString defaultWpilibHome(int year) const;

    // `java -version` text -> major version (8, 11, 17, 21...); nullopt when unparseable.
    static std::optional<int> parseJavaMajor(const QString& versionOutput);

private:
    const ISystemProbe& probe_;
    const sdk::GamePluginRegistry& plugins_;
};

}  // namespace wpiemu::app
