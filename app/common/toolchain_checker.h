#pragma once

#include <QCoreApplication>
#include <QString>

#include <optional>
#include <vector>

#include "common/system_probe.h"
#include "wpiemu/core/launcher/jdk_policy.h"
#include "wpiemu/core/launcher/project_inspector.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/season/season_registry.h"

namespace wpiemu::app {

enum class CheckStatus { Ok, Warning, Missing, NotApplicable };

// What the user can do about a failing row from the app itself (nothing is offered for a row that passes).
enum class FixAction { None, GetJava, InstallBuildTools, OpenSettings };

namespace check_id {
inline constexpr const char* kJdk = "jdk";
inline constexpr const char* kGradle = "gradle";
inline constexpr const char* kWpilib = "wpilib";
inline constexpr const char* kCppToolchain = "cpp";
inline constexpr const char* kGameModule = "game";
inline constexpr const char* kWpilibVersion = "wpilib_version";
inline constexpr const char* kHalExtension = "no_extension_for_season";
}  // namespace check_id

struct ToolchainCheck {
    QString id;
    QString title;
    CheckStatus status = CheckStatus::Warning;
    QString detail;          // what was found / why it failed
    QString fixHint;         // how to fix it (empty when Ok)
    bool requiredToRun = false;   // the robot cannot be launched without it
    FixAction fixAction = FixAction::None;
};

struct ToolchainInput {
    QString projectPath;
    QString language;          // "java" | "cpp" | ""
    int wpilibYear = 0;
    QString wpilibVersion;     // the GradleRIO version the project declares; empty = unknown
    QString gradleVersion;     // the wrapper's Gradle; empty = unknown (assumes the WPILib template's)
    int minJavaMajor = core::launcher::kMinJdkMajor;   // the Java the project's season builds with (ToolchainProvisioner::javaMajorForSeason)
    QString wpilibHomeOverride;
    QString jdkHomeOverride;
    QString managedJdkHome;    // the emulator's downloaded Java when one is installed (empty = none)
    bool managedJavaAvailable = false;   // the emulator can download Java for this computer (a pin exists for it)
    bool buildToolsInstallAvailable = false;   // the emulator can offer to install the C++ build tools on this computer
    std::optional<QString> extensionLibrary;   // unset = not inspected; empty = no matching extension installed
    QStringList installedExtensionSeasons;
};

// The JDK a launch will use, and why other candidates were passed over.
struct JdkSkip {
    QString path;       // java executable or home that was looked at
    QString reason;     // plain-language, worded for the Toolchain page
};

struct JdkResolution {
    bool found = false;
    core::launcher::JdkSource source = core::launcher::JdkSource::Path;
    QString java;       // the java executable
    QString home;       // the JDK folder for JAVA_HOME / org.gradle.java.home; empty when only `java` on PATH is known
    int major = 0;
    bool explicitFailed = false;   // the Settings folder was chosen but cannot run the project: nothing else is tried
    std::vector<JdkSkip> skipped;
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
    ToolchainChecker(const ISystemProbe& probe, const sdk::GamePluginRegistry& plugins, const sdk::SeasonRegistry& seasons)
        : probe_(probe), plugins_(plugins), seasons_(seasons) {}

    ToolchainReport run(const ToolchainInput& input) const;

    ToolchainCheck checkJdk(const ToolchainInput& input) const;
    ToolchainCheck checkGradleWrapper(const ToolchainInput& input) const;
    ToolchainCheck checkWpilibInstall(const ToolchainInput& input) const;
    ToolchainCheck checkCppToolchain(const ToolchainInput& input) const;
    ToolchainCheck checkGameModule(const ToolchainInput& input, ToolchainReport& report) const;
    // Warns when the project's WPILib version is not one its season pack verified; the run continues.
    ToolchainCheck checkWpilibVersion(const ToolchainInput& input) const;

    // Existing WPILib install folder for the season (override, then the installer's locations in core's order), else empty.
    QString findWpilibHome(const ToolchainInput& input) const;
    // Where the installer puts WPILib by default for `year` (may not exist).
    QString defaultWpilibHome(int year) const;

    // The java Gradle will be launched with: the first usable source of core's JDK order (see jdk_policy.h for "usable").
    JdkResolution resolveJdk(const ToolchainInput& input) const;
    // `java -version` text -> major version (8, 11, 17, 21...); nullopt when unparseable.
    static std::optional<int> parseJavaMajor(const QString& versionOutput);

private:
    // WPILib roots to look in, as core's launcher lists them for this machine.
    QStringList wpilibRoots() const;
    // Judges the JDK at `home` (java at <home>/bin); `trusted` skips the Gradle-compatibility and runtime checks.
    std::optional<QString> judgeJdk(const QString& java, bool trusted, const ToolchainInput& input, int* majorOut) const;

    const ISystemProbe& probe_;
    // WPILib installer folder of a season: its season pack's folder, else the year.
    QString installFolder(int year) const;

    const sdk::GamePluginRegistry& plugins_;
    const sdk::SeasonRegistry& seasons_;
};

}  // namespace wpiemu::app
