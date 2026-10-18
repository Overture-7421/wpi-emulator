#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/core/config/config_store.h"
#include "wpiemu/sdk/config/config_io.h"
#include "wpiemu/sdk/core/severity.h"

namespace wpiemu::core::launcher {

struct VendorDep {
    std::string fileName;     // vendordeps/<file>.json
    std::string name;         // "Phoenix 6", "photonlib", ...
    std::string version;
    std::string uuid;
    std::string frcYear;      // "2026" (empty when absent)
    bool readable = true;     // false: the file could not be parsed (still listed)
};

// What a read-only look at a robot project directory found. Nothing is run and nothing is written.
struct ProjectReport {
    std::filesystem::path root;
    std::string name;                      // folder name
    bool hasBuildScript = false;           // build.gradle / build.gradle.kts
    bool isWpilibProject = false;          // GradleRIO plugin found (or WPILib preferences present with a build script)
    bool gradleRioFound = false;
    std::string gradleRioVersion;          // "2026.2.1"
    int wpilibYear = 0;                    // from the GradleRIO version, else .wpilib prefs projectYear, else settings.gradle frcYear
    std::string language;                  // "java" | "cpp" | "" (undecided)
    int teamNumber = -1;                   // .wpilib/wpilib_preferences.json; -1 unknown
    bool hasGradlewScript = false;         // gradlew (POSIX) present
    bool hasGradlewBat = false;            // gradlew.bat present
    bool hasWrapperJar = false;
    bool hasWrapperProperties = false;
    std::string gradleVersion;             // from distributionUrl
    std::vector<VendorDep> vendorDeps;
    std::vector<sdk::Issue> notes;         // non-fatal oddities found while reading

    bool hasVendorDep(std::string_view nameFragment) const;  // case-insensitive substring of the dep name or file name
    // Version of the project's photonlib vendordep ("v2026.3.4"); empty when the project has none (or it declares no version).
    std::string photonLibVersion() const;
};

class ProjectInspector {
public:
    explicit ProjectInspector(const sdk::IValueTextCodec& json) : json_(json) {}
    // NotFound / InvalidArgument when `dir` is not a directory. A directory that is no WPILib project is NOT an error:
    // the report says isWpilibProject == false and why (notes).
    sdk::Result<ProjectReport> inspect(const std::filesystem::path& dir) const;

private:
    const sdk::IValueTextCodec& json_;
};

// ---- WPILib / toolchain discovery (the wizard's step 2) ----------------------------------------------------------------

struct WpilibInstall {
    std::filesystem::path home;        // <root>/<year>
    int year = 0;
    std::filesystem::path jdkHome;     // <home>/jdk when it contains bin/java; else empty
    bool hasMaven = false;             // offline maven repository (Gradle resolves GradleRIO from it)
    bool hasTools = false;
    bool hasRoboRioToolchain = false;
};

// <PUBLIC>\wpilib (C:\Users\Public\wpilib) on Windows, $HOME/wpilib on macOS and Linux.
std::vector<std::filesystem::path> wpilibRootCandidates(config::Platform platform, const config::EnvLookup& env);
// First root containing "<year>/" wins. `roots` empty => wpilibRootCandidates().
std::optional<WpilibInstall> findWpilibInstall(int year, const std::vector<std::filesystem::path>& roots);

enum class CheckStatus { Ok, Info, Warning, Error };

struct ToolchainCheck {
    std::string id;       // stable: "project", "gradlerio", "game_plugin", "gradle_wrapper", "jdk", "wpilib_install", "cpp_toolchain", "vendordeps", "team", "extension"
    std::string title;
    CheckStatus status = CheckStatus::Ok;
    std::string detail;
    std::string fix;      // what the user can do (empty when Ok)
};

struct ToolchainReport {
    std::vector<ToolchainCheck> checks;
    std::optional<WpilibInstall> wpilib;
    std::filesystem::path jdkHome;        // the JDK to launch with (WPILib's or JAVA_HOME's); empty if none was found
    std::string javaVersion;              // from the JDK's `release` file, "" if unknown

    bool ready() const;                   // no Error check
    const ToolchainCheck* find(std::string_view id) const;
};

struct ToolchainContext {
    config::Platform platform = config::currentPlatform();
    config::EnvLookup env = config::systemEnv();
    std::vector<std::filesystem::path> wpilibRoots;     // empty => wpilibRootCandidates(platform, env)
    std::function<bool(int)> hasGamePlugin;              // year -> a game plugin is registered; unset => check skipped
    std::filesystem::path extensionLibrary;              // halsim_wpiemu; empty => check skipped
    std::vector<std::filesystem::path> hostCompilerDirs; // extra places to look for a C++ compiler (tests / unusual installs)
};

ToolchainReport inspectToolchain(const ProjectReport& project, const ToolchainContext& context = {});

}  // namespace wpiemu::core::launcher
