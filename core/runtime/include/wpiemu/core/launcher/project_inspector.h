#pragma once

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
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

// ---- WPILib install and JDK discovery -------------------------------------------------------------------------------------
// Shared by the launcher (RobotSession) and the wizard's toolchain check, so the two cannot disagree about what a run uses.

struct WpilibInstall {
    std::filesystem::path home;        // <root>/<year>
    int year = 0;
    std::filesystem::path jdkHome;     // <home>/jdk when it contains bin/java; else empty
    bool hasMaven = false;             // offline maven repository (Gradle resolves GradleRIO from it)
    bool hasTools = false;
    bool hasRoboRioToolchain = false;
};

// Where the WPILib installer puts a season. Windows: <publicDir>\wpilib and C:\Users\Public\wpilib ("everyone"), then
// <homeDir>\wpilib ("this user only"); macOS and Linux: <homeDir>/wpilib. An empty folder is skipped.
std::vector<std::filesystem::path> wpilibRootCandidates(config::Platform platform, const std::filesystem::path& publicDir,
                                                        const std::filesystem::path& homeDir);
// The same with the folders taken from the environment: PUBLIC, and USERPROFILE on Windows or HOME elsewhere.
std::vector<std::filesystem::path> wpilibRootCandidates(config::Platform platform, const sdk::EnvLookup& env);
// The install at `home`, a season folder such as <root>/2026 or <root>/2027_alpha7; nullopt when it is not a directory.
std::optional<WpilibInstall> wpilibInstallAt(const std::filesystem::path& home, int year);
// WPILib installer folder of a season: `preferred` (the season pack's installFolder, for example one with a preview suffix)
// when given, else the year.
std::string wpilibInstallFolderName(int year, std::string_view preferred = {});
// First root containing the season's folder wins: `preferredFolder` (see above), then "<year>", then any "<year>_*" folder
// (newest name first), so a preview revision the manifest does not name yet is still found. `roots` empty => wpilibRootCandidates().
std::optional<WpilibInstall> findWpilibInstall(int year, const std::vector<std::filesystem::path>& roots,
                                               std::string_view preferredFolder = {});

// Where the JDK that runs Gradle comes from, in the order the launcher tries: the explicit setting, the season's WPILib JDK
// (trusted), another installed season's WPILib JDK, JAVA_HOME, `java` on PATH, then the emulator's own downloaded Java.
// Everything but the setting and the season's WPILib JDK must pass jdk_policy.h before it is used. The robot session and the
// wizard's toolchain check both walk this list.
enum class JdkSource { Setting, WpilibInstall, WpilibOtherSeason, JavaHome, Path, Managed };
inline constexpr std::array<JdkSource, 6> kJdkSources{JdkSource::Setting, JdkSource::WpilibInstall, JdkSource::WpilibOtherSeason,
                                                      JdkSource::JavaHome, JdkSource::Path,         JdkSource::Managed};

// Every season folder under `roots`, newest year first; the first root wins per year. Numeric and alpha-suffixed folders count.
// `roots` empty => wpilibRootCandidates().
std::vector<WpilibInstall> listWpilibInstalls(const std::vector<std::filesystem::path>& roots);

}  // namespace wpiemu::core::launcher
