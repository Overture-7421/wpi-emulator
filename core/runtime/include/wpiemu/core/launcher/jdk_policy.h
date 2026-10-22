#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace wpiemu::core::launcher {

// What a JDK must satisfy to run a WPILib robot project through Gradle. Pure functions over facts the caller gathered, so the
// wizard's check (through ISystemProbe) and the session launch judge a JDK the same way and tests need no machine.

inline constexpr int kMinJdkMajor = 17;                  // the Java of WPILib 2026 projects; a season may name another (see the app's pins)
inline constexpr std::string_view kMinVcRuntime = "14.40";   // WPIUtilJNI.checkMsvcRuntime: the runtime java.exe loads (Windows)

// "8.11.1" / "9.2.0" -> negative, 0, positive like strcmp. Missing components count as 0; non-numeric text compares as 0.
int compareVersions(std::string_view a, std::string_view b);

// The newest Java the given Gradle version can run on (https://docs.gradle.org/current/userguide/compatibility.html).
// Empty/unparseable -> the Gradle of the WPILib 2026 template (8.11); newer than the table -> its last row.
int maxJavaForGradle(std::string_view gradleVersion);

// `java -version` text -> major version (8, 11, 17, 21...); nullopt when unparseable.
std::optional<int> parseJavaMajor(std::string_view versionOutput);

struct JdkFacts {
    bool hasJava = false;
    bool hasCompiler = false;                   // bin/javac: a JRE cannot compile the robot
    std::optional<int> major;                   // from `java -version`
    std::optional<std::string> vcRuntime;       // Windows only: version of the msvcp140.dll the JVM will load; nullopt = unknown
};

enum class JdkProblem { None, NotRunnable, NoCompiler, TooOld, TooNewForGradle, StaleVcRuntime };

struct JdkVerdict {
    JdkProblem problem = JdkProblem::None;
    int major = 0;
    int maxMajor = 0;
    std::string vcRuntime;
    bool ok() const { return problem == JdkProblem::None; }
};

JdkVerdict evaluateJdk(const JdkFacts& facts, int minMajor, int maxMajor, bool windows);

// ---- the emulator's own Java ---------------------------------------------------------------------------------------------
// Downloaded on demand (never shipped) to <localData>/toolchains/<vendor>-<version>; <name>/.wpiemu-ready marks a finished install.
inline constexpr std::string_view kManagedJdkMarker = ".wpiemu-ready";

std::filesystem::path managedToolchainsDir(const std::filesystem::path& localDataRoot);
// The newest finished install under `toolchainsDir` that holds bin/java(.exe) and is Java `major` (0 = any); nullopt when none.
std::optional<std::filesystem::path> findManagedJdk(const std::filesystem::path& toolchainsDir, bool windows, int major = 0);

}  // namespace wpiemu::core::launcher
