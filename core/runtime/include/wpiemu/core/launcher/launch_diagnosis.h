#pragma once

#include <string>
#include <string_view>

namespace wpiemu::core::launcher {

// A failure of the robot launch the user can act on. The ids are stable keys: the app maps them to words and actions.
enum class LaunchProblemCode {
    None,
    StaleVcRuntime,           // WPILib's MsvcRuntimeException: the JDK's msvcp140.dll is older than WPILib needs (Windows)
    JavaUnsupportedByGradle,  // Gradle cannot run on (or does not understand) the selected Java
    JavaHomeInvalid,          // JAVA_HOME / org.gradle.java.home names a folder without a usable Java
    DownloadFailed,           // Gradle could not fetch the wrapper, WPILib or a vendor library (offline, proxy, TLS)
    GradleWrapperBroken,      // gradle-wrapper.jar missing or damaged
    NoNativeToolchain,        // C++ build without a usable compiler
    ExtensionLoadFailed,     // HAL could not load or initialize this emulator's extension
};

struct LaunchProblem {
    LaunchProblemCode code = LaunchProblemCode::None;
    std::string found;         // StaleVcRuntime: runtime version the JVM loaded, e.g. "14.36"
    std::string expected;      // StaleVcRuntime: version WPILib wants, e.g. "14.40"
    std::string jvmPath;       // StaleVcRuntime: the java.exe that was running
    std::string runtimePath;   // StaleVcRuntime: the msvcp140.dll it loaded
    std::string detail;        // the line that matched, trimmed

    bool any() const { return code != LaunchProblemCode::None; }
};

// Looks for a known signature in the tail of the robot/Gradle output (most specific first). Unknown output gives None, so
// callers keep showing the raw tail.
LaunchProblem diagnoseLaunchOutput(std::string_view output);

// "stale_vc_runtime", "java_unsupported_by_gradle", ...; "" for None.
const char* launchProblemId(LaunchProblemCode code);
LaunchProblemCode launchProblemFromId(std::string_view id);

}  // namespace wpiemu::core::launcher
