#include <doctest/doctest.h>

#include "wpiemu/core/launcher/launch_diagnosis.h"

using namespace wpiemu::core::launcher;

TEST_CASE("launch diagnosis: WPILib's stale Visual C++ runtime message (verbatim from wpilib.org/jvmruntime)") {
    const char* output =
        "> Task :simulateJavaRelease FAILED\n"
        "If you receive errors loading the JNI dependencies, make sure you have the latest Visual Studio C++ Redstributable installed.\n"
        "Exception in thread \"main\" edu.wpi.first.util.MsvcRuntimeException: Invalid MSVC Runtime Detected.\n"
        "Expected at least 14.40, but found 14.29\n"
        "JVM Location: C:\\Program Files\\Amazon Corretto\\jdk21.0.5_11\\bin\\java.exe\n"
        "Runtime DLL Location: C:\\Program Files\\Amazon Corretto\\jdk21.0.5_11\\bin\\msvcp140.dll\n"
        "See https://wpilib.org/jvmruntime for more information\n";
    const LaunchProblem p = diagnoseLaunchOutput(output);
    CHECK(p.code == LaunchProblemCode::StaleVcRuntime);
    CHECK(p.expected == "14.40");
    CHECK(p.found == "14.29");
    CHECK(p.jvmPath == "C:\\Program Files\\Amazon Corretto\\jdk21.0.5_11\\bin\\java.exe");
    CHECK(p.runtimePath == "C:\\Program Files\\Amazon Corretto\\jdk21.0.5_11\\bin\\msvcp140.dll");
}

TEST_CASE("launch diagnosis: Java and Gradle problems") {
    CHECK(diagnoseLaunchOutput("FAILURE: Build failed\n> Could not determine java version from '25.0.1'.\n").code == LaunchProblemCode::JavaUnsupportedByGradle);
    CHECK(diagnoseLaunchOutput("java.lang.IllegalArgumentException: Unsupported class file major version 68\n").code ==
          LaunchProblemCode::JavaUnsupportedByGradle);
    CHECK(diagnoseLaunchOutput("ERROR: JAVA_HOME is set to an invalid directory: C:\\gone\n").code == LaunchProblemCode::JavaHomeInvalid);
    CHECK(diagnoseLaunchOutput("Value 'C:\\Users\\Public\\wpilib\\2026\\jdk' given for org.gradle.java.home Gradle property is invalid (Java home supplied is invalid)\n").code ==
          LaunchProblemCode::JavaHomeInvalid);
    CHECK(diagnoseLaunchOutput("Error: Could not find or load main class org.gradle.wrapper.GradleWrapperMain\n").code == LaunchProblemCode::GradleWrapperBroken);
    CHECK(diagnoseLaunchOutput("> No tool chain is available to build C++ for platform 'windows_x86-64'\n").code == LaunchProblemCode::NoNativeToolchain);
}

TEST_CASE("launch diagnosis: network failures and unknown output") {
    CHECK(diagnoseLaunchOutput("Exception in thread \"main\" java.net.UnknownHostException: services.gradle.org\n").code == LaunchProblemCode::DownloadFailed);
    CHECK(diagnoseLaunchOutput("> Could not GET 'https://frcmaven.wpi.edu/artifactory/release/x.pom'. Read timed out\n").code == LaunchProblemCode::DownloadFailed);
    CHECK(!diagnoseLaunchOutput("BUILD FAILED\nrobot code threw NullPointerException\n").any());
    CHECK(!diagnoseLaunchOutput("").any());
}

TEST_CASE("launch diagnosis: emulator extension native loader failures, not other extensions or retry noise") {
    const std::string attempt = "HAL Extensions: Attempting to load: halsim_wpiemu\n";
    for (const char* error : {"error #127", "libhalsim_wpiemu.so: undefined symbol: HAL_GetPort",
                              "dlopen(libhalsim_wpiemu.dylib): Symbol not found: _HAL_GetPort"}) {
        const auto problem = diagnoseLaunchOutput(attempt + "HAL Extensions: Failed to load library: " + error + "\n");
        CHECK(problem.code == LaunchProblemCode::ExtensionLoadFailed);
        CHECK(problem.detail.find(error) != std::string::npos);
    }
    CHECK(diagnoseLaunchOutput(attempt + "HAL Extensions: Failed to load extension\n").code == LaunchProblemCode::ExtensionLoadFailed);
    CHECK_FALSE(diagnoseLaunchOutput(attempt + "HAL Extensions: Load failed: not found\nTrying modified name: libhalsim_wpiemu.so\n"
                                              "HAL Extensions: Successfully loaded extension\n").any());
    CHECK_FALSE(diagnoseLaunchOutput(attempt + "HAL Extensions: Successfully loaded extension\n"
                                              "HAL Extensions: Attempting to load: other_extension\n"
                                              "HAL Extensions: Failed to load library: error #127\n").any());
}

TEST_CASE("launch diagnosis: the stale runtime wins over generic noise and ids round-trip") {
    const LaunchProblem p = diagnoseLaunchOutput("Could not GET 'x'\nInvalid MSVC Runtime Detected.\nExpected at least 14.40, but found 14.36\n");
    CHECK(p.code == LaunchProblemCode::StaleVcRuntime);
    for (const LaunchProblemCode c : {LaunchProblemCode::StaleVcRuntime, LaunchProblemCode::JavaUnsupportedByGradle, LaunchProblemCode::JavaHomeInvalid,
                                      LaunchProblemCode::DownloadFailed, LaunchProblemCode::GradleWrapperBroken, LaunchProblemCode::NoNativeToolchain,
                                      LaunchProblemCode::ExtensionLoadFailed})
        CHECK(launchProblemFromId(launchProblemId(c)) == c);
    CHECK(launchProblemFromId("nonsense") == LaunchProblemCode::None);
}
