#include <doctest/doctest.h>

#include "launcher_test_support.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/launcher/jdk_policy.h"
#include "wpiemu/core/launcher/project_inspector.h"
#include "wpiemu/sdk/testing/environment.h"

using namespace wpiemu::core::launcher;
using wpiemu::sdk::testing::envOf;
using wpiemu::test_runtime::ScratchDir;
namespace fs = std::filesystem;

TEST_CASE("jdk policy: version comparison and java major parsing") {
    CHECK(compareVersions("14.36.32532.0", "14.40") < 0);
    CHECK(compareVersions("14.40", "14.40.0") == 0);
    CHECK(compareVersions("14.44.35211.0", "14.40") > 0);
    CHECK(compareVersions("8.11.1", "8.11") > 0);
    CHECK(parseJavaMajor("openjdk version \"17.0.9\" 2023-10-17") == 17);
    CHECK(parseJavaMajor("java version \"1.8.0_352\"") == 8);
    CHECK(parseJavaMajor("openjdk version \"21\" 2023-09-19") == 21);
    CHECK(parseJavaMajor("java version \"25.0.1\" 2025-10-21 LTS") == 25);
    CHECK(!parseJavaMajor("command not found").has_value());
}

TEST_CASE("jdk policy: the newest Java a Gradle version runs on") {
    CHECK(maxJavaForGradle("8.11") == 23);
    CHECK(maxJavaForGradle("8.11.1") == 23);
    CHECK(maxJavaForGradle("7.3") == 17);
    CHECK(maxJavaForGradle("9.1") == 25);
    CHECK(maxJavaForGradle("9.2.0") == 25);
    CHECK(maxJavaForGradle("12.0") == 25);   // newer than the table: its last row
    CHECK(maxJavaForGradle("") == 23);       // unknown: the WPILib 2026 template's Gradle
    CHECK(maxJavaForGradle("", 25) == 25);   // unknown: the WPILib 2027 template's Gradle
}

TEST_CASE("jdk policy: verdicts name the first problem") {
    const auto verdict = [](const JdkFacts& facts, int maxMajor, bool windows) { return evaluateJdk(facts, 17, maxMajor, windows); };
    JdkFacts f;
    f.hasJava = true;
    f.hasCompiler = true;
    f.major = 17;
    f.vcRuntime = "14.44.35211.0";
    CHECK(verdict(f, 23, true).ok());

    f.vcRuntime = "14.36.32532.0";
    CHECK(verdict(f, 23, true).problem == JdkProblem::StaleVcRuntime);
    CHECK(verdict(f, 23, false).ok());   // the runtime only matters on Windows
    f.vcRuntime.reset();
    CHECK(verdict(f, 23, true).ok());    // unknown is not a failure

    f.major = 25;
    CHECK(verdict(f, 23, true).problem == JdkProblem::TooNewForGradle);
    CHECK(evaluateJdk(f, 25, 25, true).ok());   // a season that builds with Java 25 accepts it
    f.major = 11;
    CHECK(verdict(f, 23, true).problem == JdkProblem::TooOld);
    f.major = 17;
    CHECK(evaluateJdk(f, 25, 25, true).problem == JdkProblem::TooOld);
    f.hasCompiler = false;
    CHECK(verdict(f, 23, true).problem == JdkProblem::NoCompiler);
    f.major.reset();
    CHECK(verdict(f, 23, true).problem == JdkProblem::NotRunnable);
}

TEST_CASE("wpilib installs: every season, newest first, first root wins") {
    ScratchDir dir;
    dir.write("a/2025/jdk/bin/java", "x");
    dir.write("a/2026/maven/x", "x");
    dir.write("a/notes/readme", "x");
    dir.write("b/2026/jdk/bin/java", "x");
    dir.write("a/2027_alpha7/jdk/bin/java", "x");
    dir.write("b/2024/jdk/bin/java", "x");
    const auto installs = listWpilibInstalls({dir.path() / "a", dir.path() / "b", dir.path() / "missing"});
    REQUIRE(installs.size() == 4);
    CHECK(installs[0].year == 2027);
    CHECK(installs[0].home == dir.path() / "a" / "2027_alpha7");
    CHECK(installs[1].year == 2026);
    CHECK(installs[1].home == dir.path() / "a" / "2026");   // the first root
    CHECK(installs[2].year == 2025);
    CHECK(!installs[2].jdkHome.empty());
    CHECK(installs[3].year == 2024);
    CHECK(wpilibInstallFolderName(2027) == "2027_alpha7");
    const auto alphaInstall = findWpilibInstall(2027, {dir.path() / "a"});
    REQUIRE(alphaInstall.has_value());
    CHECK(alphaInstall->home == dir.path() / "a" / "2027_alpha7");
}

TEST_CASE("managed jdk: only finished installs count, newest wins") {
    ScratchDir dir;
    const fs::path root = managedToolchainsDir(dir.path());
    CHECK(!findManagedJdk(root, false).has_value());
    dir.write("toolchains/temurin-17.0.9/bin/java", "x");                       // no marker: unfinished
    CHECK(!findManagedJdk(root, false).has_value());
    dir.write("toolchains/temurin-17.0.9/.wpiemu-ready", "17.0.9");
    dir.write("toolchains/temurin-17.0.20.1/bin/java", "x");
    dir.write("toolchains/temurin-17.0.20.1/.wpiemu-ready", "17.0.20.1");
    dir.write("toolchains/temurin-17.0.100/bin/java", "x");                     // newer but unfinished
    REQUIRE(findManagedJdk(root, false).has_value());
    CHECK(*findManagedJdk(root, false) == root / "temurin-17.0.20.1");
    CHECK(!findManagedJdk(root, true).has_value());                             // java.exe expected on Windows

    dir.write("toolchains/temurin-25.0.1/bin/java", "x");
    dir.write("toolchains/temurin-25.0.1/.wpiemu-ready", "25.0.1");
    CHECK(*findManagedJdk(root, false, 17) == root / "temurin-17.0.20.1");      // per Java major
    CHECK(*findManagedJdk(root, false, 25) == root / "temurin-25.0.1");
    CHECK(!findManagedJdk(root, false, 21).has_value());
}

TEST_CASE("local data root is the local app-data folder on Windows") {
    using wpiemu::core::config::Platform;
    using wpiemu::core::config::localDataRoot;
    CHECK(localDataRoot(Platform::Windows, envOf({{"LOCALAPPDATA", "C:/L"}, {"APPDATA", "C:/R"}})).value() == fs::path("C:/L") / "WpiEmulator");
    CHECK(localDataRoot(Platform::Windows, envOf({{"USERPROFILE", "C:/U"}})).value() == fs::path("C:/U") / "AppData" / "Local" / "WpiEmulator");
    CHECK(localDataRoot(Platform::Linux, envOf({{"XDG_DATA_HOME", "/x"}})).value() == fs::path("/x") / "wpi-emulator");
    CHECK(!localDataRoot(Platform::Windows, envOf({})).ok());
}
