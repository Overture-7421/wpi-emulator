#include <doctest/doctest.h>

#include "common/toolchain_checker.h"
#include "test_support.h"
#include "wpiemu/sdk/game/builtin_plugins.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;

namespace {

struct Fixture {
    wpiemu::sdk::GamePluginRegistry plugins;
    FakeProbe probe;
    Fixture() { (void)wpiemu::sdk::registerBuiltinGamePlugins(plugins); }
    ToolchainChecker checker() { return ToolchainChecker(probe, plugins); }
};

ToolchainInput inputFor(const QString& language, int year, const QString& project = QStringLiteral("/proj")) {
    ToolchainInput in;
    in.projectPath = project;
    in.language = language;
    in.wpilibYear = year;
    return in;
}

}  // namespace

TEST_CASE("java version parsing") {
    CHECK(ToolchainChecker::parseJavaMajor(QStringLiteral("openjdk version \"17.0.9\" 2023-10-17")) == 17);
    CHECK(ToolchainChecker::parseJavaMajor(QStringLiteral("java version \"1.8.0_352\"")) == 8);
    CHECK(ToolchainChecker::parseJavaMajor(QStringLiteral("openjdk version \"21\" 2023-09-19")) == 21);
    CHECK(!ToolchainChecker::parseJavaMajor(QStringLiteral("command not found")).has_value());
}

TEST_CASE("jdk check finds JAVA_HOME, rejects old versions and explains the fix") {
    Fixture f;
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
    f.probe.addJava(QStringLiteral("/jdk/bin/java"), QStringLiteral("openjdk version \"17.0.2\""));
    const ToolchainCheck ok = f.checker().checkJdk(inputFor(QStringLiteral("java"), 2026));
    CHECK(ok.status == CheckStatus::Ok);
    CHECK(ok.detail.contains(QStringLiteral("17")));
    CHECK(ok.requiredToRun);

    f.probe.addJava(QStringLiteral("/jdk/bin/java"), QStringLiteral("java version \"1.8.0_352\""));
    const ToolchainCheck old = f.checker().checkJdk(inputFor(QStringLiteral("java"), 2026));
    CHECK(old.status == CheckStatus::Missing);
    CHECK(old.detail.contains(QStringLiteral("Java 8")));
    CHECK(!old.fixHint.isEmpty());

    Fixture none;
    const ToolchainCheck missing = none.checker().checkJdk(inputFor(QStringLiteral("java"), 2026));
    CHECK(missing.status == CheckStatus::Missing);
    CHECK(!missing.fixHint.isEmpty());
}

TEST_CASE("jdk search order: settings override, JAVA_HOME, WPILib's JDK, PATH") {
    Fixture f;
    f.probe.dirs.insert(QStringLiteral("/home/user/wpilib/2026"));
    f.probe.addJava(QStringLiteral("/home/user/wpilib/2026/jdk/bin/java"), QStringLiteral("openjdk version \"17.0.8\""));
    CHECK(f.checker().checkJdk(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Ok);

    f.probe.addJava(QStringLiteral("/custom/bin/java"), QStringLiteral("openjdk version \"21.0.1\""));
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.jdkHomeOverride = QStringLiteral("/custom");
    const ToolchainCheck c = f.checker().checkJdk(in);
    CHECK(c.status == CheckStatus::Ok);
    CHECK(c.detail.contains(QStringLiteral("21")));
}

TEST_CASE("gradle wrapper and wpilib install checks") {
    Fixture f;
    CHECK(f.checker().checkGradleWrapper(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Missing);
    f.probe.files.insert(QStringLiteral("/proj/gradlew"));
    CHECK(f.checker().checkGradleWrapper(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Ok);

    f.probe.os = HostOs::Windows;
    CHECK(f.checker().checkGradleWrapper(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Missing);   // wants gradlew.bat
    f.probe.files.insert(QStringLiteral("/proj/gradlew.bat"));
    CHECK(f.checker().checkGradleWrapper(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Ok);

    const ToolchainChecker c = f.checker();
    CHECK(c.defaultWpilibHome(2026) == QStringLiteral("C:/Users/Public/wpilib/2026"));
    CHECK(c.checkWpilibInstall(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Warning);
    f.probe.dirs.insert(QStringLiteral("C:/Users/Public/wpilib/2026"));
    CHECK(c.checkWpilibInstall(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Ok);
}

TEST_CASE("c++ toolchain is only required for c++ projects") {
    Fixture f;
    const ToolchainCheck java = f.checker().checkCppToolchain(inputFor(QStringLiteral("java"), 2026));
    CHECK(java.status == CheckStatus::NotApplicable);
    CHECK(!java.requiredToRun);

    const ToolchainCheck missing = f.checker().checkCppToolchain(inputFor(QStringLiteral("cpp"), 2026));
    CHECK(missing.status == CheckStatus::Missing);
    CHECK(missing.requiredToRun);
    f.probe.onPath[QStringLiteral("g++")] = QStringLiteral("/usr/bin/g++");
    CHECK(f.checker().checkCppToolchain(inputFor(QStringLiteral("cpp"), 2026)).status == CheckStatus::Ok);

    f.probe.os = HostOs::Windows;
    CHECK(f.checker().checkCppToolchain(inputFor(QStringLiteral("cpp"), 2026)).status == CheckStatus::Missing);
    f.probe.environment[QStringLiteral("ProgramFiles(x86)")] = QStringLiteral("C:/Program Files (x86)");
    const QString vswhere = QStringLiteral("C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe");
    f.probe.files.insert(vswhere);
    ProcessResult r;
    r.started = true;
    r.output = QStringLiteral("C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\n");
    f.probe.processes[vswhere] = r;
    const ToolchainCheck vs = f.checker().checkCppToolchain(inputFor(QStringLiteral("cpp"), 2026));
    CHECK(vs.status == CheckStatus::Ok);
    CHECK(vs.detail.contains(QStringLiteral("Visual Studio")));
}

TEST_CASE("game module: supported season selects the plugin, other seasons run core-only") {
    Fixture f;
    ToolchainReport supported = f.checker().run(inputFor(QStringLiteral("java"), 2026));
    CHECK(!supported.coreOnly());
    CHECK(supported.gamePluginId == QStringLiteral("game_2026"));
    CHECK(supported.find(QStringLiteral("game"))->status == CheckStatus::Ok);

    ToolchainReport unsupported = f.checker().run(inputFor(QStringLiteral("java"), 2031));
    CHECK(unsupported.coreOnly());
    const ToolchainCheck* game = unsupported.find(QStringLiteral("game"));
    REQUIRE(game != nullptr);
    CHECK(game->status == CheckStatus::Warning);
    CHECK(game->detail.contains(QStringLiteral("2031")));
    CHECK(game->detail.contains(QStringLiteral("core-only")));
    CHECK(!game->requiredToRun);   // an unsupported season never blocks running

    CHECK(f.checker().run(inputFor(QStringLiteral("java"), 0)).coreOnly());
}

TEST_CASE("report says whether the robot can be launched") {
    Fixture f;
    f.probe.files.insert(QStringLiteral("/proj/gradlew"));
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
    CHECK(!f.checker().run(inputFor(QStringLiteral("java"), 2026)).canRunRobot());   // no JDK yet
    f.probe.addJava(QStringLiteral("/jdk/bin/java"), QStringLiteral("openjdk version \"17.0.9\""));
    const ToolchainReport ready = f.checker().run(inputFor(QStringLiteral("java"), 2026));
    CHECK(ready.canRunRobot());
    CHECK(ready.checks.size() == 5);
}
