#include <doctest/doctest.h>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <array>
#include "common/toolchain_checker.h"
#include "test_support.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/season/builtin_seasons.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;

namespace {

struct Fixture {
    wpiemu::sdk::GamePluginRegistry plugins;
    wpiemu::sdk::SeasonRegistry seasons;
    FakeProbe probe;
    Fixture() {
        (void)wpiemu::sdk::registerBuiltinGamePlugins(plugins);
        (void)wpiemu::sdk::registerBuiltinSeasons(seasons);
    }
    ToolchainChecker checker() { return ToolchainChecker(probe, plugins, seasons); }
};

ToolchainInput inputFor(const QString& language, int year, const QString& project = QStringLiteral("/proj")) {
    ToolchainInput in;
    in.projectPath = project;
    in.language = language;
    in.wpilibYear = year;
    return in;
}

// What `vswhere -format json` prints for the given {display name, install path, version} installs.
QString vswhereJson(std::initializer_list<std::array<const char*, 3>> installs) {
    QJsonArray all;
    for (const auto& i : installs)
        all.append(QJsonObject{{"displayName", i[0]}, {"installationPath", i[1]}, {"installationVersion", i[2]}});
    return QString::fromUtf8(QJsonDocument(all).toJson());
}

}  // namespace

TEST_CASE("java version parsing") {
    CHECK(ToolchainChecker::parseJavaMajor(QStringLiteral("openjdk version \"17.0.9\" 2023-10-17")) == 17);
    CHECK(ToolchainChecker::parseJavaMajor(QStringLiteral("java version \"1.8.0_352\"")) == 8);
    CHECK(ToolchainChecker::parseJavaMajor(QStringLiteral("openjdk version \"21\" 2023-09-19")) == 21);
    CHECK(!ToolchainChecker::parseJavaMajor(QStringLiteral("command not found")).has_value());
}

TEST_CASE("toolchain report diagnoses a missing season extension before launch") {
    Fixture f;
    auto in = inputFor(QStringLiteral("java"), 2027);
    in.extensionLibrary = QString();
    in.installedExtensionSeasons = {QStringLiteral("2026")};
    const auto missing = f.checker().run(in);
    const auto* row = missing.find(QString::fromLatin1(check_id::kHalExtension));
    REQUIRE(row);
    CHECK(row->status == CheckStatus::Missing);
    CHECK(row->requiredToRun);
    CHECK(row->detail.contains(QStringLiteral("2026")));
    CHECK_FALSE(row->fixHint.isEmpty());
    in.extensionLibrary = QStringLiteral("/extensions/2027/halsim_wpiemu.dll");
    const auto installed = f.checker().run(in);
    CHECK(installed.find(QString::fromLatin1(check_id::kHalExtension))->status == CheckStatus::Ok);
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

TEST_CASE("jdk search order: settings override, WPILib's JDK, JAVA_HOME, PATH (the launcher's order)") {
    Fixture f;
    f.probe.onPath[QStringLiteral("java")] = QStringLiteral("/usr/bin/java");
    f.probe.addJava(QStringLiteral("/usr/bin/java"), QStringLiteral("openjdk version \"17.0.1\""));
    const ToolchainCheck onPath = f.checker().checkJdk(inputFor(QStringLiteral("java"), 2026));
    CHECK(onPath.status == CheckStatus::Ok);
    CHECK(onPath.detail.contains(QDir::toNativeSeparators(QStringLiteral("/usr/bin/java"))));

    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
    f.probe.addJava(QStringLiteral("/jdk/bin/java"), QStringLiteral("openjdk version \"21.0.3\""));
    CHECK(f.checker().checkJdk(inputFor(QStringLiteral("java"), 2026)).detail.contains(QStringLiteral("Java 21")));

    f.probe.dirs.insert(QStringLiteral("/home/user/wpilib/2026"));
    f.probe.addJava(QStringLiteral("/home/user/wpilib/2026/jdk/bin/java"), QStringLiteral("openjdk version \"17.0.8\""));
    const ToolchainCheck wpilib = f.checker().checkJdk(inputFor(QStringLiteral("java"), 2026));
    CHECK(wpilib.status == CheckStatus::Ok);
    CHECK(wpilib.detail.contains(QStringLiteral("Java 17")));   // beats JAVA_HOME, as the launcher does

    f.probe.addJava(QStringLiteral("/custom/bin/java"), QStringLiteral("openjdk version \"21.0.1\""));
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.jdkHomeOverride = QStringLiteral("/custom");
    const ToolchainCheck c = f.checker().checkJdk(in);
    CHECK(c.status == CheckStatus::Ok);
    CHECK(c.detail.contains(QStringLiteral("21")));

    // The first present source is the one that counts: a broken setting is not silently replaced by a working JDK.
    in.jdkHomeOverride = QStringLiteral("/nowhere");
    const ToolchainCheck broken = f.checker().checkJdk(in);
    CHECK(broken.status == CheckStatus::Missing);
    CHECK(broken.detail.contains(QStringLiteral("does not exist")));
}

TEST_CASE("jdk: a WPILib season without a bundled JDK falls through to JAVA_HOME") {
    Fixture f;
    f.probe.dirs.insert(QStringLiteral("/home/user/wpilib/2026"));   // installed, but no jdk/bin/java
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
    f.probe.addJava(QStringLiteral("/jdk/bin/java"), QStringLiteral("openjdk version \"17.0.2\""));
    CHECK(f.checker().checkJdk(inputFor(QStringLiteral("java"), 2026)).detail.contains(QDir::toNativeSeparators(QStringLiteral("/jdk/bin/java"))));
}

TEST_CASE("wpilib install is found in the all-users and the this-user-only locations on Windows") {
    Fixture f;
    f.probe.os = HostOs::Windows;
    f.probe.home = QStringLiteral("C:/Users/me");
    CHECK(f.checker().findWpilibHome(inputFor(QStringLiteral("java"), 2026)).isEmpty());
    f.probe.dirs.insert(QStringLiteral("C:/Users/me/wpilib/2026"));
    CHECK(f.checker().findWpilibHome(inputFor(QStringLiteral("java"), 2026)) == QStringLiteral("C:/Users/me/wpilib/2026"));
    f.probe.dirs.insert(QStringLiteral("C:/Users/Public/wpilib/2026"));
    CHECK(f.checker().findWpilibHome(inputFor(QStringLiteral("java"), 2026)) == QStringLiteral("C:/Users/Public/wpilib/2026"));   // the shared one wins
    f.probe.environment[QStringLiteral("PUBLIC")] = QStringLiteral("D:/Shared");
    f.probe.dirs.insert(QStringLiteral("D:/Shared/wpilib/2026"));
    CHECK(f.checker().findWpilibHome(inputFor(QStringLiteral("java"), 2026)) == QStringLiteral("D:/Shared/wpilib/2026"));
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
    CHECK(c.defaultWpilibHome(2027) == QStringLiteral("C:/Users/Public/wpilib/2027_alpha7"));
    CHECK(c.checkWpilibInstall(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Warning);
    f.probe.dirs.insert(QStringLiteral("C:/Users/Public/wpilib/2026"));
    CHECK(c.checkWpilibInstall(inputFor(QStringLiteral("java"), 2026)).status == CheckStatus::Ok);
    f.probe.dirs.insert(QStringLiteral("C:/Users/Public/wpilib/2027_alpha7"));
    CHECK(c.findWpilibHome(inputFor(QStringLiteral("java"), 2027)) == QStringLiteral("C:/Users/Public/wpilib/2027_alpha7"));
    CHECK(c.checkWpilibInstall(inputFor(QStringLiteral("java"), 2027)).status == CheckStatus::Ok);
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
    r.output = vswhereJson({{"Visual Studio Community 2022", "C:\\Program Files\\Microsoft Visual Studio\\2022\\Community", "17.14.37111.16"}});
    f.probe.processes[vswhere] = r;
    const ToolchainCheck vs = f.checker().checkCppToolchain(inputFor(QStringLiteral("cpp"), 2026));
    CHECK(vs.status == CheckStatus::Ok);
    CHECK(vs.detail.contains(QStringLiteral("Visual Studio")));
}

TEST_CASE("c++ toolchain (Windows): a working setup is never flagged, an unusable one explains why and offers the install") {
    Fixture f;
    f.probe.os = HostOs::Windows;
    f.probe.environment[QStringLiteral("ProgramFiles(x86)")] = QStringLiteral("C:/Program Files (x86)");
    const QString vswhere = QStringLiteral("C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe");
    f.probe.files.insert(vswhere);
    const auto machine = [&](std::initializer_list<std::array<const char*, 3>> installs) {
        ProcessResult r;
        r.started = true;
        r.output = vswhereJson(installs);
        f.probe.processes[vswhere] = r;
    };
    ToolchainInput in = inputFor(QStringLiteral("cpp"), 2026);
    in.buildToolsInstallAvailable = true;

    machine({{"Visual Studio Professional 2022", "C:\\vs22", "17.14.1"}, {"Visual Studio Community 2026", "C:\\vs26", "18.11.1"}});
    ToolchainCheck check = f.checker().checkCppToolchain(in);
    CHECK(check.status == CheckStatus::Ok);   // 2022 is there next to 2026
    CHECK(check.fixAction == FixAction::None);
    CHECK(check.detail.contains(QStringLiteral("2022")));

    machine({{"Visual Studio Community 2026", "C:\\vs26", "18.11.1"}});
    check = f.checker().checkCppToolchain(in);
    CHECK(check.status == CheckStatus::Missing);
    CHECK(check.detail.contains(QStringLiteral("newer than 2022")));
    CHECK(check.fixAction == FixAction::InstallBuildTools);

    machine({{"Visual Studio Community 2019", "C:\\vs19", "16.11.3"}});
    check = f.checker().checkCppToolchain(in);
    CHECK(check.status == CheckStatus::Missing);
    CHECK(check.detail.contains(QStringLiteral("17.9")));

    machine({});
    in.buildToolsInstallAvailable = false;
    check = f.checker().checkCppToolchain(in);
    CHECK(check.status == CheckStatus::Missing);
    CHECK(check.fixAction == FixAction::None);

    in.language = QStringLiteral("java");   // a Java project never sees any of this
    CHECK(f.checker().checkCppToolchain(in).status == CheckStatus::NotApplicable);
    CHECK(f.checker().checkCppToolchain(in).fixAction == FixAction::None);
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

// ---- which Java is used, and when the emulator's own Java is offered ----------------------------------------------------

namespace {
const QString kJava17 = QStringLiteral("openjdk version \"17.0.9\" 2023-10-17");
const QString kJava25 = QStringLiteral("java version \"25.0.1\" 2025-10-21 LTS");
}  // namespace

TEST_CASE("jdk gating: the project's WPILib JDK is used as is, with no download offered and nothing else examined") {
    Fixture f;
    f.probe.dirs.insert(QStringLiteral("/home/user/wpilib/2026"));
    f.probe.addJava(QStringLiteral("/home/user/wpilib/2026/jdk/bin/java"), kJava17);
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk25");
    f.probe.addJava(QStringLiteral("/jdk25/bin/java"), kJava25);
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.managedJavaAvailable = true;
    in.managedJdkHome = QStringLiteral("/managed");
    f.probe.addJava(QStringLiteral("/managed/bin/java"), kJava17);

    const ToolchainCheck check = f.checker().checkJdk(in);
    CHECK(check.status == CheckStatus::Ok);
    CHECK(check.fixAction == FixAction::None);
    CHECK(check.detail.contains(QDir::toNativeSeparators(QStringLiteral("/home/user/wpilib/2026/jdk/bin/java"))));
    CHECK(!check.detail.contains(QStringLiteral("Passed over")));
    CHECK(f.probe.runCount == 1);   // only the WPILib JDK was run
}

TEST_CASE("jdk gating: a working JDK from JAVA_HOME or PATH means no download is offered") {
    Fixture f;
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
    f.probe.addJava(QStringLiteral("/jdk/bin/java"), kJava17);
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.managedJavaAvailable = true;
    const ToolchainCheck check = f.checker().checkJdk(in);
    CHECK(check.status == CheckStatus::Ok);
    CHECK(check.fixAction == FixAction::None);
}

TEST_CASE("jdk validation: Java too new for the project's Gradle is passed over and the download is offered") {
    Fixture f;
    f.probe.onPath[QStringLiteral("java")] = QStringLiteral("/usr/bin/java");
    f.probe.addJava(QStringLiteral("/usr/bin/java"), kJava25);
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.gradleVersion = QStringLiteral("8.11");
    CHECK(f.checker().checkJdk(in).status == CheckStatus::Missing);
    in.managedJavaAvailable = true;
    const ToolchainCheck check = f.checker().checkJdk(in);
    CHECK(check.status == CheckStatus::Missing);
    CHECK(check.fixAction == FixAction::GetJava);
    CHECK(check.detail.contains(QStringLiteral("Java 25")));
    CHECK(check.detail.contains(QStringLiteral("Gradle 8.11")));

    in.gradleVersion = QStringLiteral("9.2.0");   // a newer Gradle runs on it
    CHECK(f.checker().checkJdk(in).status == CheckStatus::Ok);
}

TEST_CASE("jdk validation: a JRE without javac is not a JDK") {
    Fixture f;
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jre");
    f.probe.addJava(QStringLiteral("/jre/bin/java"), kJava17, false);
    const ToolchainCheck check = f.checker().checkJdk(inputFor(QStringLiteral("java"), 2026));
    CHECK(check.status == CheckStatus::Missing);
    CHECK(check.detail.contains(QStringLiteral("javac")));
}

TEST_CASE("jdk validation (Windows): a JDK that bundles an old Visual C++ runtime is skipped, whatever Visual Studio is installed") {
    Fixture f;
    f.probe.os = HostOs::Windows;
    f.probe.home = QStringLiteral("C:/Users/me");
    f.probe.environment[QStringLiteral("SystemRoot")] = QStringLiteral("C:/Windows");
    f.probe.fileVersions[QStringLiteral("C:/Windows/System32/msvcp140.dll")] = QStringLiteral("14.51.36247.0");   // the latest system runtime
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("C:/jdk17");
    f.probe.addJava(QStringLiteral("C:/jdk17/bin/java.exe"), kJava17);
    f.probe.fileVersions[QStringLiteral("C:/jdk17/bin/msvcp140.dll")] = QStringLiteral("14.36.32532.0");   // java.exe loads this one first
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.managedJavaAvailable = true;

    ToolchainCheck check = f.checker().checkJdk(in);
    CHECK(check.status == CheckStatus::Missing);
    CHECK(check.fixAction == FixAction::GetJava);
    CHECK(check.detail.contains(QStringLiteral("14.36")));
    CHECK(check.detail.contains(QStringLiteral("14.40")));

    // the same JDK without its own copy loads the (current) system runtime and is fine
    f.probe.fileVersions.erase(QStringLiteral("C:/jdk17/bin/msvcp140.dll"));
    CHECK(f.checker().checkJdk(in).status == CheckStatus::Ok);

    // the WPILib JDK of the project's season is trusted even with an older runtime
    f.probe.fileVersions[QStringLiteral("C:/Users/me/wpilib/2026/jdk/bin/msvcp140.dll")] = QStringLiteral("14.29.0.0");
    f.probe.dirs.insert(QStringLiteral("C:/Users/me/wpilib/2026"));
    f.probe.addJava(QStringLiteral("C:/Users/me/wpilib/2026/jdk/bin/java.exe"), kJava17);
    check = f.checker().checkJdk(in);
    CHECK(check.status == CheckStatus::Ok);
    CHECK(check.detail.contains(QStringLiteral("wpilib")));
}

TEST_CASE("jdk order: another installed season's JDK is a validated fallback; the emulator's Java comes last") {
    Fixture f;
    f.probe.dirs.insert(QStringLiteral("/home/user/wpilib/2025"));
    f.probe.addJava(QStringLiteral("/home/user/wpilib/2025/jdk/bin/java"), kJava17);
    f.probe.dirs.insert(QStringLiteral("/home/user/wpilib/2027_alpha7"));
    f.probe.addJava(QStringLiteral("/home/user/wpilib/2027_alpha7/jdk/bin/java"), kJava25);   // too new: passed over
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.managedJdkHome = QStringLiteral("/managed");
    f.probe.addJava(QStringLiteral("/managed/bin/java"), kJava17);

    JdkResolution jdk = f.checker().resolveJdk(in);
    REQUIRE(jdk.found);
    CHECK(jdk.source == wpiemu::core::launcher::JdkSource::WpilibOtherSeason);
    CHECK(jdk.home == QStringLiteral("/home/user/wpilib/2025/jdk"));
    REQUIRE(jdk.skipped.size() == 1);
    CHECK(jdk.skipped[0].reason.contains(QStringLiteral("Java 25")));

    f.probe.files.remove(QStringLiteral("/home/user/wpilib/2025/jdk/bin/java"));
    jdk = f.checker().resolveJdk(in);
    REQUIRE(jdk.found);
    CHECK(jdk.source == wpiemu::core::launcher::JdkSource::Managed);
}

TEST_CASE("jdk validation: the minimum Java comes from the project's season") {
    Fixture f;
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
    f.probe.addJava(QStringLiteral("/jdk/bin/java"), kJava17);
    ToolchainInput in = inputFor(QStringLiteral("java"), 2027);
    in.minJavaMajor = 25;
    ToolchainCheck check = f.checker().checkJdk(in);
    CHECK(check.status == CheckStatus::Missing);
    f.probe.addJava(QStringLiteral("/jdk/bin/java"), kJava25);
    CHECK(f.checker().checkJdk(in).status == CheckStatus::Ok);   // the 2027 template default supports Java 25

    f.probe.addJava(QStringLiteral("/jdk/bin/java"), kJava17);
    in.gradleVersion = QStringLiteral("9.2.0");
    check = f.checker().checkJdk(in);
    CHECK(check.status == CheckStatus::Missing);
    CHECK(check.detail.contains(QStringLiteral("needs Java 25")));
    CHECK(check.title.contains(QStringLiteral("25")));
}

TEST_CASE("jdk setting: an explicit folder that cannot run the project is reported, never replaced") {
    Fixture f;
    f.probe.environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
    f.probe.addJava(QStringLiteral("/jdk/bin/java"), kJava17);
    f.probe.addJava(QStringLiteral("/old/bin/java"), QStringLiteral("java version \"1.8.0_352\""));
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.jdkHomeOverride = QStringLiteral("/old");
    in.managedJavaAvailable = true;
    const JdkResolution jdk = f.checker().resolveJdk(in);
    CHECK(!jdk.found);
    CHECK(jdk.explicitFailed);
    const ToolchainCheck check = f.checker().checkJdk(in);
    CHECK(check.fixAction == FixAction::OpenSettings);
    CHECK(check.detail.contains(QStringLiteral("Java 8")));
}

TEST_CASE("wpilib version: an unverified version of a known season warns and the run continues") {
    Fixture f;
    ToolchainInput in = inputFor(QStringLiteral("java"), 2026);
    in.wpilibVersion = QStringLiteral("2026.2.1");
    CHECK(f.checker().checkWpilibVersion(in).status == CheckStatus::Ok);
    CHECK(f.checker().run(in).find(QString::fromLatin1(check_id::kWpilibVersion)) == nullptr);   // verified: no row
    in.wpilibVersion = QStringLiteral("2026.9.9");
    const ToolchainCheck check = f.checker().checkWpilibVersion(in);
    CHECK(check.status == CheckStatus::Warning);
    CHECK_FALSE(check.requiredToRun);
    CHECK(check.detail.contains(QStringLiteral("2026.9.9")));
    const ToolchainReport report = f.checker().run(in);
    REQUIRE(report.find(QString::fromLatin1(check_id::kWpilibVersion)) != nullptr);
    CHECK(report.find(QString::fromLatin1(check_id::kWpilibVersion))->status == CheckStatus::Warning);
    in.wpilibYear = 1999;   // a year without a season pack has nothing to verify against
    CHECK(f.checker().checkWpilibVersion(in).status == CheckStatus::NotApplicable);
}
