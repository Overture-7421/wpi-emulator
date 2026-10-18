#include <doctest/doctest.h>

#include <algorithm>
#include <map>

#include "runtime_fixtures.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/launcher/project_inspector.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::launcher;
using wpiemu::core::config::EnvLookup;
using wpiemu::core::config::JsonValueCodec;
using wpiemu::core::config::Platform;
using wpiemu::core::config::currentPlatform;
using wpiemu::test_runtime::ScratchDir;
namespace fs = std::filesystem;

namespace {

const JsonValueCodec kJson;

EnvLookup envOf(std::map<std::string, std::string> vars) {
    return [vars = std::move(vars)](std::string_view name) -> std::optional<std::string> {
        const auto it = vars.find(std::string(name));
        if (it == vars.end()) return std::nullopt;
        return it->second;
    };
}

constexpr const char* kJavaBuild = R"gradle(plugins {
    id "java"
    id "edu.wpi.first.GradleRIO" version "2026.2.1"
}
deploy { targets { roborio(getTargetTypeClass('RoboRIO')) { artifacts { frcJava(getArtifactTypeClass('FRCJavaArtifact')) { } } } } }
)gradle";

constexpr const char* kCppBuild = R"gradle(plugins {
    id "cpp"
    id "google-test-test-suite"
    id "edu.wpi.first.GradleRIO" version "2025.3.2"
}
deploy { targets { roborio(getTargetTypeClass('RoboRIO')) { artifacts { frcNative(getArtifactTypeClass('FRCNativeArtifact')) { } } } } }
)gradle";

void addWrapper(const ScratchDir& d, const std::string& project, const std::string& gradleVersion = "8.11") {
    d.write(project + "/gradlew", "#!/bin/sh\n");
    d.write(project + "/gradlew.bat", "@echo off\r\n");
    d.write(project + "/gradle/wrapper/gradle-wrapper.jar", "jar");
    d.write(project + "/gradle/wrapper/gradle-wrapper.properties",
            "distributionBase=GRADLE_USER_HOME\ndistributionUrl=https\\://services.gradle.org/distributions/gradle-" + gradleVersion + "-bin.zip\n");
}

ProjectReport inspect(const fs::path& dir) {
    const auto r = ProjectInspector(kJson).inspect(dir);
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
    return r.value();
}

// A fake WPILib home with a JDK, so toolchain checks do not depend on this machine.
fs::path makeWpilib(const ScratchDir& d, int year, const std::string& javaVersion = "17.0.12", bool withMaven = true) {
    const std::string root = "wpilib/" + std::to_string(year);
    const char* java = currentPlatform() == Platform::Windows ? "java.exe" : "java";
    d.write(root + "/jdk/bin/" + java, "x");
    d.write(root + "/jdk/release", "IMPLEMENTOR=\"x\"\nJAVA_VERSION=\"" + javaVersion + "\"\n");
    if (withMaven) fs::create_directories(d.path() / root / "maven");
    fs::create_directories(d.path() / root / "tools");
    return d.path() / "wpilib";
}

ToolchainContext context(const ScratchDir& d, const fs::path& wpilibRoot) {
    ToolchainContext c;
    c.platform = currentPlatform();
    c.env = envOf({{"PATH", ""}});
    c.wpilibRoots = {wpilibRoot};
    (void)d;
    return c;
}

}  // namespace

TEST_CASE("inspector: a Java GradleRIO project") {
    ScratchDir d;
    d.write("bot/build.gradle", kJavaBuild);
    d.write("bot/.wpilib/wpilib_preferences.json", R"json({"enableCppIntellisense": false, "currentLanguage": "java", "projectYear": "2026", "teamNumber": 7421})json");
    d.write("bot/vendordeps/Phoenix6-26.1.0.json", R"json({"fileName": "Phoenix6.json", "name": "CTRE-Phoenix (v6)", "version": "26.1.0", "frcYear": "2026", "uuid": "e995"})json");
    d.write("bot/vendordeps/photonlib.json", R"json({"name": "photonlib", "version": "v2026.1.1", "frcYear": "2026"})json");
    d.write("bot/vendordeps/notes.txt", "ignored");
    addWrapper(d, "bot");
    const ProjectReport r = inspect(d.path() / "bot");
    CHECK(r.name == "bot");
    CHECK(r.isWpilibProject);
    CHECK(r.hasBuildScript);
    CHECK(r.gradleRioFound);
    CHECK(r.gradleRioVersion == "2026.2.1");
    CHECK(r.wpilibYear == 2026);
    CHECK(r.language == "java");
    CHECK(r.teamNumber == 7421);
    CHECK(r.hasGradlewScript);
    CHECK(r.hasGradlewBat);
    CHECK(r.hasWrapperJar);
    CHECK(r.hasWrapperProperties);
    CHECK(r.gradleVersion == "8.11");
    REQUIRE(r.vendorDeps.size() == 2);
    CHECK(r.vendorDeps[0].fileName == "Phoenix6-26.1.0.json");
    CHECK(r.vendorDeps[0].name == "CTRE-Phoenix (v6)");
    CHECK(r.vendorDeps[0].version == "26.1.0");
    CHECK(r.hasVendorDep("phoenix"));
    CHECK(r.hasVendorDep("PHOTON"));
    CHECK_FALSE(r.hasVendorDep("rev"));
    for (const Issue& n : r.notes) CHECK(n.severity != Severity::Error);
}

TEST_CASE("inspector: a C++ project, preferences decide the language, team number as a string") {
    ScratchDir d;
    d.write("cpp/build.gradle", kCppBuild);
    d.write("cpp/.wpilib/wpilib_preferences.json", R"json({"currentLanguage": "cpp", "projectYear": "2025", "teamNumber": "254"})json");
    const ProjectReport r = inspect(d.path() / "cpp");
    CHECK(r.language == "cpp");
    CHECK(r.wpilibYear == 2025);
    CHECK(r.teamNumber == 254);
    CHECK(r.gradleRioVersion == "2025.3.2");
    CHECK_FALSE(r.hasGradlewBat);

    // without preferences the build script decides
    ScratchDir d2;
    d2.write("cpp/build.gradle", kCppBuild);
    CHECK(inspect(d2.path() / "cpp").language == "cpp");
    CHECK(inspect(d2.path() / "cpp").teamNumber == -1);
}

TEST_CASE("inspector: an out-of-range or malformed team number is ignored with a note, never an exception") {
    for (const char* team : {"\"99999999999999999999\"", "99999999999999999999", "\"100000\"", "123456789012", "\"-5\"", "\"12a\"", "\"\"", "1e400", "null"}) {
        CAPTURE(team);
        ScratchDir d;
        d.write("t/build.gradle", kJavaBuild);
        d.write("t/.wpilib/wpilib_preferences.json", std::string(R"json({"currentLanguage": "java", "projectYear": "2026", "teamNumber": )json") + team + "}");
        ProjectReport r;
        CHECK_NOTHROW(r = inspect(d.path() / "t"));
        CHECK(r.teamNumber == -1);
        CHECK(r.isWpilibProject);
    }
    for (const auto& [text, expected] : {std::pair{"\"99999\"", 99999}, std::pair{"\"0007\"", 7}, std::pair{"254", 254}}) {
        ScratchDir d;
        d.write("t/build.gradle", kJavaBuild);
        d.write("t/.wpilib/wpilib_preferences.json", std::string(R"json({"currentLanguage": "java", "teamNumber": )json") + text + "}");
        CHECK(inspect(d.path() / "t").teamNumber == expected);
    }
}

TEST_CASE("inspector: build script dialects for the GradleRIO version") {
    struct Case { const char* name; const char* build; const char* version; int year; };
    const Case cases[] = {
        {"groovy-double", "plugins { id \"edu.wpi.first.GradleRIO\" version \"2027.0.0-alpha-1\" }", "2027.0.0-alpha-1", 2027},
        {"groovy-single", "plugins { id 'edu.wpi.first.GradleRIO' version '2024.3.2' }", "2024.3.2", 2024},
        {"kotlin", "plugins { id(\"edu.wpi.first.GradleRIO\") version \"2025.1.1\" }", "2025.1.1", 2025},
        {"classpath", "buildscript { dependencies { classpath \"edu.wpi.first:GradleRIO:2023.4.3\" } }", "2023.4.3", 2023},
    };
    for (const Case& c : cases) {
        ScratchDir d;
        d.write("p/build.gradle", c.build);
        const ProjectReport r = inspect(d.path() / "p");
        INFO(c.name);
        CHECK(r.isWpilibProject);
        CHECK(r.gradleRioVersion == c.version);
        CHECK(r.wpilibYear == c.year);
    }
    ScratchDir kts;
    kts.write("p/build.gradle.kts", "plugins { id(\"edu.wpi.first.GradleRIO\") version \"2026.1.1\" }");
    CHECK(inspect(kts.path() / "p").gradleRioVersion == "2026.1.1");
}

TEST_CASE("inspector: version from a variable falls back to the preferences / settings year, with a note") {
    ScratchDir d;
    d.write("p/build.gradle", "plugins { id \"edu.wpi.first.GradleRIO\" version gradleRioVersion }");
    d.write("p/.wpilib/wpilib_preferences.json", R"json({"projectYear": "2026"})json");
    ProjectReport r = inspect(d.path() / "p");
    CHECK(r.isWpilibProject);
    CHECK(r.gradleRioVersion.empty());
    CHECK(r.wpilibYear == 2026);
    CHECK(std::any_of(r.notes.begin(), r.notes.end(), [](const Issue& n) { return n.severity == Severity::Warning; }));

    ScratchDir s;
    s.write("p/build.gradle", "plugins { id 'edu.wpi.first.GradleRIO' version project.ext.v }");
    s.write("p/settings.gradle", "String frcYear = '2024'\n");
    CHECK(inspect(s.path() / "p").wpilibYear == 2024);
}

TEST_CASE("inspector: not a WPILib project is a report, not an error") {
    ScratchDir d;
    d.write("plain/readme.md", "hi");
    const ProjectReport empty = inspect(d.path() / "plain");
    CHECK_FALSE(empty.isWpilibProject);
    CHECK_FALSE(empty.hasBuildScript);
    CHECK(std::any_of(empty.notes.begin(), empty.notes.end(), [](const Issue& n) { return n.severity == Severity::Error; }));

    d.write("other/build.gradle", "plugins { id 'java' }\n");
    const ProjectReport other = inspect(d.path() / "other");
    CHECK(other.hasBuildScript);
    CHECK_FALSE(other.isWpilibProject);
    CHECK_FALSE(other.gradleRioFound);

    CHECK(ProjectInspector(kJson).inspect(d.path() / "nope").error().code == ErrorCode::NotFound);
    CHECK(ProjectInspector(kJson).inspect("").error().code == ErrorCode::InvalidArgument);
    const fs::path file = d.write("file.txt", "x");
    CHECK(ProjectInspector(kJson).inspect(file).error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("inspector: hostile or broken files do not break the report") {
    ScratchDir d;
    d.write("b/build.gradle", kJavaBuild);
    d.write("b/.wpilib/wpilib_preferences.json", "{ this is not json");
    d.write("b/vendordeps/broken.json", "[[[");
    d.write("b/vendordeps/noname.json", R"json({"version": "1"})json");
    d.write("b/gradle/wrapper/gradle-wrapper.properties", "distributionUrl=weird\n");
    const ProjectReport r = inspect(d.path() / "b");
    CHECK(r.isWpilibProject);
    CHECK(r.teamNumber == -1);
    CHECK(r.gradleVersion.empty());
    REQUIRE(r.vendorDeps.size() == 2);
    CHECK_FALSE(r.vendorDeps[0].readable);
    CHECK(r.vendorDeps[0].name == "broken");  // falls back to the file name
    CHECK(r.vendorDeps[1].name == "noname");
    CHECK(r.notes.size() >= 2);
    CHECK(r.language == "java");  // from the build script despite broken preferences

    // conflicting signals: the preferences win but it is noted
    ScratchDir c;
    c.write("p/build.gradle", kCppBuild);
    c.write("p/.wpilib/wpilib_preferences.json", R"json({"currentLanguage": "java"})json");
    const ProjectReport conflict = inspect(c.path() / "p");
    CHECK(conflict.language == "java");
    CHECK(std::any_of(conflict.notes.begin(), conflict.notes.end(), [](const Issue& n) { return n.message.find("disagrees") != std::string::npos; }));

    // no signals at all: the sources decide
    ScratchDir s;
    s.write("p/build.gradle", "plugins { id 'edu.wpi.first.GradleRIO' version '2026.1.1' }");
    s.write("p/src/main/cpp/Robot.cpp", "int main(){}");
    CHECK(inspect(s.path() / "p").language == "cpp");
    ScratchDir u;
    u.write("p/build.gradle", "plugins { id 'edu.wpi.first.GradleRIO' version '2026.1.1' }");
    CHECK(inspect(u.path() / "p").language.empty());
}

TEST_CASE("wpilib install discovery per OS") {
    const auto win = wpilibRootCandidates(Platform::Windows, envOf({{"PUBLIC", "D:\\Public"}}));
    REQUIRE(win.size() == 2);
    CHECK(win[0] == fs::path("D:\\Public") / "wpilib");
    CHECK(win[1] == fs::path("C:\\Users\\Public") / "wpilib");
    const auto winDefault = wpilibRootCandidates(Platform::Windows, envOf({}));
    CHECK(winDefault == std::vector<fs::path>{fs::path("C:\\Users\\Public") / "wpilib"});
    CHECK(wpilibRootCandidates(Platform::Linux, envOf({{"HOME", "/home/a"}})) == std::vector<fs::path>{fs::path("/home/a") / "wpilib"});
    CHECK(wpilibRootCandidates(Platform::MacOS, envOf({{"HOME", "/Users/a"}})) == std::vector<fs::path>{fs::path("/Users/a") / "wpilib"});
    CHECK(wpilibRootCandidates(Platform::Linux, envOf({})).empty());
    CHECK(wpilibRootCandidates(Platform::Linux, nullptr).empty());

    ScratchDir d;
    const fs::path root = makeWpilib(d, 2026);
    const auto install = findWpilibInstall(2026, {d.path() / "elsewhere", root});
    REQUIRE(install.has_value());
    CHECK(install->home == root / "2026");
    CHECK(install->hasMaven);
    CHECK(install->hasTools);
    CHECK_FALSE(install->hasRoboRioToolchain);
    CHECK(fs::exists(install->jdkHome));
    CHECK_FALSE(findWpilibInstall(2025, {root}).has_value());
}

TEST_CASE("toolchain: a healthy Java setup is ready") {
    ScratchDir d;
    d.write("bot/build.gradle", kJavaBuild);
    d.write("bot/.wpilib/wpilib_preferences.json", R"json({"currentLanguage": "java", "teamNumber": 1234})json");
    addWrapper(d, "bot");
    d.write("bot/vendordeps/Phoenix6.json", R"json({"name": "CTRE-Phoenix (v6)", "version": "26.1.0", "frcYear": "2026"})json");
    const fs::path root = makeWpilib(d, 2026);
    const fs::path ext = d.write("ext/halsim_wpiemu.dll", "x");

    ToolchainContext ctx = context(d, root);
    ctx.hasGamePlugin = [](int year) { return year == 2026; };
    ctx.extensionLibrary = ext;
    const ToolchainReport report = inspectToolchain(inspect(d.path() / "bot"), ctx);
    CHECK(report.ready());
    CHECK(report.find("project")->status == CheckStatus::Ok);
    CHECK(report.find("gradlerio")->status == CheckStatus::Ok);
    CHECK(report.find("game_plugin")->status == CheckStatus::Ok);
    CHECK(report.find("gradle_wrapper")->status == CheckStatus::Ok);
    CHECK(report.find("wpilib_install")->status == CheckStatus::Ok);
    CHECK(report.find("jdk")->status == CheckStatus::Ok);
    CHECK(report.find("cpp_toolchain") == nullptr);  // Java project
    CHECK(report.find("vendordeps")->detail.find("Phoenix 6 detected") != std::string::npos);
    CHECK(report.find("team")->detail == "1234");
    CHECK(report.find("extension")->status == CheckStatus::Ok);
    CHECK(report.find("nope") == nullptr);
    CHECK(report.javaVersion == "17.0.12");
    CHECK(report.jdkHome == root / "2026" / "jdk");
    REQUIRE(report.wpilib.has_value());
}

TEST_CASE("toolchain: each problem is reported with a fix") {
    ScratchDir d;
    d.write("bot/build.gradle", kJavaBuild);  // no wrapper at all
    ToolchainContext ctx = context(d, d.path() / "no-wpilib");
    ctx.hasGamePlugin = [](int) { return false; };
    ctx.extensionLibrary = d.path() / "missing-ext.dll";
    const ToolchainReport report = inspectToolchain(inspect(d.path() / "bot"), ctx);
    CHECK_FALSE(report.ready());
    CHECK(report.find("gradle_wrapper")->status == CheckStatus::Error);
    CHECK_FALSE(report.find("gradle_wrapper")->fix.empty());
    CHECK(report.find("jdk")->status == CheckStatus::Error);
    CHECK(report.find("wpilib_install")->status == CheckStatus::Warning);
    CHECK(report.find("game_plugin")->status == CheckStatus::Warning);
    CHECK(report.find("extension")->status == CheckStatus::Error);
    CHECK(report.find("team")->status == CheckStatus::Info);

    // not a WPILib project at all
    ScratchDir plain;
    plain.write("p/readme", "x");
    const ToolchainReport notProject = inspectToolchain(inspect(plain.path() / "p"), context(plain, plain.path()));
    CHECK_FALSE(notProject.ready());
    CHECK(notProject.find("project")->status == CheckStatus::Error);

    // incomplete wrapper is only a warning
    ScratchDir partial;
    partial.write("bot/build.gradle", kJavaBuild);
    partial.write("bot/gradlew", "x");
    partial.write("bot/gradlew.bat", "x");
    const ToolchainReport w = inspectToolchain(inspect(partial.path() / "bot"), context(partial, makeWpilib(partial, 2026)));
    CHECK(w.find("gradle_wrapper")->status == CheckStatus::Warning);
    CHECK(w.ready());
}

TEST_CASE("toolchain: JDK sources - JAVA_HOME, PATH, old versions") {
    ScratchDir d;
    d.write("bot/build.gradle", kJavaBuild);
    addWrapper(d, "bot");
    const ProjectReport project = inspect(d.path() / "bot");
    const char* java = currentPlatform() == Platform::Windows ? "java.exe" : "java";

    // JAVA_HOME fallback (WPILib not installed)
    d.write(std::string("jh/bin/") + java, "x");
    d.write("jh/release", "JAVA_VERSION=\"21.0.2\"\n");
    ToolchainContext ctx = context(d, d.path() / "none");
    ctx.env = envOf({{"JAVA_HOME", (d.path() / "jh").string()}, {"PATH", ""}});
    ToolchainReport r = inspectToolchain(project, ctx);
    CHECK(r.find("jdk")->status == CheckStatus::Ok);
    CHECK(r.find("jdk")->detail.find("JAVA_HOME") != std::string::npos);
    CHECK(r.javaVersion == "21.0.2");

    // too old
    d.write("old/bin/" + std::string(java), "x");
    d.write("old/release", "JAVA_VERSION=\"1.8.0_392\"\n");
    ctx.env = envOf({{"JAVA_HOME", (d.path() / "old").string()}, {"PATH", ""}});
    r = inspectToolchain(project, ctx);
    CHECK(r.find("jdk")->status == CheckStatus::Warning);
    CHECK(r.ready());

    // only java on PATH
    d.write("pathbin/" + std::string(java), "x");
    ctx.env = envOf({{"PATH", (d.path() / "pathbin").string()}});
    r = inspectToolchain(project, ctx);
    CHECK(r.find("jdk")->status == CheckStatus::Warning);
    CHECK(r.jdkHome.empty());

    // nothing
    ctx.env = envOf({{"PATH", ""}});
    CHECK(inspectToolchain(project, ctx).find("jdk")->status == CheckStatus::Error);
}

TEST_CASE("toolchain: C++ projects need a host compiler, vendordep years are compared") {
    ScratchDir d;
    d.write("cpp/build.gradle", kCppBuild);
    d.write("cpp/.wpilib/wpilib_preferences.json", R"json({"currentLanguage": "cpp"})json");
    addWrapper(d, "cpp");
    d.write("cpp/vendordeps/old.json", R"json({"name": "OldLib", "version": "1", "frcYear": "2023"})json");
    const ProjectReport project = inspect(d.path() / "cpp");
    ToolchainContext ctx = context(d, makeWpilib(d, 2025));
    ctx.env = envOf({{"PATH", ""}, {"ProgramFiles", (d.path() / "nothing").string()}});

    ToolchainReport report = inspectToolchain(project, ctx);
    CHECK(report.find("cpp_toolchain")->status == CheckStatus::Error);
    CHECK_FALSE(report.ready());
    CHECK(report.find("vendordeps")->status == CheckStatus::Warning);
    CHECK(report.find("vendordeps")->detail.find("2023") != std::string::npos);

    const char* compiler = currentPlatform() == Platform::Windows ? "cl.exe" : currentPlatform() == Platform::MacOS ? "clang++" : "g++";
    const fs::path tool = d.write(std::string("compiler/") + compiler, "x");
    fs::permissions(tool, fs::perms::owner_exec | fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::add);
    ctx.hostCompilerDirs = {d.path() / "compiler"};
    report = inspectToolchain(project, ctx);
    CHECK(report.find("cpp_toolchain")->status == CheckStatus::Ok);
    CHECK(report.ready());

    if (currentPlatform() == Platform::Windows) {  // Visual Studio detection by directory layout
        ScratchDir vs;
        fs::create_directories(vs.path() / "Microsoft Visual Studio" / "2022" / "BuildTools" / "VC" / "Tools" / "MSVC");
        ToolchainContext c2 = context(vs, makeWpilib(vs, 2025));
        c2.env = envOf({{"PATH", ""}, {"ProgramFiles", vs.path().string()}});
        vs.write("cpp/build.gradle", kCppBuild);
        addWrapper(vs, "cpp");
        CHECK(inspectToolchain(inspect(vs.path() / "cpp"), c2).find("cpp_toolchain")->status == CheckStatus::Ok);
    }
}

// ---- PhotonLib: which PhotonVision message definition the robot's PhotonLib speaks -------------------------------------------------

namespace {

constexpr const char* kPhotonUuid = "4b2ff16a964b5e2bf04be0c1454d91c4";

std::string photonlibJson(const std::string& version) {
    return R"json({"fileName": "photonlib.json", "name": "photonlib", "frcYear": "2026", "version": ")json" + version + "\"}";
}

// A healthy Java project, so ready() only depends on what a test does to the vendordeps (file name -> JSON text).
struct VisionProject {
    ProjectReport report;
    ToolchainReport toolchain;
};

VisionProject visionProject(const ScratchDir& d, const std::map<std::string, std::string>& vendordeps) {
    d.write("bot/build.gradle", kJavaBuild);
    addWrapper(d, "bot");
    for (const auto& [file, text] : vendordeps) d.write("bot/vendordeps/" + file, text);
    VisionProject p;
    p.report = inspect(d.path() / "bot");
    p.toolchain = inspectToolchain(p.report, context(d, makeWpilib(d, 2026)));
    return p;
}

}  // namespace

TEST_CASE("inspector: the PhotonLib version and the message UUID it implies") {
    struct Case { const char* version; const char* uuid; };
    const Case cases[] = {
        {"v2026.3.4", kPhotonUuid},
        {"v2026.1.1", kPhotonUuid},
        {"2025.3.2", kPhotonUuid},  // the "v" is optional
        {"2026.9.0", ""},           // newer than any verified release
        {"2024.3.1", ""},
        {"v2026.3.4-beta-1", ""},   // strict: no suffixes
        {"garbage", ""},
    };
    for (const Case& c : cases) {
        INFO(c.version);
        ScratchDir d;
        const ProjectReport r = visionProject(d, {{"photonlib.json", photonlibJson(c.version)}}).report;
        CHECK(r.hasPhotonlib());
        CHECK(r.photonlibVersion == c.version);
        CHECK(r.photonMessageUuid == c.uuid);
    }

    ScratchDir none;
    const ProjectReport absent =
        visionProject(none, {{"Phoenix6.json", R"json({"name": "CTRE-Phoenix (v6)", "version": "26.1.0", "frcYear": "2026"})json"}}).report;
    CHECK_FALSE(absent.hasPhotonlib());
    CHECK(absent.photonlibVersion.empty());
    CHECK(absent.photonMessageUuid.empty());

    ScratchDir broken;
    const ProjectReport unreadable = visionProject(broken, {{"photonlib.json", "[[["}}).report;
    CHECK(unreadable.hasPhotonlib());  // still listed, by its file name
    CHECK_FALSE(unreadable.vendorDeps.at(0).readable);
    CHECK(unreadable.photonlibVersion.empty());
    CHECK(unreadable.photonMessageUuid.empty());

    ScratchDir versionless;
    const ProjectReport noVersion = visionProject(versionless, {{"photonlib.json", R"json({"name": "photonlib"})json"}}).report;
    CHECK(noVersion.hasPhotonlib());
    CHECK(noVersion.photonlibVersion.empty());
    CHECK(noVersion.photonMessageUuid.empty());

    // the name or the file name finds it, in any case; a copy without a version does not hide the real one
    ScratchDir named;
    CHECK(visionProject(named, {{"photon.json", R"json({"name": "PhotonLib", "version": "v2026.3.4"})json"}}).report.photonMessageUuid == kPhotonUuid);
    ScratchDir filed;
    CHECK(visionProject(filed, {{"PhotonLib-2026.json", R"json({"name": "PhotonVision", "version": "2025.3.2"})json"}}).report.photonMessageUuid == kPhotonUuid);
    ScratchDir twice;
    const ProjectReport copies =
        visionProject(twice, {{"photonlib-a.json", R"json({"name": "photonlib"})json"}, {"photonlib.json", photonlibJson("v2026.3.4")}}).report;
    CHECK(copies.photonlibVersion == "v2026.3.4");
    CHECK(copies.photonMessageUuid == kPhotonUuid);
}

TEST_CASE("toolchain: the vision check explains PhotonLib support and never blocks the launch") {
    ScratchDir none;
    const VisionProject absent =
        visionProject(none, {{"Phoenix6.json", R"json({"name": "CTRE-Phoenix (v6)", "version": "26.1.0", "frcYear": "2026"})json"}});
    const ToolchainCheck* noPhoton = absent.toolchain.find("vision");
    REQUIRE(noPhoton != nullptr);
    CHECK(noPhoton->status == CheckStatus::Info);
    CHECK(noPhoton->detail == "No PhotonLib vendor library: PhotonVision cameras are not used (Limelight cameras need no vendor library)");
    CHECK(noPhoton->fix.empty());
    CHECK(absent.toolchain.ready());

    for (const char* version : {"v2026.3.4", "2025.3.2"}) {
        INFO(version);
        ScratchDir d;
        const VisionProject p = visionProject(d, {{"photonlib.json", photonlibJson(version)}});
        const ToolchainCheck* c = p.toolchain.find("vision");
        REQUIRE(c != nullptr);
        CHECK(c->status == CheckStatus::Ok);
        CHECK(c->detail == std::string("PhotonLib ") + version + ": supported (message definition 4b2ff16a...)");
        CHECK(c->fix.empty());
        CHECK(p.toolchain.ready());
    }

    for (const char* version : {"2026.9.0", "garbage"}) {  // a release the emulator has not verified: PhotonVision cameras stay off
        INFO(version);
        ScratchDir d;
        const VisionProject p = visionProject(d, {{"photonlib.json", photonlibJson(version)}});
        const ToolchainCheck* c = p.toolchain.find("vision");
        REQUIRE(c != nullptr);
        CHECK(c->status == CheckStatus::Warning);
        CHECK(c->detail.find(std::string("PhotonLib ") + version) != std::string::npos);
        CHECK(c->detail.find("PhotonVision cameras are disabled for this project to avoid crashing the robot") != std::string::npos);
        CHECK(c->fix.find("vision.photonMessageUuid") != std::string::npos);
        CHECK(c->fix.find("robot.json") != std::string::npos);
        CHECK(c->fix.find(kPhotonUuid) != std::string::npos);
        CHECK(c->fix.find("Update the emulator") != std::string::npos);
        CHECK(p.toolchain.ready());
    }

    for (const char* broken : {"[[[", "{\"name\": \"photonlib\"}"}) {  // unreadable file, no version
        INFO(broken);
        ScratchDir d;
        const VisionProject p = visionProject(d, {{"photonlib.json", broken}});
        const ToolchainCheck* c = p.toolchain.find("vision");
        REQUIRE(c != nullptr);
        CHECK(c->status == CheckStatus::Warning);
        CHECK(c->detail.find("could not be read") != std::string::npos);
        CHECK(c->detail.find("PhotonVision cameras are disabled") != std::string::npos);
        CHECK_FALSE(c->fix.empty());
        CHECK(p.toolchain.ready());
    }
}

// ---- the real project on this machine (read-only) ------------------------------------------------------------------------------

namespace {
std::map<std::string, std::pair<std::uintmax_t, fs::file_time_type>> snapshotTree(const fs::path& root) {
    std::map<std::string, std::pair<std::uintmax_t, fs::file_time_type>> out;
    std::error_code ec;
    for (const char* sub : {"", ".wpilib", "vendordeps", "gradle/wrapper"}) {
        for (fs::directory_iterator it(root / sub, ec), end; !ec && it != end; it.increment(ec))
            if (it->is_regular_file(ec)) out[it->path().string()] = {it->file_size(ec), it->last_write_time(ec)};
    }
    return out;
}
}  // namespace

TEST_CASE("inspector: the FRC-Shelby-2026OS project, read only") {
    const fs::path shelby = "C:\\Users\\alber\\Documents\\Gits\\FRC-Shelby-2026OS";
    std::error_code ec;
    if (!fs::is_directory(shelby, ec)) {
        MESSAGE("FRC-Shelby-2026OS is not on this machine: skipped");
        return;
    }
    const auto before = snapshotTree(shelby);
    const ProjectReport r = inspect(shelby);
    CHECK(r.isWpilibProject);
    CHECK(r.language == "java");
    CHECK(r.gradleRioVersion == "2026.2.1");
    CHECK(r.wpilibYear == 2026);
    CHECK(r.teamNumber == 7421);
    CHECK(r.hasGradlewBat);
    CHECK(r.hasGradlewScript);
    CHECK(r.hasWrapperJar);
    CHECK(r.gradleVersion == "8.11");
    CHECK(r.vendorDeps.size() >= 5);
    CHECK(r.hasVendorDep("Phoenix6"));
    CHECK(r.hasVendorDep("photonlib"));
    CHECK(r.hasVendorDep("WPILibNewCommands"));
    for (const VendorDep& d : r.vendorDeps) CHECK(d.readable);

    ToolchainContext ctx;
    ctx.hasGamePlugin = [](int y) { return y == 2026; };
    const ToolchainReport report = inspectToolchain(r, ctx);
    CHECK(report.find("project")->status == CheckStatus::Ok);
    CHECK(report.find("gradlerio")->status == CheckStatus::Ok);
    CHECK(report.find("game_plugin")->status == CheckStatus::Ok);
    CHECK(report.find("vendordeps")->detail.find("Phoenix 6 detected") != std::string::npos);
    if (fs::is_directory("C:\\Users\\Public\\wpilib\\2026", ec)) {  // only meaningful where WPILib 2026 is installed
        CHECK(report.find("wpilib_install")->status == CheckStatus::Ok);
        CHECK(report.find("jdk")->status == CheckStatus::Ok);
        CHECK(report.ready());
        CHECK_FALSE(report.javaVersion.empty());
    }
    CHECK(snapshotTree(shelby) == before);  // nothing was touched
}
