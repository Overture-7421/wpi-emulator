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

// A fake WPILib root holding one season with a JDK, so discovery does not depend on this machine.
fs::path makeWpilib(const ScratchDir& d, int year) {
    const std::string root = "wpilib/" + std::to_string(year);
    const char* java = currentPlatform() == Platform::Windows ? "java.exe" : "java";
    d.write(root + "/jdk/bin/" + java, "x");
    fs::create_directories(d.path() / root / "maven");
    fs::create_directories(d.path() / root / "tools");
    return d.path() / "wpilib";
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
    CHECK(r.photonLibVersion() == "v2026.1.1");
    for (const Issue& n : r.notes) CHECK(n.severity != Severity::Error);
}

TEST_CASE("inspector: photonlib version comes from the photon vendordep and is empty without one") {
    ScratchDir d;
    d.write("bot/build.gradle", kJavaBuild);
    d.write("bot/vendordeps/Phoenix6.json", R"json({"name": "CTRE-Phoenix (v6)", "version": "26.1.0"})json");
    CHECK(inspect(d.path() / "bot").photonLibVersion().empty());
    d.write("bot/vendordeps/photonlib.json", R"json({"name": "photonlib", "version": "v2026.3.4"})json");
    CHECK(inspect(d.path() / "bot").photonLibVersion() == "v2026.3.4");
    d.write("bot/vendordeps/photonlib.json", R"json({"name": "photonlib"})json");  // no version declared
    CHECK(inspect(d.path() / "bot").photonLibVersion().empty());
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
    // "install for this user only" puts the season under the profile folder
    const auto perUser = wpilibRootCandidates(Platform::Windows, envOf({{"USERPROFILE", "C:\\Users\\me"}}));
    CHECK(perUser == std::vector<fs::path>{fs::path("C:\\Users\\Public") / "wpilib", fs::path("C:\\Users\\me") / "wpilib"});
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

    // an explicit season folder (the Settings override) is read the same way
    const auto at = wpilibInstallAt(root / "2026", 2026);
    REQUIRE(at.has_value());
    CHECK(at->jdkHome == root / "2026" / "jdk");
    CHECK_FALSE(wpilibInstallAt(root / "2024", 2024).has_value());
}

// ---- a real WPILib project (read-only) ----------------------------------------------------------------------------------------

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

TEST_CASE("inspector: a real WPILib sample project, read only") {
    const fs::path sample = fs::path(WPIEMU_SAMPLES_DIR) / "vision_robot" / "java";
    REQUIRE(fs::is_directory(sample));
    const auto before = snapshotTree(sample);
    const ProjectReport r = inspect(sample);
    CHECK(r.isWpilibProject);
    CHECK(r.language == "java");
    CHECK(r.gradleRioVersion == "2026.2.1");
    CHECK(r.wpilibYear == 2026);
    CHECK(r.teamNumber == 7421);
    CHECK(r.hasGradlewBat);
    CHECK(r.hasGradlewScript);
    CHECK(r.hasWrapperJar);
    CHECK(r.gradleVersion == "8.11");
    CHECK(r.vendorDeps.size() >= 3);
    CHECK(r.hasVendorDep("Phoenix6"));
    CHECK(r.hasVendorDep("photonlib"));
    CHECK(r.hasVendorDep("WPILibNewCommands"));
    for (const VendorDep& d : r.vendorDeps) CHECK(d.readable);
    CHECK(snapshotTree(sample) == before);  // nothing was touched
}
