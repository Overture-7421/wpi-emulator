#include <doctest/doctest.h>

#include <algorithm>

#include "runtime_fixtures.h"
#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/core/launcher/robot_launcher.h"
#include "wpiemu/sdk/testing/environment.h"

using namespace wpiemu::core::launcher;
using wpiemu::sdk::EnvLookup;
using wpiemu::sdk::testing::envOf;
using wpiemu::test_runtime::ScratchDir;
namespace fs = std::filesystem;

namespace {

const std::string kLib = halsimExtensionFileName();

bool sameFile(const fs::path& a, const fs::path& b) {
    std::error_code ec;
    return fs::equivalent(a, b, ec) && !ec;
}

}  // namespace

TEST_CASE("extension locator: every documented layout is found relative to the executable") {
    struct Layout {
        const char* name;
        const char* appDir;   // directory of the executable
        const char* library;  // where the extension sits
    };
    const Layout layouts[] = {
        {"build tree / portable", "bin", "bin/"},
        {"windows install", "inst/bin", "inst/extensions/"},
        {"linux prefix / AppImage", "usr/bin", "usr/lib/wpiemu/"},
        {"macOS bundle", "App.app/Contents/MacOS", "App.app/Contents/PlugIns/wpiemu/"},
        {"macOS build tree", "out/bin/Release/App.app/Contents/MacOS", "out/bin/Release/"},
    };
    for (const Layout& l : layouts) {
        CAPTURE(l.name);
        ScratchDir sub;
        fs::create_directories(sub.path() / l.appDir);
        const fs::path lib = sub.write(std::string(l.library) + kLib, "x");
        const fs::path found = locateHalsimExtension(sub.path() / l.appDir, envOf({}));
        REQUIRE_FALSE(found.empty());
        CHECK(found.is_absolute());
        CHECK(sameFile(found, lib));
        CHECK(found == fs::canonical(lib));
        CHECK(found.string().find("..") == std::string::npos);
    }
}

TEST_CASE("extension locator: priority order and the fallback") {
    ScratchDir dir;
    const fs::path appDir = dir.path() / "app" / "Contents" / "MacOS";
    fs::create_directories(appDir);
    const fs::path build = dir.write("build/" + kLib, "build");
    const fs::path macTree = dir.write("" + kLib, "mac-tree");   // appDir/../../.. == dir
    const fs::path macBundle = dir.write("app/Contents/PlugIns/wpiemu/" + kLib, "bundle");
    const fs::path linux = dir.write("app/Contents/lib/wpiemu/" + kLib, "linux");
    const fs::path windows = dir.write("app/Contents/extensions/" + kLib, "windows");
    const fs::path beside = dir.write("app/Contents/MacOS/" + kLib, "beside");
    const fs::path extDir = dir.write("extdir/" + kLib, "extdir");
    const fs::path explicitFile = dir.write("explicit/custom.dll", "explicit");

    // exercise the list from the last candidate upwards
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({}), build), beside));
    fs::remove(beside);
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({}), build), windows));
    fs::remove(windows);
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({}), build), linux));
    fs::remove(linux);
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({}), build), macBundle));
    fs::remove(macBundle);
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({}), build), macTree));
    fs::remove(macTree);
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({}), build), build));
    CHECK(locateHalsimExtension(appDir, envOf({})).empty());  // no fallback given, nothing left

    // environment wins over everything; a missing file is skipped, not an error
    dir.write("app/Contents/MacOS/" + kLib, "beside");
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({{"WPIEMU_EXTENSION_DIR", (dir.path() / "extdir").string()}}), build), extDir));
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({{"WPIEMU_EXTENSION", explicitFile.string()},
                                                         {"WPIEMU_EXTENSION_DIR", (dir.path() / "extdir").string()}}), build), explicitFile));
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({{"WPIEMU_EXTENSION", (dir.path() / "nope.dll").string()},
                                                         {"WPIEMU_EXTENSION_DIR", (dir.path() / "extdir").string()}}), build), extDir));
    CHECK(sameFile(locateHalsimExtension(appDir, envOf({{"WPIEMU_EXTENSION_DIR", (dir.path() / "missing").string()}}), build),
                   appDir / kLib));
}

TEST_CASE("extension locator: candidate list is the documented one and a directory is not a library") {
    const fs::path appDir = fs::path("/opt/app/bin");
    std::vector<fs::path> searched;
    CHECK(locateHalsimExtension(appDir, envOf({{"WPIEMU_EXTENSION", "/x/y.dll"}, {"WPIEMU_EXTENSION_DIR", "/z"}}), "/b/lib", &searched).empty());
    const std::vector<fs::path> expected = {fs::path("/x/y.dll"),
                                            fs::path("/z") / kLib,
                                            (appDir / kLib).lexically_normal(),
                                            (appDir / ".." / "extensions" / kLib).lexically_normal(),
                                            (appDir / ".." / "lib" / "wpiemu" / kLib).lexically_normal(),
                                            (appDir / ".." / "PlugIns" / "wpiemu" / kLib).lexically_normal(),
                                            (appDir / ".." / ".." / ".." / kLib).lexically_normal(),
                                            fs::path("/b/lib")};
    CHECK(searched == expected);

    ScratchDir dir;
    fs::create_directories(dir.path() / "bin" / kLib);  // a folder named like the library
    CHECK(locateHalsimExtension(dir.path() / "bin", envOf({})).empty());
    CHECK(locateHalsimExtension({}, nullptr).empty());   // no app dir, no environment
}

// ---- packaged environment ---------------------------------------------------------------------------------------------

TEST_CASE("packaged env: Qt variables are dropped and AppDir entries stripped from the path-like variables") {
    const EnvLookup host = envOf({{"WPIEMU_PACKAGED", "appimage"},
                                  {"APPDIR", "/tmp/.mount_x"},
                                  {"LD_LIBRARY_PATH", "/tmp/.mount_x/usr/lib:/opt/keep/lib:/tmp/.mount_x"},
                                  {"PATH", "/tmp/.mount_x/usr/bin:/usr/bin:/tmp/.mount_xyz/bin:/bin"},
                                  {"XDG_DATA_DIRS", "/tmp/.mount_x/usr/share/"},
                                  {"QT_PLUGIN_PATH", "/tmp/.mount_x/usr/plugins"}});
    ProcessSpec spec;
    spec.env["QML2_IMPORT_PATH"] = "from-extra-env";  // an explicit request wins over the unset
    sanitizePackagedEnvironment(spec, host, false);

    for (const char* v : {"QT_PLUGIN_PATH", "QML2_IMPORT_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"})
        CHECK(std::find(spec.unsetEnv.begin(), spec.unsetEnv.end(), v) != spec.unsetEnv.end());
    CHECK(spec.env.at("QML2_IMPORT_PATH") == "from-extra-env");
    CHECK(spec.env.at("LD_LIBRARY_PATH") == "/opt/keep/lib");
    CHECK(spec.env.at("PATH") == "/usr/bin:/tmp/.mount_xyz/bin:/bin");  // a sibling dir with the same prefix stays
    CHECK(std::find(spec.unsetEnv.begin(), spec.unsetEnv.end(), "XDG_DATA_DIRS") != spec.unsetEnv.end());
    CHECK(spec.env.count("XDG_DATA_DIRS") == 0);
}

TEST_CASE("packaged env: untouched when not packaged, when nothing is under AppDir, or without AppDir") {
    {
        ProcessSpec spec;
        sanitizePackagedEnvironment(spec, envOf({{"APPDIR", "/a"}, {"PATH", "/a/bin:/usr/bin"}}), false);
        CHECK(spec.env.empty());
        CHECK(spec.unsetEnv.empty());
    }
    {
        ProcessSpec spec;
        sanitizePackagedEnvironment(spec, envOf({{"WPIEMU_PACKAGED", "appimage"}, {"APPDIR", "/a"}, {"PATH", "/usr/bin:/bin"}}), false);
        CHECK(spec.env.empty());  // PATH had nothing to strip
        CHECK(spec.unsetEnv.size() == 3);
    }
    {
        ProcessSpec spec;
        sanitizePackagedEnvironment(spec, envOf({{"WPIEMU_PACKAGED", "1"}, {"PATH", "/a/bin:/usr/bin"}}), false);
        CHECK(spec.env.empty());
        CHECK(spec.unsetEnv.size() == 3);
    }
}

TEST_CASE("packaged env: the launch plans apply it") {
    ScratchDir dir;
    LaunchRequest r;
    r.projectDir = dir.path() / "robot";
    fs::create_directories(r.projectDir);
    dir.write("robot/gradlew", "#!/bin/sh\n");
    dir.write("robot/gradlew.bat", "@echo off\r\n");
    r.extensionLibrary = dir.write("ext/" + kLib, "x");
    r.ipcEndpoint = "ep";
    r.scratchDir = dir.path() / "scratch";
    r.hostEnv = envOf({{"WPIEMU_PACKAGED", "appimage"}, {"APPDIR", "/app"}, {"PATH", "/app/usr/bin:/usr/bin"}});

    const auto plan = planGradleLaunch(r, false);
    REQUIRE(plan.ok());
    CHECK(plan.value().env.at("PATH") == "/usr/bin");
    CHECK(std::find(plan.value().unsetEnv.begin(), plan.value().unsetEnv.end(), "QT_PLUGIN_PATH") != plan.value().unsetEnv.end());

    const auto describe = planExternalDescribe(r, false);
    REQUIRE(describe.ok());
    CHECK(describe.value().env.at("PATH") == "/usr/bin");

    r.hostEnv = envOf({});  // not packaged: nothing is touched
    CHECK(planGradleLaunch(r, false).value().env.count("PATH") == 0);
    CHECK(planGradleLaunch(r, false).value().unsetEnv.empty());
}
