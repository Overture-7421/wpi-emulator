#include <doctest/doctest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "runtime_fixtures.h"
#include "wpiemu/core/launcher/init_script.h"
#include "wpiemu/core/launcher/process.h"
#include "wpiemu/core/launcher/robot_launcher.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::launcher;
using wpiemu::core::config::JsonValueCodec;
using wpiemu::test_runtime::ScratchDir;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

#ifdef _WIN32
constexpr bool kWindows = true;
#else
constexpr bool kWindows = false;
#endif
#if defined(__APPLE__)
constexpr bool kMac = true;
#else
constexpr bool kMac = false;
#endif

std::string lowerCase(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

struct CollectingSink final : ILogSink {
    void line(OutputStream stream, std::string_view text) override {
        std::lock_guard lock(m);
        lines.push_back({stream, std::string(text)});
    }
    std::vector<std::string> texts(OutputStream s) const {
        std::lock_guard lock(m);
        std::vector<std::string> out;
        for (const auto& [st, t] : lines)
            if (st == s) out.push_back(t);
        return out;
    }
    bool has(OutputStream s, const std::string& text) const {
        const auto v = texts(s);
        return std::find(v.begin(), v.end(), text) != v.end();
    }
    mutable std::mutex m;
    std::vector<std::pair<OutputStream, std::string>> lines;
};

// A command that works with the platform shell.
ProcessSpec shell(const std::string& script) {
    ProcessSpec s;
    if (kWindows) {
        s.program = "cmd.exe";
        s.args = {"/d", "/s", "/c", script};
    } else {
        s.program = "/bin/sh";
        s.args = {"-c", script};
    }
    s.label = "shell";
    return s;
}

LaunchRequest makeRequest(const ScratchDir& dir, LaunchMode mode = LaunchMode::GradleInitScript, const std::string& language = "java") {
    LaunchRequest r;
    r.projectDir = dir.path() / "robot";
    fs::create_directories(r.projectDir);
    dir.write("robot/gradlew", "#!/bin/sh\n");
    dir.write("robot/gradlew.bat", "@echo off\r\n");
    r.extensionLibrary = dir.write("ext/halsim_wpiemu.dll", "x");
    r.ipcEndpoint = "\\\\.\\pipe\\wpiemu-test";
    r.scratchDir = dir.path() / "scratch";
    r.mode = mode;
    r.language = language;
    return r;
}

}  // namespace

// ---- pure helpers ---------------------------------------------------------------------------------------------------

TEST_CASE("line splitter: any chunking yields the same lines") {
    const std::string text = "alpha\r\nbeta\n\ngamma\r\ntail without newline";
    for (std::size_t chunk : {std::size_t{1}, std::size_t{3}, std::size_t{7}, text.size()}) {
        LineSplitter splitter;
        std::vector<std::string> lines;
        const auto on = [&](std::string_view l) { lines.emplace_back(l); };
        for (std::size_t i = 0; i < text.size(); i += chunk) splitter.feed(std::string_view(text).substr(i, chunk), on);
        CHECK(lines == std::vector<std::string>{"alpha", "beta", "", "gamma"});
        splitter.flush(on);
        REQUIRE(lines.size() == 5);
        CHECK(lines.back() == "tail without newline");
        splitter.flush(on);  // nothing left
        CHECK(lines.size() == 5);
    }
}

TEST_CASE("line splitter: runaway lines are cut at the limit, lone CR stays") {
    LineSplitter splitter(8);
    std::vector<std::string> lines;
    const auto on = [&](std::string_view l) { lines.emplace_back(l); };
    splitter.feed("0123456789abcdef\n", on);
    CHECK(lines == std::vector<std::string>{"01234567", "89abcdef"});
    lines.clear();
    LineSplitter cr;
    cr.feed("progress 1\rprogress 2\n", [&](std::string_view l) { lines.emplace_back(l); });
    CHECK(lines == std::vector<std::string>{"progress 1\rprogress 2"});
}

TEST_CASE("environment merge: overrides, unsets, Windows case-insensitivity, odd entries") {
    ProcessSpec spec;
    spec.env = {{"Path", "C:\\new"}, {"ADDED", "1"}};
    spec.unsetEnv = {"REMOVE_ME"};
    const std::vector<std::string> base = {"PATH=C:\\old", "REMOVE_ME=x", "KEEP=y", "=C:=C:\\dir", "garbage", "EMPTY="};
    auto merged = mergeEnvironment(base, spec, true);
    CHECK(std::find(merged.begin(), merged.end(), "Path=C:\\new") != merged.end());  // replaced the PATH entry despite the case
    CHECK(std::count_if(merged.begin(), merged.end(), [](const std::string& s) { return s.size() > 4 && (s.rfind("PATH=", 0) == 0 || s.rfind("Path=", 0) == 0); }) == 1);
    CHECK(std::find(merged.begin(), merged.end(), "KEEP=y") != merged.end());
    CHECK(std::find(merged.begin(), merged.end(), "ADDED=1") != merged.end());
    CHECK(std::find(merged.begin(), merged.end(), "EMPTY=") != merged.end());
    CHECK(std::find(merged.begin(), merged.end(), "REMOVE_ME=x") == merged.end());
    CHECK(std::find(merged.begin(), merged.end(), "=C:=C:\\dir") != merged.end());
    CHECK(std::find(merged.begin(), merged.end(), "garbage") == merged.end());
    CHECK(std::is_sorted(merged.begin(), merged.end(), [](const std::string& a, const std::string& b) {
        std::string x = a.substr(0, a.find('=', 1)), y = b.substr(0, b.find('=', 1));
        for (char& c : x) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        for (char& c : y) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        return x < y;
    }));

    auto posix = mergeEnvironment(base, spec, false);  // case-sensitive: both Path and PATH survive
    CHECK(std::find(posix.begin(), posix.end(), "PATH=C:\\old") != posix.end());
    CHECK(std::find(posix.begin(), posix.end(), "Path=C:\\new") != posix.end());
}

TEST_CASE("Windows argument quoting follows CommandLineToArgvW") {
    CHECK(quoteWindowsArgument("simple") == "simple");
    CHECK(quoteWindowsArgument("") == "\"\"");
    CHECK(quoteWindowsArgument("with space") == "\"with space\"");
    CHECK(quoteWindowsArgument("say \"hi\"") == "\"say \\\"hi\\\"\"");
    CHECK(quoteWindowsArgument("C:\\dir with space\\") == "\"C:\\dir with space\\\\\"");  // trailing backslashes doubled before the closing quote
    CHECK(quoteWindowsArgument("a\\b") == "a\\b");
    CHECK(quoteWindowsArgument("back\\\\\"quote") == "\"back\\\\\\\\\\\"quote\"");
    CHECK(isSafeForBatchFile("C:\\Users\\a b\\x.gradle"));
    for (const char* bad : {"100%", "say \"x\"", "a\" & calc & \"", "a\rb", "a\nb"}) CHECK_FALSE(isSafeForBatchFile(bad));
    CHECK_FALSE(isSafeForBatchFile(std::string_view("a\0b", 3)));
    // cmd metacharacters are fine: they get quoted instead
    for (const char* ok : {"R&D", "Team 1&2", "a^b", "a|b", "a>b", "a<b", "a(b)", "wow!", "C:\\Program Files (x86)\\x"}) CHECK(isSafeForBatchFile(ok));
}

TEST_CASE("cmd quoting wraps every argument cmd would otherwise act on") {
    CHECK(quoteBatchArgument("simple") == "simple");
    CHECK(quoteBatchArgument("C:\\dir\\file.gradle") == "C:\\dir\\file.gradle");
    CHECK(quoteBatchArgument("") == "\"\"");
    CHECK(quoteBatchArgument("with space") == "\"with space\"");
    for (const char* meta : {"R&D", "a^b", "a|b", "a<b", "a>b", "a(b", "a)b", "a!b", "a,b", "a;b", "a=b"}) {
        const std::string q = quoteBatchArgument(meta);
        CHECK(q == std::string("\"") + meta + "\"");
    }
    CHECK(quoteBatchArgument("C:\\R&D\\") == "\"C:\\R&D\\\\\"");  // trailing backslashes doubled before the closing quote
}

TEST_CASE("PATH lookup: finds executables, honours PATHEXT on Windows, ignores directories and non-executables") {
    ScratchDir dir;
    const fs::path bin1 = dir.path() / "bin1", bin2 = dir.path() / "bin2";
    fs::create_directories(bin1);
    fs::create_directories(bin2);
    dir.write("bin2/tool.exe", "x");
    if (!kWindows) dir.write("bin2/tool.EXE", "x");   // PATHEXT candidates are upper case; case-sensitive file systems need that spelling
    dir.write("bin1/script.cmd", "x");
    fs::create_directories(bin1 / "folder.exe");
    const std::string path = bin1.string() + ";" + bin2.string();

    const auto exe = findOnPath("tool", path, true);
    REQUIRE(exe.has_value());
    CHECK(lowerCase(exe->filename().string()) == "tool.exe");  // PATHEXT candidates keep their own case
    if (kWindows) CHECK(lowerCase(findOnPath("script", path, true)->extension().string()) == ".cmd");  // PATHEXT matching needs a case-insensitive file system
    CHECK(findOnPath("tool.exe", path, true).has_value());
    if (kWindows) CHECK_FALSE(findOnPath("folder", path, true).has_value());
    CHECK_FALSE(findOnPath("missing", path, true).has_value());
    CHECK_FALSE(findOnPath("", path, true).has_value());
    CHECK(findOnPath((bin2 / "tool").string(), "", true).has_value());       // path with separators: no PATH search
    CHECK(findOnPath("tool", "\"" + bin2.string() + "\"", true).has_value());   // quoted PATH entry

    // POSIX semantics: needs an exec bit (set explicitly; on Windows the permission bits are synthetic, so only check the positive case)
    const fs::path script = dir.write("bin1/runme", "#!/bin/sh\n");
    fs::permissions(script, fs::perms::owner_exec | fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::add);
    if (!kWindows) CHECK(findOnPath("runme", bin1.string() + ":" + bin2.string(), false).has_value());
}

TEST_CASE("init script: ADR 0001 content, no machine specific paths") {
    const std::string s = generateInitScript();
    CHECK(s == generateInitScript());  // deterministic: identical for every run
    CHECK(s.find("WPIEMU_EXT") != std::string::npos);
    CHECK(s.find("HALSIM_EXTENSIONS") != std::string::npos);
    CHECK(s.find("defaultEnabled.set(false)") != std::string::npos);
    CHECK(s.find("JavaSimulationTask") != std::string::npos);
    CHECK(s.find("NativeSimulationTask") != std::string::npos);
    CHECK(s.find("C:\\") == std::string::npos);
    CHECK(s.find("/home/") == std::string::npos);
}

// ---- launch planning ------------------------------------------------------------------------------------------------------

TEST_CASE("launch env: contract variables and overrides") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir);
    r.javaHome = "/opt/jdk";
    r.extraEnv = {{"CUSTOM", "1"}, {"WPIEMU_CLOCK", "override"}};
    CHECK(halExtensionsValue(r, true) == r.extensionLibrary.string());
    const auto env = emulatorEnvironment(r, false);
    CHECK(env.at(env_var::kExtension) == r.extensionLibrary.string());
    CHECK(env.at(env_var::kEndpoint) == r.ipcEndpoint);
    CHECK(env.at(env_var::kEndpointAlias) == r.ipcEndpoint);
    CHECK(env.at("JAVA_HOME") == fs::path("/opt/jdk").string());
    CHECK(env.at("CUSTOM") == "1");
    CHECK(env.at(env_var::kClock) == "override");  // extraEnv wins
    CHECK(env.at(env_var::kProgramName) == "robot");
    r.extraEnv.clear();
    r.realtimeClock = true;
    r.javaHome.clear();
    const auto plain = emulatorEnvironment(r, true);
    CHECK(plain.at(env_var::kClock) == "realtime");
    CHECK(plain.count("JAVA_HOME") == 0);
}

TEST_CASE("gradle launch plan: wrapper, flags, init script written to the scratch dir") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir);
    r.gradleOffline = true;
    r.extraGradleArgs = {"-Pfoo=bar"};
    for (const bool windows : {true, false}) {
        const auto plan = planGradleLaunch(r, windows);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        const ProcessSpec& s = plan.value();
        CHECK(fs::path(s.program).filename() == (windows ? "gradlew.bat" : "gradlew"));
        CHECK(s.workingDir == r.projectDir);
        const std::vector<std::string> expected = {"--no-daemon", "--console=plain", "--offline", "-I", (r.scratchDir / "wpiemu-init.gradle").string(), "-Pfoo=bar", "simulateJava"};
        CHECK(s.args == expected);
        CHECK(s.env.at(env_var::kExtension) == r.extensionLibrary.string());
        CHECK(s.label == "gradlew simulateJava");
    }
    CHECK(ScratchDir::read(r.scratchDir / "wpiemu-init.gradle") == generateInitScript());

    r.language = "cpp";
    CHECK(planGradleLaunch(r, false).value().args.back() == "simulateNative");
}

TEST_CASE("gradle launch plan: failure modes") {
    ScratchDir dir;
    LaunchRequest ok = makeRequest(dir);
    {
        LaunchRequest r = ok;
        r.projectDir = dir.path() / "nowhere";
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::NotFound);
    }
    {
        LaunchRequest r = ok;
        r.extensionLibrary = dir.path() / "missing.dll";
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::NotFound);
    }
    {
        LaunchRequest r = ok;
        r.extensionLibrary.clear();
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        r.ipcEndpoint.clear();
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        r.language = "kotlin";
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        r.scratchDir.clear();
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        r.projectDir.clear();
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        dir.write("blocker", "x");
        r.scratchDir = dir.path() / "blocker";  // a file where the directory should be
        CHECK_FALSE(planGradleLaunch(r, false).ok());
    }
}

TEST_CASE("gradle launch plan: paths gradlew.bat cannot handle launch the wrapper jar directly on Windows") {
    ScratchDir dir;
    for (const char* folder : {"R&D", "Team 1&2", "a^b", "Plain Dir (x86)"}) {
        CAPTURE(folder);
        LaunchRequest r = makeRequest(dir);
        r.projectDir = dir.path() / folder / "robot";
        fs::create_directories(r.projectDir);
        dir.write(std::string(folder) + "/robot/gradlew.bat", "@echo off\r\n");
        dir.write(std::string(folder) + "/robot/gradlew", "#!/bin/sh\n");
        r.gradleOffline = false;
        const fs::path jar = r.projectDir / "gradle" / "wrapper" / "gradle-wrapper.jar";
        dir.write(std::string(folder) + "/robot/gradle/wrapper/gradle-wrapper.jar", "x");
        r.javaHome = dir.path() / "jdk";
        const bool special = std::string_view(folder).find_first_of("&^") != std::string_view::npos;

        const auto plan = planGradleLaunch(r, true);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        const ProcessSpec& s = plan.value();
        if (special) {
            CHECK(s.program == (r.javaHome / "bin" / "java.exe").string());
            const std::vector<std::string> head = {"-Xmx64m", "-Xms64m", "-Dorg.gradle.appname=gradlew", "-classpath", jar.string(),
                                                   "org.gradle.wrapper.GradleWrapperMain", "--no-daemon", "--console=plain", "-I"};
            REQUIRE(s.args.size() > head.size());
            CHECK(std::vector<std::string>(s.args.begin(), s.args.begin() + static_cast<std::ptrdiff_t>(head.size())) == head);
            CHECK(s.args.back() == "simulateJava");
        } else {
            CHECK(fs::path(s.program).filename() == "gradlew.bat");
            CHECK(s.args.front() == "--no-daemon");
        }
        CHECK(s.workingDir == r.projectDir);

        const auto describe = planExternalDescribe(r, true);
        REQUIRE(describe.ok());
        CHECK((fs::path(describe.value().program).filename() == "gradlew.bat") == !special);
        CHECK(describe.value().args.back() == "simulateExternalJavaRelease");

        // non-Windows plans are never rewritten, and a missing wrapper jar is reported instead of falling back to a broken script
        CHECK(fs::path(planGradleLaunch(r, false).value().program).filename() == "gradlew");
        if (special) {
            fs::remove(jar);
            const auto missing = planGradleLaunch(r, true);
            REQUIRE_FALSE(missing.ok());
            CHECK(missing.error().code == ErrorCode::NotFound);
        }
    }
}

TEST_CASE("external launch plan: describe task names and descriptor paths") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir, LaunchMode::ExternalDirect);
    CHECK(externalTaskName(r) == "simulateExternalJavaRelease");
    CHECK(externalDescriptorPath(r) == r.projectDir / "build" / "sim" / "release_java.json");
    r.release = false;
    r.language = "cpp";
    CHECK(externalTaskName(r) == "simulateExternalNativeDebug");
    CHECK(externalDescriptorPath(r) == r.projectDir / "build" / "sim" / "debug_native.json");
    r.javaHome = "/jdk";
    const auto plan = planExternalDescribe(r, false);
    REQUIRE(plan.ok());
    CHECK(plan.value().args.back() == "simulateExternalNativeDebug");
    CHECK(plan.value().args.front() == "--no-daemon");
    CHECK(plan.value().env.at("JAVA_HOME") == "/jdk");
    CHECK(plan.value().env.count(env_var::kEndpoint) == 0);  // the build step needs no emulator variables
    r.projectDir = dir.path() / "none";
    CHECK(planExternalDescribe(r, false).error().code == ErrorCode::NotFound);
}

TEST_CASE("external launch plan: Java descriptor -> JVM command line") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir, LaunchMode::ExternalDirect);
    dir.write("robot/build/libs/robot-sources.jar", "tiny");
    dir.write("robot/build/libs/robot-plain.jar", "tiny");
    dir.write("robot/build/libs/small.jar", "12345");
    const fs::path fat = dir.write("robot/build/libs/robot.jar", std::string(5000, 'x'));
    const std::string json = R"json({"libraryDir": "C:\\jni\\release", "mainClassName": "frc.robot.Main",
        "environment": {"HALSIM_EXTENSIONS": "C:\\sim\\halsim_gui.dll", "KEEP": "me", "NUM": 5}, "extensions": []})json";
    const JsonValueCodec codec;
    const auto envNone = [](std::string_view) -> std::optional<std::string> { return std::nullopt; };

    {
        const auto plan = planExternalLaunch(r, json, codec, envNone, true, false);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        const ProcessSpec& s = plan.value();
        CHECK(s.program == "java");  // no JAVA_HOME anywhere: rely on PATH
        REQUIRE(s.args.size() == 3);
        CHECK(s.args[0] == "-Djava.library.path=C:\\jni\\release");
        CHECK(s.args[1] == "-jar");
        std::error_code same;
        CHECK(fs::equivalent(fs::path(s.args[2]), fat, same));  // the biggest non-sources jar
        CHECK(s.workingDir == r.projectDir);
        CHECK(s.env.at("HALSIM_EXTENSIONS") == r.extensionLibrary.string());  // ours replaces the stock Sim GUI
        CHECK(s.env.at("KEEP") == "me");
        CHECK(s.env.count("NUM") == 0);  // non-string values are ignored
        CHECK(s.env.at(env_var::kEndpoint) == r.ipcEndpoint);
        CHECK(s.env.at("PATH").rfind("C:\\jni\\release", 0) == 0);  // library dir first
    }
    {  // JAVA_HOME from the host, macOS flag, DYLD path, host value appended
        const auto host = [](std::string_view name) -> std::optional<std::string> {
            if (name == "JAVA_HOME") return std::string("/jdk17");
            if (name == "DYLD_LIBRARY_PATH") return std::string("/existing");
            return std::nullopt;
        };
        const auto plan = planExternalLaunch(r, R"json({"libraryDir": "/jni"})json", codec, host, false, true);
        REQUIRE(plan.ok());
        const ProcessSpec& s = plan.value();
        CHECK(s.program == (fs::path("/jdk17") / "bin" / "java").string());
        CHECK(s.args.front() == "-XstartOnFirstThread");
        CHECK(s.env.at("DYLD_LIBRARY_PATH") == "/jni:/existing");
    }
    {  // the request's JDK beats the environment; Linux uses LD_LIBRARY_PATH
        r.javaHome = "/wpilib/jdk";
        const auto plan = planExternalLaunch(r, R"json({"libraryDir": "/jni"})json", codec, envNone, false, false);
        REQUIRE(plan.ok());
        CHECK(plan.value().program == (fs::path("/wpilib/jdk") / "bin" / "java").string());
        CHECK(plan.value().env.at("LD_LIBRARY_PATH") == "/jni");
        CHECK(plan.value().env.at("JAVA_HOME") == "/wpilib/jdk");
        CHECK(planExternalLaunch(r, R"json({"libraryDir": "C:\\x"})json", codec, envNone, true, false).value().program.find("java.exe") != std::string::npos);
    }
    {  // GradleRIO writes a LIST with one entry per simulation target (real FRC project); the matching entry is used
        const std::string list = R"json([{"type": "native", "launchfile": "x.exe"},
            {"type": "java", "name": "jar (in project p)", "libraryDir": "/jni", "mainClassName": "frc.robot.Main", "extensions": []}])json";
        const auto plan = planExternalLaunch(r, list, codec, envNone, false, false);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        CHECK(plan.value().args.front() == "-Djava.library.path=/jni");
        CHECK(planExternalLaunch(r, "[]", codec, envNone, false, false).error().code == ErrorCode::ParseError);
    }
    {  // failure modes
        CHECK(planExternalLaunch(r, "{not json", codec, envNone, true, false).error().code == ErrorCode::ParseError);
        CHECK(planExternalLaunch(r, "[1]", codec, envNone, true, false).error().code == ErrorCode::ParseError);
        CHECK(planExternalLaunch(r, "{}", codec, envNone, true, false).error().code == ErrorCode::InvalidArgument);  // no libraryDir
        LaunchRequest noJar = r;
        noJar.projectDir = dir.path() / "empty";
        fs::create_directories(noJar.projectDir / "build" / "libs");
        CHECK(planExternalLaunch(noJar, json, codec, envNone, true, false).error().code == ErrorCode::NotFound);
    }
}

TEST_CASE("external launch plan: native descriptor -> the exe") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir, LaunchMode::ExternalDirect, "cpp");
    const JsonValueCodec codec;
    const auto host = [](std::string_view name) -> std::optional<std::string> { return name == "PATH" ? std::optional<std::string>("C:\\Windows") : std::nullopt; };
    const std::string json = R"json({"launchfile": "build/install/frcUserProgram/windowsx86-64/release/lib/frcUserProgram.exe",
        "libpaths": ["C:\\libs\\a", "C:\\libs\\b"], "environment": {"X": "1"}})json";
    const auto plan = planExternalLaunch(r, json, codec, host, true, false);
    REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
    const ProcessSpec& s = plan.value();
    CHECK(s.program == (r.projectDir / "build/install/frcUserProgram/windowsx86-64/release/lib/frcUserProgram.exe").string());  // relative to the project
    CHECK(s.args.empty());
    CHECK(s.workingDir == r.projectDir);
    CHECK(s.env.at("PATH") == "C:\\libs\\a;C:\\libs\\b;C:\\Windows");
    CHECK(s.env.at("X") == "1");
    CHECK(s.env.at(env_var::kHalExtensions) == r.extensionLibrary.string());

    const fs::path elsewhere = dir.path() / "elsewhere" / "robot";
    std::string escaped;
    for (const char c : elsewhere.string()) escaped += c == '\\' ? std::string("\\\\") : std::string(1, c);
    const auto absolute = planExternalLaunch(r, "{\"launchfile\": \"" + escaped + "\"}", codec, host, false, false);
    REQUIRE(absolute.ok());
    CHECK(absolute.value().program == elsewhere.string());  // already absolute: not re-based on the project
    CHECK(planExternalLaunch(r, "{}", codec, host, true, false).error().code == ErrorCode::InvalidArgument);
}

// ---- real processes ---------------------------------------------------------------------------------------------------------

TEST_CASE("process: captures stdout and stderr line by line and reports the exit code") {
    CollectingSink sink;
    auto p = createSystemProcess();
    CHECK_FALSE(p->running());
    CHECK(p->pid() == 0);
    CHECK_FALSE(p->exitInfo().has_value());
    CHECK(p->waitForExit(0ms));  // never started
    ProcessSpec spec = shell(kWindows ? "echo out line& echo err line>&2& exit 3" : "echo out line; echo err line 1>&2; exit 3");
    REQUIRE(p->start(spec, sink).ok());
    CHECK(p->pid() > 0);
    REQUIRE(p->waitForExit(10s));
    const auto info = p->exitInfo();
    REQUIRE(info.has_value());
    CHECK(info->exitCode == 3);
    CHECK_FALSE(info->killedByUs);
    CHECK_FALSE(p->running());
    CHECK(sink.has(OutputStream::Stdout, "out line"));
    CHECK(sink.has(OutputStream::Stderr, "err line"));
    CHECK_FALSE(p->start(spec, sink).ok());  // one start per object
}

TEST_CASE("process: environment, working directory and unset variables reach the child") {
    ScratchDir dir;
    CollectingSink sink;
    auto p = createSystemProcess();
    ProcessSpec spec = shell(kWindows ? "echo var=%WPIEMU_TEST_VAR%& echo gone=%WPIEMU_TEST_GONE%& cd" : "echo var=$WPIEMU_TEST_VAR; echo gone=$WPIEMU_TEST_GONE; pwd");
    spec.env = {{"WPIEMU_TEST_VAR", "hello world"}};
    spec.unsetEnv = {"WPIEMU_TEST_GONE"};
    spec.workingDir = dir.path();
    REQUIRE(p->start(spec, sink).ok());
    REQUIRE(p->waitForExit(10s));
    CHECK(p->exitInfo()->exitCode == 0);
    CHECK(sink.has(OutputStream::Stdout, "var=hello world"));
    CHECK(sink.has(OutputStream::Stdout, kWindows ? "gone=%WPIEMU_TEST_GONE%" : "gone="));
    const auto lines = sink.texts(OutputStream::Stdout);
    REQUIRE_FALSE(lines.empty());
    std::error_code ec;
    CHECK(fs::equivalent(fs::path(lines.back()), dir.path(), ec));
}

TEST_CASE("process: unknown program, bad working directory, bad batch arguments fail to start cleanly") {
    CollectingSink sink;
    {
        auto p = createSystemProcess();
        ProcessSpec spec;
        spec.program = "definitely-not-a-program-wpiemu";
        const Status s = p->start(spec, sink);
        REQUIRE_FALSE(s.ok());
        CHECK(s.error().code == ErrorCode::NotFound);
        CHECK_FALSE(p->running());
    }
    {
        auto p = createSystemProcess();
        ProcessSpec spec = shell("echo hi");
        spec.workingDir = fs::path("no") / "such" / "dir" / "wpiemu";
        CHECK_FALSE(p->start(spec, sink).ok());
        CHECK_FALSE(p->running());
    }
    if (kWindows) {
        ScratchDir dir;
        const fs::path bat = dir.write("echo args.bat", "@echo off\r\necho batch got [%~1]\r\n");
        auto p = createSystemProcess();
        ProcessSpec spec;
        spec.program = bat.string();
        spec.args = {"a\" & calc & \""};  // would end the quoting: refused rather than interpreted
        const Status s = p->start(spec, sink);
        REQUIRE_FALSE(s.ok());
        CHECK(s.error().code == ErrorCode::InvalidArgument);
    }
}

TEST_CASE("process: a batch file with cmd metacharacters in its path and arguments runs them as plain text") {
    if (!kWindows) return;
    ScratchDir dir;
    // %1 keeps its quotes, so the batch's own echo does not interpret what is inside them
    const std::string body = "@echo off\r\necho got %1 %2\r\necho dir \"%~dp0\"\r\n";
    for (const char* folder : {"R&D", "Team 1&2", "a^b", "p(x86)", "w!ow", "x,y;z=1"}) {
        CAPTURE(folder);
        CollectingSink sink;
        auto p = createSystemProcess();
        ProcessSpec spec;
        spec.program = dir.write(std::string(folder) + "/echo args.bat", body).string();
        spec.args = {"x & echo INJECTED", "b^c|d>e<f"};
        const Status started = p->start(spec, sink);
        REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
        REQUIRE(p->waitForExit(10s));
        CHECK(p->exitInfo()->exitCode == 0);
        CHECK(sink.has(OutputStream::Stdout, "got \"x & echo INJECTED\" \"b^c|d>e<f\""));
        CHECK(sink.has(OutputStream::Stdout, "dir \"" + (dir.path() / folder).string() + "\\\""));
        CHECK_FALSE(sink.has(OutputStream::Stdout, "INJECTED"));
        CHECK(sink.texts(OutputStream::Stderr).empty());
    }
}

TEST_CASE("process: delayed expansion is off for batch files, so ! stays literal") {
    if (!kWindows) return;
    ScratchDir dir;
    CollectingSink sink;
    auto p = createSystemProcess();
    ProcessSpec spec;
    spec.program = dir.write("bang.bat", "@echo off\r\necho got %1\r\n").string();
    spec.args = {"!WPIEMU_BANG!"};
    spec.env = {{"WPIEMU_BANG", "boom"}};
    REQUIRE(p->start(spec, sink).ok());
    REQUIRE(p->waitForExit(10s));
    CHECK(sink.has(OutputStream::Stdout, "got \"!WPIEMU_BANG!\""));
}

TEST_CASE("process: a batch file / script with spaces in its path runs with its arguments") {
    ScratchDir dir;
    CollectingSink sink;
    auto p = createSystemProcess();
    ProcessSpec spec;
    if (kWindows) {
        spec.program = dir.write("my dir/echo args.bat", "@echo off\r\necho batch got [%~1] [%~2]\r\n").string();
        spec.args = {"first arg", "second"};
    } else {
        const fs::path script = dir.write("my dir/echo args.sh", "#!/bin/sh\necho \"batch got [$1] [$2]\"\n");
        fs::permissions(script, fs::perms::owner_exec | fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::add);
        spec.program = script.string();
        spec.args = {"first arg", "second"};
    }
    REQUIRE(p->start(spec, sink).ok());
    REQUIRE(p->waitForExit(10s));
    CHECK(sink.has(OutputStream::Stdout, "batch got [first arg] [second]"));
}

TEST_CASE("process: kill() ends the whole tree quickly and marks the exit as ours") {
    CollectingSink sink;
    auto p = createSystemProcess();
    // a shell that waits on a grandchild that holds the stdout pipe open
    ProcessSpec spec = shell(kWindows ? "ping -n 60 127.0.0.1 > nul & ping -n 60 127.0.0.1" : "sleep 60 & sleep 60");
    REQUIRE(p->start(spec, sink).ok());
    std::this_thread::sleep_for(300ms);
    CHECK(p->running());
    CHECK_FALSE(p->waitForExit(50ms));
    const auto t0 = std::chrono::steady_clock::now();
    p->kill();
    REQUIRE(p->waitForExit(5s));
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    CHECK(elapsed < 1200ms);  // the pipe readers ended because the grandchildren died too (otherwise the monitor waits 1.5 s)
    CHECK(p->exitInfo()->killedByUs);
    CHECK_FALSE(p->running());
    p->kill();  // idempotent
}

TEST_CASE("process: the destructor kills the tree (no orphans)") {
    CollectingSink sink;
    std::int64_t pid = 0;
    {
        auto p = createSystemProcess();
        REQUIRE(p->start(shell(kWindows ? "ping -n 60 127.0.0.1" : "sleep 60"), sink).ok());
        pid = p->pid();
        std::this_thread::sleep_for(200ms);
        CHECK(p->running());
    }
    std::this_thread::sleep_for(200ms);
    CHECK(killProcessTree(pid).error().code == ErrorCode::NotFound);  // already gone
}

TEST_CASE("process: killProcessTree by pid, and it refuses to kill the emulator") {
    CollectingSink sink;
    auto p = createSystemProcess();
    REQUIRE(p->start(shell(kWindows ? "ping -n 60 127.0.0.1" : "sleep 60"), sink).ok());
    std::this_thread::sleep_for(200ms);
    REQUIRE(killProcessTree(p->pid()).ok());
    REQUIRE(p->waitForExit(5s));
    CHECK(killProcessTree(0).error().code == ErrorCode::InvalidArgument);
    CHECK(killProcessTree(-5).error().code == ErrorCode::InvalidArgument);
#ifdef _WIN32
    CHECK(killProcessTree(static_cast<std::int64_t>(GetCurrentProcessId())).error().code == ErrorCode::InvalidArgument);
#else
    CHECK(killProcessTree(static_cast<std::int64_t>(getpid())).error().code == ErrorCode::InvalidArgument);
#endif
}

TEST_CASE("process: a lot of output is captured completely and in order") {
    CollectingSink sink;
    auto p = createSystemProcess();
    ProcessSpec spec = shell(kWindows ? "for /L %i in (1,1,2000) do @echo line %i" : "i=1; while [ $i -le 2000 ]; do echo line $i; i=$((i+1)); done");
    REQUIRE(p->start(spec, sink).ok());
    REQUIRE(p->waitForExit(60s));
    const auto lines = sink.texts(OutputStream::Stdout);
    REQUIRE(lines.size() == 2000);
    CHECK(lines.front() == "line 1");
    CHECK(lines.back() == "line 2000");
}

#ifndef _WIN32
TEST_CASE("process: requestStop sends SIGTERM to the group") {
    CollectingSink sink;
    auto p = createSystemProcess();
    REQUIRE(p->start(shell("trap 'echo got term; exit 7' TERM; echo ready; while :; do sleep 0.1; done"), sink).ok());
    for (int i = 0; i < 100 && !sink.has(OutputStream::Stdout, "ready"); ++i) std::this_thread::sleep_for(20ms);
    p->requestStop();
    REQUIRE(p->waitForExit(5s));
    CHECK(p->exitInfo()->exitCode == 7);
    CHECK_FALSE(p->exitInfo()->killedByUs);
}
#endif

// ---- the launcher with scripted fake processes -------------------------------------------------------------------------------

namespace {

struct FakeScript {
    int exitCode = 0;
    bool blockUntilKilled = false;
    bool failToStart = false;
    std::function<void(const ProcessSpec&)> onStart;
    std::vector<std::string> lines;
};

struct FakeWorld {
    std::mutex m;
    std::vector<ProcessSpec> started;
    std::deque<FakeScript> scripts;
    std::atomic<int> killed{0};
    std::atomic<int> stopRequested{0};
    std::atomic<bool> rescue{false};
    std::function<void()> onWait;
};

class FakeProcess final : public IRobotProcess {
public:
    FakeProcess(FakeWorld& w, FakeScript s) : world_(w), script_(std::move(s)) {}
    ~FakeProcess() override { kill(); }
    Status start(const ProcessSpec& spec, ILogSink& sink) override {
        if (script_.failToStart) return makeError(ErrorCode::NotFound, "program not found: " + spec.program);
        {
            std::lock_guard lock(world_.m);
            world_.started.push_back(spec);
        }
        if (script_.onStart) script_.onStart(spec);
        for (const std::string& l : script_.lines) sink.line(OutputStream::Stdout, l);
        std::lock_guard lock(m_);
        started_ = true;
        if (!script_.blockUntilKilled) finish(script_.exitCode, false);
        return okStatus();
    }
    bool running() const override {
        std::lock_guard lock(m_);
        return started_ && !exited_;
    }
    std::int64_t pid() const override { return 1234; }
    std::optional<ExitInfo> exitInfo() const override {
        std::lock_guard lock(m_);
        return exited_ ? std::optional<ExitInfo>(exit_) : std::nullopt;
    }
    bool waitForExit(std::chrono::milliseconds t) override {
        if (world_.onWait) world_.onWait();
        std::unique_lock lock(m_);
        const bool done = cv_.wait_for(lock, t, [this] { return exited_ || !started_ || world_.rescue; });
        if (world_.rescue && !exited_) finish(-1, false);
        return done;
    }
    void requestStop() override { ++world_.stopRequested; }
    void kill() override {
        std::lock_guard lock(m_);
        if (started_ && !exited_) {
            ++world_.killed;
            finish(137, true);
        }
    }

private:
    void finish(int code, bool killed) {
        exit_.exitCode = code;
        exit_.killedByUs = killed;
        exited_ = true;
        cv_.notify_all();
    }
    FakeWorld& world_;
    FakeScript script_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    bool started_ = false, exited_ = false;
    ExitInfo exit_;
};

ProcessFactory fakeFactory(FakeWorld& w) {
    return [&w]() -> std::unique_ptr<IRobotProcess> {
        FakeScript s;
        {
            std::lock_guard lock(w.m);
            if (!w.scripts.empty()) {
                s = std::move(w.scripts.front());
                w.scripts.pop_front();
            }
        }
        return std::make_unique<FakeProcess>(w, std::move(s));
    };
}

bool waitFor(const std::function<bool()>& pred, std::chrono::milliseconds timeout = 5s) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(5ms);
    }
    return pred();
}

struct StartupGate {
    void hold() {
        std::unique_lock lock(mutex);
        entered = true;
        cv.notify_all();
        if (!cv.wait_for(lock, 5s, [this] { return released; })) timedOut = true;
    }
    bool waitUntilEntered() {
        std::unique_lock lock(mutex);
        return cv.wait_for(lock, 2s, [this] { return entered; });
    }
    void release() {
        std::lock_guard lock(mutex);
        released = true;
        cv.notify_all();
    }
    std::mutex mutex;
    std::condition_variable cv;
    bool entered = false;
    bool released = false;
    bool timedOut = false;
};

}  // namespace

TEST_CASE("launcher: cancellation during startup is handed to the published process") {
    ScratchDir dir;
    FakeWorld world;
    StartupGate gate;
    world.scripts.push_back({0, true, false, [&](const ProcessSpec&) { gate.hold(); }, {}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    CollectingSink sink;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
    const bool entered = gate.waitUntilEntered();
    launcher.requestStop(0ms);
    launcher.requestStop(10s);  // cannot extend the first cancellation deadline
    gate.release();
    const bool completed = launcher.waitUntilFinished(2s);
    // The fake never exits on its own. Rescue only after recording failure, so a regression cannot hang the suite.
    if (!completed) world.rescue = true;
    launcher.stop(0ms);
    CHECK(entered);
    CHECK_FALSE(gate.timedOut);
    CHECK(completed);
    CHECK(world.killed == 1);
    CHECK(world.stopRequested == 1);
    CHECK(launcher.status().phase == LaunchPhase::Stopped);
}

TEST_CASE("launcher: cancellation is observed again after a bounded process wait") {
    ScratchDir dir;
    FakeWorld world;
    StartupGate gate;
    world.scripts.push_back({0, true, false, nullptr, {}});
    world.onWait = [&] { gate.hold(); };
    JsonValueCodec json;
    testing::CapturingLogger logger;
    CollectingSink sink;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
    const bool entered = gate.waitUntilEntered();
    launcher.requestStop(0ms);
    gate.release();
    const bool completed = launcher.waitUntilFinished(2s);
    if (!completed) world.rescue = true;
    launcher.stop(0ms);
    CHECK(entered);
    CHECK_FALSE(gate.timedOut);
    CHECK(completed);
    CHECK(world.killed == 1);
    CHECK(world.stopRequested == 1);
}

TEST_CASE("process: cancellation before native startup prevents creating a child") {
    CollectingSink sink;
    auto process = createSystemProcess();
    process->kill();
    const Status started = process->start(shell("echo should not run"), sink);
    CHECK_FALSE(started.ok());
    if (!started) CHECK(started.error().code == ErrorCode::Cancelled);
    CHECK(process->pid() == 0);
    CHECK_FALSE(process->running());
    CHECK(process->waitForExit(0ms));
    CHECK(sink.texts(OutputStream::Stdout).empty());
}

TEST_CASE("launcher: gradle mode runs one process, phases Running -> Exited, output is forwarded") {
    ScratchDir dir;
    FakeWorld world;
    world.scripts.push_back({0, false, false, nullptr, {"> Task :simulateJava", "HAL Extensions: Successfully loaded extension"}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    std::vector<LaunchPhase> phases;
    std::mutex pm;
    launcher.setOnChange([&](const LaunchStatus& s) {
        std::lock_guard lock(pm);
        if (phases.empty() || phases.back() != s.phase) phases.push_back(s.phase);
    });
    CollectingSink sink;
    const LaunchRequest request = makeRequest(dir);
    REQUIRE(launcher.launch(request, sink).ok());
    REQUIRE(launcher.waitUntilFinished(5s));
    const LaunchStatus st = launcher.status();
    CHECK(st.phase == LaunchPhase::Exited);
    REQUIRE(st.exit.has_value());
    CHECK(st.exit->exitCode == 0);
    {
        std::lock_guard lock(pm);
        CHECK(phases == std::vector<LaunchPhase>{LaunchPhase::Running, LaunchPhase::Exited});
    }
    REQUIRE(world.started.size() == 1);
    CHECK(world.started[0].args.back() == "simulateJava");
    CHECK(world.started[0].env.at(env_var::kEndpoint) == request.ipcEndpoint);

    // a finished launcher can launch again
    world.scripts.push_back({5, false, false, nullptr, {}});
    REQUIRE(launcher.launch(request, sink).ok());
    REQUIRE(launcher.waitUntilFinished(5s));
    CHECK(launcher.status().phase == LaunchPhase::Failed);
    CHECK(launcher.status().detail.find("code 5") != std::string::npos);
    CHECK(launcher.status().exit->exitCode == 5);
}

TEST_CASE("launcher: plan errors are returned immediately, a second launch while active is refused") {
    ScratchDir dir;
    FakeWorld world;
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    CollectingSink sink;
    LaunchRequest bad = makeRequest(dir);
    bad.projectDir = dir.path() / "none";
    CHECK(launcher.launch(bad, sink).error().code == ErrorCode::NotFound);
    CHECK(launcher.status().phase == LaunchPhase::Idle);
    CHECK(world.started.empty());

    world.scripts.push_back({0, true, false, nullptr, {}});
    const LaunchRequest good = makeRequest(dir);
    REQUIRE(launcher.launch(good, sink).ok());
    REQUIRE(waitFor([&] { return launcher.status().phase == LaunchPhase::Running; }));
    CHECK(launcher.launch(good, sink).error().code == ErrorCode::InvalidState);
    CHECK_FALSE(launcher.waitUntilFinished(20ms));
    launcher.stop(0ms);
    CHECK(launcher.waitUntilFinished(2s));
}

TEST_CASE("launcher: a program that cannot start is a Failed launch with the reason") {
    ScratchDir dir;
    FakeWorld world;
    world.scripts.push_back({0, false, true, nullptr, {}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    CollectingSink sink;
    REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
    REQUIRE(launcher.waitUntilFinished(5s));
    CHECK(launcher.status().phase == LaunchPhase::Failed);
    CHECK(launcher.status().detail.find("program not found") != std::string::npos);
}

TEST_CASE("launcher: stop() asks politely, then kills the tree after the grace period") {
    ScratchDir dir;
    FakeWorld world;
    world.scripts.push_back({0, true, false, nullptr, {}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    CollectingSink sink;
    REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
    REQUIRE(waitFor([&] { return launcher.status().phase == LaunchPhase::Running; }));
    CHECK(launcher.status().pid == 1234);
    launcher.stop(100ms);
    CHECK(world.stopRequested >= 1);
    CHECK(world.killed == 1);
    const LaunchStatus st = launcher.status();
    CHECK(st.phase == LaunchPhase::Stopped);
    CHECK(st.exit->killedByUs);
    launcher.stop(0ms);  // idempotent
}

TEST_CASE("launcher: the destructor kills a running robot") {
    ScratchDir dir;
    FakeWorld world;
    world.scripts.push_back({0, true, false, nullptr, {}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    CollectingSink sink;
    {
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(makeRequest(dir), sink).ok());
        REQUIRE(waitFor([&] { return launcher.status().phase == LaunchPhase::Running; }));
    }
    CHECK(world.killed == 1);
}

TEST_CASE("launcher: ExternalDirect builds first, then runs the JVM from the descriptor") {
    ScratchDir dir;
    FakeWorld world;
    const LaunchRequest request = makeRequest(dir, LaunchMode::ExternalDirect);
    dir.write("robot/build/libs/robot.jar", std::string(100, 'x'));
    // step 1 (describe) "produces" the descriptor; step 2 is the robot
    world.scripts.push_back({0, false, false,
                             [&](const ProcessSpec&) { dir.write("robot/build/sim/release_java.json", R"json({"libraryDir": "/jni", "mainClassName": "frc.robot.Main"})json"); },
                             {"BUILD SUCCESSFUL"}});
    world.scripts.push_back({0, false, false, nullptr, {"robot output"}});
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(fakeFactory(world), json, logger);
    std::vector<LaunchPhase> phases;
    std::mutex pm;
    launcher.setOnChange([&](const LaunchStatus& s) {
        std::lock_guard lock(pm);
        if (phases.empty() || phases.back() != s.phase) phases.push_back(s.phase);
    });
    CollectingSink sink;
    REQUIRE(launcher.launch(request, sink).ok());
    REQUIRE(launcher.waitUntilFinished(5s));
    CHECK(launcher.status().phase == LaunchPhase::Exited);
    {
        std::lock_guard lock(pm);
        CHECK(phases == std::vector<LaunchPhase>{LaunchPhase::Building, LaunchPhase::Running, LaunchPhase::Exited});
    }
    REQUIRE(world.started.size() == 2);
    CHECK(world.started[0].args.back() == "simulateExternalJavaRelease");
    const auto& jvmArgs = world.started[1].args;   // macOS adds -XstartOnFirstThread before -jar
    CHECK(std::find(jvmArgs.begin(), jvmArgs.end(), "-jar") != jvmArgs.end());
    CHECK(world.started[1].env.at(env_var::kHalExtensions) == request.extensionLibrary.string());
}

TEST_CASE("launcher: ExternalDirect failure modes - describe fails, descriptor missing, descriptor unusable") {
    ScratchDir dir;
    JsonValueCodec json;
    testing::CapturingLogger logger;
    CollectingSink sink;
    const LaunchRequest request = makeRequest(dir, LaunchMode::ExternalDirect);
    {
        FakeWorld world;
        world.scripts.push_back({1, false, false, nullptr, {}});
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(request, sink).ok());
        REQUIRE(launcher.waitUntilFinished(5s));
        CHECK(launcher.status().phase == LaunchPhase::Failed);
        CHECK(world.started.size() == 1);  // no second step
    }
    {
        FakeWorld world;
        world.scripts.push_back({0, false, false, nullptr, {}});
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(request, sink).ok());
        REQUIRE(launcher.waitUntilFinished(5s));
        CHECK(launcher.status().phase == LaunchPhase::Failed);
        CHECK(launcher.status().detail.find("did not produce") != std::string::npos);
    }
    {
        FakeWorld world;
        world.scripts.push_back({0, false, false, [&](const ProcessSpec&) { dir.write("robot/build/sim/release_java.json", "{}"); }, {}});
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(request, sink).ok());
        REQUIRE(launcher.waitUntilFinished(5s));
        CHECK(launcher.status().phase == LaunchPhase::Failed);
        CHECK(launcher.status().detail.find("cannot plan") != std::string::npos);
    }
    {  // stopped during the build step: the robot is never launched
        FakeWorld world;
        world.scripts.push_back({0, true, false, nullptr, {}});
        RobotLauncher launcher(fakeFactory(world), json, logger);
        REQUIRE(launcher.launch(request, sink).ok());
        REQUIRE(waitFor([&] { return launcher.status().phase == LaunchPhase::Building; }));
        launcher.stop(0ms);
        CHECK(launcher.status().phase == LaunchPhase::Stopped);
        CHECK(world.started.size() == 1);
    }
}

TEST_CASE("launcher: end to end with a real fake gradlew - env, init script path and output") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir);
    if (kWindows) {
        dir.write("robot/gradlew.bat",
                  "@echo off\r\necho gradlew args: %*\r\necho ext=%WPIEMU_EXT%\r\necho endpoint=%WPIEMU_IPC_ENDPOINT%\r\necho clock=%WPIEMU_CLOCK%\r\ncd\r\nexit /b 0\r\n");
    } else {
        const fs::path g = dir.write("robot/gradlew", "#!/bin/sh\necho \"gradlew args: $*\"\necho \"ext=$WPIEMU_EXT\"\necho \"endpoint=$WPIEMU_IPC_ENDPOINT\"\necho \"clock=$WPIEMU_CLOCK\"\npwd\n");
        fs::permissions(g, fs::perms::owner_exec | fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::add);
    }
    r.ipcEndpoint = "wpiemu-e2e";
    JsonValueCodec json;
    testing::CapturingLogger logger;
    RobotLauncher launcher(systemProcessFactory(), json, logger);
    CollectingSink sink;
    REQUIRE(launcher.launch(r, sink).ok());
    REQUIRE(launcher.waitUntilFinished(30s));
    const LaunchStatus st = launcher.status();
    CHECK_MESSAGE(st.phase == LaunchPhase::Exited, st.detail);

    const std::string initPath = (r.scratchDir / "wpiemu-init.gradle").string();
    const auto lines = sink.texts(OutputStream::Stdout);
    bool args = false;
    for (const std::string& l : lines)
        if (l.rfind("gradlew args:", 0) == 0) {
            args = true;
            CHECK(l.find("--no-daemon") != std::string::npos);
            CHECK(l.find("--console=plain") != std::string::npos);
            CHECK(l.find("-I") != std::string::npos);
            CHECK(l.find(initPath) != std::string::npos);
            CHECK(l.find("simulateJava") != std::string::npos);
        }
    CHECK(args);
    CHECK(sink.has(OutputStream::Stdout, "ext=" + r.extensionLibrary.string()));
    CHECK(sink.has(OutputStream::Stdout, "endpoint=wpiemu-e2e"));
    CHECK(sink.has(OutputStream::Stdout, "clock=master"));
    REQUIRE_FALSE(lines.empty());
    std::error_code ec;
    CHECK(fs::equivalent(fs::path(lines.back()), r.projectDir, ec));  // ran in the project directory
}
