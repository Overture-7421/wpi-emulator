#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <thread>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "launcher_test_support.h"

// The process layer: line splitting, environment and command-line building, PATH lookup, and real child processes.

using namespace wpiemu::sdk;
using namespace wpiemu::core::launcher;
using namespace std::chrono_literals;
using wpiemu::test_runtime::CollectingSink;
using wpiemu::test_runtime::ScratchDir;
using wpiemu::test_runtime::kWindows;
using wpiemu::test_runtime::lowerCase;
using wpiemu::test_runtime::shell;
namespace fs = std::filesystem;

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

TEST_CASE("environment lookup matches names the way the merge does, with the same odd entries") {
    const std::vector<std::string> env = {"Path=C:\\win", "PATH=/posix", "EMPTY=", "=C:=C:\\dir", "garbage", "WITH=EQ=UALS"};
    CHECK(lookupEnvironment(env, "Path", true) == std::optional<std::string>("C:\\win"));      // Windows: the first spelling wins
    CHECK(lookupEnvironment(env, "PATH", true) == std::optional<std::string>("C:\\win"));
    CHECK(lookupEnvironment(env, "path", true) == std::optional<std::string>("C:\\win"));
    CHECK(lookupEnvironment(env, "PATH", false) == std::optional<std::string>("/posix"));      // POSIX: exact case only
    CHECK_FALSE(lookupEnvironment(env, "path", false).has_value());
    CHECK(lookupEnvironment(env, "EMPTY", false) == std::optional<std::string>(""));
    CHECK(lookupEnvironment(env, "WITH", false) == std::optional<std::string>("EQ=UALS"));     // only the first '=' separates
    CHECK(lookupEnvironment(env, "=C:", true) == std::optional<std::string>("C:\\dir"));       // a name that starts with '='
    CHECK_FALSE(lookupEnvironment(env, "garbage", false).has_value());
    CHECK_FALSE(lookupEnvironment(env, "PAT", false).has_value());                             // no prefix matches
    CHECK_FALSE(lookupEnvironment(env, "", false).has_value());
    CHECK_FALSE(lookupEnvironment({}, "PATH", true).has_value());
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
