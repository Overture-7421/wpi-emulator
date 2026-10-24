#pragma once

// Shared by the launcher suites (test_process.cpp, test_launch_plan.cpp, test_launcher.cpp): the host platform, a log sink that
// collects what a process prints, a command the platform shell can run, and a LaunchRequest that plans cleanly.

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "runtime_fixtures.h"
#include "wpiemu/core/launcher/process.h"
#include "wpiemu/core/launcher/robot_launcher.h"

namespace wpiemu::test_runtime {

using namespace wpiemu::sdk;
using namespace wpiemu::core::launcher;
namespace fs = std::filesystem;

#ifdef _WIN32
inline constexpr bool kWindows = true;
#else
inline constexpr bool kWindows = false;
#endif

inline std::string lowerCase(std::string s) {
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
inline ProcessSpec shell(const std::string& script) {
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

inline LaunchRequest makeRequest(const ScratchDir& dir, LaunchMode mode = LaunchMode::GradleInitScript, const std::string& language = "java") {
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
    r.wpilibYear = 2026;
    return r;
}

}  // namespace wpiemu::test_runtime
