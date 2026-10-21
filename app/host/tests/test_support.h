#pragma once

// Shared fixtures for the host tests and the opt-in end-to-end tests: the registries a composition root builds, a
// platform-neutral environment lookup, a process-liveness probe, scratch directories, robot-project copies and the
// robot console.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <doctest/doctest.h>

#include "host/robot_session.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/sdk/core/environment.h"
#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/game/builtin_plugins.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <csignal>
#include <sys/types.h>
#endif

namespace wpiemu::app::host::testing {

inline std::string env(const char* name) { return sdk::readEnvironment(name).value_or(std::string()); }

inline bool processAlive(std::int64_t pid) {
    if (pid <= 0) return false;
#if defined(_WIN32)
    HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
    if (h == nullptr) return false;
    DWORD code = 0;
    const bool alive = GetExitCodeProcess(h, &code) && code == STILL_ACTIVE;
    CloseHandle(h);
    return alive;
#else
    return ::kill(static_cast<pid_t>(pid), 0) == 0;
#endif
}

// A fresh path under the system temp directory (not created).
inline std::filesystem::path uniqueTempPath(const std::string& prefix) {
    static std::atomic<int> counter{0};
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
    return std::filesystem::temp_directory_path() / (prefix + std::to_string(ms % 100000000) + "-" + std::to_string(counter++));
}

// Removes a directory tree, retrying while Windows still holds files of an exiting process. True once it is gone.
inline bool removeTree(const std::filesystem::path& root, int attempts, std::chrono::milliseconds delay) {
    std::error_code ec;
    for (int i = 0; i < attempts && std::filesystem::exists(root, ec); ++i) {
        std::filesystem::remove_all(root, ec);
        if (std::filesystem::exists(root, ec)) std::this_thread::sleep_for(delay);
    }
    return !std::filesystem::exists(root, ec);
}

// Exit code of an e2e executable whose prerequisites are missing; must match SKIP_RETURN_CODE in app/host/CMakeLists.txt.
inline constexpr int kSkipExitCode = 77;

// The scratch directory of one e2e test, removed afterwards unless WPIEMU_E2E_KEEP is set.
struct Scratch {
    std::filesystem::path root;

    explicit Scratch(const std::string& prefix) : root(uniqueTempPath(prefix)) {}
    Scratch(const Scratch&) = delete;
    Scratch& operator=(const Scratch&) = delete;
    ~Scratch() {
        if (!env("WPIEMU_E2E_KEEP").empty()) {
            std::printf("kept %s\n", root.string().c_str());
        } else if (!removeTree(root, 20, std::chrono::milliseconds(500))) {
            std::printf("WARNING: could not delete %s\n", root.string().c_str());
        }
    }
};

// Copy of a robot project without VCS / build / Gradle state, so Gradle rebuilds in the copy and the original is never touched.
inline void copyProject(const std::filesystem::path& from, const std::filesystem::path& to) {
    namespace fs = std::filesystem;
    fs::create_directories(to);
    for (fs::directory_iterator it(from), end; it != end; ++it) {
        const std::string name = it->path().filename().string();
        if (name == ".git" || name == "build" || name == ".gradle" || name == "logs" || name == "bin") continue;
        fs::copy(it->path(), to / name, fs::copy_options::recursive | fs::copy_options::overwrite_existing);
    }
}

// Appends the session's new log lines to `console` ("[emu source] text" for emulator lines, "[robot] text" otherwise),
// echoing them when `verbose`. Returns the entries for tests that parse them.
inline std::vector<sdk::SessionLogEntry> drainConsole(RobotSession& session, std::vector<std::string>& console, bool verbose) {
    std::vector<sdk::SessionLogEntry> entries = session.takeLogs(5000);
    for (const sdk::SessionLogEntry& e : entries) {
        const std::string line = (e.stream == sdk::LogStream::Emulator ? "[emu " + e.source + "] " : "[robot] ") + e.text;
        console.push_back(line);
        if (verbose) std::printf("%s\n", line.c_str());
    }
    return entries;
}

// Prints the tail of a failed run's console.
inline void dumpConsole(const std::vector<std::string>& console, std::size_t lastLines = 80) {
    const std::size_t from = console.size() > lastLines ? console.size() - lastLines : 0;
    for (std::size_t i = from; i < console.size(); ++i) std::printf("    | %s\n", console[i].c_str());
}

// A Phoenix 6 device as the robot declares it; the bus is unknown because the firmware sim cannot tell CAN buses apart.
inline sdk::DeviceKey phoenixKey(const char* type, int id) { return sdk::DeviceKey{"phoenix6", type, id, sdk::kBusUnknown, -1}; }

// What AppServices builds, for tests that have no Qt.
struct TestRegistries {
    sdk::GamePluginRegistry plugins;
    sdk::MechanismFactory mechanisms;
    sdk::DrivetrainRegistry drivetrains;
    sdk::TireModelRegistry tireModels;
    core::config::PluginRegistrar registrar{mechanisms};
    core::config::JsonValueCodec json;
    core::config::RobotConfigCodec codec;
    core::config::ConfigValidator validator{&tireModels, {}};
    core::config::ConfigPipeline pipeline{codec, validator, registrar, drivetrains, plugins};

    TestRegistries() {
        REQUIRE(sdk::registerBuiltinGamePlugins(plugins).ok());
        REQUIRE(core::physics::registerPhysicsComponents(drivetrains, tireModels).ok());
        REQUIRE(core::mechanisms::registerGenericMechanisms(mechanisms).ok());
    }
    Registries view() {
        Registries r;
        r.plugins = &plugins;
        r.mechanisms = &mechanisms;
        r.drivetrains = &drivetrains;
        r.tireModels = &tireModels;
        r.registrar = &registrar;
        r.pipeline = &pipeline;
        r.json = &json;
        return r;
    }
};

}  // namespace wpiemu::app::host::testing
