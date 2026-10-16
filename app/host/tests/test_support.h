#pragma once

// Shared fixtures for the host tests and the opt-in end-to-end test: the registries a composition root builds, a
// platform-neutral environment lookup and a process-liveness probe.
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <string>

#include <doctest/doctest.h>

#include "host/robot_session.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
#include "wpiemu/core/physics/chassis_simulator.h"
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

inline std::string env(const char* name) {
#if defined(_WIN32)
    char* value = nullptr;
    size_t len = 0;
    if (_dupenv_s(&value, &len, name) != 0 || value == nullptr) return {};
    std::string out(value);
    std::free(value);
    return out;
#else
    const char* v = std::getenv(name);
    return v != nullptr ? v : "";
#endif
}

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
