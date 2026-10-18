#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/version.h"

namespace wpiemu::sdk {

// Owns the registered year plugins. An INSTANCE created by the composition root (app/tests);
// no global state. Built-ins register through a free function per plugin (see
// WPIEMU_REGISTER_GAME_PLUGIN) invoked by the plugins/ aggregation seam (builtin_plugins.h).
// A future dynamic loader (shared-library plugins) would resolve the same entry point
// `Status registerPlugin(GamePluginRegistry&)` from the library and call it identically.
//
// ABI CAVEAT (ADR 0006): v1 supports STATIC BUILT-IN plugins only (one binary, one compiler,
// one CRT). The SDK is C++ with virtual interfaces, std::string/std::vector/std::variant/std::function
// in signatures and header-only helpers; none of that is a stable ABI across separately built shared
// libraries. Shared-library plugins would therefore need a dedicated entry descriptor: an
// `extern "C"` POD struct (versioned, size-prefixed; sdk major/minor, plugin id/year/version as
// C strings/ints, a create/destroy function pair) that hands out the IGameModule, with the plugin
// built by the SAME toolchain/CRT/flags as the host. Do not export C++ types across a DLL boundary
// before that descriptor exists; `GameMetadata::minSdkMajor` alone is not a sufficient check.
class GamePluginRegistry {
public:
    // Validates metadata (non-empty unique id, year > 0, SDK major compatible).
    Status add(std::unique_ptr<IGameModule> module) {
        if (!module) return makeError(ErrorCode::InvalidArgument, "null game module");
        const GameMetadata meta = module->metadata();
        if (meta.id.empty()) return makeError(ErrorCode::InvalidArgument, "game module id must not be empty");
        if (meta.year <= 0 && meta.id.rfind("_", 0) != 0)
            return makeError(ErrorCode::InvalidArgument, "game module year must be > 0: " + meta.id);
        if (meta.minSdkMajor > kSdkVersionMajor)
            return makeError(ErrorCode::VersionMismatch, "plugin '" + meta.id + "' needs a newer SDK");
        if (meta.minSdkMajor == kSdkVersionMajor && meta.minSdkMinor > kSdkVersionMinor)
            return makeError(ErrorCode::VersionMismatch, "plugin '" + meta.id + "' needs a newer SDK minor");
        if (find(meta.id) != nullptr) return makeError(ErrorCode::AlreadyExists, "duplicate plugin id: " + meta.id);
        modules_.push_back(std::move(module));
        return okStatus();
    }

    IGameModule* find(std::string_view id) const {
        for (const auto& m : modules_)
            if (m->metadata().id == id) return m.get();
        return nullptr;
    }
    // First registered module for a season, nullptr => "unsupported year": the app runs with core features only.
    IGameModule* findByYear(int year) const {
        for (const auto& m : modules_)
            if (m->metadata().year == year) return m.get();
        return nullptr;
    }
    std::vector<IGameModule*> list() const {
        std::vector<IGameModule*> out;
        out.reserve(modules_.size());
        for (const auto& m : modules_) out.push_back(m.get());
        return out;
    }
    std::size_t size() const { return modules_.size(); }

private:
    std::vector<std::unique_ptr<IGameModule>> modules_;
};

}  // namespace wpiemu::sdk

// Defines `wpiemu::sdk::Status registerPlugin(wpiemu::sdk::GamePluginRegistry&)` in the CURRENT
// namespace (use the plugin's own namespace, e.g. wpiemu::plugins::game_2026) that registers
// one ModuleClass instance (default-constructible, derives from IGameModule). Declare the same
// function in the plugin's public header so the aggregation seam can call it.
#define WPIEMU_REGISTER_GAME_PLUGIN(ModuleClass)                                              \
    ::wpiemu::sdk::Status registerPlugin(::wpiemu::sdk::GamePluginRegistry& registry) {       \
        return registry.add(std::make_unique<ModuleClass>());                                 \
    }
