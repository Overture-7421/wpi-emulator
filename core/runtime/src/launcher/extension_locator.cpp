#include "wpiemu/core/launcher/extension_locator.h"

namespace wpiemu::core::launcher {

namespace fs = std::filesystem;

std::string halsimExtensionFileName() {
#if defined(_WIN32)
    return "halsim_wpiemu.dll";
#elif defined(__APPLE__)
    return "libhalsim_wpiemu.dylib";
#else
    return "libhalsim_wpiemu.so";
#endif
}

std::vector<fs::path> halsimExtensionCandidates(const fs::path& appDir, const config::EnvLookup& getenv, const fs::path& buildTreeFallback) {
    const std::string lib = halsimExtensionFileName();
    const auto env = [&](const char* name) -> std::string { return getenv ? getenv(name).value_or(std::string()) : std::string(); };
    std::vector<fs::path> out;
    if (const std::string file = env("WPIEMU_EXTENSION"); !file.empty()) out.emplace_back(file);
    if (const std::string dir = env("WPIEMU_EXTENSION_DIR"); !dir.empty()) out.push_back(fs::path(dir) / lib);
    if (!appDir.empty()) {
        for (const fs::path& rel : {fs::path(lib), fs::path("..") / "extensions" / lib, fs::path("..") / "lib" / "wpiemu" / lib,
                                    fs::path("..") / "PlugIns" / "wpiemu" / lib, fs::path("..") / ".." / ".." / lib})
            out.push_back((appDir / rel).lexically_normal());
    }
    if (!buildTreeFallback.empty()) out.push_back(buildTreeFallback);
    return out;
}

fs::path locateHalsimExtension(const fs::path& appDir, const config::EnvLookup& getenv, const fs::path& buildTreeFallback,
                               std::vector<fs::path>* searched) {
    const std::vector<fs::path> candidates = halsimExtensionCandidates(appDir, getenv, buildTreeFallback);
    if (searched != nullptr) *searched = candidates;
    for (const fs::path& c : candidates) {
        std::error_code ec;
        if (!fs::is_regular_file(c, ec)) continue;
        fs::path canonical = fs::canonical(c, ec);  // resolves "..", symlinks and relative inputs
        if (ec) canonical = fs::absolute(c, ec);
        if (!ec) return canonical;
    }
    return {};
}

}  // namespace wpiemu::core::launcher
