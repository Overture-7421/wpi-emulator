#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/environment.h"

namespace wpiemu::core::launcher {

// "halsim_wpiemu.dll" / "libhalsim_wpiemu.dylib" / "libhalsim_wpiemu.so" for the OS this was built for.
std::string halsimExtensionFileName();

// Where the packaged / built HALSim extension may live, in priority order (docs/releasing.md). `lib` = halsimExtensionFileName():
//   1. $WPIEMU_EXTENSION               (full file path)
//   2. $WPIEMU_EXTENSION_DIR/<lib>
//   3. appDir/<lib>                    (build tree, portable zip)
//   4. appDir/../extensions/<lib>      (Windows install)
//   5. appDir/../lib/wpiemu/<lib>      (Linux prefix, AppImage)
//   6. appDir/../PlugIns/wpiemu/<lib>  (macOS bundle)
//   7. appDir/../../../<lib>           (macOS build tree: bin/<Config>/wpiemu_app.app/Contents/MacOS)
//   8. buildTreeFallback               (path baked in by CMake; may be empty)
// Candidates are lexically normalised; the list is the same on every OS so layouts can be tested anywhere.
std::vector<std::filesystem::path> halsimExtensionCandidates(const std::filesystem::path& appDir, const sdk::EnvLookup& getenv,
                                                             const std::filesystem::path& buildTreeFallback = {});

// First candidate that is an existing regular file, as an absolute canonical path; empty when none exists.
// Pure lookup: never creates or writes anything. `searched` (optional) receives the candidate list for diagnostics.
std::filesystem::path locateHalsimExtension(const std::filesystem::path& appDir, const sdk::EnvLookup& getenv,
                                            const std::filesystem::path& buildTreeFallback = {},
                                            std::vector<std::filesystem::path>* searched = nullptr);

}  // namespace wpiemu::core::launcher
