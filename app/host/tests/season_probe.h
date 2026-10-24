#pragma once

// The season a probe end-to-end test runs against. CTest registers one test per season pack that declares `probes`
// (cmake in app/host/CMakeLists.txt) and passes the season through the environment, so the test sources name no year.

#include <cstdlib>
#include <filesystem>
#include <string>

#include "wpiemu/sdk/core/environment.h"

namespace wpiemu::app::host::testing::season {

inline std::string var(const char* name) { return wpiemu::sdk::readEnvironment(name).value_or(std::string()); }

inline int year() { return std::atoi(var("WPIEMU_SEASON_YEAR").c_str()); }
inline std::string wpilibVersion() { return var("WPIEMU_SEASON_VERSION"); }
inline std::filesystem::path samples() { return var("WPIEMU_SEASON_SAMPLES"); }
inline std::filesystem::path binaryDir() { return var("WPIEMU_SEASON_BINARY_DIR"); }
// The build-tree extension, when this build's primary season is the season under test; empty otherwise.
inline std::filesystem::path buildExtension() { return var("WPIEMU_SEASON_EXTENSION"); }

}  // namespace wpiemu::app::host::testing::season
