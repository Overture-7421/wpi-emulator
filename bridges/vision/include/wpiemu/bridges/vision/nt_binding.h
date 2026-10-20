#pragma once

// Real ntcore binding of the vision bridges. Only compiled/linked when ntcore was found (target
// wpiemu_bridges_vision_nt, CMake variable WPIEMU_NTCORE_FOUND); `wpiemu_bridges_vision` itself never needs it.
//
// The binding has NO link-time dependency on ntcore. It attaches at run time to the `ntcore` module the robot
// process already loaded and resolves the C API (ntcore_c.h) from it, exactly like the vendor bridges do for
// CTRE (docs/reference/bridges.md): a Java robot loads ntcore from an extracted temp directory through JNI, possibly after the
// extension was loaded, and a load-time import would either break the extension (ntcore not on the loader
// path) or bind a private second copy. Until the module is present `available()` is false and the bridges
// skip quietly; a robot program that never loads ntcore is unaffected.
//
// Time base: `nowUs()` is HAL_GetFPGATime() (resolved from the already loaded wpiHal) - the value
// Timer.getFPGATimestamp() reports. In a simulated robot this equals nt::Now(): the sim HAL calls
// wpi::SetNowImpl(GetFPGATime) in hal::init::InitializeMockHooks (allwpilib hal/src/main/native/sim/MockHooks.cpp,
// 2026.2.1), so timestamps ntcore stamps itself and the explicit ones written here share one clock.
// Without a loaded HAL (unit tests) NT_Now() is used.
//
// Verified on Windows (ntcore 2026.2.1). Linux/macOS use dlopen(RTLD_NOLOAD) by soname and are untested.

#include <memory>
#include <string>

#include "wpiemu/bridges/vision/nt_backend.h"
#include "wpiemu/bridges/vision/vision_bridges.h"

namespace wpiemu::bridges::vision {

struct NtcoreBackendOptions {
    // Load the ntcore module when the process has not loaded it yet. Tools and tests only: the robot process
    // must always be attached to, never loaded into.
    bool loadIfAbsent = false;
    // Module file names; empty = the platform default (ntcore.dll / libntcore.so / libntcore.dylib).
    std::string ntcoreModule;
    std::string halModule;
};

std::shared_ptr<INtBackend> makeNtcoreBackend(NtcoreBackendOptions options = {});

// registerVisionBridges(registry, makeNtcoreBackend(), options).
sdk::Status registerNtVisionBridges(sdk::VisionBridgeRegistry& registry, VisionBridgeOptions options = {});

}  // namespace wpiemu::bridges::vision
