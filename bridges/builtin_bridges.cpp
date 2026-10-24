// The bridges/ aggregation seam (mirrors seasons/builtin_plugins.cpp): the ONLY translation unit that
// knows every built-in device-bridge module. Declared in sdk/devices/builtin_bridges.h so the HALSim
// extension can call it without including or linking any bridges/<x> module.
//
// Adding a bridge module: add_subdirectory(<name>) in bridges/CMakeLists.txt, link its target into
// wpiemu_bridges_builtin there, and add one call below. The extension itself does not change.
//
// Which calls exist is decided by bridges/CMakeLists.txt (compile definitions), so this file keeps
// compiling while a module's registration function is still being written or its WPILib binding is
// not available on this machine.
#include <type_traits>
#include <utility>

#include "wpiemu/sdk/devices/builtin_bridges.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/vision/vision_bridge.h"

#if defined(WPIEMU_BUILTIN_HAL_HALSIM)
#include "wpiemu/bridges/hal/hal_sim_binding.h"
#endif
#if defined(WPIEMU_BUILTIN_VENDOR_REGISTER)
#include "wpiemu/bridges/vendor/vendor_bridges.h"
#endif
#include "wpiemu/bridges/vision/vision_bridges.h"
#if defined(WPIEMU_BUILTIN_VISION_NT)
#include "wpiemu/bridges/vision/nt_binding.h"
#endif

namespace wpiemu::sdk {

namespace {

// Runs one module's registration; modules return either Status or void.
template <class Fn>
void attempt(Status& first, Fn&& registration) {
    if constexpr (std::is_void_v<std::invoke_result_t<Fn>>) {
        registration();
    } else {
        Status s = registration();
        if (first.ok() && !s.ok()) first = std::move(s);
    }
}

}  // namespace

Status registerBuiltinBridges(DeviceBridgeRegistry& registry) {
    Status first = okStatus();
    (void)registry;

#if defined(WPIEMU_BUILTIN_HAL_HALSIM)
    attempt(first, [&] { return bridges::hal::registerHalBridges(registry); });
#endif
#if defined(WPIEMU_BUILTIN_VENDOR_REGISTER)
    attempt(first, [&] { return bridges::vendor::WPIEMU_BUILTIN_VENDOR_REGISTER(registry); });
#endif

    return first;
}

Status registerBuiltinVisionBridges(VisionBridgeRegistry& registry) {
    Status first = okStatus();

#if defined(WPIEMU_BUILTIN_VISION_NT)
    attempt(first, [&] { return bridges::vision::registerNtVisionBridges(registry); });
#else
    // Without the ntcore binding the bridges are registered but never publish.
    attempt(first, [&] { return bridges::vision::registerVisionBridges(registry); });
#endif

    return first;
}

}  // namespace wpiemu::sdk
