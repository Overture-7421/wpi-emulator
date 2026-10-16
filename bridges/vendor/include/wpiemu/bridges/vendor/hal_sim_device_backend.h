#pragma once

// Real HAL SimDevice binding (ISimDeviceBackend over HALSIM_* / HAL_*SimValue). Compiled only when the WPILib
// HAL headers were found at configure time (CMake: WPIEMU_WPILIB_ROOT, macro WPIEMU_VENDOR_HAS_WPILIB). The
// HAL functions are resolved dynamically from the module the robot process already loaded (wpiHal), so this
// library has NO link-time dependency on WPILib and unit-test binaries run without it.

#include <memory>
#include <string>

#include "wpiemu/bridges/vendor/symbol_resolver.h"
#include "wpiemu/sdk/devices/sim_device_bridge.h"

namespace wpiemu::bridges::vendor {

// True when the HAL binding was compiled in (WPILib headers available at build time).
bool halBindingCompiledIn();

// Returns nullptr (and fills `error`) when the binding is not compiled in or wpiHal is not loaded / lacks
// the required exports. `hal` overrides symbol lookup (tests); default: attach to the loaded wpiHal module.
std::unique_ptr<sdk::ISimDeviceBackend> makeHalSimDeviceBackend(std::shared_ptr<const ISymbolResolver> hal = nullptr,
                                                                std::string* error = nullptr);

}  // namespace wpiemu::bridges::vendor
