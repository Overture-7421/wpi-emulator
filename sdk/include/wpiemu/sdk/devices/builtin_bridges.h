#pragma once

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::sdk {

class DeviceBridgeRegistry;

// Aggregation seam for device bridges (mirrors game/builtin_plugins.h).
// DECLARED here so the extension composition root can call it without including or linking any
// bridges/<x> module; DEFINED by the `wpiemu_bridges_builtin` target (bridges/builtin_bridges.cpp),
// the only place that knows every built-in bridge module. Adding a vendor = a bridges/<name>/
// module + one call in that file; the extension does not change.
// Returns the first registration error (all modules are still attempted).
Status registerBuiltinBridges(DeviceBridgeRegistry& registry);

}  // namespace wpiemu::sdk
