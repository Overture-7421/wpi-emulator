#pragma once

// The real INtApi: a run-time binding of the ntcore C API of the ROBOT process (work stream nt-binding).
// ntcore is never linked or loaded by us: the binding attaches to the copy the robot already mapped
// (a private copy would be a different NetworkTables instance) and uses its default instance.

#include <memory>

#include "wpiemu/bridges/vision/nt_api.h"

namespace wpiemu::bridges::vision {

// Never null. available() turns true once ntcore is found and every required symbol resolved; until then (and
// forever when this build has no ntcore headers, or on an unsupported platform) it behaves like NullNtApi and
// unavailableReason() says why.
std::unique_ptr<INtApi> makeNtCoreApi();

}  // namespace wpiemu::bridges::vision
