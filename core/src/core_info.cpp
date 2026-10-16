#include "wpiemu/core/core_info.h"

#include <box3d/box3d.h>

namespace wpiemu::core {

std::string_view coreName() { return "wpiemu_core"; }

std::string_view physicsBackendName() { return "box3d"; }

}  // namespace wpiemu::core
