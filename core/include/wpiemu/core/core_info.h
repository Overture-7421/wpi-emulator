#pragma once

#include <string_view>

namespace wpiemu::core {

std::string_view coreName();

std::string_view physicsBackendName();

}  // namespace wpiemu::core
