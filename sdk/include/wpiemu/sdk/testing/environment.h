#pragma once

// A fixed environment for code that takes an EnvLookup.

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>

#include "wpiemu/sdk/core/environment.h"

namespace wpiemu::sdk::testing {

inline EnvLookup envOf(std::map<std::string, std::string, std::less<>> vars) {
    return [vars = std::move(vars)](std::string_view name) -> std::optional<std::string> {
        const auto it = vars.find(name);
        if (it == vars.end()) return std::nullopt;
        return it->second;
    };
}

}  // namespace wpiemu::sdk::testing
