#pragma once

// Locale-independent ASCII string helpers for modules that cannot depend on each other.

#include <string>
#include <string_view>

namespace wpiemu::sdk {

// `text` with the letters A-Z lowered; every other byte (including non-ASCII) is untouched.
inline std::string asciiLower(std::string_view text) {
    std::string out(text);
    for (char& c : out)
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    return out;
}

}  // namespace wpiemu::sdk
