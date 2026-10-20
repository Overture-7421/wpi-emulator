#pragma once

// Locale-independent ASCII string helpers for modules that cannot depend on each other.

#include <charconv>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

namespace wpiemu::sdk {

// `text` with the letters A-Z lowered; every other byte (including non-ASCII) is untouched.
inline std::string asciiLower(std::string_view text) {
    std::string out(text);
    for (char& c : out)
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    return out;
}

// `text` without leading and trailing ASCII whitespace (space, \t, \n, \v, \f, \r) or NUL padding.
inline std::string_view asciiTrim(std::string_view text) {
    const auto padding = [](char c) { return c == ' ' || c == '\0' || (c >= '\t' && c <= '\r'); };
    while (!text.empty() && padding(text.front())) text.remove_prefix(1);
    while (!text.empty() && padding(text.back())) text.remove_suffix(1);
    return text;
}

// 1/true/yes/on or 0/false/no/off, case-insensitive and trimmed; anything else is nullopt.
inline std::optional<bool> parseBool(std::string_view text) {
    const std::string t = asciiLower(asciiTrim(text));
    if (t == "1" || t == "true" || t == "yes" || t == "on") return true;
    if (t == "0" || t == "false" || t == "no" || t == "off") return false;
    return std::nullopt;
}

// The whole of `text` as a decimal integer (a leading '-' only for signed types); nullopt on junk, whitespace or overflow.
template <class Int>
std::optional<Int> parseInteger(std::string_view text) {
    Int value{};
    const auto [end, ec] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (ec != std::errc{} || end != text.data() + text.size()) return std::nullopt;
    return value;
}

}  // namespace wpiemu::sdk
