#include "wpiemu/core/launcher/output_level.h"

#include <array>

namespace wpiemu::core::launcher {

namespace {

bool isWordChar(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; }

// `word` as a whole word (no word character directly before or after it).
bool containsWord(std::string_view line, std::string_view word) {
    for (std::size_t at = line.find(word); at != std::string_view::npos; at = line.find(word, at + 1)) {
        const bool startsWord = at == 0 || !isWordChar(line[at - 1]);
        const std::size_t end = at + word.size();
        const bool endsWord = end >= line.size() || !isWordChar(line[end]);
        if (startsWord && endsWord) return true;
    }
    return false;
}

template <std::size_t N>
bool containsAnyWord(std::string_view line, const std::array<std::string_view, N>& words) {
    for (const std::string_view word : words)
        if (containsWord(line, word)) return true;
    return false;
}

bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\f' || c == '\v'; }

// A Java stack frame: whitespace, then "at", then whitespace.
bool isStackFrame(std::string_view line) {
    std::size_t i = 0;
    while (i < line.size() && isSpace(line[i])) ++i;
    return i > 0 && line.substr(i, 2) == "at" && i + 2 < line.size() && isSpace(line[i + 2]);
}

}  // namespace

sdk::LogLevel classifyOutputLine(OutputStream stream, std::string_view line) {
    static constexpr std::array<std::string_view, 5> kError{"ERROR", "SEVERE", "FATAL", "Exception", "Caused by"};
    static constexpr std::array<std::string_view, 3> kWarn{"WARN", "WARNING", "Warning"};
    static constexpr std::array<std::string_view, 3> kTrace{"TRACE", "FINER", "FINEST"};
    static constexpr std::array<std::string_view, 2> kDebug{"DEBUG", "FINE"};
    if (containsAnyWord(line, kError) || isStackFrame(line)) return sdk::LogLevel::Error;
    if (containsAnyWord(line, kWarn)) return sdk::LogLevel::Warn;
    if (containsAnyWord(line, kTrace)) return sdk::LogLevel::Trace;
    if (containsAnyWord(line, kDebug)) return sdk::LogLevel::Debug;
    return stream == OutputStream::Stderr ? sdk::LogLevel::Warn : sdk::LogLevel::Info;
}

}  // namespace wpiemu::core::launcher
