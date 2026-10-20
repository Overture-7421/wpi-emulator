#pragma once

// Logging abstraction. Core/app provide the real sink (file, console, UI log panel); plugins and
// bridges only ever see ILogger via SimContext / bridge host.

#include <optional>
#include <string_view>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/strings.h"

namespace wpiemu::sdk {

enum class LogLevel { Trace, Debug, Info, Warn, Error };

// The upper-case name used on the robot process's console ("WARN"), and its inverse: case-insensitive, trimmed,
// "warning" accepted; anything else is nullopt. (Not called toString: doctest's stringifier would find it by ADL.)
inline std::string_view logLevelName(LogLevel level) {
    switch (level) {
        case LogLevel::Trace: return "TRACE";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info: return "INFO";
        case LogLevel::Warn: return "WARN";
        case LogLevel::Error: return "ERROR";
    }
    return "?";
}

inline std::optional<LogLevel> parseLogLevel(std::string_view text) {
    const std::string t = asciiLower(asciiTrim(text));
    if (t == "trace") return LogLevel::Trace;
    if (t == "debug") return LogLevel::Debug;
    if (t == "info") return LogLevel::Info;
    if (t == "warn" || t == "warning") return LogLevel::Warn;
    if (t == "error") return LogLevel::Error;
    return std::nullopt;
}

// Thread-safety: implementations MUST be safe to call from any thread (the sim thread, the IPC
// thread and the robot threads inside the extension all log). Never blocks on the UI.
class ILogger {
    WPIEMU_INTERFACE(ILogger)
public:
    // `category` is a short dotted origin such as "physics", "bridge.phoenix6", "plugin.game_2026".
    virtual void log(LogLevel level, std::string_view category, std::string_view message) = 0;

    void trace(std::string_view cat, std::string_view msg) { log(LogLevel::Trace, cat, msg); }
    void debug(std::string_view cat, std::string_view msg) { log(LogLevel::Debug, cat, msg); }
    void info(std::string_view cat, std::string_view msg) { log(LogLevel::Info, cat, msg); }
    void warn(std::string_view cat, std::string_view msg) { log(LogLevel::Warn, cat, msg); }
    void error(std::string_view cat, std::string_view msg) { log(LogLevel::Error, cat, msg); }
};

class NullLogger final : public ILogger {
public:
    void log(LogLevel, std::string_view, std::string_view) override {}
};

}  // namespace wpiemu::sdk
