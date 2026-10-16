#pragma once

// Logging abstraction. Core/app provide the real sink (file, console, UI log panel); plugins and
// bridges only ever see ILogger via SimContext / bridge host.

#include <string_view>

#include "wpiemu/sdk/core/interface.h"

namespace wpiemu::sdk {

enum class LogLevel { Trace, Debug, Info, Warn, Error };

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
