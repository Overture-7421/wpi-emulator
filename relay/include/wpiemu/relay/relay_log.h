#pragma once

// Structured log lines of the relay: JSON (one object per line: ts, level, event, then the fields) or plain text. Never
// log frame contents. Room ids (one-way hashes of the invite) and truncated peer addresses are fine.

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace wpiemu::relay {

enum class LogLevel : std::uint8_t { Debug, Info, Warn, Error };
enum class LogFormat : std::uint8_t { Json, Text };

struct LogField {
    std::string key;
    std::string value;
    bool numeric = false;

    LogField(std::string k, std::string v) : key(std::move(k)), value(std::move(v)) {}
    LogField(std::string k, const char* v) : key(std::move(k)), value(v) {}
    template <std::integral T>
    LogField(std::string k, T v) : key(std::move(k)), value(std::to_string(v)), numeric(true) {}
};

// "2026-11-08T22:21:55.166Z" for the system clock's current time.
std::string utcTimestamp();

class RelayLog {
public:
    using Sink = std::function<void(const std::string& line)>;
    using Timestamp = std::function<std::string()>;

    // A null sink writes to stdout and flushes; a null timestamp uses utcTimestamp().
    explicit RelayLog(LogFormat format = LogFormat::Json, Sink sink = nullptr, Timestamp timestamp = nullptr,
                      LogLevel minLevel = LogLevel::Info);

    void log(LogLevel level, std::string_view event, std::initializer_list<LogField> fields = {});
    // Like log(), but at most `perSecond` lines per second overall; the rest are counted and summarized by one
    // "log_suppressed" line, so a flood of hostile connections cannot flood the log.
    void logLimited(LogLevel level, std::string_view event, std::initializer_list<LogField> fields,
                    std::int64_t nowMs, std::uint32_t perSecond = 20);

    // Emits the pending log_suppressed summary once its window has passed (call it from a timer).
    void flushSuppressed(std::int64_t nowMs);

    // Formats one line; public for tests.
    std::string format(LogLevel level, std::string_view event, const LogField* fields, std::size_t count) const;

private:
    void write(LogLevel level, std::string_view event, const LogField* fields, std::size_t count);

    LogFormat format_;
    Sink sink_;
    Timestamp timestamp_;
    LogLevel minLevel_;
    std::mutex mutex_;
    std::int64_t windowStartMs_ = 0;
    std::uint32_t windowCount_ = 0;
    std::uint64_t suppressed_ = 0;
};

const char* logLevelName(LogLevel level);

}  // namespace wpiemu::relay
