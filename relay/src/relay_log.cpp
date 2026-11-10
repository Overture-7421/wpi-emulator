#include "wpiemu/relay/relay_log.h"

#include <chrono>
#include <cstdio>
#include <iostream>

namespace wpiemu::relay {

namespace {

void appendJsonString(std::string& out, std::string_view text) {
    out.push_back('"');
    for (const char c : text) {
        const auto u = static_cast<unsigned char>(c);
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (u < 0x20 || u == 0x7F) {
                    char buffer[8];
                    std::snprintf(buffer, sizeof buffer, "\\u%04x", u);
                    out += buffer;
                } else {
                    out.push_back(c);
                }
        }
    }
    out.push_back('"');
}

// Text format values are quoted only when they need it.
void appendTextValue(std::string& out, const LogField& field) {
    bool plain = !field.value.empty();
    for (const char c : field.value) plain = plain && c > ' ' && c != '"' && c != '=' && c != '\\' && c != 0x7F;
    if (plain) out += field.value;
    else appendJsonString(out, field.value);
}

}  // namespace

const char* logLevelName(LogLevel level) {
    switch (level) {
        case LogLevel::Debug: return "debug";
        case LogLevel::Info: return "info";
        case LogLevel::Warn: return "warn";
        case LogLevel::Error: return "error";
    }
    return "info";
}

std::string utcTimestamp() {
    using namespace std::chrono;
    const auto now = time_point_cast<milliseconds>(system_clock::now());
    const std::int64_t totalMs = now.time_since_epoch().count();
    std::int64_t days = totalMs / 86'400'000;
    std::int64_t msOfDay = totalMs % 86'400'000;
    if (msOfDay < 0) {
        msOfDay += 86'400'000;
        --days;
    }
    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
    const std::int64_t z = days + 719468;
    const std::int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const std::int64_t doe = z - era * 146097;
    const std::int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const std::int64_t mp = (5 * doy + 2) / 153;
    const std::int64_t day = doy - (153 * mp + 2) / 5 + 1;
    const std::int64_t month = mp < 10 ? mp + 3 : mp - 9;
    const std::int64_t year = yoe + era * 400 + (month <= 2 ? 1 : 0);
    char buffer[40];
    std::snprintf(buffer, sizeof buffer, "%04lld-%02lld-%02lldT%02lld:%02lld:%02lld.%03lldZ", static_cast<long long>(year),
                  static_cast<long long>(month), static_cast<long long>(day), static_cast<long long>(msOfDay / 3'600'000),
                  static_cast<long long>(msOfDay / 60'000 % 60), static_cast<long long>(msOfDay / 1000 % 60),
                  static_cast<long long>(msOfDay % 1000));
    return buffer;
}

RelayLog::RelayLog(LogFormat format, Sink sink, Timestamp timestamp, LogLevel minLevel)
    : format_(format), sink_(std::move(sink)), timestamp_(std::move(timestamp)), minLevel_(minLevel) {
    if (!sink_) {
        sink_ = [](const std::string& line) {
            std::cout << line << '\n';
            std::cout.flush();
        };
    }
    if (!timestamp_) timestamp_ = [] { return utcTimestamp(); };
}

std::string RelayLog::format(LogLevel level, std::string_view event, const LogField* fields, std::size_t count) const {
    std::string line;
    const std::string ts = timestamp_();
    if (format_ == LogFormat::Json) {
        line = "{\"ts\":";
        appendJsonString(line, ts);
        line += ",\"level\":";
        appendJsonString(line, logLevelName(level));
        line += ",\"event\":";
        appendJsonString(line, event);
        for (std::size_t i = 0; i < count; ++i) {
            line.push_back(',');
            appendJsonString(line, fields[i].key);
            line.push_back(':');
            if (fields[i].numeric) line += fields[i].value;
            else appendJsonString(line, fields[i].value);
        }
        line.push_back('}');
    } else {
        line = ts + " " + logLevelName(level) + " " + std::string(event);
        for (std::size_t i = 0; i < count; ++i) {
            line.push_back(' ');
            line += fields[i].key;
            line.push_back('=');
            appendTextValue(line, fields[i]);
        }
    }
    return line;
}

void RelayLog::emit(LogLevel level, std::string_view event, const LogField* fields, std::size_t count) {
    sink_(format(level, event, fields, count));
}

void RelayLog::log(LogLevel level, std::string_view event, std::initializer_list<LogField> fields) {
    if (level < minLevel_) return;
    std::lock_guard lock(mutex_);
    emit(level, event, fields.begin(), fields.size());
}

void RelayLog::flushSuppressed(std::int64_t nowMs) {
    std::lock_guard lock(mutex_);
    if (suppressed_ == 0 || nowMs - windowStartMs_ < 1000) return;
    const LogField summary[] = {{"count", suppressed_}};
    emit(LogLevel::Warn, "log_suppressed", summary, 1);
    suppressed_ = 0;
    windowStartMs_ = nowMs;
    windowCount_ = 0;
}

void RelayLog::logLimited(LogLevel level, std::string_view event, std::initializer_list<LogField> fields,
                          std::int64_t nowMs, std::uint32_t perSecond) {
    if (level < minLevel_) return;
    std::lock_guard lock(mutex_);
    if (nowMs - windowStartMs_ >= 1000) {
        if (suppressed_ != 0) {
            const LogField summary[] = {{"count", suppressed_}};
            emit(LogLevel::Warn, "log_suppressed", summary, 1);
            suppressed_ = 0;
        }
        windowStartMs_ = nowMs;
        windowCount_ = 0;
    }
    if (windowCount_ >= perSecond) {
        ++suppressed_;
        return;
    }
    ++windowCount_;
    emit(level, event, fields.begin(), fields.size());
}

}  // namespace wpiemu::relay
