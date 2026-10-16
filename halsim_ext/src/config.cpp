#include "wpiemu/halsim_ext/config.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdlib>
#include <string>

namespace wpiemu::halsim {

namespace {

std::string lower(std::string_view text) {
    std::string out(text);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

std::optional<std::uint32_t> parseUnsigned(std::string_view text) {
    text = trim(text);
    std::uint32_t v = 0;
    const auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), v);
    if (ec != std::errc{} || ptr != text.data() + text.size()) return std::nullopt;
    return v;
}

std::optional<bool> parseBool(std::string_view text) {
    const std::string t = lower(trim(text));
    if (t == "1" || t == "true" || t == "yes" || t == "on") return true;
    if (t == "0" || t == "false" || t == "no" || t == "off") return false;
    return std::nullopt;
}

}  // namespace

std::optional<std::string> processEnvironment(std::string_view name) {
    const std::string key(name);
#if defined(_MSC_VER)
    char* buffer = nullptr;
    std::size_t size = 0;
    if (_dupenv_s(&buffer, &size, key.c_str()) != 0 || buffer == nullptr) return std::nullopt;
    std::string value(buffer);
    std::free(buffer);
    return value;
#else
    const char* value = std::getenv(key.c_str());  // NOLINT(concurrency-mt-unsafe): read at init only
    if (value == nullptr) return std::nullopt;
    return std::string(value);
#endif
}

std::optional<sdk::LogLevel> parseLogLevel(std::string_view text) {
    const std::string t = lower(trim(text));
    if (t == "trace") return sdk::LogLevel::Trace;
    if (t == "debug") return sdk::LogLevel::Debug;
    if (t == "info") return sdk::LogLevel::Info;
    if (t == "warn" || t == "warning") return sdk::LogLevel::Warn;
    if (t == "error") return sdk::LogLevel::Error;
    return std::nullopt;
}

std::string_view toString(sdk::LogLevel level) {
    switch (level) {
        case sdk::LogLevel::Trace: return "TRACE";
        case sdk::LogLevel::Debug: return "DEBUG";
        case sdk::LogLevel::Info: return "INFO";
        case sdk::LogLevel::Warn: return "WARN";
        case sdk::LogLevel::Error: return "ERROR";
    }
    return "?";
}

ExtensionConfig parseExtensionConfig(const EnvLookup& env, std::vector<std::string>* warnings) {
    ExtensionConfig cfg;
    auto warn = [&](std::string_view name, const std::string& value, std::string_view why) {
        if (warnings != nullptr)
            warnings->push_back(std::string(name) + "='" + value + "': " + std::string(why) + "; using the default");
    };
    auto get = [&](std::string_view name, std::string_view alias = {}) -> std::optional<std::string> {
        std::optional<std::string> v = env(name);
        if ((!v || trim(*v).empty()) && !alias.empty()) v = env(alias);
        if (v && trim(*v).empty()) return std::nullopt;
        return v;
    };
    auto getMs = [&](std::string_view name, std::string_view alias, std::chrono::milliseconds& target,
                     std::uint32_t minMs, std::uint32_t maxMs) {
        const std::optional<std::string> v = get(name, alias);
        if (!v) return;
        const std::optional<std::uint32_t> parsed = parseUnsigned(*v);
        if (!parsed || *parsed < minMs || *parsed > maxMs) {
            warn(name, *v, "expected milliseconds in [" + std::to_string(minMs) + ", " + std::to_string(maxMs) + "]");
            return;
        }
        target = std::chrono::milliseconds(*parsed);
    };

    if (const auto v = get(kEnvEndpoint, kEnvEndpointAlias)) cfg.endpoint = std::string(trim(*v));
    if (const auto v = get(kEnvDisable)) {
        if (const auto b = parseBool(*v)) cfg.disabled = *b;
        else warn(kEnvDisable, *v, "expected 0/1");
    }
    if (const auto v = get(kEnvClock)) {
        const std::string t = lower(trim(*v));
        if (t == "master" || t == "lockstep") cfg.clock = ClockPreference::FollowEmulator;
        else if (t == "realtime" || t == "free") cfg.clock = ClockPreference::Realtime;
        else warn(kEnvClock, *v, "expected master|realtime");
    }
    if (const auto v = get(kEnvWatchdogMs, kEnvWatchdogMsAlias)) {
        const std::optional<std::uint32_t> parsed = parseUnsigned(*v);
        if (!parsed) warn(kEnvWatchdogMs, *v, "expected milliseconds");
        else cfg.proposedWatchdogMs = *parsed;
    }
    if (const auto v = get(kEnvWatchdog)) {
        const std::string t = lower(trim(*v));
        if (t == "off") cfg.watchdogEnabled = false;
        else if (t == "on") cfg.watchdogEnabled = true;
        else if (const auto b = parseBool(*v)) cfg.watchdogEnabled = *b;
        else warn(kEnvWatchdog, *v, "expected on|off");
    }
    getMs(kEnvConnectTimeoutMs, kEnvConnectTimeoutMsAlias, cfg.connectTimeout, 0, 600000);
    getMs(kEnvHandshakeTimeoutMs, {}, cfg.handshakeTimeout, 1, 600000);
    if (const auto v = get(kEnvReconnect)) {
        if (const auto b = parseBool(*v)) cfg.reconnect = *b;
        else warn(kEnvReconnect, *v, "expected 0/1");
    }
    if (const auto v = get(kEnvLogLevel)) {
        if (const auto level = parseLogLevel(*v)) cfg.logLevel = *level;
        else warn(kEnvLogLevel, *v, "expected trace|debug|info|warn|error");
    }
    if (const auto v = get(kEnvLiveDiscovery)) {
        if (const auto b = parseBool(*v)) cfg.pollWhileConstructing = *b;
        else warn(kEnvLiveDiscovery, *v, "expected 0/1");
    }
    if (const auto v = get(kEnvProgramName)) cfg.programName = std::string(trim(*v));
    return cfg;
}

}  // namespace wpiemu::halsim
