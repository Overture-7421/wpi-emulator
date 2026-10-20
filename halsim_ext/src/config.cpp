#include "wpiemu/halsim_ext/config.h"

#include <string>

#include "wpiemu/sdk/core/strings.h"

namespace wpiemu::halsim {

namespace {

using sdk::asciiLower;
using sdk::asciiTrim;
using sdk::parseBool;
namespace names = sdk::env_names;

std::optional<std::uint32_t> parseUnsigned(std::string_view text) { return sdk::parseInteger<std::uint32_t>(asciiTrim(text)); }

}  // namespace

ExtensionConfig parseExtensionConfig(const sdk::EnvLookup& env, std::vector<std::string>* warnings) {
    ExtensionConfig cfg;
    auto warn = [&](std::string_view name, const std::string& value, std::string_view why) {
        if (warnings != nullptr)
            warnings->push_back(std::string(name) + "='" + value + "': " + std::string(why) + "; using the default");
    };
    auto get = [&](std::string_view name, std::string_view alias = {}) -> std::optional<std::string> {
        std::optional<std::string> v = env(name);
        if ((!v || asciiTrim(*v).empty()) && !alias.empty()) v = env(alias);
        if (v && asciiTrim(*v).empty()) return std::nullopt;
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

    if (const auto v = get(names::kIpcEndpoint, names::kIpcEndpointAlias)) cfg.endpoint = std::string(asciiTrim(*v));
    if (const auto v = get(kEnvDisable)) {
        if (const auto b = parseBool(*v)) cfg.disabled = *b;
        else warn(kEnvDisable, *v, "expected 0/1");
    }
    if (const auto v = get(names::kClock)) {
        const std::string t = asciiLower(asciiTrim(*v));
        if (t == "master" || t == "lockstep") cfg.clock = ClockPreference::FollowEmulator;
        else if (t == "realtime" || t == "free") cfg.clock = ClockPreference::Realtime;
        else warn(names::kClock, *v, "expected master|realtime");
    }
    if (const auto v = get(names::kIpcWatchdogMs, kEnvWatchdogMsAlias)) {
        const std::optional<std::uint32_t> parsed = parseUnsigned(*v);
        if (!parsed) warn(names::kIpcWatchdogMs, *v, "expected milliseconds");
        else cfg.proposedWatchdogMs = *parsed;
    }
    if (const auto v = get(kEnvWatchdog)) {
        const std::string t = asciiLower(asciiTrim(*v));
        if (t == "off") cfg.watchdogEnabled = false;
        else if (t == "on") cfg.watchdogEnabled = true;
        else if (const auto b = parseBool(*v)) cfg.watchdogEnabled = *b;
        else warn(kEnvWatchdog, *v, "expected on|off");
    }
    getMs(names::kIpcConnectTimeoutMs, kEnvConnectTimeoutMsAlias, cfg.connectTimeout, 0, 600000);
    getMs(kEnvHandshakeTimeoutMs, {}, cfg.handshakeTimeout, 1, 600000);
    if (const auto v = get(kEnvReconnect)) {
        if (const auto b = parseBool(*v)) cfg.reconnect = *b;
        else warn(kEnvReconnect, *v, "expected 0/1");
    }
    if (const auto v = get(names::kLogLevel)) {
        if (const auto level = sdk::parseLogLevel(*v)) cfg.logLevel = *level;
        else warn(names::kLogLevel, *v, "expected trace|debug|info|warn|error");
    }
    if (const auto v = get(names::kLiveDiscovery)) {
        if (const auto b = parseBool(*v)) cfg.pollWhileConstructing = *b;
        else warn(names::kLiveDiscovery, *v, "expected 0/1");
    }
    if (const auto v = get(names::kProgramName)) cfg.programName = std::string(asciiTrim(*v));
    return cfg;
}

}  // namespace wpiemu::halsim
