#include "host/hosting.h"

#include <algorithm>

#include "wpiemu/net/endpoint.h"
#include "wpiemu/sdk/core/strings.h"

namespace wpiemu::app::host {

namespace {

// A syntactically valid room id (not the relay's reserved ping room): the relay address is parsed with the real endpoint parser.
constexpr const char* kPlaceholderRoom = "11111111";

}  // namespace

sdk::Result<RelayAddress> parseRelayAddress(const std::string& text) {
    std::string_view rest = sdk::asciiTrim(text);
    if (rest.empty()) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "Enter the relay address as host:port.");
    if (const std::size_t scheme = rest.find("://"); scheme != std::string_view::npos) {
        if (sdk::asciiLower(rest.substr(0, scheme)) != "relay")
            return sdk::makeError(sdk::ErrorCode::InvalidArgument, "The relay address must be host:port or relay://host:port.");
        rest.remove_prefix(scheme + 3);
    }
    const std::string authority(rest.substr(0, rest.find('/')));
    auto parsed = net::NetEndpoint::parse("relay://" + authority + "/" + kPlaceholderRoom);
    if (!parsed) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "The relay address is not valid: " + parsed.error().message);
    return RelayAddress{parsed.value().host, parsed.value().port};
}

sdk::Status validateHosting(const HostingOptions& options) {
    if (options.mode == HostingMode::Off) return sdk::okStatus();
    if (options.maxPlayers < kMinHostingPlayers || options.maxPlayers > sdk::match::kMaxRobotsPerMatch)
        return sdk::makeError(sdk::ErrorCode::InvalidArgument,
                              "The player count must be between " + std::to_string(kMinHostingPlayers) + " and " +
                                  std::to_string(sdk::match::kMaxRobotsPerMatch) + ".");
    if (options.phaseHudKey.size() > sdk::match::kMaxNameBytes)
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, "The match phase HUD key is too long.");
    if (options.mode == HostingMode::Direct) {
        if (options.bindHost.empty()) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "Enter the address to listen on.");
        return sdk::okStatus();
    }
    auto relay = parseRelayAddress(options.relayEndpoint);
    if (!relay) return relay.error();
    return sdk::okStatus();
}

HostingOptions normalizedHosting(HostingOptions options) {
    options.maxPlayers = std::clamp(options.maxPlayers, kMinHostingPlayers, sdk::match::kMaxRobotsPerMatch);
    if (options.bindHost.empty()) options.bindHost = "::";
    return options;
}

const char* hostingModeKey(HostingMode mode) {
    switch (mode) {
        case HostingMode::Direct: return "direct";
        case HostingMode::Relay: return "relay";
        case HostingMode::Off: break;
    }
    return "off";
}

HostingMode hostingModeFromKey(const std::string& key, HostingMode fallback) {
    if (key == "off") return HostingMode::Off;
    if (key == "direct") return HostingMode::Direct;
    if (key == "relay") return HostingMode::Relay;
    return fallback;
}

}  // namespace wpiemu::app::host
