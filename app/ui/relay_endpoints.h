#pragma once

// Where the practice-match relay runs. The two Azure relays (infra/relay/main.bicep) are reachable at
//   production:  wpiemu-relay<suffix>.southcentralus.azurecontainer.io:7777
//   development: wpiemu-relay-dev<suffix>.southcentralus.azurecontainer.io:7777
// where <suffix> is the deployment's `dnsLabelSuffix` (a hyphen and a few characters that make the DNS label unique). The
// suffix is not known to a build until the relay is deployed, so it is a setting: the environment variable
// WPIEMU_RELAY_DNS_SUFFIX (the one the Bicep parameters read), then the stored preference "hosting/relayDnsSuffix", then the
// value a release build bakes in (CMake: WPIEMU_RELAY_DNS_SUFFIX, empty by default). Without any of them the presets are
// empty and the UI asks for a custom address.

#include <QString>

#include <cstdint>
#include <string>

#include "ui/key_value_store.h"

namespace wpiemu::app {

// Same as `location`, `namePrefix` and `udpPort` in infra/relay/main.bicep.
inline constexpr char kRelayHostPrefix[] = "wpiemu-relay";
inline constexpr char kRelayDevelopmentInfix[] = "-dev";
inline constexpr char kRelayAzureDomain[] = "southcentralus.azurecontainer.io";
inline constexpr std::uint16_t kRelayUdpPort = 7777;

inline constexpr char kRelayDnsSuffixSetting[] = "hosting/relayDnsSuffix";
inline constexpr char kRelayDnsSuffixEnvironment[] = "WPIEMU_RELAY_DNS_SUFFIX";

enum class RelayChoice { Production, Development, Custom };

// The two known relays, "host:port" each; empty while the suffix is unknown (`configured` false).
struct RelayPresets {
    bool configured = false;
    std::string dnsSuffix;
    std::string production;
    std::string development;

    const std::string& endpointOf(RelayChoice choice) const { return choice == RelayChoice::Development ? development : production; }
};

// A DNS label suffix as the Bicep parameter documents it: a hyphen first, then lower-case letters, digits and hyphens.
bool isValidRelayDnsSuffix(const std::string& suffix);
// "wpiemu-relay[-dev]<suffix>.southcentralus.azurecontainer.io:7777"; empty for an invalid suffix.
std::string relayEndpointFor(RelayChoice choice, const std::string& dnsSuffix);
RelayPresets relayPresetsFor(const std::string& dnsSuffix);
// The suffix in force: environment, then the stored preference, then the build's. May be empty.
std::string resolveRelayDnsSuffix(const IKeyValueStore& store);
inline RelayPresets relayPresets(const IKeyValueStore& store) { return relayPresetsFor(resolveRelayDnsSuffix(store)); }

// Which preset (or custom address) an endpoint text stands for; an empty endpoint is the default, Production.
RelayChoice relayChoiceOf(const RelayPresets& presets, const std::string& endpoint);

}  // namespace wpiemu::app
