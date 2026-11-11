#include "ui/relay_endpoints.h"

#include <cctype>

#include "wpiemu/sdk/core/environment.h"
#include "wpiemu/sdk/core/strings.h"

#ifndef WPIEMU_RELAY_DNS_SUFFIX
#define WPIEMU_RELAY_DNS_SUFFIX ""
#endif

namespace wpiemu::app {

bool isValidRelayDnsSuffix(const std::string& suffix) {
    // The label is <prefix>[-dev]<suffix>, so the suffix needs no hyphen of its own; it only must not end in one (a DNS label
    // cannot) and must fit, with the prefix, in a 63-character label.
    if (suffix.empty() || suffix.size() > 40 || suffix.back() == '-') return false;
    for (const char c : suffix)
        if (!(std::islower(static_cast<unsigned char>(c)) || std::isdigit(static_cast<unsigned char>(c)) || c == '-')) return false;
    return true;
}

std::string relayEndpointFor(RelayChoice choice, const std::string& dnsSuffix) {
    if (choice == RelayChoice::Custom || !isValidRelayDnsSuffix(dnsSuffix)) return {};
    std::string host = kRelayHostPrefix;
    if (choice == RelayChoice::Development) host += kRelayDevelopmentInfix;
    return host + dnsSuffix + "." + kRelayAzureDomain + ":" + std::to_string(kRelayUdpPort);
}

RelayPresets relayPresetsFor(const std::string& dnsSuffix) {
    RelayPresets presets;
    presets.configured = isValidRelayDnsSuffix(dnsSuffix);
    if (!presets.configured) return presets;
    presets.dnsSuffix = dnsSuffix;
    presets.production = relayEndpointFor(RelayChoice::Production, dnsSuffix);
    presets.development = relayEndpointFor(RelayChoice::Development, dnsSuffix);
    return presets;
}

std::string resolveRelayDnsSuffix(const IKeyValueStore& store) {
    const std::string fromEnvironment(sdk::asciiTrim(sdk::readEnvironment(kRelayDnsSuffixEnvironment).value_or(std::string())));
    if (!fromEnvironment.empty()) return fromEnvironment;
    const std::string stored = store.value(QString::fromLatin1(kRelayDnsSuffixSetting)).toString().trimmed().toStdString();
    if (!stored.empty()) return stored;
    return WPIEMU_RELAY_DNS_SUFFIX;
}

RelayChoice relayChoiceOf(const RelayPresets& presets, const std::string& endpoint) {
    const std::string text(sdk::asciiTrim(endpoint));
    if (text.empty()) return RelayChoice::Production;
    if (presets.configured && text == presets.production) return RelayChoice::Production;
    if (presets.configured && text == presets.development) return RelayChoice::Development;
    return RelayChoice::Custom;
}

}  // namespace wpiemu::app
