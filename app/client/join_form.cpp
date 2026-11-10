#include "client/join_form.h"

#include "wpiemu/sdk/core/strings.h"

namespace wpiemu::app::client {

std::string relayAddressOf(const JoinFormInput& input, const RelayPresets& presets) {
    if (input.relay == RelayChoice::Custom) return std::string(sdk::asciiTrim(input.customRelay));
    return presets.endpointOf(input.relay);
}

JoinValidation validateJoin(const JoinFormInput& input, const RelayPresets& presets) {
    JoinValidation out;
    const auto fail = [&](JoinField field, const char* key, std::string detail = {}) {
        out.errors.push_back({field, key, std::move(detail)});
    };

    std::optional<net::InviteSecret> secret;
    if (sdk::asciiTrim(input.invite).empty()) {
        fail(JoinField::Invite, "invite.empty");
    } else if (auto parsed = net::parseInvite(input.invite); parsed) {
        secret = std::move(parsed).value();
    } else {
        fail(JoinField::Invite, "invite.format", parsed.error().message);
    }

    const std::string name(sdk::asciiTrim(input.displayName));
    if (name.empty()) fail(JoinField::DisplayName, "name.empty");
    else if (name.size() > sdk::match::kMaxNameBytes) fail(JoinField::DisplayName, "name.too_long");

    if (sdk::asciiTrim(input.robotPath).empty()) fail(JoinField::Robot, "robot.missing");

    std::optional<net::NetEndpoint> endpoint;
    if (!sdk::asciiTrim(input.directAddress).empty()) {
        auto parsed = net::NetEndpoint::parseDirectAddress(input.directAddress);
        if (parsed) endpoint = std::move(parsed).value();
        else fail(JoinField::DirectAddress, "direct.address", parsed.error().message);
    } else if (const std::string address = relayAddressOf(input, presets); address.empty()) {
        fail(JoinField::Relay, input.relay == RelayChoice::Custom ? "relay.address" : "relay.unconfigured");
    } else if (secret) {
        auto parsed = net::NetEndpoint::parseRelayAddress(address, net::roomId(*secret));
        if (parsed) endpoint = std::move(parsed).value();
        else fail(JoinField::Relay, "relay.address", parsed.error().message);
    }

    if (out.errors.empty() && secret && endpoint) {
        JoinPlan plan{std::move(*secret), std::move(*endpoint), name, std::string(sdk::asciiTrim(input.robotPath))};
        out.plan = std::move(plan);
    }
    return out;
}

}  // namespace wpiemu::app::client
