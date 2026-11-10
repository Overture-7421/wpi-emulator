#pragma once

// The join form as a pure function: what a person typed (invite, an optional direct address, the relay choice, a display name
// and the robot file) becomes either a plan to connect or the list of fields that are wrong, each with a stable key. No widgets
// here, so the rules are unit-tested and the page only shows the result.

#include <optional>
#include <string>
#include <vector>

#include "ui/relay_endpoints.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::app::client {

struct JoinFormInput {
    std::string invite;            // XXXXX-XXXXX-XXXXX-XXXXX, any case, dashes optional
    std::string directAddress;     // empty: through the relay; "host", "host:port" or "enet://host:port": direct
    RelayChoice relay = RelayChoice::Production;
    std::string customRelay;       // RelayChoice::Custom: "host:port" or "relay://host:port"
    std::string displayName;
    std::string robotPath;         // the robot.json file; its content is checked by loadRobotFile
};

enum class JoinField { Invite, DirectAddress, Relay, DisplayName, Robot };

struct JoinFieldError {
    JoinField field = JoinField::Invite;
    // Stable key for tests and translation: invite.empty, invite.format, direct.address, relay.unconfigured, relay.address,
    // name.empty, name.too_long, robot.missing.
    std::string key;
    std::string detail;   // the parser's explanation, when it has one

    bool operator==(const JoinFieldError&) const = default;
};

// Where and how to connect, and who to be.
struct JoinPlan {
    net::InviteSecret secret;
    net::NetEndpoint endpoint;     // enet:// (direct) or relay://host:port/ROOM (the room derives from the invite)
    std::string displayName;
    std::string robotPath;

    bool direct() const { return endpoint.scheme == net::EndpointScheme::Enet; }
};

struct JoinValidation {
    std::optional<JoinPlan> plan;           // set exactly when `errors` is empty
    std::vector<JoinFieldError> errors;

    bool ok() const { return plan.has_value(); }
    const JoinFieldError* errorOf(JoinField field) const {
        for (const JoinFieldError& e : errors)
            if (e.field == field) return &e;
        return nullptr;
    }
};

// `presets` are the relays this build knows (empty while the DNS suffix is not configured: choose Custom then).
JoinValidation validateJoin(const JoinFormInput& input, const RelayPresets& presets);

// The relay address text a choice stands for: the preset, or the custom text.
std::string relayAddressOf(const JoinFormInput& input, const RelayPresets& presets);

}  // namespace wpiemu::app::client
