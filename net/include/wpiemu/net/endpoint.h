#pragma once

// Where a match host can be reached, as text:
//   enet://host:port             direct connect (LAN or port forwarding); `host` is a name, an IPv4 literal or an IPv6
//                                literal in brackets: enet://[::1]:7777
//   relay://host:port/ROOMID     through the relay; ROOMID is the 8-character room id derived from the invite
//                                (sdk::match::relay::isValidRoomId)
// Parsing is syntactic: it never resolves a name or touches the network. The port is required. Port 0 is accepted for
// enet:// (a listening endpoint asking for an ephemeral port) and rejected for relay://.

#include <cstdint>
#include <string>
#include <string_view>

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::net {

enum class EndpointScheme { Enet, Relay };

struct NetEndpoint {
    EndpointScheme scheme = EndpointScheme::Enet;
    std::string host;    // name or IP literal; IPv6 literals are stored without brackets
    std::uint16_t port = 0;
    std::string roomId;  // relay:// only; empty for enet://

    // InvalidArgument for a bad scheme, host, port or room id.
    static sdk::Result<NetEndpoint> parse(std::string_view text);
    // The relay's address as a person writes it, "host:port" or "relay://host:port" (surrounding spaces and a trailing
    // "/anything" are ignored; IPv6 literals use brackets), turned into the relay endpoint of `roomId`. InvalidArgument with
    // a message fit for the user otherwise.
    static sdk::Result<NetEndpoint> parseRelayAddress(std::string_view text, std::string_view roomId);
    // A host's address as a person writes it, "host", "host:port", "[::1]" or "enet://host:port" (spaces ignored). A missing
    // port is sdk::match::kDefaultDirectPort. InvalidArgument with a message fit for the user otherwise.
    static sdk::Result<NetEndpoint> parseDirectAddress(std::string_view text);
    // The canonical text; parse(format()) yields an equal endpoint.
    std::string format() const;

    bool operator==(const NetEndpoint&) const = default;
};

}  // namespace wpiemu::net
