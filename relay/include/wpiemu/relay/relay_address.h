#pragma once

// The remote address of a connection, as far as the relay can know it. The ENet peer transports report it through
// ITransport::describe() as "enet://<ip>:<port>" (the UDP source address ENet saw; the form is "enet://?" when ENet could
// not render it). Nothing else is trusted: there is no X-Forwarded-For on UDP, and the relay is addressed directly.

#include <optional>
#include <string>

namespace wpiemu::relay {

struct SourceAddress {
    std::string key;      // lockout key: the IPv4 address, or the /64 prefix of an IPv6 address (one subscriber)
    std::string display;  // for logs: a.b.c.x (the last octet hidden) or the first three IPv6 groups
    bool loopback = false;
};

// nullopt when `describe` is not an enet:// endpoint with an IP literal: the caller must not guess.
std::optional<SourceAddress> sourceAddressOf(const std::string& describe);

}  // namespace wpiemu::relay
