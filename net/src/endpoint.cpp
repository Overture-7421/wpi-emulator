#include "wpiemu/net/endpoint.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <vector>

#include "wpiemu/sdk/core/strings.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::net {
namespace {

using sdk::ErrorCode;
using sdk::makeError;

sdk::Error bad(const std::string& why) { return makeError(ErrorCode::InvalidArgument, "invalid endpoint: " + why); }

bool isDigit(char c) { return c >= '0' && c <= '9'; }
bool isHex(char c) { return isDigit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }
bool isAlnum(char c) { return isDigit(c) || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

std::string lower(std::string_view s) {
    std::string out(s);
    std::transform(out.begin(), out.end(), out.begin(), [](char c) { return static_cast<char>(std::tolower(static_cast<unsigned char>(c))); });
    return out;
}

std::vector<std::string_view> split(std::string_view s, char sep) {
    std::vector<std::string_view> parts;
    std::size_t start = 0;
    for (;;) {
        const std::size_t at = s.find(sep, start);
        if (at == std::string_view::npos) {
            parts.push_back(s.substr(start));
            return parts;
        }
        parts.push_back(s.substr(start, at - start));
        start = at + 1;
    }
}

bool isIpv4Literal(std::string_view s) {
    const auto parts = split(s, '.');
    if (parts.size() != 4) return false;
    for (const auto part : parts) {
        if (part.empty() || part.size() > 3 || (part.size() > 1 && part.front() == '0')) return false;
        int value = 0;
        for (const char c : part) {
            if (!isDigit(c)) return false;
            value = value * 10 + (c - '0');
        }
        if (value > 255) return false;
    }
    return true;
}

bool isIpv6Literal(std::string_view s) {
    if (s.size() < 2 || s.size() > 45) return false;
    const std::size_t doubleColon = s.find("::");
    if (doubleColon != std::string_view::npos && s.find("::", doubleColon + 1) != std::string_view::npos) return false;
    const bool compressed = doubleColon != std::string_view::npos;
    std::string_view head = s;
    std::string_view tail;
    if (compressed) {
        head = s.substr(0, doubleColon);
        tail = s.substr(doubleColon + 2);
    }
    std::size_t groups = 0;
    auto countGroups = [&groups](std::string_view part, bool mayEndWithIpv4) {
        if (part.empty()) return true;
        const auto items = split(part, ':');
        for (std::size_t i = 0; i < items.size(); ++i) {
            const auto item = items[i];
            if (item.empty()) return false;
            if (mayEndWithIpv4 && i + 1 == items.size() && item.find('.') != std::string_view::npos) {
                if (!isIpv4Literal(item)) return false;
                groups += 2;
                continue;
            }
            if (item.size() > 4 || !std::all_of(item.begin(), item.end(), isHex)) return false;
            ++groups;
        }
        return true;
    };
    if (!countGroups(head, !compressed) || !countGroups(tail, true)) return false;
    return compressed ? groups <= 7 : groups == 8;
}

bool isHostname(std::string_view s) {
    if (s.empty() || s.size() > 253) return false;
    for (const auto label : split(s, '.')) {
        if (label.empty() || label.size() > 63 || label.front() == '-' || label.back() == '-') return false;
        if (!std::all_of(label.begin(), label.end(), [](char c) { return isAlnum(c) || c == '-'; })) return false;
    }
    return true;
}

bool isValidHost(std::string_view s) {
    if (std::all_of(s.begin(), s.end(), [](char c) { return isDigit(c) || c == '.'; })) return isIpv4Literal(s);
    return isHostname(s);
}

bool parsePort(std::string_view s, std::uint16_t& port) {
    if (s.empty() || s.size() > 5) return false;
    unsigned value = 0;
    for (const char c : s) {
        if (!isDigit(c)) return false;
        value = value * 10 + static_cast<unsigned>(c - '0');
    }
    if (value > 65535) return false;
    port = static_cast<std::uint16_t>(value);
    return true;
}

}  // namespace

sdk::Result<NetEndpoint> NetEndpoint::parse(std::string_view text) {
    const std::size_t schemeEnd = text.find("://");
    if (schemeEnd == std::string_view::npos) return bad("missing scheme, expected enet:// or relay://");
    const std::string scheme = lower(text.substr(0, schemeEnd));
    NetEndpoint endpoint;
    if (scheme == "enet") endpoint.scheme = EndpointScheme::Enet;
    else if (scheme == "relay") endpoint.scheme = EndpointScheme::Relay;
    else return bad("unknown scheme '" + scheme + "'");

    std::string_view rest = text.substr(schemeEnd + 3);
    const std::size_t slash = rest.find('/');
    const std::string_view authority = rest.substr(0, slash);
    const std::string_view path = slash == std::string_view::npos ? std::string_view{} : rest.substr(slash);

    std::string_view hostPart;
    std::string_view portPart;
    if (!authority.empty() && authority.front() == '[') {
        const std::size_t close = authority.find(']');
        if (close == std::string_view::npos) return bad("unterminated '[' in the host");
        hostPart = authority.substr(1, close - 1);
        if (!isIpv6Literal(hostPart)) return bad("malformed IPv6 address");
        const std::string_view after = authority.substr(close + 1);
        if (after.empty() || after.front() != ':') return bad("missing port");
        portPart = after.substr(1);
    } else {
        const std::size_t colon = authority.rfind(':');
        if (colon == std::string_view::npos) return bad("missing port");
        hostPart = authority.substr(0, colon);
        portPart = authority.substr(colon + 1);
        if (hostPart.empty()) return bad("empty host");
        if (!isValidHost(hostPart)) return bad("malformed host '" + std::string(hostPart) + "'");
    }
    if (!parsePort(portPart, endpoint.port)) return bad("port must be a number from 0 to 65535");
    endpoint.host = std::string(hostPart);

    if (endpoint.scheme == EndpointScheme::Enet) {
        if (!path.empty()) return bad("enet:// takes no path");
        return endpoint;
    }
    if (endpoint.port == 0) return bad("relay:// needs a nonzero port");
    if (path.size() < 2) return bad("relay:// needs a room id: relay://host:port/ROOMID");
    endpoint.roomId = std::string(path.substr(1));
    if (!sdk::match::relay::isValidRoomId(endpoint.roomId)) return bad("malformed room id");
    return endpoint;
}

sdk::Result<NetEndpoint> NetEndpoint::parseRelayAddress(std::string_view text, std::string_view roomId) {
    std::string_view rest = sdk::asciiTrim(text);
    if (rest.empty()) return makeError(ErrorCode::InvalidArgument, "Enter the relay address as host:port.");
    if (const std::size_t scheme = rest.find("://"); scheme != std::string_view::npos) {
        if (lower(rest.substr(0, scheme)) != "relay")
            return makeError(ErrorCode::InvalidArgument, "The relay address must be host:port or relay://host:port.");
        rest.remove_prefix(scheme + 3);
    }
    const std::string authority(rest.substr(0, rest.find('/')));
    auto parsed = parse("relay://" + authority + "/" + std::string(roomId));
    if (!parsed) return makeError(ErrorCode::InvalidArgument, "The relay address is not valid: " + parsed.error().message);
    return parsed;
}

sdk::Result<NetEndpoint> NetEndpoint::parseDirectAddress(std::string_view text) {
    std::string_view rest = sdk::asciiTrim(text);
    if (rest.empty()) return makeError(ErrorCode::InvalidArgument, "Enter the host's address, for example 192.168.1.20:47300.");
    if (const std::size_t scheme = rest.find("://"); scheme != std::string_view::npos) {
        if (lower(rest.substr(0, scheme)) != "enet")
            return makeError(ErrorCode::InvalidArgument, "The host's address must be host:port or enet://host:port.");
        rest.remove_prefix(scheme + 3);
    }
    const std::string authority(rest);
    const bool bracketed = !authority.empty() && authority.front() == '[';
    const bool hasPort = bracketed ? authority.find("]:") != std::string::npos : authority.find(':') != std::string::npos;
    const std::string withPort = hasPort ? authority : authority + ':' + std::to_string(sdk::match::kDefaultDirectPort);
    auto parsed = parse("enet://" + withPort);
    if (!parsed) return makeError(ErrorCode::InvalidArgument, "The host's address is not valid: " + parsed.error().message);
    return parsed;
}

std::string NetEndpoint::format() const {
    std::string out = scheme == EndpointScheme::Enet ? "enet://" : "relay://";
    if (host.find(':') != std::string::npos) out += '[' + host + ']';
    else out += host;
    out += ':' + std::to_string(port);
    if (scheme == EndpointScheme::Relay) out += '/' + roomId;
    return out;
}

}  // namespace wpiemu::net
