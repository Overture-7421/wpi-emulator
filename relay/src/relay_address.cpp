#include "wpiemu/relay/relay_address.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "wpiemu/net/endpoint.h"

namespace wpiemu::relay {

namespace {

bool parseIpv4(const std::string& text, std::array<std::uint8_t, 4>& out) {
    std::size_t index = 0;
    std::size_t pos = 0;
    while (index < 4) {
        std::size_t digits = 0;
        unsigned value = 0;
        while (pos < text.size() && text[pos] >= '0' && text[pos] <= '9' && digits < 4) {
            value = value * 10 + static_cast<unsigned>(text[pos] - '0');
            ++pos;
            ++digits;
        }
        if (digits == 0 || digits > 3 || value > 255) return false;
        out[index++] = static_cast<std::uint8_t>(value);
        if (index < 4) {
            if (pos >= text.size() || text[pos] != '.') return false;
            ++pos;
        }
    }
    return pos == text.size();
}

bool parseGroup(const std::string& text, std::uint16_t& out) {
    if (text.empty() || text.size() > 4) return false;
    unsigned value = 0;
    for (const char c : text) {
        unsigned digit;
        if (c >= '0' && c <= '9') digit = static_cast<unsigned>(c - '0');
        else if (c >= 'a' && c <= 'f') digit = static_cast<unsigned>(c - 'a') + 10;
        else if (c >= 'A' && c <= 'F') digit = static_cast<unsigned>(c - 'A') + 10;
        else return false;
        value = value * 16 + digit;
    }
    out = static_cast<std::uint16_t>(value);
    return true;
}

// Splits `text` on ':' into groups; an embedded IPv4 tail counts as two groups. `compressedAt` is the index where "::"
// stood, or -1.
bool splitIpv6(const std::string& text, std::vector<std::uint16_t>& groups, int& compressedAt) {
    compressedAt = -1;
    std::size_t pos = 0;
    if (text.rfind("::", 0) == 0) {
        compressedAt = 0;
        pos = 2;
    } else if (!text.empty() && text[0] == ':') {
        return false;
    }
    while (pos < text.size()) {
        const std::size_t next = text.find(':', pos);
        const std::string piece = text.substr(pos, next == std::string::npos ? std::string::npos : next - pos);
        if (piece.find('.') != std::string::npos) {
            std::array<std::uint8_t, 4> v4{};
            if (next != std::string::npos || !parseIpv4(piece, v4)) return false;
            groups.push_back(static_cast<std::uint16_t>((v4[0] << 8) | v4[1]));
            groups.push_back(static_cast<std::uint16_t>((v4[2] << 8) | v4[3]));
            break;
        }
        std::uint16_t group = 0;
        if (!parseGroup(piece, group)) return false;
        groups.push_back(group);
        if (next == std::string::npos) break;
        pos = next + 1;
        if (pos < text.size() && text[pos] == ':') {
            if (compressedAt >= 0) return false;
            compressedAt = static_cast<int>(groups.size());
            ++pos;
        } else if (pos >= text.size()) {
            return false;
        }
    }
    return groups.size() <= 8;
}

bool parseIpv6(const std::string& text, std::array<std::uint16_t, 8>& out) {
    std::vector<std::uint16_t> groups;
    int compressedAt = -1;
    if (!splitIpv6(text, groups, compressedAt)) return false;
    if (compressedAt < 0) {
        if (groups.size() != 8) return false;
        for (std::size_t i = 0; i < 8; ++i) out[i] = groups[i];
        return true;
    }
    if (groups.size() > 7) return false;
    out.fill(0);
    const std::size_t head = static_cast<std::size_t>(compressedAt);
    for (std::size_t i = 0; i < head; ++i) out[i] = groups[i];
    const std::size_t tail = groups.size() - head;
    for (std::size_t i = 0; i < tail; ++i) out[8 - tail + i] = groups[head + i];
    return true;
}

std::string hex(std::uint16_t value) {
    char buffer[8];
    std::snprintf(buffer, sizeof buffer, "%x", static_cast<unsigned>(value));
    return buffer;
}

SourceAddress fromIpv4(const std::array<std::uint8_t, 4>& a) {
    SourceAddress source;
    source.key = std::to_string(a[0]) + "." + std::to_string(a[1]) + "." + std::to_string(a[2]) + "." + std::to_string(a[3]);
    source.display = std::to_string(a[0]) + "." + std::to_string(a[1]) + "." + std::to_string(a[2]) + ".x";
    source.loopback = a[0] == 127;
    return source;
}

}  // namespace

std::optional<SourceAddress> sourceAddressOf(const std::string& describe) {
    const auto endpoint = net::NetEndpoint::parse(describe);
    if (!endpoint || endpoint.value().scheme != net::EndpointScheme::Enet) return std::nullopt;
    const std::string& host = endpoint.value().host;

    std::array<std::uint8_t, 4> v4{};
    if (parseIpv4(host, v4)) return fromIpv4(v4);

    std::array<std::uint16_t, 8> g{};
    if (!parseIpv6(host, g)) return std::nullopt;
    const bool mapped = g[0] == 0 && g[1] == 0 && g[2] == 0 && g[3] == 0 && g[4] == 0 && g[5] == 0xFFFF;
    if (mapped) {
        return fromIpv4({static_cast<std::uint8_t>(g[6] >> 8), static_cast<std::uint8_t>(g[6] & 0xFF),
                         static_cast<std::uint8_t>(g[7] >> 8), static_cast<std::uint8_t>(g[7] & 0xFF)});
    }
    SourceAddress source;
    source.key = hex(g[0]) + ":" + hex(g[1]) + ":" + hex(g[2]) + ":" + hex(g[3]) + "::/64";
    source.display = hex(g[0]) + ":" + hex(g[1]) + ":" + hex(g[2]) + "::x";
    source.loopback = g[0] == 0 && g[1] == 0 && g[2] == 0 && g[3] == 0 && g[4] == 0 && g[5] == 0 && g[6] == 0 && g[7] == 1;
    return source;
}

}  // namespace wpiemu::relay
