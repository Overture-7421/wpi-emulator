#include "wpiemu/net/invite.h"

#include <cstring>

#include "monocypher.h"
#include "wpiemu/net/os_random.h"

namespace wpiemu::net {

namespace {

constexpr std::size_t kHashBytes = 32;

std::array<std::uint8_t, kHashBytes> derive(std::string_view domain, const InviteSecret::Bytes& secret) {
    std::array<std::uint8_t, kHashBytes> out{};
    crypto_blake2b_keyed(out.data(), out.size(), reinterpret_cast<const std::uint8_t*>(domain.data()), domain.size(),
                         secret.data(), secret.size());
    return out;
}

// The 5-bit value of symbol `index` in the MSB-first bit stream of `bytes`.
unsigned symbolAt(std::span<const std::uint8_t> bytes, std::size_t index) {
    unsigned value = 0;
    for (std::size_t bit = index * 5; bit < index * 5 + 5; ++bit)
        value = (value << 1) | ((bytes[bit / 8] >> (7 - bit % 8)) & 1u);
    return value;
}

int symbolValue(char c) {
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
    switch (c) {
        case 'O': c = '0'; break;
        case 'I':
        case 'L': c = '1'; break;
        default: break;
    }
    for (int i = 0; sdk::match::relay::kRoomIdAlphabet[i] != '\0'; ++i)
        if (sdk::match::relay::kRoomIdAlphabet[i] == c) return i;
    return -1;
}

}  // namespace

sdk::Result<InviteSecret> InviteSecret::fromBytes(const Bytes& bytes) {
    if ((bytes[kBytes - 1] & 0x0f) != 0)
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, "an invite secret has 100 bits; the last 4 must be zero");
    return InviteSecret(bytes);
}

InviteSecret::InviteSecret(const InviteSecret& other) noexcept : bytes_(other.bytes_) {}

InviteSecret::InviteSecret(InviteSecret&& other) noexcept : bytes_(other.bytes_) { other.wipe(); }

InviteSecret& InviteSecret::operator=(const InviteSecret& other) noexcept {
    if (this != &other) bytes_ = other.bytes_;
    return *this;
}

InviteSecret& InviteSecret::operator=(InviteSecret&& other) noexcept {
    if (this != &other) {
        bytes_ = other.bytes_;
        other.wipe();
    }
    return *this;
}

InviteSecret::~InviteSecret() { wipe(); }

void InviteSecret::wipe() { crypto_wipe(bytes_.data(), bytes_.size()); }

bool InviteSecret::isZero() const {
    std::uint8_t any = 0;
    for (const std::uint8_t b : bytes_) any = static_cast<std::uint8_t>(any | b);
    return any == 0;
}

bool InviteSecret::operator==(const InviteSecret& other) const { return constantTimeEqual(bytes_, other.bytes_); }

sdk::Result<InviteSecret> generateInvite() { return generateInvite(fillRandom, sdk::match::relay::kRelayPingRoomId); }

sdk::Result<InviteSecret> generateInvite(const std::function<sdk::Status(std::span<std::byte>)>& fill,
                                         std::string_view reservedRoomId) {
    constexpr int kMaxDraws = 8;
    for (int draw = 0; draw < kMaxDraws; ++draw) {
        InviteSecret::Bytes bytes{};
        if (sdk::Status s = fill(std::as_writable_bytes(std::span(bytes))); !s) {
            crypto_wipe(bytes.data(), bytes.size());
            return s.error();
        }
        bytes[InviteSecret::kBytes - 1] = static_cast<std::uint8_t>(bytes[InviteSecret::kBytes - 1] & 0xf0);
        sdk::Result<InviteSecret> secret = InviteSecret::fromBytes(bytes);
        crypto_wipe(bytes.data(), bytes.size());
        if (secret && roomId(secret.value()) != reservedRoomId) return secret;
    }
    return sdk::makeError(sdk::ErrorCode::IoError, "the random source keeps producing the reserved room id");
}

std::string inviteText(const InviteSecret& secret) {
    std::string text;
    text.reserve(InviteSecret::kTextChars + InviteSecret::kTextChars / InviteSecret::kGroupChars - 1);
    for (std::size_t i = 0; i < InviteSecret::kTextChars; ++i) {
        if (i != 0 && i % InviteSecret::kGroupChars == 0) text.push_back('-');
        text.push_back(sdk::match::relay::kRoomIdAlphabet[symbolAt(secret.bytes(), i)]);
    }
    return text;
}

sdk::Result<InviteSecret> parseInvite(std::string_view text) {
    InviteSecret::Bytes bytes{};
    std::size_t count = 0;
    for (const char c : text) {
        if (c == '-' || c == ' ') continue;
        const int value = symbolValue(c);
        if (value < 0) {
            crypto_wipe(bytes.data(), bytes.size());
            return sdk::makeError(sdk::ErrorCode::ParseError,
                                  std::string("invite code contains an invalid character '") + c +
                                      "'; use the digits and letters shown in the invite (no U)");
        }
        if (count < InviteSecret::kTextChars) {
            for (int bit = 4; bit >= 0; --bit) {
                const std::size_t pos = count * 5 + static_cast<std::size_t>(4 - bit);
                if ((value >> bit) & 1) bytes[pos / 8] = static_cast<std::uint8_t>(bytes[pos / 8] | (0x80u >> (pos % 8)));
            }
        }
        ++count;
    }
    if (count != InviteSecret::kTextChars) {
        crypto_wipe(bytes.data(), bytes.size());
        return sdk::makeError(sdk::ErrorCode::ParseError,
                              "an invite code has " + std::to_string(InviteSecret::kTextChars) +
                                  " characters (XXXXX-XXXXX-XXXXX-XXXXX), got " + std::to_string(count));
    }
    sdk::Result<InviteSecret> secret = InviteSecret::fromBytes(bytes);
    crypto_wipe(bytes.data(), bytes.size());
    return secret;
}

std::string roomId(const InviteSecret& secret) {
    const auto hash = derive(kInviteRoomDomain, secret.bytes());
    std::string id;
    id.reserve(sdk::match::relay::kRelayRoomIdChars);
    for (std::size_t i = 0; i < sdk::match::relay::kRelayRoomIdChars; ++i)
        id.push_back(sdk::match::relay::kRoomIdAlphabet[symbolAt(hash, i)]);
    return id;
}

sdk::match::relay::HostAuth hostAuth(const InviteSecret& secret) {
    static_assert(sdk::match::relay::kRelayHostAuthBytes == kHashBytes);
    return derive(kInviteHostDomain, secret.bytes());
}

std::array<std::uint8_t, 32> channelKey(const InviteSecret& secret) { return derive(kInviteChannelDomain, secret.bytes()); }

bool constantTimeEqual(std::span<const std::uint8_t> a, std::span<const std::uint8_t> b) {
    if (a.size() != b.size()) return false;
    std::uint8_t diff = 0;
    for (std::size_t i = 0; i < a.size(); ++i) diff = static_cast<std::uint8_t>(diff | (a[i] ^ b[i]));
    return diff == 0;
}

}  // namespace wpiemu::net
