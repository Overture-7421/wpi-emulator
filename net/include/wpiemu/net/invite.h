#pragma once

// Invite secrets for practice matches (docs: "Transport encryption").
//
// The host generates a random 100-bit secret per hosting session and shows it as `XXXXX-XXXXX-XXXXX-XXXXX` in Crockford
// base32 (alphabet 0123456789ABCDEFGHJKMNPQRSTVWXYZ, the same one the relay uses for room ids). Everything else is derived
// from it with BLAKE2b, using a different fixed domain string as the BLAKE2b key for each purpose, so the values are
// independent and knowing one (the relay sees roomId and hostAuth) reveals nothing usable about the secret or the others:
//
//   roomId     first 8 base32 chars of BLAKE2b-256(key = "wpiemu-room", message = secret bytes)   public lookup name
//   hostAuth   BLAKE2b-256(key = "wpiemu-host", message = secret bytes)                           proves the host to the relay
//   channelKey BLAKE2b-256(key = "wpiemu-chan", message = secret bytes)                           keys the SecureTransport handshake
//
// `secret bytes` are the 13 packed bytes of the 100 bits, most significant bit first, the last 4 bits zero. The type wipes
// its memory on destruction.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::net {

inline constexpr std::string_view kInviteRoomDomain = "wpiemu-room";
inline constexpr std::string_view kInviteHostDomain = "wpiemu-host";
inline constexpr std::string_view kInviteChannelDomain = "wpiemu-chan";

class InviteSecret {
public:
    static constexpr std::size_t kBits = 100;
    static constexpr std::size_t kBytes = 13;  // ceil(kBits / 8); the last 4 bits are always zero
    static constexpr std::size_t kTextChars = 20;
    static constexpr std::size_t kGroupChars = 5;
    using Bytes = std::array<std::uint8_t, kBytes>;

    // InvalidArgument unless the 4 unused trailing bits are zero.
    static sdk::Result<InviteSecret> fromBytes(const Bytes& bytes);

    InviteSecret(const InviteSecret& other) noexcept;
    InviteSecret(InviteSecret&& other) noexcept;
    InviteSecret& operator=(const InviteSecret& other) noexcept;
    InviteSecret& operator=(InviteSecret&& other) noexcept;
    ~InviteSecret();

    const Bytes& bytes() const { return bytes_; }
    bool isZero() const;

    // Constant time in the content.
    bool operator==(const InviteSecret& other) const;
    bool operator!=(const InviteSecret& other) const { return !(*this == other); }

    // Overwrites the secret with zeros (the destructor does this too).
    void wipe();

private:
    explicit InviteSecret(const Bytes& bytes) : bytes_(bytes) {}

    Bytes bytes_{};
};

// A fresh secret from the operating system random source. IoError if it is unavailable (never a weak fallback).
sdk::Result<InviteSecret> generateInvite();

// "XXXXX-XXXXX-XXXXX-XXXXX", upper case.
std::string inviteText(const InviteSecret& secret);

// Accepts what a person types or pastes: any case, dashes and spaces anywhere (or nowhere), O read as 0 and I or L read as
// 1. ParseError with the reason (wrong length, the first invalid character) otherwise.
sdk::Result<InviteSecret> parseInvite(std::string_view text);

// kRelayRoomIdChars characters of sdk::match::relay::kRoomIdAlphabet; satisfies relay::isValidRoomId.
std::string roomId(const InviteSecret& secret);
sdk::match::relay::HostAuth hostAuth(const InviteSecret& secret);
// Key of the SecureTransport handshake (32 bytes). The caller wipes it when done.
std::array<std::uint8_t, 32> channelKey(const InviteSecret& secret);

// Compares without an early exit on the first difference (the lengths are not secret).
bool constantTimeEqual(std::span<const std::uint8_t> a, std::span<const std::uint8_t> b);

}  // namespace wpiemu::net
