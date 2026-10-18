#include "wpiemu/bridges/vision/photon_md5.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace wpiemu::bridges::vision {

namespace {

constexpr std::array<std::uint32_t, 64> kShifts = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                                                   5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20,
                                                   4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                                                   6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};

// K[i] = floor(2^32 * |sin(i + 1)|), the definition in RFC 1321 section 3.4.
const std::array<std::uint32_t, 64>& constants() {
    static const std::array<std::uint32_t, 64> table = [] {
        std::array<std::uint32_t, 64> k{};
        for (std::size_t i = 0; i < k.size(); ++i) {
            k[i] = static_cast<std::uint32_t>(4294967296.0 * std::fabs(std::sin(static_cast<double>(i + 1))));
        }
        return k;
    }();
    return table;
}

constexpr std::uint32_t rotateLeft(std::uint32_t value, std::uint32_t bits) {
    return (value << bits) | (value >> (32U - bits));
}

struct State {
    std::uint32_t a = 0x67452301U;
    std::uint32_t b = 0xefcdab89U;
    std::uint32_t c = 0x98badcfeU;
    std::uint32_t d = 0x10325476U;
};

void processBlock(State& state, const unsigned char* block) {
    std::array<std::uint32_t, 16> words{};
    for (std::size_t i = 0; i < words.size(); ++i) {
        words[i] = static_cast<std::uint32_t>(block[4 * i]) | (static_cast<std::uint32_t>(block[4 * i + 1]) << 8U) |
                   (static_cast<std::uint32_t>(block[4 * i + 2]) << 16U) |
                   (static_cast<std::uint32_t>(block[4 * i + 3]) << 24U);
    }

    const auto& k = constants();
    std::uint32_t a = state.a;
    std::uint32_t b = state.b;
    std::uint32_t c = state.c;
    std::uint32_t d = state.d;
    for (std::uint32_t i = 0; i < 64; ++i) {
        std::uint32_t mix = 0;
        std::uint32_t index = 0;
        if (i < 16) {
            mix = (b & c) | (~b & d);
            index = i;
        } else if (i < 32) {
            mix = (d & b) | (~d & c);
            index = (5 * i + 1) % 16;
        } else if (i < 48) {
            mix = b ^ c ^ d;
            index = (3 * i + 5) % 16;
        } else {
            mix = c ^ (b | ~d);
            index = (7 * i) % 16;
        }
        const std::uint32_t next = b + rotateLeft(a + mix + k[i] + words[index], kShifts[i]);
        a = d;
        d = c;
        c = b;
        b = next;
    }
    state.a += a;
    state.b += b;
    state.c += c;
    state.d += d;
}

}  // namespace

std::string md5Hex(std::string_view data) {
    State state;
    const auto* bytes = reinterpret_cast<const unsigned char*>(data.data());
    std::size_t offset = 0;
    for (; offset + 64 <= data.size(); offset += 64) processBlock(state, bytes + offset);

    // The tail: the remaining bytes, 0x80, zeros up to 56 mod 64, then the bit length as 64 little-endian bits.
    std::array<unsigned char, 128> tail{};
    const std::size_t rest = data.size() - offset;
    for (std::size_t i = 0; i < rest; ++i) tail[i] = bytes[offset + i];
    tail[rest] = 0x80;
    const std::size_t tailSize = rest < 56 ? 64 : 128;
    const std::uint64_t bitLength = static_cast<std::uint64_t>(data.size()) * 8U;
    for (std::size_t i = 0; i < 8; ++i) tail[tailSize - 8 + i] = static_cast<unsigned char>(bitLength >> (8U * i));
    for (std::size_t block = 0; block < tailSize; block += 64) processBlock(state, tail.data() + block);

    static constexpr char kDigits[] = "0123456789abcdef";
    std::string hex;
    hex.reserve(32);
    for (const std::uint32_t word : {state.a, state.b, state.c, state.d}) {
        for (std::uint32_t i = 0; i < 4; ++i) {
            const auto byte = static_cast<unsigned>((word >> (8U * i)) & 0xffU);
            hex.push_back(kDigits[byte >> 4U]);
            hex.push_back(kDigits[byte & 0xfU]);
        }
    }
    return hex;
}

}  // namespace wpiemu::bridges::vision
