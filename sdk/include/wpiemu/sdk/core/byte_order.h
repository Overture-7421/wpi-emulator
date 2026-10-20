#pragma once

// Little-endian integer encoding shared by the wire formats (IPC frames, PhotonVision packets).

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace wpiemu::sdk {

// Appends the low `bytes` (1..8) bytes of `value`, least significant first. `Byte` is std::byte or std::uint8_t.
template <typename Byte>
void appendLittleEndian(std::vector<Byte>& out, std::uint64_t value, int bytes) {
    for (int i = 0; i < bytes; ++i) out.push_back(static_cast<Byte>((value >> (8 * i)) & 0xFF));
}

// The `bytes` (1..8) little-endian bytes at `data[pos]`; the caller has checked that they exist.
inline std::uint64_t readLittleEndian(std::span<const std::byte> data, std::size_t pos, int bytes) {
    std::uint64_t value = 0;
    for (int i = 0; i < bytes; ++i)
        value |= static_cast<std::uint64_t>(static_cast<std::uint8_t>(data[pos + static_cast<std::size_t>(i)])) << (8 * i);
    return value;
}

}  // namespace wpiemu::sdk
