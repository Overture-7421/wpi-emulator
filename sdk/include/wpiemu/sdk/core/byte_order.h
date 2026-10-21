#pragma once

// Little-endian encoding shared by the wire formats (IPC frames, PhotonVision packets).

#include <bit>
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

// Typed scalar appends (two's complement integers, IEEE-754 floats, booleans as one byte) onto a byte vector.
// Only the scalar encodings are shared: each wire format adds its own framing, lengths and strings.
template <typename Byte>
class LittleEndianWriter {
public:
    explicit LittleEndianWriter(std::vector<Byte>& out) : out_(out) {}

    void u8(std::uint8_t v) { out_.push_back(static_cast<Byte>(v)); }
    void u16(std::uint16_t v) { appendLittleEndian(out_, v, 2); }
    void u32(std::uint32_t v) { appendLittleEndian(out_, v, 4); }
    void u64(std::uint64_t v) { appendLittleEndian(out_, v, 8); }
    void i16(std::int16_t v) { u16(static_cast<std::uint16_t>(v)); }
    void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
    void i64(std::int64_t v) { u64(static_cast<std::uint64_t>(v)); }
    void f32(float v) { u32(std::bit_cast<std::uint32_t>(v)); }
    void f64(double v) { u64(std::bit_cast<std::uint64_t>(v)); }
    void boolean(bool v) { u8(v ? 1 : 0); }

protected:
    std::vector<Byte>& out_;
};

}  // namespace wpiemu::sdk
