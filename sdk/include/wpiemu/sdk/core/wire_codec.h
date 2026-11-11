#pragma once

// Symmetric binary Writer/Reader shared by every wire format of the product (the HALSim extension protocol in ipc/, the
// match protocol, the relay envelope).
//
// The field order of a struct is written ONCE, as a `fields(codec, value)` function. The Writer and the Reader offer
// the same operations (a writer takes the value, a reader fills it in), so one description encodes and decodes and the
// two cannot drift apart:
//
//     template <class C, wire::Is<Ping> M> void fields(C& c, M& m) { c.u64(m.sequence); c.str(m.note); }
//
// Layout rules (docs/reference/extension-ipc.md): little-endian scalars, LEB128 varints for string and list lengths,
// bounded enums, and length-prefixed records that can grow fields later (an older reader skips what it does not know).
// A message that gains fields appends them at the end; `if (!c.more()) return;` ends a message that an older peer sent
// without them. Decoding never trusts a length: every count is checked against a cap and against the bytes left.
//
// `Domain` only names the namespace where the codec's own `fields` functions live: `list` calls `fields(*this, item)`
// found by argument-dependent lookup, and a template argument is an associated namespace. Instantiate with a tag type
// declared next to those functions (`using Writer = wire::Writer<Tag>;`).

#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

#include "wpiemu/sdk/core/byte_order.h"

namespace wpiemu::sdk::wire {

// Longest string any wire format carries; bounds the memory a hostile frame can make a decoder allocate.
inline constexpr std::size_t kMaxStringBytes = std::size_t{1} << 20;

// T, possibly const: the writer visits a const message, the reader a mutable one.
template <class T, class U>
concept Is = std::same_as<std::remove_const_t<T>, U>;

template <class Domain = void>
class Writer : public LittleEndianWriter<std::byte> {
public:
    explicit Writer(std::vector<std::byte>& out) : LittleEndianWriter(out) {}

    // True once a value did not fit the wire limits; the caller discards the output.
    bool bad() const { return bad_; }
    // Marks the output unencodable (the writer's counterpart of Reader::fail); `what` documents the reason at the call site.
    void fail(const char* /*what*/) { bad_ = true; }
    // A writer always has more: fields after an `if (!c.more()) return;` are the optional appended tail.
    constexpr bool more() const { return true; }

    // The bound only matters when reading.
    template <class E>
    void enumeration(E v, std::uint8_t /*maxValue*/ = 0xFF, const char* /*what*/ = "") {
        u8(static_cast<std::uint8_t>(v));
    }

    // `cap` is the field's own limit (never above kMaxStringBytes); a longer string cannot be encoded. `what` only names the
    // field on the reading side.
    void str(const std::string& s, std::size_t cap = kMaxStringBytes, const char* /*what*/ = "") {
        if (s.size() > cap || s.size() > kMaxStringBytes) {
            bad_ = true;
            return;
        }
        varint(s.size());
        const auto* p = reinterpret_cast<const std::byte*>(s.data());
        out_.insert(out_.end(), p, p + s.size());
    }

    // Counted list, each element written by its own `fields`. The cap and the minimum element size are the decoder's
    // limits: a list it would refuse is not worth sending.
    template <class Items>
    void list(const Items& items, std::size_t cap, std::size_t /*minElementBytes*/) {
        if (items.size() > cap) {
            bad_ = true;
            return;
        }
        varint(items.size());
        for (const auto& item : items) fields(*this, item);
    }

    // u16 byte-length prefix, so the record can grow fields later and older readers skip them.
    template <class Body>
    void record(Body body) {
        const std::size_t at = out_.size();
        u16(0);
        body();
        const std::size_t len = out_.size() - at - 2;
        if (len > 0xFFFF) {
            bad_ = true;
            return;
        }
        out_[at] = static_cast<std::byte>(len & 0xFF);
        out_[at + 1] = static_cast<std::byte>((len >> 8) & 0xFF);
    }

    void varint(std::uint64_t v) {
        while (v >= 0x80) {
            u8(static_cast<std::uint8_t>(v | 0x80));
            v >>= 7;
        }
        u8(static_cast<std::uint8_t>(v));
    }

private:
    bool bad_ = false;
};

// Shared by every Reader of one decode: the first failure sticks and later reads return zeros.
struct ReadState {
    bool ok = true;
    const char* what = "";
};

template <class Domain = void>
class Reader {
public:
    Reader(std::span<const std::byte> data, ReadState* state) : d_(data), s_(state) {}

    bool ok() const { return s_->ok; }
    // True while there is unread data: absent trailing (appended) fields mean an older peer, so they keep their defaults.
    bool more() const { return s_->ok && pos_ < d_.size(); }
    void fail(const char* what) {
        if (s_->ok) {
            s_->ok = false;
            s_->what = what;
        }
    }

    void u8(std::uint8_t& v) { v = static_cast<std::uint8_t>(le(1)); }
    void u16(std::uint16_t& v) { v = static_cast<std::uint16_t>(le(2)); }
    void u32(std::uint32_t& v) { v = static_cast<std::uint32_t>(le(4)); }
    void u64(std::uint64_t& v) { v = le(8); }
    void i16(std::int16_t& v) { v = static_cast<std::int16_t>(le(2)); }
    void i32(std::int32_t& v) { v = static_cast<std::int32_t>(le(4)); }
    void i64(std::int64_t& v) { v = static_cast<std::int64_t>(le(8)); }
    void f32(float& v) { v = std::bit_cast<float>(static_cast<std::uint32_t>(le(4))); }
    void f64(double& v) { v = std::bit_cast<double>(le(8)); }
    void boolean(bool& v) { v = le(1) != 0; }

    // Unbounded, any value is accepted (the receiver answers unknown commands itself); with a bound, a larger
    // value is a malformed frame.
    template <class E>
    void enumeration(E& v, std::uint8_t maxValue = 0xFF, const char* what = "") {
        std::uint8_t raw = 0;
        u8(raw);
        if (raw > maxValue) {
            fail(what);
            return;
        }
        v = static_cast<E>(raw);
    }

    // `cap` is the field's own limit (never above kMaxStringBytes); a longer declared length is rejected before any copy, and
    // `what` names the field in the error.
    void str(std::string& s, std::size_t cap = kMaxStringBytes, const char* what = "string length out of range") {
        const std::uint64_t n = varint();
        if (!s_->ok) return;
        if (n > kMaxStringBytes || n > remaining()) {
            fail("string length out of range");
            return;
        }
        if (n > cap) {
            fail(what);
            return;
        }
        s.assign(reinterpret_cast<const char*>(d_.data() + pos_), static_cast<std::size_t>(n));
        pos_ += static_cast<std::size_t>(n);
    }

    // Counted list; the count is checked against the cap and against the bytes left (`minElementBytes` each).
    template <class Items>
    void list(Items& items, std::size_t cap, std::size_t minElementBytes) {
        items.resize(count(minElementBytes, cap));
        for (auto& item : items) {
            if (!ok()) break;
            fields(*this, item);
        }
    }

    // The body reads inside the record; this reader then continues after it, whatever the body consumed.
    template <class Body>
    void record(Body body) {
        std::uint16_t n = 0;
        u16(n);
        if (!s_->ok) return;
        if (n > remaining()) {
            fail("record length out of range");
            return;
        }
        const std::span<const std::byte> outer = d_;
        const std::size_t resume = pos_ + n;
        d_ = outer.subspan(pos_, n);
        pos_ = 0;
        body();
        d_ = outer;
        pos_ = resume;
    }

    std::uint64_t varint() {
        std::uint64_t v = 0;
        for (int i = 0; i < 10; ++i) {
            if (remaining() == 0) {
                fail("truncated varint");
                return 0;
            }
            const auto b = static_cast<std::uint8_t>(d_[pos_++]);
            if (i == 9 && b > 1) {
                fail("varint overflow");
                return 0;
            }
            v |= static_cast<std::uint64_t>(b & 0x7F) << (7 * i);
            if ((b & 0x80) == 0) return v;
        }
        fail("varint too long");
        return 0;
    }

private:
    std::size_t remaining() const { return d_.size() - pos_; }

    // Element count of a vector whose elements occupy at least `minElemBytes` on the wire.
    std::size_t count(std::size_t minElemBytes, std::size_t cap) {
        const std::uint64_t n = varint();
        if (!s_->ok) return 0;
        if (n > cap) {
            fail("element count exceeds limit");
            return 0;
        }
        if (minElemBytes > 0 && n > remaining() / minElemBytes) {
            fail("element count exceeds frame");
            return 0;
        }
        return static_cast<std::size_t>(n);
    }

    std::uint64_t le(int bytes) {
        if (!s_->ok) return 0;
        if (remaining() < static_cast<std::size_t>(bytes)) {
            fail("truncated field");
            return 0;
        }
        const std::uint64_t v = readLittleEndian(d_, pos_, bytes);
        pos_ += static_cast<std::size_t>(bytes);
        return v;
    }

    std::span<const std::byte> d_;
    std::size_t pos_ = 0;
    ReadState* s_;
};

}  // namespace wpiemu::sdk::wire
