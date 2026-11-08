#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/wire_codec.h"
#include "wpiemu/sdk/core/wire_fields.h"

using namespace wpiemu::sdk;

namespace {

struct TestDomain {};
using W = wire::Writer<TestDomain>;
using R = wire::Reader<TestDomain>;
using wire::Is;

enum class Mode : std::uint8_t { A = 0, B = 1, C = 2 };

struct Item {
    std::uint16_t id = 0;
    std::string name;
    bool operator==(const Item&) const = default;
};

// Base fields, then an appended tail an older peer does not send.
struct Message {
    std::uint32_t seq = 0;
    Mode mode = Mode::A;
    std::vector<Item> items;
    std::uint64_t tail = 7;
    std::string note = "default";
    bool operator==(const Message&) const = default;
};

constexpr std::size_t kMaxItems = 4;
constexpr std::size_t kItemMinBytes = 4;

template <class C, Is<Item> M>
void fields(C& c, M& i) {
    c.u16(i.id);
    c.str(i.name);
}

template <class C, Is<Message> M>
void fields(C& c, M& m) {
    c.u32(m.seq);
    c.enumeration(m.mode, 2, "invalid Mode");
    c.list(m.items, kMaxItems, kItemMinBytes);
    if (!c.more()) return;
    c.u64(m.tail);
    if (!c.more()) return;
    c.str(m.note);
}

std::vector<std::byte> bytes(std::initializer_list<int> v) {
    std::vector<std::byte> out;
    for (int b : v) out.push_back(static_cast<std::byte>(b));
    return out;
}

template <class T>
std::vector<std::byte> encode(const T& value, bool* bad = nullptr) {
    std::vector<std::byte> out;
    W w(out);
    fields(w, value);
    if (bad != nullptr) *bad = w.bad();
    return out;
}

template <class T>
bool decode(const std::vector<std::byte>& in, T& value, const char** what = nullptr) {
    wire::ReadState state;
    R r(in, &state);
    fields(r, value);
    if (what != nullptr) *what = state.what;
    return state.ok;
}

std::string whyFails(const std::vector<std::byte>& in) {
    Message m;
    const char* what = "";
    return decode(in, m, &what) ? std::string("ok") : std::string(what);
}

std::uint64_t readVarint(const std::vector<std::byte>& in, bool& ok, const char** what = nullptr) {
    wire::ReadState state;
    R r(in, &state);
    const std::uint64_t v = r.varint();
    ok = state.ok;
    if (what != nullptr) *what = state.what;
    return v;
}

std::vector<std::byte> varintBytes(std::uint64_t v) {
    std::vector<std::byte> out;
    W(out).varint(v);
    return out;
}

Message sample() {
    Message m;
    m.seq = 0xDEADBEEF;
    m.mode = Mode::C;
    m.items = {{1, "one"}, {65535, ""}, {3, "caf\xC3\xA9"}};
    m.tail = 0x0123456789ABCDEFULL;
    m.note = "hello";
    return m;
}

}  // namespace

TEST_CASE("wire codec: scalars round trip at their extremes and are little-endian") {
    std::vector<std::byte> out;
    W w(out);
    w.u8(0xFF);
    w.u16(0xBEEF);
    w.u32(0xDEADBEEF);
    w.u64(std::numeric_limits<std::uint64_t>::max());
    w.i16(std::numeric_limits<std::int16_t>::min());
    w.i32(std::numeric_limits<std::int32_t>::min());
    w.i64(std::numeric_limits<std::int64_t>::min());
    w.f32(-0.0f);
    w.f64(std::numeric_limits<double>::infinity());
    w.boolean(true);
    CHECK(out.size() == 1 + 2 + 4 + 8 + 2 + 4 + 8 + 4 + 8 + 1);
    CHECK(out[1] == std::byte{0xEF});
    CHECK(out[2] == std::byte{0xBE});

    wire::ReadState state;
    R r(out, &state);
    std::uint8_t u8 = 0;
    std::uint16_t u16 = 0;
    std::uint32_t u32 = 0;
    std::uint64_t u64 = 0;
    std::int16_t i16 = 0;
    std::int32_t i32 = 0;
    std::int64_t i64 = 0;
    float f32 = 1.f;
    double f64 = 0;
    bool b = false;
    r.u8(u8);
    r.u16(u16);
    r.u32(u32);
    r.u64(u64);
    r.i16(i16);
    r.i32(i32);
    r.i64(i64);
    r.f32(f32);
    r.f64(f64);
    r.boolean(b);
    CHECK(state.ok);
    CHECK_FALSE(r.more());
    CHECK(u8 == 0xFF);
    CHECK(u16 == 0xBEEF);
    CHECK(u32 == 0xDEADBEEF);
    CHECK(u64 == std::numeric_limits<std::uint64_t>::max());
    CHECK(i16 == std::numeric_limits<std::int16_t>::min());
    CHECK(i32 == std::numeric_limits<std::int32_t>::min());
    CHECK(i64 == std::numeric_limits<std::int64_t>::min());
    CHECK((f32 == 0.f && std::signbit(f32)));
    CHECK(std::isinf(f64));
    CHECK(b);
}

TEST_CASE("wire codec: varints use the shortest LEB128 form") {
    CHECK(varintBytes(0) == bytes({0x00}));
    CHECK(varintBytes(127) == bytes({0x7F}));
    CHECK(varintBytes(128) == bytes({0x80, 0x01}));
    CHECK(varintBytes(300) == bytes({0xAC, 0x02}));
    CHECK(varintBytes(16383) == bytes({0xFF, 0x7F}));
    CHECK(varintBytes(16384) == bytes({0x80, 0x80, 0x01}));
    CHECK(varintBytes(std::numeric_limits<std::uint64_t>::max()) ==
          bytes({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01}));
}

TEST_CASE("wire codec: varints round trip at every length boundary") {
    for (int bits = 0; bits < 64; ++bits) {
        for (const std::uint64_t base : {std::uint64_t{1} << bits, (std::uint64_t{1} << bits) - 1}) {
            bool ok = false;
            CHECK(readVarint(varintBytes(base), ok) == base);
            CHECK(ok);
        }
    }
    bool ok = false;
    CHECK(readVarint(varintBytes(std::numeric_limits<std::uint64_t>::max()), ok) ==
          std::numeric_limits<std::uint64_t>::max());
    CHECK(ok);
}

TEST_CASE("wire codec: malformed varints are rejected") {
    bool ok = true;
    const char* what = "";
    readVarint({}, ok, &what);
    CHECK_FALSE(ok);
    CHECK(std::string(what) == "truncated varint");

    ok = true;
    readVarint(bytes({0x80, 0x80}), ok, &what);
    CHECK_FALSE(ok);
    CHECK(std::string(what) == "truncated varint");

    // A tenth byte above 1 would shift bits past 64.
    ok = true;
    readVarint(bytes({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02}), ok, &what);
    CHECK_FALSE(ok);
    CHECK(std::string(what) == "varint overflow");

    ok = true;
    readVarint(bytes({0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00}), ok, &what);
    CHECK_FALSE(ok);
}

TEST_CASE("wire codec: strings round trip including empty, embedded NUL and UTF-8") {
    for (const std::string s : {std::string(), std::string("a"), std::string("a\0b", 3), std::string("caf\xC3\xA9"),
                                std::string(300, 'x')}) {
        std::vector<std::byte> out;
        W(out).str(s);
        wire::ReadState state;
        R r(out, &state);
        std::string back = "junk";
        r.str(back);
        CHECK(state.ok);
        CHECK(back == s);
        CHECK_FALSE(r.more());
    }
}

TEST_CASE("wire codec: a string beyond the protocol cap is refused on both sides") {
    std::vector<std::byte> out;
    W w(out);
    w.str(std::string(wire::kMaxStringBytes + 1, 'x'));
    CHECK(w.bad());
    CHECK(out.empty());

    std::vector<std::byte> ok;
    W w2(ok);
    w2.str(std::string(wire::kMaxStringBytes, 'x'));
    CHECK_FALSE(w2.bad());
    wire::ReadState s2;
    R r2(ok, &s2);
    std::string back;
    r2.str(back);
    CHECK(s2.ok);
    CHECK(back.size() == wire::kMaxStringBytes);

    wire::ReadState state;
    const std::vector<std::byte> hostile = varintBytes(wire::kMaxStringBytes + 1);
    R r(hostile, &state);
    r.str(back);
    CHECK_FALSE(state.ok);
}

TEST_CASE("wire codec: a per-field string cap is enforced when writing and before copying when reading") {
    std::vector<std::byte> out;
    W w(out);
    w.str(std::string(8, 'x'), 8);   // exactly at the field's own limit
    CHECK_FALSE(w.bad());

    std::vector<std::byte> over;
    W wOver(over);
    wOver.str(std::string(9, 'x'), 8);
    CHECK(wOver.bad());
    CHECK(over.empty());

    std::vector<std::byte> wide;   // legal for the protocol, over the field's cap
    W(wide).str(std::string(9, 'x'));
    wire::ReadState state;
    R r(wide, &state);
    std::string back = "keep";
    r.str(back, 8, "field too long");
    CHECK_FALSE(state.ok);
    CHECK(std::string(state.what) == "field too long");
    CHECK(back == "keep");   // nothing was copied

    wire::ReadState okState;
    R rOk(out, &okState);
    rOk.str(back, 8);
    CHECK(okState.ok);
    CHECK(back == "xxxxxxxx");
}

TEST_CASE("wire codec: Writer fail marks the output unencodable like a bound violation") {
    std::vector<std::byte> out;
    W w(out);
    CHECK_FALSE(w.bad());
    w.fail("reason");
    CHECK(w.bad());
}

TEST_CASE("wire codec: a string longer than the bytes left is rejected") {
    std::vector<std::byte> frame = varintBytes(10);
    frame.push_back(std::byte{'a'});
    wire::ReadState state;
    R r(frame, &state);
    std::string s;
    r.str(s);
    CHECK_FALSE(state.ok);
    CHECK(std::string(state.what) == "string length out of range");
}

TEST_CASE("wire codec: a message round trips and re-encodes byte for byte") {
    const Message m = sample();
    bool bad = true;
    const std::vector<std::byte> frame = encode(m, &bad);
    CHECK_FALSE(bad);
    Message back;
    REQUIRE(decode(frame, back));
    CHECK(back == m);
    CHECK(encode(back) == frame);
}

TEST_CASE("wire codec: truncating a frame is always rejected before the optional tail") {
    const Message m = sample();
    const std::vector<std::byte> frame = encode(m);
    const std::size_t baseBytes = frame.size() - 8 - 1 - 5;  // minus tail (u64) and note ("hello")
    for (std::size_t n = 0; n < baseBytes; ++n) {
        Message out;
        CHECK_FALSE(decode(std::vector<std::byte>(frame.begin(), frame.begin() + static_cast<std::ptrdiff_t>(n)), out));
    }
}

TEST_CASE("wire codec: an older peer's frame without the appended tail keeps the defaults") {
    Message old;
    old.seq = 9;
    old.mode = Mode::B;
    std::vector<std::byte> frame = encode(old);
    frame.resize(frame.size() - 8 - 1 - 7);  // drop tail (u64) and note ("default")
    Message back;
    back.tail = 0;
    back.note = "";
    CHECK(decode(frame, back));
    CHECK(back.seq == 9);
    CHECK(back.mode == Mode::B);
    CHECK(back.tail == 0);  // untouched: the reader stops where the data ends

    Message fresh;
    CHECK(decode(frame, fresh));
    CHECK(fresh.tail == 7);
    CHECK(fresh.note == "default");
}

TEST_CASE("wire codec: a newer peer's extra trailing bytes are ignored") {
    std::vector<std::byte> frame = encode(sample());
    for (int i = 0; i < 40; ++i) frame.push_back(static_cast<std::byte>(i));
    Message back;
    CHECK(decode(frame, back));
    CHECK(back == sample());
}

TEST_CASE("wire codec: bounded enums reject values above the bound") {
    std::vector<std::byte> frame = encode(sample());
    frame[4] = std::byte{3};
    CHECK(whyFails(frame) == "invalid Mode");
    frame[4] = std::byte{2};
    CHECK(whyFails(frame) == "ok");
}

TEST_CASE("wire codec: an unbounded enum accepts any byte and a writer ignores the bound") {
    std::vector<std::byte> out;
    W w(out);
    w.enumeration(Mode::C, 1, "never used on write");
    CHECK(out == bytes({2}));
    out = bytes({200});
    wire::ReadState state;
    R r(out, &state);
    Mode m = Mode::A;
    r.enumeration(m);
    CHECK(state.ok);
    CHECK(static_cast<std::uint8_t>(m) == 200);
}

TEST_CASE("wire codec: a list count above the cap is rejected and an oversized list is not written") {
    std::vector<std::byte> frame = encode(sample());
    frame[5] = static_cast<std::byte>(kMaxItems + 1);
    CHECK(whyFails(frame) == "element count exceeds limit");

    Message big;
    big.items.assign(kMaxItems + 1, Item{1, "x"});
    bool bad = false;
    encode(big, &bad);
    CHECK(bad);

    big.items.resize(kMaxItems);
    encode(big, &bad);
    CHECK_FALSE(bad);
}

TEST_CASE("wire codec: a list count larger than the bytes left is rejected without allocating") {
    Message m;
    std::vector<std::byte> frame = encode(m);
    frame[5] = static_cast<std::byte>(kMaxItems);  // claims 4 items; nothing but the optional tail follows
    frame.resize(6);
    CHECK(whyFails(frame) == "element count exceeds frame");

    std::vector<std::byte> hostile = varintBytes(std::numeric_limits<std::uint64_t>::max());
    std::vector<std::uint8_t> numbers;
    wire::ReadState state;
    R r(hostile, &state);
    r.list(numbers, std::size_t{1} << 40, 1);
    CHECK_FALSE(state.ok);
    CHECK(numbers.empty());
}

TEST_CASE("wire codec: lists of scalars and geometry round trip") {
    std::vector<std::byte> out;
    W w(out);
    const std::vector<float> axes{0.f, -1.f, 0.5f};
    const std::vector<std::int16_t> povs{-1, 0, 315};
    const std::vector<std::uint64_t> ids{0, std::numeric_limits<std::uint64_t>::max()};
    const std::vector<Vec3> points{{1.f, 2.f, 3.f}, {-4.f, 5.f, 6.f}};
    w.list(axes, 8, 4);
    w.list(povs, 8, 2);
    w.list(ids, 8, 8);
    w.list(points, 8, 12);
    CHECK_FALSE(w.bad());

    wire::ReadState state;
    R r(out, &state);
    std::vector<float> axes2;
    std::vector<std::int16_t> povs2;
    std::vector<std::uint64_t> ids2;
    std::vector<Vec3> points2;
    r.list(axes2, 8, 4);
    r.list(povs2, 8, 2);
    r.list(ids2, 8, 8);
    r.list(points2, 8, 12);
    CHECK(state.ok);
    CHECK(axes2 == axes);
    CHECK(povs2 == povs);
    CHECK(ids2 == ids);
    CHECK(points2 == points);
}

TEST_CASE("wire codec: Vec3, Quat and Pose3 use the pinned layout") {
    const Pose3 pose({1.f, 2.f, 3.f}, {0.f, 0.f, 0.5f, 1.f});
    std::vector<std::byte> out;
    W w(out);
    fields(w, pose);
    // 7 x f32: x y z then qx qy qz qw.
    CHECK(out == bytes({0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x80, 0x3F}));
    wire::ReadState state;
    R r(out, &state);
    Pose3 back;
    fields(r, back);
    CHECK(state.ok);
    CHECK(back.position == pose.position);
    CHECK(back.rotation == pose.rotation);
}

TEST_CASE("wire codec: a record skips the fields an older reader does not know") {
    std::vector<std::byte> out;
    W w(out);
    w.record([&] {
        w.u32(11);
        w.str("newer field");
        w.u64(99);
    });
    w.u16(0xABCD);  // follows the record

    wire::ReadState state;
    R r(out, &state);
    std::uint32_t first = 0;
    std::uint16_t after = 0;
    r.record([&] { r.u32(first); });
    r.u16(after);
    CHECK(state.ok);
    CHECK(first == 11);
    CHECK(after == 0xABCD);
}

TEST_CASE("wire codec: a record shorter than the reader expects leaves the missing fields alone") {
    std::vector<std::byte> out;
    W w(out);
    w.record([&] { w.u32(5); });
    w.u8(77);

    wire::ReadState state;
    R r(out, &state);
    std::uint32_t a = 0;
    std::uint32_t b = 42;
    std::uint8_t after = 0;
    r.record([&] {
        r.u32(a);
        if (r.more()) r.u32(b);
    });
    r.u8(after);
    CHECK(state.ok);
    CHECK(a == 5);
    CHECK(b == 42);
    CHECK(after == 77);
}

TEST_CASE("wire codec: a record body cannot read past its own length") {
    std::vector<std::byte> out;
    W w(out);
    w.record([&] { w.u8(1); });
    w.u64(0x1122334455667788ULL);  // bytes right after the record must not be consumed by it

    wire::ReadState state;
    R r(out, &state);
    std::uint32_t v = 0;
    r.record([&] { r.u32(v); });
    CHECK_FALSE(state.ok);
    CHECK(std::string(state.what) == "truncated field");
}

TEST_CASE("wire codec: a record length beyond the frame is rejected and one over 64 KiB is not written") {
    wire::ReadState state;
    const std::vector<std::byte> frame = bytes({0x10, 0x00, 0x01});
    R r(frame, &state);
    r.record([&] { FAIL("the body must not run"); });
    CHECK_FALSE(state.ok);
    CHECK(std::string(state.what) == "record length out of range");

    std::vector<std::byte> out;
    W w(out);
    w.record([&] {
        for (int i = 0; i < 9000; ++i) w.u64(0);
    });
    CHECK(w.bad());
}

TEST_CASE("wire codec: the first failure sticks and later reads return zeros") {
    const std::vector<std::byte> frame = bytes({0x01});
    wire::ReadState state;
    R r(frame, &state);
    std::uint32_t a = 5;
    std::uint8_t b = 5;
    r.u32(a);
    CHECK_FALSE(state.ok);
    r.fail("another reason");
    r.u8(b);
    CHECK(a == 0);
    CHECK(b == 0);
    CHECK(std::string(state.what) == "truncated field");
    CHECK_FALSE(r.more());
}

TEST_CASE("wire codec: a writer reports more and appends to existing bytes") {
    std::vector<std::byte> out = bytes({0xAA});
    W w(out);
    CHECK(w.more());
    w.u8(0xBB);
    CHECK(out == bytes({0xAA, 0xBB}));
}
