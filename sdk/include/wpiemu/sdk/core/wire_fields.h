#pragma once

// `fields` functions of the value types every wire format shares, for the Writer/Reader of wire_codec.h: fixed-width
// scalars (so a `list` of them works) and the geometry types. Layout is the one the HALSim protocol has always used
// (docs/reference/extension-ipc.md): Vec3 = 3 x f32, Quat = x y z w as f32, Pose3 = position then rotation (7 x f32).

#include <cstdint>
#include <type_traits>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/wire_codec.h"

namespace wpiemu::sdk::wire {

template <class T>
concept FixedScalar = Is<T, bool> || Is<T, std::uint8_t> || Is<T, std::uint16_t> || Is<T, std::uint32_t> ||
                      Is<T, std::uint64_t> || Is<T, std::int16_t> || Is<T, std::int32_t> || Is<T, std::int64_t> ||
                      Is<T, float> || Is<T, double>;

template <class C, FixedScalar M>
void fields(C& c, M& v) {
    if constexpr (Is<M, bool>) c.boolean(v);
    else if constexpr (Is<M, std::uint8_t>) c.u8(v);
    else if constexpr (Is<M, std::uint16_t>) c.u16(v);
    else if constexpr (Is<M, std::uint32_t>) c.u32(v);
    else if constexpr (Is<M, std::uint64_t>) c.u64(v);
    else if constexpr (Is<M, std::int16_t>) c.i16(v);
    else if constexpr (Is<M, std::int32_t>) c.i32(v);
    else if constexpr (Is<M, std::int64_t>) c.i64(v);
    else if constexpr (Is<M, float>) c.f32(v);
    else c.f64(v);
}

template <class C, Is<Vec3> M>
void fields(C& c, M& v) {
    c.f32(v.x);
    c.f32(v.y);
    c.f32(v.z);
}

template <class C, Is<Quat> M>
void fields(C& c, M& q) {
    c.f32(q.x);
    c.f32(q.y);
    c.f32(q.z);
    c.f32(q.w);
}

template <class C, Is<Pose3> M>
void fields(C& c, M& p) {
    fields(c, p.position);
    fields(c, p.rotation);
}

}  // namespace wpiemu::sdk::wire
