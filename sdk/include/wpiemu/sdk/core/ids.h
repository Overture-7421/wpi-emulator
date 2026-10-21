#pragma once

// Strong (tag-typed) ids: a BodyId can never be passed where a ShapeId is expected.
// A default-constructed id (value 0) is the "invalid / none" id.

#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace wpiemu::sdk {

template <class Tag, class Rep = std::uint64_t>
struct StrongId {
    using rep_type = Rep;
    Rep value{};

    constexpr StrongId() = default;
    constexpr explicit StrongId(Rep v) : value(v) {}

    constexpr bool valid() const { return value != Rep{}; }
    constexpr explicit operator bool() const { return valid(); }
    constexpr auto operator<=>(const StrongId&) const = default;
};

// Opaque physics handles. Their bit layout is private to the IPhysicsWorld implementation.
using BodyId = StrongId<struct BodyIdTag>;
using ShapeId = StrongId<struct ShapeIdTag>;
// Runtime device handle assigned by the extension when a device is announced (stable for one session).
using DeviceId = StrongId<struct DeviceIdTag, std::uint32_t>;

}  // namespace wpiemu::sdk

namespace std {
template <class Tag, class Rep>
struct hash<wpiemu::sdk::StrongId<Tag, Rep>> {
    std::size_t operator()(const wpiemu::sdk::StrongId<Tag, Rep>& id) const noexcept {
        return std::hash<Rep>{}(id.value);
    }
};
}  // namespace std
