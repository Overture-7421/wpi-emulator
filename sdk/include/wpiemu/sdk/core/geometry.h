#pragma once

// Primitive-only geometry (box / cylinder) shared by chassis, mechanisms and the editor.

#include <string>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

enum class PrimitiveKind { Box, Cylinder };

inline constexpr const char* kDefaultPrimitiveColor = "#808080";

struct PrimitiveSpec {
    std::string name;
    PrimitiveKind kind = PrimitiveKind::Box;
    Pose3 local;                       // relative to the owning frame (chassis or mechanism mount)
    Vec3 sizeM{0.1f, 0.1f, 0.1f};      // Box: FULL extents (x, y, z)
    float radiusM = 0.05f;             // Cylinder
    float heightM = 0.1f;              // Cylinder (axis = local Z)
    std::string color = kDefaultPrimitiveColor;   // "#rrggbb"
    bool collides = false;             // true: contributes a collision shape on the owning body
    float massKg = 0.f;                // informational (editor mass budget); physics uses the owner's mass

    // Full extents along the local axes: the box size, or (diameter, diameter, height) for a cylinder.
    Vec3 extentsM() const { return kind == PrimitiveKind::Box ? sizeM : Vec3{radiusM * 2.f, radiusM * 2.f, heightM}; }

    bool operator==(const PrimitiveSpec&) const = default;
};

}  // namespace wpiemu::sdk
