#pragma once

// Primitive-only geometry (box / cylinder) shared by chassis, mechanisms and the editor.

#include <string>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

enum class PrimitiveKind { Box, Cylinder };

struct PrimitiveSpec {
    std::string name;
    PrimitiveKind kind = PrimitiveKind::Box;
    Pose3 local;                       // relative to the owning frame (chassis or mechanism mount)
    Vec3 sizeM{0.1f, 0.1f, 0.1f};      // Box: FULL extents (x, y, z)
    float radiusM = 0.05f;             // Cylinder
    float heightM = 0.1f;              // Cylinder (axis = local Z)
    std::string color = "#808080";     // "#rrggbb"
    bool collides = false;             // true: contributes a collision shape on the owning body
    float massKg = 0.f;                // informational (editor mass budget); physics uses the owner's mass
    // Mechanism primitives only: the mechanism link this primitive follows. Empty (default) = derive it from the
    // primitive name prefix (see app/runview linkForPrimitive); "root" = fixed to the mechanism root frame.
    std::string link;

    bool operator==(const PrimitiveSpec&) const = default;
};

}  // namespace wpiemu::sdk
