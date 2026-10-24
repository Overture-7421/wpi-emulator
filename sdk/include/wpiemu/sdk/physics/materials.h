#pragma once

// Contact materials shared by every field (docs/reference/physics.md#contact-materials). A contact
// mixes the two shapes: friction = sqrt(muA * muB), restitution = max(eA, eB). The robot hull
// defaults to {0.3, 0.1} (ChassisSpec::material), so bumpers against kFieldStructure slide with an
// effective friction of about 0.21.

#include "wpiemu/sdk/physics/physics_types.h"

namespace wpiemu::sdk::materials {

// Carpet under the wheels. Wheel traction comes from the tire model, so this only affects fuel and
// other bodies.
inline constexpr Material kCarpet{0.8f, 0.f, 0.f};

// Walls, guardrails, posts and other rigid field structure. Polycarbonate and painted steel are
// slicker and less bouncy than the engine default, so a robot driving at an angle into them slides
// along instead of sticking.
inline constexpr Material kFieldStructure{0.15f, 0.1f, 0.f};

}  // namespace wpiemu::sdk::materials
