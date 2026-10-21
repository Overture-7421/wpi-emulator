#pragma once

// The default wheels of the built-in drivetrain kinds. The wizard lays out a whole drivetrain with the layouts, the
// editor adds one wheel with drivetrainWheel() and the demo robots use the corner layout. Every layout copies `base`
// (radius, motors, gearing) into each wheel and sets what the layout owns: name, kind, steering, the hub position (x
// forward, y left, one radius above the floor) and, for mecanum, the roller tire.

#include <array>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/physics/chassis_spec.h"

namespace wpiemu::sdk {

// `base` as a wheel of a drivetrain of this kind: swerve wheels are steered modules, mecanum wheels are Mecanum and the
// wheels of every other kind are Standard.
inline WheelSpec drivetrainWheel(std::string_view drivetrainKind, WheelSpec base = {}) {
    const bool swerve = drivetrainKind == drivetrain_kind::kSwerve;
    base.kind = swerve ? WheelKind::Swerve : drivetrainKind == drivetrain_kind::kMecanum ? WheelKind::Mecanum : WheelKind::Standard;
    base.steered = swerve;
    return base;
}

namespace wheel_layout_detail {

struct Corner {
    const char* name;
    float sx;
    float sy;
};
inline constexpr std::array<Corner, 4> kCorners{{{"fl", 1.f, 1.f}, {"fr", 1.f, -1.f}, {"bl", -1.f, 1.f}, {"br", -1.f, -1.f}}};

inline WheelSpec placedWheel(std::string_view drivetrainKind, const WheelSpec& base, std::string name, float xM, float yM) {
    WheelSpec wheel = drivetrainWheel(drivetrainKind, base);
    wheel.name = std::move(name);
    wheel.positionM = {xM, yM, wheel.radiusM};
    return wheel;
}

}  // namespace wheel_layout_detail

// Four steered modules "fl", "fr", "bl", "br" at (+-halfLengthM, +-halfWidthM).
inline std::vector<WheelSpec> swerveWheels(const WheelSpec& base, float halfLengthM, float halfWidthM) {
    using namespace wheel_layout_detail;
    std::vector<WheelSpec> out;
    for (const Corner& c : kCorners)
        out.push_back(placedWheel(drivetrain_kind::kSwerve, base, c.name, c.sx * halfLengthM, c.sy * halfWidthM));
    return out;
}

// Four mecanum wheels named and placed like swerveWheels() in the X layout: the roller traction axes of "fl" and "br"
// point +45 deg, those of "fr" and "bl" -45 deg.
inline std::vector<WheelSpec> mecanumWheels(const WheelSpec& base, float halfLengthM, float halfWidthM) {
    using namespace wheel_layout_detail;
    constexpr float kQuarterPi = 0.78539816339f;
    std::vector<WheelSpec> out;
    for (const Corner& c : kCorners) {
        WheelSpec wheel = placedWheel(drivetrain_kind::kMecanum, base, c.name, c.sx * halfLengthM, c.sy * halfWidthM);
        wheel.widthM = 0.05f;
        wheel.tire.tractionAngleRad = kQuarterPi * c.sx * c.sy;
        wheel.tire.lateralScale = 0.15f;
        out.push_back(std::move(wheel));
    }
    return out;
}

// `perSide` fixed wheels per side, "left0".."left<n-1>" then "right0".., spread evenly from the rear (-halfSpanM) to the
// front (+halfSpanM) at y = +-halfWidthM.
inline std::vector<WheelSpec> tankWheels(const WheelSpec& base, int perSide, float halfSpanM, float halfWidthM) {
    using namespace wheel_layout_detail;
    std::vector<WheelSpec> out;
    for (const char* side : {"left", "right"}) {
        const float sy = side[0] == 'l' ? 1.f : -1.f;
        for (int i = 0; i < perSide; ++i) {
            const float t = perSide == 1 ? 0.f : (static_cast<float>(i) / static_cast<float>(perSide - 1)) * 2.f - 1.f;
            out.push_back(placedWheel(drivetrain_kind::kTank, base, std::string(side) + std::to_string(i), t * halfSpanM, sy * halfWidthM));
        }
    }
    return out;
}

}  // namespace wpiemu::sdk
