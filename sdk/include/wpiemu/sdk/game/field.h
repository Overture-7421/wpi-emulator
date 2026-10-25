#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/start_position.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::sdk {

// How the season's field maps one alliance's half onto the other's: rotated 180 degrees about the field centre,
// or mirrored across the centre line x = length / 2.
enum class FieldSymmetry { None, Rotational, Mirrored };

struct FieldFrame {
    float lengthM = 0.f;   // along +X
    float widthM = 0.f;    // along +Y
    FieldSymmetry symmetry = FieldSymmetry::None;
};

// The same physical start for the other alliance. nullopt when the field has no known symmetry.
inline std::optional<RobotStartPose> flipToOtherAlliance(const RobotStartPose& pose, const FieldFrame& frame) {
    constexpr double pi = kPi;
    double heading = 0.0;
    RobotStartPose out;
    switch (frame.symmetry) {
        case FieldSymmetry::Rotational:
            out.xM = frame.lengthM - pose.xM;
            out.yM = frame.widthM - pose.yM;
            heading = static_cast<double>(pose.headingRad) + pi;
            break;
        case FieldSymmetry::Mirrored:
            out.xM = frame.lengthM - pose.xM;
            out.yM = pose.yM;
            heading = pi - static_cast<double>(pose.headingRad);
            break;
        case FieldSymmetry::None: return std::nullopt;
    }
    heading = sdk::wrapAngleRad(heading);
    out.headingRad = static_cast<float>(heading);
    return out;
}

// A static visual element of the field for the renderer (colliders are created in the physics
// world; this is what to DRAW). Primitive-based so plugins need no mesh pipeline; large meshes
// can be referenced by asset path instead.
struct FieldVisual {
    enum class Kind { Box, Cylinder, Mesh };
    std::string name;
    Kind kind = Kind::Box;
    Pose3 pose;
    Vec3 sizeM{1.f, 1.f, 1.f};   // Box: full extents; Cylinder: (diameter, diameter, height); Mesh: scale
    std::string color = "#9090a0";
    float opacity = 1.f;         // 1 = opaque; below 1 the renderer alpha-blends it (clear plastic)
    std::string meshAsset;       // Kind::Mesh: path relative to the plugin's asset root
};

// A drive-team eye position behind an alliance wall. `number` is 1..3, left to right as seen by the
// drive team facing the field. The view looks from `eyeM` toward the field.
struct DriverStationSpot {
    Alliance alliance = Alliance::Blue;
    int number = 1;
    Vec3 eyeM;
};

struct FieldDescription {
    std::string name;
    float lengthM = 0.f;         // along +X
    float widthM = 0.f;          // along +Y
    std::string assetRoot;       // directory for meshAsset, resolved by the app
    std::vector<FieldVisual> visuals;
    std::vector<DriverStationSpot> driverStations;  // optional driver viewpoints, field frame
};

inline constexpr float kDriverEyeHeightM = 1.6f;
inline constexpr float kDriverEyeSetbackM = 0.8f;

inline std::vector<DriverStationSpot> genericDriverStations(float lengthM, float widthM) {
    std::vector<DriverStationSpot> spots;
    if (!(lengthM > 0.f) || !(widthM > 0.f) || !std::isfinite(lengthM) || !std::isfinite(widthM)) return spots;
    spots.reserve(6);
    for (int number = 1; number <= 3; ++number) {
        const float fromLeft = (2.f * static_cast<float>(number) - 1.f) / 6.f;
        spots.push_back({Alliance::Blue, number, {-kDriverEyeSetbackM, widthM * (1.f - fromLeft), kDriverEyeHeightM}});
        spots.push_back({Alliance::Red, number, {lengthM + kDriverEyeSetbackM, widthM * fromLeft, kDriverEyeHeightM}});
    }
    return spots;
}

inline std::optional<DriverStationSpot> driverStationSpot(const FieldDescription& field, Alliance alliance, int number) {
    const int clamped = std::clamp(number, 1, 3);
    for (const DriverStationSpot& spot : field.driverStations)
        if (spot.alliance == alliance && spot.number == clamped) return spot;
    if (!(field.lengthM > 0.f) || !(field.widthM > 0.f) || !std::isfinite(field.lengthM) ||
        !std::isfinite(field.widthM))
        return std::nullopt;
    const float fromLeft = (2.f * static_cast<float>(clamped) - 1.f) / 6.f;
    if (alliance == Alliance::Blue)
        return DriverStationSpot{alliance, clamped,
                                 {-kDriverEyeSetbackM, field.widthM * (1.f - fromLeft), kDriverEyeHeightM}};
    return DriverStationSpot{alliance, clamped,
                             {field.lengthM + kDriverEyeSetbackM, field.widthM * fromLeft, kDriverEyeHeightM}};
}

// Handle to the static world a plugin created (colliders, sensor volumes). RAII: destroying it
// removes what createField() added to the IPhysicsWorld. It must be destroyed BEFORE the world.
class IField {
    WPIEMU_INTERFACE(IField)
public:
    virtual const FieldDescription& description() const = 0;
};

}  // namespace wpiemu::sdk
