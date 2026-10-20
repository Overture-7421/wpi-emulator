#pragma once

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
    std::string meshAsset;       // Kind::Mesh: path relative to the plugin's asset root
};

struct FieldDescription {
    std::string name;
    float lengthM = 0.f;         // along +X
    float widthM = 0.f;          // along +Y
    std::string assetRoot;       // directory for meshAsset, resolved by the app
    std::vector<FieldVisual> visuals;
};

// Handle to the static world a plugin created (colliders, sensor volumes). RAII: destroying it
// removes what createField() added to the IPhysicsWorld. It must be destroyed BEFORE the world.
class IField {
    WPIEMU_INTERFACE(IField)
public:
    virtual const FieldDescription& description() const = 0;
};

}  // namespace wpiemu::sdk
