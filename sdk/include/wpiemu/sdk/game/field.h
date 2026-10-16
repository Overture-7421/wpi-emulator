#pragma once

#include <memory>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

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
