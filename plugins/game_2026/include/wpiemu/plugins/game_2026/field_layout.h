#pragma once

// The 2026 field as DATA: every static collider / visual / sensor volume and the geometry the game
// logic needs (hub exits, outpost chute, depots). `buildFieldLayout()` is pure; `Game2026Field`
// turns it into physics shapes. Tests inspect the layout without any physics world.

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "wpiemu/plugins/game_2026/types.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::plugins::game_2026 {

struct FieldBox {
    std::string name;
    sdk::Pose3 pose;
    sdk::Vec3 sizeM;  // full extents
    std::string color;
    sdk::Material material;
    bool collides = true;
};

struct FieldCylinder {
    std::string name;
    sdk::Pose3 pose;
    float radiusM = 0.f;
    float heightM = 0.f;  // axis = local Z
    std::string color;
    sdk::Material material;
};

// Two 15 degree ramps meeting at a ridge; symmetric about both horizontal axes.
struct BumpSpec {
    std::string name;
    sdk::Vec3 center;   // centre of the footprint on the floor
    float widthM = 0.f; // along Y
    float depthM = 0.f; // along X
    float ridgeHeightM = 0.f;
    float edgeHeightM = 0.f;
    std::string color;
};

struct SensorVolume {
    std::string name;
    sdk::Pose3 pose;
    sdk::Vec3 halfExtentsM;
    std::uint64_t tag = 0;
};

struct HubExit {
    sdk::Vec3 position;
    sdk::Vec3 direction;  // unit
    float speedMps = 0.f;
};

struct HubSpec {
    Alliance alliance = Alliance::Red;
    sdk::Vec3 center;  // footprint centre on the floor
    std::array<HubExit, 4> exits;
};

struct OutpostSpec {
    Alliance alliance = Alliance::Red;
    sdk::Vec3 chuteExit;      // where released chute fuel appears
    sdk::Vec3 chuteVelocity;  // initial velocity of released fuel
    float chuteHalfWidthM = 0.f;
    sdk::Vec3 inwardY;        // unit vector along the chute opening (field Y direction)
};

struct DepotSpec {
    Alliance alliance = Alliance::Red;
    sdk::Vec3 center;   // centre of the fuel grid on the floor
    float xDir = 1.f;   // +1: grid extends toward +X from the wall (blue), -1: toward -X (red)
};

struct FieldLayout {
    std::vector<FieldBox> boxes;
    std::vector<FieldCylinder> cylinders;
    std::vector<BumpSpec> bumps;
    std::vector<SensorVolume> sensors;
    std::array<HubSpec, 2> hubs;
    std::array<OutpostSpec, 2> outposts;
    std::array<DepotSpec, 2> depots;
    sdk::FieldDescription description;

    const HubSpec& hub(Alliance a) const { return hubs[index(a)]; }
    const OutpostSpec& outpost(Alliance a) const { return outposts[index(a)]; }
    const DepotSpec& depot(Alliance a) const { return depots[index(a)]; }
};

FieldLayout buildFieldLayout();

// Closed triangle mesh (outward winding) of a bump: pentagonal prism extruded along Y.
struct BumpMesh {
    std::vector<float> xyz;
    std::vector<std::uint32_t> triangles;
};
BumpMesh buildBumpMesh(const BumpSpec& bump);

// RAII owner of the static colliders and sensor volumes created in `world`.
class Game2026Field final : public sdk::IField {
public:
    static sdk::Result<std::unique_ptr<Game2026Field>> create(sdk::IPhysicsWorld& world, FieldLayout layout);
    ~Game2026Field() override;

    Game2026Field(const Game2026Field&) = delete;
    Game2026Field& operator=(const Game2026Field&) = delete;

    const sdk::FieldDescription& description() const override { return layout_.description; }
    const FieldLayout& layout() const { return layout_; }
    sdk::BodyId body() const { return body_; }
    std::size_t shapeCount() const { return shapeCount_; }

private:
    Game2026Field(sdk::IPhysicsWorld& world, FieldLayout layout, sdk::BodyId body);

    sdk::IPhysicsWorld& world_;
    FieldLayout layout_;
    sdk::BodyId body_;
    std::size_t shapeCount_ = 0;
};

}  // namespace wpiemu::plugins::game_2026
