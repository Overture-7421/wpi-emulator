#pragma once

// Shared helpers of the vision engine tests: poses, cameras, tag layouts and worlds.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/vision/camera_model.h"
#include "wpiemu/core/vision/tag_geometry.h"
#include "wpiemu/core/vision/vision_math.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/physics/physics_types.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"
#include "wpiemu/sdk/vision/camera_config.h"
#include "wpiemu/sdk/vision/fiducial_layout.h"

namespace vision_test {

namespace vis = wpiemu::core::vision;
namespace sdk = wpiemu::sdk;
using vis::Mat3d;
using vis::Pose3d;
using vis::Vec3d;

inline constexpr double kPi = 3.14159265358979323846;

// Frame of a body at `position` whose +X axis is turned by yaw about +Z, then pitch about the
// body's +Y (positive pitch lowers +X, the right-hand rule), then roll about its +X.
inline Pose3d poseYpr(const Vec3d& position, double yawRad, double pitchRad = 0.0,
                      double rollRad = 0.0) {
    const Mat3d r = vis::rotationFromAxisAngle({0.0, 0.0, 1.0}, yawRad) *
                    vis::rotationFromAxisAngle({0.0, 1.0, 0.0}, pitchRad) *
                    vis::rotationFromAxisAngle({1.0, 0.0, 0.0}, rollRad);
    return {r, position};
}

// A rotation whose +X axis is `xAxis`, spun by `rollRad` about it; the reference "up" is +Z.
inline Mat3d rotationWithXAxis(const Vec3d& xAxis, double rollRad) {
    const Vec3d x = vis::normalized(xAxis);
    const Vec3d y = vis::normalized(vis::cross({0.0, 0.0, 1.0}, x));
    const Vec3d z = vis::cross(x, y);
    const double c = std::cos(rollRad), s = std::sin(rollRad);
    return Mat3d::fromColumns(x, y * c + z * s, y * (-s) + z * c);
}

inline sdk::Pose3 sdkPose(float x, float y, float z, float yawRad = 0.f) {
    return {sdk::Vec3{x, y, z}, sdk::Quat::fromYaw(yawRad)};
}

// A tag whose outward normal points along yaw (yaw = pi: facing -X, i.e. toward the field origin).
inline sdk::FiducialTag tagAt(int id, float x, float y, float z, float yawRad = 3.14159265f) {
    sdk::FiducialTag t;
    t.id = id;
    t.pose = sdkPose(x, y, z, yawRad);
    return t;
}

inline sdk::FiducialLayout layoutOf(std::vector<sdk::FiducialTag> tags) {
    sdk::FiducialLayout l;
    l.id = "test.layout";
    l.tags = std::move(tags);
    return l;
}

// 1280x800, 90 degree horizontal FOV (fx = fy = 640), ideal, no line of sight check.
inline sdk::CameraConfig makeCamera(const std::string& id = "cam") {
    sdk::CameraConfig c;
    c.id = id;
    c.occlusion = false;
    return c;
}

inline sdk::VisionConfig visionOf(std::vector<sdk::CameraConfig> cameras) {
    sdk::VisionConfig v;
    v.cameras = std::move(cameras);
    return v;
}

// Pose error helpers (SDK float pose against the exact double pose).
inline double positionError(const sdk::Pose3& got, const Pose3d& want) {
    return vis::norm(vis::fromSdk(got.position) - want.t);
}
inline double rotationError(const sdk::Pose3& got, const Pose3d& want) {
    return vis::rotationDistanceRad(vis::fromSdk(got).R, want.R);
}

// A fake world with one kinematic "chassis" body.
struct FakeWorld {
    sdk::testing::FakePhysicsWorld world;
    sdk::BodyId chassis;

    FakeWorld() {
        world.setFloorEnabled(false);
        sdk::BodyDesc bd;
        bd.type = sdk::BodyType::Kinematic;
        chassis = world.createBody(bd);
    }
    void place(const sdk::Pose3& pose) { world.setPose(chassis, pose); }
};

// A real Box3D world (ray casts) with a kinematic chassis body and static Field boxes.
struct BoxWorld {
    std::unique_ptr<sdk::IPhysicsWorld> world;
    sdk::BodyId chassis;

    BoxWorld() {
        sdk::PhysicsConfig cfg;
        cfg.gravity = {0.f, 0.f, 0.f};
        world = wpiemu::core::physics::createBox3dWorld(cfg);
        sdk::BodyDesc bd;
        bd.type = sdk::BodyType::Kinematic;
        chassis = world->createBody(bd);
    }
    void addBox(const sdk::Vec3& center, const sdk::Vec3& halfExtents,
                std::uint64_t category = sdk::layer::Field) {
        sdk::BodyDesc bd;
        bd.type = sdk::BodyType::Static;
        bd.pose.position = center;
        const sdk::BodyId body = world->createBody(bd);
        sdk::ShapeDesc sd;
        sd.geometry = sdk::BoxShape{halfExtents};
        sd.category = category;
        sd.mask = sdk::layer::All;
        world->addShape(body, sd);
    }
};

}  // namespace vision_test
