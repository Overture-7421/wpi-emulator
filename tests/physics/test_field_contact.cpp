#include <doctest/doctest.h>

#include "field_contact_support.h"
#include "wpiemu/sdk/physics/materials.h"

using namespace physics_test;

namespace {

constexpr double kApproachMps = 2.5;
const Vec3 kIntoWall{-1.f, 0.f, 0.f};
const Vec3 kAlongWall{0.f, 1.f, 0.f};

// A wall whose face is the plane x = 0, approached from +X.
void addWall(IPhysicsWorld& world, const Material& material) {
    BodyDesc body;
    body.type = BodyType::Static;
    const BodyId id = world.createBody(body);
    ShapeDesc shape;
    shape.geometry = BoxShape{{0.15f, 6.f, 1.f}};
    shape.local = Pose3{{-0.15f, 0.f, 1.f}, Quat::identity()};
    shape.material = material;
    shape.category = layer::Field;
    world.addShape(id, shape);
}

struct WallRig {
    explicit WallRig(const Material& wall)
        : rig(bumperedSwerve(), bumperedChassis(), PhysicsConfig{}, placement()) {
        addWall(*rig.world, wall);
    }
    static phys::ChassisPlacement placement() {
        phys::ChassisPlacement p;
        p.xM = 1.3f;
        p.yawRad = kPiF;
        return p;
    }
    WallRun drive(double angleDeg, double seconds) {
        return driveIntoWall(rig, kIntoWall, kAlongWall, angleDeg, kApproachMps, seconds, kPi);
    }
    Rig rig;
};

double commandedAlong(double angleDeg) {
    return kApproachMps * std::sin(angleDeg * kPi / 180.0);
}

}  // namespace

TEST_CASE(
    "wall contact: a bumpered swerve robot driven at an angle into field structure slides along "
    "it") {
    struct Case {
        double angleDeg;
        double minFraction;  // of the commanded along-wall speed
    };
    for (const Case c : {Case{20.0, 0.25}, Case{30.0, 0.45}, Case{45.0, 0.55}}) {
        CAPTURE(c.angleDeg);
        WallRig wall(materials::kFieldStructure);
        const WallRun run = wall.drive(c.angleDeg, 3.0);
        CHECK(run.alongMps > c.minFraction * commandedAlong(c.angleDeg));
        CHECK(run.maxTiltDeg < 4.0);
    }
}

TEST_CASE("wall contact: a head-on hit barely bounces and the robot does not climb the wall") {
    WallRig wall(materials::kFieldStructure);
    const WallRun run = wall.drive(0.0, 3.0);
    CHECK(run.maxReboundMps < 0.35);
    CHECK(run.maxTiltDeg < 3.0);
    CHECK(std::fabs(run.alongMps) < 0.1);
}

TEST_CASE("wall contact: a robot pinned head-on drives away from the wall when reversed") {
    WallRig wall(materials::kFieldStructure);
    wall.drive(0.0, 1.5);
    const float pinnedX = wall.rig.position().x;
    wall.drive(180.0, 1.0);
    CHECK(wall.rig.position().x > pinnedX + 0.5f);
}
