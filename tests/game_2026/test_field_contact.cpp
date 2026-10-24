// The real REBUILT walls against the wizard's default bumpered swerve robot, in the actual Box3D
// engine: a robot driven at an angle into a field wall must slide along it instead of sticking
// (docs/reference/physics.md).
#include <doctest/doctest.h>

#include <memory>

#include "../physics/field_contact_support.h"
#include "wpiemu/plugins/game_2026/field_layout.h"

using namespace physics_test;
namespace g26 = wpiemu::plugins::game_2026;

namespace {

constexpr double kApproachMps = 2.5;

// The alliance wall (face x = 0) is approached beside the outpost opening and short of the tower;
// the low-Y guardrail (face y = 0) runs the whole field length. `alongM` is where the robot starts
// along the wall.
struct FieldWallRig {
    FieldWallRig(bool guardrail, float alongM)
        : guardrail_(guardrail),
          rig(bumperedSwerve(), bumperedChassis(), PhysicsConfig{}, placement(guardrail, alongM),
              false) {
        auto created = g26::Game2026Field::create(*rig.world, g26::buildFieldLayout());
        REQUIRE(created.ok());
        field = std::move(created).value();
    }
    ~FieldWallRig() { field.reset(); }

    static phys::ChassisPlacement placement(bool guardrail, float alongM) {
        phys::ChassisPlacement p;
        p.xM = guardrail ? alongM : 1.3f;
        p.yM = guardrail ? 1.3f : alongM;
        p.yawRad = guardrail ? -0.5f * kPiF : kPiF;
        return p;
    }
    WallRun drive(double angleDeg, double seconds) {
        const Vec3 into = guardrail_ ? Vec3{0.f, -1.f, 0.f} : Vec3{-1.f, 0.f, 0.f};
        const Vec3 along = guardrail_ ? Vec3{1.f, 0.f, 0.f} : Vec3{0.f, 1.f, 0.f};
        return driveIntoWall(rig, into, along, angleDeg, kApproachMps, seconds,
                             guardrail_ ? -0.5 * kPi : kPi);
    }

    bool guardrail_;
    Rig rig;
    std::unique_ptr<g26::Game2026Field> field;
};

double commandedAlong(double angleDeg) {
    return kApproachMps * std::sin(angleDeg * kPi / 180.0);
}

}  // namespace

TEST_CASE("REBUILT guardrail: a bumpered swerve robot driven at an angle into it slides along") {
    struct Case {
        double angleDeg;
        double minFraction;  // of the commanded along-wall speed
    };
    for (const Case c : {Case{20.0, 0.25}, Case{30.0, 0.45}, Case{45.0, 0.55}}) {
        CAPTURE(c.angleDeg);
        FieldWallRig wall(true, 8.f);
        const WallRun run = wall.drive(c.angleDeg, 3.0);
        CHECK(run.alongMps > c.minFraction * commandedAlong(c.angleDeg));
    }
}

TEST_CASE(
    "REBUILT alliance wall: a bumpered swerve robot driven at 30 degrees into it slides along") {
    FieldWallRig wall(false, 1.4f);
    const double alongSpeed = commandedAlong(30.0);
    const WallRun run = wall.drive(30.0, 0.3 + 1.7 / alongSpeed);  // stops short of the tower
    CHECK(run.alongMps > 0.45 * alongSpeed);
}

TEST_CASE("REBUILT walls: a head-on hit barely bounces and does not tip the robot") {
    for (const bool guardrail : {false, true}) {
        CAPTURE(guardrail);
        FieldWallRig wall(guardrail, guardrail ? 8.f : 2.2f);
        const WallRun run = wall.drive(0.0, 3.0);
        CHECK(run.maxReboundMps < 0.35);
        CHECK(run.maxTiltDeg < 3.0);
    }
}
