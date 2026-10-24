#include <doctest/doctest.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>

#include "field_contact_support.h"
#include "refusing_world.h"
#include "wpiemu/sdk/game/default_field.h"

using namespace physics_test;
namespace df = wpiemu::sdk::default_field;

namespace {

struct FieldRig {
    explicit FieldRig(phys::ChassisPlacement placement)
        : rig(bumperedSwerve(), bumperedChassis(), PhysicsConfig{}, placement, false) {
        auto created = createDefaultField(*rig.world);
        REQUIRE(created.ok());
        field = std::move(created).value();
    }
    ~FieldRig() { field.reset(); }

    Rig rig;
    std::unique_ptr<IField> field;
};

phys::ChassisPlacement centre() {
    phys::ChassisPlacement p;
    p.xM = df::kLengthM / 2;
    p.yM = df::kWidthM / 2;
    return p;
}

}  // namespace

TEST_CASE(
    "default field: describes an FRC-sized field with uniquely named visuals and see-through "
    "panels") {
    const FieldDescription d = defaultFieldDescription();
    CHECK(d.lengthM == df::kLengthM);
    CHECK(d.widthM == df::kWidthM);
    CHECK(d.visuals.size() > 8);
    std::set<std::string> names;
    for (const FieldVisual& v : d.visuals) CHECK(names.insert(v.name).second);
    CHECK(std::any_of(d.visuals.begin(), d.visuals.end(),
                      [](const FieldVisual& v) { return v.opacity < 1.f; }));
}

TEST_CASE("default field: the robot rests on the carpet and ground rays find it") {
    FieldRig f(centre());
    f.rig.run(1.f);
    CHECK(f.rig.position().z ==
          doctest::Approx(f.rig.sim->chassisBody().restHeightM).epsilon(0.05));
    for (const Vec3 at : {Vec3{0.5f, 0.5f, 1.f}, Vec3{df::kLengthM - 0.5f, df::kWidthM - 0.5f, 1.f},
                          Vec3{df::kLengthM / 2, df::kWidthM / 2, 1.f}}) {
        const RayHit hit = f.rig.world->rayCastClosest(at, {0.f, 0.f, -1.f}, 2.f, layer::Field);
        REQUIRE(hit.hit);
        CHECK(hit.point.z == doctest::Approx(0.f).epsilon(0.01));
    }
}

TEST_CASE("default field: the walls and guardrails sit on the field boundary") {
    FieldRig f(centre());
    const float y = df::kWidthM / 2, x = df::kLengthM / 2, z = 0.3f;
    struct Probe {
        Vec3 from;
        Vec3 direction;
        Vec3 expected;
    };
    for (const Probe p : {Probe{{x, y, z}, {-1.f, 0.f, 0.f}, {0.f, y, z}},
                          Probe{{x, y, z}, {1.f, 0.f, 0.f}, {df::kLengthM, y, z}},
                          Probe{{x, y, z}, {0.f, -1.f, 0.f}, {x, 0.f, z}},
                          Probe{{x, y, z}, {0.f, 1.f, 0.f}, {x, df::kWidthM, z}}}) {
        const RayHit hit = f.rig.world->rayCastClosest(p.from, p.direction, 20.f, layer::Field);
        REQUIRE(hit.hit);
        CHECK(hit.point.x == doctest::Approx(p.expected.x).epsilon(0.001));
        CHECK(hit.point.y == doctest::Approx(p.expected.y).epsilon(0.001));
    }
}

TEST_CASE("default field: a robot driven hard at every wall and corner stays inside the field") {
    const float c = 0.7071f;
    for (const Vec3 direction :
         {Vec3{1.f, 0.f, 0.f}, Vec3{-1.f, 0.f, 0.f}, Vec3{0.f, 1.f, 0.f}, Vec3{0.f, -1.f, 0.f},
          Vec3{c, c, 0.f}, Vec3{c, -c, 0.f}, Vec3{-c, c, 0.f}, Vec3{-c, -c, 0.f}}) {
        CAPTURE(direction.x);
        CAPTURE(direction.y);
        FieldRig f(centre());
        for (int i = 0; i < static_cast<int>(6.0 / kDt); ++i) {
            driveFieldOriented(f.rig, 3.0 * direction.x, 3.0 * direction.y, 0.0);
            f.rig.step();
        }
        const Vec3 p = f.rig.position();
        CHECK(p.x > 0.f);
        CHECK(p.x < df::kLengthM);
        CHECK(p.y > 0.f);
        CHECK(p.y < df::kWidthM);
        CHECK(p.z < 0.5f);
    }
}

TEST_CASE(
    "default field: destroying it removes its body and a refusing world leaves nothing behind") {
    auto world = makeWorld();
    const int bodies = world->stats().bodies;
    {
        auto field = createDefaultField(*world);
        REQUIRE(field.ok());
        CHECK(world->stats().bodies == bodies + 1);
    }
    CHECK(world->stats().bodies == bodies);

    for (const int budget : {0, 1, 4}) {
        INFO(budget);
        RefusingWorld refusing;
        refusing.shapeBudget = budget;
        CHECK_FALSE(createDefaultField(refusing).ok());
        CHECK(refusing.stats().bodies == 0);
        CHECK(refusing.inner.shapeCount() == 0);
    }
}
