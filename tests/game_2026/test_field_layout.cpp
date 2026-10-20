#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
namespace dim = wpiemu::plugins::game_2026::dim;

namespace {

const FieldBox* findBox(const FieldLayout& l, const std::string& name) {
    for (const FieldBox& b : l.boxes)
        if (b.name == name) return &b;
    return nullptr;
}

std::string swapAlliance(std::string name) {
    const auto pos = name.find("blue");
    if (pos != std::string::npos) name.replace(pos, 4, "red");
    return name;
}

float signedVolume(const BumpMesh& m) {
    float v = 0.f;
    for (std::size_t t = 0; t + 2 < m.triangles.size(); t += 3) {
        auto p = [&](std::uint32_t i) { return sdk::Vec3{m.xyz[i * 3], m.xyz[i * 3 + 1], m.xyz[i * 3 + 2]}; };
        v += sdk::dot(p(m.triangles[t]), sdk::cross(p(m.triangles[t + 1]), p(m.triangles[t + 2]))) / 6.f;
    }
    return v;
}

}  // namespace

TEST_CASE("field layout uses the official REBUILT dimensions") {
    const FieldLayout l = buildFieldLayout();
    CHECK(l.description.lengthM == doctest::Approx(16.541f));
    CHECK(l.description.widthM == doctest::Approx(8.069f));

    // hub: 47 in square, opening front edge 72 in, centred between the bumps, 4.03 m from the wall
    const FieldBox* front = findBox(l, "hub_blue_wall_front");
    REQUIRE(front != nullptr);
    CHECK(front->sizeM.y == doctest::Approx(1.1938f));
    CHECK(front->sizeM.z == doctest::Approx(1.8288f));
    CHECK(l.hub(Alliance::Blue).center.x - dim::kHubSizeM / 2 == doctest::Approx(4.029f).epsilon(0.002));
    CHECK(l.hub(Alliance::Blue).center.y == doctest::Approx(dim::kFieldWidthM / 2));
    CHECK(l.hub(Alliance::Red).center.x == doctest::Approx(dim::kFieldLengthM - dim::kHubCenterXM));

    // bumps: 73 in wide, 44.4 in deep, 6.513 in tall; four of them
    REQUIRE(l.bumps.size() == 4);
    for (const BumpSpec& b : l.bumps) {
        CHECK(b.widthM == doctest::Approx(1.8542f));
        CHECK(b.depthM == doctest::Approx(1.1278f).epsilon(0.001));
        CHECK(b.ridgeHeightM == doctest::Approx(0.16543f));
        CHECK(b.edgeHeightM > 0.f);
        CHECK(b.edgeHeightM < b.ridgeHeightM);
    }
    // the 15 degree slope reproduces the ridge: edge + half the depth * tan(15)
    const BumpSpec& b0 = l.bumps.front();
    CHECK(b0.edgeHeightM + b0.depthM / 2 * std::tan(15.f * 3.14159265f / 180.f) == doctest::Approx(b0.ridgeHeightM).epsilon(0.001));

    // trench: arm underside 22.25 in, top 29.75 in (the tag panels stand on it up to 40.25 in), 47 in deep; four arms
    const FieldBox* arm = findBox(l, "trench_blue_low_y_arm");
    REQUIRE(arm != nullptr);
    CHECK(arm->pose.position.z - arm->sizeM.z / 2 == doctest::Approx(0.56515f));
    CHECK(arm->pose.position.z + arm->sizeM.z / 2 == doctest::Approx(0.75565f));
    CHECK(dim::kTrenchArmTopM + dim::inch(10.5) == doctest::Approx(dim::kTrenchHeightM));
    CHECK(arm->sizeM.x == doctest::Approx(1.1938f));
    int arms = 0;
    for (const FieldBox& b : l.boxes) arms += b.name.find("_arm") != std::string::npos ? 1 : 0;
    CHECK(arms == 4);

    // tower: 3 rungs per alliance at 27 / 45 / 63 in
    REQUIRE(l.cylinders.size() == 6);
    std::vector<float> rungZ;
    for (const FieldCylinder& c : l.cylinders)
        if (c.name.find("tower_blue") != std::string::npos) rungZ.push_back(c.pose.position.z);
    std::sort(rungZ.begin(), rungZ.end());
    REQUIRE(rungZ.size() == 3);
    CHECK(rungZ[0] == doctest::Approx(0.6858f));
    CHECK(rungZ[1] == doctest::Approx(1.143f));
    CHECK(rungZ[2] == doctest::Approx(1.6002f));
}

TEST_CASE("red is blue rotated 180 degrees about the field centre") {
    const FieldLayout l = buildFieldLayout();
    int checked = 0;
    for (const FieldBox& blue : l.boxes) {
        if (blue.name.find("blue") == std::string::npos) continue;
        const FieldBox* red = findBox(l, swapAlliance(blue.name));
        REQUIRE_MESSAGE(red != nullptr, blue.name);
        CHECK(red->pose.position.x == doctest::Approx(dim::kFieldLengthM - blue.pose.position.x).epsilon(1e-4));
        CHECK(red->pose.position.y == doctest::Approx(dim::kFieldWidthM - blue.pose.position.y).epsilon(1e-4));
        CHECK(red->pose.position.z == doctest::Approx(blue.pose.position.z));
        CHECK(red->sizeM.x == doctest::Approx(blue.sizeM.x));
        CHECK(red->sizeM.y == doctest::Approx(blue.sizeM.y));
        ++checked;
    }
    CHECK(checked > 15);
    for (const sdk::Vec3 d : {l.hub(Alliance::Blue).exits[0].direction}) {
        CHECK(d.x > 0.f);  // blue exits face +X (neutral zone)
    }
    for (const HubExit& e : l.hub(Alliance::Red).exits) CHECK(e.direction.x < 0.f);
    for (const Alliance a : kAlliances)
        for (const HubExit& e : l.hub(a).exits) {
            CHECK(sdk::length(e.direction) == doctest::Approx(1.f).epsilon(1e-4));
            CHECK(e.direction.z < 0.f);  // fuel leaves downhill
            CHECK(e.position.z > 0.f);
        }
}

TEST_CASE("every field element lies inside the field or behind an alliance wall") {
    const FieldLayout l = buildFieldLayout();
    for (const FieldBox& b : l.boxes) {
        if (b.name == "ground" || b.name.find("guardrail") == 0 || b.name.find("wall_") == 0) continue;
        const float x0 = b.pose.position.x - b.sizeM.x / 2, x1 = b.pose.position.x + b.sizeM.x / 2;
        const float y0 = b.pose.position.y - b.sizeM.y / 2, y1 = b.pose.position.y + b.sizeM.y / 2;
        CHECK_MESSAGE(x0 >= -1e-3f, b.name);
        CHECK_MESSAGE(x1 <= dim::kFieldLengthM + 1e-3f, b.name);
        CHECK_MESSAGE(y0 >= -1e-3f, b.name);
        CHECK_MESSAGE(y1 <= dim::kFieldWidthM + 1e-3f, b.name);
    }
    // sensors: one scoring volume per hub below the rim, one corral per outpost behind the wall
    int hubSensors = 0, corrals = 0;
    for (const SensorVolume& s : l.sensors) {
        if (tagKind(s.tag) == EntityKind::HubSensor) {
            ++hubSensors;
            CHECK(s.pose.position.z + s.halfExtentsM.z < dim::kHubTopZM);
            CHECK(s.halfExtentsM.z * 2 >= 0.3f);  // ADR 0003: sensors have no CCD
        } else if (tagKind(s.tag) == EntityKind::CorralSensor) {
            ++corrals;
        }
    }
    CHECK(hubSensors == 2);
    CHECK(corrals == 2);
}

TEST_CASE("bump mesh is a closed outward-wound prism with the expected volume") {
    const FieldLayout l = buildFieldLayout();
    const BumpSpec& b = l.bumps.front();
    const BumpMesh m = buildBumpMesh(b);
    CHECK(m.xyz.size() == 30);
    CHECK(m.triangles.size() == 3 * 16);  // 2 caps x 3 + 5 sides x 2 triangles = 16

    // watertight: every directed edge has exactly one opposite
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    for (std::size_t t = 0; t < m.triangles.size(); t += 3)
        for (int e = 0; e < 3; ++e) ++edges[{m.triangles[t + e], m.triangles[t + (e + 1) % 3]}];
    for (const auto& [edge, count] : edges) {
        CHECK(count == 1);
        CHECK(edges.count({edge.second, edge.first}) == 1);
    }

    const float area = b.depthM * (b.edgeHeightM + b.ridgeHeightM) / 2;
    CHECK(signedVolume(m) == doctest::Approx(area * b.widthM).epsilon(1e-3));  // positive => outward

    float maxZ = 0.f;
    for (std::size_t i = 2; i < m.xyz.size(); i += 3) maxZ = std::max(maxZ, m.xyz[i]);
    CHECK(maxZ == doctest::Approx(b.ridgeHeightM));
}

TEST_CASE("bump ramp visuals rise toward the ridge, never form a valley") {
    const FieldLayout l = buildFieldLayout();
    for (const BumpSpec& b : l.bumps) {
        int found = 0;
        for (const sdk::FieldVisual& v : l.description.visuals) {
            const bool low = v.name == b.name + "_ramp_a";   // on the -X side of the ridge
            const bool high = v.name == b.name + "_ramp_b";  // on the +X side
            if (!low && !high) continue;
            ++found;
            const float slopeAlongX = v.pose.rotation.rotate({1.f, 0.f, 0.f}).z;
            if (low) CHECK(slopeAlongX > 0.f);
            else CHECK(slopeAlongX < 0.f);
        }
        CHECK(found == 2);
    }
}

TEST_CASE("field is built from the layout and removed again by RAII") {
    sdk::testing::FakePhysicsWorld world;
    const FieldLayout layout = buildFieldLayout();
    const std::size_t expected = layout.boxes.size() + layout.cylinders.size() + layout.bumps.size() + layout.sensors.size();
    {
        auto field = Game2026Field::create(world, layout);
        REQUIRE(field.ok());
        CHECK(field.value()->shapeCount() == expected);
        CHECK(world.stats().bodies == 1);
        CHECK(world.shapeCount() == expected);
        CHECK(field.value()->description().name == "REBUILT (2026)");
        CHECK(field.value()->description().visuals.size() > 40);
        CHECK(world.isValid(field.value()->body()));
    }
    CHECK(world.stats().bodies == 0);
    CHECK(world.shapeCount() == 0);
}

TEST_CASE("hub sensors carry alliance tags and report fuel through the world's sensor events") {
  for (const std::uint64_t category : {sdk::layer::Fuel, kShotFuelLayer}) {
    sdk::testing::FakePhysicsWorld world;
    auto field = Game2026Field::create(world, buildFieldLayout());
    REQUIRE(field.ok());

    sdk::BodyDesc bd;
    bd.type = sdk::BodyType::Dynamic;
    bd.pose.position = {dim::kFieldLengthM - dim::kHubCenterXM, dim::kFieldWidthM / 2, dim::kHubSensorCenterZM};
    const sdk::BodyId ball = world.createBody(bd);
    sdk::ShapeDesc sd;
    sd.geometry = sdk::SphereShape{dim::kFuelRadiusM};
    sd.category = category;
    sd.mask = sdk::layer::All;
    sd.sensorEvents = true;
    sd.userTag = makeTag(EntityKind::Fuel, Alliance::Red, 7);
    REQUIRE(world.addShape(ball, sd).valid());
    world.setLinearVelocity(ball, {});
    world.step(0.001f);

    bool sawRedHub = false;
    for (const sdk::SensorEvent& e : world.sensorEvents())
        if (e.begin && tagKind(e.sensorTag) == EntityKind::HubSensor && tagAlliance(e.sensorTag) == Alliance::Red &&
            tagId(e.visitorTag) == 7)
            sawRedHub = true;
    CHECK(sawRedHub);
  }
}

TEST_CASE("field creation reports a refusing physics world instead of leaking") {
    SUBCASE("no body") {
        g26test::ForwardingWorld world;
        world.refuseBodies = true;
        auto field = Game2026Field::create(world, buildFieldLayout());
        REQUIRE_FALSE(field.ok());
        CHECK(field.error().code == sdk::ErrorCode::Internal);
    }
    SUBCASE("a shape in the middle is refused: body is destroyed") {
        g26test::ForwardingWorld world;
        world.shapeBudget = 10;
        auto field = Game2026Field::create(world, buildFieldLayout());
        REQUIRE_FALSE(field.ok());
        CHECK(field.error().message.find("refused field shape") != std::string::npos);
        CHECK(world.stats().bodies == 0);
    }
    SUBCASE("an adapter without mesh support gets tilted slabs instead") {
        g26test::ForwardingWorld world;
        world.refuseMeshes = true;
        const FieldLayout layout = buildFieldLayout();
        auto field = Game2026Field::create(world, layout);
        REQUIRE(field.ok());
        const std::size_t expected = layout.boxes.size() + layout.cylinders.size() + 3 * layout.bumps.size() + layout.sensors.size();
        CHECK(field.value()->shapeCount() == expected);
    }
}
