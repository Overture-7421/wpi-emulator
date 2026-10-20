// Integration with the REAL Box3D-backed IPhysicsWorld (core/physics adapter): validates that the 2026
// field geometry, fuel bodies, hub sensors and shooter behave in the actual engine and not only in the
// fakes. Tests may link anything (the plugin itself links only the SDK).
#include <doctest/doctest.h>

#include <cmath>
#include <memory>

#include "test_support.h"
#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/plugins/game_2026/fuel_zones.h"
#include "wpiemu/plugins/game_2026/game_state.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;

namespace {

// A session over the Box3D world: the same wiring as g26test::Session but without the fake physics.
struct Box3dRig {
    Box3dRig() {
        world = core::physics::createBox3dWorld();
        REQUIRE(world != nullptr);
        ctx = std::make_unique<sdk::SimContext>(
            sdk::SimContextServices{*world, clock, hub, logger, config, rng, ds, sdk::BodyId{}, nullptr, nullptr});
        sdk::BodyDesc chassisDesc;
        chassisDesc.type = sdk::BodyType::Kinematic;
        chassis = world->createBody(chassisDesc);
        ctx->bindChassisBody(chassis);
        auto f = Game2026Field::create(*world, buildFieldLayout());
        REQUIRE(f.ok());
        field = std::move(f).value();
    }
    ~Box3dRig() {
        state.reset();
        field.reset();
    }

    void makeState(const GameConfig& cfg) {
        state = std::make_unique<Game2026State>(*ctx, cfg, buildFieldLayout());
        state->reset(*ctx);
    }
    void tick(int n = 1) {
        for (int i = 0; i < n; ++i) {
            world->step(static_cast<float>(clock.fixedStepSeconds()));
            clock.advance();
            state->step(*ctx, clock.fixedStepSeconds());
        }
    }

    sdk::testing::FakeClock clock;
    sdk::testing::FakeDeviceHub hub;
    sdk::testing::CapturingLogger logger;
    sdk::RobotConfig config;
    sdk::Rng rng{1};
    sdk::proto::DsState ds;
    std::unique_ptr<sdk::IPhysicsWorld> world;
    std::unique_ptr<sdk::SimContext> ctx;
    sdk::BodyId chassis;
    std::unique_ptr<Game2026Field> field;
    std::unique_ptr<Game2026State> state;
};

GameConfig emptyConfig() {
    GameConfig c;
    c.neutralFuel = 0;
    c.depotFuel = 0;
    return c;
}

}  // namespace

TEST_CASE("Box3D: the field's surfaces are where the manual says they are") {
    Box3dRig rig;
    const std::uint64_t field = sdk::layer::Field;
    const float hubX = dim::kHubCenterXM, midY = dim::kFieldWidthM / 2;

    // carpet
    sdk::RayHit hit = rig.world->rayCastClosest({8.f, 4.f, 1.f}, {0.f, 0.f, -1.f}, 3.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.z == doctest::Approx(0.f).epsilon(0.01));

    // bump: ridge height and a 15 degree slope 0.4 m away from the ridge
    const float bumpY = midY - (dim::kHubSizeM / 2 + dim::kBumpWidthM / 2);
    hit = rig.world->rayCastClosest({hubX, bumpY, 1.f}, {0.f, 0.f, -1.f}, 3.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.z == doctest::Approx(dim::kBumpHeightM).epsilon(0.01));
    hit = rig.world->rayCastClosest({hubX + 0.4f, bumpY, 1.f}, {0.f, 0.f, -1.f}, 3.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.z == doctest::Approx(dim::kBumpHeightM - 0.4f * dim::kBumpRampSlope).epsilon(0.02));
    CHECK(std::fabs(hit.normal.x) == doctest::Approx(std::sin(15.f * 3.14159265f / 180.f)).epsilon(0.02));
    CHECK(hit.normal.z == doctest::Approx(std::cos(15.f * 3.14159265f / 180.f)).epsilon(0.01));

    // trench: free below 22.25 in, solid up to the 29.75 in arm top, free again in the tag band above it
    hit = rig.world->rayCastClosest({hubX - 2.f, 0.6f, 0.3f}, {1.f, 0.f, 0.f}, 4.f, field);
    CHECK_FALSE(hit.hit);
    hit = rig.world->rayCastClosest({hubX - 2.f, 0.6f, 0.65f}, {1.f, 0.f, 0.f}, 4.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.x == doctest::Approx(hubX - dim::kTrenchDepthM / 2).epsilon(0.01));
    hit = rig.world->rayCastClosest({hubX - 2.f, 0.6f, 0.8f}, {1.f, 0.f, 0.f}, 4.f, field);
    CHECK_FALSE(hit.hit);
    hit = rig.world->rayCastClosest({hubX, 0.6f, 2.f}, {0.f, 0.f, -1.f}, 3.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.z == doctest::Approx(dim::kTrenchArmTopM).epsilon(0.01));

    // hub: solid 47 in faces up to the 72 in rim, open on top
    hit = rig.world->rayCastClosest({hubX - 2.f, midY, 1.f}, {1.f, 0.f, 0.f}, 4.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.x == doctest::Approx(hubX - dim::kHubSizeM / 2).epsilon(0.005));
    hit = rig.world->rayCastClosest({hubX, midY, 3.f}, {0.f, 0.f, -1.f}, 5.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.z == doctest::Approx(dim::kHubBaseHeightM).epsilon(0.01));  // looks straight down the tube

    // tower: the high rung (63 in, 1.66 in pipe) lies along Y
    hit = rig.world->rayCastClosest({dim::kUprightCenterXM, dim::kTowerCenterYM, 3.f}, {0.f, 0.f, -1.f}, 5.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.z == doctest::Approx(dim::kHighRungZM + dim::kRungRadiusM).epsilon(0.01));
    hit = rig.world->rayCastClosest({dim::kUprightCenterXM, dim::kTowerCenterYM + 0.2f, 3.f}, {0.f, 0.f, -1.f}, 5.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.z == doctest::Approx(dim::kHighRungZM + dim::kRungRadiusM).epsilon(0.01));  // still the same pipe

    // the red side mirrors it: same tower rung seen from the other wall
    hit = rig.world->rayCastClosest({dim::kFieldLengthM - dim::kUprightCenterXM, dim::kFieldWidthM - dim::kTowerCenterYM, 3.f},
                                    {0.f, 0.f, -1.f}, 5.f, field);
    REQUIRE(hit.hit);
    CHECK(hit.point.z == doctest::Approx(dim::kHighRungZM + dim::kRungRadiusM).epsilon(0.01));
}

TEST_CASE("Box3D: staged fuel settles on the carpet, rests and stays put") {
    Box3dRig rig;
    GameConfig cfg;
    cfg.neutralFuel = 60;
    cfg.depotFuel = 24;
    rig.makeState(cfg);
    REQUIRE(rig.state->fuelOnField() == 60 + 48);
    rig.tick(600);  // 3 s
    CHECK(rig.state->fuelOnField() == 108);
    rig.state->pool().forEach([&](std::uint32_t, const sdk::Vec3& p) {
        CHECK(p.z == doctest::Approx(dim::kFuelRadiusM).epsilon(0.1));
        CHECK(p.x > -0.05f);
        CHECK(p.x < dim::kFieldLengthM + 0.05f);
    });
    CHECK(rig.world->stats().awakeBodies < rig.world->stats().bodies);  // resting fuel sleeps
}

TEST_CASE("Box3D: fuel dropped through the hub opening scores, fuel dropped beside it does not") {
    Box3dRig rig;
    GameConfig noRecycle = emptyConfig();
    noRecycle.recycleScoredFuel = false;
    rig.makeState(noRecycle);
    rig.ds.enabled = true;
    rig.ds.mode = sdk::proto::RobotMode::Autonomous;
    const FieldLayout layout = buildFieldLayout();
    const HubSpec& blue = layout.hub(Alliance::Blue);

    rig.state->pool().spawn({blue.center.x, blue.center.y, 2.6f}, {});
    rig.state->pool().spawn({blue.center.x + 1.5f, blue.center.y, 2.6f}, {});  // lands on the carpet, outside
    rig.tick(300);
    CHECK(rig.state->score(Alliance::Blue).points() == 1);
    CHECK(rig.state->score(Alliance::Red).points() == 0);
    CHECK(rig.state->fuelOnField() == 1);
}

namespace {

// Two fuel balls driven at each other along X; returns {left ball x, right ball x} after `ticks`.
std::pair<float, float> headOn(bool ghostLeft, bool ghostRight, int ticks) {
    Box3dRig rig;
    rig.makeState(emptyConfig());
    const sdk::SimTimeUs farFuture = 10'000'000;
    FuelPool::SpawnOptions left;
    left.velocity = {3.f, 0.f, 0.f};
    left.ghostUntilUs = ghostLeft ? farFuture : 0;
    FuelPool::SpawnOptions right;
    right.velocity = {-3.f, 0.f, 0.f};
    right.ghostUntilUs = ghostRight ? farFuture : 0;
    const std::uint32_t a = rig.state->pool().spawn({7.5f, 4.f, 1.f}, left);
    const std::uint32_t b = rig.state->pool().spawn({8.5f, 4.f, 1.f}, right);
    REQUIRE(a != 0);
    REQUIRE(b != 0);
    rig.tick(ticks);
    return {rig.state->pool().position(a).x, rig.state->pool().position(b).x};
}

}  // namespace

TEST_CASE("Box3D: just-shot fuel passes through other just-shot fuel but not through settled fuel") {
    const auto ghosts = headOn(true, true, 60);
    CHECK(ghosts.first > ghosts.second);  // swapped sides

    const auto solid = headOn(false, false, 60);
    CHECK(solid.first < solid.second);

    const auto mixed = headOn(true, false, 60);
    CHECK(mixed.first < mixed.second);
}

TEST_CASE("Box3D: just-shot fuel still scores in the hub") {
    Box3dRig rig;
    GameConfig noRecycle = emptyConfig();
    noRecycle.recycleScoredFuel = false;
    rig.makeState(noRecycle);
    rig.ds.enabled = true;
    rig.ds.mode = sdk::proto::RobotMode::Autonomous;
    const HubSpec& blue = buildFieldLayout().hub(Alliance::Blue);
    FuelPool::SpawnOptions ghost;
    ghost.ghostUntilUs = 10'000'000;
    rig.state->pool().spawn({blue.center.x, blue.center.y, 2.6f}, ghost);
    rig.tick(300);
    CHECK(rig.state->score(Alliance::Blue).points() == 1);
}

TEST_CASE("Box3D: a shot from the robot flies into the hub, scores, and is recycled out of an exit") {
    Box3dRig rig;
    GameConfig cfg = emptyConfig();
    cfg.hubProcessingDelayS = 0.2;
    // Fixed-speed shooter zone 2.0 m from the blue hub face; angle/speed solved for the opening (see docs).
    sdk::ZoneConfig zone;
    zone.id = "shooter";
    zone.kind = kShooterKind;
    zone.local.position = {0.25f, 0.f, 0.55f};
    zone.properties["fixedExitSpeedMps"] = 6.15;
    zone.properties["hoodAngleRad"] = 1.1345;  // 65 degrees
    zone.properties["fireRateHz"] = 5.0;
    rig.config.zones.push_back(zone);
    rig.makeState(cfg);  // the state builds its zones from the config
    rig.ds.enabled = true;
    rig.ds.mode = sdk::proto::RobotMode::Autonomous;

    const FieldLayout layout = buildFieldLayout();
    const HubSpec& blue = layout.hub(Alliance::Blue);
    rig.world->setPose(rig.chassis, sdk::Pose3{{2.0f, blue.center.y, 0.f}, sdk::Quat{}});
    for (int i = 0; i < 400; ++i) rig.tick();  // 2 s: about 8 shots at 5 Hz (the preload)
    CHECK(rig.state->shotsFired() >= 7);
    CHECK(rig.state->score(Alliance::Blue).points() >= 5);  // most of a perfect-repeat volley goes in
    rig.tick(400);
    // scored fuel came back out of the hub exits into the neutral zone
    int nearBlueExits = 0;
    rig.state->pool().forEach([&](std::uint32_t, const sdk::Vec3& p) {
        if (p.x > blue.center.x + 0.5f && p.x < blue.center.x + 4.f) ++nearBlueExits;
    });
    CHECK(nearBlueExits >= 3);
}
