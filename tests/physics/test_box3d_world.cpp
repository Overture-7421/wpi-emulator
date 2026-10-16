#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "physics_test_support.h"

using namespace physics_test;

namespace {

PhysicsConfig zeroG() {
    PhysicsConfig c;
    c.gravity = {0.f, 0.f, 0.f};
    return c;
}

void run(IPhysicsWorld& w, int steps) {
    for (int i = 0; i < steps; ++i) w.step(kDt);
}

}  // namespace

TEST_CASE("box3d world: creation, version and bad configuration") {
    auto w = makeWorld();
    REQUIRE(w);
    CHECK(phys::box3dVersionString().rfind("0.", 0) == 0);

    PhysicsConfig bad;
    bad.gravity = {0.f, 0.f, std::numeric_limits<float>::quiet_NaN()};
    CHECK(phys::createBox3dWorld(bad) == nullptr);

    auto viaFactory = phys::box3dWorldFactory()(PhysicsConfig{});
    CHECK(viaFactory != nullptr);
}

TEST_CASE("box3d world: a ball falls under Z-up gravity and rests on the floor") {
    auto w = makeWorld();
    addFloor(*w);
    const BodyId ball = addBall(*w, {0.f, 0.f, 1.0f});
    run(*w, 400);
    const Pose3 p = w->pose(ball);
    CHECK(p.position.z == doctest::Approx(0.075f).epsilon(0.05));
    CHECK(std::fabs(w->linearVelocity(ball).z) < 0.05f);
    CHECK(w->stats().bodies == 2);
}

TEST_CASE("box3d world: invalid ids are tolerated with neutral values") {
    auto w = makeWorld();
    const BodyId none;
    const ShapeId noShape;
    CHECK_FALSE(w->isValid(none));
    CHECK_FALSE(w->isValid(noShape));
    CHECK(w->pose(none) == Pose3{});
    CHECK(w->linearVelocity(none) == Vec3{});
    CHECK(w->angularVelocity(none) == Vec3{});
    CHECK(w->pointVelocity(none, {1.f, 2.f, 3.f}) == Vec3{});
    CHECK(w->shapeTag(noShape) == 0);
    CHECK_FALSE(w->shapeBody(noShape).valid());
    w->setPose(none, {});
    w->setLinearVelocity(none, {1.f, 0.f, 0.f});
    w->setAngularVelocity(none, {1.f, 0.f, 0.f});
    w->setEnabled(none, false);
    w->applyForce(none, {1.f, 0.f, 0.f}, {});
    w->applyForceToCenter(none, {1.f, 0.f, 0.f});
    w->applyTorque(none, {1.f, 0.f, 0.f});
    w->applyImpulse(none, {1.f, 0.f, 0.f}, {});
    w->destroyBody(none);
    w->destroyShape(noShape);
    w->setShapeFilter(noShape, 1, 1);
    w->setShapeMaterial(noShape, {});
    w->setMassProperties(none, {});
    ShapeDesc sd;
    CHECK_FALSE(w->addShape(none, sd).valid());

    BodyDesc bad;
    bad.pose.position.x = std::numeric_limits<float>::infinity();
    CHECK_FALSE(w->createBody(bad).valid());

    BodyDesc ok;
    ok.type = BodyType::Dynamic;
    const BodyId b = w->createBody(ok);
    REQUIRE(b.valid());
    ShapeDesc badShape;
    badShape.geometry = SphereShape{-1.f};
    CHECK_FALSE(w->addShape(b, badShape).valid());
    w->destroyBody(b);
    CHECK_FALSE(w->isValid(b));
    w->destroyBody(b);  // idempotent
    CHECK(w->pose(b) == Pose3{});
}

TEST_CASE("box3d world: destroying a body invalidates its shapes and removes it from the hash order") {
    auto w = makeWorld();
    const BodyId a = addBall(*w, {0.f, 0.f, 2.f});
    const BodyId b = addBall(*w, {1.f, 0.f, 2.f});
    ShapeDesc extra;
    extra.geometry = BoxShape{{0.1f, 0.1f, 0.1f}};
    extra.density = 10.f;
    const ShapeId s = w->addShape(b, extra);
    REQUIRE(s.valid());
    CHECK(w->shapeBody(s) == b);
    const std::uint64_t before = w->stateHash();
    w->destroyBody(b);
    CHECK_FALSE(w->isValid(s));
    CHECK(w->isValid(a));
    CHECK(w->stateHash() != before);
    CHECK(w->stats().bodies == 1);

    const ShapeId s2 = w->addShape(a, extra);
    REQUIRE(s2.valid());
    w->destroyShape(s2);
    CHECK_FALSE(w->isValid(s2));
    w->destroyShape(s2);
}

TEST_CASE("box3d world: category and mask filtering, also changed at runtime") {
    auto w = makeWorld();
    addFloor(*w);
    // Passes through the field: its mask excludes the Field category.
    const BodyId ghost = addBall(*w, {0.f, 0.f, 0.5f}, 0.075f, 113.2f, layer::Fuel, layer::Fuel);
    const BodyId solid = addBall(*w, {1.f, 0.f, 0.5f}, 0.075f, 113.2f, layer::Fuel, layer::All);
    run(*w, 300);
    CHECK(w->pose(ghost).position.z < -0.5f);
    CHECK(w->pose(solid).position.z == doctest::Approx(0.075f).epsilon(0.1));

    // Re-enable collisions on a fresh ball by editing the filter before it lands.
    const BodyId late = addBall(*w, {2.f, 0.f, 1.0f}, 0.075f, 113.2f, layer::Fuel, layer::Fuel);
    std::vector<ShapeId> hits;
    w->overlapSphere({2.f, 0.f, 1.0f}, 0.2f, layer::Fuel, hits);
    REQUIRE(hits.size() == 1);
    CHECK(w->shapeBody(hits[0]) == late);
    w->setShapeFilter(hits[0], layer::Fuel, layer::All);
    run(*w, 300);
    CHECK(w->pose(late).position.z == doctest::Approx(0.075f).epsilon(0.1));
}

TEST_CASE("box3d world: ray casts return point, normal, fraction, tag and honour masks") {
    auto w = makeWorld();
    addStaticBox(*w, {0.f, 0.f, -1.f}, {10.f, 10.f, 1.f}, Quat::identity(), 77);
    const RayHit hit = w->rayCastClosest({1.f, 2.f, 0.5f}, {0.f, 0.f, -1.f}, 2.f, layer::Field);
    REQUIRE(hit.hit);
    CHECK(hit.point.x == doctest::Approx(1.f));
    CHECK(hit.point.y == doctest::Approx(2.f));
    CHECK(hit.point.z == doctest::Approx(0.f).epsilon(0.001));
    CHECK(hit.normal.z == doctest::Approx(1.f));
    CHECK(hit.fraction == doctest::Approx(0.25f));
    CHECK(hit.userTag == 77);
    CHECK(w->shapeTag(hit.shape) == 77);

    CHECK_FALSE(w->rayCastClosest({1.f, 2.f, 0.5f}, {0.f, 0.f, -1.f}, 0.2f, layer::Field).hit);  // too short
    CHECK_FALSE(w->rayCastClosest({1.f, 2.f, 0.5f}, {0.f, 0.f, -1.f}, 2.f, layer::Fuel).hit);     // mask
    CHECK_FALSE(w->rayCastClosest({1.f, 2.f, 0.5f}, {0.f, 0.f, 0.f}, 2.f, layer::Field).hit);     // no direction
    CHECK(w->rayCastClosest({1.f, 2.f, 0.5f}, {0.f, 0.f, -10.f}, 2.f, layer::Field).hit);        // not normalised
}

TEST_CASE("box3d world: sensor events carry tags, begin precedes end, shapesOverlapping lists visitors") {
    auto w = makeWorld(zeroG());
    BodyDesc sbd;
    sbd.type = BodyType::Static;
    sbd.pose.position = {0.f, 0.f, 0.f};
    const BodyId sensorBody = w->createBody(sbd);
    ShapeDesc sensor;
    sensor.geometry = BoxShape{{1.f, 1.f, 1.f}};
    sensor.sensor = true;
    sensor.sensorEvents = true;
    sensor.category = layer::Sensor;
    sensor.mask = layer::All;
    sensor.userTag = 1001;
    const ShapeId sensorShape = w->addShape(sensorBody, sensor);
    REQUIRE(sensorShape.valid());

    const BodyId ball = addBall(*w, {-3.f, 0.f, 0.f}, 0.1f, 100.f, layer::Fuel, layer::All, 2002);
    w->setLinearVelocity(ball, {2.f, 0.f, 0.f});

    bool began = false, ended = false;
    std::vector<ShapeId> inside;
    for (int i = 0; i < 600; ++i) {
        w->step(kDt);
        for (const SensorEvent& e : w->sensorEvents()) {
            CHECK(e.sensorTag == 1001);
            CHECK(e.visitorTag == 2002);
            CHECK(e.sensor == sensorShape);
            if (e.begin) {
                CHECK_FALSE(began);
                began = true;
                CHECK_FALSE(ended);
            } else {
                CHECK(began);
                ended = true;
            }
        }
        if (began && !ended && inside.empty()) w->shapesOverlapping(sensorShape, inside);
    }
    CHECK(began);
    CHECK(ended);
    REQUIRE(inside.size() == 1);
    CHECK(w->shapeTag(inside[0]) == 2002);
}

TEST_CASE("box3d world: a visitor whose mask lacks the sensor category produces no events") {
    auto w = makeWorld(zeroG());
    BodyDesc sbd;
    const BodyId sensorBody = w->createBody(sbd);
    ShapeDesc sensor;
    sensor.geometry = BoxShape{{1.f, 1.f, 1.f}};
    sensor.sensor = true;
    sensor.sensorEvents = true;
    sensor.category = layer::Sensor;
    w->addShape(sensorBody, sensor);
    const BodyId ball = addBall(*w, {0.f, 0.f, 0.f}, 0.1f, 100.f, layer::Fuel, layer::Field);
    (void)ball;
    int events = 0;
    for (int i = 0; i < 20; ++i) {
        w->step(kDt);
        events += static_cast<int>(w->sensorEvents().size());
    }
    CHECK(events == 0);
}

TEST_CASE("box3d world: sensors on dynamic bodies move with the body") {
    auto w = makeWorld(zeroG());
    BodyDesc bd;
    bd.type = BodyType::Dynamic;
    bd.pose.position = {0.f, 0.f, 0.f};
    bd.allowSleep = false;
    const BodyId carrier = w->createBody(bd);
    ShapeDesc sensor;
    sensor.geometry = BoxShape{{0.3f, 0.3f, 0.3f}};
    sensor.local.position = {1.f, 0.f, 0.f};
    sensor.sensor = true;
    sensor.sensorEvents = true;
    sensor.category = layer::Sensor;
    sensor.density = 1.f;
    const ShapeId zone = w->addShape(carrier, sensor);
    REQUIRE(zone.valid());
    addBall(*w, {1.f, 0.f, 0.f}, 0.05f, 10.f, layer::Fuel, layer::All);  // inside the zone
    w->step(kDt);
    w->step(kDt);
    std::vector<ShapeId> contents;
    w->shapesOverlapping(zone, contents);
    CHECK(contents.size() == 1);
    // Move the carrier away: the ball leaves the zone.
    w->setPose(carrier, Pose3{{0.f, 5.f, 0.f}, Quat::identity()});
    w->step(kDt);
    contents.clear();
    w->shapesOverlapping(zone, contents);
    CHECK(contents.empty());
}

TEST_CASE("box3d world: contact hit events report tags and approach speed") {
    auto w = makeWorld();
    addStaticBox(*w, {0.f, 0.f, -1.f}, {10.f, 10.f, 1.f}, Quat::identity(), 5);
    addBall(*w, {0.f, 0.f, 2.f}, 0.1f, 100.f, layer::Fuel, layer::All, 6);
    bool seen = false;
    for (int i = 0; i < 300 && !seen; ++i) {
        w->step(kDt);
        for (const ContactEvent& e : w->contactEvents()) {
            const bool ballFirst = e.tagA == 6;
            CHECK(((e.tagA == 6 && e.tagB == 5) || (e.tagA == 5 && e.tagB == 6)));
            CHECK(e.approachSpeed > 3.f);
            CHECK(std::fabs(e.normal.z) > 0.9f);
            CHECK(ballFirst == (e.tagA == 6));
            seen = true;
        }
    }
    CHECK(seen);
}

TEST_CASE("box3d world: forces last one step, point forces create torque, impulses change velocity") {
    auto w = makeWorld(zeroG());
    const BodyId b = addBall(*w, {0.f, 0.f, 0.f}, 0.1f, 100.f);
    w->setMassProperties(b, MassProperties{2.f, {}, {0.1f, 0.1f, 0.1f}});
    // F = 10 N for 1 s on 2 kg => 5 m/s, then coasting.
    for (int i = 0; i < 200; ++i) {
        w->applyForceToCenter(b, {10.f, 0.f, 0.f});
        w->step(kDt);
    }
    CHECK(w->linearVelocity(b).x == doctest::Approx(5.f).epsilon(0.02));
    run(*w, 100);
    CHECK(w->linearVelocity(b).x == doctest::Approx(5.f).epsilon(0.02));

    // Off-centre force: torque = r x F = (0,1,0) x (10,0,0)... r = +y, F = +x => torque -z.
    w->setLinearVelocity(b, {});
    w->applyForce(b, {10.f, 0.f, 0.f}, {0.f, 1.f, 0.f});
    w->step(kDt);
    CHECK(w->angularVelocity(b).z < -0.3f);
    CHECK(w->linearVelocity(b).x > 0.f);

    // Pure torque
    w->setLinearVelocity(b, {});
    w->setAngularVelocity(b, {});
    w->applyTorque(b, {0.f, 0.f, 1.f});
    w->step(kDt);
    CHECK(w->angularVelocity(b).z == doctest::Approx(kDt / 0.1f).epsilon(0.05));

    // Impulse at the centre: dv = J / m.
    w->setAngularVelocity(b, {});
    w->applyImpulse(b, {4.f, 0.f, 0.f}, w->worldPoint(b, {}));
    CHECK(w->linearVelocity(b).x == doctest::Approx(2.f).epsilon(0.02));
    CHECK(w->pointVelocity(b, w->worldPoint(b, {0.f, 1.f, 0.f})).x == doctest::Approx(2.f).epsilon(0.05));
}

TEST_CASE("box3d world: explicit mass properties survive adding shapes later") {
    auto w = makeWorld(zeroG());
    BodyDesc bd;
    bd.type = BodyType::Dynamic;
    bd.allowSleep = false;
    const BodyId b = w->createBody(bd);
    ShapeDesc s;
    s.geometry = BoxShape{{0.2f, 0.2f, 0.2f}};
    w->addShape(b, s);
    w->setMassProperties(b, MassProperties{10.f, {}, {1.f, 1.f, 1.f}});
    ShapeDesc pad;
    pad.geometry = SphereShape{0.1f};
    w->addShape(b, pad);  // zero density: must not reset the override
    w->applyForceToCenter(b, {10.f, 0.f, 0.f});
    w->step(0.1f);
    CHECK(w->linearVelocity(b).x == doctest::Approx(0.1f).epsilon(0.02));  // a = 1 m/s^2

    // Invalid mass data is rejected, the previous one stays.
    w->setMassProperties(b, MassProperties{-1.f, {}, {1.f, 1.f, 1.f}});
    w->setLinearVelocity(b, {});
    w->applyForceToCenter(b, {10.f, 0.f, 0.f});
    w->step(0.1f);
    CHECK(w->linearVelocity(b).x == doctest::Approx(0.1f).epsilon(0.02));
}

TEST_CASE("box3d world: a mass centre offset makes a centre force produce a turning moment about the origin pose") {
    auto w = makeWorld(zeroG());
    BodyDesc bd;
    bd.type = BodyType::Dynamic;
    bd.allowSleep = false;
    const BodyId b = w->createBody(bd);
    ShapeDesc s;
    s.geometry = BoxShape{{0.2f, 0.2f, 0.2f}};
    w->addShape(b, s);
    w->setMassProperties(b, MassProperties{10.f, {0.5f, 0.f, 0.f}, {1.f, 1.f, 1.f}});
    // Force through the body origin (not the COM) at +y produces torque about the COM at x = 0.5.
    w->applyForce(b, {0.f, 10.f, 0.f}, w->worldPoint(b, {}));
    w->step(kDt);
    CHECK(w->angularVelocity(b).z < 0.f);
}

TEST_CASE("box3d world: static triangle meshes collide, give slope normals and are refused on dynamic bodies") {
    auto w = makeWorld();
    // 10x10 m plane descending towards +X at 20 degrees (two triangles, normal tilted towards +X).
    const float t = std::tan(20.f * kPiF / 180.f);
    const float xyz[] = {-5.f, -5.f, 5.f * t, 5.f, -5.f, -5.f * t, 5.f, 5.f, -5.f * t, -5.f, 5.f, 5.f * t};
    const std::uint32_t tri[] = {0, 1, 2, 0, 2, 3};
    BodyDesc bd;
    bd.type = BodyType::Static;
    const BodyId ground = w->createBody(bd);
    ShapeDesc md;
    md.geometry = MeshShape{xyz, 4, tri, 2};
    md.category = layer::Field;
    md.userTag = 9;
    const ShapeId meshShape = w->addShape(ground, md);
    REQUIRE(meshShape.valid());

    const RayHit hit = w->rayCastClosest({0.f, 0.f, 3.f}, {0.f, 0.f, -1.f}, 6.f, layer::Field);
    REQUIRE(hit.hit);
    CHECK(hit.userTag == 9);
    CHECK(std::fabs(hit.normal.x) == doctest::Approx(std::sin(20.f * kPiF / 180.f)).epsilon(0.02));
    CHECK(hit.normal.z > 0.9f);

    const BodyId ball = addBall(*w, {0.f, 0.f, hit.point.z + 0.5f});
    const float xStart = w->pose(ball).position.x;
    run(*w, 200);
    CHECK(std::fabs(w->pose(ball).position.x - xStart) > 0.3f);  // rolled down the slope

    BodyDesc dyn;
    dyn.type = BodyType::Dynamic;
    const BodyId d = w->createBody(dyn);
    CHECK_FALSE(w->addShape(d, md).valid());
    ShapeDesc bad = md;
    bad.geometry = MeshShape{xyz, 4, tri, 0};
    CHECK_FALSE(w->addShape(ground, bad).valid());
    const std::uint32_t outOfRange[] = {0, 1, 9};
    bad.geometry = MeshShape{xyz, 4, outOfRange, 1};
    CHECK_FALSE(w->addShape(ground, bad).valid());

    w->destroyBody(ground);  // releases the mesh data with the shape
    CHECK_FALSE(w->isValid(meshShape));
}

TEST_CASE("box3d world: cylinders stand on their local Z axis") {
    auto w = makeWorld();
    addFloor(*w);
    BodyDesc bd;
    bd.type = BodyType::Dynamic;
    bd.pose.position = {0.f, 0.f, 0.5f};
    bd.allowSleep = false;
    const BodyId b = w->createBody(bd);
    ShapeDesc cyl;
    cyl.geometry = CylinderShape{0.3f, 0.1f};  // wide and flat: only stands on its Z axis when oriented right
    cyl.density = 200.f;
    cyl.material = {0.6f, 0.f, 0.f};
    REQUIRE(w->addShape(b, cyl).valid());
    run(*w, 400);
    CHECK(w->pose(b).position.z == doctest::Approx(0.1f).epsilon(0.1));
}

TEST_CASE("box3d world: kinematic bodies follow their velocity and setEnabled(false) freezes a body") {
    auto w = makeWorld(zeroG());
    BodyDesc kd;
    kd.type = BodyType::Kinematic;
    kd.linearVelocity = {1.f, 0.f, 0.f};
    const BodyId k = w->createBody(kd);
    ShapeDesc s;
    s.geometry = BoxShape{{0.1f, 0.1f, 0.1f}};
    w->addShape(k, s);
    run(*w, 200);
    CHECK(w->pose(k).position.x == doctest::Approx(1.f).epsilon(0.02));

    const BodyId ball = addBall(*w, {0.f, 5.f, 0.f});
    w->setLinearVelocity(ball, {1.f, 0.f, 0.f});
    w->setEnabled(ball, false);
    run(*w, 100);
    CHECK(std::fabs(w->pose(ball).position.x) < 1e-4f);
    w->setEnabled(ball, true);
    w->setLinearVelocity(ball, {1.f, 0.f, 0.f});
    run(*w, 100);
    CHECK(w->pose(ball).position.x > 0.3f);
}

TEST_CASE("box3d world: setPose teleports and normalises the rotation") {
    auto w = makeWorld(zeroG());
    const BodyId b = addBall(*w, {0.f, 0.f, 0.f});
    w->setPose(b, Pose3{{1.f, 2.f, 3.f}, Quat{0.f, 0.f, 2.f, 2.f}});  // 90 degrees about Z, not unit length
    const Pose3 p = w->pose(b);
    CHECK(p.position.y == doctest::Approx(2.f));
    CHECK(p.rotation.yaw() == doctest::Approx(0.5f * kPiF).epsilon(0.001));
    const Vec3 wp = w->worldPoint(b, {1.f, 0.f, 0.f});
    CHECK(wp.x == doctest::Approx(1.f).epsilon(0.001));
    CHECK(wp.y == doctest::Approx(3.f).epsilon(0.001));
}

TEST_CASE("box3d world: overlapSphere appends masked shapes") {
    auto w = makeWorld(zeroG());
    addBall(*w, {0.f, 0.f, 0.f}, 0.1f, 100.f, layer::Fuel);
    addBall(*w, {0.3f, 0.f, 0.f}, 0.1f, 100.f, layer::Projectile);
    addBall(*w, {5.f, 0.f, 0.f}, 0.1f, 100.f, layer::Fuel);
    std::vector<ShapeId> out;
    w->overlapSphere({0.f, 0.f, 0.f}, 0.5f, layer::All, out);
    CHECK(out.size() == 2);
    w->overlapSphere({0.f, 0.f, 0.f}, 0.5f, layer::Fuel, out);  // appends, does not clear
    CHECK(out.size() == 3);
}

TEST_CASE("box3d world: stats are populated and a zero or negative step is ignored") {
    auto w = makeWorld();
    addFloor(*w);
    addBall(*w, {0.f, 0.f, 1.f});
    w->step(0.f);
    w->step(-1.f);
    CHECK(w->pose(BodyId{}).position.z == 0.f);
    w->step(kDt);
    const PhysicsStats s = w->stats();
    CHECK(s.bodies == 2);
    CHECK(s.awakeBodies >= 1);
    CHECK(s.lastStepMs >= 0.0);
}

TEST_CASE("box3d world: stateHash is reproducible and sensitive to state") {
    auto scenario = [](float vx) {
        auto w = makeWorld();
        addFloor(*w);
        for (int i = 0; i < 20; ++i) addBall(*w, {0.1f * static_cast<float>(i % 5), 0.1f * static_cast<float>(i / 5), 0.3f + 0.2f * static_cast<float>(i)});
        const BodyId pusher = addBall(*w, {-2.f, 0.f, 0.075f});
        w->setLinearVelocity(pusher, {vx, 0.f, 0.f});
        run(*w, 300);
        return w->stateHash();
    };
    CHECK(scenario(3.f) == scenario(3.f));
    CHECK(scenario(3.f) != scenario(3.5f));
}

TEST_CASE("box3d world: results are bit-identical for any worker count") {
    auto scenario = [](int workers) {
        PhysicsConfig cfg;
        cfg.workerCount = workers;
        auto w = makeWorld(cfg);
        addFloor(*w);
        addStaticBox(*w, {2.f, 0.f, 0.25f}, {0.1f, 1.5f, 0.25f});
        for (int i = 0; i < 80; ++i)
            addBall(*w, {0.12f * static_cast<float>(i % 8), 0.12f * static_cast<float>((i / 8) % 5), 0.3f + 0.16f * static_cast<float>(i / 40)});
        const BodyId pusher = addBall(*w, {-2.f, 0.3f, 0.075f});
        w->setLinearVelocity(pusher, {4.f, 0.f, 0.f});
        run(*w, 600);
        return w->stateHash();
    };
    const std::uint64_t h1 = scenario(1);
    CHECK(h1 == scenario(1));
    CHECK(h1 == scenario(2));
    CHECK(h1 == scenario(4));
}
