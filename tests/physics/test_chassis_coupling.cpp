#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "physics_test_support.h"
#include "wpiemu/core/physics/chassis_coupling.h"
#include "wpiemu/core/physics/drivetrains.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

using namespace physics_test;

namespace {

PhysicsConfig zeroG() {
    PhysicsConfig c;
    c.gravity = {0.f, 0.f, 0.f};
    return c;
}

BodyId makeBox(IPhysicsWorld& w, Vec3 at, float mass, Quat rot = Quat::identity()) {
    BodyDesc bd;
    bd.type = BodyType::Dynamic;
    bd.pose = Pose3{at, rot};
    bd.allowSleep = false;
    const BodyId b = w.createBody(bd);
    ShapeDesc sd;
    sd.geometry = BoxShape{{0.2f, 0.2f, 0.2f}};
    sd.category = layer::Robot;
    sd.mask = layer::Field;  // never collides with the test anchors
    w.addShape(b, sd);
    w.setMassProperties(b, MassProperties{mass, {}, {1.f, 1.f, 1.f}});
    return b;
}

MassProperties massOf(float kg) { return MassProperties{kg, {}, {1.f, 1.f, 1.f}}; }

// Steps the world and lets the coupling see the completed step, like ChassisSimulator does.
void stepWith(IPhysicsWorld& w, phys::ChassisPhysicsCoupling& c, int n = 1, float dt = kDt) {
    for (int i = 0; i < n; ++i) {
        w.step(dt);
        c.onStepCompleted();
    }
}

struct MassSpy final : phys::IChassisMassAware {
    int calls = 0;
    MassProperties last;
    void setChassisMass(const MassProperties& m) override {
        ++calls;
        last = m;
    }
};

}  // namespace

TEST_CASE("coupling: capabilities are honest about joint support") {
    auto box3d = makeWorld(zeroG());
    const BodyId b = makeBox(*box3d, {0.f, 0.f, 0.f}, 10.f);
    phys::ChassisPhysicsCoupling full(*box3d, b, massOf(10.f));
    const CouplingCapabilities caps = full.capabilities();
    CHECK(caps.wrench);
    CHECK(caps.payload);
    CHECK(caps.constraints);

    sdk::testing::FakePhysicsWorld fake;
    const BodyId fb = fake.createBody(BodyDesc{BodyType::Dynamic, {}, {}, true, false, 0});
    phys::ChassisPhysicsCoupling limited(fake, fb, massOf(10.f));
    CHECK(limited.capabilities().wrench);
    CHECK(limited.capabilities().payload);
    CHECK_FALSE(limited.capabilities().constraints);
    AnchorConstraintDesc d;
    d.kind = AnchorKind::Distance;
    CHECK_FALSE(limited.addConstraint(d).valid());
    CHECK_FALSE(limited.constraintLoad(ConstraintHandle{1}).valid);
}

TEST_CASE("coupling: a wrench is converted from the chassis frame and lasts one step") {
    auto w = makeWorld(zeroG());
    const Quat yaw90 = Quat::fromYaw(0.5f * kPiF);
    const BodyId b = makeBox(*w, {0.f, 0.f, 0.f}, 10.f, yaw90);
    phys::ChassisPhysicsCoupling c(*w, b, massOf(10.f));

    // Chassis +x is world +y after the 90 degree yaw. 10 N for 0.5 s on 10 kg => 0.5 m/s.
    for (int i = 0; i < 100; ++i) {
        c.applyWrench({0.f, 0.f, 0.f}, Wrench{{10.f, 0.f, 0.f}, {}});
        stepWith(*w, c);
    }
    CHECK(w->linearVelocity(b).y == doctest::Approx(0.5f).epsilon(0.02));
    CHECK(std::fabs(w->linearVelocity(b).x) < 1e-3f);
    stepWith(*w, c, 50);  // nothing re-submitted: velocity unchanged
    CHECK(w->linearVelocity(b).y == doctest::Approx(0.5f).epsilon(0.02));
    CHECK(std::fabs(w->angularVelocity(b).z) < 1e-3f);
}

TEST_CASE("coupling: force at an offset point and a pure couple create the expected moments") {
    auto w = makeWorld(zeroG());
    const BodyId b = makeBox(*w, {0.f, 0.f, 0.f}, 10.f);
    phys::ChassisPhysicsCoupling c(*w, b, massOf(10.f));

    c.applyWrench({0.f, 1.f, 0.f}, Wrench{{10.f, 0.f, 0.f}, {}});  // r x F = (0,1,0) x (10,0,0) = -10 z
    stepWith(*w, c);
    CHECK(w->angularVelocity(b).z == doctest::Approx(-10.f * kDt).epsilon(0.05));

    w->setAngularVelocity(b, {});
    w->setLinearVelocity(b, {});
    c.applyWrench({0.f, 0.f, 0.f}, Wrench{{}, {0.f, 0.f, 5.f}});
    stepWith(*w, c);
    CHECK(w->angularVelocity(b).z == doctest::Approx(5.f * kDt).epsilon(0.05));

    // Several wrenches in one tick add up.
    w->setAngularVelocity(b, {});
    c.applyWrench({}, Wrench{{3.f, 0.f, 0.f}, {}});
    c.applyWrench({}, Wrench{{3.f, 0.f, 0.f}, {}});
    stepWith(*w, c);
    CHECK(w->linearVelocity(b).x == doctest::Approx(6.f * kDt / 10.f).epsilon(0.05));
}

TEST_CASE("coupling: non-finite wrenches are ignored") {
    auto w = makeWorld(zeroG());
    const BodyId b = makeBox(*w, {0.f, 0.f, 0.f}, 10.f);
    phys::ChassisPhysicsCoupling c(*w, b, massOf(10.f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    c.applyWrench({0.f, 0.f, 0.f}, Wrench{{nan, 0.f, 0.f}, {}});
    c.applyWrench({nan, 0.f, 0.f}, Wrench{{1.f, 0.f, 0.f}, {}});
    c.applyWrench({}, Wrench{{}, {0.f, nan, 0.f}});
    stepWith(*w, c);
    CHECK(length(w->linearVelocity(b)) == 0.f);
    CHECK(std::isfinite(w->pose(b).position.x));
}

TEST_CASE("coupling: payloads compose mass, COM and inertia and notify the mass listener") {
    auto w = makeWorld(zeroG());
    const BodyId b = makeBox(*w, {0.f, 0.f, 0.f}, 10.f);
    phys::ChassisPhysicsCoupling c(*w, b, massOf(10.f));
    MassSpy spy;
    c.setMassListener(&spy);
    CHECK(spy.calls == 1);
    CHECK(spy.last.massKg == 10.f);

    PayloadMass carriage;
    carriage.massKg = 5.f;
    carriage.comChassisLocal = {1.f, 0.f, 0.f};
    carriage.inertiaDiagAboutComKgM2 = {0.1f, 0.1f, 0.1f};
    c.setPayload("elevator", carriage);
    CHECK(spy.calls == 2);
    CHECK(c.composedMass().massKg == 15.f);
    CHECK(c.composedMass().comLocal.x == doctest::Approx(5.f / 15.f));
    const PayloadMass list[] = {carriage};
    const MassProperties expected = composeMassProperties(massOf(10.f), list);
    CHECK(c.composedMass().inertiaDiagLocal == expected.inertiaDiagLocal);
    CHECK(spy.last.massKg == 15.f);

    // The world now accelerates the heavier body less.
    w->applyForceToCenter(b, {15.f, 0.f, 0.f});
    w->step(0.1f);
    CHECK(w->linearVelocity(b).x == doctest::Approx(0.1f).epsilon(0.03));  // 15 N / 15 kg

    c.setPayload("elevator", carriage);  // unchanged: no recomposition
    CHECK(spy.calls == 2);

    PayloadMass second = carriage;
    second.massKg = 5.f;
    second.comChassisLocal = {-1.f, 0.f, 0.f};
    c.setPayload("intake", second);
    CHECK(c.composedMass().massKg == 20.f);
    CHECK(c.composedMass().comLocal.x == doctest::Approx(0.f).epsilon(1e-4));

    carriage.massKg = 0.f;  // zero mass clears
    c.setPayload("elevator", carriage);
    CHECK(c.composedMass().massKg == 15.f);
    c.clearPayload("intake");
    CHECK(c.composedMass().massKg == 10.f);
    c.clearPayload("intake");  // idempotent
    CHECK(c.composedMass().massKg == 10.f);
    CHECK(c.composedMass().comLocal == Vec3{});

    PayloadMass bad = second;
    bad.comChassisLocal.x = std::numeric_limits<float>::quiet_NaN();
    c.setPayload("bad", bad);
    CHECK(c.composedMass().massKg == 10.f);
}

TEST_CASE("coupling: a rigid distance constraint carries the weight and reports the reaction") {
    auto w = makeWorld();
    const BodyId b = makeBox(*w, {0.f, 0.f, 2.f}, 10.f);
    phys::ChassisPhysicsCoupling c(*w, b, massOf(10.f));
    AnchorConstraintDesc d;
    d.kind = AnchorKind::Distance;
    d.owner = "climber";
    d.chassisLocalPoint = {0.f, 0.f, 0.f};
    d.fieldPoint = {0.f, 0.f, 3.f};
    d.lengthM = 1.f;
    const ConstraintHandle h = c.addConstraint(d);
    REQUIRE(h.valid());
    CHECK_FALSE(c.constraintLoad(h).valid);  // no step completed yet
    stepWith(*w, c, 400);
    const ConstraintLoad load = c.constraintLoad(h);
    REQUIRE(load.valid);
    CHECK(load.engaged);
    CHECK(load.force.z == doctest::Approx(98.1f).epsilon(0.05));  // pulls the chassis up
    CHECK(std::fabs(load.force.x) < 5.f);
    CHECK(load.forceMagnitudeN == doctest::Approx(98.1f).epsilon(0.05));
    CHECK(load.travelM == doctest::Approx(1.f).epsilon(0.03));
    CHECK(w->pose(b).position.z == doctest::Approx(2.f).epsilon(0.03));

    c.removeConstraint(h);
    c.removeConstraint(h);  // idempotent
    CHECK_FALSE(c.constraintLoad(h).valid);
    stepWith(*w, c, 100);
    CHECK(w->pose(b).position.z < 1.5f);  // free fall again
}

TEST_CASE("coupling: an anchor lets go above its maximum force") {
    auto w = makeWorld();
    const BodyId b = makeBox(*w, {0.f, 0.f, 2.f}, 10.f);
    phys::ChassisPhysicsCoupling c(*w, b, massOf(10.f));
    AnchorConstraintDesc d;
    d.kind = AnchorKind::ContactAnchor;
    d.chassisLocalPoint = {0.f, 0.f, 1.f};
    d.fieldPoint = {0.f, 0.f, 3.f};
    d.maxForceN = 50.f;  // the weight is 98 N
    const ConstraintHandle h = c.addConstraint(d);
    REQUIRE(h.valid());
    stepWith(*w, c, 50);
    const ConstraintLoad load = c.constraintLoad(h);
    CHECK(load.valid);
    CHECK_FALSE(load.engaged);
    stepWith(*w, c, 150);
    CHECK(w->pose(b).position.z < 1.0f);
    CHECK_FALSE(c.constraintLoad(h).engaged);

    // A strong enough grip holds: pinned point stays, force ~ weight.
    auto w2 = makeWorld();
    const BodyId b2 = makeBox(*w2, {0.f, 0.f, 2.f}, 10.f);
    phys::ChassisPhysicsCoupling c2(*w2, b2, massOf(10.f));
    d.maxForceN = 500.f;
    const ConstraintHandle h2 = c2.addConstraint(d);
    REQUIRE(h2.valid());
    stepWith(*w2, c2, 400);
    CHECK(c2.constraintLoad(h2).engaged);
    CHECK(c2.constraintLoad(h2).forceMagnitudeN == doctest::Approx(98.1f).epsilon(0.1));
    CHECK(w2->pose(b2).position.z == doctest::Approx(2.f).epsilon(0.05));
}

TEST_CASE("coupling: a prismatic constraint slides along its axis between the limits and locks the rest") {
    auto w = makeWorld();
    const BodyId b = makeBox(*w, {0.f, 0.f, 0.f}, 10.f);
    phys::ChassisPhysicsCoupling c(*w, b, massOf(10.f));
    AnchorConstraintDesc d;
    d.kind = AnchorKind::Prismatic;
    d.chassisLocalPoint = {0.f, 0.f, 0.f};
    d.fieldPoint = {0.f, 0.f, 0.f};
    d.axisWorld = {0.f, 0.f, 5.f};  // not unit length on purpose
    d.minTravelM = 0.f;
    d.maxTravelM = 1.f;
    const ConstraintHandle h = c.addConstraint(d);
    REQUIRE(h.valid());

    // Gravity pulls it onto the lower limit; the constraint carries the weight.
    stepWith(*w, c, 300);
    CHECK(c.constraintLoad(h).travelM == doctest::Approx(0.f).epsilon(0.05));
    CHECK(c.constraintLoad(h).force.z == doctest::Approx(98.1f).epsilon(0.05));

    // Sideways pushes and torques are resisted.
    for (int i = 0; i < 100; ++i) {
        c.applyWrench({}, Wrench{{50.f, 20.f, 0.f}, {5.f, 5.f, 5.f}});
        stepWith(*w, c);
    }
    CHECK(std::fabs(w->pose(b).position.x) < 0.03f);
    CHECK(std::fabs(w->pose(b).position.y) < 0.03f);
    CHECK(std::fabs(w->angularVelocity(b).z) < 0.2f);

    // Lift it to the upper limit.
    for (int i = 0; i < 400; ++i) {
        c.applyWrench({}, Wrench{{0.f, 0.f, 300.f}, {}});
        stepWith(*w, c);
    }
    CHECK(c.constraintLoad(h).travelM == doctest::Approx(1.f).epsilon(0.05));
    CHECK(w->pose(b).position.z < 1.05f);
}

TEST_CASE("coupling: unusable constraint descriptions are rejected and reset drops everything") {
    auto w = makeWorld();
    const BodyId b = makeBox(*w, {0.f, 0.f, 2.f}, 10.f);
    phys::ChassisPhysicsCoupling c(*w, b, massOf(10.f));
    const float nan = std::numeric_limits<float>::quiet_NaN();

    AnchorConstraintDesc d;
    d.kind = AnchorKind::Prismatic;
    d.axisWorld = {0.f, 0.f, 0.f};
    CHECK_FALSE(c.addConstraint(d).valid());  // zero axis
    d.axisWorld = {0.f, 0.f, 1.f};
    d.fieldPoint = {nan, 0.f, 0.f};
    CHECK_FALSE(c.addConstraint(d).valid());
    d.fieldPoint = {0.f, 0.f, 3.f};
    d.maxForceN = 0.f;
    CHECK_FALSE(c.addConstraint(d).valid());
    d.maxForceN = std::numeric_limits<float>::infinity();
    d.kind = AnchorKind::Distance;
    d.lengthM = nan;
    CHECK_FALSE(c.addConstraint(d).valid());
    CHECK(c.constraintCount() == 0);

    d.lengthM = -1.f;  // distance at registration
    const ConstraintHandle h = c.addConstraint(d);
    REQUIRE(h.valid());
    CHECK(c.constraintCount() == 1);

    PayloadMass p;
    p.massKg = 3.f;
    c.setPayload("x", p);
    CHECK(c.composedMass().massKg == 13.f);
    c.reset();
    CHECK(c.constraintCount() == 0);
    CHECK(c.composedMass().massKg == 10.f);
    CHECK_FALSE(c.constraintLoad(h).valid);
    stepWith(*w, c, 100);
    CHECK(w->pose(b).position.z < 1.5f);  // the dropped constraint no longer holds the body
}

TEST_CASE("coupling: payloads flow into the drivetrain's mass and load distribution") {
    Rig rig(swerveSpec());
    REQUIRE(rig.sim);
    phys::ChassisPhysicsCoupling& coupling = rig.sim->coupling();
    CHECK(coupling.capabilities().constraints);

    PayloadMass battery;
    battery.massKg = 30.f;
    battery.comChassisLocal = {0.3f, 0.f, -0.1f};
    battery.inertiaDiagAboutComKgM2 = {0.2f, 0.2f, 0.2f};
    coupling.setPayload("front-load", battery);
    const auto* drive = dynamic_cast<const phys::TractionDrivetrain*>(&rig.sim->drivetrain());
    REQUIRE(drive != nullptr);
    CHECK(drive->chassisMass().massKg == 80.f);
    CHECK(drive->chassisMass().comLocal.x == doctest::Approx(30.f * 0.3f / 80.f));

    rig.run(1.0f);
    const auto s = rig.sim->drivetrain().wheelStates();  // fl fr rl rr
    CHECK(s[0].normalLoadN + s[1].normalLoadN > 1.2f * (s[2].normalLoadN + s[3].normalLoadN));
    CHECK(s[0].normalLoadN + s[1].normalLoadN + s[2].normalLoadN + s[3].normalLoadN == doctest::Approx(80.f * 9.81f).epsilon(0.02));
    CHECK(rig.world->pose(rig.sim->body()).position.z == doctest::Approx(0.2f).epsilon(0.05));  // still level

    // A heavier robot accelerates more slowly at the same (torque-limited) voltage.
    Rig light(swerveSpec());
    REQUIRE(light.sim);
    light.run(0.5f);
    light.setDrive(1.0);
    light.run(0.1f);
    rig.run(0.0f);
    rig.setDrive(1.0);
    rig.run(0.1f);
    CHECK(rig.bodyVelocity().x < 0.85f * light.bodyVelocity().x);

    rig.sim->reset({});  // a session reset drops payloads
    CHECK(drive->chassisMass().massKg == 50.f);
}

TEST_CASE("coupling: a wrench through the simulator pushes the robot") {
    Rig rig(withMu(swerveSpec(), 0.001f));  // nearly frictionless tires so the wrench is not just held by traction
    REQUIRE(rig.sim);
    rig.run(0.5f);
    for (int i = 0; i < 100; ++i) {
        rig.sim->coupling().applyWrench({0.f, 0.f, 0.f}, Wrench{{0.f, 150.f, 0.f}, {}});
        rig.step();
    }
    CHECK(rig.bodyVelocity().y > 0.5f);
    CHECK(std::fabs(rig.bodyVelocity().x) < 0.05f);
}
