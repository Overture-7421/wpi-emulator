#include <doctest/doctest.h>

#include <cmath>
#include <vector>

#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/mechanisms/physics_coupling.h"
#include "wpiemu/sdk/testing/fake_mechanism_physics_coupling.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

using namespace wpiemu::sdk;

namespace {

SimContextServices servicesWithoutCoupling(testing::FakeSimEnvironment& env) {
    return SimContextServices{env.physics, env.clock, env.hub, env.logger, env.config, env.rng, env.driverStation, BodyId{}};
}

}  // namespace

TEST_CASE("SimContext hands out a null coupling when the runtime binds none") {
    testing::FakeSimEnvironment env;
    SimContext bare(servicesWithoutCoupling(env));

    IMechanismPhysicsCoupling& c = bare.coupling();
    CHECK_FALSE(c.capabilities().wrench);
    CHECK_FALSE(c.capabilities().constraints);
    CHECK_FALSE(c.capabilities().payload);

    c.applyWrench({0.f, 0.f, 0.f}, Wrench{{1.f, 0.f, 0.f}, {}});  // no-op, must not crash
    CHECK_FALSE(c.addConstraint(AnchorConstraintDesc{}).valid());  // invalid handle: unsupported
    CHECK_FALSE(c.constraintLoad(ConstraintHandle{1}).valid);
    c.setPayload("elevator", PayloadMass{5.f, {}, {}});
    c.removeConstraint(ConstraintHandle{});

    // the environment binds the fake service instead
    CHECK(&env.ctx.coupling() == &env.coupling);
    CHECK(env.ctx.coupling().capabilities().constraints);

    SimContext rebound(servicesWithoutCoupling(env));
    rebound.bindCoupling(&env.coupling);
    CHECK(&rebound.coupling() == &env.coupling);
    rebound.bindCoupling(nullptr);
    CHECK(&rebound.coupling() != &env.coupling);
}

TEST_CASE("wrenches are submitted before the step and applied to the chassis") {
    testing::FakeSimEnvironment env;
    BodyDesc desc;
    desc.type = BodyType::Dynamic;
    const BodyId chassis = env.physics.createBody(desc);
    env.ctx.bindChassisBody(chassis);

    // a mechanism pushes forward at a point 0.5 m left of the COM
    env.ctx.coupling().applyWrench({0.f, 0.5f, 0.f}, Wrench{{10.f, 0.f, 0.f}, {0.f, 0.f, 1.f}});
    env.ctx.coupling().applyWrench({0.f, 0.5f, 0.f}, Wrench{{0.f, 0.f, 0.f}, {0.f, 0.f, 2.f}});  // additive
    const Wrench net = env.coupling.netPendingWrench();
    CHECK(net.force.x == doctest::Approx(10.f));
    CHECK(net.torque.z == doctest::Approx(1.f + 2.f - 0.5f * 10.f));  // couples + r x F

    env.advance();
    CHECK(env.coupling.pendingWrenches().empty());  // consumed by the step: re-submit every tick
    REQUIRE_FALSE(env.physics.lastStepForces().empty());
    CHECK(env.physics.lastStepForces().front().force.x == doctest::Approx(10.f));
}

TEST_CASE("constraint loads always report the last COMPLETED physics step") {
    testing::FakeSimEnvironment env;
    AnchorConstraintDesc rope;
    rope.kind = AnchorKind::Distance;
    rope.owner = "climber";
    rope.chassisLocalPoint = {0.f, 0.f, 0.3f};
    rope.fieldPoint = {5.f, 0.f, 2.f};
    const ConstraintHandle h = env.ctx.coupling().addConstraint(rope);
    REQUIRE(h.valid());

    CHECK_FALSE(env.ctx.coupling().constraintLoad(h).valid);  // no step has completed yet

    ConstraintLoad produced;
    produced.engaged = true;
    produced.force = {0.f, 0.f, 120.f};
    produced.forceMagnitudeN = 120.f;
    env.coupling.scriptLoad(h, produced);                      // the world "computes" it during the step
    CHECK_FALSE(env.ctx.coupling().constraintLoad(h).valid);  // still invisible while the tick is running
    env.advance();

    const ConstraintLoad load = env.ctx.coupling().constraintLoad(h);  // next tick's mechanism reads the prior step
    CHECK(load.valid);
    CHECK(load.engaged);
    CHECK(load.forceMagnitudeN == doctest::Approx(120.f));

    env.ctx.coupling().removeConstraint(h);
    env.ctx.coupling().removeConstraint(h);  // idempotent
    CHECK_FALSE(env.ctx.coupling().constraintLoad(h).valid);
    CHECK(env.coupling.constraintCount() == 0);
}

TEST_CASE("unsupported or unusable constraints yield an invalid handle") {
    testing::FakeMechanismPhysicsCoupling fake;
    AnchorConstraintDesc slide;
    slide.kind = AnchorKind::Prismatic;
    slide.axisWorld = {0.f, 0.f, 0.f};
    CHECK_FALSE(fake.addConstraint(slide).valid());  // zero axis
    slide.axisWorld = {0.f, 0.f, 1.f};
    CHECK(fake.addConstraint(slide).valid());

    AnchorConstraintDesc bad;
    bad.fieldPoint = {std::nanf(""), 0.f, 0.f};
    CHECK_FALSE(fake.addConstraint(bad).valid());

    fake.setCapabilities({true, false, false});
    CHECK_FALSE(fake.addConstraint(AnchorConstraintDesc{}).valid());
    fake.setPayload("lift", PayloadMass{4.f, {}, {}});
    CHECK(fake.payloads().empty());
}

TEST_CASE("payloads compose with the chassis (mass, COM, parallel-axis inertia)") {
    MassProperties chassis;
    chassis.massKg = 50.f;
    chassis.comLocal = {0.f, 0.f, 0.f};
    chassis.inertiaDiagLocal = {1.f, 2.f, 3.f};

    const std::vector<PayloadMass> payloads{PayloadMass{10.f, {0.6f, 0.f, 0.f}, {0.f, 0.f, 0.f}}};
    const MassProperties total = composeMassProperties(chassis, payloads);
    CHECK(total.massKg == doctest::Approx(60.f));
    CHECK(total.comLocal.x == doctest::Approx(0.1f));
    // about the new COM: chassis offset -0.1, payload offset +0.5 along x
    CHECK(total.inertiaDiagLocal.x == doctest::Approx(1.f));
    CHECK(total.inertiaDiagLocal.y == doctest::Approx(2.f + 50.f * 0.01f + 10.f * 0.25f));
    CHECK(total.inertiaDiagLocal.z == doctest::Approx(3.f + 50.f * 0.01f + 10.f * 0.25f));

    CHECK(composeMassProperties(chassis, {}).massKg == doctest::Approx(50.f));
    const PayloadMass none{0.f, {1.f, 1.f, 1.f}, {}};
    CHECK(composeMassProperties(chassis, std::vector<PayloadMass>{none}).comLocal == Vec3{});
}

TEST_CASE("the fake replaces and clears payloads per owner and forwards the composed mass") {
    testing::FakeSimEnvironment env;
    BodyDesc desc;
    desc.type = BodyType::Dynamic;
    const BodyId chassis = env.physics.createBody(desc);
    env.ctx.bindChassisBody(chassis);

    MassProperties base;
    base.massKg = 40.f;
    env.coupling.setChassisMass(base);

    env.ctx.coupling().setPayload("elevator", PayloadMass{4.f, {0.f, 0.f, 0.5f}, {}});
    env.ctx.coupling().setPayload("elevator", PayloadMass{6.f, {0.f, 0.f, 0.5f}, {}});  // replaces
    env.ctx.coupling().setPayload("intake", PayloadMass{2.f, {0.3f, 0.f, 0.1f}, {}});
    CHECK(env.coupling.payloads().size() == 2);
    CHECK(env.coupling.composedMass(base).massKg == doctest::Approx(48.f));

    env.ctx.coupling().setPayload("intake", PayloadMass{});  // massKg <= 0 clears
    CHECK(env.coupling.payloads().size() == 1);
    env.ctx.coupling().clearPayload("elevator");
    CHECK(env.coupling.payloads().empty());
    env.advance();  // no payloads: nothing to forward, must not fail
}
