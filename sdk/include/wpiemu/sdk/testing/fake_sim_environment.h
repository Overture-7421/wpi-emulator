#pragma once

// One-stop test environment: fake physics + clock + hub + logger + config wired into a SimContext.
//
//   FakeSimEnvironment env;
//   env.hub.autoBind({"intake", "motor"}, SlotType::Motor);
//   mechanism.tick(env.ctx, 0.005); env.advance();

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/fake_device_hub.h"
#include "wpiemu/sdk/testing/fake_mechanism_physics_coupling.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

namespace wpiemu::sdk::testing {

class FakeSimEnvironment {
public:
    FakeSimEnvironment()
        : ctx(SimContextServices{physics, clock, hub, logger, config, rng, driverStation, BodyId{}, nullptr, &coupling}) {}

    FakeSimEnvironment(const FakeSimEnvironment&) = delete;
    FakeSimEnvironment& operator=(const FakeSimEnvironment&) = delete;

    // Advance the clock and step the fake world by one fixed step: submitted coupling wrenches/payloads
    // are applied to the chassis body first, and constraint loads become readable after the step.
    void advance() {
        coupling.flushTo(physics, ctx.chassisBody());
        physics.step(static_cast<float>(clock.fixedStepSeconds()));
        coupling.endStep();
        clock.advance();
    }

    FakePhysicsWorld physics;
    FakeClock clock;
    FakeDeviceHub hub;
    CapturingLogger logger;
    RobotConfig config;
    Rng rng{1};
    proto::DsState driverStation;
    FakeMechanismPhysicsCoupling coupling;
    SimContext ctx;  // declared last: references the members above
};

}  // namespace wpiemu::sdk::testing
