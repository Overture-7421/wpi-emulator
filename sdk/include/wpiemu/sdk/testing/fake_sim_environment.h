#pragma once

#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/game/null_objects.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/fake_device_hub.h"
#include "wpiemu/sdk/testing/fake_mechanism_physics_coupling.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

namespace wpiemu::sdk::testing {

class FakeSimEnvironment {
public:
    FakeSimEnvironment()
        : physics(config.sim.physics),
          clock(config.sim.stepUs),
          rng(config.sim.seed),
          ctx(SimContextServices{physics, clock, hub, logger, config, rng, driverStation, BodyId{}, nullptr, &coupling}) {}

    void advance() {
        const BodyId chassis = ctx.chassisBody();
        if (chassis.valid()) {
            coupling.flushTo(physics, chassis);
        } else {
            (void)coupling.consumePendingWrenches();
        }
        physics.step(static_cast<float>(clock.fixedStepSeconds()));
        coupling.endStep();
        clock.advance();
    }

    RobotConfig config;
    FakePhysicsWorld physics;
    FakeClock clock;
    FakeDeviceHub hub;
    CapturingLogger logger;
    Rng rng;
    proto::DsState driverStation;
    FakeMechanismPhysicsCoupling coupling;
    SimContext ctx;
};

}  // namespace wpiemu::sdk::testing
