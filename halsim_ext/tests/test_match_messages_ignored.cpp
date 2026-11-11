#include <doctest/doctest.h>

#include "../../tests/ipc/match_samples.h"
#include "fakes.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

// The practice-match and relay messages (MessageType 13..28) are not part of the robot-process protocol. A robot
// process that receives one (a misrouted frame, a newer emulator) must ignore it and keep running lock-step.

TEST_CASE("extension: match and relay messages from the emulator are ignored and lock-step continues") {
    Rig rig;
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const auto first = rig.emulator->tick();
    REQUIRE(first);
    CHECK(first->result == proto::TickResult::Completed);
    const std::uint64_t ticksBefore = rig.runtime->stats().ticksCompleted;
    const std::uint64_t errorsBefore = rig.runtime->stats().protocolErrors;
    const std::uint64_t lostBefore = rig.runtime->stats().emulatorLost;

    for (const test_ipc::NamedMessage& m : test_ipc::matchSamples()) {
        CAPTURE(m.name);
        REQUIRE(rig.emulator->send(m.message));
    }

    const auto next = rig.emulator->tick();
    REQUIRE(next);
    CHECK(next->result == proto::TickResult::Completed);
    CHECK(next->ackedTick() == first->ackedTick() + 1);
    CHECK(rig.runtime->stats().ticksCompleted == ticksBefore + 1);
    CHECK(rig.runtime->stats().protocolErrors == errorsBefore);
    CHECK(rig.runtime->stats().emulatorLost == lostBefore);
    CHECK(rig.runtime->stats().stalls == 0);
    CHECK_FALSE(rig.emulator->connectionClosed());
}
