#include <doctest/doctest.h>

#include "../ipc/match_samples.h"
#include "runner_rig.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;
using namespace std::chrono_literals;
using wpiemu::test_runtime::Rig;

// The practice-match and relay messages (MessageType 13..28) are not part of the robot link. The runner must drop
// them (counted, logged) without touching the session, the epoch or lock-step.

TEST_CASE("runner: match and relay messages on the robot link are dropped without changing state") {
    Rig rig;
    rig.handshake();
    REQUIRE(rig.run([&] { return rig.runner->ticksSent() >= 3; }));

    const RunnerState state = rig.runner->state();
    const std::uint64_t epoch = rig.runner->runEpoch();
    const std::uint64_t dropped = rig.runner->droppedFrames();
    const std::size_t devices = rig.session->hub().discovered().size();
    const std::size_t messages = wpiemu::test_ipc::matchSamples().size();

    for (const wpiemu::test_ipc::NamedMessage& m : wpiemu::test_ipc::matchSamples()) {
        CAPTURE(m.name);
        REQUIRE(rig.ext->send(m.message));
    }
    const std::uint64_t ticksAtSend = rig.runner->ticksSent();
    REQUIRE(rig.run([&] { return rig.runner->droppedFrames() >= dropped + messages && rig.runner->ticksSent() >= ticksAtSend + 3; }));

    CHECK(rig.runner->droppedFrames() == dropped + messages);
    CHECK(rig.runner->state() == state);
    CHECK(rig.runner->runEpoch() == epoch);
    CHECK(rig.session->hub().discovered().size() == devices);
    CHECK(rig.runner->robotHello().has_value());
}

TEST_CASE("runner: a match message before the Hello does not start or break the handshake") {
    Rig rig;
    rig.connect();
    for (const wpiemu::test_ipc::NamedMessage& m : wpiemu::test_ipc::matchSamples()) REQUIRE(rig.ext->send(m.message));
    rig.ext->hello(0);
    rig.ext->announceAll();
    REQUIRE(rig.run([&] {
        return rig.ext->helloAck.has_value() && rig.runner->ticksSent() >= 1 && rig.session->hub().discovered().size() == 7;
    }));
    CHECK(rig.ext->helloAck->accepted);
}
