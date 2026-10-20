#include <doctest/doctest.h>

#include "wpiemu/sdk/telemetry/session_status.h"

using namespace wpiemu::sdk;

TEST_CASE("session status: a robot process is active until it ends") {
    for (const ProcessState p : {ProcessState::Starting, ProcessState::Running, ProcessState::Stopping}) CHECK(isActive(p));
    for (const ProcessState p : {ProcessState::Idle, ProcessState::Exited, ProcessState::Crashed, ProcessState::FailedToStart}) CHECK_FALSE(isActive(p));

    SessionStatus status;
    CHECK_FALSE(status.active());
    status.process = ProcessState::Stopping;
    CHECK(status.active());
}

TEST_CASE("session status: armed is a linked robot whose simulation is idle") {
    CHECK(isArmed(ProcessState::Running, LinkState::Connected, false));
    CHECK(isArmed(ProcessState::Running, LinkState::Connecting, false));
    CHECK_FALSE(isArmed(ProcessState::Running, LinkState::Connected, true));
    CHECK_FALSE(isArmed(ProcessState::Running, LinkState::Disconnected, false));
    CHECK_FALSE(isArmed(ProcessState::Starting, LinkState::Connected, false));

    SessionStatus status;
    status.process = ProcessState::Running;
    status.link = LinkState::Connected;
    CHECK(status.armed());
    status.simulating = true;
    CHECK_FALSE(status.armed());
}
