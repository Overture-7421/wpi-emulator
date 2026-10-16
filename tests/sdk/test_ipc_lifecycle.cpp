#include <doctest/doctest.h>

#include "wpiemu/sdk/ipc/lifecycle.h"

using namespace wpiemu::sdk::proto;

TEST_CASE("lifecycle log round-trips through the Log message") {
    const Log log = makeLifecycleLog(kLifecycleProgramStarted, 1234);
    CHECK(isLifecycleLog(log, kLifecycleProgramStarted));
    CHECK(log.simTimeUs == 1234);

    Log other = log;
    other.source = "extension";
    CHECK_FALSE(isLifecycleLog(other, kLifecycleProgramStarted));
    CHECK_FALSE(isLifecycleLog(log, "something_else"));
}
