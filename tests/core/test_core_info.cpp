#include <doctest/doctest.h>

#include "wpiemu/core/core_info.h"

TEST_CASE("core links against the physics backend") {
    CHECK(wpiemu::core::coreName() == "wpiemu_core");
    CHECK(wpiemu::core::physicsBackendName() == "box3d");
}
