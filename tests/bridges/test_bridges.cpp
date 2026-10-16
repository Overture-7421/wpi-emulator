#include <doctest/doctest.h>

#include "wpiemu/bridges/hal/hal_bridges.h"
#include "wpiemu/bridges/vendor/vendor_bridges.h"

TEST_CASE("bridge families are distinct") {
    CHECK(wpiemu::bridges::hal::bridgeFamily() == "hal");
    CHECK(wpiemu::bridges::vendor::bridgeFamily() == "vendor");
}
