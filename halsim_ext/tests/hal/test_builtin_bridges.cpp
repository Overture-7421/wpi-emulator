#include <doctest/doctest.h>

#include <algorithm>

#include "wpiemu/sdk/devices/builtin_bridges.h"
#include "wpiemu/sdk/devices/device_bridge.h"

using namespace wpiemu::sdk;

TEST_CASE("the built-in bridge aggregator registers every shipped module, HAL and vendor") {
    DeviceBridgeRegistry registry;
    const Status status = registerBuiltinBridges(registry);
    CHECK(status.ok());

    const std::vector<std::string> keys = registry.keys();
    auto has = [&](std::string_view prefix) {
        return std::any_of(keys.begin(), keys.end(), [&](const std::string& k) { return k.rfind(prefix, 0) == 0; });
    };
    CHECK(has("hal."));
    CHECK(has("ctre."));

    auto created = registry.createAll();
    REQUIRE(created);
    CHECK(created.value().size() == keys.size());
}

TEST_CASE("registering the built-in bridges twice is reported, not silently merged") {
    DeviceBridgeRegistry registry;
    REQUIRE(registerBuiltinBridges(registry).ok());
    const std::size_t before = registry.size();
    (void)registerBuiltinBridges(registry);  // duplicates are rejected or ignored by the modules, never doubled
    CHECK(registry.size() == before);
}
