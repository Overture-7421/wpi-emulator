#include <doctest/doctest.h>

#include <string>

#include "wpiemu/net/module.h"
#include "wpiemu/relay/module.h"

TEST_CASE("relay: the module links the net module") {
    CHECK(std::string(wpiemu::relay::moduleName()) == "wpiemu_relay_core");
    CHECK(std::string(wpiemu::net::moduleName()) == "wpiemu_net");
}
