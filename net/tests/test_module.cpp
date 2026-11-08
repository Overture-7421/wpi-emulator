#include <doctest/doctest.h>

#include <string>

#include "wpiemu/net/module.h"

TEST_CASE("net: the module links and identifies itself") {
    CHECK(std::string(wpiemu::net::moduleName()) == "wpiemu_net");
}
