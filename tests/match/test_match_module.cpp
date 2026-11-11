#include <doctest/doctest.h>

#include <string>

#include "wpiemu/core/match/module.h"

TEST_CASE("match: the module links and identifies itself") {
    CHECK(std::string(wpiemu::core::match::moduleName()) == "wpiemu_match");
}
