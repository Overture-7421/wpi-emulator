#include <doctest/doctest.h>

#include "scoped_env.h"

using wpiemu::test_support::ScopedEnv;

namespace {
constexpr const char* kVariable = "WPIEMU_TEST_SCOPED_ENV";
}

TEST_CASE("ScopedEnv sets, replaces and removes a variable, then restores the previous value") {
    const ScopedEnv outer(kVariable, "outer");
    {
        ScopedEnv inner(kVariable, "inner");
        CHECK(ScopedEnv::read(kVariable) == "inner");
        inner.unset();
        CHECK_FALSE(ScopedEnv::read(kVariable).has_value());
        inner.set("again");
        CHECK(ScopedEnv::read(kVariable) == "again");
    }
    CHECK(ScopedEnv::read(kVariable) == "outer");
}

TEST_CASE("ScopedEnv leaves an absent variable absent") {
    const ScopedEnv absent(kVariable, std::nullopt);
    {
        const ScopedEnv inner(kVariable, "set");
        CHECK(ScopedEnv::read(kVariable) == "set");
    }
    CHECK_FALSE(ScopedEnv::read(kVariable).has_value());
}

#if !defined(_WIN32)
TEST_CASE("ScopedEnv keeps an empty variable apart from an absent one") {
    const ScopedEnv empty(kVariable, "");
    REQUIRE(ScopedEnv::read(kVariable).has_value());
    {
        const ScopedEnv inner(kVariable, "set");
        CHECK(ScopedEnv::read(kVariable) == "set");
    }
    CHECK(ScopedEnv::read(kVariable) == std::string());
}
#endif
