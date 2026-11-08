#include <doctest/doctest.h>

#include <chrono>

#include "wpiemu/sdk/core/clock.h"

using namespace wpiemu::sdk;

namespace {

class FixedClock final : public IWallClock {
public:
    std::chrono::steady_clock::time_point now() const override { return std::chrono::steady_clock::time_point{} + std::chrono::seconds(5); }
};

}  // namespace

TEST_CASE("wall clock: the steady clock never goes backwards") {
    const SteadyWallClock clock;
    const auto a = clock.now();
    const auto b = clock.now();
    CHECK(b >= a);
}

TEST_CASE("wall clock: an injected clock is read through the interface") {
    const FixedClock fixed;
    const IWallClock& clock = fixed;
    CHECK(clock.now() == fixed.now());
    CHECK(clock.now().time_since_epoch() == std::chrono::seconds(5));
}
