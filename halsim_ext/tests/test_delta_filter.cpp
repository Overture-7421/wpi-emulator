#include <doctest/doctest.h>

#include <limits>

#include "wpiemu/halsim_ext/signal_delta_filter.h"

using namespace wpiemu;
using namespace wpiemu::halsim;

namespace {
sdk::SignalSample s(std::uint32_t dev, std::uint16_t sig, double v) { return {sdk::DeviceId{dev}, sig, v}; }
}  // namespace

TEST_CASE("delta filter: first frame is full, later frames carry only changes") {
    SignalDeltaFilter f;
    std::vector<sdk::SignalSample> a{s(1, 0, 1.0), s(1, 1, 2.0), s(2, 0, 3.0)};
    CHECK(f.filter(a, false));
    CHECK(a.size() == 3);

    std::vector<sdk::SignalSample> b{s(1, 0, 1.0), s(1, 1, 2.5), s(2, 0, 3.0)};
    CHECK_FALSE(f.filter(b, false));
    REQUIRE(b.size() == 1);
    CHECK(b[0] == s(1, 1, 2.5));

    std::vector<sdk::SignalSample> c{s(1, 0, 1.0), s(1, 1, 2.5), s(2, 0, 3.0)};
    CHECK_FALSE(f.filter(c, false));
    CHECK(c.empty());
}

TEST_CASE("delta filter: forceFull and reset resend everything, new signals are always sent") {
    SignalDeltaFilter f;
    std::vector<sdk::SignalSample> a{s(1, 0, 1.0)};
    f.filter(a, false);

    std::vector<sdk::SignalSample> b{s(1, 0, 1.0), s(3, 0, 9.0)};
    CHECK_FALSE(f.filter(b, false));
    REQUIRE(b.size() == 1);
    CHECK(b[0].device.value == 3);

    std::vector<sdk::SignalSample> c{s(1, 0, 1.0), s(3, 0, 9.0)};
    CHECK(f.filter(c, true));
    CHECK(c.size() == 2);

    f.reset();
    std::vector<sdk::SignalSample> d{s(1, 0, 1.0)};
    CHECK(f.filter(d, false));
    CHECK(d.size() == 1);
}

TEST_CASE("delta filter: NaN and signed zero are compared bitwise") {
    SignalDeltaFilter f;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<sdk::SignalSample> a{s(1, 0, nan), s(1, 1, 0.0)};
    f.filter(a, false);
    std::vector<sdk::SignalSample> b{s(1, 0, nan), s(1, 1, -0.0)};
    f.filter(b, false);
    REQUIRE(b.size() == 1);  // only the -0.0 differs bitwise; the NaN is stable
    CHECK(b[0].signal == 1);
}

TEST_CASE("delta filter: forget re-sends a retired device that comes back") {
    SignalDeltaFilter f;
    std::vector<sdk::SignalSample> a{s(1, 0, 1.0), s(2, 0, 2.0)};
    f.filter(a, false);
    f.forget(sdk::DeviceId{1});
    std::vector<sdk::SignalSample> b{s(1, 0, 1.0), s(2, 0, 2.0)};
    f.filter(b, false);
    REQUIRE(b.size() == 1);
    CHECK(b[0].device.value == 1);
}
