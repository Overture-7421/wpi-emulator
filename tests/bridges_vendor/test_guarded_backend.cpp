#include <doctest/doctest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "fakes.h"
#include "wpiemu/bridges/vendor/ctre_rules.h"
#include "wpiemu/bridges/vendor/guarded_backend.h"

using namespace wpiemu::vendor_tests;
using wpiemu::bridges::vendor::GuardedSimDeviceBackend;
using wpiemu::bridges::vendor::GuardOptions;

namespace {

struct GuardRig {
    FakeSimBackend inner;
    GuardedSimDeviceBackend guarded;
    wpiemu::sdk::testing::CapturingLogger log;
    SimDeviceHandle pigeon;
    SimDeviceHandle cancoder;
    SimValueHandle rawYaw, supply, rawPos;

    explicit GuardRig(GuardOptions options = wpiemu::bridges::vendor::ctre::phoenix6GuardOptions())
        : guarded(inner, std::move(options)) {
        pigeon = addPigeon2(inner, 13);
        cancoder = addCanCoder(inner, 2);
        guarded.setLogger(&log);
        std::vector<SimDeviceInfo> devices;
        guarded.listDevices(devices);
        for (const auto& d : devices) {
            std::vector<SimValueInfo> values;
            guarded.listValues(d.handle, values);
        }
        rawYaw = inner.handle(pigeon, "rawYawInput");
        supply = inner.handle(pigeon, "supplyVoltage");
        rawPos = inner.handle(cancoder, "rawPositionInput");
    }
};

}  // namespace

TEST_CASE("vendor guard: yaw is unwrapped into a continuous angle") {
    GuardRig r;
    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 170.0);
    r.guarded.endWrites();
    CHECK(r.inner.getValue(r.rawYaw) == doctest::Approx(170.0));

    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, -170.0);  // hub wrapped past +-180: physically +20 deg further
    r.guarded.endWrites();
    CHECK(r.inner.getValue(r.rawYaw) == doctest::Approx(190.0));

    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 170.0);  // and back
    r.guarded.endWrites();
    CHECK(r.inner.getValue(r.rawYaw) == doctest::Approx(170.0));
}

TEST_CASE("vendor guard: one coalesced write per tick, unchanged values are not rewritten") {
    GuardRig r;
    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 10.0);
    r.guarded.setValue(r.rawYaw, 11.0);
    r.guarded.setValue(r.rawYaw, 12.0);
    CHECK(r.inner.writesTo(r.pigeon, "rawYawInput") == 0);  // held back until the batch ends
    r.guarded.endWrites();
    CHECK(r.inner.writesTo(r.pigeon, "rawYawInput") == 1);
    CHECK(r.inner.get(r.pigeon, "rawYawInput") == doctest::Approx(12.0));

    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 12.0);
    r.guarded.endWrites();
    CHECK(r.inner.writesTo(r.pigeon, "rawYawInput") == 1);  // identical, intact: skipped

    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 12.5);
    r.guarded.endWrites();
    CHECK(r.inner.writesTo(r.pigeon, "rawYawInput") == 2);
}

TEST_CASE("vendor guard: equivalent wrapped yaw readback is not a foreign writer") {
    GuardRig r;
    r.guarded.setValue(r.rawYaw, -180.00006);
    for (int tick = 0; tick < 10; ++tick) {
        r.inner.poke(r.pigeon, "rawYawInput", 179.99994);
        CHECK(r.guarded.reassertAfterMirror() == 0);
        r.guarded.setValue(r.rawYaw, -180.00006);
    }
    CHECK(r.guarded.disturbances().empty());
    CHECK(r.log.count(wpiemu::sdk::LogLevel::Warn) == 0);
    CHECK(r.inner.writesTo(r.pigeon, "rawYawInput") == 1);

    r.guarded.setValue(r.rawPos, 1.0);
    r.inner.poke(r.cancoder, "rawPositionInput", 361.0);
    CHECK(r.guarded.reassertAfterMirror() == 1);  // rotor revolutions must not be wrapped
}

TEST_CASE("vendor guard: unguarded inputs are written through every time") {
    GuardRig r;
    r.guarded.beginWrites();
    r.guarded.setValue(r.supply, 12.0);
    CHECK(r.inner.writesTo(r.pigeon, "supplyVoltage") == 1);  // immediate, not batched
    r.guarded.endWrites();
    r.guarded.beginWrites();
    r.guarded.setValue(r.supply, 12.0);
    r.guarded.endWrites();
    CHECK(r.inner.writesTo(r.pigeon, "supplyVoltage") == 2);
}

TEST_CASE("vendor guard: writes outside a batch go straight through") {
    GuardRig r;
    r.guarded.setValue(r.rawPos, 3.0);
    CHECK(r.inner.get(r.cancoder, "rawPositionInput") == doctest::Approx(3.0));
    r.guarded.setValue(r.rawPos, 3.5);
    CHECK(r.inner.get(r.cancoder, "rawPositionInput") == doctest::Approx(3.5));
}

TEST_CASE("vendor guard: values with unknown handles pass through untouched") {
    FakeSimBackend inner;
    GuardedSimDeviceBackend guarded(inner, {});
    const SimDeviceHandle d = addPigeon2(inner, 1);  // never listed through the guard
    guarded.setValue(inner.handle(d, "rawYawInput"), 5.0);
    CHECK(inner.get(d, "rawYawInput") == doctest::Approx(5.0));
}

TEST_CASE("vendor guard: a disturbed input is detected by read-back and rewritten") {
    GuardRig r;
    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 45.0);
    r.guarded.endWrites();
    REQUIRE(r.inner.writesTo(r.pigeon, "rawYawInput") == 1);

    r.inner.poke(r.pigeon, "rawYawInput", 0.0);  // someone (mirror / robot-side sim) overwrote it

    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 45.0);  // same truth as before
    r.guarded.endWrites();
    CHECK(r.inner.writesTo(r.pigeon, "rawYawInput") == 2);  // skipped-if-unchanged does not hide a disturbance
    CHECK(r.inner.get(r.pigeon, "rawYawInput") == doctest::Approx(45.0));
    const auto d = r.guarded.disturbances();
    REQUIRE(d.size() == 1);
    CHECK(d[0].value == "rawYawInput");
    CHECK(d[0].device == "CANGyro:Pigeon 2 (v6)[13]");
    CHECK(d[0].mismatches == 1);
    CHECK_FALSE(d[0].foreignWriter);
}

TEST_CASE("vendor guard: reassertAfterMirror restores only disturbed guarded values") {
    GuardRig r;
    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 30.0);
    r.guarded.setValue(r.rawPos, 2.0);
    r.guarded.setValue(r.supply, 12.0);
    r.guarded.endWrites();

    CHECK(r.guarded.reassertAfterMirror() == 0);  // nothing disturbed

    r.inner.poke(r.pigeon, "rawYawInput", 0.0);
    r.inner.poke(r.pigeon, "supplyVoltage", 4.0);  // unguarded: not restored here
    CHECK(r.guarded.reassertAfterMirror() == 1);
    CHECK(r.inner.get(r.pigeon, "rawYawInput") == doctest::Approx(30.0));
    CHECK(r.inner.get(r.pigeon, "supplyVoltage") == doctest::Approx(4.0));
    CHECK(r.inner.get(r.cancoder, "rawPositionInput") == doctest::Approx(2.0));
}

TEST_CASE("vendor guard: a persistent foreign writer is reported once") {
    GuardRig r;
    for (int tick = 0; tick < 8; ++tick) {
        r.guarded.beginWrites();
        r.guarded.setValue(r.rawYaw, 10.0 + tick);
        r.guarded.endWrites();
        r.inner.poke(r.pigeon, "rawYawInput", -999.0);  // another simulation clobbers it every loop
        r.guarded.reassertAfterMirror();
        r.inner.poke(r.pigeon, "rawYawInput", -999.0);
    }
    const auto d = r.guarded.disturbances();
    REQUIRE(d.size() == 1);
    CHECK(d[0].foreignWriter);
    CHECK(d[0].reasserts >= 5);
    CHECK(r.log.count(wpiemu::sdk::LogLevel::Warn) == 1);  // exactly one warning
}

TEST_CASE("vendor guard: transient disturbance does not become a foreign writer") {
    GuardRig r;
    for (int tick = 0; tick < 20; ++tick) {
        r.guarded.beginWrites();
        r.guarded.setValue(r.rawYaw, 1.0 * tick);
        r.guarded.endWrites();
        if (tick == 3) r.inner.poke(r.pigeon, "rawYawInput", 0.0);
    }
    const auto d = r.guarded.disturbances();
    REQUIRE(d.size() == 1);
    CHECK(d[0].mismatches == 1);
    CHECK_FALSE(d[0].foreignWriter);
    CHECK(r.log.count(wpiemu::sdk::LogLevel::Warn) == 0);
}

TEST_CASE("vendor guard: resetHistory forgets the unwrapping reference") {
    GuardRig r;
    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 170.0);
    r.guarded.endWrites();
    r.guarded.resetHistory();
    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, -170.0);
    r.guarded.endWrites();
    CHECK(r.inner.get(r.pigeon, "rawYawInput") == doctest::Approx(-170.0));  // no unwrap against stale history
}

TEST_CASE("vendor guard: an invalid guard regex never matches and never throws") {
    GuardOptions g;
    g.guarded.push_back({"([unclosed", "rawYawInput", true});
    GuardRig r(g);
    r.guarded.beginWrites();
    r.guarded.setValue(r.rawYaw, 5.0);
    r.guarded.setValue(r.rawYaw, 6.0);
    CHECK(r.inner.writesTo(r.pigeon, "rawYawInput") == 2);  // unguarded => immediate write-through
    r.guarded.endWrites();
}

// Firmware model per the CTRE docs: changes of rawYawInput are INTEGRATED into the reported yaw; the robot's
// setYaw only moves an offset. Proves the strategy keeps the robot-visible yaw correct across wraps and resets.
TEST_CASE("vendor guard: robot setYaw is respected, wraps never leak into the integrated yaw") {
    GuardRig r;
    double lastRaw = 0.0;
    double visible = 0.0;  // firmware-reported yaw (what Pigeon2.getYaw() returns)
    r.inner.onWrite = [&](SimValueHandle h, double v) {
        if (h != r.rawYaw) return;
        visible += v - lastRaw;
        lastRaw = v;
    };
    const auto wrap = [](double deg) { return deg - 360.0 * std::round(deg / 360.0); };

    double truth = 0.0;  // continuous physical heading, degrees
    double zeroOffset = 0.0;
    for (int tick = 0; tick < 400; ++tick) {
        truth += 2.5;  // 1000 deg total: crosses the +-180 wrap several times
        if (tick == 150) {  // robot code calls pigeon.setYaw(0): the firmware moves its offset, raw is untouched
            zeroOffset = visible;
            visible -= zeroOffset;
        }
        r.guarded.beginWrites();
        r.guarded.setValue(r.rawYaw, wrap(truth));  // the hub only knows the wrapped heading
        r.guarded.endWrites();
        CHECK(visible == doctest::Approx(truth - zeroOffset).epsilon(1e-9));
    }
}
