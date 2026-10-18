#include <doctest/doctest.h>

#include "wpiemu/bridges/vision/limelight_orientation.h"

namespace {

namespace ll = wpiemu::bridges::vision::limelight;
using ll::OrientationHistory;
using ll::RobotOrientation;

RobotOrientation orientation(double yawDeg, double pitchDeg = 0.0, double rollDeg = 0.0) {
    RobotOrientation o;
    o.yawDeg = yawDeg;
    o.pitchDeg = pitchDeg;
    o.rollDeg = rollDeg;
    return o;
}

TEST_CASE("limelight orientation: before the first write the device has its power-on default, all zeros") {
    OrientationHistory history;
    CHECK(history.empty());
    CHECK(history.latest() == nullptr);
    const RobotOrientation none = history.at(1'000'000);
    CHECK(none.yawDeg == 0.0);
    CHECK(none.pitchDeg == 0.0);
    CHECK(none.rollDeg == 0.0);

    history.add(500'000, orientation(40.0));
    CHECK(history.at(400'000).yawDeg == 0.0);  // the capture is older than the first write
    CHECK(history.at(500'000).yawDeg == 40.0);
    REQUIRE(history.latest() != nullptr);
    CHECK(history.latest()->yawDeg == 40.0);
}

TEST_CASE("limelight orientation: the latest sample at or before the capture time holds") {
    OrientationHistory history;
    history.add(100'000, orientation(10.0));
    history.add(1'000'000, orientation(70.0));  // 900 ms later: nothing was written in between, nothing changed
    CHECK(history.at(100'000).yawDeg == 10.0);
    CHECK(history.at(600'000).yawDeg == 10.0);   // not smeared towards 70
    CHECK(history.at(999'999).yawDeg == 10.0);
    CHECK(history.at(1'000'000).yawDeg == 70.0);
    CHECK(history.at(5'000'000).yawDeg == 70.0);  // after the newest sample it holds
}

TEST_CASE("limelight orientation: samples written every loop are interpolated to the capture time") {
    OrientationHistory history;
    history.add(960'000, orientation(30.0, 2.0, -4.0));
    history.add(980'000, orientation(50.0, 6.0, 4.0));
    const RobotOrientation mid = history.at(975'000);  // 3/4 of the way
    CHECK(mid.yawDeg == doctest::Approx(45.0));
    CHECK(mid.pitchDeg == doctest::Approx(5.0));
    CHECK(mid.rollDeg == doctest::Approx(2.0));
    CHECK(history.at(960'000).yawDeg == doctest::Approx(30.0));
    CHECK(history.at(970'000).yawDeg == doctest::Approx(40.0));
}

TEST_CASE("limelight orientation: interpolation goes the short way round the circle") {
    OrientationHistory history;
    history.add(960'000, orientation(170.0));
    history.add(980'000, orientation(-170.0));
    CHECK(history.at(970'000).yawDeg == doctest::Approx(180.0));
    CHECK(history.at(975'000).yawDeg == doctest::Approx(-175.0));
    CHECK(history.at(965'000).yawDeg == doctest::Approx(175.0));
}

TEST_CASE("limelight orientation: samples exactly at the interpolation limit interpolate, one more microsecond holds") {
    OrientationHistory atLimit;
    atLimit.add(1'000'000, orientation(0.0));
    atLimit.add(1'000'000 + OrientationHistory::kMaxInterpolationGapUs, orientation(100.0));
    CHECK(atLimit.at(1'000'000 + OrientationHistory::kMaxInterpolationGapUs / 2).yawDeg == doctest::Approx(50.0));

    OrientationHistory beyond;
    beyond.add(1'000'000, orientation(0.0));
    beyond.add(1'000'000 + OrientationHistory::kMaxInterpolationGapUs + 1, orientation(100.0));
    CHECK(beyond.at(1'000'000 + OrientationHistory::kMaxInterpolationGapUs / 2).yawDeg == 0.0);
}

TEST_CASE("limelight orientation: rates are held, only the angles are interpolated") {
    OrientationHistory history;
    RobotOrientation a = orientation(0.0);
    a.yawRateDegPerS = 90.0;
    RobotOrientation b = orientation(10.0);
    b.yawRateDegPerS = 100.0;
    history.add(1'000'000, a);
    history.add(1'020'000, b);
    const RobotOrientation mid = history.at(1'010'000);
    CHECK(mid.yawDeg == doctest::Approx(5.0));
    CHECK(mid.yawRateDegPerS == 90.0);
}

TEST_CASE("limelight orientation: a stale write is ignored and a second write at the same time replaces the first") {
    OrientationHistory history;
    history.add(2'000'000, orientation(20.0));
    history.add(1'000'000, orientation(99.0));  // older than the newest: never delivered by ntcore, ignored
    CHECK(history.size() == 1);
    CHECK(history.at(2'000'000).yawDeg == 20.0);
    history.add(2'000'000, orientation(25.0));
    CHECK(history.size() == 1);
    CHECK(history.at(2'000'000).yawDeg == 25.0);
}

TEST_CASE("limelight orientation: the history is bounded in time and size but keeps the sample that still holds") {
    OrientationHistory history;
    // one write per 20 ms for 10 s, yaw = the time in seconds * 10
    for (int i = 0; i <= 500; ++i) history.add(static_cast<std::int64_t>(i) * 20'000, orientation(i * 0.2));
    CHECK(history.size() < 150);  // about kSpanUs / 20 ms, nowhere near 500
    CHECK(history.size() <= OrientationHistory::kMaxSamples);

    const std::int64_t newest = 500 * 20'000;
    CHECK(history.at(newest - 25'000).yawDeg == doctest::Approx((500 - 1.25) * 0.2));  // inside the window

    OrientationHistory quiet;  // a robot that wrote once and then nothing for much longer than the span
    quiet.add(1'000'000, orientation(12.0));
    quiet.add(50'000'000, orientation(13.0));
    CHECK(quiet.at(49'000'000).yawDeg == 12.0);  // the first sample still holds
}

TEST_CASE("limelight orientation: a burst of writes cannot grow the history past its cap") {
    OrientationHistory history;
    for (int i = 0; i < 5000; ++i) history.add(1'000'000 + i, orientation(i * 0.01));  // 1 us apart, all in the span
    CHECK(history.size() == OrientationHistory::kMaxSamples);
    CHECK(history.latest()->yawDeg == doctest::Approx(4999 * 0.01));
}

}  // namespace
