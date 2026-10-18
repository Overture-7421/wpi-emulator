#include <doctest/doctest.h>

#include <cmath>

#include "vision_frame_support.h"

namespace {

using namespace wpiemu;
using namespace wpiemu::tests::vision;

TEST_CASE("vision frame support: the sample scene projects tags inside the image, corners BL BR TR TL") {
    FrameSpec spec;
    spec.fieldToRobot = sampleRobotPose();
    spec.robotToCamera = sampleFrontMount();
    spec.tagIds = {1, 2, 3, 4};
    const sdk::VisionFrame frame = makeFrame(spec, sampleLayout());

    REQUIRE(frame.observations.size() == 4);
    CHECK(frame.observations.front().id == 4);  // the closest tag has the largest area, observations are sorted by it
    for (const sdk::FiducialObservation& o : frame.observations) {
        for (const sdk::PixelPoint& p : o.cornersPx) {
            CHECK(p.x > 0.0);
            CHECK(p.x < 1280.0);
            CHECK(p.y > 0.0);
            CHECK(p.y < 800.0);
        }
        const auto& c = o.cornersPx;
        CHECK(c[0].x < c[1].x);  // BL left of BR
        CHECK(c[3].x < c[2].x);  // TL left of TR
        CHECK(c[0].y > c[3].y);  // BL below TL (image y grows downwards)
        CHECK(c[1].y > c[2].y);  // BR below TR
        CHECK(o.areaPct > 0.0);
        CHECK(o.distanceM > 4.0);
    }
    // Tag 1 sits to the robot's right (y = 3.6 < 4.0): positive yaw, and below the pitched-up optical axis: negative pitch.
    const auto tag1 = std::find_if(frame.observations.begin(), frame.observations.end(),
                                   [](const sdk::FiducialObservation& o) { return o.id == 1; });
    REQUIRE(tag1 != frame.observations.end());
    CHECK(tag1->yawRad > 0.0);
    CHECK(tag1->pitchRad < 0.0);
}

TEST_CASE("vision frame support: the multi-tag solution is the camera pose in the field") {
    FrameSpec spec;
    spec.fieldToRobot = sampleRobotPose();
    spec.robotToCamera = sampleFrontMount();
    spec.tagIds = {1, 2};
    const sdk::VisionFrame frame = makeFrame(spec, sampleLayout());
    REQUIRE(frame.multiTag.valid);
    CHECK(frame.multiTag.tagIds.size() == 2);
    CHECK(frame.multiTag.fieldToCamera.position.x == doctest::Approx(2.30f).epsilon(1e-4));
    CHECK(frame.multiTag.fieldToCamera.position.y == doctest::Approx(4.0f).epsilon(1e-4));
    CHECK(frame.multiTag.fieldToCamera.position.z == doctest::Approx(0.5f).epsilon(1e-4));

    spec.tagIds = {1};
    CHECK_FALSE(makeFrame(spec, sampleLayout()).multiTag.valid);
}

}  // namespace
