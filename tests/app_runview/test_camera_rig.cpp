#include <doctest/doctest.h>

#include <cmath>
#include <memory>

#include "camera_rig.h"

using namespace wpiemu;
using namespace wpiemu::app;
using namespace wpiemu::app::runview;

namespace {

bool close(const sdk::Vec3& a, const sdk::Vec3& b, float epsilon = 1e-3f) {
    return sdk::length(a - b) <= epsilon;
}

}  // namespace

TEST_CASE("camera rig station views use explicit and generic field positions") {
    CameraRig rig;
    rig.setField(16.f, 8.f, {8.f, 4.f, 0.f});
    sdk::FieldDescription generic{};
    generic.lengthM = 16.f;
    generic.widthM = 8.f;
    for (const sdk::Alliance alliance : {sdk::Alliance::Blue, sdk::Alliance::Red})
        for (int number = 1; number <= 3; ++number) {
            rig.showStation(alliance, number);
            const auto expected = sdk::driverStationSpot(generic, alliance, number);
            REQUIRE(expected);
            CHECK(close(rig.camera().position(), expected->eyeM));
        }
    rig.showStation(sdk::Alliance::Red, 1);
    CHECK(rig.camera().basis().forward.x < -0.9f);

    auto description = std::make_shared<sdk::FieldDescription>();
    description->lengthM = 16.f;
    description->widthM = 8.f;
    description->driverStations.push_back({sdk::Alliance::Blue, 2, {-1.f, 3.f, 1.7f}});
    rig.setField(16.f, 8.f, {8.f, 4.f, 0.f}, description);
    rig.showStation(sdk::Alliance::Blue, 2);
    CHECK(close(rig.camera().position(), {-1.f, 3.f, 1.7f}));
    rig.setDriverStation(sdk::Alliance::Red, 1);
    CHECK(close(rig.camera().position(), {-1.f, 3.f, 1.7f}));
}

TEST_CASE("camera rig follows with smoothed gaze, and follow-off leaves gaze manual") {
    CameraRig rig;
    rig.setField(16.f, 8.f, {8.f, 4.f, 0.f});
    rig.setDriverStation(sdk::Alliance::Blue, 1);
    const sdk::Vec3 robot{12.f, 7.f, 0.f};
    rig.showDriverPov();
    const sdk::Vec3 before = rig.camera().basis().forward;
    rig.setRobot(robot);
    rig.advance(0.02);
    const sdk::Vec3 partial = rig.camera().basis().forward;
    CHECK(sdk::length(partial - before) > 1e-4f);
    CHECK(sdk::length(partial - sdk::normalized(robot - rig.camera().position())) <
          sdk::length(before - sdk::normalized(robot - rig.camera().position())));
    for (int i = 0; i < 100; ++i) rig.advance(0.1);
    const auto centered = rig.camera().project(robot, 800.f, 600.f);
    REQUIRE(centered);
    CHECK(centered->first == doctest::Approx(400.f).epsilon(1e-3));
    CHECK(centered->second == doctest::Approx(300.f).epsilon(1e-3));
    const sdk::Vec3 stable = rig.camera().basis().forward;
    rig.advance(0.0);
    CHECK(close(rig.camera().basis().forward, stable));

    rig.setFollow(false);
    rig.setRobot(sdk::Vec3{1.f, 7.f, 0.f});
    rig.advance(0.1);
    CHECK(close(rig.camera().basis().forward, stable));
    rig.orbit(20.f, 10.f);
    CHECK_FALSE(close(rig.camera().basis().forward, stable));
    rig.orbit(0.f, 100000.f);
    CHECK(std::abs(rig.camera().pitchRad) <= sdk::degToRadF(89.f) + 1e-4f);
    const sdk::Vec3 looked = rig.camera().basis().forward;
    rig.setRobot(sdk::Vec3{15.f, 1.f, 0.f});
    rig.advance(0.1);
    CHECK(close(rig.camera().basis().forward, looked));
}

TEST_CASE("camera rig driver zoom, station retarget and field changes preserve view") {
    CameraRig rig;
    rig.setField(16.f, 8.f, {8.f, 4.f, 0.f});
    rig.setDriverStation(sdk::Alliance::Blue, 1);
    rig.showDriverPov();
    const sdk::Vec3 initialEye = rig.camera().position();
    rig.zoom(1.f);
    CHECK(rig.camera().fovDeg == doctest::Approx(39.6f));
    for (int i = 0; i < 100; ++i) rig.zoom(10.f);
    CHECK(rig.camera().fovDeg == doctest::Approx(15.f));
    for (int i = 0; i < 100; ++i) rig.zoom(-10.f);
    CHECK(rig.camera().fovDeg == doctest::Approx(75.f));
    rig.setDriverStation(sdk::Alliance::Red, 3);
    CHECK(rig.view().kind == CameraView::Kind::DriverPov);
    CHECK(rig.camera().position().x > 16.f);
    rig.showHome();
    const OrbitCamera oldHome = rig.camera();
    rig.showHome();
    CHECK(close(rig.camera().position(), oldHome.position()));
    CHECK(close(initialEye, {-0.8f, 6.6666665f, 1.6f}));
    rig.showStation(sdk::Alliance::Blue, 2);
    auto field = std::make_shared<sdk::FieldDescription>();
    field->lengthM = 20.f;
    field->widthM = 10.f;
    field->driverStations.push_back({sdk::Alliance::Blue, 2, {-2.f, 5.f, 1.8f}});
    rig.setField(20.f, 10.f, {10.f, 5.f, 0.f}, field);
    CHECK(rig.view().kind == CameraView::Kind::Station);
    CHECK(close(rig.camera().position(), {-2.f, 5.f, 1.8f}));
}

TEST_CASE("camera rig view keys round trip and reject unknown values") {
    for (const std::string key : {"home", "driver", "blue1", "blue2", "blue3", "red1", "red2", "red3"}) {
        const auto view = parseCameraViewKey(key);
        REQUIRE(view);
        CHECK(cameraViewKey(*view) == key);
    }
    CHECK_FALSE(parseCameraViewKey("blue0"));
    CHECK_FALSE(parseCameraViewKey("red4"));
    CHECK_FALSE(parseCameraViewKey("unknown"));
}

TEST_CASE("camera rig follow wraps yaw across the red-wall boundary by the short arc") {
    CameraRig rig;
    rig.setField(16.f, 8.f, {8.f, 4.f, 0.f});
    rig.setDriverStation(sdk::Alliance::Red, 1);
    rig.showDriverPov();
    rig.setRobot(sdk::Vec3{8.f, 0.f, 0.f});
    rig.advance(0.1);
    CHECK(rig.camera().basis().forward.x < -0.95f);
}
