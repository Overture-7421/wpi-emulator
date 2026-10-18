#include <doctest/doctest.h>

#include <QApplication>
#include <QSettings>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtGlobal>

#include <cmath>

#include "fake_joystick.h"
#include "fake_run_backend.h"
#include "run_page.h"
#include "scene_model.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

constexpr float kViewW = 1000.f;
constexpr float kViewH = 700.f;

// What one robotPoseRequested emission carries.
struct Requested {
    double x, y, z, yaw;
};
Requested requested(const QSignalSpy& spy, int index) {
    const QList<QVariant>& args = spy.at(index);
    return {args.at(0).toDouble(), args.at(1).toDouble(), args.at(2).toDouble(), args.at(3).toDouble()};
}

struct SceneFixture {
    FakeRunBackend backend{[] {
        FakeRunBackend::Options o;
        o.autoRun = false;
        o.startupDelayMs = 0;
        return o;
    }()};
    SceneModel scene;
    std::shared_ptr<const sdk::ISimSnapshot> snapshot;

    SceneFixture() {
        scene.setSceneInfo(backend.sceneInfo());
        scene.setViewSize(kViewW, kViewH);
        backend.startSimulation();
        backend.tick(3);
        refresh();
    }
    void refresh() {
        snapshot = backend.latestSnapshot();
        REQUIRE(snapshot != nullptr);
        scene.update(snapshot.get());
    }
    sdk::Pose3 robot() const { return snapshot->chassis().pose; }
    QPointF pixelOf(const sdk::Vec3& world) {
        const auto px = scene.camera().project(world, kViewW, kViewH);
        REQUIRE(px);
        return {static_cast<qreal>(px->first), static_cast<qreal>(px->second)};
    }
    QPointF onRobot() { return pixelOf(robot().position); }
    bool begin(const QPointF& p) { return scene.beginRobotDrag(static_cast<float>(p.x()), static_cast<float>(p.y())); }
    void drag(const QPointF& p, int modifiers = 0) {
        scene.dragRobot(static_cast<float>(p.x()), static_cast<float>(p.y()), modifiers);
    }
};

}  // namespace

TEST_CASE("run view placement: the robot can only be picked up while placement is enabled and the pointer is on it") {
    SceneFixture f;
    const QPointF onRobot = f.onRobot();
    CHECK_FALSE(f.scene.placementEnabled());
    CHECK_FALSE(f.scene.robotAt(static_cast<float>(onRobot.x()), static_cast<float>(onRobot.y())));
    CHECK_FALSE(f.begin(onRobot));   // not while the simulation is not live

    f.scene.setPlacementEnabled(true);
    CHECK(f.scene.robotAt(static_cast<float>(onRobot.x()), static_cast<float>(onRobot.y())));
    const QPointF beside = f.pixelOf(f.robot().position + sdk::Vec3{3.f, 0.f, 0.f});
    CHECK_FALSE(f.scene.robotAt(static_cast<float>(beside.x()), static_cast<float>(beside.y())));
    CHECK_FALSE(f.begin(beside));
    CHECK_FALSE(f.scene.robotDragging());

    QSignalSpy drags(&f.scene, &SceneModel::robotDragChanged);
    REQUIRE(f.begin(onRobot));
    CHECK(f.scene.robotDragging());
    CHECK(drags.count() == 1);
    CHECK_FALSE(f.begin(onRobot));       // already holding it
    f.scene.setPlacementEnabled(false);  // the simulation ended
    CHECK_FALSE(f.scene.robotDragging());
    CHECK(drags.count() == 2);
}

TEST_CASE("run view placement: dragging requests poses that follow the pointer, keep the height and the heading") {
    SceneFixture f;
    f.scene.setPlacementEnabled(true);
    const sdk::Pose3 start = f.robot();
    QSignalSpy requests(&f.scene, &SceneModel::robotPoseRequested);

    f.drag(f.onRobot());   // nothing is held yet: ignored
    CHECK(requests.count() == 0);

    REQUIRE(f.begin(f.onRobot()));
    const QPointF target = f.pixelOf(start.position + sdk::Vec3{1.f, 0.5f, 0.f});
    f.drag(target);
    REQUIRE(requests.count() == 1);
    const Requested moved = requested(requests, 0);
    CHECK(moved.x == doctest::Approx(start.position.x + 1.0).epsilon(0.05));
    CHECK(moved.y == doctest::Approx(start.position.y + 0.5).epsilon(0.1));
    CHECK(moved.z == doctest::Approx(start.position.z));                      // the pick-up height is kept
    CHECK(moved.yaw == doctest::Approx(start.rotation.yaw()).epsilon(1e-4));  // moving does not turn the robot

    f.drag(target);   // the same pixel asks for nothing new, another one does
    CHECK(requests.count() == 1);
    f.drag(target + QPointF(30, 0));
    CHECK(requests.count() == 2);

    f.scene.endRobotDrag();
    f.drag(target + QPointF(60, 0));
    CHECK(requests.count() == 2);   // released: the pointer no longer moves the robot
}

TEST_CASE("run view placement: the wheel turns the held robot about the grabbed spot, Ctrl snaps") {
    SceneFixture f;
    f.scene.setPlacementEnabled(true);
    QSignalSpy requests(&f.scene, &SceneModel::robotPoseRequested);
    CHECK_FALSE(f.scene.turnRobotDrag(1.f, 0));   // not holding it: the wheel keeps zooming the camera
    CHECK(requests.count() == 0);

    REQUIRE(f.begin(f.onRobot()));
    const float yaw0 = f.robot().rotation.yaw();
    REQUIRE(f.scene.turnRobotDrag(2.f, 0));   // two notches counter-clockwise
    REQUIRE(requests.count() == 1);
    const Requested turned = requested(requests, 0);
    CHECK(turned.yaw == doctest::Approx(yaw0 + 10.0 * sdk::kPi / 180.0).epsilon(1e-3));
    CHECK(turned.z == doctest::Approx(f.robot().position.z));

    // with Ctrl the heading lands on a multiple of 15 degrees
    REQUIRE(f.scene.turnRobotDrag(1.f, Qt::ControlModifier));
    CHECK(requested(requests, 1).yaw == doctest::Approx(15.0 * sdk::kPi / 180.0).epsilon(1e-3));
    f.scene.endRobotDrag();
}

TEST_CASE("run view placement: Ctrl snaps the dragged position to 10 cm") {
    SceneFixture f;
    f.scene.setPlacementEnabled(true);
    QSignalSpy requests(&f.scene, &SceneModel::robotPoseRequested);
    REQUIRE(f.begin(f.onRobot()));
    f.drag(f.pixelOf(f.robot().position + sdk::Vec3{0.777f, -0.333f, 0.f}), Qt::ControlModifier);
    REQUIRE(requests.count() == 1);
    const Requested pose = requested(requests, 0);
    CHECK(std::fabs(pose.x * 10.0 - std::round(pose.x * 10.0)) < 1e-3);
    CHECK(std::fabs(pose.y * 10.0 - std::round(pose.y * 10.0)) < 1e-3);
    f.scene.endRobotDrag();
}

TEST_CASE("run page: dragging the robot sends poses to the backend and the next snapshot shows the robot there") {
    FakeRunBackend backend{[] {
        FakeRunBackend::Options o;
        o.autoRun = false;
        o.startupDelayMs = 0;
        return o;
    }()};
    QTemporaryDir dir;
    QSettings settings(dir.filePath(QStringLiteral("s.ini")), QSettings::IniFormat);
    FakeJoystickProvider gamepads;
    RunPageOptions options;
    options.settings = &settings;
    options.gamepads = &gamepads;
    RunPage page(backend, options);
    page.resize(1400, 850);
    page.show();
    QApplication::processEvents();
    page.scene().setViewSize(kViewW, kViewH);

    CHECK_FALSE(page.scene().placementEnabled());   // nothing runs yet
    backend.startSimulation();
    backend.tick(3);
    page.pollNow();
    REQUIRE(page.scene().robotVisible());
    CHECK(page.scene().placementEnabled());   // the simulation is live

    const sdk::Pose3 start = backend.latestSnapshot()->chassis().pose;
    const auto grab = page.scene().camera().project(start.position, kViewW, kViewH);
    const auto target = page.scene().camera().project(start.position + sdk::Vec3{1.f, 1.f, 0.f}, kViewW, kViewH);
    REQUIRE(grab);
    REQUIRE(target);
    REQUIRE(page.scene().beginRobotDrag(grab->first, grab->second));
    page.scene().dragRobot(target->first, target->second, 0);
    page.scene().endRobotDrag();

    CHECK(backend.poseCount() == 1);
    CHECK(backend.lastRobotPose().position.x == doctest::Approx(start.position.x + 1.f).epsilon(0.05));
    CHECK(backend.lastRobotPose().position.y == doctest::Approx(start.position.y + 1.f).epsilon(0.05));
    CHECK(backend.lastRobotPose().position.z == doctest::Approx(start.position.z));
    page.pollNow();   // the view reads the snapshot the backend published
    CHECK(page.scene().chassisPosition().x() == doctest::Approx(backend.lastRobotPose().position.x));
    CHECK(page.scene().chassisPosition().y() == doctest::Approx(backend.lastRobotPose().position.y));

    backend.stopSimulation();
    page.pollNow();
    CHECK_FALSE(page.scene().placementEnabled());   // no robot to hold once the simulation ended
}
