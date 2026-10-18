#include <doctest/doctest.h>

#include <QAction>
#include <QSettings>
#include <QTemporaryDir>

#include "fake_joystick.h"
#include "fake_run_backend.h"
#include "run_page.h"

using namespace wpiemu::app::runview;

namespace {

FakeRunBackend::Options options() {
    FakeRunBackend::Options o;
    o.autoRun = false;
    o.startupDelayMs = 0;
    return o;
}

QAction* actionNamed(RunPage& page, const QString& text) {
    for (QAction* a : page.viewActions())
        if (a->text() == text) return a;
    return nullptr;
}

}  // namespace

TEST_CASE("run page vision: the overlay toggles are remembered between runs") {
    QTemporaryDir dir;
    const QString path = dir.filePath(QStringLiteral("view.ini"));
    FakeRunBackend backend(options());
    FakeJoystickProvider gamepads;
    const auto pageOptions = [&](QSettings& settings) {
        RunPageOptions o;
        o.settings = &settings;
        o.gamepads = &gamepads;
        return o;
    };
    {
        QSettings settings(path, QSettings::IniFormat);
        RunPage page(backend, pageOptions(settings));
        REQUIRE(actionNamed(page, QStringLiteral("Cameras")) != nullptr);
        REQUIRE(actionNamed(page, QStringLiteral("AprilTags")) != nullptr);
        CHECK(page.scene().camerasVisible());  // both layers start on
        CHECK(page.scene().tagsVisible());
        actionNamed(page, QStringLiteral("Cameras"))->setChecked(false);
        actionNamed(page, QStringLiteral("AprilTags"))->setChecked(false);
        actionNamed(page, QStringLiteral("Game HUD"))->setChecked(false);
        CHECK_FALSE(page.scene().camerasVisible());
        CHECK_FALSE(page.scene().tagsVisible());
    }
    {
        QSettings settings(path, QSettings::IniFormat);
        RunPage page(backend, pageOptions(settings));
        CHECK_FALSE(page.scene().camerasVisible());  // remembered
        CHECK_FALSE(page.scene().tagsVisible());
        CHECK_FALSE(actionNamed(page, QStringLiteral("Cameras"))->isChecked());
        CHECK_FALSE(actionNamed(page, QStringLiteral("Game HUD"))->isChecked());
        CHECK(page.scene().zonesVisible());  // what was not touched stays on
        CHECK(actionNamed(page, QStringLiteral("Mechanism values"))->isChecked());
        actionNamed(page, QStringLiteral("Cameras"))->setChecked(true);
        CHECK(page.scene().camerasVisible());
    }
}

TEST_CASE("run page vision: problems appear on the camera cards and once in the console") {
    QTemporaryDir dir;
    QSettings settings(dir.filePath(QStringLiteral("s.ini")), QSettings::IniFormat);
    FakeRunBackend backend(options());
    FakeJoystickProvider gamepads;
    RunPageOptions o;
    o.settings = &settings;
    o.gamepads = &gamepads;
    RunPage page(backend, o);
    backend.startSimulation();
    backend.tick(200);
    page.pollNow();
    CHECK(page.cameraHud().rowCount() == 2);
    CHECK(page.cameraHud().noticeModel().rowCount() == 0);

    backend.simulateUnconfiguredCamera(QStringLiteral("photon-extra"));
    page.pollNow();
    REQUIRE(page.cameraHud().noticeModel().rowCount() == 1);
    CHECK(page.console()->visibleText().contains(QStringLiteral("Camera not configured")));
    CHECK(page.console()->visibleText().contains(QStringLiteral("photon-extra")));
    const int lines = page.console()->totalLineCount();
    page.pollNow();
    CHECK(page.console()->totalLineCount() == lines);  // the same problem is not logged again

    backend.simulateCameraFault(QStringLiteral("front"),
                                FakeRunBackend::CameraFault::PhotonUuidUnsupported);
    page.pollNow();
    CHECK(page.cameraHud().noticeModel().rowCount() == 2);
    CHECK(page.console()->visibleText().contains(QStringLiteral("Camera problem")));
    CHECK(page.cameraHud().card(0).state == CameraState::Problem);

    backend.simulateCrash();  // the run is over: the cards go away with the snapshot
    page.pollNow();
    CHECK(page.cameraHud().rowCount() == 0);
    CHECK(page.cameraHud().noticeModel().rowCount() == 0);
}
