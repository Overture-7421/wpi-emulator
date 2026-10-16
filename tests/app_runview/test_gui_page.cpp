#include <doctest/doctest.h>

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QKeySequence>
#include <QLabel>
#include <QPushButton>
#include <QSettings>
#include <QTemporaryDir>
#include <QToolBar>

#include "fake_joystick.h"
#include "fake_run_backend.h"
#include "joystick_editor.h"
#include "run_page.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

struct Fixture {
    QTemporaryDir dir;
    QSettings settings{dir.filePath(QStringLiteral("s.ini")), QSettings::IniFormat};
    FakeRunBackend backend{[] {
        FakeRunBackend::Options o;
        o.autoRun = false;
        o.startupDelayMs = 0;
        return o;
    }()};
    FakeJoystickProvider gamepads;
    RunPage page{backend, [this] {
                     RunPageOptions o;
                     o.settings = &settings;
                     o.gamepads = &gamepads;
                     return o;
                 }()};

    Fixture() {
        page.resize(1400, 850);
        page.show();
        QApplication::processEvents();
    }

    void connectRobot() {
        page.startAction()->trigger();
        page.pollNow();
    }
};

}  // namespace

TEST_CASE("run page: the 3D scene loads and the idle state invites the user to start") {
    Fixture f;
    CHECK_MESSAGE(f.page.viewport()->sceneLoaded(), f.page.viewport()->sceneError().toStdString());
    CHECK(f.page.viewport()->overlay()->isVisible());
    CHECK(f.page.viewport()->overlay()->startButton()->isVisible());
    CHECK(f.page.startAction()->isEnabled());
    CHECK_FALSE(f.page.stopAction()->isEnabled());
    CHECK_FALSE(f.page.resetAction()->isEnabled());
    CHECK_FALSE(f.page.driverStationPanel()->enableButton()->isEnabled());   // no robot to enable
    CHECK(f.page.statusChip()->accessibleName() == QStringLiteral("Robot stopped"));
}

TEST_CASE("run page: start connects, drives the view models and enables the DS") {
    Fixture f;
    f.connectRobot();
    CHECK(f.backend.startCount() == 1);
    CHECK(f.backend.status().link == LinkState::Connected);
    CHECK(f.page.statusChip()->accessibleName() == QStringLiteral("Connected"));
    CHECK_FALSE(f.page.viewport()->overlay()->isVisible());
    CHECK(f.page.stopAction()->isEnabled());
    CHECK_FALSE(f.page.startAction()->isEnabled());
    CHECK(f.page.resetAction()->isEnabled());
    CHECK(f.page.scene().robotVisible());
    CHECK(f.page.hud().rowCount() == 5);
    CHECK(f.page.mechanisms().rowCount() == 3);
    CHECK(f.page.stats().hasData);
    CHECK(f.page.driverStationPanel()->enableButton()->isEnabled());
    CHECK(f.page.console()->totalLineCount() >= 3);   // start-up output arrived through the log signal
}

TEST_CASE("run page: enable, space to disable, e-stop and mode") {
    Fixture f;
    f.connectRobot();
    auto* panel = f.page.driverStationPanel();

    panel->enableButton()->click();
    f.page.publishDriverStation();
    CHECK(f.backend.lastDriverStation().enabled);

    panel->disableAction()->trigger();   // what the Space shortcut triggers
    f.page.publishDriverStation();
    CHECK_FALSE(f.backend.lastDriverStation().enabled);
    CHECK(panel->disableButton()->isChecked());

    panel->enableButton()->click();
    panel->eStopAction()->trigger();
    f.page.publishDriverStation();
    CHECK(f.backend.lastDriverStation().eStop);
    CHECK_FALSE(f.backend.lastDriverStation().enabled);
    CHECK_FALSE(panel->enableButton()->isEnabled());   // latched until cleared
    panel->eStopButton()->click();                     // clears
    CHECK(panel->enableButton()->isEnabled());

    // Space has no effect as a joystick input.
    CHECK_FALSE(f.page.handleKey(Qt::Key_Space, true));
    CHECK_FALSE(f.page.handleKey(Qt::Key_Shift, true));
}

TEST_CASE("run page: keyboard joystick reaches the robot and moves it") {
    Fixture f;
    f.connectRobot();
    f.page.driverStation().setEnabled(true);
    f.page.handleKey(Qt::Key_W, true);   // forward
    f.page.publishDriverStation();
    CHECK(f.backend.lastDriverStation().joysticks[0].axes[1] == doctest::Approx(-1.f));
    const float x0 = f.backend.latestSnapshot()->chassis().pose.position.x;
    f.backend.tick(200);
    f.page.pollNow();
    CHECK(f.backend.latestSnapshot()->chassis().pose.position.x > x0 + 0.5f);
    CHECK(f.page.scene().chassisPosition().x() > x0 + 0.5f);
    f.page.handleKey(Qt::Key_W, false);
    f.page.publishDriverStation();
    CHECK(f.backend.lastDriverStation().joysticks[0].axes[1] == doctest::Approx(0.f));
}

TEST_CASE("run page: a robot crash shows the error state and offers a restart") {
    Fixture f;
    f.connectRobot();
    f.page.console()->setFollowTail(true);
    f.backend.simulateCrash(3);
    f.page.pollNow();
    auto* overlay = f.page.viewport()->overlay();
    CHECK(overlay->isVisible());
    CHECK(overlay->titleLabel()->text().contains(QStringLiteral("crashed")));
    CHECK(f.page.statusChip()->accessibleName().contains(QStringLiteral("3")));
    CHECK(f.page.consoleVisible());
    CHECK(overlay->startButton()->isVisible());
    CHECK(f.page.startAction()->isEnabled());
    CHECK_FALSE(f.page.driverStationPanel()->enableButton()->isEnabled());
    CHECK_FALSE(f.page.scene().robotVisible());

    overlay->startButton()->click();
    CHECK(f.backend.startCount() == 2);
    f.page.pollNow();
    CHECK(f.page.statusChip()->accessibleName() == QStringLiteral("Connected"));
}

TEST_CASE("run page: handshake timeout and protocol mismatch are explained") {
    Fixture f;
    f.backend.startRobot();
    f.backend.simulateHandshakeTimeout();
    f.page.pollNow();
    auto* overlay = f.page.viewport()->overlay();
    CHECK(overlay->isVisible());
    CHECK(f.page.statusChip()->accessibleName().contains(QStringLiteral("Handshake")));
    CHECK(overlay->restartButton()->isVisible());
    overlay->restartButton()->click();
    CHECK(f.backend.status().link == LinkState::Connected);

    f.backend.simulateProtocolMismatch(QStringLiteral("robot v1, app v2"));
    f.page.pollNow();
    CHECK(overlay->detailLabel()->text() == QStringLiteral("robot v1, app v2"));
    CHECK(f.page.statusChip()->accessibleName().contains(QStringLiteral("mismatch")));
}

TEST_CASE("run page: pause, step, speed and reset reach the backend") {
    Fixture f;
    f.connectRobot();
    CHECK_FALSE(f.page.stepAction()->isEnabled());
    f.page.pauseAction()->trigger();
    CHECK(f.backend.paused());
    f.page.pollNow();
    CHECK(f.page.stepAction()->isEnabled());
    f.page.stepAction()->trigger();
    CHECK(f.backend.stepRequests() == 1);

    f.page.speedCombo()->setCurrentIndex(4);
    emit f.page.speedCombo()->activated(4);
    CHECK(f.backend.timeScale() == doctest::Approx(2.0));

    f.page.pauseAction()->trigger();
    CHECK_FALSE(f.backend.paused());

    f.page.driverStation().setEnabled(true);
    f.page.resetAction()->trigger();
    CHECK(f.backend.resetCount() == 1);
    CHECK_FALSE(f.page.driverStation().enabled());

    f.page.stopAction()->trigger();
    f.page.pollNow();
    CHECK(f.backend.status().process == RobotProcessState::Exited);
    CHECK(f.page.viewport()->overlay()->isVisible());
}

TEST_CASE("run page: console receives logs and can be searched and copied") {
    Fixture f;
    f.backend.emitLog(sdk::LogLevel::Error, LogStream::Stderr, QStringLiteral("robot"), QStringLiteral("something exploded"));
    f.backend.emitLog(sdk::LogLevel::Info, LogStream::Stdout, QStringLiteral("robot"), QStringLiteral("all fine"));
    auto* console = f.page.console();
    CHECK(console->totalLineCount() == 2);
    console->filter().setSearchText(QStringLiteral("exploded"));
    CHECK(console->visibleLineCount() == 1);
    CHECK(console->visibleText().contains(QStringLiteral("something exploded")));
    console->copy();
    CHECK(QApplication::clipboard()->text().contains(QStringLiteral("something exploded")));
    console->filter().setSearchText({});
    console->clear();
    CHECK(console->totalLineCount() == 0);
}

TEST_CASE("run page: camera and overlay controls") {
    Fixture f;
    f.connectRobot();
    f.page.setCameraMode(2);
    CHECK(f.page.scene().cameraMode() == CameraMode::TopDown);
    f.page.setCameraMode(1);
    CHECK(f.page.scene().cameraMode() == CameraMode::Follow);
    f.page.setConsoleVisible(false);
    CHECK_FALSE(f.page.consoleVisible());
    f.page.setConsoleVisible(true);
    CHECK(f.page.consoleVisible());

    f.backend.setRealDsState(RealDsState::Connected);
    CHECK(f.page.findChild<StatusChip*>(QStringLiteral("runview.realDsChip"))->accessibleName().contains(QStringLiteral("connected")));
}

TEST_CASE("run page: joystick editor learns a key binding and a gamepad appears") {
    Fixture f;
    auto* editor = f.page.driverStationPanel()->joystickEditor();
    editor->setCurrentSlot(0);
    CHECK(editor->deviceCombo()->count() >= 2);   // not assigned + keyboard

    editor->buttonBindButton(0)->click();
    CHECK(f.page.joysticks().learning());
    CHECK(f.page.handleKey(Qt::Key_G, true));
    CHECK_FALSE(f.page.joysticks().learning());
    CHECK(f.page.joysticks().profile(0).buttons[0] == joy::Input::key(Qt::Key_G));
    CHECK(editor->buttonBindButton(0)->text() == QKeySequence(Qt::Key_G).toString(QKeySequence::NativeText));

    auto pad = f.gamepads.addGamepad(QStringLiteral("Test Pad"));
    CHECK(f.page.joysticks().slot(1).sourceId == pad->info().id);
    CHECK_FALSE(editor->notice()->isHidden());
    editor->setCurrentSlot(1);
    CHECK(editor->axisFirstButton(0)->text() == joy::Input::padAxis(0).displayText());

    pad->setAxis(0, 1.f);
    f.page.publishDriverStation();
    CHECK(f.backend.lastDriverStation().joysticks[1].present);
    CHECK(f.backend.lastDriverStation().joysticks[1].axes[0] > 0.9f);
}
