#include <doctest/doctest.h>

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QDialog>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QTemporaryDir>
#include <QFrame>
#include <QToolButton>
#include <QTest>

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
        backend.startSimulation();
        page.pollNow();
    }
};

}  // namespace

TEST_CASE("run page: the 3D scene loads and the idle state invites the user to start") {
    Fixture f;
    CHECK_MESSAGE(f.page.viewport()->sceneLoaded(), f.page.viewport()->sceneError().toStdString());
    CHECK(f.page.viewport()->overlay()->isVisible());
    CHECK(f.page.viewport()->overlay()->restartButton()->isVisible());
    CHECK(f.page.restartRobotAction()->isEnabled());
    CHECK_FALSE(f.page.restartAction()->isEnabled());
    CHECK_FALSE(f.page.speedCombo()->isEnabled());
    CHECK_FALSE(f.page.driverStationPanel()->enableButton()->isEnabled());   // no robot to enable
    CHECK(f.page.driverStationPanel()->modeButton(DsMode::Teleop)->isChecked());   // Teleop is selected from the start
    CHECK(f.page.viewActions().size() == 6);   // overlays and console live in the shell's View menu
}

TEST_CASE("run page: start connects, drives the view models and enables the DS") {
    Fixture f;
    f.connectRobot();
    CHECK(f.backend.startCount() == 1);
    CHECK(f.backend.status().link == LinkState::Connected);
    CHECK(f.backend.status().simulating);
    CHECK_FALSE(f.page.viewport()->overlay()->isVisible());
    CHECK(f.page.restartRobotAction()->isEnabled());
    CHECK(f.page.restartAction()->isEnabled());
    CHECK(f.page.speedCombo()->isEnabled());
    CHECK(f.page.scene().robotVisible());
    CHECK(f.page.hud().rowCount() == 5);
    CHECK(f.page.stateWindow()->isHidden());   // the Robot state window opens on demand
    CHECK(f.page.stats().hasData);
    CHECK(f.page.driverStationPanel()->enableButton()->isEnabled());
    CHECK(f.page.console()->totalLineCount() >= 3);   // start-up output arrived through the log signal
}

TEST_CASE("run page: the Robot state window lists mechanisms and sensors and follows its toggle") {
    Fixture f;
    f.connectRobot();
    auto* button = f.page.viewport()->findChild<QToolButton*>(QStringLiteral("runview.stateButton"));
    REQUIRE(button != nullptr);
    CHECK(button->isCheckable());
    CHECK(f.page.viewActions().contains(f.page.stateAction()));
    CHECK_FALSE(f.page.stateWindow()->isVisible());

    button->click();
    CHECK(f.page.stateAction()->isChecked());
    REQUIRE(f.page.stateWindow()->isVisible());
    f.page.pollNow();
    StateModel* model = f.page.stateWindow()->model();
    CHECK(model->rowCount(model->findKey(QStringLiteral("section:mechanisms"))) == 3);
    const QModelIndex sensors = model->findKey(QStringLiteral("section:sensors"));
    REQUIRE(sensors.isValid());
    CHECK(model->rowCount(sensors) == 2);
    CHECK(model->findKey(QStringLiteral("zone:pickup")).isValid());
    CHECK(model->findKey(QStringLiteral("camera:cam_rear")).isValid());

    f.page.stateWindow()->close();   // the user closes it: the toggle follows
    CHECK_FALSE(f.page.stateAction()->isChecked());
    CHECK_FALSE(f.page.stateWindow()->isVisible());

    f.page.stateAction()->setChecked(true);
    CHECK(f.page.stateWindow()->isVisible());
    f.page.stateAction()->setChecked(false);
    CHECK_FALSE(f.page.stateWindow()->isVisible());
}

TEST_CASE("run page: simulation stats live in a framed group of the Driver Station panel") {
    Fixture f;
    f.connectRobot();
    f.page.pollNow();
    auto* frame = f.page.driverStationPanel()->findChild<QFrame*>(QStringLiteral("runview.dsStats"));
    REQUIRE(frame != nullptr);
    bool sawHz = false;
    for (QLabel* l : frame->findChildren<QLabel*>()) sawHz = sawHz || l->text().contains(QStringLiteral(" Hz"));
    CHECK(sawHz);
}

TEST_CASE("run page: practice highlights the running segment; setup and joysticks open in a popup") {
    Fixture f;
    f.connectRobot();
    auto* panel = f.page.driverStationPanel();
    auto& ds = f.page.driverStation();
    const auto live = [&](DsMode m) { return panel->modeButton(m)->property("live").toBool(); };

    ds.setDsMode(DsMode::Practice);
    ds.setEnabled(true);
    panel->refresh();
    CHECK(panel->modeButton(DsMode::Practice)->isChecked());
    CHECK(live(DsMode::Auto));
    CHECK(f.page.viewport()->matchClockLabel()->text().contains(QStringLiteral(":")));   // timer shows in the 3D view
    CHECK_FALSE(live(DsMode::Teleop));

    const double auto_s = ds.matchSettings().autoSeconds;
    ds.onSimTime(0);
    const auto jumpTo = [&](double s) {
        for (double t = 0.5; t <= s; t += 0.5) ds.onSimTime(static_cast<std::int64_t>(t * 1e6));
    };
    jumpTo(auto_s + ds.matchSettings().gapSeconds + 1.0);   // into teleop
    panel->refresh();
    CHECK(live(DsMode::Teleop));
    CHECK_FALSE(live(DsMode::Auto));

    ds.disable();
    panel->refresh();
    CHECK_FALSE(live(DsMode::Teleop));

    CHECK_FALSE(panel->settingsDialog()->isVisible());
    panel->showSettings(1);
    CHECK(panel->settingsDialog()->isVisible());
    panel->settingsDialog()->hide();
}

TEST_CASE("run page: enable, space to disable, e-stop and mode") {
    Fixture f;
    f.connectRobot();
    auto* panel = f.page.driverStationPanel();

    panel->enableButton()->click();
    f.page.publishDriverStation();
    CHECK(f.backend.lastDriverStation().enabled);

    panel->disableAction()->trigger();   // what the Enter shortcut triggers
    f.page.publishDriverStation();
    CHECK_FALSE(f.backend.lastDriverStation().enabled);
    CHECK_FALSE(panel->enableButton()->isChecked());

    panel->enableButton()->click();   // the enable button toggles
    f.page.publishDriverStation();
    CHECK(f.backend.lastDriverStation().enabled);
    panel->enableButton()->click();
    f.page.publishDriverStation();
    CHECK_FALSE(f.backend.lastDriverStation().enabled);

    panel->enableButton()->click();
    panel->eStopAction()->trigger();
    f.page.publishDriverStation();
    CHECK(f.backend.lastDriverStation().eStop);
    CHECK_FALSE(f.backend.lastDriverStation().enabled);
    CHECK_FALSE(panel->enableButton()->isEnabled());   // latched until cleared
    panel->eStopButton()->click();                     // clears
    CHECK(panel->enableButton()->isEnabled());

    // Enter disables and Space is the e-stop: neither is a joystick input.
    CHECK_FALSE(f.page.handleKey(Qt::Key_Space, true));
    CHECK_FALSE(f.page.handleKey(Qt::Key_Return, true));
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
    CHECK(f.page.consoleVisible());
    CHECK(overlay->restartButton()->isVisible());
    CHECK(f.page.restartRobotAction()->isEnabled());
    CHECK_FALSE(f.page.driverStationPanel()->enableButton()->isEnabled());
    CHECK_FALSE(f.page.scene().robotVisible());

    overlay->restartButton()->click();
    CHECK(f.backend.startCount() == 2);
    f.page.pollNow();
    f.backend.startSimulation();
    f.page.pollNow();
    CHECK_FALSE(overlay->isVisible());
}

TEST_CASE("run page: handshake timeout and protocol mismatch are explained") {
    Fixture f;
    f.backend.launchRobot();
    f.backend.simulateHandshakeTimeout();
    f.page.pollNow();
    auto* overlay = f.page.viewport()->overlay();
    CHECK(overlay->isVisible());
    CHECK(overlay->restartButton()->isVisible());
    overlay->restartButton()->click();
    CHECK(f.backend.status().link == LinkState::Connected);

    f.backend.simulateProtocolMismatch(QStringLiteral("robot v1, app v2"));
    f.page.pollNow();
    CHECK(overlay->detailLabel()->text() == QStringLiteral("robot v1, app v2"));
}

TEST_CASE("run page: restart resets the physics, the dropdown also reboots the robot code, and speed reaches the backend") {
    Fixture f;
    f.connectRobot();
    f.page.speedCombo()->setCurrentIndex(4);
    emit f.page.speedCombo()->activated(4);
    CHECK(f.backend.timeScale() == doctest::Approx(2.0));

    f.page.driverStation().setEnabled(true);
    f.page.restartAction()->trigger();
    CHECK(f.backend.resetCount() == 1);
    CHECK(f.backend.startCount() == 1);   // the robot code was not relaunched
    CHECK_FALSE(f.page.driverStation().enabled());

    f.page.driverStation().setEnabled(true);
    f.page.restartRobotAction()->trigger();
    CHECK(f.backend.startCount() == 2);
    CHECK_FALSE(f.page.driverStation().enabled());
    f.page.pollNow();
    CHECK(f.backend.status().simulating);   // the simulation resumes by itself after the reboot

    auto* viewport = f.page.viewport();
    CHECK(viewport->findChild<QToolButton*>(QStringLiteral("runview.restartButton")) != nullptr);
    CHECK(viewport->findChild<QToolButton*>(QStringLiteral("runview.speedButton")) != nullptr);
    CHECK(f.page.findChild<QToolButton*>(QStringLiteral("runview.homeButton")) != nullptr);
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

TEST_CASE("run page: restart shortcuts work with the dropdown closed") {
    Fixture f;
    f.connectRobot();
    f.page.activateWindow();
    const int starts = f.backend.startCount();
    auto* search = f.page.console()->findChild<QLineEdit*>();
    REQUIRE(search != nullptr);
    int restarts = 0;
    for (QWidget* focus : {static_cast<QWidget*>(f.page.viewport()),
                           static_cast<QWidget*>(f.page.driverStationPanel()->enableButton()),
                           static_cast<QWidget*>(search)}) {
        focus->setFocus();
        QApplication::processEvents();
        QTest::keyClick(focus, Qt::Key_R, Qt::ControlModifier | Qt::ShiftModifier);
        QApplication::processEvents();
        CHECK(f.backend.startCount() == starts + ++restarts);
        CHECK_FALSE(f.page.driverStation().enabled());
    }
    QTest::keyClick(f.page.viewport(), Qt::Key_R, Qt::ControlModifier);
    CHECK(f.backend.startCount() == starts + restarts);
    CHECK(f.backend.resetCount() == 1);
}

TEST_CASE("run page: home button in the 3D view resets the camera") {
    Fixture f;
    f.connectRobot();
    auto* home = f.page.findChild<QToolButton*>(QStringLiteral("runview.homeButton"));
    REQUIRE(home != nullptr);
    const float homeDistance = f.page.scene().camera().distance;
    f.page.scene().camera().zoom(5.f);
    CHECK(f.page.scene().camera().distance != doctest::Approx(homeDistance));
    home->click();
    CHECK(f.page.scene().camera().distance == doctest::Approx(homeDistance));
}

TEST_CASE("run page: console visibility control") {
    Fixture f;
    f.connectRobot();
    f.page.setConsoleVisible(false);
    CHECK_FALSE(f.page.consoleVisible());
    f.page.setConsoleVisible(true);
    CHECK(f.page.consoleVisible());
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
