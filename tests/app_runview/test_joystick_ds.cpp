#include <doctest/doctest.h>

#include <QSettings>
#include <QSignalSpy>
#include <QTemporaryDir>

#include "driver_station_model.h"
#include "fake_joystick.h"
#include "joystick_manager.h"
#include "joystick_types.h"
#include "run_backend.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

struct TempSettings {
    QTemporaryDir dir;
    QString path = dir.filePath(QStringLiteral("settings.ini"));
    QSettings make() const { return QSettings(path, QSettings::IniFormat); }
};

joy::RawInput keys(std::initializer_list<int> list) {
    joy::RawInput r;
    for (int k : list) r.keys.insert(k);
    return r;
}

}  // namespace

TEST_CASE("inputs round-trip through their persisted text form") {
    for (const joy::Input in : {joy::Input::none(), joy::Input::key(Qt::Key_W), joy::Input::padAxis(3), joy::Input::padButton(9),
                                joy::Input::padPov(0)})
        CHECK(joy::Input::fromString(in.toString()) == in);
    CHECK(joy::Input::fromString(QStringLiteral("garbage")).isNone());
    CHECK(joy::Input::fromString(QStringLiteral("axis:-1")).isNone());
    CHECK(joy::Input::fromString(QStringLiteral("key:x")).isNone());
}

TEST_CASE("keyboard axes, buttons and POV evaluate like a joystick") {
    const joy::JoystickProfile p = joy::JoystickProfile::defaultKeyboard();
    const auto idle = joy::evaluate(p, keys({}), true);
    CHECK(idle.present);
    CHECK(idle.axes.size() == joy::kAxisCount);
    CHECK(idle.buttons == 0);
    CHECK(idle.povs.at(0) == -1);

    const auto fwd = joy::evaluate(p, keys({Qt::Key_W, Qt::Key_D, Qt::Key_Z}), true);
    CHECK(fwd.axes[1] == doctest::Approx(-1.f));   // W pushes the left stick up (negative Y)
    CHECK(fwd.axes[0] == doctest::Approx(1.f));
    CHECK((fwd.buttons & 1u) == 1u);
    CHECK(fwd.buttonCount == joy::kButtonCount);

    const auto both = joy::evaluate(p, keys({Qt::Key_A, Qt::Key_D}), true);
    CHECK(both.axes[0] == doctest::Approx(0.f));   // opposing keys cancel

    const auto trigger = joy::evaluate(p, keys({Qt::Key_U}), true);
    CHECK(trigger.axes[2] == doctest::Approx(1.f));   // trigger: only the positive key is bound

    CHECK(joy::evaluate(p, keys({Qt::Key_I}), true).povs[0] == 0);
    CHECK(joy::evaluate(p, keys({Qt::Key_I, Qt::Key_L}), true).povs[0] == 45);
    CHECK(joy::evaluate(p, keys({Qt::Key_K}), true).povs[0] == 180);
    CHECK(joy::evaluate(p, keys({Qt::Key_K, Qt::Key_J}), true).povs[0] == 225);
    CHECK(joy::evaluate(p, keys({Qt::Key_J}), true).povs[0] == 270);

    const auto absent = joy::evaluate(p, keys({Qt::Key_W}), false);
    CHECK_FALSE(absent.present);
    CHECK(absent.axes.empty());
}

TEST_CASE("gamepad axes apply deadband and inversion") {
    joy::JoystickProfile p = joy::JoystickProfile::defaultGamepad();
    joy::RawInput raw;
    raw.axes = {0.05f, -1.f, 0.f, 0.5f, 0.f, 0.f};
    raw.buttons = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    raw.povs = {90};
    auto s = joy::evaluate(p, raw, true);
    CHECK(s.axes[0] == doctest::Approx(0.f));    // inside the deadband
    CHECK(s.axes[1] == doctest::Approx(-1.f));
    CHECK(s.axes[3] == doctest::Approx(0.5f).epsilon(0.1));
    CHECK((s.buttons & 1u) == 1u);
    CHECK(s.povs[0] == 90);

    p.axes[1].invert = true;
    CHECK(joy::evaluate(p, raw, true).axes[1] == doctest::Approx(1.f));

    raw.axes.resize(2);   // a device with fewer axes than bound: missing axes read 0
    CHECK(joy::evaluate(p, raw, true).axes[5] == doctest::Approx(0.f));
}

TEST_CASE("profiles survive a variant round trip and tolerate junk") {
    joy::JoystickProfile p = joy::JoystickProfile::defaultKeyboard();
    p.axes[0].invert = true;
    p.buttons[3] = joy::Input::key(Qt::Key_Y);
    p.pov.up = joy::Input::key(Qt::Key_8);
    const auto back = joy::JoystickProfile::fromVariant(p.toVariant(), joy::JoystickProfile::defaultGamepad());
    CHECK(back == p);

    const auto fallback = joy::JoystickProfile::defaultGamepad();
    CHECK(joy::JoystickProfile::fromVariant({}, fallback) == fallback);
}

TEST_CASE("joystick manager: keyboard slot 0 by default, keys drive the DS joystick") {
    TempSettings ts;
    QSettings settings = ts.make();
    JoystickManager mgr(nullptr, &settings);
    CHECK(mgr.slot(0).sourceId == QLatin1String("keyboard"));
    CHECK(mgr.slot(1).sourceId.isEmpty());
    CHECK_FALSE(mgr.gamepadBackendAvailable());

    mgr.handleKey(Qt::Key_W, true);
    auto states = mgr.sample();
    CHECK(states[0].present);
    CHECK(states[0].axes[1] == doctest::Approx(-1.f));
    CHECK_FALSE(states[1].present);
    mgr.handleKey(Qt::Key_W, false);
    CHECK(mgr.sample()[0].axes[1] == doctest::Approx(0.f));

    mgr.handleKey(Qt::Key_W, true);
    mgr.releaseAllKeys();
    CHECK(mgr.sample()[0].axes[1] == doctest::Approx(0.f));
}

TEST_CASE("joystick manager: learning a key rebinds and persists") {
    TempSettings ts;
    {
        QSettings settings = ts.make();
        JoystickManager mgr(nullptr, &settings);
        QSignalSpy learning(&mgr, &JoystickManager::learningChanged);
        mgr.beginLearn(0, {JoystickManager::LearnTarget::Kind::Button, 0});
        CHECK(mgr.learning());
        CHECK(mgr.handleKey(Qt::Key_B, true));   // consumed by learn mode
        CHECK_FALSE(mgr.learning());
        CHECK(learning.count() == 2);
        CHECK(mgr.profile(0).buttons[0] == joy::Input::key(Qt::Key_B));

        // Escape cancels without changing anything.
        mgr.beginLearn(0, {JoystickManager::LearnTarget::Kind::Button, 1});
        CHECK(mgr.handleKey(Qt::Key_Escape, true));
        CHECK_FALSE(mgr.learning());
        CHECK(mgr.profile(0).buttons[1] == joy::Input::key(Qt::Key_X));

        // Axis: two keys.
        mgr.beginLearn(0, {JoystickManager::LearnTarget::Kind::AxisFirst, 4});
        mgr.handleKey(Qt::Key_1, true);
        mgr.beginLearn(0, {JoystickManager::LearnTarget::Kind::AxisSecond, 4});
        mgr.handleKey(Qt::Key_2, true);
        CHECK(mgr.profile(0).axes[4].first == joy::Input::key(Qt::Key_1));
        CHECK(mgr.profile(0).axes[4].second == joy::Input::key(Qt::Key_2));

        // Unassigned slots cannot learn.
        mgr.beginLearn(3, {JoystickManager::LearnTarget::Kind::Button, 0});
        CHECK_FALSE(mgr.learning());
    }
    {
        QSettings settings = ts.make();
        JoystickManager again(nullptr, &settings);
        CHECK(again.slot(0).sourceId == QLatin1String("keyboard"));
        CHECK(again.profile(0).buttons[0] == joy::Input::key(Qt::Key_B));
        CHECK(again.profile(0).axes[4].second == joy::Input::key(Qt::Key_2));
        again.resetProfile(0);
        CHECK(again.profile(0) == joy::JoystickProfile::defaultKeyboard());
    }
}

TEST_CASE("joystick manager: gamepad detection, assignment, learning and unplugging") {
    TempSettings ts;
    QSettings settings = ts.make();
    FakeJoystickProvider provider;
    JoystickManager mgr(&provider, &settings);
    CHECK(mgr.gamepadBackendAvailable());
    CHECK(mgr.gamepads().empty());

    QSignalSpy connected(&mgr, &JoystickManager::gamepadConnected);
    auto pad = provider.addGamepad(QStringLiteral("Test Pad"));
    REQUIRE(connected.count() == 1);
    CHECK(connected.at(0).at(1).toInt() == 1);   // first free port (0 is the keyboard)
    CHECK(mgr.slot(1).sourceId == pad->info().id);
    CHECK(mgr.slotDeviceName(1) == QStringLiteral("Test Pad"));
    CHECK(mgr.availableSources().size() == 2);

    pad->setAxis(0, 0.8f);
    pad->setButton(1, true);
    pad->setPov(180);
    const auto states = mgr.sample();
    CHECK(states[1].present);
    CHECK(states[1].axes[0] > 0.7f);
    CHECK((states[1].buttons & 2u) == 2u);
    CHECK(states[1].povs[0] == 180);

    // Learn: move a different axis for "Left trigger" binding (axis 2).
    mgr.beginLearn(1, {JoystickManager::LearnTarget::Kind::AxisFirst, 2});
    mgr.sample();   // captures the baseline
    pad->setAxis(4, 1.0f);
    mgr.sample();
    CHECK_FALSE(mgr.learning());
    CHECK(mgr.profile(1).axes[2].first == joy::Input::padAxis(4));

    // Unplug: the port stays assigned but reports not present; replugging announces it again.
    const QString id = pad->info().id;
    provider.removeGamepad(id);
    CHECK_FALSE(mgr.sample()[1].present);
    CHECK_FALSE(mgr.slotConnected(1));
    CHECK(mgr.slot(1).sourceId == id);
    CHECK(mgr.slotDeviceName(1) == QStringLiteral("Test Pad"));
    provider.addGamepad(QStringLiteral("Test Pad"));
    CHECK(mgr.slotConnected(1));
    CHECK(mgr.sample()[1].present);
}

TEST_CASE("driver station: enabling needs a robot and an unlatched e-stop") {
    DriverStationModel ds;
    CHECK_FALSE(ds.canEnable());
    ds.setEnabled(true);
    CHECK_FALSE(ds.enabled());

    ds.setRobotAvailable(true);
    ds.setEnabled(true);
    CHECK(ds.enabled());
    CHECK(ds.state().enabled);

    ds.disable();
    CHECK_FALSE(ds.enabled());

    ds.setEnabled(true);
    ds.eStop();
    CHECK_FALSE(ds.enabled());
    CHECK(ds.eStopped());
    CHECK(ds.state().eStop);
    ds.setEnabled(true);
    CHECK_FALSE(ds.enabled());
    ds.clearEStop();
    ds.setEnabled(true);
    CHECK(ds.enabled());

    ds.setRobotAvailable(false);   // robot went away
    CHECK_FALSE(ds.enabled());
    ds.setRobotAvailable(true);
}

TEST_CASE("driver station: mode change disables, alliance maps to a station") {
    DriverStationModel ds;
    ds.setRobotAvailable(true);
    ds.setEnabled(true);
    ds.setMode(sdk::proto::RobotMode::Autonomous);
    CHECK_FALSE(ds.enabled());
    CHECK(ds.state().mode == sdk::proto::RobotMode::Autonomous);

    ds.setAlliance(sdk::Alliance::Blue);
    ds.setStation(3);
    CHECK(ds.state().station == sdk::proto::AllianceStation::Blue3);
    ds.setAlliance(sdk::Alliance::Red);
    ds.setStation(2);
    CHECK(ds.state().station == sdk::proto::AllianceStation::Red2);
    ds.setStation(9);
    CHECK(ds.station() == 3);

    ds.setGameMessage(QStringLiteral("RBG"));
    CHECK(ds.state().gameSpecificMessage == "RBG");
}

TEST_CASE("match timer runs segments with sim-time deltas") {
    MatchTimer t;
    t.begin({{sdk::proto::RobotMode::Autonomous, true, 2.0}, {sdk::proto::RobotMode::Teleop, false, 1.0},
             {sdk::proto::RobotMode::Teleop, true, 3.0}});
    CHECK(t.running());
    CHECK(t.remainingS() == doctest::Approx(2.0));
    CHECK_FALSE(t.advance(1.0));
    CHECK(t.remainingS() == doctest::Approx(1.0));
    CHECK(t.advance(1.5));
    CHECK(t.segmentIndex() == 1);
    CHECK_FALSE(t.current()->enabled);
    CHECK(t.advance(0.5 + 1.0));   // crosses into segment 2 by 1.0
    CHECK(t.segmentIndex() == 2);
    CHECK(t.advance(3.0));
    CHECK_FALSE(t.running());
    CHECK(t.finished());
}

TEST_CASE("driver station: auto -> teleop sequencing follows sim time and respects pause") {
    DriverStationModel ds;
    ds.setRobotAvailable(true);
    MatchSettings m;
    m.autoSeconds = 2.0;
    m.gapSeconds = 1.0;
    m.teleopSeconds = 3.0;
    ds.setMatchSettings(m);

    ds.setDsMode(DsMode::Practice);
    CHECK(ds.dsMode() == DsMode::Practice);
    CHECK_FALSE(ds.enabled());
    ds.setEnabled(true);   // Enable starts the practice sequence
    CHECK(ds.matchActive());
    CHECK(ds.enabled());
    CHECK(ds.mode() == sdk::proto::RobotMode::Autonomous);
    CHECK(ds.state().matchTimeS == doctest::Approx(2.0));

    ds.onSimTime(0);
    ds.onSimTime(1'000'000);
    CHECK(ds.state().matchTimeS == doctest::Approx(1.0));
    ds.onSimTime(1'000'000);   // paused: sim time does not advance
    CHECK(ds.state().matchTimeS == doctest::Approx(1.0));
    ds.onSimTime(2'500'000);
    CHECK_FALSE(ds.enabled());   // transition gap
    CHECK(ds.matchPhaseText() == QCoreApplication::translate("DriverStation", "Transition"));
    ds.onSimTime(3'500'000);
    CHECK(ds.enabled());
    CHECK(ds.mode() == sdk::proto::RobotMode::Teleop);
    ds.onSimTime(7'000'000);
    CHECK_FALSE(ds.enabled());
    CHECK_FALSE(ds.matchActive());
    CHECK(ds.state().matchTimeS == doctest::Approx(-1.0));

    CHECK(ds.dsMode() == DsMode::Practice);
    CHECK(ds.mode() == sdk::proto::RobotMode::Autonomous);   // ready for the next run

    // A manual disable ends the match.
    ds.setEnabled(true);
    CHECK(ds.matchActive());
    ds.disable();
    CHECK_FALSE(ds.matchActive());
    // A sim reset ends it too, and a jump backwards in sim time never advances the clock.
    ds.setEnabled(true);
    ds.onSimReset();
    CHECK_FALSE(ds.matchActive());
    CHECK_FALSE(ds.enabled());
}

TEST_CASE("driver station: single-period match and test mode") {
    DriverStationModel ds;
    ds.setRobotAvailable(true);
    MatchSettings m;
    m.autoSeconds = 1.0;
    ds.setMatchSettings(m);
    ds.setDsMode(DsMode::Auto);
    CHECK(ds.dsMode() == DsMode::Auto);
    CHECK(ds.startMatch());
    CHECK(ds.enabled());
    CHECK_FALSE(ds.matchActive());   // only Practice has a match clock
    CHECK(ds.state().matchTimeS == doctest::Approx(-1.0));
    ds.onSimTime(0);
    ds.onSimTime(1'100'000);
    CHECK(ds.enabled());   // runs until Disable
    ds.disable();
    CHECK_FALSE(ds.enabled());

    ds.setMode(sdk::proto::RobotMode::Test);
    CHECK(ds.startMatch());
    CHECK(ds.enabled());
    CHECK_FALSE(ds.matchActive());   // test mode has no clock
}

TEST_CASE("driver station publishes on change and as a keep-alive") {
    DriverStationModel ds;
    std::array<sdk::proto::JoystickState, 6> js{};
    QSignalSpy spy(&ds, &DriverStationModel::stateReady);
    CHECK(ds.publish(js, 1000));
    CHECK_FALSE(ds.publish(js, 1100));
    CHECK(spy.count() == 1);
    js[0].present = true;
    js[0].axes = {0.5f};
    CHECK(ds.publish(js, 1120));
    CHECK_FALSE(ds.publish(js, 1130));
    CHECK(ds.publish(js, 1130 + DriverStationModel::kKeepAliveMs));   // heartbeat
    ds.setRobotAvailable(true);
    ds.setEnabled(true);
    CHECK(ds.publish(js, 1200 + DriverStationModel::kKeepAliveMs));
    CHECK(spy.count() == 4);
    const auto last = spy.last().at(0).value<sdk::proto::DsState>();
    CHECK(last.enabled);
    CHECK(last.joysticks[0].present);
}

TEST_CASE("driver station settings persist alliance, station, message and match times") {
    TempSettings ts;
    {
        QSettings s = ts.make();
        DriverStationModel ds(&s);
        ds.setAlliance(sdk::Alliance::Blue);
        ds.setStation(2);
        ds.setGameMessage(QStringLiteral("ABC"));
        MatchSettings m = ds.matchSettings();
        m.autoSeconds = 15;
        m.gapSeconds = 5;
        ds.setMatchSettings(m);
    }
    QSettings s = ts.make();
    DriverStationModel again(&s);
    CHECK(again.alliance() == sdk::Alliance::Blue);
    CHECK(again.station() == 2);
    CHECK(again.gameMessage() == QStringLiteral("ABC"));
    CHECK(again.matchSettings().autoSeconds == doctest::Approx(15.0));
    CHECK(again.matchSettings().gapSeconds == doctest::Approx(5.0));
}
