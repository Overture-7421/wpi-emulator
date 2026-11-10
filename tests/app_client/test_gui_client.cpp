// The client window under the offscreen platform, driven by the scripted demo session: screens, banners for every connection
// state, the join form's errors and robot report, the zone panel, the diagnostics overlay, keyboard control and persisted settings.
// (Offscreen runs do not validate Quick 3D rendering; the models behind the scene are checked.)
#include <doctest/doctest.h>

#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QStackedWidget>
#include <QTemporaryDir>
#include <QTest>

#include "client/client_text.h"
#include "client/client_window.h"
#include "client/demo_session.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/net/invite.h"

using namespace wpiemu;
using namespace wpiemu::app::client;
namespace mc = wpiemu::core::match;

namespace {

struct Fixture {
    QTemporaryDir dir;
    std::unique_ptr<ClientServices> services = ClientServices::create().value();
    std::unique_ptr<ClientWindow> window;

    explicit Fixture(DemoScenario scenario) {
        ClientWindowOptions options;
        options.settingsFile = dir.filePath(QStringLiteral("client.ini"));
        options.session = std::make_unique<DemoClientSession>(*services, scenario);
        options.useGamepads = false;
        window = std::make_unique<ClientWindow>(std::move(options));
        window->show();
        pump(120);
    }
    // A window on the same settings file (no demo session needed).
    static std::unique_ptr<ClientWindow> reopen(const QString& ini, ClientServices& services) {
        ClientWindowOptions options;
        options.settingsFile = ini;
        options.session = std::make_unique<DemoClientSession>(services, DemoScenario::Join);
        options.useGamepads = false;
        auto w = std::make_unique<ClientWindow>(std::move(options));
        w->show();
        return w;
    }
    void pump(int ms) { QTest::qWait(ms); }

    QString robotFile(float massKg = 55.f, bool broken = false) {
        sdk::RobotConfig robot = demoRobotConfig(*services);
        robot.chassis.massKg = massKg;
        const QString path = dir.filePath(QStringLiteral("robot.json"));
        QFile file(path);
        REQUIRE(file.open(QIODevice::WriteOnly));
        if (broken) {
            file.write("{ this is not json");
        } else {
            const core::config::RobotConfigCodec codec;
            file.write(QByteArray::fromStdString(core::config::JsonValueCodec().dump(codec.toValue(robot), true)));
        }
        return path;
    }
};

}  // namespace

TEST_CASE("client window: the first screen is the join form with the production relay chosen") {
    Fixture f(DemoScenario::Join);
    CHECK_FALSE(f.window->showingDrive());
    JoinPage* page = f.window->joinPage();
    REQUIRE(page != nullptr);
    CHECK(page->relayCombo()->currentData().toInt() == static_cast<int>(app::RelayChoice::Production));
    CHECK(page->joinButton()->isEnabled());
    CHECK_FALSE(page->cancelButton()->isVisible());
    CHECK(page->inviteEdit()->placeholderText() == "XXXXX-XXXXX-XXXXX-XXXXX");
}

TEST_CASE("client window: joining with nothing filled in lists every field that is wrong, and a wrong invite format says so") {
    Fixture f(DemoScenario::Join);
    JoinPage* page = f.window->joinPage();
    page->nameEdit()->clear();
    page->inviteEdit()->setText("ABCDE-12");
    page->joinButton()->click();
    f.pump(50);
    CHECK_FALSE(f.window->showingDrive());
    CHECK_FALSE(page->fieldError(JoinField::Invite)->isHidden());
    CHECK(page->fieldError(JoinField::Invite)->text() == fieldErrorText({JoinField::Invite, "invite.format", ""}));
    CHECK_FALSE(page->fieldError(JoinField::DisplayName)->isHidden());
    CHECK_FALSE(page->fieldError(JoinField::Robot)->isHidden());
    CHECK(f.window->controller().presentation().screen == Screen::Join);

    // The errors follow the edits: fixing the invite clears its message.
    page->inviteEdit()->setText(QString::fromStdString(net::inviteText(net::generateInvite().value())));
    CHECK(page->fieldError(JoinField::Invite)->isHidden());
}

TEST_CASE("client window: the robot picker shows the robot, what the host will clamp and why a file is refused") {
    Fixture f(DemoScenario::Join);
    JoinPage* page = f.window->joinPage();

    f.window->controller().chooseRobot(f.robotFile(55.f));
    CHECK_FALSE(page->robotReport()->isHidden());
    CHECK(page->robotReport()->text().contains("Demo Bot"));
    CHECK_FALSE(page->robotReport()->text().contains("adjust"));

    f.window->controller().chooseRobot(f.robotFile(500.f));   // far beyond an FRC robot
    CHECK(page->robotReport()->text().contains("adjust"));
    CHECK(page->robotReport()->text().contains("mass"));

    f.window->controller().chooseRobot(f.robotFile(55.f, true));
    CHECK(page->robotReport()->text().contains("cannot be used"));
    CHECK(f.window->controller().robot().ok == false);

    f.window->controller().chooseRobot(f.dir.filePath(QStringLiteral("missing.json")));
    CHECK(page->robotReport()->text().contains("cannot be used"));
}

TEST_CASE("client window: a complete form joins, shows the drive screen, and leaving returns to the form") {
    Fixture f(DemoScenario::Join);
    JoinPage* page = f.window->joinPage();
    page->setInvite(QString::fromStdString(net::inviteText(net::generateInvite().value())));
    page->nameEdit()->setText("Iron Owls");
    page->robotEdit()->setText(f.robotFile());
    page->directEdit()->setText("192.168.1.20");
    page->joinButton()->click();
    f.pump(200);

    REQUIRE(f.window->showingDrive());
    DrivePage* drive = f.window->drivePage();
    CHECK(drive->banner()->isHidden());                      // live: nothing to say
    CHECK(drive->whoLabel()->text().contains("You"));
    CHECK(f.window->controller().presentation().banner == Banner::None);

    drive->leaveButton()->click();
    f.pump(100);
    CHECK_FALSE(f.window->showingDrive());
}

TEST_CASE("client window: the drive screen draws through the shared scene: own robot, the other robots, fuel and the game HUD") {
    Fixture f(DemoScenario::Live);
    REQUIRE(f.window->showingDrive());
    DrivePage* drive = f.window->drivePage();
    f.pump(200);
    CHECK(drive->scene().robotVisible());
    CHECK(drive->scene().remoteModel().rowCount() == 3);      // the host's robot and two players
    CHECK(drive->scene().entityModel().totalInstances() > 0);
    CHECK(drive->hud().rowCount() > 0);
    CHECK(drive->scene().fieldLength() > 10.f);
    CHECK(drive->viewport() != nullptr);
}

TEST_CASE("client window: every connection state has a banner, and only a lost link can be reconnected") {
    struct Case {
        DemoScenario scenario;
        Banner banner;
        bool visible;
        bool reconnect;
    };
    const Case cases[] = {
        {DemoScenario::Live, Banner::None, false, false},
        {DemoScenario::Lobby, Banner::Lobby, true, false},
        {DemoScenario::Paused, Banner::Paused, true, false},
        {DemoScenario::HostStalled, Banner::HostStalled, true, false},
        {DemoScenario::Reconnecting, Banner::Reconnecting, true, false},
        {DemoScenario::Lost, Banner::LinkLost, true, true},
        {DemoScenario::Kicked, Banner::Kicked, true, false},
        {DemoScenario::Closed, Banner::ServerClosing, true, false},
    };
    for (const Case& c : cases) {
        CAPTURE(static_cast<int>(c.scenario));
        Fixture f(c.scenario);
        REQUIRE(f.window->showingDrive());
        const ClientBanner* banner = f.window->drivePage()->banner();
        CHECK(f.window->controller().presentation().banner == c.banner);
        CHECK(banner->isHidden() == !c.visible);
        if (c.visible) {
            CHECK(banner->titleLabel()->text() == bannerTitle(c.banner));
            CHECK(banner->reconnectButton()->isHidden() == !c.reconnect);
        }
    }
}

TEST_CASE("client window: the host's reasons are shown, and reconnecting a lost link goes live again") {
    Fixture kicked(DemoScenario::Kicked);
    CHECK(kicked.window->drivePage()->banner()->detailLabel()->text().contains("Removed by the host"));

    Fixture lost(DemoScenario::Lost);
    ClientBanner* banner = lost.window->drivePage()->banner();
    REQUIRE_FALSE(banner->reconnectButton()->isHidden());
    banner->reconnectButton()->click();
    lost.pump(150);
    CHECK(lost.window->controller().presentation().banner == Banner::None);
    CHECK(banner->isHidden());

    Fixture closed(DemoScenario::Closed);
    closed.window->drivePage()->banner()->leaveButton()->click();
    closed.pump(150);
    CHECK_FALSE(closed.window->showingDrive());
}

TEST_CASE("client window: the diagnostics overlay shows ping, loss, prediction error and snapshot age on demand") {
    Fixture f(DemoScenario::Live);
    DrivePage* drive = f.window->drivePage();
    CHECK(drive->diagnostics()->isHidden());
    drive->setDiagnosticsVisible(true);
    f.pump(100);
    CHECK_FALSE(drive->diagnostics()->isHidden());
    const QString text = drive->diagnostics()->textLabel()->text();
    for (const char* word : {"Ping", "Packet loss", "Prediction error", "Snapshot age", "Buffer delay", "Jitter"}) {
        CAPTURE(word);
        CHECK(text.contains(QString::fromLatin1(word)));
    }
    drive->setDiagnosticsVisible(false);
    CHECK(drive->diagnostics()->isHidden());
}

TEST_CASE("diagnostics lines: every figure of the view appears with its unit") {
    mc::MatchClientView view;
    view.connection.state = mc::ConnectionState::Live;
    view.diagnostics.rttMs = 42.5;
    view.diagnostics.rttMinMs = 30.0;
    view.diagnostics.jitterMs = 3.5;
    view.diagnostics.lossPermille = 12.0;
    view.diagnostics.snapshotRateHz = 30.0;
    view.diagnostics.snapshotAgeMs = 17.0;
    view.diagnostics.bufferDelayMs = 80.0;
    view.diagnostics.predictionErrorM = 0.05;
    view.diagnostics.predictionErrorP95M = 0.2;
    view.diagnostics.inputLeadTicks = 9;
    const QString all = diagnosticsLines(view).join('\n');
    CHECK(all.contains("42.5 ms"));
    CHECK(all.contains("30.0 ms"));
    CHECK(all.contains("12.0 ‰"));
    CHECK(all.contains("30 Hz"));
    CHECK(all.contains("5.0 cm"));
    CHECK(all.contains("20.0 cm"));
    CHECK(all.contains("9 ticks"));
    CHECK(all.contains("Live"));
}

TEST_CASE("client window: zone buttons and sliders reach the controller, and reset forgets the move") {
    Fixture f(DemoScenario::Live);
    DrivePage* drive = f.window->drivePage();
    f.pump(150);
    ZonePanel* zones = drive->zonePanel();
    REQUIRE(zones->zoneCount() == 2);
    CHECK_FALSE(zones->isHidden());

    zones->zoneButton(1)->click();
    CHECK((f.window->controller().zoneButtons() & 0b10u) != 0);
    zones->zoneButton(1)->click();
    CHECK((f.window->controller().zoneButtons() & 0b10u) == 0);

    REQUIRE(zones->slider(0) != nullptr);
    const int before = zones->slider(0)->value();
    zones->slider(0)->setValue(before + 100);
    CHECK(f.window->controller().zonePoses().moved(0));
    CHECK(f.window->controller().zonePoses().pose(0).position.x == doctest::Approx(static_cast<float>(before + 100) / 1000.f).epsilon(0.01));
    zones->resetButton()->click();
    CHECK_FALSE(f.window->controller().zonePoses().moved(0));
    CHECK(zones->slider(0)->value() == before);
}

TEST_CASE("client window: the keyboard drives the own robot at the limits of its spec, and letting go stops it") {
    Fixture f(DemoScenario::Live);
    DrivePage* drive = f.window->drivePage();
    f.pump(150);
    const double top = f.window->controller().driveLimits().maxLinearMps;
    REQUIRE(top > 2.0);

    drive->handleKey(Qt::Key_W, true);   // left stick up
    f.pump(200);
    const auto view = f.window->controller().view();
    REQUIRE(view);
    CHECK(std::hypot(view->own.linearVelocity.x, view->own.linearVelocity.y) == doctest::Approx(top).epsilon(0.05));
    drive->handleKey(Qt::Key_W, false);
    f.pump(200);
    CHECK(std::hypot(f.window->controller().view()->own.linearVelocity.x, f.window->controller().view()->own.linearVelocity.y) < 0.1);
}

TEST_CASE("client window: the choices persist across windows and the invite never does") {
    Fixture f(DemoScenario::Join);
    const QString ini = f.dir.filePath(QStringLiteral("client.ini"));
    {
        JoinPage* page = f.window->joinPage();
        page->nameEdit()->setText("Gearheads");
        page->directEdit()->setText("10.0.0.7:5000");
        page->relayCombo()->setCurrentIndex(page->relayCombo()->findData(static_cast<int>(app::RelayChoice::Custom)));
        page->customRelayEdit()->setText("relay.example.org:7000");
        page->setInvite(QString::fromStdString(net::inviteText(net::generateInvite().value())));
        page->robotEdit()->setText(f.robotFile());
        page->joinButton()->click();   // joins the demo session; the form values are stored
        f.pump(100);
    }
    f.window.reset();

    auto again = Fixture::reopen(ini, *f.services);
    JoinPage* page = again->joinPage();
    CHECK(page->nameEdit()->text() == "Gearheads");
    CHECK(page->directEdit()->text() == "10.0.0.7:5000");
    CHECK(page->relayCombo()->currentData().toInt() == static_cast<int>(app::RelayChoice::Custom));
    CHECK(page->customRelayEdit()->text() == "relay.example.org:7000");
    CHECK(page->robotEdit()->text().endsWith("robot.json"));
    CHECK(page->inviteEdit()->text().isEmpty());
    CHECK_FALSE(page->advancedToggle()->isHidden());
    again.reset();
}

TEST_CASE("client text: every banner, reject reason and connection state has words") {
    for (const Banner b : {Banner::Connecting, Banner::Reconnecting, Banner::Lobby, Banner::Paused, Banner::HostStalled, Banner::LinkLost,
                           Banner::HostSilent, Banner::Kicked, Banner::ServerClosing, Banner::Refused, Banner::ProtocolError})
        CHECK_FALSE(bannerTitle(b).isEmpty());
    CHECK(bannerTitle(Banner::None).isEmpty());
    using R = sdk::match::JoinRejectReason;
    for (const R r : {R::BadProtocol, R::WrongPlugin, R::WrongSeason, R::RoomFull, R::BadSpec, R::Kicked, R::Banned, R::HostClosing,
                      R::BadInvite, R::BadResumeToken})
        CHECK_FALSE(rejectReasonText(r).isEmpty());
    for (const mc::ConnectionState s : {mc::ConnectionState::Connecting, mc::ConnectionState::Lobby, mc::ConnectionState::Live,
                                        mc::ConnectionState::Paused, mc::ConnectionState::HostStalled, mc::ConnectionState::Disconnected})
        CHECK_FALSE(stateWord(s).isEmpty());

    Presentation refused;
    refused.banner = Banner::Refused;
    refused.rejectReason = sdk::match::JoinRejectReason::RoomFull;
    refused.detail = "6 of 6";
    const QString text = bannerDetail(refused);
    CHECK(text.contains(rejectReasonText(sdk::match::JoinRejectReason::RoomFull)));
    CHECK(text.contains("6 of 6"));

    Presentation again;
    again.banner = Banner::Reconnecting;
    again.reconnectAttempt = 3;
    CHECK(bannerDetail(again).contains("3"));
}

TEST_CASE("client text: join form errors have a message for every key") {
    for (const char* key : {"invite.empty", "invite.format", "direct.address", "relay.unconfigured", "relay.address", "name.empty",
                            "name.too_long", "robot.missing", "robot.invalid"}) {
        CAPTURE(key);
        CHECK_FALSE(fieldErrorText({JoinField::Invite, key, ""}).isEmpty());
    }
    CHECK(fieldErrorText({JoinField::DirectAddress, "direct.address", "The host's address is not valid: malformed host 'x y'"})
              .contains("malformed host 'x y'"));
}
