// The practice-match panel: its model (settings draft, status presentation, copy invite, kick, launch hint), the widgets on
// top of it, the relay presets and the dock in the main window. Offscreen: no network and no robot.
#include <doctest/doctest.h>

#include <QApplication>
#include <QAction>
#include <QComboBox>
#include <QDockWidget>
#include <QLabel>
#include <QLocale>
#include <QMenu>
#include <QMenuBar>
#include <QPushButton>
#include <QSignalSpy>
#include <QTest>
#include <QToolButton>
#include <QTranslator>

#include "common/hosting_service.h"
#include "ui/relay_endpoints.h"
#include "contract/runtime_host.h"
#include "gui_support.h"
#include "shell/host_panel.h"
#include "shell/host_panel_model.h"
#include "shell/main_window.h"
#include "ui/qt_text.h"
#include "ui/scene_support.h"
#include "wpiemu/sdk/testing/demo_game.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace host = wpiemu::app::host;
namespace sdk = wpiemu::sdk;

namespace {

constexpr const char* kSuffix = "-a1b2";

RelayPresets configuredPresets() { return relayPresetsFor(kSuffix); }

host::HostingParticipant remote(sdk::match::ParticipantId id, const char* name, sdk::proto::AllianceStation station,
                                std::uint32_t pingMs = 20, std::uint32_t lossPermille = 5, bool connected = true) {
    host::HostingParticipant p;
    p.id = id;
    p.name = name;
    p.station = station;
    p.pingMs = pingMs;
    p.lossPermille = lossPermille;
    p.connected = connected;
    return p;
}

std::shared_ptr<const host::HostingStatus> status(host::HostingPhase phase, std::vector<host::HostingParticipant> remotes = {},
                                                  const char* detail = "", host::HostingMode mode = host::HostingMode::Relay) {
    host::HostingStatus s;
    s.mode = mode;
    s.phase = phase;
    s.detail = detail;
    s.invite = "ABCDE-FGHJK-MNPQR-STVWX";
    s.maxPlayers = 6;
    s.endpoint = mode == host::HostingMode::Direct ? "enet://192.168.1.20:47300" : "relay://relay.example:7777/ROOM";
    host::HostingParticipant self;
    self.isHost = true;
    self.name = "Me";
    self.station = sdk::proto::AllianceStation::Red1;
    s.roster.push_back(self);
    for (auto& r : remotes) s.roster.push_back(std::move(r));
    return std::make_shared<const host::HostingStatus>(std::move(s));
}

struct ModelFixture {
    MemoryKeyValueStore store;
    HostingService service{store};
    QString clipboard;
    HostPanelModel model;

    explicit ModelFixture(RelayPresets presets = configuredPresets())
        : model(service, std::move(presets), [this](const QString& text) { clipboard = text; }) {}
};

// Marks every string of the panel's contexts, so a hard-coded user string stands out.
class MarkingTranslator : public QTranslator {
public:
    MarkingTranslator() { QCoreApplication::installTranslator(this); }
    ~MarkingTranslator() override { QCoreApplication::removeTranslator(this); }
    bool isEmpty() const override { return false; }
    QString translate(const char* context, const char* source, const char*, int) const override {
        const std::string_view c(context);
        if (c.find("HostPanel") != std::string_view::npos || c == "Station")
            return QStringLiteral("«") + QString::fromUtf8(source) + QStringLiteral("»");
        return {};
    }
};

bool marked(const QString& text) { return text.contains(QChar(0x00AB)); }

}  // namespace

// ---- relay endpoints -------------------------------------------------------------------------------------------------------

TEST_CASE("relay endpoints: production and development follow the Bicep naming and need a valid suffix") {
    // A suffix with a leading hyphen and one without both build the label the same way: <prefix>[-dev]<suffix>.
    CHECK(relayEndpointFor(RelayChoice::Production, "-a1b2") == "wpiemu-relay-a1b2.southcentralus.azurecontainer.io:7777");
    CHECK(relayEndpointFor(RelayChoice::Development, "-a1b2") == "wpiemu-relay-dev-a1b2.southcentralus.azurecontainer.io:7777");
    CHECK(relayEndpointFor(RelayChoice::Production, "a1b2") == "wpiemu-relaya1b2.southcentralus.azurecontainer.io:7777");
    // The suffix of the deployed relay (bootstrap.ps1 picks six hex characters): these are the names Azure gave the relays.
    CHECK(relayEndpointFor(RelayChoice::Development, "47c926") == "wpiemu-relay-dev47c926.southcentralus.azurecontainer.io:7777");
    CHECK(relayEndpointFor(RelayChoice::Production, "47c926") == "wpiemu-relay47c926.southcentralus.azurecontainer.io:7777");
    const std::string longest(40, 'a'), tooLong(41, 'a');
    for (const char* good : {"a", "47c926", "-a1b2", "a-b", longest.c_str()}) CHECK(isValidRelayDnsSuffix(good));
    for (const char* bad : {"", "-", "a-", "-a1b2-", "A1B2", "-A1B2", "-a_b", "-a b", "-a1b2.evil.example", "47c926.evil", tooLong.c_str()}) {
        CHECK_FALSE(isValidRelayDnsSuffix(bad));
        CHECK(relayEndpointFor(RelayChoice::Production, bad).empty());
    }
    CHECK(relayEndpointFor(RelayChoice::Custom, "-a1b2").empty());

    const RelayPresets none = relayPresetsFor("");
    CHECK_FALSE(none.configured);
    CHECK(none.production.empty());
    const RelayPresets presets = configuredPresets();
    CHECK(presets.configured);
    CHECK(presets.endpointOf(RelayChoice::Production) == presets.production);
    CHECK(presets.endpointOf(RelayChoice::Development) == presets.development);
}

TEST_CASE("relay endpoints: the suffix comes from the environment, then the stored preference") {
    MemoryKeyValueStore store;
    qunsetenv(kRelayDnsSuffixEnvironment);
    const std::string built = resolveRelayDnsSuffix(store);   // whatever this build bakes in (empty by default)
    store.setValue(QString::fromLatin1(kRelayDnsSuffixSetting), QStringLiteral(" -stored "));
    CHECK(resolveRelayDnsSuffix(store) == "-stored");
    qputenv(kRelayDnsSuffixEnvironment, "-fromenv");
    CHECK(resolveRelayDnsSuffix(store) == "-fromenv");
    CHECK(relayPresets(store).production == "wpiemu-relay-fromenv.southcentralus.azurecontainer.io:7777");
    qunsetenv(kRelayDnsSuffixEnvironment);
    store.remove(QString::fromLatin1(kRelayDnsSuffixSetting));
    CHECK(resolveRelayDnsSuffix(store) == built);
}

TEST_CASE("relay endpoints: a stored endpoint maps back to its choice") {
    const RelayPresets presets = configuredPresets();
    CHECK(relayChoiceOf(presets, "") == RelayChoice::Production);   // the default
    CHECK(relayChoiceOf(presets, presets.production) == RelayChoice::Production);
    CHECK(relayChoiceOf(presets, presets.development) == RelayChoice::Development);
    CHECK(relayChoiceOf(presets, "relay.example.org:9000") == RelayChoice::Custom);
    CHECK(relayChoiceOf(relayPresetsFor(""), "wpiemu-relay-a1b2.southcentralus.azurecontainer.io:7777") == RelayChoice::Custom);
}

// ---- the model -------------------------------------------------------------------------------------------------------------

TEST_CASE("host panel model: starts from the saved options and saves valid edits") {
    ModelFixture f;
    CHECK(f.model.mode() == host::HostingMode::Off);
    CHECK(f.model.relayChoice() == RelayChoice::Production);
    CHECK(f.model.maxPlayers() == static_cast<int>(sdk::match::kMaxRobotsPerMatch));
    QSignalSpy changed(&f.model, &HostPanelModel::changed);

    f.model.setMode(host::HostingMode::Direct);
    f.model.setPort(4242);
    f.model.setMaxPlayers(4);
    CHECK(f.model.problem().isEmpty());
    CHECK(changed.count() == 3);
    CHECK(f.service.options().mode == host::HostingMode::Direct);
    CHECK(f.service.options().port == 4242);
    CHECK(f.service.options().maxPlayers == 4);
    CHECK(f.store.value(QStringLiteral("hosting/port")).toInt() == 4242);

    f.model.setMaxPlayers(99);   // clamped like the service does
    CHECK(f.model.maxPlayers() == static_cast<int>(sdk::match::kMaxRobotsPerMatch));

    f.model.setMode(host::HostingMode::Relay);   // the default relay is the production preset
    CHECK(f.service.options().relayEndpoint == configuredPresets().production);
    f.model.setRelayChoice(RelayChoice::Development);
    CHECK(f.service.options().relayEndpoint == configuredPresets().development);
    f.model.setRelayChoice(RelayChoice::Custom);
    f.model.setCustomRelay(QStringLiteral("relay.example.org:9000"));
    CHECK(f.service.options().relayEndpoint == "relay.example.org:9000");
    CHECK(f.model.problem().isEmpty());

    HostingService again(f.store);
    HostPanelModel second(again, configuredPresets());
    CHECK(second.mode() == host::HostingMode::Relay);
    CHECK(second.relayChoice() == RelayChoice::Custom);
    CHECK(second.customRelay() == QStringLiteral("relay.example.org:9000"));
}

TEST_CASE("host panel model: an edit that would stop hosting from starting is explained and not saved") {
    ModelFixture f;
    f.model.setMode(host::HostingMode::Relay);
    f.model.setRelayChoice(RelayChoice::Custom);
    f.model.setCustomRelay(QStringLiteral("not a relay"));
    CHECK_FALSE(f.model.problem().isEmpty());
    CHECK(f.service.options().relayEndpoint == configuredPresets().production);   // the saved options still launch the robot
    CHECK(f.model.relayChoice() == RelayChoice::Custom);   // the controls keep what the user typed

    f.model.setCustomRelay(QStringLiteral("relay.example.org:9000"));
    CHECK(f.model.problem().isEmpty());
    CHECK(f.service.options().mode == host::HostingMode::Relay);

    f.model.setBindHost(QString());   // Direct needs an address to listen on
    f.model.setMode(host::HostingMode::Direct);
    CHECK_FALSE(f.model.problem().isEmpty());
    f.model.setMode(host::HostingMode::Off);   // turning hosting off always works
    CHECK(f.model.problem().isEmpty());
    CHECK(f.service.options().mode == host::HostingMode::Off);
}

TEST_CASE("host panel model: the production relay of a build without a configured suffix cannot be chosen") {
    ModelFixture f(relayPresetsFor(""));
    f.model.setMode(host::HostingMode::Relay);
    CHECK(f.model.problem().contains(QStringLiteral("WPIEMU_RELAY_DNS_SUFFIX")));
    CHECK(f.model.relayEndpoint().isEmpty());
    CHECK(f.service.options().mode == host::HostingMode::Off);
    f.model.setRelayChoice(RelayChoice::Custom);
    f.model.setCustomRelay(QStringLiteral("relay.example.org:9000"));
    CHECK(f.model.problem().isEmpty());
    CHECK(f.service.options().mode == host::HostingMode::Relay);
}

TEST_CASE("host panel model: the panel keeps the options it does not edit, such as the game's phase key") {
    ModelFixture f;
    f.service.setPhaseHudKey("game.phase");
    f.model.setMode(host::HostingMode::Direct);
    CHECK(f.service.options().phaseHudKey == "game.phase");
    f.model.setBindHost(QStringLiteral(" 127.0.0.1 "));
    CHECK(f.service.options().bindHost == "127.0.0.1");
}

TEST_CASE("host panel model: the phase banner names Lobby, Live, Paused, Host stalled and Error") {
    ModelFixture f;
    CHECK_FALSE(f.model.phase().visible);   // not hosting

    struct Case { host::HostingPhase phase; BannerKind kind; const char* title; };
    const Case cases[] = {
        {host::HostingPhase::Starting, BannerKind::Info, "Starting"},
        {host::HostingPhase::Lobby, BannerKind::Info, "Lobby"},
        {host::HostingPhase::Live, BannerKind::Success, "Live"},
        {host::HostingPhase::Paused, BannerKind::Warning, "Paused"},
        {host::HostingPhase::HostStalled, BannerKind::Warning, "Host stalled"},
        {host::HostingPhase::Error, BannerKind::Error, "Error"},
    };
    for (const Case& c : cases) {
        f.service.publish(status(c.phase));
        const HostPhaseBanner b = f.model.phase();
        CAPTURE(c.title);
        CHECK(b.visible);
        CHECK(b.kind == c.kind);
        CHECK(b.title == QString::fromLatin1(c.title));
        CHECK_FALSE(b.detail.isEmpty());   // every phase says what it means
    }
    f.service.publish(status(host::HostingPhase::Error, {}, "The relay did not answer."));
    CHECK(f.model.phase().detail == QStringLiteral("The relay did not answer."));   // the host's own detail wins
    f.service.publish(nullptr);
    CHECK_FALSE(f.model.phase().visible);
}

TEST_CASE("host panel model: the launch hint says when settings take effect") {
    ModelFixture f;
    CHECK(f.model.launchHint().isEmpty());   // hosting is off

    f.model.setMode(host::HostingMode::Direct);
    CHECK_FALSE(f.model.launchHint().isEmpty());   // saved, starts at the next launch
    const QString nextLaunch = f.model.launchHint();

    f.service.sessionStarted(f.service.options());   // the robot launched with these options
    f.service.publish(status(host::HostingPhase::Lobby, {}, "", host::HostingMode::Direct));
    CHECK(f.model.launchHint().isEmpty());   // running with exactly them

    f.model.setPort(5555);
    CHECK(f.service.restartRequired());
    CHECK_FALSE(f.model.launchHint().isEmpty());
    CHECK(f.model.launchHint() != nextLaunch);   // a different message: relaunch to change a running session

    f.model.setPort(f.service.appliedOptions()->port);
    CHECK(f.model.launchHint().isEmpty());
    f.service.sessionEnded();
    CHECK(f.model.launchHint() == nextLaunch);
}

TEST_CASE("host panel model: the invite is shown and copied only while hosting") {
    ModelFixture f;
    CHECK_FALSE(f.model.canCopyInvite());
    CHECK_FALSE(f.model.copyInvite());
    CHECK(f.clipboard.isEmpty());

    f.service.publish(status(host::HostingPhase::Lobby));
    CHECK(f.model.invite() == QStringLiteral("ABCDE-FGHJK-MNPQR-STVWX"));
    CHECK(f.model.canCopyInvite());
    CHECK(f.model.copyInvite());
    CHECK(f.clipboard == QStringLiteral("ABCDE-FGHJK-MNPQR-STVWX"));
    CHECK(f.store.value(QStringLiteral("hosting/invite")).isNull());   // never persisted

    f.service.publish(nullptr);
    CHECK(f.model.invite().isEmpty());
}

TEST_CASE("host panel model: the roster lists the host first with alliance, ping and loss") {
    ModelFixture f;
    f.service.publish(status(host::HostingPhase::Live,
                             {remote(1, "Owls", sdk::proto::AllianceStation::Blue2, 23, 5),
                              remote(2, "Gone", sdk::proto::AllianceStation::Red3, 0, 1000, false)}));
    const QList<HostRosterRow> rows = f.model.roster();
    REQUIRE(rows.size() == 3);
    CHECK(rows[0].isHost);
    CHECK_FALSE(rows[0].canKick);
    CHECK(rows[0].station == ui::stationText(sdk::proto::AllianceStation::Red1));
    CHECK(rows[0].stationColor == ui::allianceColor(sdk::Alliance::Red));

    CHECK(rows[1].name == QStringLiteral("Owls"));
    CHECK(rows[1].station == ui::stationText(sdk::proto::AllianceStation::Blue2));
    CHECK(rows[1].stationColor == ui::allianceColor(sdk::Alliance::Blue));
    CHECK(rows[1].ping == QStringLiteral("23 ms"));
    CHECK(rows[1].loss == QStringLiteral("%1%").arg(QLocale().toString(0.5, 'f', 1)));
    CHECK(rows[1].canKick);

    CHECK_FALSE(rows[2].connected);
    CHECK(rows[2].loss == QStringLiteral("Disconnected"));   // a dropped player shows no figures
    CHECK(rows[2].canKick);                                  // but can still be removed
}

TEST_CASE("host panel model: kick goes to the backend for a listed remote player only") {
    ModelFixture f;
    QSignalSpy kicks(&f.service, &HostingService::kickRequested);
    CHECK_FALSE(f.model.kick(1));   // not hosting
    f.service.publish(status(host::HostingPhase::Live, {remote(1, "Owls", sdk::proto::AllianceStation::Blue2)}));
    CHECK_FALSE(f.model.kick(sdk::match::kHostParticipantId));
    CHECK_FALSE(f.model.kick(9));   // nobody by that id
    CHECK(kicks.count() == 0);
    CHECK(f.model.kick(1));
    REQUIRE(kicks.count() == 1);
    CHECK(kicks.at(0).at(0).toUInt() == 1);
}

// ---- the widgets -------------------------------------------------------------------------------------------------------------

TEST_CASE("host panel: the controls edit the settings and show only what the mode needs") {
    GuiEnvironment env;
    HostingService& service = env.services->hosting();
    HostPanelModel model(service, configuredPresets(), [](const QString&) {});
    HostPanel panel(model, env.ui);
    panel.show();

    CHECK_FALSE(panel.portSpin()->isVisible());
    CHECK_FALSE(panel.advancedToggle()->isVisible());
    CHECK_FALSE(panel.inviteEdit()->isVisible());

    panel.modeCombo()->setCurrentIndex(panel.modeCombo()->findData(static_cast<int>(host::HostingMode::Direct)));
    emit panel.modeCombo()->activated(panel.modeCombo()->currentIndex());
    CHECK(service.options().mode == host::HostingMode::Direct);
    CHECK(panel.portSpin()->isVisible());
    CHECK(panel.advancedToggle()->isVisible());
    CHECK(panel.hintBanner()->isVisible());   // saved, starts at the next launch

    panel.portSpin()->setValue(4321);
    emit panel.portSpin()->editingFinished();
    CHECK(service.options().port == 4321);

    panel.modeCombo()->setCurrentIndex(panel.modeCombo()->findData(static_cast<int>(host::HostingMode::Relay)));
    emit panel.modeCombo()->activated(panel.modeCombo()->currentIndex());
    CHECK_FALSE(panel.portSpin()->isVisible());
    CHECK_FALSE(panel.customRelayEdit()->isVisible());
    panel.advancedToggle()->setChecked(true);
    CHECK(panel.relayCombo()->isVisible());
    panel.relayCombo()->setCurrentIndex(panel.relayCombo()->findData(static_cast<int>(RelayChoice::Custom)));
    emit panel.relayCombo()->activated(panel.relayCombo()->currentIndex());
    CHECK(panel.customRelayEdit()->isVisible());
    panel.customRelayEdit()->setText(QStringLiteral("broken address"));
    emit panel.customRelayEdit()->editingFinished();
    CHECK(panel.problemBanner()->isVisible());
    panel.customRelayEdit()->setText(QStringLiteral("relay.example.org:9000"));
    emit panel.customRelayEdit()->editingFinished();
    CHECK_FALSE(panel.problemBanner()->isVisible());
    CHECK(service.options().relayEndpoint == "relay.example.org:9000");
}

TEST_CASE("host panel: lobby shows the invite with a working Copy button and the roster with Kick buttons") {
    GuiEnvironment env;
    HostingService& service = env.services->hosting();
    QString clipboard;
    HostPanelModel model(service, configuredPresets(), [&](const QString& text) { clipboard = text; });
    HostPanel panel(model, env.ui);
    panel.show();

    service.publish(status(host::HostingPhase::Lobby, {remote(1, "Owls", sdk::proto::AllianceStation::Blue1)}));
    CHECK(panel.inviteEdit()->isVisible());
    CHECK(panel.inviteEdit()->text() == QStringLiteral("ABCDE-FGHJK-MNPQR-STVWX"));
    CHECK(panel.phaseBanner()->isVisible());
    CHECK(panel.phaseBanner()->kind() == BannerKind::Info);
    CHECK(panel.phaseBanner()->text().contains(QStringLiteral("Lobby")));

    panel.copyButton()->click();
    CHECK(clipboard == QStringLiteral("ABCDE-FGHJK-MNPQR-STVWX"));

    REQUIRE(panel.rosterTable()->rowCount() == 2);
    CHECK(panel.kickButton(0) == nullptr);   // the host cannot be kicked
    REQUIRE(panel.kickButton(1) != nullptr);
    QSignalSpy kicks(&service, &HostingService::kickRequested);
    panel.kickButton(1)->click();
    REQUIRE(kicks.count() == 1);
    CHECK(kicks.at(0).at(0).toUInt() == 1);

    service.publish(status(host::HostingPhase::HostStalled, {remote(1, "Owls", sdk::proto::AllianceStation::Blue1)}));
    CHECK(panel.phaseBanner()->kind() == BannerKind::Warning);
    service.publish(status(host::HostingPhase::Error, {}, "The relay did not answer."));
    CHECK(panel.phaseBanner()->kind() == BannerKind::Error);
    CHECK(panel.phaseBanner()->text().contains(QStringLiteral("The relay did not answer.")));
    service.publish(nullptr);
    CHECK_FALSE(panel.phaseBanner()->isVisible());
    CHECK_FALSE(panel.inviteEdit()->isVisible());
    CHECK(panel.rosterTable()->rowCount() == 0);
}

TEST_CASE("host panel: every user-visible string goes through a translation context") {
    GuiEnvironment env;
    const MarkingTranslator translator;
    HostingService& service = env.services->hosting();
    host::HostingOptions o;
    o.mode = host::HostingMode::Direct;
    service.setOptions(o);
    HostPanelModel model(service, configuredPresets(), [](const QString&) {});
    HostPanel panel(model, env.ui);
    panel.show();
    panel.advancedToggle()->setChecked(true);
    service.sessionStarted(service.options());
    o.port = 1234;
    service.setOptions(o);   // a launch hint
    model.setBindHost(QString());   // a problem banner
    service.publish(status(host::HostingPhase::Lobby, {remote(1, "Owls", sdk::proto::AllianceStation::Blue1)}, "", host::HostingMode::Direct));

    for (QLabel* label : panel.findChildren<QLabel*>()) {
        if (label->text().isEmpty() || !label->isVisibleTo(&panel)) continue;
        CAPTURE(label->text().toStdString());
        CHECK(marked(label->text()));
    }
    for (QAbstractButton* button : panel.findChildren<QAbstractButton*>()) {
        if (button->text().isEmpty()) continue;
        CAPTURE(button->text().toStdString());
        CHECK(marked(button->text()));
    }
    for (QComboBox* combo : {panel.modeCombo(), panel.relayCombo()})
        for (int i = 0; i < combo->count(); ++i) {
            CAPTURE(combo->itemText(i).toStdString());
            CHECK(marked(combo->itemText(i)));
        }
    for (int c = 0; c < panel.rosterTable()->columnCount(); ++c) {
        const QTableWidgetItem* header = panel.rosterTable()->horizontalHeaderItem(c);
        if (header != nullptr && !header->text().isEmpty()) CHECK(marked(header->text()));
    }
    // the phase, the launch hint and the station names come from the model and the shared text helper
    CHECK(marked(model.phase().title));
    CHECK(marked(model.launchHint()));
    CHECK(marked(ui::stationText(sdk::proto::AllianceStation::Blue1)));
    CHECK(marked(panel.portSpin()->specialValueText()));
    CHECK(marked(panel.customRelayEdit()->placeholderText()));
}

TEST_CASE("the phase key of a game comes from its scoreboard") {
    std::vector<sdk::HudWidgetDescriptor> hud;
    CHECK(phaseHudKeyOf(hud).empty());
    sdk::HudWidgetDescriptor counter;
    counter.kind = sdk::HudWidgetKind::Counter;
    counter.valueKey = "not.the.phase";
    hud.push_back(counter);
    CHECK(phaseHudKeyOf(hud).empty());
    sdk::HudWidgetDescriptor board;
    board.kind = sdk::HudWidgetKind::Scoreboard;
    board.valueKey = "game.match_phase";
    hud.push_back(board);
    CHECK(phaseHudKeyOf(hud) == "game.match_phase");
    CHECK(phaseHudKeyOf(wpiemu::sdk::testing::demo_game::hud()) == wpiemu::sdk::testing::demo_game::kPhase);
}

TEST_CASE("hosting service: the phase key is stored with the options and does not ask for a relaunch when it is the same") {
    MemoryKeyValueStore store;
    HostingService service(store);
    host::HostingOptions o;
    o.mode = host::HostingMode::Direct;
    service.setOptions(o);
    service.setPhaseHudKey("game.phase");
    service.sessionStarted(service.options());
    service.setPhaseHudKey("game.phase");   // the same game again
    CHECK_FALSE(service.restartRequired());
    HostingService reloaded(store);
    CHECK(reloaded.options().phaseHudKey == "game.phase");
}

// ---- the dock in the main window -------------------------------------------------------------------------------------------------

TEST_CASE("main window: the practice match panel is a dock that starts hidden and edits the saved options") {
    GuiEnvironment env;
    ShellController controller(*env.services);
    MainWindow window(*env.services, controller, env.ui, createShellPages(*env.services, env.ui, &controller));
    window.show();
    REQUIRE(window.hostDock() != nullptr);
    REQUIRE(window.hostPanel() != nullptr);
    CHECK_FALSE(window.hostDock()->isVisible());
    CHECK(window.hostDock()->windowTitle() == QStringLiteral("Practice match"));

    window.hostDock()->toggleViewAction()->trigger();
    CHECK(window.hostDock()->isVisible());
    window.hostPanel()->modeCombo()->setCurrentIndex(window.hostPanel()->modeCombo()->findData(static_cast<int>(host::HostingMode::Direct)));
    emit window.hostPanel()->modeCombo()->activated(window.hostPanel()->modeCombo()->currentIndex());
    CHECK(env.services->hosting().options().mode == host::HostingMode::Direct);

    // the View menu offers the panel, except for the scripted demo host, which cannot host a match
    auto viewMenuOffers = [](MainWindow& w) {
        for (QAction* menu : w.menuBar()->actions())
            if (menu->menu() != nullptr && menu->menu()->actions().contains(w.hostDock()->toggleViewAction())) return true;
        return false;
    };
    CHECK_FALSE(env.services->demo());
    CHECK(viewMenuOffers(window));

    GuiEnvironment demoEnv(true, false, [](AppServicesOptions& o) { o.demo = true; });
    ShellController demoController(*demoEnv.services);
    MainWindow demoWindow(*demoEnv.services, demoController, demoEnv.ui, createShellPages(*demoEnv.services, demoEnv.ui, &demoController));
    CHECK(demoEnv.services->demo());
    CHECK_FALSE(viewMenuOffers(demoWindow));
}
