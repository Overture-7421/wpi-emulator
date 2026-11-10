#include "shell/host_panel_model.h"

#include <QClipboard>
#include <QGuiApplication>
#include <QLocale>

#include <algorithm>

#include "ui/qt_text.h"
#include "ui/scene_support.h"

namespace wpiemu::app {

namespace {

using ui::qs;

QString dash() { return QString(QChar(0x2014)); }

}  // namespace

HostPanelModel::HostPanelModel(HostingService& service, RelayPresets presets, ClipboardSetter clipboard, QObject* parent)
    : QObject(parent), service_(service), presets_(std::move(presets)), clipboard_(std::move(clipboard)) {
    if (!clipboard_) clipboard_ = [](const QString& text) { QGuiApplication::clipboard()->setText(text); };
    loadDraft();
    connect(&service_, &HostingService::optionsChanged, this, [this] {
        if (!applying_ && service_.options() != seen_) loadDraft();   // changed elsewhere; session start/end keep the draft
        emit changed();
    });
    connect(&service_, &HostingService::statusChanged, this, &HostPanelModel::changed);
}

void HostPanelModel::loadDraft() {
    const host::HostingOptions& o = service_.options();
    seen_ = o;
    mode_ = o.mode;
    port_ = o.port;
    maxPlayers_ = static_cast<int>(o.maxPlayers);
    bindHost_ = qs(o.bindHost);
    choice_ = relayChoiceOf(presets_, o.relayEndpoint);
    if (choice_ == RelayChoice::Custom) customRelay_ = qs(o.relayEndpoint);
    problem_.clear();
}

QString HostPanelModel::relayEndpoint() const {
    return choice_ == RelayChoice::Custom ? customRelay_.trimmed() : qs(presets_.endpointOf(choice_));
}

void HostPanelModel::apply() {
    host::HostingOptions o = service_.options();   // keeps what the panel does not edit (the phase key)
    o.mode = mode_;
    o.port = static_cast<std::uint16_t>(std::clamp(port_, 0, 65535));
    o.maxPlayers = static_cast<std::size_t>(std::max(maxPlayers_, 0));
    o.bindHost = bindHost_.trimmed().toStdString();
    o.relayEndpoint = relayEndpoint().toStdString();

    problem_.clear();
    if (o.mode == host::HostingMode::Relay && choice_ != RelayChoice::Custom && !presets_.configured) {
        problem_ = tr("This build does not know the relay's address yet. Choose Custom under Advanced and enter host:port, or set the "
                      "WPIEMU_RELAY_DNS_SUFFIX environment variable.");
    } else if (o.mode != host::HostingMode::Off) {
        if (const sdk::Status valid = host::validateHosting(o); !valid) problem_ = qs(valid.error().message);
    }
    if (problem_.isEmpty()) {
        applying_ = true;
        service_.setOptions(o);
        applying_ = false;
        seen_ = service_.options();
    }
    emit changed();
}

void HostPanelModel::setMode(host::HostingMode mode) {
    mode_ = mode;
    apply();
}

void HostPanelModel::setPort(int port) {
    port_ = std::clamp(port, 0, 65535);
    apply();
}

void HostPanelModel::setMaxPlayers(int players) {
    maxPlayers_ = std::clamp(players, static_cast<int>(host::kMinHostingPlayers), static_cast<int>(sdk::match::kMaxRobotsPerMatch));
    apply();
}

void HostPanelModel::setRelayChoice(RelayChoice choice) {
    choice_ = choice;
    apply();
}

void HostPanelModel::setCustomRelay(const QString& endpoint) {
    customRelay_ = endpoint;
    apply();
}

void HostPanelModel::setBindHost(const QString& address) {
    bindHost_ = address;
    apply();
}

HostPhaseBanner HostPanelModel::phase() const {
    const host::HostingStatus& s = *service_.status();
    HostPhaseBanner b;
    b.visible = s.hosting();
    const QString detail = qs(s.detail);
    switch (s.phase) {
        case host::HostingPhase::Off:
            break;
        case host::HostingPhase::Starting:
            b.kind = BannerKind::Info;
            b.title = tr("Starting");
            b.detail = detail.isEmpty() ? tr("Opening the connection point.") : detail;
            break;
        case host::HostingPhase::Lobby:
            b.kind = BannerKind::Info;
            b.title = tr("Lobby");
            b.detail = detail.isEmpty() ? tr("Players can join now. The match runs once you start the simulation.") : detail;
            break;
        case host::HostingPhase::Live:
            b.kind = BannerKind::Success;
            b.title = tr("Live");
            b.detail = detail.isEmpty() ? tr("%1 of %2 players are in the match.").arg(s.roster.size()).arg(s.maxPlayers) : detail;
            break;
        case host::HostingPhase::Paused:
            b.kind = BannerKind::Warning;
            b.title = tr("Paused");
            b.detail = detail.isEmpty() ? tr("You paused the simulation, so the match is paused for everyone.") : detail;
            break;
        case host::HostingPhase::HostStalled:
            b.kind = BannerKind::Warning;
            b.title = tr("Host stalled");
            b.detail = detail.isEmpty() ? tr("Your robot program stopped answering. The match waits for it.") : detail;
            break;
        case host::HostingPhase::Error:
            b.kind = BannerKind::Error;
            b.title = tr("Error");
            b.detail = detail.isEmpty() ? tr("Hosting stopped working.") : detail;
            break;
    }
    return b;
}

QString HostPanelModel::launchHint() const {
    if (service_.restartRequired())
        return tr("These settings apply the next time the robot program is launched: stop it and run again.");
    if (service_.options().mode != host::HostingMode::Off && !service_.status()->hosting())
        return tr("Hosting starts the next time the robot program is launched.");
    return {};
}

QString HostPanelModel::invite() const { return qs(service_.status()->invite); }

bool HostPanelModel::canCopyInvite() const { return !service_.status()->invite.empty(); }

QString HostPanelModel::directEndpoint() const {
    const host::HostingStatus& s = *service_.status();
    return s.mode == host::HostingMode::Direct ? qs(s.endpoint) : QString();
}

QList<HostRosterRow> HostPanelModel::roster() const {
    const host::HostingStatus& s = *service_.status();
    QList<HostRosterRow> rows;
    for (const host::HostingParticipant& p : s.roster) {
        HostRosterRow r;
        r.id = p.id;
        r.name = qs(p.name);
        r.station = ui::stationText(p.station);
        r.stationColor = ui::allianceColor(p.station);
        r.isHost = p.isHost;
        r.connected = p.connected;
        r.canKick = !p.isHost && s.hosting();
        if (p.isHost) {
            r.ping = dash();
            r.loss = dash();
        } else if (!p.connected) {
            r.ping = dash();
            r.loss = tr("Disconnected");
        } else {
            r.ping = tr("%1 ms").arg(p.pingMs);
            r.loss = tr("%1%").arg(QLocale().toString(static_cast<double>(p.lossPermille) / 10.0, 'f', 1));
        }
        rows.push_back(std::move(r));
    }
    return rows;
}

bool HostPanelModel::copyInvite() {
    if (!canCopyInvite()) return false;
    clipboard_(invite());
    return true;
}

bool HostPanelModel::kick(sdk::match::ParticipantId participant) {
    const host::HostingStatus& s = *service_.status();
    if (!s.hosting() || participant == sdk::match::kHostParticipantId) return false;
    const bool listed = std::any_of(s.roster.begin(), s.roster.end(), [&](const host::HostingParticipant& p) { return p.id == participant && !p.isHost; });
    if (!listed) return false;
    service_.kick(participant);
    return true;
}

}  // namespace wpiemu::app
