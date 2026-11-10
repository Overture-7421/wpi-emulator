#pragma once

// The practice-match panel's logic without widgets: the settings draft the controls edit, what the status snapshot says
// (phase banner, launch hint, invite, roster rows) and the two actions (copy the invite, kick a player). Reads
// HostingService's immutable status; settings go back through HostingService::setOptions, which persists them. An edit
// that would make hosting fail to start is NOT saved: the draft keeps it and `problem()` says why, so a bad port or relay
// address can never stop the robot from launching.

#include <QColor>
#include <QList>
#include <QObject>
#include <QString>

#include <functional>

#include "common/hosting_service.h"
#include "common/relay_endpoints.h"
#include "common/ui_helpers.h"

namespace wpiemu::app {

struct HostRosterRow {
    sdk::match::ParticipantId id = sdk::match::kHostParticipantId;
    QString name;
    QString station;        // "Red 1"
    QColor stationColor;    // the alliance's colour
    QString ping;           // "23 ms"; a dash for the host and a dropped player
    QString loss;           // "0.5%"; "Disconnected" for a dropped player; a dash for the host
    bool isHost = false;
    bool connected = true;
    bool canKick = false;
};

struct HostPhaseBanner {
    bool visible = false;
    BannerKind kind = BannerKind::Info;
    QString title;    // Starting, Lobby, Live, Paused, Host stalled, Error
    QString detail;
};

class HostPanelModel : public QObject {
    Q_OBJECT
public:
    using ClipboardSetter = std::function<void(const QString&)>;

    // `clipboard` defaults to the system clipboard.
    HostPanelModel(HostingService& service, RelayPresets presets, ClipboardSetter clipboard = {}, QObject* parent = nullptr);

    // ---- settings draft (what the controls show) ----
    host::HostingMode mode() const { return mode_; }
    int port() const { return port_; }
    int maxPlayers() const { return maxPlayers_; }
    RelayChoice relayChoice() const { return choice_; }
    QString customRelay() const { return customRelay_; }
    QString bindHost() const { return bindHost_; }
    const RelayPresets& presets() const { return presets_; }
    // The relay the draft stands for, "host:port"; empty while the presets are not configured and the choice is not Custom.
    QString relayEndpoint() const;
    // Why the draft is not saved; empty when it is (or hosting is off).
    QString problem() const { return problem_; }

    void setMode(host::HostingMode mode);
    void setPort(int port);
    void setMaxPlayers(int players);
    void setRelayChoice(RelayChoice choice);
    void setCustomRelay(const QString& endpoint);
    void setBindHost(const QString& address);

    // ---- what the running session says ----
    std::shared_ptr<const host::HostingStatus> status() const { return service_.status(); }
    HostPhaseBanner phase() const;
    // The next-launch hint: settings saved but not running yet, or changed since the session started. Empty when none applies.
    QString launchHint() const;
    QString invite() const;
    bool canCopyInvite() const;
    // The endpoint players use in Direct mode ("enet://host:port"); empty until listening or in Relay mode.
    QString directEndpoint() const;
    QList<HostRosterRow> roster() const;

    // ---- actions ----
    // Puts the invite on the clipboard; false when there is none.
    bool copyInvite();
    // Removes a remote player (never the host); false when nobody can be kicked.
    bool kick(sdk::match::ParticipantId participant);

signals:
    void changed();

private:
    void loadDraft();
    void apply();

    HostingService& service_;
    RelayPresets presets_;
    ClipboardSetter clipboard_;
    host::HostingMode mode_ = host::HostingMode::Off;
    int port_ = host::kDefaultHostingPort;
    int maxPlayers_ = static_cast<int>(sdk::match::kMaxRobotsPerMatch);
    RelayChoice choice_ = RelayChoice::Production;
    QString customRelay_;
    QString bindHost_;
    QString problem_;
    host::HostingOptions seen_;   // the options the draft was last in step with
    bool applying_ = false;
};

}  // namespace wpiemu::app
