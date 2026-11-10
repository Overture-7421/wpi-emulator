#pragma once

// The hosting preferences and the live status of a practice match, for the UI (docs/reference/app.md). The service owns the
// persisted options (it is the only code that writes them) and relays the status of the running session; it starts and stops
// nothing itself. RealRuntimeHost applies the options when it launches the robot and publishes the status while it runs.
//
// Threading: UI thread only. The status it hands out is an immutable snapshot (a new pointer for every change); the invite
// inside it is the only place the secret is exposed. Never persists the invite: a fresh one is drawn for every session.

#include <QObject>
#include <QString>

#include <memory>
#include <optional>

#include "common/key_value_store.h"
#include "host/hosting.h"

namespace wpiemu::app {

class HostingService : public QObject {
    Q_OBJECT
public:
    explicit HostingService(IKeyValueStore& store, QObject* parent = nullptr);

    // What the next robot launch will use (loaded from the store, normalised).
    const host::HostingOptions& options() const { return options_; }
    // Normalises (player count clamped, empty bind address reset), persists and announces a change. The running session keeps
    // the options it was launched with; restartRequired() says whether a relaunch would change anything.
    void setOptions(const host::HostingOptions& options);
    // Ok when a launch with options() could start hosting; otherwise the message to show next to the settings.
    sdk::Status validate() const { return host::validateHosting(options_); }

    // The game's match phase key, which a hosted match copies into its snapshots: stored with the options so the running
    // session and the preferences agree (RealRuntimeHost sets it from the open project's game).
    void setPhaseHudKey(const std::string& key);

    // The latest status of the running session; never null (phase Off when not hosting).
    std::shared_ptr<const host::HostingStatus> status() const { return status_; }
    // The options the running session was launched with (nullopt when no session runs).
    const std::optional<host::HostingOptions>& appliedOptions() const { return applied_; }
    bool restartRequired() const { return applied_ && *applied_ != options_; }

    // Ask the running session to remove a participant (MatchControl::Kick). Ignored when nobody is hosting.
    void kick(sdk::match::ParticipantId participant, const QString& reason = {});

    // ---- backend side (RealRuntimeHost) ----
    void sessionStarted(const host::HostingOptions& applied);
    void sessionEnded();
    void publish(std::shared_ptr<const host::HostingStatus> status);

signals:
    void optionsChanged();
    void statusChanged();
    // The backend's cue: remove this participant from the running match.
    void kickRequested(quint16 participant, const QString& reason);

private:
    void load();
    void save() const;

    IKeyValueStore& store_;
    host::HostingOptions options_;
    std::optional<host::HostingOptions> applied_;
    std::shared_ptr<const host::HostingStatus> status_;
};

}  // namespace wpiemu::app
