#pragma once

// The game client's UI-thread brain, without widgets: it validates the join form, loads the robot file, runs the session,
// turns the player's joystick into commands, keeps the zone controls and hands the window one immutable picture per change.
// A timer calls tick() (~60 Hz); tests call it directly with a fake joystick, so the whole flow runs without a window.
//
// Threading: UI thread only (the session has its own worker for the network).

#include <QObject>
#include <QString>

#include <chrono>
#include <memory>
#include <string>

#include "client/client_services.h"
#include "client/client_session.h"
#include "client/client_settings.h"
#include "client/client_state.h"
#include "client/input_mapping.h"
#include "client/join_form.h"
#include "client/robot_file.h"
#include "client/scene_feed.h"

namespace wpiemu::app::client {

// Where the player's joystick comes from: the run view's keyboard and gamepad bindings in the app, a fake in tests.
class IClientInput {
public:
    virtual ~IClientInput() = default;
    virtual sdk::proto::JoystickState sample() = 0;
};

class ClientController : public QObject {
    Q_OBJECT
public:
    ClientController(ClientServices& services, IKeyValueStore& store, IClientInput* input, ClientSessionOptions session = {},
                     QObject* parent = nullptr);
    // Runs against another session (the scripted one of the demo).
    ClientController(ClientServices& services, IKeyValueStore& store, IClientInput* input, std::unique_ptr<IClientSession> session,
                     QObject* parent = nullptr);
    ~ClientController() override;

    // ---- settings ----
    const ClientPrefs& prefs() const { return prefs_; }
    void setPrefs(const ClientPrefs& prefs);   // normalised and stored
    RelayPresets relayPresets() const { return settings_.relayPresets(); }

    // ---- join ----
    // The checked robot file (nothing chosen yet: not ok, no errors).
    const RobotFileReport& robot() const { return robot_; }
    void chooseRobot(const QString& path);
    JoinValidation validate(const JoinFormInput& input) const;
    // Validates the form and the robot, stores the preferences (never the invite) and starts joining. The returned validation
    // lists the fields that are wrong (the robot file included, as robot.missing / robot.invalid); the join started when ok.
    JoinValidation join(const JoinFormInput& input);
    void leave();
    void reconnect();

    // ---- state ----
    SessionState sessionState() const { return session_.state(); }
    const Presentation& presentation() const { return presentation_; }
    std::shared_ptr<const core::match::MatchClientView> view() const { return view_; }
    // What the scene draws; null until the own robot exists.
    std::shared_ptr<const sdk::ISimSnapshot> snapshot() const { return snapshot_; }
    const HostScene& scene() const { return scene_; }
    std::shared_ptr<const MatchRoster> roster() const { return roster_; }
    // Increase when the scene (robot, field, game) or the roster must be rebuilt by the view.
    std::uint64_t sceneRevision() const { return sceneRevision_; }
    std::uint64_t rosterRevision() const { return rosterRevision_; }
    const std::string& gamePluginId() const { return pluginId_; }

    // ---- the player's controls ----
    const DriveInputConfig& driveConfig() const { return prefs_.drive; }
    const DriveLimits& driveLimits() const { return limits_; }
    // Zones held active by the on-screen buttons (joystick buttons add to them).
    void setZoneButton(std::size_t zone, bool active);
    std::uint32_t zoneButtons() const { return uiZoneFlags_; }
    ZonePoseControls& zonePoses() { return zonePoses_; }
    const ZonePoseControls& zonePoses() const { return zonePoses_; }
    // Slider side of the zone pose controls: sets and sends the pose of a zone (or goes back to the rest pose).
    void moveZone(std::size_t zone, const sdk::Vec3& position, float yawOffsetRad);
    void resetZone(std::size_t zone);

    // One UI step: refresh the presentation and the picture, read the joystick, send the command. `dtSeconds` <= 0 measures.
    void tick(double dtSeconds = 0.0);

    IClientSession& session() { return session_; }

signals:
    void robotChanged();
    void presentationChanged();
    void viewChanged();       // a new picture (snapshot()) is ready
    void sceneChanged();      // the scene or the roster changed: see sceneRevision / rosterRevision

private:
    void refreshPicture();
    void sendCommand(double dtSeconds);

    ClientServices& services_;
    ClientSettings settings_;
    IClientInput* input_;
    std::unique_ptr<IClientSession> ownedSession_;
    IClientSession& session_;
    ClientPrefs prefs_;
    RobotFileReport robot_;
    Presentation presentation_;

    std::shared_ptr<const core::match::MatchClientView> view_;
    std::shared_ptr<const sdk::ISimSnapshot> snapshot_;
    HostScene scene_;
    std::shared_ptr<const MatchRoster> roster_;
    std::shared_ptr<const sdk::RobotConfig> sceneConfig_;
    std::shared_ptr<const sdk::match::RobotAppearance> ownAppearance_;
    std::shared_ptr<const sdk::RobotConfig> accepted_;
    std::shared_ptr<const sdk::FieldDescription> field_;
    std::string pluginId_;
    std::string phaseKey_;
    std::uint64_t sceneRevision_ = 0;
    std::uint64_t rosterRevision_ = 0;

    DriveLimits limits_;
    bool commandHeld_ = false;   // a drive command is standing (it is cancelled when the controls go away)
    ZonePoseControls zonePoses_;
    std::uint32_t uiZoneFlags_ = 0;
    std::chrono::steady_clock::time_point lastTick_{};
};

}  // namespace wpiemu::app::client
