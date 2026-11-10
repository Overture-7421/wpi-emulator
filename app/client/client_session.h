#pragma once

// One player's connection to a match: opens the encrypted link, owns the MatchClient and runs it on a worker thread, so a slow
// dial, the invite handshake or a reconnect never freezes the window. The UI thread asks for things (join, leave, reconnect),
// sets the player's command and reads immutable state; nothing here touches Qt.
//
// Threading: join/leave/reconnect, the command setters and every query may be called from any one UI thread; the worker calls
// MatchClient::update() (>= 120 Hz) and is the only thread that connects, resumes or leaves.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>

#include "client/client_services.h"
#include "client/join_form.h"
#include "client/secure_link.h"
#include "wpiemu/core/match/client_view.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/ipc/transport.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"

namespace wpiemu::app::client {

enum class LinkPhase : std::uint8_t {
    Idle,      // not joining
    Opening,   // dialling the host or the relay and running the invite handshake
    Active,    // the MatchClient has the link (its own status says Connecting, Lobby, Live, ... or why it ended)
    Failed,    // the link could not be opened: see SessionState::openError
};

struct SessionState {
    LinkPhase phase = LinkPhase::Idle;
    std::string openError;                 // Failed
    core::match::ConnectionStatus status;  // Active (and while reconnecting by hand)
    bool everJoined = false;               // the host accepted this player during this session
    bool reopening = false;                // a manual reconnect is dialling again

    bool operator==(const SessionState&) const = default;
};

using LinkOpener = std::function<sdk::Result<std::unique_ptr<sdk::proto::ITransport>>(const JoinPlan&)>;

struct ClientSessionOptions {
    LinkOptions link;
    std::chrono::microseconds updateInterval{2000};   // how often the worker calls MatchClient::update()
    core::match::MatchClientConfig match;             // timeouts and prediction settings; displayName comes from the plan
    // Replaces openSecureLink (tests run the match over in-memory links); it is called for the first dial and every reconnect.
    LinkOpener opener;
};

class ClientSession {
public:
    explicit ClientSession(const ClientServices& services, ClientSessionOptions options = {}, sdk::ILogger* logger = nullptr);
    ~ClientSession();
    ClientSession(const ClientSession&) = delete;
    ClientSession& operator=(const ClientSession&) = delete;

    // Starts (asynchronously) joining with `robot`. Replaces a session in progress.
    void join(JoinPlan plan, sdk::RobotConfig robot);
    // Tells the host, closes the link and returns to Idle; also cancels a dial in progress.
    void leave();
    // After TransportLost or HostSilent: dials the same host again and rejoins with the resume token.
    void reconnect();

    SessionState state() const;
    std::shared_ptr<const core::match::MatchClientView> view() const;
    std::shared_ptr<const sdk::RobotConfig> acceptedSpec() const;
    std::shared_ptr<const sdk::FieldDescription> fieldDescription() const;
    std::string gamePluginId() const;

    // The player's command (see MatchClient). Refused while there is no joined robot.
    sdk::Status driveRobotRelative(const sdk::ChassisSpeeds& speeds);
    sdk::Status driveFieldRelative(const sdk::ChassisSpeeds& fieldSpeeds);
    void setZoneFlags(std::uint32_t flags);
    sdk::Status setZonePose(std::size_t zone, const sdk::Pose3& chassisPose);
    void clearZonePose(std::size_t zone);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::app::client
