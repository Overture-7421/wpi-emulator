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

// What the controller needs of a session, so the window can run against the real network session or a scripted one (the demo).
class IClientSession {
public:
    virtual ~IClientSession() = default;

    // Starts (asynchronously) joining with `robot`. Replaces a session in progress.
    virtual void join(JoinPlan plan, sdk::RobotConfig robot) = 0;
    // Tells the host, closes the link and returns to Idle; also cancels a dial in progress.
    virtual void leave() = 0;
    // After TransportLost or HostSilent: dials the same host again and rejoins with the resume token.
    virtual void reconnect() = 0;

    virtual SessionState state() const = 0;
    virtual std::shared_ptr<const core::match::MatchClientView> view() const = 0;
    virtual std::shared_ptr<const sdk::RobotConfig> acceptedSpec() const = 0;
    virtual std::shared_ptr<const sdk::FieldDescription> fieldDescription() const = 0;
    virtual std::string gamePluginId() const = 0;

    // The player's command (see MatchClient). Refused while there is no joined robot.
    virtual sdk::Status driveRobotRelative(const sdk::ChassisSpeeds& speeds) = 0;
    virtual sdk::Status driveFieldRelative(const sdk::ChassisSpeeds& fieldSpeeds) = 0;
    virtual void setZoneFlags(std::uint32_t flags) = 0;
    virtual sdk::Status setZonePose(std::size_t zone, const sdk::Pose3& chassisPose) = 0;
    virtual void clearZonePose(std::size_t zone) = 0;
};

struct ClientSessionOptions {
    LinkOptions link;
    std::chrono::microseconds updateInterval{2000};   // how often the worker calls MatchClient::update()
    core::match::MatchClientConfig match;             // timeouts and prediction settings; displayName comes from the plan
    // Replaces openSecureLink (tests run the match over in-memory links); it is called for the first dial and every reconnect.
    LinkOpener opener;
};

class ClientSession final : public IClientSession {
public:
    explicit ClientSession(const ClientServices& services, ClientSessionOptions options = {}, sdk::ILogger* logger = nullptr);
    ~ClientSession() override;
    ClientSession(const ClientSession&) = delete;
    ClientSession& operator=(const ClientSession&) = delete;

    void join(JoinPlan plan, sdk::RobotConfig robot) override;
    void leave() override;
    void reconnect() override;

    SessionState state() const override;
    std::shared_ptr<const core::match::MatchClientView> view() const override;
    std::shared_ptr<const sdk::RobotConfig> acceptedSpec() const override;
    std::shared_ptr<const sdk::FieldDescription> fieldDescription() const override;
    std::string gamePluginId() const override;

    sdk::Status driveRobotRelative(const sdk::ChassisSpeeds& speeds) override;
    sdk::Status driveFieldRelative(const sdk::ChassisSpeeds& fieldSpeeds) override;
    void setZoneFlags(std::uint32_t flags) override;
    sdk::Status setZonePose(std::size_t zone, const sdk::Pose3& chassisPose) override;
    void clearZonePose(std::size_t zone) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::app::client
