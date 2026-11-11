#pragma once

// A scripted match for screenshots, the `--demo` switch and the window tests: the same IClientSession the controller drives, but
// nothing leaves the process. The own robot answers the keyboard and gamepad (simple kinematics, not the physics of a real match),
// the host's robot and two more players drive circles, fuel lies on the field, and the connection sits in the state the scenario
// names, with believable diagnostics. The field is the real one of the robot's game (a prediction world supplies it).

#include <memory>
#include <optional>
#include <string_view>

#include "client/client_services.h"
#include "client/client_session.h"

namespace wpiemu::app::client {

enum class DemoScenario : std::uint8_t {
    Join,           // not joined: the join screen
    Lobby,          // joined, the host has not started
    Live,           // a running match
    Paused,
    HostStalled,
    Reconnecting,   // the link broke, redialling
    Lost,           // the link broke and did not come back
    Kicked,
    Closed,         // the host closed the room
};

std::optional<DemoScenario> parseDemoScenario(std::string_view name);
// "join, lobby, live, paused, stalled, reconnecting, lost, kicked, closed".
const char* demoScenarioNames();

// The robot description of the scripted player (what a robot.json would hold), in the first game plugin this build has.
sdk::RobotConfig demoRobotConfig(const ClientServices& services);

class DemoClientSession final : public IClientSession {
public:
    DemoClientSession(const ClientServices& services, DemoScenario scenario);
    ~DemoClientSession() override;

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
