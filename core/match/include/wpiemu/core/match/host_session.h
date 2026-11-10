#pragma once

// The narrow view of the simulation the match host needs. SimulationMatchSession adapts the real SimulationSession; tests
// use a fake. Threading: every method is called on the SIM THREAD only (inside MatchHost::onPoll / onTickEnd), the same
// rule as SimulationSession's remote robot API.

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::core::match {

class IMatchHostSession {
    WPIEMU_INTERFACE(IMatchHostSession)
public:
    // The host's game: a Join must name the same plugin and season. An empty plugin id means "no game".
    virtual std::string gamePluginId() const = 0;
    virtual int wpilibYear() const = 0;
    // The host robot's display name for the roster.
    virtual std::string hostName() const = 0;
    // The plugin's HUD descriptors: they define the HUD key table (hud_keys.h).
    virtual std::vector<sdk::HudWidgetDescriptor> hudDescriptors() const = 0;

    virtual std::int64_t stepUs() const = 0;
    virtual std::uint64_t tickIndex() const = 0;   // completed ticks of the current epoch
    virtual std::int64_t simTimeUs() const = 0;
    // The host Driver Station state (the host robot's alliance station, enable, mode, e-stop, match time).
    virtual sdk::proto::DsState driverStation() const = 0;

    // SimulationSession's remote robot API (same contracts; see simulation_session.h).
    virtual sdk::Status addRemoteRobot(const runtime::RemoteRobotRequest& request) = 0;
    virtual sdk::Status removeRemoteRobot(sdk::match::ParticipantId id) = 0;
    virtual sdk::Result<runtime::RemoteInputOutcome> submitRemoteInput(sdk::match::ParticipantId id,
                                                                       const sdk::match::MatchInput& input) = 0;
    virtual std::optional<runtime::RemoteRobotInfo> remoteRobotInfo(sdk::match::ParticipantId id) const = 0;

    // The immutable telemetry snapshot of the current instant (SimulationSession::buildSnapshot).
    virtual std::shared_ptr<const sdk::ISimSnapshot> buildSnapshot(bool paused, bool robotConnected) = 0;
};

// Forwards to a SimulationSession (not owned; it must outlive the adapter).
class SimulationMatchSession final : public IMatchHostSession {
public:
    explicit SimulationMatchSession(runtime::SimulationSession& session) : session_(session) {}

    std::string gamePluginId() const override;
    int wpilibYear() const override;
    std::string hostName() const override { return session_.config().name; }
    std::vector<sdk::HudWidgetDescriptor> hudDescriptors() const override;
    std::int64_t stepUs() const override { return session_.clock().fixedStepUs(); }
    std::uint64_t tickIndex() const override { return session_.clock().tickIndex(); }
    std::int64_t simTimeUs() const override { return session_.clock().simTimeUs(); }
    sdk::proto::DsState driverStation() const override { return session_.driverStation(); }
    sdk::Status addRemoteRobot(const runtime::RemoteRobotRequest& request) override { return session_.addRemoteRobot(request); }
    sdk::Status removeRemoteRobot(sdk::match::ParticipantId id) override { return session_.removeRemoteRobot(id); }
    sdk::Result<runtime::RemoteInputOutcome> submitRemoteInput(sdk::match::ParticipantId id,
                                                               const sdk::match::MatchInput& input) override {
        return session_.submitRemoteInput(id, input);
    }
    std::optional<runtime::RemoteRobotInfo> remoteRobotInfo(sdk::match::ParticipantId id) const override {
        return session_.remoteRobotInfo(id);
    }
    std::shared_ptr<const sdk::ISimSnapshot> buildSnapshot(bool paused, bool robotConnected) override {
        return session_.buildSnapshot(paused, robotConnected);
    }

private:
    runtime::SimulationSession& session_;
};

}  // namespace wpiemu::core::match
