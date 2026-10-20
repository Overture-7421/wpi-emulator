#pragma once

#include <string>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/devices/device_hub.h"
#include "wpiemu/sdk/game/game_state.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::core::runtime {

// Reading a sensor produces from the value of its source (docs/runtime.md "Sensors"). Pure; `properties` are the
// sensor's response properties (sdk::sensorResponseSchema), missing keys fall back to the schema defaults.
double distanceReadingM(const sdk::Value& properties, double input);
bool beamBreakReading(const sdk::Value& properties, double input);
// What a sensor reads when its source is unavailable: nothing detected.
double idleDistanceM(const sdk::Value& properties);
bool idleBeamBreakReading(const sdk::Value& properties);

// Game-fed sensors (ADR 0019). Owned by SimulationSession and stepped once per tick right after the game state, so the
// robot sees the reading of the tick that just ran. Core never knows what a source means: it reads
// IGameState::gameValue(source) and writes the resulting reading to the sensor's slot.
// Threading: sim thread only.
class SensorSystem {
public:
    explicit SensorSystem(const sdk::RobotConfig& config);

    bool empty() const { return config_.sensors.empty(); }
    // `state` may be null (no game plugin): every sensor then reads idle.
    void step(const sdk::IGameState* state, sdk::ISensorWriter& hub);
    // One warning per sensor whose source was unavailable in the last step ("sensor_source_unavailable").
    std::vector<sdk::HubWarning> warnings() const;
    // Adds every sensor's latest state to the snapshot (idle/unavailable before the first step).
    void publish(sdk::SnapshotBuilder& builder) const;

private:
    const sdk::RobotConfig& config_;
    std::vector<std::string> unavailable_;
    std::vector<sdk::SensorState> states_;
};

}  // namespace wpiemu::core::runtime
