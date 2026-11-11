#pragma once

// Everything the game client composes once, Qt-free: the physics registries, the built-in game plugins and the real config
// pipeline (the same decode, defaults and validation the host and the editor run), the match codec and the wall clock.
// Nothing here needs WPILib, a JDK or a project folder: the client only describes a robot, it never runs robot code.

#include <memory>
#include <string>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/core/match/match_client.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/tire_model.h"

namespace wpiemu::app::client {

class ClientServices {
public:
    // Registers the physics components and the built-in game plugins; an error names what failed.
    static sdk::Result<std::unique_ptr<ClientServices>> create(std::shared_ptr<sdk::IWallClock> clock = nullptr);

    const core::config::ConfigPipeline& pipeline() const { return pipeline_; }
    const core::config::JsonValueCodec& json() const { return json_; }
    const sdk::GamePluginRegistry& plugins() const { return plugins_; }
    const std::shared_ptr<const sdk::proto::IMessageCodec>& codec() const { return codec_; }
    const std::shared_ptr<sdk::IWallClock>& clock() const { return clock_; }
    const sdk::DrivetrainRegistry& drivetrains() const { return drivetrains_; }
    const sdk::TireModelRegistry& tireModels() const { return tireModels_; }

    // The game module a robot description names (empty id: none), nullptr when this build does not have it.
    const sdk::IGameModule* gameOf(const std::string& gamePluginId) const;

    // What a MatchClient is built from: the services above, the registries for its prediction world and the HUD key table of
    // the robot's game plugin (the host builds the same table from the same plugin).
    core::match::MatchClientServices matchServices(const std::string& gamePluginId, sdk::ILogger* logger = nullptr) const;

private:
    ClientServices();

    sdk::DrivetrainRegistry drivetrains_;
    sdk::TireModelRegistry tireModels_;
    sdk::MechanismFactory mechanisms_;
    sdk::GamePluginRegistry plugins_;
    core::config::JsonValueCodec json_;
    core::config::RobotConfigCodec configCodec_;
    core::config::ConfigValidator validator_;
    core::config::PluginRegistrar registrar_;
    core::config::ConfigPipeline pipeline_;
    std::shared_ptr<const sdk::proto::IMessageCodec> codec_;
    std::shared_ptr<sdk::IWallClock> clock_;
};

}  // namespace wpiemu::app::client
