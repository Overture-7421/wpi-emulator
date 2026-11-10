#include "client/client_services.h"

#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/net/match_codec.h"
#include "wpiemu/sdk/game/builtin_plugins.h"

namespace wpiemu::app::client {

ClientServices::ClientServices()
    : validator_(&tireModels_, {}),
      registrar_(mechanisms_),
      pipeline_(configCodec_, validator_, registrar_, drivetrains_, plugins_),
      codec_(net::makeMatchCodec()) {}

sdk::Result<std::unique_ptr<ClientServices>> ClientServices::create(std::shared_ptr<sdk::IWallClock> clock) {
    std::unique_ptr<ClientServices> services(new ClientServices());
    services->clock_ = clock ? std::move(clock) : std::make_shared<sdk::SteadyWallClock>();
    if (const sdk::Status s = core::physics::registerPhysicsComponents(services->drivetrains_, services->tireModels_); !s.ok())
        return sdk::makeError(s.error().code, "drivetrains: " + s.error().message);
    if (const sdk::Status s = sdk::registerBuiltinGamePlugins(services->plugins_); !s.ok())
        return sdk::makeError(s.error().code, "game plugins: " + s.error().message);
    return services;
}

const sdk::IGameModule* ClientServices::gameOf(const std::string& gamePluginId) const {
    return gamePluginId.empty() ? nullptr : plugins_.find(gamePluginId);
}

core::match::MatchClientServices ClientServices::matchServices(const std::string& gamePluginId, sdk::ILogger* logger) const {
    core::match::MatchClientServices s;
    s.codec = codec_;
    s.clock = clock_;
    s.logger = logger;
    s.specPipeline = &pipeline_;
    s.plugins = &plugins_;
    s.drivetrains = &drivetrains_;
    s.tireModels = &tireModels_;
    if (const sdk::IGameModule* game = gameOf(gamePluginId)) s.hudKeys = core::match::HudKeyTable::fromDescriptors(game->contributeHud());
    return s;
}

}  // namespace wpiemu::app::client
