#include "wpiemu/core/match/host_session.h"

namespace wpiemu::core::match {

std::string SimulationMatchSession::gamePluginId() const {
    if (const sdk::IGameModule* module = session_.gameModule()) return module->metadata().id;
    return session_.config().gamePluginId;
}

int SimulationMatchSession::wpilibYear() const {
    if (const sdk::IGameModule* module = session_.gameModule()) return module->metadata().year;
    return session_.config().project.wpilibYear;
}

std::vector<sdk::HudWidgetDescriptor> SimulationMatchSession::hudDescriptors() const {
    if (const sdk::IGameModule* module = session_.gameModule()) return module->contributeHud();
    return {};
}

}  // namespace wpiemu::core::match
