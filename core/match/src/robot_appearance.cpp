#include "wpiemu/core/match/robot_appearance.h"

#include <vector>

#include "wpiemu/core/physics/chassis_builder.h"

namespace wpiemu::core::match {

sdk::match::RobotAppearance appearanceOf(const sdk::RobotConfig& config) {
    const std::vector<sdk::WheelSpec>& wheels = config.drivetrain.spec.wheels;
    std::vector<sdk::Vec3> hubs;
    hubs.reserve(wheels.size());
    for (const sdk::WheelSpec& wheel : wheels) hubs.push_back(physics::wheelHubLocal(config.chassis, wheel));
    return sdk::match::makeRobotAppearance(config.chassis, wheels, hubs, config.zones);
}

}  // namespace wpiemu::core::match
