#pragma once

// The generic, game-agnostic mechanism template (registered into a MechanismFactory):
//
//   type        display     axis                                         published values
//   mechanism   Mechanism   ONE motor-driven degree of freedom:          position, velocity, voltage, current,
//                           Motion = rotary | arm | linear               atLowerLimit, atUpperLimit
//
// Flywheels, rollers, belts, turrets (rotary), hoods and pivots (arm), elevators and climbers (linear) are all this one
// template with different properties; intakes and other season constructs belong to the game plugin, which composes
// mechanisms by id. See docs/mechanisms.md for slots, properties and published state.

#include <memory>

#include "wpiemu/core/mechanisms/motor_catalog.h"
#include "wpiemu/sdk/dynamics/plant.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::core::mechanisms {

namespace mechanism_type {
inline constexpr const char* kMechanism = "mechanism";
}  // namespace mechanism_type

// Collaborators shared by all templates. Null members are replaced by the builtin motor catalog /
// a PlantFactory with the builtin plants. Both are shared (not borrowed) so the registered factory
// callbacks can never outlive them.
struct MechanismTemplateServices {
    std::shared_ptr<const MotorCatalog> motors;
    std::shared_ptr<const sdk::PlantFactory> plants;
};

// Registers the template above. Stops at and returns the first registration error
// (e.g. AlreadyExists when called twice).
sdk::Status registerGenericMechanisms(sdk::MechanismFactory& factory, MechanismTemplateServices services = {});

}  // namespace wpiemu::core::mechanisms
