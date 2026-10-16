#pragma once

// Generic, game-agnostic mechanism templates (registered into a MechanismFactory):
//
//   type           display            axes                                   notable named values
//   roller         Generic roller     1 rotary                               surfaceSpeed
//   generic_1dof   Generic 1-DOF      1 rotary / arm / linear                position, velocity
//   intake         Intake             deploy (arm/linear) + roller           deployed, rollerSurfaceSpeed
//   shooter        Shooter / flywheel 1 rotary flywheel                      surfaceSpeed
//   pivot          Hood / pivot       1 arm (gravity optional)               position (rad from horizontal)
//   turret         Turret             1 rotary with optional stops           position (rad, yaw)
//   elevator       Elevator / climber 1 linear (gravity on by default)       position (m)
//   indexer        Indexer / conveyor 1 rotary belt                          surfaceSpeed
//
// See docs/mechanisms.md for slots, properties and published state. Templates know no season: a
// game plugin composes them through the same factory or reads their MechanismState.

#include <memory>

#include "wpiemu/core/mechanisms/motor_catalog.h"
#include "wpiemu/sdk/dynamics/plant.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::core::mechanisms {

namespace mechanism_type {
inline constexpr const char* kRoller = "roller";
inline constexpr const char* kGeneric1Dof = "generic_1dof";
inline constexpr const char* kIntake = "intake";
inline constexpr const char* kShooter = "shooter";
inline constexpr const char* kPivot = "pivot";
inline constexpr const char* kTurret = "turret";
inline constexpr const char* kElevator = "elevator";
inline constexpr const char* kIndexer = "indexer";
}  // namespace mechanism_type

// Collaborators shared by all templates. Null members are replaced by the builtin motor catalog /
// a PlantFactory with the builtin plants. Both are shared (not borrowed) so the registered factory
// callbacks can never outlive them.
struct MechanismTemplateServices {
    std::shared_ptr<const MotorCatalog> motors;
    std::shared_ptr<const sdk::PlantFactory> plants;
};

// Registers the eight templates above. Stops at and returns the first registration error
// (e.g. AlreadyExists when called twice).
sdk::Status registerGenericMechanisms(sdk::MechanismFactory& factory, MechanismTemplateServices services = {});

}  // namespace wpiemu::core::mechanisms
