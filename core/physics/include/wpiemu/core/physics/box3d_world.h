#pragma once

// Box3D-backed IPhysicsWorld (docs/reference/physics.md). Box3D types never appear in this header.

#include <memory>
#include <string>

#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::core::physics {

// The returned world also implements IPhysicsJointHost (see joint_host.h). Returns null if Box3D rejects the
// configuration (e.g. non-finite gravity).
std::unique_ptr<sdk::IPhysicsWorld> createBox3dWorld(const sdk::PhysicsConfig& config = {});

// Composition-root hook: pass to code that takes an sdk::PhysicsWorldFactory.
sdk::PhysicsWorldFactory box3dWorldFactory();

// Box3D version this adapter was built against ("major.minor.revision").
std::string box3dVersionString();

}  // namespace wpiemu::core::physics
