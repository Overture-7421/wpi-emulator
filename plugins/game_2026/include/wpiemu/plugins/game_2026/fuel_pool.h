#pragma once

// Fuel bodies: spawn / despawn / sleeping configuration on top of IPhysicsWorld. Slots are reused;
// a fuel id (slot index + 1) is stable for the life of that fuel and is carried in the physics
// userTag so sensor events map back to it. RAII: destruction removes every body.

#include <cstdint>
#include <vector>

#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::plugins::game_2026 {

class FuelPool {
public:
    FuelPool(sdk::IPhysicsWorld& world, const FuelPhysicsConfig& physics, int maxBodies);
    ~FuelPool();

    FuelPool(const FuelPool&) = delete;
    FuelPool& operator=(const FuelPool&) = delete;

    struct SpawnOptions {
        sdk::Vec3 velocity;
        bool fast = false;                // continuous collision for fast shots
        sdk::SimTimeUs ghostUntilUs = 0;  // ignore the robot until then (shots only)
    };

    // 0 = refused (body cap reached or the world refused the body).
    std::uint32_t spawn(const sdk::Vec3& position, const SpawnOptions& options);
    void despawn(std::uint32_t id);
    void clear();
    // Restores robot collisions of ghosted fuel whose time has come.
    void updateGhosts(sdk::SimTimeUs nowUs);

    bool alive(std::uint32_t id) const;
    sdk::Vec3 position(std::uint32_t id) const;
    sdk::BodyId bodyOf(std::uint32_t id) const { return alive(id) ? slots_[id - 1].body : sdk::BodyId{}; }
    std::size_t count() const { return alive_; }

    // fn(id, position) for every live fuel in id order.
    template <class Fn>
    void forEach(Fn&& fn) const {
        for (std::size_t i = 0; i < slots_.size(); ++i)
            if (slots_[i].alive) fn(static_cast<std::uint32_t>(i + 1), world_.pose(slots_[i].body).position);
    }

private:
    struct Slot {
        sdk::BodyId body;
        sdk::ShapeId shape;
        bool alive = false;
        bool ghost = false;
        sdk::SimTimeUs ghostUntilUs = 0;
    };

    sdk::IPhysicsWorld& world_;
    FuelPhysicsConfig physics_;
    int maxBodies_;
    std::uint64_t normalMask_;
    std::vector<Slot> slots_;
    std::vector<std::uint32_t> freeList_;
    std::size_t alive_ = 0;
};

}  // namespace wpiemu::plugins::game_2026
