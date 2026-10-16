#include "wpiemu/plugins/game_2026/fuel_pool.h"

#include <cmath>

#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/types.h"

namespace wpiemu::plugins::game_2026 {

namespace {
constexpr float kPi = 3.14159265358979f;
}

FuelPool::FuelPool(sdk::IPhysicsWorld& world, const FuelPhysicsConfig& physics, int maxBodies)
    : world_(world), physics_(physics), maxBodies_(maxBodies),
      normalMask_(sdk::layer::Field | sdk::layer::Robot | sdk::layer::Fuel | sdk::layer::Sensor) {}

FuelPool::~FuelPool() { clear(); }

std::uint32_t FuelPool::spawn(const sdk::Vec3& position, const SpawnOptions& options) {
    if (static_cast<int>(alive_) >= maxBodies_) return 0;

    sdk::BodyDesc bd;
    bd.type = sdk::BodyType::Dynamic;
    bd.pose = sdk::Pose3{position, sdk::Quat{}};
    bd.linearVelocity = options.velocity;
    bd.allowSleep = physics_.sleeping;
    bd.bullet = options.fast;
    const sdk::BodyId body = world_.createBody(bd);
    if (!body.valid()) return 0;

    std::uint32_t id;
    if (!freeList_.empty()) {
        id = freeList_.back();
        freeList_.pop_back();
    } else {
        slots_.emplace_back();
        id = static_cast<std::uint32_t>(slots_.size());
    }

    const float r = dim::kFuelRadiusM;
    const float volume = 4.f / 3.f * kPi * r * r * r;
    sdk::ShapeDesc sd;
    sd.geometry = sdk::SphereShape{r};
    sd.material = {physics_.friction, physics_.restitution, physics_.rollingResistance};
    sd.density = physics_.massKg / volume;
    sd.category = sdk::layer::Fuel;
    const bool ghost = options.ghostUntilUs > 0;
    sd.mask = ghost ? (normalMask_ & ~sdk::layer::Robot) : normalMask_;
    sd.sensorEvents = true;
    sd.userTag = makeTag(EntityKind::Fuel, Alliance::Red, id);
    const sdk::ShapeId shape = world_.addShape(body, sd);
    if (!shape.valid()) {
        world_.destroyBody(body);
        freeList_.push_back(id);
        return 0;
    }

    Slot& slot = slots_[id - 1];
    slot.body = body;
    slot.shape = shape;
    slot.alive = true;
    slot.ghost = ghost;
    slot.ghostUntilUs = options.ghostUntilUs;
    ++alive_;
    return id;
}

void FuelPool::despawn(std::uint32_t id) {
    if (!alive(id)) return;
    Slot& slot = slots_[id - 1];
    world_.destroyBody(slot.body);
    slot = Slot{};
    freeList_.push_back(id);
    --alive_;
}

void FuelPool::clear() {
    for (std::size_t i = 0; i < slots_.size(); ++i)
        if (slots_[i].alive) world_.destroyBody(slots_[i].body);
    slots_.clear();
    freeList_.clear();
    alive_ = 0;
}

void FuelPool::updateGhosts(sdk::SimTimeUs nowUs) {
    for (Slot& slot : slots_)
        if (slot.alive && slot.ghost && nowUs >= slot.ghostUntilUs) {
            world_.setShapeFilter(slot.shape, sdk::layer::Fuel, normalMask_);
            slot.ghost = false;
        }
}

bool FuelPool::alive(std::uint32_t id) const { return id >= 1 && id <= slots_.size() && slots_[id - 1].alive; }

sdk::Vec3 FuelPool::position(std::uint32_t id) const {
    return alive(id) ? world_.pose(slots_[id - 1].body).position : sdk::Vec3{};
}

}  // namespace wpiemu::plugins::game_2026
