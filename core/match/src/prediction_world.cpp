#include "wpiemu/core/match/prediction_world.h"

#include <algorithm>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/runtime/physics_wiring.h"
#include "wpiemu/sdk/game/default_field.h"

namespace wpiemu::core::match {

namespace {

constexpr std::uint64_t kKinematicRobotMask = sdk::layer::Field | sdk::layer::Robot;

bool sameLook(const sdk::match::RobotAppearance& a, const sdk::match::RobotAppearance& b) {
    return a.sizeXMm == b.sizeXMm && a.sizeYMm == b.sizeYMm && a.sizeZMm == b.sizeZMm && a.centreXMm == b.centreXMm &&
           a.centreYMm == b.centreYMm && a.centreZMm == b.centreZMm;
}

}  // namespace

PredictionWorld::~PredictionWorld() {
    clearRemotes();
    chassis_.reset();
    field_.reset();
    world_.reset();
}

sdk::Result<std::unique_ptr<PredictionWorld>> PredictionWorld::create(const PredictionWorldArgs& args) {
    std::unique_ptr<PredictionWorld> out(new PredictionWorld());
    sdk::PhysicsConfig config;
    config.workerCount = 1;
    config.sleeping = false;
    out->world_ = args.worldFactory ? args.worldFactory(config) : physics::createBox3dWorld(config);
    if (!out->world_) return sdk::makeError(sdk::ErrorCode::InvalidState, "the prediction physics world could not be created");

    sdk::IGameModule* module = args.plugins != nullptr && !args.spec.gamePluginId.empty() ? args.plugins->find(args.spec.gamePluginId) : nullptr;
    sdk::Result<std::unique_ptr<sdk::IField>> field = module != nullptr ? module->createField(*out->world_) : sdk::createDefaultField(*out->world_);
    if (!field) return field.error();
    out->field_ = std::move(field).value();

    sdk::Result<std::unique_ptr<runtime::IRemoteChassis>> chassis = runtime::PhysicsRemoteChassis::create(
        *out->world_, args.drivetrains, args.tireModels, args.spec, args.ownTag, args.placement);
    if (!chassis) return chassis.error();
    out->chassis_ = std::move(chassis).value();

    const float dt = 0.005f;
    for (int i = 0; i < args.settleTicks; ++i) out->step(false, dt);
    return out;
}

std::size_t PredictionWorld::wheelCount() const { return chassis_->wheelCount(); }

const sdk::FieldDescription& PredictionWorld::field() const { return field_->description(); }

OwnBodyState PredictionWorld::capture() const {
    OwnBodyState s;
    const sdk::BodyId body = chassis_->body();
    s.pose = world_->pose(body);
    s.linearVelocity = world_->linearVelocity(body);
    s.angularVelocity = world_->angularVelocity(body);
    for (const sdk::WheelState& w : chassis_->wheelStates())
        s.wheels.push_back({w.driveWheelAngleRad, w.driveWheelOmegaRadPerSec, w.steerAngleRad, w.steerOmegaRadPerSec});
    return s;
}

sdk::Status PredictionWorld::restore(const OwnBodyState& state) {
    if (state.wheels.size() != chassis_->wheelCount())
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, "restored wheel count differs from the robot's");
    const sdk::BodyId body = chassis_->body();
    world_->setPose(body, state.pose);
    world_->setLinearVelocity(body, state.linearVelocity);
    world_->setAngularVelocity(body, state.angularVelocity);
    if (!chassis_->restoreWheelKinematics(state.wheels))
        return sdk::makeError(sdk::ErrorCode::Unsupported, "the drivetrain cannot restore its wheel state");
    return sdk::okStatus();
}

void PredictionWorld::setSetpoints(std::span<const sdk::WheelSetpoint> setpoints) { chassis_->setSetpoints(setpoints); }

void PredictionWorld::brake() { chassis_->brake(); }

void PredictionWorld::step(bool enabled, float dtSeconds) {
    chassis_->prepareStep(enabled, dtSeconds);
    world_->step(dtSeconds);
    chassis_->completeStep(dtSeconds);
}

void PredictionWorld::destroyRemote(RemoteBody& body) {
    if (body.body.valid()) world_->destroyBody(body.body);
    body.body = {};
}

void PredictionWorld::setRemote(sdk::match::ParticipantId id, const sdk::match::RobotAppearance& look, const sdk::Pose3& pose,
                                const sdk::Vec3& linearVelocity, float yawRateRadPerSec) {
    auto it = remotes_.find(id);
    if (it != remotes_.end() && !sameLook(it->second.look, look)) {
        destroyRemote(it->second);
        remotes_.erase(it);
        it = remotes_.end();
    }
    if (it == remotes_.end()) {
        sdk::BodyDesc desc;
        desc.type = sdk::BodyType::Kinematic;
        desc.pose = pose;
        desc.allowSleep = false;
        RemoteBody body;
        body.body = world_->createBody(desc);
        if (!body.body.valid()) return;
        sdk::ShapeDesc shape;
        shape.geometry = sdk::BoxShape{{std::max(0.01f, sdk::match::appearanceMetres(look.sizeXMm) * 0.5f),
                                        std::max(0.01f, sdk::match::appearanceMetres(look.sizeYMm) * 0.5f),
                                        std::max(0.01f, sdk::match::appearanceMetres(look.sizeZMm) * 0.5f)}};
        shape.local.position = {sdk::match::appearanceMetres(look.centreXMm), sdk::match::appearanceMetres(look.centreYMm),
                                sdk::match::appearanceMetres(look.centreZMm)};
        shape.category = sdk::layer::Robot;
        shape.mask = kKinematicRobotMask;
        world_->addShape(body.body, shape);
        body.look = look;
        it = remotes_.emplace(id, std::move(body)).first;
    }
    world_->setPose(it->second.body, pose);
    world_->setLinearVelocity(it->second.body, linearVelocity);
    world_->setAngularVelocity(it->second.body, {0.f, 0.f, yawRateRadPerSec});
}

void PredictionWorld::removeRemote(sdk::match::ParticipantId id) {
    const auto it = remotes_.find(id);
    if (it == remotes_.end()) return;
    destroyRemote(it->second);
    remotes_.erase(it);
}

void PredictionWorld::clearRemotes() {
    for (auto& [id, body] : remotes_) destroyRemote(body);
    remotes_.clear();
}

std::uint64_t PredictionWorld::stateHash() const { return world_->stateHash(); }

}  // namespace wpiemu::core::match
