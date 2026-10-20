#pragma once

// Test scaffolding for the game_2026 plugin: a scripted mechanism whose published values the test
// controls, and a Session that wires module + field + game state + mechanisms the way the runtime does.

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/plugins/game_2026/fuel_zones.h"
#include "wpiemu/plugins/game_2026/game_state.h"
#include "wpiemu/plugins/game_2026/plugin.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/game/zone_activation.h"
#include "wpiemu/sdk/testing/fake_mechanism_directory.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

namespace g26test {

using namespace wpiemu;

// A mechanism that just publishes the values the test sets (flywheel speed, indexer speed, hood angle...).
class ScriptedMechanism final : public sdk::IMechanism {
public:
    explicit ScriptedMechanism(std::string id, std::string type = "test.scripted")
        : id_(std::move(id)), type_(std::move(type)) {}
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return type_; }
    void tick(sdk::SimContext&, double) override {}
    void reset() override {}
    sdk::MechanismState state() const override {
        sdk::MechanismState s;
        s.id = id_;
        s.type = type_;
        s.active = active;
        s.values = values;
        s.links = links;
        return s;
    }
    void set(const std::string& name, double value, const std::string& unit = "") {
        for (sdk::NamedValue& v : values)
            if (v.name == name) {
                v.value = value;
                v.unit = unit;
                return;
            }
        values.push_back({name, value, unit});
    }
    bool active = false;
    std::vector<sdk::NamedValue> values;
    std::vector<sdk::LinkPose> links;

private:
    std::string id_, type_;
};

// Activation conditions on a mechanism's published value (SI): |value| > threshold / |value - position| <= tolerance.
inline sdk::ZoneCondition above(const std::string& mechanism, double threshold, const std::string& value = "velocity") {
    sdk::ZoneCondition c;
    c.mechanismId = mechanism;
    c.value = value;
    c.op = sdk::ConditionOp::Above;
    c.threshold = threshold;
    return c;
}

inline sdk::ZoneCondition within(const std::string& mechanism, double position, double tolerance,
                                 const std::string& value = "position") {
    sdk::ZoneCondition c;
    c.mechanismId = mechanism;
    c.value = value;
    c.op = sdk::ConditionOp::Within;
    c.threshold = position;
    c.tolerance = tolerance;
    return c;
}

// A zone of one of the plugin's kinds on the chassis with the kind's usual pose; `local` / `sizeM` /
// `activation` can be adjusted before Session::addZone.
inline sdk::ZoneConfig makeZone(const std::string& id, const char* kind, sdk::Value properties = sdk::Value::makeObject(),
                                std::vector<sdk::ZoneCondition> conditions = {}) {
    sdk::ZoneConfig z;
    z.id = id;
    z.kind = kind;
    z.properties = std::move(properties);
    z.activation.conditions = std::move(conditions);
    if (z.kind == plugins::game_2026::kPickupKind) {
        z.local.position = {0.45f, 0.f, 0.2f};
        z.sizeM = {0.30f, 0.60f, 0.40f};
    } else {
        z.local.position = {0.25f, 0.f, 0.55f};
    }
    return z;
}

// Forwards to a FakePhysicsWorld; can refuse bodies / meshes / shapes past a budget and records what
// the plugin asked for (shape descriptions, filter changes).
class ForwardingWorld final : public sdk::IPhysicsWorld {
public:
    bool refuseBodies = false;
    bool refuseMeshes = false;
    int shapeBudget = 1 << 30;
    int filterCalls = 0;
    int bodyAttempts = 0;
    std::uint64_t lastMask = 0;
    std::uint64_t lastCategory = 0;
    std::vector<sdk::ShapeDesc> shapeLog;
    sdk::testing::FakePhysicsWorld inner;

    sdk::BodyId createBody(const sdk::BodyDesc& d) override {
        ++bodyAttempts;
        return refuseBodies ? sdk::BodyId{} : inner.createBody(d);
    }
    void destroyBody(sdk::BodyId b) override { inner.destroyBody(b); }
    sdk::ShapeId addShape(sdk::BodyId b, const sdk::ShapeDesc& d) override {
        if (refuseMeshes && std::holds_alternative<sdk::MeshShape>(d.geometry)) return {};
        if (shapeBudget-- <= 0) return {};
        shapeLog.push_back(d);
        return inner.addShape(b, d);
    }
    void destroyShape(sdk::ShapeId s) override { inner.destroyShape(s); }
    void setMassProperties(sdk::BodyId b, const sdk::MassProperties& m) override { inner.setMassProperties(b, m); }
    void setShapeFilter(sdk::ShapeId s, std::uint64_t c, std::uint64_t m) override {
        ++filterCalls;
        lastMask = m;
        lastCategory = c;
        inner.setShapeFilter(s, c, m);
    }
    void setShapeMaterial(sdk::ShapeId s, const sdk::Material& m) override { inner.setShapeMaterial(s, m); }
    void step(float dt) override { inner.step(dt); }
    sdk::Pose3 pose(sdk::BodyId b) const override { return inner.pose(b); }
    sdk::Vec3 linearVelocity(sdk::BodyId b) const override { return inner.linearVelocity(b); }
    sdk::Vec3 angularVelocity(sdk::BodyId b) const override { return inner.angularVelocity(b); }
    sdk::Vec3 pointVelocity(sdk::BodyId b, const sdk::Vec3& p) const override { return inner.pointVelocity(b, p); }
    sdk::Vec3 worldPoint(sdk::BodyId b, const sdk::Vec3& p) const override { return inner.worldPoint(b, p); }
    void setPose(sdk::BodyId b, const sdk::Pose3& p) override { inner.setPose(b, p); }
    void setLinearVelocity(sdk::BodyId b, const sdk::Vec3& v) override { inner.setLinearVelocity(b, v); }
    void setAngularVelocity(sdk::BodyId b, const sdk::Vec3& w) override { inner.setAngularVelocity(b, w); }
    void setEnabled(sdk::BodyId b, bool e) override { inner.setEnabled(b, e); }
    void applyForce(sdk::BodyId b, const sdk::Vec3& f, const sdk::Vec3& p) override { inner.applyForce(b, f, p); }
    void applyForceToCenter(sdk::BodyId b, const sdk::Vec3& f) override { inner.applyForceToCenter(b, f); }
    void applyTorque(sdk::BodyId b, const sdk::Vec3& t) override { inner.applyTorque(b, t); }
    void applyImpulse(sdk::BodyId b, const sdk::Vec3& i, const sdk::Vec3& p) override { inner.applyImpulse(b, i, p); }
    sdk::RayHit rayCastClosest(const sdk::Vec3& o, const sdk::Vec3& d, float m, std::uint64_t mask) const override {
        return inner.rayCastClosest(o, d, m, mask);
    }
    void overlapSphere(const sdk::Vec3& c, float r, std::uint64_t mask, std::vector<sdk::ShapeId>& out) const override {
        inner.overlapSphere(c, r, mask, out);
    }
    void shapesOverlapping(sdk::ShapeId s, std::vector<sdk::ShapeId>& out) const override { inner.shapesOverlapping(s, out); }
    std::span<const sdk::SensorEvent> sensorEvents() const override { return inner.sensorEvents(); }
    std::span<const sdk::ContactEvent> contactEvents() const override { return inner.contactEvents(); }
    sdk::PhysicsStats stats() const override { return inner.stats(); }
    std::uint64_t stateHash() const override { return inner.stateHash(); }
    std::uint64_t shapeTag(sdk::ShapeId s) const override { return inner.shapeTag(s); }
    sdk::BodyId shapeBody(sdk::ShapeId s) const override { return inner.shapeBody(s); }
    bool isValid(sdk::ShapeId s) const override { return inner.isValid(s); }
    bool isValid(sdk::BodyId b) const override { return inner.isValid(b); }
};

class Session {
public:
    Session() {
        (void)plugins::game_2026::registerPlugin(registry_);
        module_ = registry_.list().front();
    }

    sdk::IGameModule& module() { return *module_; }

    // Section data of robot.json for game_2026 (edit before start()).
    sdk::Value& section() { return env.config.pluginSection("game_2026").data; }

    void addMechanism(const std::string& id, const std::string& type, sdk::Value properties = sdk::Value::makeObject(),
                      const sdk::Pose3& mount = {}) {
        sdk::MechanismConfig m;
        m.id = id;
        m.type = type;
        m.mount = mount;
        m.properties = std::move(properties);
        env.config.mechanisms.push_back(std::move(m));
    }
    ScriptedMechanism& addScripted(const std::string& id) {
        scripted_.push_back(std::make_unique<ScriptedMechanism>(id));
        return *scripted_.back();
    }
    void addZone(sdk::ZoneConfig zone) { env.config.zones.push_back(std::move(zone)); }

    // Registers templates, builds the field, the configured mechanisms and the game state (which builds its zones).
    void start() {
        if (extraTemplates) extraTemplates(factory);
        (void)module_->registerMechanisms(factory);
        fieldOwner = module_->createField(env.physics).value();
        sdk::BodyDesc chassisDesc;
        chassisDesc.type = sdk::BodyType::Kinematic;
        chassis = env.physics.createBody(chassisDesc);
        env.ctx.bindChassisBody(chassis);

        directory = std::make_unique<sdk::testing::FakeMechanismDirectory>(env.config, &factory);
        for (auto& s : scripted_) directory->add(s.get());
        for (const sdk::MechanismConfig& cfg : env.config.mechanisms) {
            if (!factory.contains(cfg.type)) continue;
            mechanisms.push_back(factory.create(cfg.type, sdk::MechanismBuildContext{cfg, env.ctx}).value());
            directory->add(mechanisms.back().get());
        }
        env.ctx.bindMechanisms(directory.get());

        stateOwner = module_->createGameState(env.ctx).value();
        env.ctx.bindGameState(stateOwner.get());
        game = dynamic_cast<plugins::game_2026::Game2026State*>(stateOwner.get());
        stateOwner->reset(env.ctx);
    }

    // One runtime tick: mechanisms, physics, game state, module hook.
    void tick() {
        const double dt = env.clock.fixedStepSeconds();
        for (auto& m : mechanisms) m->tick(env.ctx, dt);
        env.advance();
        stateOwner->step(env.ctx, dt);
        module_->onTick(env.ctx, dt);
    }
    void tick(int n) {
        for (int i = 0; i < n; ++i) tick();
    }

    void setChassis(const sdk::Vec3& position, float yawRad = 0.f) {
        env.physics.setPose(chassis, sdk::Pose3{position, sdk::Quat::fromYaw(yawRad)});
    }
    int fuelNear(const sdk::Vec3& c, float radius) const {
        int n = 0;
        game->pool().forEach([&](std::uint32_t, const sdk::Vec3& p) { n += sdk::length(p - c) <= radius ? 1 : 0; });
        return n;
    }

    sdk::testing::FakeSimEnvironment env;  // owns the physics world: must outlive everything below
    std::function<void(sdk::MechanismFactory&)> extraTemplates;  // e.g. core's generic mechanism templates
    sdk::MechanismFactory factory;
    std::unique_ptr<sdk::testing::FakeMechanismDirectory> directory;
    std::vector<std::unique_ptr<sdk::IMechanism>> mechanisms;
    sdk::BodyId chassis;
    plugins::game_2026::Game2026State* game = nullptr;
    std::unique_ptr<sdk::IField> fieldOwner;
    std::unique_ptr<sdk::IGameState> stateOwner;

private:
    sdk::GamePluginRegistry registry_;
    sdk::IGameModule* module_ = nullptr;
    std::vector<std::unique_ptr<ScriptedMechanism>> scripted_;
};

}  // namespace g26test
