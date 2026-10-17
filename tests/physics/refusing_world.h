#pragma once

#include "wpiemu/sdk/testing/fake_physics_world.h"

namespace sdk = wpiemu::sdk;

namespace physics_test {

class RefusingWorld final : public wpiemu::sdk::IPhysicsWorld {
public:
    int shapeBudget = 100;
    wpiemu::sdk::testing::FakePhysicsWorld inner;

    sdk::BodyId createBody(const sdk::BodyDesc& d) override { return inner.createBody(d); }
    void destroyBody(sdk::BodyId b) override { inner.destroyBody(b); }
    sdk::ShapeId addShape(sdk::BodyId b, const sdk::ShapeDesc& d) override {
        return shapeBudget-- > 0 ? inner.addShape(b, d) : sdk::ShapeId{};
    }
    void destroyShape(sdk::ShapeId s) override { inner.destroyShape(s); }
    void setMassProperties(sdk::BodyId b, const sdk::MassProperties& m) override {
        inner.setMassProperties(b, m);
    }
    void setShapeFilter(sdk::ShapeId s, std::uint64_t c, std::uint64_t m) override {
        inner.setShapeFilter(s, c, m);
    }
    void setShapeMaterial(sdk::ShapeId s, const sdk::Material& m) override {
        inner.setShapeMaterial(s, m);
    }
    void step(float dt) override { inner.step(dt); }
    sdk::Pose3 pose(sdk::BodyId b) const override { return inner.pose(b); }
    sdk::Vec3 linearVelocity(sdk::BodyId b) const override { return inner.linearVelocity(b); }
    sdk::Vec3 angularVelocity(sdk::BodyId b) const override { return inner.angularVelocity(b); }
    sdk::Vec3 pointVelocity(sdk::BodyId b, const sdk::Vec3& p) const override {
        return inner.pointVelocity(b, p);
    }
    sdk::Vec3 worldPoint(sdk::BodyId b, const sdk::Vec3& p) const override {
        return inner.worldPoint(b, p);
    }
    void setPose(sdk::BodyId b, const sdk::Pose3& p) override { inner.setPose(b, p); }
    void setLinearVelocity(sdk::BodyId b, const sdk::Vec3& v) override {
        inner.setLinearVelocity(b, v);
    }
    void setAngularVelocity(sdk::BodyId b, const sdk::Vec3& w) override {
        inner.setAngularVelocity(b, w);
    }
    void setEnabled(sdk::BodyId b, bool e) override { inner.setEnabled(b, e); }
    void applyForce(sdk::BodyId b, const sdk::Vec3& f, const sdk::Vec3& p) override {
        inner.applyForce(b, f, p);
    }
    void applyForceToCenter(sdk::BodyId b, const sdk::Vec3& f) override {
        inner.applyForceToCenter(b, f);
    }
    void applyTorque(sdk::BodyId b, const sdk::Vec3& t) override { inner.applyTorque(b, t); }
    void applyImpulse(sdk::BodyId b, const sdk::Vec3& i, const sdk::Vec3& p) override {
        inner.applyImpulse(b, i, p);
    }
    sdk::RayHit rayCastClosest(const sdk::Vec3& o, const sdk::Vec3& d, float m,
                               std::uint64_t mask) const override {
        return inner.rayCastClosest(o, d, m, mask);
    }
    void overlapSphere(const sdk::Vec3& c, float r, std::uint64_t mask,
                       std::vector<sdk::ShapeId>& out) const override {
        inner.overlapSphere(c, r, mask, out);
    }
    void shapesOverlapping(sdk::ShapeId s, std::vector<sdk::ShapeId>& out) const override {
        inner.shapesOverlapping(s, out);
    }
    std::span<const sdk::SensorEvent> sensorEvents() const override { return inner.sensorEvents(); }
    std::span<const sdk::ContactEvent> contactEvents() const override {
        return inner.contactEvents();
    }
    sdk::PhysicsStats stats() const override { return inner.stats(); }
    std::uint64_t stateHash() const override { return inner.stateHash(); }
    std::uint64_t shapeTag(sdk::ShapeId s) const override { return inner.shapeTag(s); }
    sdk::BodyId shapeBody(sdk::ShapeId s) const override { return inner.shapeBody(s); }
    bool isValid(sdk::ShapeId s) const override { return inner.isValid(s); }
    bool isValid(sdk::BodyId b) const override { return inner.isValid(b); }
};

}  // namespace physics_test
