#pragma once

// In-memory IPhysicsWorld for isolated tests of mechanisms, drivetrains, plugins and UI.
// NOT a physics engine: semi-implicit Euler for dynamic bodies, optional flat floor at z = 0,
// gravity, applied forces/torques, point-in-volume sensor events and a floor-only ray cast.
// Deterministic (stateHash works) so determinism plumbing can be tested without Box3D.

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <span>
#include <utility>
#include <vector>

#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::sdk::testing {

class FakePhysicsWorld final : public IPhysicsWorld {
public:
    struct AppliedForce {
        BodyId body;
        Vec3 force;
        Vec3 worldPoint;
        bool atPoint = false;
    };

    explicit FakePhysicsWorld(PhysicsConfig config = {}) : config_(config) {}

    // ---- test knobs -----------------------------------------------------------------------
    void setFloorEnabled(bool enabled) { floor_ = enabled; }
    int stepCount() const { return steps_; }
    const std::vector<AppliedForce>& pendingForces() const { return pending_; }
    const std::vector<AppliedForce>& lastStepForces() const { return lastStep_; }
    std::size_t shapeCount() const { return liveShapes_; }

    // ---- IPhysicsWorld ----------------------------------------------------------------------
    BodyId createBody(const BodyDesc& desc) override {
        Body b;
        b.desc = desc;
        b.pose = desc.pose;
        b.v = desc.linearVelocity;
        b.order = nextBody_;
        bodies_[nextBody_] = std::move(b);
        return BodyId{nextBody_++};
    }
    void destroyBody(BodyId id) override {
        auto it = bodies_.find(id.value);
        if (it == bodies_.end()) return;
        for (ShapeId s : it->second.shapes) {
            if (shapes_.erase(s.value) != 0) --liveShapes_;
            prevOverlaps_.erase(s.value);
        }
        bodies_.erase(it);
    }
    ShapeId addShape(BodyId body, const ShapeDesc& desc) override {
        Body* b = find(body);
        if (b == nullptr) return {};
        const ShapeId id{nextShape_++};
        shapes_[id.value] = Shape{desc, body};
        b->shapes.push_back(id);
        ++liveShapes_;
        if (desc.density > 0.f && b->desc.type == BodyType::Dynamic && !b->massSet) b->mass = std::max(b->mass, 1.f);
        return id;
    }
    void destroyShape(ShapeId id) override {
        auto it = shapes_.find(id.value);
        if (it == shapes_.end()) return;
        if (Body* b = find(it->second.body)) std::erase(b->shapes, id);
        shapes_.erase(it);
        prevOverlaps_.erase(id.value);
        --liveShapes_;
    }
    void setMassProperties(BodyId body, const MassProperties& mp) override {
        if (Body* b = find(body)) {
            b->mass = mp.massKg;
            b->com = mp.comLocal;
            b->inertia = mp.inertiaDiagLocal;
            b->massSet = true;
        }
    }
    void setShapeFilter(ShapeId id, std::uint64_t category, std::uint64_t mask) override {
        if (Shape* s = findShape(id)) { s->desc.category = category; s->desc.mask = mask; }
    }
    void setShapeMaterial(ShapeId id, const Material& material) override {
        if (Shape* s = findShape(id)) s->desc.material = material;
    }

    void step(float dt) override {
        lastStep_ = std::move(pending_);
        pending_.clear();
        sensorEvents_.clear();
        contactEvents_.clear();
        for (auto& [id, b] : bodies_) {
            if (b.desc.type == BodyType::Dynamic && b.enabled) {
                const float invM = b.mass > 0.f ? 1.f / b.mass : 0.f;
                b.v += (b.force * invM + config_.gravity) * dt;
                b.w += Vec3{b.torque.x / b.inertia.x, b.torque.y / b.inertia.y, b.torque.z / b.inertia.z} * dt;
            }
            if (b.desc.type != BodyType::Static && b.enabled) {
                b.pose.position += b.v * dt;
                const Quat dq{b.w.x * 0.5f * dt, b.w.y * 0.5f * dt, b.w.z * 0.5f * dt, 0.f};
                const Quat d = dq * b.pose.rotation;
                b.pose.rotation = Quat{b.pose.rotation.x + d.x, b.pose.rotation.y + d.y, b.pose.rotation.z + d.z,
                                       b.pose.rotation.w + d.w}.normalized();
                if (floor_ && b.pose.position.z < 0.f) {
                    b.pose.position.z = 0.f;
                    if (b.v.z < 0.f) b.v.z = 0.f;
                }
            }
            b.force = {};
            b.torque = {};
        }
        ++steps_;
        updateSensorEvents();
    }

    Pose3 pose(BodyId id) const override { const Body* b = find(id); return b ? b->pose : Pose3{}; }
    Vec3 linearVelocity(BodyId id) const override { const Body* b = find(id); return b ? b->v : Vec3{}; }
    Vec3 angularVelocity(BodyId id) const override { const Body* b = find(id); return b ? b->w : Vec3{}; }
    Vec3 pointVelocity(BodyId id, const Vec3& wp) const override {
        const Body* b = find(id);
        if (b == nullptr) return {};
        return b->v + cross(b->w, wp - (b->pose.position + b->pose.rotation.rotate(b->com)));
    }
    Vec3 worldPoint(BodyId id, const Vec3& local) const override {
        const Body* b = find(id);
        return b ? b->pose.transformPoint(local) : Vec3{};
    }
    void setPose(BodyId id, const Pose3& p) override { if (Body* b = find(id)) b->pose = p; }
    void setLinearVelocity(BodyId id, const Vec3& v) override { if (Body* b = find(id)) b->v = v; }
    void setAngularVelocity(BodyId id, const Vec3& w) override { if (Body* b = find(id)) b->w = w; }
    void setEnabled(BodyId id, bool enabled) override { if (Body* b = find(id)) b->enabled = enabled; }

    void applyForce(BodyId id, const Vec3& f, const Vec3& wp) override {
        Body* b = find(id);
        if (b == nullptr) return;
        b->force += f;
        b->torque += cross(wp - (b->pose.position + b->pose.rotation.rotate(b->com)), f);
        pending_.push_back({id, f, wp, true});
    }
    void applyForceToCenter(BodyId id, const Vec3& f) override {
        if (Body* b = find(id)) { b->force += f; pending_.push_back({id, f, {}, false}); }
    }
    void applyTorque(BodyId id, const Vec3& t) override { if (Body* b = find(id)) b->torque += t; }
    void applyImpulse(BodyId id, const Vec3& impulse, const Vec3& /*wp*/) override {
        if (Body* b = find(id); b != nullptr && b->mass > 0.f) b->v += impulse / b->mass;
    }

    RayHit rayCastClosest(const Vec3& origin, const Vec3& direction, float maxDistance,
                          std::uint64_t maskBits) const override {
        RayHit hit;
        const Vec3 d = normalized(direction);
        if (!floor_ || (maskBits & layer::Field) == 0 || d.z >= -1e-6f) return hit;
        const float t = -origin.z / d.z;
        if (t < 0.f || t > maxDistance) return hit;
        hit.hit = true;
        hit.point = origin + d * t;
        hit.normal = {0.f, 0.f, 1.f};
        hit.fraction = maxDistance > 0.f ? t / maxDistance : 0.f;
        return hit;
    }
    void overlapSphere(const Vec3& c, float r, std::uint64_t maskBits, std::vector<ShapeId>& out) const override {
        for (const auto& [id, s] : shapes_) {
            if ((s.desc.category & maskBits) == 0) continue;
            const Vec3 p = shapeCenter(s);
            if (length(p - c) <= r) out.push_back(ShapeId{id});
        }
    }
    void shapesOverlapping(ShapeId sensor, std::vector<ShapeId>& out) const override {
        const auto it = shapes_.find(sensor.value);
        if (it == shapes_.end() || !it->second.desc.sensor) return;
        for (const auto& [id, s] : shapes_)
            if (id != sensor.value && canVisit(it->second, s) && inside(it->second, shapeCenter(s)))
                out.push_back(ShapeId{id});
    }

    std::span<const SensorEvent> sensorEvents() const override { return sensorEvents_; }
    std::span<const ContactEvent> contactEvents() const override { return contactEvents_; }

    PhysicsStats stats() const override {
        PhysicsStats s;
        s.bodies = static_cast<int>(bodies_.size());
        s.awakeBodies = s.bodies;
        return s;
    }
    std::uint64_t stateHash() const override {
        std::uint64_t h = 1469598103934665603ull;
        auto mix = [&h](const void* p, std::size_t n) {
            const auto* bytes = static_cast<const unsigned char*>(p);
            for (std::size_t i = 0; i < n; ++i) { h ^= bytes[i]; h *= 1099511628211ull; }
        };
        for (const auto& [id, b] : bodies_) {
            mix(&b.pose, sizeof(Pose3));
            mix(&b.v, sizeof(Vec3));
            mix(&b.w, sizeof(Vec3));
        }
        return h;
    }
    std::uint64_t shapeTag(ShapeId id) const override { const Shape* s = findShape(id); return s ? s->desc.userTag : 0; }
    BodyId shapeBody(ShapeId id) const override { const Shape* s = findShape(id); return s ? s->body : BodyId{}; }
    bool isValid(ShapeId id) const override { return findShape(id) != nullptr; }
    bool isValid(BodyId id) const override { return find(id) != nullptr; }

private:
    struct Body {
        BodyDesc desc;
        Pose3 pose;
        Vec3 v, w, force, torque, com;
        Vec3 inertia{1.f, 1.f, 1.f};
        float mass = 1.f;
        bool massSet = false;
        bool enabled = true;
        std::uint64_t order = 0;
        std::vector<ShapeId> shapes;
    };
    struct Shape {
        ShapeDesc desc;
        BodyId body;
    };

    Body* find(BodyId id) { auto it = bodies_.find(id.value); return it == bodies_.end() ? nullptr : &it->second; }
    const Body* find(BodyId id) const { auto it = bodies_.find(id.value); return it == bodies_.end() ? nullptr : &it->second; }
    Shape* findShape(ShapeId id) { auto it = shapes_.find(id.value); return it == shapes_.end() ? nullptr : &it->second; }
    const Shape* findShape(ShapeId id) const { auto it = shapes_.find(id.value); return it == shapes_.end() ? nullptr : &it->second; }

    Pose3 shapePose(const Shape& s) const {
        const Body* b = find(s.body);
        return b ? b->pose * s.desc.local : s.desc.local;
    }
    Vec3 shapeCenter(const Shape& s) const { return shapePose(s).position; }

    static bool canVisit(const Shape& sensor, const Shape& visitor) {
        return !visitor.desc.sensor && visitor.desc.sensorEvents && sensor.desc.sensorEvents &&
               (sensor.desc.mask & visitor.desc.category) != 0 && (visitor.desc.mask & sensor.desc.category) != 0;
    }
    bool inside(const Shape& sensor, const Vec3& worldPoint) const {
        const Vec3 p = shapePose(sensor).inverse().transformPoint(worldPoint);
        if (const auto* box = std::get_if<BoxShape>(&sensor.desc.geometry))
            return std::fabs(p.x) <= box->halfExtents.x && std::fabs(p.y) <= box->halfExtents.y &&
                   std::fabs(p.z) <= box->halfExtents.z;
        if (const auto* sph = std::get_if<SphereShape>(&sensor.desc.geometry)) return length(p) <= sph->radius;
        if (const auto* cyl = std::get_if<CylinderShape>(&sensor.desc.geometry))
            return std::sqrt(p.x * p.x + p.y * p.y) <= cyl->radius && std::fabs(p.z) <= cyl->halfHeight;
        return false;
    }

    void updateSensorEvents() {
        for (const auto& [sid, sensor] : shapes_) {
            if (!sensor.desc.sensor) continue;
            std::set<std::uint64_t> now;
            for (const auto& [vid, visitor] : shapes_)
                if (vid != sid && canVisit(sensor, visitor) && inside(sensor, shapeCenter(visitor))) now.insert(vid);
            std::set<std::uint64_t>& prev = prevOverlaps_[sid];
            for (std::uint64_t v : now)
                if (!prev.contains(v))
                    sensorEvents_.push_back({ShapeId{sid}, ShapeId{v}, sensor.desc.userTag, shapes_.at(v).desc.userTag, true});
            for (std::uint64_t v : prev)
                if (!now.contains(v)) {
                    const auto vit = shapes_.find(v);
                    sensorEvents_.push_back({ShapeId{sid}, ShapeId{v}, sensor.desc.userTag,
                                             vit != shapes_.end() ? vit->second.desc.userTag : 0, false});
                }
            prev = std::move(now);
        }
    }

    PhysicsConfig config_;
    bool floor_ = true;
    int steps_ = 0;
    std::size_t liveShapes_ = 0;
    std::uint64_t nextBody_ = 1;
    std::uint64_t nextShape_ = 1;
    std::map<std::uint64_t, Body> bodies_;   // ordered by creation => deterministic hash
    std::map<std::uint64_t, Shape> shapes_;
    std::map<std::uint64_t, std::set<std::uint64_t>> prevOverlaps_;
    std::vector<AppliedForce> pending_;
    std::vector<AppliedForce> lastStep_;
    std::vector<SensorEvent> sensorEvents_;
    std::vector<ContactEvent> contactEvents_;
};

}  // namespace wpiemu::sdk::testing
