#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::sdk::testing {

class FakePhysicsWorld final : public IPhysicsWorld {
public:
    struct AppliedForce {
        BodyId body;
        Vec3 force;
        Vec3 point;
    };

    explicit FakePhysicsWorld(const PhysicsConfig& config = {}) : config_(config) {}

    BodyId createBody(const BodyDesc& desc) override {
        const BodyId id{nextBody_++};
        Body body;
        body.desc = desc;
        bodies_.emplace(id, body);
        return id;
    }

    void destroyBody(BodyId id) override {
        bodies_.erase(id);
        std::erase_if(shapes_, [id](const auto& entry) { return entry.second.body == id; });
        std::erase_if(pendingForces_, [id](const AppliedForce& force) { return force.body == id; });
        pendingTorques_.erase(id);
    }

    ShapeId addShape(BodyId body, const ShapeDesc& desc) override {
        if (!bodies_.contains(body)) return {};
        const ShapeId id{nextShape_++};
        shapes_.emplace(id, Shape{body, desc});
        return id;
    }
    void destroyShape(ShapeId id) override { shapes_.erase(id); }
    void setMassProperties(BodyId id, const MassProperties& mass) override {
        if (auto it = bodies_.find(id); it != bodies_.end()) it->second.mass = mass;
    }
    void setShapeFilter(ShapeId id, std::uint64_t category, std::uint64_t mask) override {
        if (auto it = shapes_.find(id); it != shapes_.end()) {
            it->second.desc.category = category;
            it->second.desc.mask = mask;
        }
    }
    void setShapeMaterial(ShapeId id, const Material& material) override {
        if (auto it = shapes_.find(id); it != shapes_.end()) it->second.desc.material = material;
    }

    void step(float dt) override {
        sensorEvents_.clear();
        contactEvents_.clear();
        lastStepForces_.swap(pendingForces_);
        pendingForces_.clear();
        ++stepCount_;

        std::map<BodyId, Vec3> netForce;
        std::map<BodyId, Vec3> netTorque;
        for (const AppliedForce& applied : lastStepForces_) {
            const auto it = bodies_.find(applied.body);
            if (it == bodies_.end() || it->second.desc.type != BodyType::Dynamic || !it->second.enabled) continue;
            netForce[applied.body] += applied.force;
            netTorque[applied.body] += cross(applied.point - it->second.desc.pose.position, applied.force);
        }
        for (const auto& [body, torque] : pendingTorques_) netTorque[body] += torque;
        pendingTorques_.clear();
        for (auto& [id, body] : bodies_) {
            if (body.desc.type != BodyType::Dynamic || !body.enabled) continue;
            const float mass = std::max(body.mass.massKg, 1e-6f);
            body.desc.linearVelocity += (config_.gravity + netForce[id] / mass) * dt;
            body.desc.pose.position += body.desc.linearVelocity * dt;
            body.angularVelocity += Vec3{netTorque[id].x / std::max(body.mass.inertiaDiagLocal.x, 1e-6f),
                                         netTorque[id].y / std::max(body.mass.inertiaDiagLocal.y, 1e-6f),
                                         netTorque[id].z / std::max(body.mass.inertiaDiagLocal.z, 1e-6f)} * dt;
            integrateRotation(body.desc.pose.rotation, body.angularVelocity, dt);
            if (floorEnabled_) {
                const float bottom = body.desc.pose.position.z - bodyRadius(id);
                if (bottom < 0.f) {
                    body.desc.pose.position.z -= bottom;
                    if (body.desc.linearVelocity.z < 0.f) body.desc.linearVelocity.z = 0.f;
                }
            }
        }

        std::set<Pair> overlaps;
        for (auto first = shapes_.begin(); first != shapes_.end(); ++first) {
            auto second = first;
            for (++second; second != shapes_.end(); ++second) {
                const Shape& a = first->second;
                const Shape& b = second->second;
                if (a.body == b.body || !filtersAllow(a.desc, b.desc) || !shapesOverlap(first->first, a, second->first, b)) continue;
                if (a.desc.sensor && a.desc.sensorEvents && b.desc.sensorEvents)
                    overlaps.emplace(first->first, second->first);
                else if (b.desc.sensor && b.desc.sensorEvents && a.desc.sensorEvents)
                    overlaps.emplace(second->first, first->first);
                if (!a.desc.sensor && !b.desc.sensor && a.desc.contactEvents && b.desc.contactEvents) {
                    ContactEvent event;
                    event.a = first->first;
                    event.b = second->first;
                    event.tagA = a.desc.userTag;
                    event.tagB = b.desc.userTag;
                    event.point = (shapePose(first->first, a).position + shapePose(second->first, b).position) * 0.5f;
                    event.normal = normalized(shapePose(second->first, b).position - shapePose(first->first, a).position);
                    contactEvents_.push_back(event);
                }
            }
        }

        for (const Pair& pair : overlaps) {
            if (!previousOverlaps_.contains(pair)) appendSensorEvent(pair, true);
        }
        for (const Pair& pair : previousOverlaps_) {
            if (!overlaps.contains(pair)) appendSensorEvent(pair, false);
        }
        previousOverlaps_ = std::move(overlaps);
    }

    Pose3 pose(BodyId id) const override {
        const auto it = bodies_.find(id);
        return it == bodies_.end() ? Pose3{} : it->second.desc.pose;
    }
    Vec3 linearVelocity(BodyId id) const override {
        const auto it = bodies_.find(id);
        return it == bodies_.end() ? Vec3{} : it->second.desc.linearVelocity;
    }
    Vec3 angularVelocity(BodyId id) const override {
        const auto it = bodies_.find(id);
        return it == bodies_.end() ? Vec3{} : it->second.angularVelocity;
    }
    Vec3 pointVelocity(BodyId id, const Vec3& point) const override {
        const auto it = bodies_.find(id);
        return it == bodies_.end() ? Vec3{} : it->second.desc.linearVelocity +
                                                   cross(it->second.angularVelocity, point - it->second.desc.pose.position);
    }
    Vec3 worldPoint(BodyId id, const Vec3& local) const override { return pose(id).transformPoint(local); }
    void setPose(BodyId id, const Pose3& value) override {
        if (auto it = bodies_.find(id); it != bodies_.end()) it->second.desc.pose = value;
    }
    void setLinearVelocity(BodyId id, const Vec3& value) override {
        if (auto it = bodies_.find(id); it != bodies_.end()) it->second.desc.linearVelocity = value;
    }
    void setAngularVelocity(BodyId id, const Vec3& value) override {
        if (auto it = bodies_.find(id); it != bodies_.end()) it->second.angularVelocity = value;
    }
    void setEnabled(BodyId id, bool enabled) override {
        if (auto it = bodies_.find(id); it != bodies_.end()) it->second.enabled = enabled;
    }

    void applyForce(BodyId id, const Vec3& force, const Vec3& point) override {
        if (isValid(id)) pendingForces_.push_back({id, force, point});
    }
    void applyForceToCenter(BodyId id, const Vec3& force) override {
        if (isValid(id)) applyForce(id, force, pose(id).position);
    }
    void applyTorque(BodyId id, const Vec3& torque) override {
        if (isValid(id)) pendingTorques_[id] += torque;
    }
    void applyImpulse(BodyId id, const Vec3& impulse, const Vec3& point) override {
        auto it = bodies_.find(id);
        if (it == bodies_.end() || it->second.desc.type != BodyType::Dynamic) return;
        it->second.desc.linearVelocity += impulse / std::max(it->second.mass.massKg, 1e-6f);
        it->second.angularVelocity += cross(point - it->second.desc.pose.position, impulse);
    }

    RayHit rayCastClosest(const Vec3& origin, const Vec3& direction, float maxDistance,
                          std::uint64_t mask) const override {
        RayHit best;
        best.fraction = 1.f;
        const float dirLength = length(direction);
        if (dirLength <= 1e-8f || maxDistance < 0.f) return best;
        const Vec3 unit = direction / dirLength;
        const float rayLength = maxDistance * dirLength;
        if (floorEnabled_ && (mask & layer::Field) != 0 && unit.z < -1e-8f && origin.z >= 0.f) {
            const float distance = -origin.z / unit.z;
            if (distance <= rayLength) {
                best.hit = true;
                best.point = origin + unit * distance;
                best.normal = {0.f, 0.f, 1.f};
                best.fraction = rayLength == 0.f ? 0.f : distance / rayLength;
            }
        }
        for (const auto& [id, shape] : shapes_) {
            if ((mask & shape.desc.category) == 0) continue;
            const Pose3 world = shapePose(id, shape);
            const float radius = shapeRadius(shape.desc);
            const Vec3 offset = origin - world.position;
            const float b = dot(offset, unit);
            const float c = dot(offset, offset) - radius * radius;
            const float discriminant = b * b - c;
            if (discriminant < 0.f) continue;
            const float distance = -b - std::sqrt(discriminant);
            if (distance < 0.f || distance > rayLength ||
                (best.hit && best.fraction <= distance / rayLength)) continue;
            best.hit = true;
            best.point = origin + unit * distance;
            best.normal = normalized(best.point - world.position);
            best.fraction = rayLength == 0.f ? 0.f : distance / rayLength;
            best.shape = id;
            best.userTag = shape.desc.userTag;
        }
        return best;
    }

    void overlapSphere(const Vec3& center, float radius, std::uint64_t mask, std::vector<ShapeId>& out) const override {
        for (const auto& [id, shape] : shapes_) {
            if ((mask & shape.desc.category) == 0) continue;
            const Pose3 world = shapePose(id, shape);
            if (lengthSquared(center - world.position) <= (radius + shapeRadius(shape.desc)) * (radius + shapeRadius(shape.desc)))
                out.push_back(id);
        }
    }
    void shapesOverlapping(ShapeId sensor, std::vector<ShapeId>& out) const override {
        const auto it = shapes_.find(sensor);
        if (it == shapes_.end()) return;
        for (const auto& [id, shape] : shapes_)
            if (id != sensor && shape.body != it->second.body && filtersAllow(it->second.desc, shape.desc) &&
                shapesOverlap(sensor, it->second, id, shape))
                out.push_back(id);
    }
    std::span<const SensorEvent> sensorEvents() const override { return sensorEvents_; }
    std::span<const ContactEvent> contactEvents() const override { return contactEvents_; }
    PhysicsStats stats() const override {
        PhysicsStats result;
        result.bodies = static_cast<int>(bodies_.size());
        for (const auto& [id, body] : bodies_)
            if (body.desc.type == BodyType::Dynamic && body.enabled &&
                (lengthSquared(body.desc.linearVelocity) > 1e-8f || lengthSquared(body.angularVelocity) > 1e-8f))
                ++result.awakeBodies;
        result.contacts = static_cast<int>(contactEvents_.size());
        return result;
    }
    std::uint64_t stateHash() const override {
        std::uint64_t hash = 14695981039346656037ull;
        const auto append = [&hash](float value) {
            std::uint32_t bits = std::bit_cast<std::uint32_t>(value);
            for (int i = 0; i < 4; ++i) {
                hash ^= static_cast<std::uint8_t>(bits & 0xffu);
                hash *= 1099511628211ull;
                bits >>= 8;
            }
        };
        for (const auto& [id, body] : bodies_) {
            hash ^= id.value;
            hash *= 1099511628211ull;
            for (float value : {body.desc.pose.position.x, body.desc.pose.position.y, body.desc.pose.position.z,
                                body.desc.pose.rotation.w, body.desc.pose.rotation.x, body.desc.pose.rotation.y,
                                body.desc.pose.rotation.z, body.desc.linearVelocity.x, body.desc.linearVelocity.y,
                                body.desc.linearVelocity.z, body.angularVelocity.x, body.angularVelocity.y,
                                body.angularVelocity.z})
                append(value);
        }
        return hash;
    }
    std::uint64_t shapeTag(ShapeId id) const override {
        const auto it = shapes_.find(id);
        return it == shapes_.end() ? 0 : it->second.desc.userTag;
    }
    BodyId shapeBody(ShapeId id) const override {
        const auto it = shapes_.find(id);
        return it == shapes_.end() ? BodyId{} : it->second.body;
    }
    bool isValid(ShapeId id) const override { return shapes_.contains(id); }
    bool isValid(BodyId id) const override { return bodies_.contains(id); }

    void setFloorEnabled(bool enabled) { floorEnabled_ = enabled; }
    std::uint64_t stepCount() const { return stepCount_; }
    std::size_t shapeCount() const { return shapes_.size(); }
    const std::vector<AppliedForce>& pendingForces() const { return pendingForces_; }
    const std::vector<AppliedForce>& lastStepForces() const { return lastStepForces_; }

private:
    struct Body {
        BodyDesc desc;
        MassProperties mass;
        Vec3 angularVelocity;
        bool enabled = true;
    };
    struct Shape {
        BodyId body;
        ShapeDesc desc;
    };
    using Pair = std::pair<ShapeId, ShapeId>;

    static float shapeRadius(const ShapeDesc& desc) {
        return std::visit([](const auto& shape) -> float {
            using T = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<T, SphereShape>) return shape.radius;
            else if constexpr (std::is_same_v<T, CylinderShape>) return std::sqrt(shape.radius * shape.radius + shape.halfHeight * shape.halfHeight);
            else if constexpr (std::is_same_v<T, BoxShape>) return length(shape.halfExtents);
            else {
                float radius = 0.f;
                for (std::size_t i = 0; i < shape.vertexCount; ++i)
                    radius = std::max(radius, length(Vec3{shape.xyz[i * 3], shape.xyz[i * 3 + 1], shape.xyz[i * 3 + 2]}));
                return radius;
            }
        }, desc.geometry);
    }
    static Vec3 normalized(const Vec3& value) {
        const float magnitude = length(value);
        return magnitude > 1e-8f ? value / magnitude : Vec3{0.f, 0.f, 1.f};
    }
    static float lengthSquared(const Vec3& value) { return dot(value, value); }
    static void integrateRotation(Quat& rotation, const Vec3& angularVelocity, float dt) {
        const float speed = length(angularVelocity);
        if (speed <= 1e-8f) return;
        const float half = speed * dt * 0.5f;
        const Vec3 axis = angularVelocity / speed;
        const Quat delta{axis.x * std::sin(half), axis.y * std::sin(half), axis.z * std::sin(half), std::cos(half)};
        rotation = (delta * rotation).normalized();
    }
    static bool filtersAllow(const ShapeDesc& a, const ShapeDesc& b) {
        return (a.mask & b.category) != 0 && (b.mask & a.category) != 0;
    }
    Pose3 shapePose(ShapeId, const Shape& shape) const { return pose(shape.body) * shape.desc.local; }
    bool shapesOverlap(ShapeId, const Shape& a, ShapeId, const Shape& b) const {
        const Pose3 poseA = shapePose({}, a);
        const Pose3 poseB = shapePose({}, b);
        if (const auto* box = std::get_if<BoxShape>(&a.desc.geometry)) {
            if (const auto* sphere = std::get_if<SphereShape>(&b.desc.geometry)) {
                const Vec3 local = poseA.inverse().transformPoint(poseB.position);
                const Vec3 closest{std::clamp(local.x, -box->halfExtents.x, box->halfExtents.x),
                                   std::clamp(local.y, -box->halfExtents.y, box->halfExtents.y),
                                   std::clamp(local.z, -box->halfExtents.z, box->halfExtents.z)};
                return lengthSquared(local - closest) <= sphere->radius * sphere->radius;
            }
        }
        if (const auto* box = std::get_if<BoxShape>(&b.desc.geometry)) {
            if (const auto* sphere = std::get_if<SphereShape>(&a.desc.geometry)) {
                const Vec3 local = poseB.inverse().transformPoint(poseA.position);
                const Vec3 closest{std::clamp(local.x, -box->halfExtents.x, box->halfExtents.x),
                                   std::clamp(local.y, -box->halfExtents.y, box->halfExtents.y),
                                   std::clamp(local.z, -box->halfExtents.z, box->halfExtents.z)};
                return lengthSquared(local - closest) <= sphere->radius * sphere->radius;
            }
        }
        const float reach = shapeRadius(a.desc) + shapeRadius(b.desc);
        return lengthSquared(poseA.position - poseB.position) <= reach * reach;
    }
    float bodyRadius(BodyId id) const {
        (void)id;
        return 0.f;
    }
    void appendSensorEvent(const Pair& pair, bool begin) {
        const auto sensor = shapes_.find(pair.first);
        const auto visitor = shapes_.find(pair.second);
        sensorEvents_.push_back({pair.first, pair.second,
                                 sensor == shapes_.end() ? 0 : sensor->second.desc.userTag,
                                 visitor == shapes_.end() ? 0 : visitor->second.desc.userTag, begin});
    }

    PhysicsConfig config_;
    std::uint64_t nextBody_ = 1;
    std::uint64_t nextShape_ = 1;
    std::uint64_t stepCount_ = 0;
    bool floorEnabled_ = true;
    std::map<BodyId, Body> bodies_;
    std::map<ShapeId, Shape> shapes_;
    std::vector<AppliedForce> pendingForces_;
    std::vector<AppliedForce> lastStepForces_;
    std::map<BodyId, Vec3> pendingTorques_;
    std::set<Pair> previousOverlaps_;
    std::vector<SensorEvent> sensorEvents_;
    std::vector<ContactEvent> contactEvents_;
};

}  // namespace wpiemu::sdk::testing
