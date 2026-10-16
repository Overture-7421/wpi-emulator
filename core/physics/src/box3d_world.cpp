// Box3D adapter for sdk::IPhysicsWorld (ADR 0003, ADR 0008). The only translation unit that sees Box3D.

#include "wpiemu/core/physics/box3d_world.h"

#include <box3d/box3d.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "wpiemu/core/physics/joint_host.h"

namespace wpiemu::core::physics {
namespace {

using namespace sdk;

constexpr int kCylinderSides = 24;
constexpr float kPi = 3.14159265358979323846f;

bool finite(float v) { return std::isfinite(v); }
bool finite(const Vec3& v) { return finite(v.x) && finite(v.y) && finite(v.z); }
bool finite(const Quat& q) { return finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w); }
bool finite(const Pose3& p) { return finite(p.position) && finite(p.rotation); }

b3Vec3 toB3(const Vec3& v) { return {v.x, v.y, v.z}; }
b3Quat toB3(const Quat& q) { return {{q.x, q.y, q.z}, q.w}; }
Vec3 fromB3(const b3Vec3& v) { return {v.x, v.y, v.z}; }
Quat fromB3(const b3Quat& q) { return {q.v.x, q.v.y, q.v.z, q.s}; }
b3Transform toB3(const Pose3& p) { return {toB3(p.position), toB3(p.rotation.normalized())}; }

// Shortest-arc rotation taking +X onto `axis` (unit length).
Quat rotationFromX(const Vec3& axis) {
    const float d = axis.x;
    if (d < -0.999999f) return Quat::fromAxisAngle({0.f, 0.f, 1.f}, kPi);
    const Vec3 c{0.f, -axis.z, axis.y};  // (1,0,0) x axis
    return Quat{c.x, c.y, c.z, 1.f + d}.normalized();
}

class Fnv1a {
public:
    void mix(const void* data, std::size_t size) {
        const auto* bytes = static_cast<const unsigned char*>(data);
        for (std::size_t i = 0; i < size; ++i) {
            hash_ ^= bytes[i];
            hash_ *= 1099511628211ull;
        }
    }
    void mix(float v) { mix(&v, sizeof v); }
    void mix(std::uint64_t v) { mix(&v, sizeof v); }
    std::uint64_t value() const { return hash_; }

private:
    std::uint64_t hash_ = 1469598103934665603ull;
};

class Box3dWorld final : public IPhysicsWorld, public IPhysicsJointHost {
public:
    explicit Box3dWorld(const PhysicsConfig& cfg) : cfg_(cfg) {
        b3WorldDef def = b3DefaultWorldDef();
        def.gravity = toB3(cfg.gravity);
        def.enableSleep = cfg.sleeping;
        def.workerCount = static_cast<std::uint32_t>(std::max(1, cfg.workerCount));
        world_ = b3CreateWorld(&def);
    }

    ~Box3dWorld() override {
        if (b3World_IsValid(world_)) b3DestroyWorld(world_);
        for (auto& [id, rec] : shapes_)
            if (rec.mesh != nullptr) b3DestroyMesh(rec.mesh);
    }

    Box3dWorld(const Box3dWorld&) = delete;
    Box3dWorld& operator=(const Box3dWorld&) = delete;

    bool ok() const { return b3World_IsValid(world_); }

    // ---- lifecycle ----------------------------------------------------------------------
    BodyId createBody(const BodyDesc& d) override {
        if (!finite(d.pose) || !finite(d.linearVelocity)) return {};
        b3BodyDef def = b3DefaultBodyDef();
        def.type = d.type == BodyType::Dynamic ? b3_dynamicBody
                   : d.type == BodyType::Kinematic ? b3_kinematicBody
                                                    : b3_staticBody;
        def.position = toB3(d.pose.position);
        def.rotation = toB3(d.pose.rotation.normalized());
        def.linearVelocity = toB3(d.linearVelocity);
        def.enableSleep = d.allowSleep && cfg_.sleeping;
        def.isBullet = d.bullet;
        const b3BodyId id = b3CreateBody(world_, &def);
        if (B3_IS_NULL(id)) return {};
        const std::uint64_t key = b3StoreBodyId(id);
        BodyRec rec;
        rec.id = id;
        rec.seq = nextSeq_++;
        bodyOrder_.emplace(rec.seq, key);
        bodies_.emplace(key, std::move(rec));
        return BodyId{key};
    }

    void destroyBody(BodyId body) override {
        auto it = bodies_.find(body.value);
        if (it == bodies_.end()) return;
        // Joints first: they hold references to the body and own an internal anchor body.
        for (auto jit = joints_.begin(); jit != joints_.end();) {
            if (jit->second.body == body.value) {
                destroyJointRecord(jit->second);
                jit = joints_.erase(jit);
            } else {
                ++jit;
            }
        }
        BodyRec& rec = it->second;
        std::vector<std::uint64_t> shapeKeys = rec.shapes;
        b3DestroyBody(rec.id);
        for (std::uint64_t key : shapeKeys) releaseShapeRecord(key);
        bodyOrder_.erase(rec.seq);
        bodies_.erase(it);
    }

    ShapeId addShape(BodyId body, const ShapeDesc& d) override {
        BodyRec* rec = findBody(body);
        if (rec == nullptr || !finite(d.local) || !finite(d.material.friction) ||
            !finite(d.material.restitution) || !finite(d.material.rollingResistance) || !finite(d.density))
            return {};

        b3ShapeDef sd = b3DefaultShapeDef();
        sd.density = std::max(0.f, d.density);
        sd.baseMaterial.friction = d.material.friction;
        sd.baseMaterial.restitution = d.material.restitution;
        sd.baseMaterial.rollingResistance = d.material.rollingResistance;
        sd.filter.categoryBits = d.category;
        sd.filter.maskBits = d.mask;
        sd.isSensor = d.sensor;
        sd.enableSensorEvents = d.sensorEvents;
        sd.enableContactEvents = d.contactEvents;
        sd.enableHitEvents = d.contactEvents;
        // An explicit mass override is authoritative; zero-density shapes would otherwise reset it.
        sd.updateBodyMass = !rec->massOverride.has_value();

        const b3Transform xf = toB3(d.local);
        b3ShapeId created = b3_nullShapeId;
        b3MeshData* mesh = nullptr;

        if (const auto* box = std::get_if<BoxShape>(&d.geometry)) {
            if (!(box->halfExtents.x > 0.f && box->halfExtents.y > 0.f && box->halfExtents.z > 0.f) ||
                !finite(box->halfExtents))
                return {};
            const b3BoxHull hull = b3MakeTransformedBoxHull(box->halfExtents.x, box->halfExtents.y,
                                                             box->halfExtents.z, xf);
            created = b3CreateHullShape(rec->id, &sd, &hull.base);
        } else if (const auto* sph = std::get_if<SphereShape>(&d.geometry)) {
            if (!(sph->radius > 0.f) || !finite(sph->radius)) return {};
            const b3Sphere sphere{toB3(d.local.position), sph->radius};
            created = b3CreateSphereShape(rec->id, &sd, &sphere);
        } else if (const auto* cyl = std::get_if<CylinderShape>(&d.geometry)) {
            if (!(cyl->radius > 0.f && cyl->halfHeight > 0.f) || !finite(cyl->radius) || !finite(cyl->halfHeight))
                return {};
            // Box3D cylinders have their axis on Y; rotate it onto the SDK's local Z.
            b3HullData* hull = b3CreateCylinder(2.f * cyl->halfHeight, cyl->radius, -cyl->halfHeight, kCylinderSides);
            if (hull == nullptr) return {};
            const Pose3 axisFix{{}, Quat::fromAxisAngle({1.f, 0.f, 0.f}, 0.5f * kPi)};
            created = b3CreateTransformedHullShape(rec->id, &sd, hull, toB3(d.local * axisFix), {1.f, 1.f, 1.f});
            b3DestroyHull(hull);
        } else if (const auto* m = std::get_if<MeshShape>(&d.geometry)) {
            if (b3Body_GetType(rec->id) != b3_staticBody || m->xyz == nullptr || m->triangles == nullptr ||
                m->vertexCount < 3 || m->triangleCount < 1)
                return {};
            std::vector<b3Vec3> verts(m->vertexCount);
            for (std::size_t i = 0; i < m->vertexCount; ++i) {
                const Vec3 v{m->xyz[3 * i], m->xyz[3 * i + 1], m->xyz[3 * i + 2]};
                if (!finite(v)) return {};
                verts[i] = toB3(d.local.transformPoint(v));
            }
            std::vector<std::int32_t> idx(m->triangleCount * 3);
            for (std::size_t i = 0; i < idx.size(); ++i) {
                if (m->triangles[i] >= m->vertexCount) return {};
                idx[i] = static_cast<std::int32_t>(m->triangles[i]);
            }
            b3MeshDef md{};
            md.vertices = verts.data();
            md.indices = idx.data();
            md.vertexCount = static_cast<int>(verts.size());
            md.triangleCount = static_cast<int>(m->triangleCount);
            md.identifyEdges = true;
            mesh = b3CreateMesh(&md, nullptr, 0);
            if (mesh == nullptr) return {};
            created = b3CreateMeshShape(rec->id, &sd, mesh, {1.f, 1.f, 1.f});
            if (B3_IS_NULL(created)) {
                b3DestroyMesh(mesh);
                return {};
            }
        } else {
            return {};
        }
        if (B3_IS_NULL(created)) return {};

        const std::uint64_t key = b3StoreShapeId(created);
        ShapeRec srec;
        srec.id = created;
        srec.body = body.value;
        srec.tag = d.userTag;
        srec.mesh = mesh;
        shapes_.emplace(key, srec);
        rec->shapes.push_back(key);
        reapplyMassOverride(*rec);
        return ShapeId{key};
    }

    void destroyShape(ShapeId shape) override {
        auto it = shapes_.find(shape.value);
        if (it == shapes_.end()) return;
        BodyRec* rec = findBody(BodyId{it->second.body});
        b3DestroyShape(it->second.id, rec == nullptr || !rec->massOverride.has_value());
        if (rec != nullptr) {
            std::erase(rec->shapes, shape.value);
            reapplyMassOverride(*rec);
        }
        releaseShapeRecord(shape.value);
    }

    void setMassProperties(BodyId body, const MassProperties& mp) override {
        BodyRec* rec = findBody(body);
        if (rec == nullptr || !(mp.massKg > 0.f) || !finite(mp.massKg) || !finite(mp.comLocal) ||
            !finite(mp.inertiaDiagLocal) || !(mp.inertiaDiagLocal.x > 0.f && mp.inertiaDiagLocal.y > 0.f &&
                                               mp.inertiaDiagLocal.z > 0.f))
            return;
        rec->massOverride = mp;
        applyMass(*rec);
    }

    void setShapeFilter(ShapeId shape, std::uint64_t category, std::uint64_t mask) override {
        const ShapeRec* rec = findShape(shape);
        if (rec == nullptr) return;
        b3Filter f = b3Shape_GetFilter(rec->id);
        f.categoryBits = category;
        f.maskBits = mask;
        b3Shape_SetFilter(rec->id, f, true);
    }

    void setShapeMaterial(ShapeId shape, const Material& m) override {
        const ShapeRec* rec = findShape(shape);
        if (rec == nullptr || !finite(m.friction) || !finite(m.restitution) || !finite(m.rollingResistance)) return;
        b3SurfaceMaterial sm = b3Shape_GetSurfaceMaterial(rec->id);
        sm.friction = m.friction;
        sm.restitution = m.restitution;
        sm.rollingResistance = m.rollingResistance;
        b3Shape_SetSurfaceMaterial(rec->id, sm);
    }

    // ---- stepping -----------------------------------------------------------------------
    void step(float dt) override {
        if (!(dt > 0.f) || !finite(dt)) return;
        const auto t0 = std::chrono::steady_clock::now();
        b3World_Step(world_, dt, std::max(1, cfg_.subSteps));
        const auto t1 = std::chrono::steady_clock::now();
        lastStepMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
        collectEvents();
    }

    // ---- state --------------------------------------------------------------------------
    Pose3 pose(BodyId body) const override {
        const BodyRec* rec = findBody(body);
        if (rec == nullptr) return {};
        const b3WorldTransform t = b3Body_GetTransform(rec->id);
        return {fromB3(t.p), fromB3(t.q)};
    }
    Vec3 linearVelocity(BodyId body) const override {
        const BodyRec* rec = findBody(body);
        return rec ? fromB3(b3Body_GetLinearVelocity(rec->id)) : Vec3{};
    }
    Vec3 angularVelocity(BodyId body) const override {
        const BodyRec* rec = findBody(body);
        return rec ? fromB3(b3Body_GetAngularVelocity(rec->id)) : Vec3{};
    }
    Vec3 pointVelocity(BodyId body, const Vec3& worldPoint) const override {
        const BodyRec* rec = findBody(body);
        return rec ? fromB3(b3Body_GetWorldPointVelocity(rec->id, toB3(worldPoint))) : Vec3{};
    }
    Vec3 worldPoint(BodyId body, const Vec3& local) const override {
        const BodyRec* rec = findBody(body);
        return rec ? fromB3(b3Body_GetWorldPoint(rec->id, toB3(local))) : Vec3{};
    }
    void setPose(BodyId body, const Pose3& p) override {
        BodyRec* rec = findBody(body);
        if (rec == nullptr || !finite(p)) return;
        b3Body_SetTransform(rec->id, toB3(p.position), toB3(p.rotation.normalized()));
    }
    void setLinearVelocity(BodyId body, const Vec3& v) override {
        const BodyRec* rec = findBody(body);
        if (rec != nullptr && finite(v)) b3Body_SetLinearVelocity(rec->id, toB3(v));
    }
    void setAngularVelocity(BodyId body, const Vec3& w) override {
        const BodyRec* rec = findBody(body);
        if (rec != nullptr && finite(w)) b3Body_SetAngularVelocity(rec->id, toB3(w));
    }
    void setEnabled(BodyId body, bool enabled) override {
        const BodyRec* rec = findBody(body);
        if (rec == nullptr || b3Body_IsEnabled(rec->id) == enabled) return;
        if (enabled) b3Body_Enable(rec->id);
        else b3Body_Disable(rec->id);
    }

    // ---- forces -------------------------------------------------------------------------
    void applyForce(BodyId body, const Vec3& force, const Vec3& worldPoint) override {
        const BodyRec* rec = findBody(body);
        if (rec != nullptr && finite(force) && finite(worldPoint))
            b3Body_ApplyForce(rec->id, toB3(force), toB3(worldPoint), true);
    }
    void applyForceToCenter(BodyId body, const Vec3& force) override {
        const BodyRec* rec = findBody(body);
        if (rec != nullptr && finite(force)) b3Body_ApplyForceToCenter(rec->id, toB3(force), true);
    }
    void applyTorque(BodyId body, const Vec3& torque) override {
        const BodyRec* rec = findBody(body);
        if (rec != nullptr && finite(torque)) b3Body_ApplyTorque(rec->id, toB3(torque), true);
    }
    void applyImpulse(BodyId body, const Vec3& impulse, const Vec3& worldPoint) override {
        const BodyRec* rec = findBody(body);
        if (rec != nullptr && finite(impulse) && finite(worldPoint))
            b3Body_ApplyLinearImpulse(rec->id, toB3(impulse), toB3(worldPoint), true);
    }

    // ---- queries ------------------------------------------------------------------------
    RayHit rayCastClosest(const Vec3& origin, const Vec3& direction, float maxDistance,
                          std::uint64_t maskBits) const override {
        RayHit out;
        const Vec3 dir = normalized(direction);
        if (!finite(origin) || !finite(direction) || !(maxDistance > 0.f) || !finite(maxDistance) ||
            dot(dir, dir) < 0.5f)
            return out;
        b3QueryFilter filter = b3DefaultQueryFilter();
        filter.categoryBits = layer::All;
        filter.maskBits = maskBits;
        const b3RayResult r = b3World_CastRayClosest(world_, toB3(origin), toB3(dir * maxDistance), filter);
        if (!r.hit) return out;
        out.hit = true;
        out.point = fromB3(r.point);
        out.normal = fromB3(r.normal);
        out.fraction = r.fraction;
        const std::uint64_t key = b3StoreShapeId(r.shapeId);
        out.shape = ShapeId{key};
        if (const ShapeRec* rec = findShapeKey(key)) out.userTag = rec->tag;
        return out;
    }

    void overlapSphere(const Vec3& center, float radius, std::uint64_t maskBits,
                       std::vector<ShapeId>& out) const override {
        if (!finite(center) || !(radius > 0.f) || !finite(radius)) return;
        const b3Vec3 point{0.f, 0.f, 0.f};
        const b3ShapeProxy proxy{&point, 1, radius};
        b3QueryFilter filter = b3DefaultQueryFilter();
        filter.categoryBits = layer::All;
        filter.maskBits = maskBits;
        b3World_OverlapShape(world_, toB3(center), &proxy, filter,
                             [](b3ShapeId shape, void* ctx) {
                                 static_cast<std::vector<ShapeId>*>(ctx)->push_back(ShapeId{b3StoreShapeId(shape)});
                                 return true;
                             },
                             &out);
    }

    void shapesOverlapping(ShapeId sensor, std::vector<ShapeId>& out) const override {
        const ShapeRec* rec = findShape(sensor);
        if (rec == nullptr || !b3Shape_IsSensor(rec->id)) return;
        const int capacity = b3Shape_GetSensorCapacity(rec->id);
        if (capacity <= 0) return;
        std::vector<b3ShapeId> visitors(static_cast<std::size_t>(capacity));
        const int count = b3Shape_GetSensorData(rec->id, visitors.data(), capacity);
        for (int i = 0; i < count; ++i)
            if (b3Shape_IsValid(visitors[static_cast<std::size_t>(i)]))
                out.push_back(ShapeId{b3StoreShapeId(visitors[static_cast<std::size_t>(i)])});
    }

    std::span<const SensorEvent> sensorEvents() const override { return sensorEvents_; }
    std::span<const ContactEvent> contactEvents() const override { return contactEvents_; }

    // ---- diagnostics --------------------------------------------------------------------
    PhysicsStats stats() const override {
        PhysicsStats s;
        s.bodies = static_cast<int>(bodies_.size());
        s.awakeBodies = b3World_GetAwakeBodyCount(world_);
        s.contacts = b3World_GetCounters(world_).contactCount;
        s.lastStepMs = lastStepMs_;
        return s;
    }

    std::uint64_t stateHash() const override {
        Fnv1a h;
        h.mix(static_cast<std::uint64_t>(bodyOrder_.size()));
        for (const auto& [seq, key] : bodyOrder_) {
            const BodyRec& rec = bodies_.at(key);
            const b3WorldTransform t = b3Body_GetTransform(rec.id);
            const b3Vec3 v = b3Body_GetLinearVelocity(rec.id);
            const b3Vec3 w = b3Body_GetAngularVelocity(rec.id);
            for (float f : {t.p.x, t.p.y, t.p.z, t.q.v.x, t.q.v.y, t.q.v.z, t.q.s, v.x, v.y, v.z, w.x, w.y, w.z})
                h.mix(f == 0.f ? 0.f : f);  // fold -0 into +0
        }
        return h.value();
    }

    std::uint64_t shapeTag(ShapeId shape) const override {
        const ShapeRec* rec = findShape(shape);
        return rec ? rec->tag : 0;
    }
    BodyId shapeBody(ShapeId shape) const override {
        const ShapeRec* rec = findShape(shape);
        return rec ? BodyId{rec->body} : BodyId{};
    }
    bool isValid(ShapeId shape) const override { return findShape(shape) != nullptr; }
    bool isValid(BodyId body) const override { return findBody(body) != nullptr; }

    // ---- IPhysicsJointHost --------------------------------------------------------------
    JointId createAnchoredJoint(const AnchoredJointDesc& d) override {
        BodyRec* rec = findBody(d.body);
        if (rec == nullptr || b3Body_GetType(rec->id) != b3_dynamicBody || !finite(d.bodyLocalPoint) ||
            !finite(d.fieldPoint) || !finite(d.axisWorld))
            return {};

        b3BodyDef anchorDef = b3DefaultBodyDef();  // static
        anchorDef.position = toB3(d.fieldPoint);
        const b3BodyId anchor = b3CreateBody(world_, &anchorDef);
        if (B3_IS_NULL(anchor)) return {};

        const Pose3 bodyPose = pose(d.body);
        const Vec3 attachWorld = bodyPose.transformPoint(d.bodyLocalPoint);
        b3Transform frameA = b3Transform_identity;
        b3Transform frameB = b3Transform_identity;
        frameB.p = toB3(d.bodyLocalPoint);

        JointRec jr;
        jr.kind = d.kind;
        jr.body = d.body.value;
        jr.anchor = anchor;
        jr.fieldPoint = d.fieldPoint;

        b3JointId joint = b3_nullJointId;
        switch (d.kind) {
            case JointKind::Distance: {
                b3DistanceJointDef def = b3DefaultDistanceJointDef();
                def.base.bodyIdA = anchor;
                def.base.bodyIdB = rec->id;
                def.base.localFrameA = frameA;
                def.base.localFrameB = frameB;
                const float current = length(attachWorld - d.fieldPoint);
                def.length = d.lengthM >= 0.f ? d.lengthM : current;
                def.enableSpring = false;
                joint = b3CreateDistanceJoint(world_, &def);
                break;
            }
            case JointKind::Prismatic: {
                const float axisLen = length(d.axisWorld);
                if (!(axisLen > 1e-6f)) break;
                const Vec3 axis = d.axisWorld / axisLen;
                const Quat qa = rotationFromX(axis);
                frameA.q = toB3(qa);
                frameB.q = toB3((bodyPose.rotation.conjugate() * qa).normalized());
                b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
                def.base.bodyIdA = anchor;
                def.base.bodyIdB = rec->id;
                def.base.localFrameA = frameA;
                def.base.localFrameB = frameB;
                const bool hasMax = std::isfinite(d.maxTravelM);
                const bool hasMin = std::isfinite(d.minTravelM);
                if (hasMin || hasMax) {
                    def.enableLimit = true;
                    def.lowerTranslation = hasMin ? d.minTravelM : -1.0e6f;
                    def.upperTranslation = hasMax ? d.maxTravelM : 1.0e6f;
                    if (def.upperTranslation < def.lowerTranslation) std::swap(def.upperTranslation, def.lowerTranslation);
                    jr.lowerTravel = def.lowerTranslation;
                    jr.upperTravel = def.upperTranslation;
                }
                jr.frameRot = qa;
                joint = b3CreatePrismaticJoint(world_, &def);
                break;
            }
            case JointKind::Spherical: {
                b3SphericalJointDef def = b3DefaultSphericalJointDef();
                def.base.bodyIdA = anchor;
                def.base.bodyIdB = rec->id;
                def.base.localFrameA = frameA;
                def.base.localFrameB = frameB;
                joint = b3CreateSphericalJoint(world_, &def);
                break;
            }
        }
        if (B3_IS_NULL(joint)) {
            b3DestroyBody(anchor);
            return {};
        }
        jr.joint = joint;
        const std::uint64_t key = nextJoint_++;
        joints_.emplace(key, jr);
        return JointId{key};
    }

    void destroyJoint(JointId joint) override {
        auto it = joints_.find(joint.value);
        if (it == joints_.end()) return;
        destroyJointRecord(it->second);
        joints_.erase(it);
    }

    JointLoad jointLoad(JointId joint) const override {
        JointLoad out;
        const auto it = joints_.find(joint.value);
        if (it == joints_.end() || !b3Joint_IsValid(it->second.joint)) return out;
        const JointRec& jr = it->second;
        const BodyRec* rec = findBody(BodyId{jr.body});
        if (rec == nullptr) return out;
        // Box3D reports the constraint wrench on body B (= our body) in the world frame.
        out.valid = true;
        out.force = fromB3(b3Joint_GetConstraintForce(jr.joint));
        if (jr.kind == JointKind::Prismatic) {
            // b3GetPrismaticJointForce (v0.1.0) returns (perp.x, perp.y, axial) in joint space although the slide
            // axis is the joint X axis, and adds the upper-limit impulse with the wrong sign. Undo both.
            const Vec3 local = jr.frameRot.conjugate().rotate(out.force);
            Vec3 axisFirst{local.z, local.x, local.y};
            const float travel = b3PrismaticJoint_GetTranslation(jr.joint);
            if (travel > 0.5f * (jr.lowerTravel + jr.upperTravel)) axisFirst.x = -axisFirst.x;
            out.force = jr.frameRot.rotate(axisFirst);
        }
        const Vec3 com = fromB3(b3Body_GetWorldCenterOfMass(rec->id));
        const Vec3 attach = fromB3(b3Joint_GetLocalFrameB(jr.joint).p);
        const Vec3 attachWorld = fromB3(b3Body_GetWorldPoint(rec->id, toB3(attach)));
        out.torque = fromB3(b3Joint_GetConstraintTorque(jr.joint)) + cross(attachWorld - com, out.force);
        switch (jr.kind) {
            case JointKind::Distance: out.travelM = b3DistanceJoint_GetCurrentLength(jr.joint); break;
            case JointKind::Prismatic: out.travelM = b3PrismaticJoint_GetTranslation(jr.joint); break;
            case JointKind::Spherical: out.travelM = length(attachWorld - jr.fieldPoint); break;
        }
        return out;
    }

private:
    struct BodyRec {
        b3BodyId id = b3_nullBodyId;
        std::uint64_t seq = 0;
        std::vector<std::uint64_t> shapes;
        std::optional<MassProperties> massOverride;
        bool inertiaRefreshed = false;
    };
    struct ShapeRec {
        b3ShapeId id = b3_nullShapeId;
        std::uint64_t body = 0;
        std::uint64_t tag = 0;
        b3MeshData* mesh = nullptr;
    };
    struct JointRec {
        JointKind kind = JointKind::Distance;
        std::uint64_t body = 0;
        b3BodyId anchor = b3_nullBodyId;
        b3JointId joint = b3_nullJointId;
        Vec3 fieldPoint;
        Quat frameRot;          // Prismatic: world rotation of the joint frame (anchor body has identity rotation)
        float lowerTravel = 0.f;
        float upperTravel = 0.f;
    };

    BodyRec* findBody(BodyId id) {
        const auto it = bodies_.find(id.value);
        return it == bodies_.end() ? nullptr : &it->second;
    }
    const BodyRec* findBody(BodyId id) const {
        const auto it = bodies_.find(id.value);
        return it == bodies_.end() ? nullptr : &it->second;
    }
    const ShapeRec* findShape(ShapeId id) const { return findShapeKey(id.value); }
    const ShapeRec* findShapeKey(std::uint64_t key) const {
        const auto it = shapes_.find(key);
        return it == shapes_.end() ? nullptr : &it->second;
    }

    void releaseShapeRecord(std::uint64_t key) {
        const auto it = shapes_.find(key);
        if (it == shapes_.end()) return;
        if (it->second.mesh != nullptr) b3DestroyMesh(it->second.mesh);
        shapes_.erase(it);
    }

    void destroyJointRecord(JointRec& jr) {
        if (b3Joint_IsValid(jr.joint)) b3DestroyJoint(jr.joint, true);
        if (b3Body_IsValid(jr.anchor)) b3DestroyBody(jr.anchor);
    }

    static b3MassData massToB3(const MassProperties& mp) {
        b3MassData md{};
        md.mass = mp.massKg;
        md.center = {mp.comLocal.x, mp.comLocal.y, mp.comLocal.z};
        md.inertia = {{mp.inertiaDiagLocal.x, 0.f, 0.f}, {0.f, mp.inertiaDiagLocal.y, 0.f},
                      {0.f, 0.f, mp.inertiaDiagLocal.z}};
        return md;
    }

    void applyMass(BodyRec& rec) {
        b3Body_SetMassData(rec.id, massToB3(*rec.massOverride));
        if (!rec.inertiaRefreshed) {
            // b3Body_SetMassData does not refresh the cached world-space inverse inertia, which stays zero for a
            // body created without density; re-posing the body once (at setup, no contacts yet) fixes that.
            const b3WorldTransform t = b3Body_GetTransform(rec.id);
            b3Body_SetTransform(rec.id, t.p, t.q);
            rec.inertiaRefreshed = true;
        }
    }
    void reapplyMassOverride(BodyRec& rec) {
        if (rec.massOverride.has_value()) applyMass(rec);
    }

    void collectEvents() {
        sensorEvents_.clear();
        contactEvents_.clear();

        const b3SensorEvents se = b3World_GetSensorEvents(world_);
        auto addSensor = [this](b3ShapeId sensor, b3ShapeId visitor, bool begin) {
            if (!b3Shape_IsValid(sensor) || !b3Shape_IsValid(visitor)) return;  // destroyed since the event
            const std::uint64_t sk = b3StoreShapeId(sensor);
            const std::uint64_t vk = b3StoreShapeId(visitor);
            const ShapeRec* s = findShapeKey(sk);
            const ShapeRec* v = findShapeKey(vk);
            if (s == nullptr || v == nullptr) return;
            sensorEvents_.push_back({ShapeId{sk}, ShapeId{vk}, s->tag, v->tag, begin});
        };
        for (int i = 0; i < se.beginCount; ++i) addSensor(se.beginEvents[i].sensorShapeId, se.beginEvents[i].visitorShapeId, true);
        for (int i = 0; i < se.endCount; ++i) addSensor(se.endEvents[i].sensorShapeId, se.endEvents[i].visitorShapeId, false);

        const b3ContactEvents ce = b3World_GetContactEvents(world_);
        for (int i = 0; i < ce.hitCount; ++i) {
            const b3ContactHitEvent& e = ce.hitEvents[i];
            if (!b3Shape_IsValid(e.shapeIdA) || !b3Shape_IsValid(e.shapeIdB)) continue;
            const std::uint64_t ak = b3StoreShapeId(e.shapeIdA);
            const std::uint64_t bk = b3StoreShapeId(e.shapeIdB);
            const ShapeRec* a = findShapeKey(ak);
            const ShapeRec* b = findShapeKey(bk);
            if (a == nullptr || b == nullptr) continue;
            contactEvents_.push_back({ShapeId{ak}, ShapeId{bk}, a->tag, b->tag, fromB3(e.point), fromB3(e.normal),
                                      e.approachSpeed});
        }
    }

    PhysicsConfig cfg_;
    b3WorldId world_ = b3_nullWorldId;
    std::uint64_t nextSeq_ = 1;
    std::uint64_t nextJoint_ = 1;
    double lastStepMs_ = 0.0;
    std::unordered_map<std::uint64_t, BodyRec> bodies_;
    std::map<std::uint64_t, std::uint64_t> bodyOrder_;  // creation sequence -> body key (hash order)
    std::unordered_map<std::uint64_t, ShapeRec> shapes_;
    std::map<std::uint64_t, JointRec> joints_;
    std::vector<SensorEvent> sensorEvents_;
    std::vector<ContactEvent> contactEvents_;
};

}  // namespace

std::unique_ptr<IPhysicsWorld> createBox3dWorld(const PhysicsConfig& config) {
    if (!finite(config.gravity)) return nullptr;
    auto world = std::make_unique<Box3dWorld>(config);
    if (!world->ok()) return nullptr;
    return world;
}

PhysicsWorldFactory box3dWorldFactory() {
    return [](const PhysicsConfig& cfg) { return createBox3dWorld(cfg); };
}

std::string box3dVersionString() {
    const b3Version v = b3GetVersion();
    return std::to_string(v.major) + "." + std::to_string(v.minor) + "." + std::to_string(v.revision);
}

}  // namespace wpiemu::core::physics
