# ADR 0003 - Box3D chassis physics and custom tire model

- Status: accepted (spike `spike-box3d` complete, spike code deleted)
- Date: 2026-10-15
- Scope: how `wpiemu_core` uses Box3D v0.1.0 for the chassis, fuel, field and sensor volumes, and the
  per-wheel traction model that replaces Box3D wheel friction.

## Decision

1. Use **Box3D v0.1.0** (`erincatto/box3d`, MIT, C17), pinned by tag, consumed with
   `add_subdirectory`/`FetchContent` and **hidden behind `IPhysicsWorld`**. The API is v0.x; nothing outside
   the adapter may include `box3d/*.h`.
2. **The chassis is one rigid body. There is no suspension** (no springs, dampers or travel). The body rests
   on the floor through four small frictionless *wheel-pad spheres* (radius = wheel radius, friction 0,
   restitution 0, density 0) at the wheel centres. Box3D provides support and normal contact; **all tangential
   (traction) force comes from our tire model**, applied with `b3Body_ApplyForce` at each wheel's ground
   contact point.
3. **Normal load per wheel comes from a quasi-static model** (static weight distribution plus load transfer
   from chassis acceleration). Box3D contact normal impulses on the pad are a validated alternative but are
   *not* the default (spikes on edges, see Findings).
4. Rays are used only to find the ground contact point/normal under each wheel (slopes, bumps). A ray is not a
   spring.
5. **World convention: Z-up, right-handed, meters/kg/seconds**, identical to WPILib NWU field coordinates
   (+X toward the opposing alliance wall, +Y left, +Z up). Box3D has no built-in up axis; only the default
   gravity is Y-down, so we set `gravity = {0,0,-9.81}`. Everything below was validated Z-up. Convert to Y-up
   only at the Qt Quick 3D render boundary.
6. Run the sim step at **fixed 200 Hz, 4 substeps** (Box3D's recommended substep count), 1 worker by default.

## Box3D v0.1.0 API cheat-sheet (only what we need)

Headers: `box3d/box3d.h` (functions), `types.h` (defs/events), `collision.h` (hulls, meshes), `math_functions.h`
(vector math; also provides C++ `operator+ - *` for `b3Vec3` when compiled as C++, do not redefine them),
`id.h` (ids are small structs passed **by value**; 8 bytes: `{int32 index1; uint16 world0; uint16 generation}`).
In the default single-precision build `b3Pos == b3Vec3` and `b3WorldTransform == b3Transform`.

| Need | API |
|---|---|
| Version | `b3GetVersion()` -> `{major,minor,revision}` |
| World | `b3WorldDef d = b3DefaultWorldDef(); d.gravity; d.workerCount; d.enableSleep; d.enableContinuous; d.restitutionThreshold(1 m/s); d.hitEventThreshold; d.contactHertz(30) d.contactDampingRatio(10) d.contactSpeed(3 m/s) d.maximumLinearSpeed(400); b3CreateWorld(&d)`, `b3DestroyWorld`, `b3World_SetGravity`, `b3World_EnableSleeping`, `b3World_EnableContinuous`, `b3World_SetWorkerCount`, `b3World_GetAwakeBodyCount`, `b3World_GetCounters`, `b3World_GetProfile` |
| Step | `b3World_Step(world, dt, subStepCount)` - `void`, no accumulator; **we** own the fixed loop |
| Body | `b3BodyDef b = b3DefaultBodyDef()` (default type is **static**; set `b3_dynamicBody`), `.position .rotation(b3Quat{v,s}) .linearVelocity .enableSleep .isBullet`; `b3CreateBody`, `b3DestroyBody`, `b3Body_IsValid` |
| Body state | `b3Body_GetPosition/GetRotation/GetTransform`, `b3Body_GetLinearVelocity/GetAngularVelocity`, `b3Body_GetWorldPoint/GetLocalPoint/GetWorldVector/GetLocalVector`, `b3Body_GetWorldPointVelocity(body, worldPt)`, `b3Body_SetTransform`, `b3Body_SetLinearVelocity`, `b3Body_SetAngularVelocity`, `b3Body_SetAwake`, `b3Body_Enable...` |
| Forces | `b3Body_ApplyForce(body, force, worldPoint, wake)`, `b3Body_ApplyForceToCenter`, `b3Body_ApplyTorque`, `b3Body_ApplyLinearImpulse(+ToCenter)`, `b3Body_ApplyAngularImpulse`. **Forces/torques are cleared at the end of every `b3World_Step`**, so reapply each step. A force at a point produces torque about the COM. |
| Mass | `b3Body_SetMassData(body, b3MassData{mass, center, inertia(b3Matrix3)})` (call after all shapes are added; overrides shape-derived mass), `b3Body_GetMass`, `b3Body_GetWorldCenterOfMass`, `b3Body_SetLinearDamping/AngularDamping` |
| Box shape | `b3BoxHull h = b3MakeBoxHull(hx,hy,hz)` (**half**-extents; stack object, do not `b3DestroyHull`), `b3MakeOffsetBoxHull(hx,hy,hz,offset)`, `b3MakeTransformedBoxHull(hx,hy,hz,xf)`; `b3CreateHullShape(body,&shapeDef,&h.base)` |
| Sphere | `b3Sphere s = {center, radius}; b3CreateSphereShape(body,&sd,&s)` |
| Capsule | `b3CreateCapsuleShape(body,&sd,&b3Capsule)` |
| Cylinder | `b3HullData* c = b3CreateCylinder(height, radius, yOffset, sides)`: axis is **Y**, base at `yOffset` (use `-height/2` to centre). For Z-axis: `b3CreateTransformedHullShape(body,&sd,c,{p, b3MakeQuatFromAxisAngle({1,0,0},pi/2)},{1,1,1})`; free with `b3DestroyHull(c)` |
| Convex hull | `b3CreateHull(points,count,maxVerts)` |
| Static mesh | `b3MeshDef{vertices,indices,vertexCount,triangleCount,...}; b3CreateMesh(&def, degenerateIdx, cap)` -> `b3MeshData*`; `b3CreateMeshShape(body,&sd,mesh,scale)`; mesh must outlive the shape; **static bodies only**; also `b3CreateBoxMesh`, `b3CreateHeightFieldShape`, `b3CreateCompoundShape` (static only) |
| Shape def | `b3ShapeDef sd = b3DefaultShapeDef()`: `.density`(kg/m^3, default 1000) `.baseMaterial{friction(0.6), restitution(0), rollingResistance(0), tangentVelocity, userMaterialId}` `.filter{categoryBits,maskBits,groupIndex}` `.isSensor` `.enableSensorEvents` `.enableContactEvents` `.enableHitEvents` `.enablePreSolveEvents` `.enableCustomFiltering` `.userData` `.updateBodyMass` |
| Shape runtime | `b3Shape_GetBody`, `b3Shape_GetUserData/SetUserData`, `b3Shape_GetFilter/SetFilter(shape,filter,invokeContacts)`, `b3Shape_SetFriction/SetRestitution/SetSurfaceMaterial`, `b3Shape_EnableSensorEvents`, `b3Shape_IsSensor`, `b3Shape_IsValid`, `b3Shape_GetAABB`, `b3Shape_GetSensorData(shape, visitors, cap)` / `b3Shape_GetSensorCapacity` (current overlaps), `b3Shape_GetContactData(shape, out, cap)` -> `b3ContactData{shapeIdA,shapeIdB,manifolds,manifoldCount}`, `b3DestroyShape(shape, updateBodyMass)` |
| Filtering | pair collides if `(A.mask & B.cat) && (A.cat & B.mask)`; `groupIndex` != 0 overrides (equal negative = never collide). Same rule gates **sensor** detection. Ray queries: `b3QueryFilter{categoryBits, maskBits}` (`b3DefaultQueryFilter()`), tested against shape category/mask |
| Ray casts | `b3World_CastRayClosest(world, origin, translation, filter)` -> `b3RayResult{shapeId, point, normal, fraction, hit, ...}` (**translation = direction*length, not an end point**); `b3World_CastRay` (callback), `b3World_CastShape`, `b3World_OverlapAABB`, `b3World_OverlapShape` |
| Sensor events | after step: `b3SensorEvents e = b3World_GetSensorEvents(world)`: `e.beginEvents[i]{sensorShapeId, visitorShapeId}`, `e.endEvents[i]` (+ counts). Transient; valid until next step/destroy |
| Contact events | `b3World_GetContactEvents(world)` -> begin/end/hit arrays (`b3ContactHitEvent{point, normal, approachSpeed}`); dynamic/kinematic bodies only, shape must set `enableContactEvents`/`enableHitEvents` |
| Contact forces | `b3ManifoldPoint.normalImpulse` is the impulse of the **final substep**; force = impulse / (dt/substeps). `totalNormalImpulse` is over all substeps |
| Other | `b3World_SetCustomFilterCallback`, `b3World_SetPreSolveCallback` (must be thread-safe), `b3World_SetFrictionCallback`/`SetRestitutionCallback` (mixing rule defaults: friction `sqrt(a*b)`, restitution `max(a,b)`), `b3Body_SetBullet` |

Threads: `workerCount > 1` **without** task callbacks makes Box3D create its own scheduler/threads
(`physics_world.c`; the prose in `docs/foundation.md` says it creates none, the code does). With callbacks you
provide `enqueueTask/finishTask/userTaskContext`. Read-only queries (rays, overlaps) are thread-safe *outside*
`b3World_Step`; never write to the world concurrently. Max 128 worlds, `B3_MAX_WORKERS` = 32.

## Build / integration

```cmake
FetchContent_Declare(box3d GIT_REPOSITORY https://github.com/erincatto/box3d.git GIT_TAG v0.1.0)
set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)      # no sokol/imgui, no GPU deps
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(box3d)               # not top-level => samples/tests/docs are already hidden
# Box3D hard-codes a static CRT (CMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded) in its own scope; Qt needs /MD:
set_target_properties(box3d PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
target_link_libraries(wpiemu_physics_box3d PRIVATE box3d)
```

- Verified on MSVC 19.44 / VS2022, Ninja, Release: builds in about 20 s as a static library, links into a `/MD`
  C++20 executable, no extra dependencies.
- `box3d` is a C library; the headers are C++-safe and give `b3Vec3` operators for free.
- **Determinism**: Box3D compiles with `-ffp-contract=off` on GCC/Clang (directory scoped) and precise math on
  MSVC. Our own physics code (tire model, load solver) is **not** covered, so add `-ffp-contract=off` to the
  `wpiemu_core`/physics targets (Apple Clang contracts to FMA by default) and avoid `-ffast-math`.
- Box3D's CMake also sets `CMAKE_VERBOSE_MAKEFILE ON` and a `FETCHCONTENT_BASE_DIR` cache default; harmless but
  noisy.

## Recommended `IPhysicsWorld` interface

Engine-agnostic (no Box3D types), `float` math, ids are opaque 64-bit handles (pack the 8-byte
Box3D ids). Z-up SI units. Everything the chassis, fuel, field and sensor code needs, and nothing else:

```cpp
namespace wpiemu::sdk {

struct Vec3 { float x, y, z; };
struct Quat { float x, y, z, w; };
struct Pose { Vec3 p; Quat q; };

using BodyId  = uint64_t;   // 0 = invalid
using ShapeId = uint64_t;

enum class BodyType { Static, Kinematic, Dynamic };

// Collision categories are a game-agnostic bitmask; plugins may define more bits >= 1<<16.
namespace layer { enum : uint64_t { Field = 1, Robot = 2, Fuel = 4, Sensor = 8, Projectile = 16, All = ~0ull }; }

struct Material { float friction = 0.6f, restitution = 0.f, rollingResistance = 0.f; };

struct BoxShape      { Vec3 halfExtents; };
struct SphereShape   { float radius; };
struct CylinderShape { float radius, halfHeight; };            // axis = local Z
struct MeshShape     { const float* xyz; size_t vertexCount; const uint32_t* tri; size_t triCount; }; // static only

struct ShapeDesc {
    std::variant<BoxShape, SphereShape, CylinderShape, MeshShape> geometry;
    Pose local;                       // pose relative to the body
    Material material;
    float density = 0.f;              // 0 for pads/sensors; use setMassProperties for the chassis
    uint64_t category = layer::Field, mask = layer::All;
    bool sensor = false;              // overlap volume only
    bool sensorEvents = false;        // required on BOTH the sensor and the visitor
    bool contactEvents = false;
    uint64_t userTag = 0;             // returned in events (entity id, wheel index, ...)
};

struct BodyDesc {
    BodyType type = BodyType::Static;
    Pose pose{};
    Vec3 linearVelocity{};
    bool allowSleep = true;
    bool bullet = false;
};

struct MassProperties { float mass; Vec3 comLocal; Vec3 inertiaDiagLocal; };

struct RayHit { bool hit; Vec3 point, normal; float fraction; ShapeId shape; uint64_t userTag; };
struct SensorEvent  { ShapeId sensor, visitor; uint64_t sensorTag, visitorTag; bool begin; };
struct ContactEvent { ShapeId a, b; uint64_t tagA, tagB; Vec3 point, normal; float approachSpeed; };

struct PhysicsConfig { int workerCount = 1; int subSteps = 4; bool sleeping = true; Vec3 gravity{0,0,-9.81f}; };
struct PhysicsStats  { int bodies, awakeBodies, contacts; double lastStepMs; };

class IPhysicsWorld {
public:
    virtual ~IPhysicsWorld() = default;

    // lifecycle
    virtual BodyId  createBody(const BodyDesc&) = 0;
    virtual void    destroyBody(BodyId) = 0;                        // call outside event iteration
    virtual ShapeId addShape(BodyId, const ShapeDesc&) = 0;
    virtual void    destroyShape(ShapeId) = 0;
    virtual void    setMassProperties(BodyId, const MassProperties&) = 0;   // after all shapes added
    virtual void    setShapeFilter(ShapeId, uint64_t category, uint64_t mask) = 0;
    virtual void    setShapeMaterial(ShapeId, const Material&) = 0;

    // stepping (fixed dt owned by the caller; forces are consumed by one step)
    virtual void step(float dt) = 0;

    // state
    virtual Pose    pose(BodyId) const = 0;
    virtual Vec3    linearVelocity(BodyId) const = 0;
    virtual Vec3    angularVelocity(BodyId) const = 0;              // world frame
    virtual Vec3    pointVelocity(BodyId, Vec3 worldPoint) const = 0;
    virtual Vec3    worldPoint(BodyId, Vec3 local) const = 0;
    virtual void    setPose(BodyId, const Pose&) = 0;
    virtual void    setLinearVelocity(BodyId, Vec3) = 0;
    virtual void    setAngularVelocity(BodyId, Vec3) = 0;
    virtual void    setEnabled(BodyId, bool) = 0;

    // forces for this step only
    virtual void applyForce(BodyId, Vec3 force, Vec3 worldPoint) = 0;
    virtual void applyForceToCenter(BodyId, Vec3 force) = 0;
    virtual void applyTorque(BodyId, Vec3 torque) = 0;
    virtual void applyImpulse(BodyId, Vec3 impulse, Vec3 worldPoint) = 0;

    // queries
    virtual RayHit rayCastClosest(Vec3 origin, Vec3 direction, float maxDistance, uint64_t maskBits) const = 0;
    virtual void   overlapSphere(Vec3 c, float r, uint64_t maskBits, std::vector<ShapeId>& out) const = 0;
    virtual void   shapesOverlapping(ShapeId sensor, std::vector<ShapeId>& out) const = 0;  // current sensor contents

    // events of the last step (valid until the next step()); copy what you need, then mutate the world
    virtual std::span<const SensorEvent>  sensorEvents() const = 0;
    virtual std::span<const ContactEvent> contactEvents() const = 0;

    // diagnostics
    virtual PhysicsStats stats() const = 0;
    virtual uint64_t stateHash() const = 0;     // FNV over pose+velocity of all bodies in creation order (determinism tests)
    virtual uint64_t shapeTag(ShapeId) const = 0;
    virtual BodyId   shapeBody(ShapeId) const = 0;
    virtual bool     isValid(ShapeId) const = 0;
};

std::unique_ptr<IPhysicsWorld> createBox3dWorld(const PhysicsConfig&);   // lives in wpiemu_physics_box3d
}
```

Notes for the adapter:

- Store `userTag` in `b3ShapeDef::userData` (cast `uint64_t` to `void*`); surface it in events/ray hits.
- `step()` = `b3World_Step(world, dt, cfg.subSteps)`, then copy `b3World_GetSensorEvents`/`GetContactEvents` into
  adapter-owned vectors (translate ids/tags; drop `end` events whose visitor `!b3Shape_IsValid`).
- Destroying a body that appears in the current event arrays: gather the list first, destroy afterwards (done in
  the spike, 30 intake absorptions, no invalid access). End events for destroyed shapes arrive with invalid ids.
- Provide `IPhysicsWorld` mock/in-memory implementation for UI and unit tests; the tire model below depends only on
  `IPhysicsWorld`, never on Box3D.

## Chassis model

```
body origin == centre of mass, COM height h = 0.20 m above the floor
chassis hull : box 0.70 x 0.70 x 0.12 m, bottom 3 cm above the floor (friction 0.3, restitution 0.1,
               category Robot, collides with Field | Fuel | Robot)
wheel pads   : one sphere per wheel at the hub, radius = wheel radius, friction 0, restitution 0, density 0
mass         : b3Body_SetMassData(50 kg, centre {0,0,0}, box inertia diag(Ixx=Iyy=m/12(a^2+h_box^2), Izz=m/12*2a^2))
sleep        : disabled for the chassis (forces are applied every step anyway)
```

With four pads the chassis settled at COM z = 0.1999 m and the pad contacts alone supported 490.5 N (exactly mg).
Because the pad is a sphere of wheel radius the contact point is where a real wheel touches the floor, and the
sphere rolls over bump edges and ramps like a wheel. Keep pad friction at 0: Box3D mixes friction as
`sqrt(a*b)`, so a zero on the pad kills tangential contact force regardless of the floor.

## Tire model

Per wheel, per step (200 Hz), forces applied at the ground contact point `P`:

1. **Contact frame.** `n` = ground normal, `h` = wheel heading (chassis yaw + steer angle) projected onto the
   ground plane. Traction axis `c = cos(phi) h + sin(phi) (n x h)`, free axis `p = n x c`. `phi = 0` for normal
   wheels, `+-45 deg` for mecanum (rollers axis is perpendicular to `c`). Rim speed along `c` is
   `omega r cos(phi)`.
2. **Slip velocities** (positive = wheel faster than ground): `uL = omega r cos(phi) - v.c`, `uT = -v.p`, with
   `v = b3Body_GetWorldPointVelocity(chassis, P)`.
3. **Regularised slip**: `aL = uL / (vRefL * kappaPeak)`, `aT = uT / (vRefT * alphaPeak)` where
   `vRefL = max(vRef, |v.c|, |omega r cos phi|)` (slip ratio), `vRefT = max(vRef, |v|)` (tan slip angle), and
   `vRef = 0.5 m/s` so slip is finite at standstill (acts as static friction creep, no division by ~0).
4. **Brush curve with friction circle**: `s = |(aL, aT)|`, `g(s) = 1-(1-s)^3` for `s<=1`, else
   `slide + (1-slide) e^{-2(s-1)}`; `FL = mu N g aL/s`, `FT = lateralScale mu N g aT/s`. `lateralScale` is both the
   friction multiplier on the free axis and the coupling between axes (1 = true friction circle for tank/swerve
   wheels, ~0.1-0.15 = independent rollers for omni/mecanum). Load dependence is built in (`mu N` scales
   everything) so load transfer changes grip per wheel.
5. **Semi-implicit coupled solve** (this is the part that makes it stable at 200 Hz, see Findings): the wheel
   inertia, motor back-EMF and the chassis mass share at the contact are advanced together, the force returned is
   the end-of-step force, and the same force produces the wheel reaction torque `-r_eff FL`. The lateral force is
   damped implicitly with the per-wheel chassis share `M / nWheels`.
6. **Normal load** `N_i` per wheel from the quasi-static load solver (below). `N_i <= 0` means airborne: no force,
   wheel free-spins.

### Reference code (validated, `wpiemu_core/physics`, header-only, no Box3D dependency)

```cpp
struct TireParams {
    float mu = 1.1f;            // peak friction coefficient
    float slideRatio = 0.85f;   // sliding friction / peak friction
    float peakSlipLong = 0.15f; // slip ratio where force peaks
    float peakSlipLat = 0.20f;  // tan(slip angle) where force peaks
    float vRef = 0.5f;          // m/s, low-speed slip regularisation
    float lateralScale = 1.0f;  // free-axis friction multiplier and axis coupling (omni/mecanum ~0.15)
    float tractionAngle = 0.0f; // rad; 0 normal wheel, +-pi/4 mecanum
    float radius = 0.0508f;     // m
};

struct TireInput {
    float normalLoad;  // N
    float vLong, vLat; // contact-point velocity along traction / free axis (m/s)
    float chassisMass; // kg, per-wheel share of the chassis (M / numWheels)
    float dt;
};

// Motor/gearbox side of one wheel, linearised about the current wheel speed:
// torque(w') = torque - damping * (w' - w)
struct WheelDrive {
    float omega;    // rad/s
    float torque;   // N*m at the wheel at the current omega (current limit already applied)
    float damping;  // N*m per rad/s (back-EMF slope), 0 when current limited
    float inertia;  // kg*m^2 reflected to the wheel (rotor*G^2 + wheel)
};

struct TireOutput {
    float fLong = 0, fLat = 0; // force on chassis (N) along traction / free axis
    float omegaNext = 0;       // wheel speed after this step (rad/s)
    float slipRatio = 0;
    float load = 0;            // |F| / (mu N), 1 = at the friction limit
};

inline float frictionCurve(float s, float slideRatio) {
    if (s <= 1.0f) { float t = 1.0f - s; return 1.0f - t * t * t; }   // slope 3 at 0, peak 1 at s = 1
    return slideRatio + (1.0f - slideRatio) * std::exp(-2.0f * (s - 1.0f));
}

struct FricForce { float fLong, fLat; };
inline FricForce frictionForce(const TireParams& p, float N, float uLong, float uLat, float vRefLong, float vRefLat) {
    float a = uLong / (vRefLong * p.peakSlipLong);
    float b = uLat / (vRefLat * p.peakSlipLat);
    float k = p.lateralScale;
    float sC = std::hypot(a, k * b), sP = std::hypot(b, k * a);
    float fMax = p.mu * N;
    float fL = sC > 1e-9f ? fMax * frictionCurve(sC, p.slideRatio) * a / sC : 0.0f;
    float fT = sP > 1e-9f ? fMax * k * frictionCurve(sP, p.slideRatio) * b / sP : 0.0f;
    return {fL, fT};
}

inline TireOutput solveTire(const TireParams& p, const TireInput& in, const WheelDrive& d) {
    TireOutput out;
    out.omegaNext = d.omega;
    const float ca = std::cos(p.tractionAngle), reff = p.radius * ca;
    const float rim = d.omega * reff;
    const float Ie = d.inertia + in.dt * d.damping;
    if (in.normalLoad <= 0.0f) { out.omegaNext = d.omega + in.dt * d.torque / Ie; return out; }   // airborne

    float vRefLong = std::max({p.vRef, std::fabs(in.vLong), std::fabs(rim)});
    float vRefLat  = std::max(p.vRef, std::hypot(in.vLong, in.vLat));
    float uL = rim - in.vLong, uT = -in.vLat;
    FricForce f0 = frictionForce(p, in.normalLoad, uL, uT, vRefLong, vRefLat);
    const float eps = 0.02f;                                   // slip-velocity stiffness by finite difference
    float kL = std::max(0.0f, (frictionForce(p, in.normalLoad, uL + eps, uT, vRefLong, vRefLat).fLong - f0.fLong) / eps);
    float kT = std::max(0.0f, (frictionForce(p, in.normalLoad, uL, uT + eps, vRefLong, vRefLat).fLat - f0.fLat) / eps);
    const float invM = 1.0f / in.chassisMass;

    // longitudinal: wheel and chassis share one slip velocity; solve u' = u + du with F' = F0 + kL du
    float c = reff * reff / Ie + invM;
    float du = in.dt * (reff * d.torque / Ie - c * f0.fLong) / (1.0f + kL * in.dt * c);
    float fLong = f0.fLong + kL * du;
    // lateral: chassis only
    float fLat = f0.fLat / (1.0f + kT * in.dt * invM);

    float fMaxL = p.mu * in.normalLoad, fMaxT = fMaxL * p.lateralScale;
    fLong = std::clamp(fLong, -fMaxL, fMaxL);
    fLat  = std::clamp(fLat,  -fMaxT, fMaxT);
    out.fLong = fLong; out.fLat = fLat;
    out.omegaNext = d.omega + in.dt * (d.torque - reff * fLong) / Ie;
    out.slipRatio = uL / vRefLong;
    out.load = std::max(std::fabs(fLong) / fMaxL, std::fabs(fLat) / std::max(fMaxT, 1e-6f));
    return out;
}

// Quasi-static normal loads for ANY wheel layout (swerve, 4/6-wheel tank, mecanum).
// Min-norm solution of: sum N = W, sum N*x = -h*m*ax, sum N*y = -h*m*ay (x,y relative to the COM in the body
// frame; ax, ay = tire-force acceleration, i.e. measured acceleration minus the gravity component; W = m*g*cos(tilt)).
inline void quasiStaticLoads(const std::vector<std::pair<float,float>>& xy, float mass, float comHeight,
                             float ax, float ay, float gNormal, std::vector<float>& N) {
    const int n = (int)xy.size();
    const float W = mass * gNormal;
    N.assign(n, W / n);
    double A[3][3] = {};
    for (int i = 0; i < n; ++i) {
        double r[3] = {1.0, xy[i].first, xy[i].second};
        for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) A[a][b] += r[a] * r[b];
    }
    double N0 = W / n, sx = 0, sy = 0;
    for (int i = 0; i < n; ++i) { sx += xy[i].first; sy += xy[i].second; }
    double res[3] = {0.0, -comHeight * mass * ax - N0 * sx, -comHeight * mass * ay - N0 * sy};
    auto det3 = [](double m[3][3]) {
        return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
             + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]); };
    double d = det3(A);
    if (std::fabs(d) < 1e-9) return;                           // degenerate (collinear) layout: keep equal split
    double lam[3];
    for (int c = 0; c < 3; ++c) { double M[3][3]; memcpy(M, A, sizeof M);
        for (int r = 0; r < 3; ++r) M[r][c] = res[r]; lam[c] = det3(M) / d; }
    float sum = 0;
    for (int i = 0; i < n; ++i) {
        N[i] = (float)std::max(0.0, N0 + lam[0] + lam[1] * xy[i].first + lam[2] * xy[i].second);   // clamp: lifted wheel
        sum += N[i];
    }
    if (sum > 1e-6f) for (auto& v : N) v *= W / sum;           // renormalise after clamping
}
```

(The spike's `res[0]` is `W - N0*n`, which is exactly 0, hence the literal 0 above.)

### Wiring to `IPhysicsWorld` (per step, per wheel)

```cpp
// 0. chassis acceleration from the previous step, low-passed (tau = 20 ms) to avoid a 1-step algebraic loop:
//    a = (v - vPrev)/dt ; aTire_body = R^T a - R^T g       (subtract the gravity component along the tilted chassis)
quasiStaticLoads(wheelXY, mass, comHeight, aTire.x, aTire.y, std::max(0.f, -gBody.z), N);
for each wheel i:
    // 1. ground contact point/normal only (ray from the hub, length radius+2 cm, mask = Field)
    RayHit hit = world.rayCastClosest(hubWorld, chassisDown, wheel.radius + 0.02f, layer::Field);
    if (!hit.hit) { wheel.omega = solveTire(..., normalLoad = 0 ...).omegaNext; continue; }  // airborne
    // 2. contact frame (h projected on the ground plane, c, p as above), contact velocity
    Vec3 vc = world.pointVelocity(chassis, hit.point);
    // 3. drive: motor model -> WheelDrive{omega, torque, damping, inertia}  (current limit sets damping = 0)
    TireOutput o = solveTire(wheel.tire, {N[i], dot(vc, c), dot(vc, p), mass / nWheels, dt}, drive);
    world.applyForce(chassis, o.fLong * c + o.fLat * p, hit.point);     // at the ground point -> pitch/roll torque emerges
    wheel.omegaNext = o.omegaNext;
world.step(dt);
for each wheel: wheel.omega = wheel.omegaNext;   // feeds back to the motor/CANcoder/encoder sim
```

Motor side used in the spike (Kraken X60, `tau = eta*G*Kt*V/R - eta*G^2*Kt*Ke/R * omega`, clamped to
`eta*G*Kt*Ilimit`, damping 0 when clamped): `Kt = 0.01937 N*m/A`, `R = 0.03279 ohm`, `Ke = 0.0191 V/(rad/s)`,
`G = 6.75`, `eta = 0.9`, reflected inertia `J_rotor*G^2 + J_wheel` with `J_rotor = 5e-5`, `J_wheel = 1.5e-3`
kg m^2. In the product this comes from the WPILib `DCMotor` plant; the tire solver only needs
`{torque, damping, inertia}` per wheel, so it stays independent of the motor model.

### Parameters (defaults for FRC)

| Param | Default | Notes |
|---|---|---|
| mass | 50 kg | 60 kg typical with battery/bumpers, user setting |
| COM height | 0.20 m | drives load transfer; 0.15-0.30 m realistic |
| wheel radius | 0.0508 m (4 in) | |
| `mu` | 1.1 | 0.8-1.4 (carpet vs tread); also settable per wheel/per module |
| `slideRatio` | 0.85 | sliding / peak; gives the "skid on over-torque" step down |
| `peakSlipLong` | 0.15 | slip ratio at peak force |
| `peakSlipLat` | 0.20 | tan(slip angle) at peak force (11 deg) |
| `vRef` | 0.5 m/s | creep at standstill; larger = stickier, more damped |
| `lateralScale` | 1.0 swerve/traction, 0.12-0.15 omni, 0.15 mecanum | |
| `tractionAngle` | 0, `+-pi/4` mecanum | WPILib X layout (+y left): FL `-45`, FR `+45`, RL `+45`, RR `-45` deg |
| substeps | 4 at 200 Hz | |
| fuel | r 0.075 m, 0.2 kg (density 113.2 kg/m^3), friction 0.6, restitution 0.55, rolling resistance 0.02 | see Findings |

Mecanum/tank inverse kinematics used for the checks: wheel contact velocity `vc = (vx - wz*y, vy + wz*x)`;
wheel speed `omega = (vc . c) / (r cos phi)` (for tank, `c` is forward, so `omega = vc.x / r`). This reproduces the
WPILib `MecanumDriveKinematics` row pattern.

## Findings (all measured, Release build, AMD Ryzen 9 9950X, MSVC 19.44)

Setup: 50 kg, 0.5 m x 0.5 m wheel square, COM 0.2 m, `mu` 1.1, 4 in wheels, Kraken X60 @ 6.75:1, 200 Hz,
4 substeps. Theoretical `mu g` = 10.79 m/s^2, sliding `0.85 mu g` = 9.17 m/s^2.

**Tire behaviour**

| Test | Result |
|---|---|
| Static settle | each pad 122.6 N, sum 490.5 N = mg, COM z = 0.1999, no jitter (both load sources) |
| Swerve fwd, 1 V (below traction) | no slip (slip ratio 0.02), first 0.1 s average 2.8 m/s^2, ends at free speed 0.39 m/s |
| Swerve fwd, 12 V, no current limit (over-torque) | 9.24 m/s^2 = traction-limited at the **sliding** level; wheel rim 4.58 m/s vs body 2.77 m/s at 0.3 s (slip ratio 0.40), reaches 4.73 m/s |
| 12 V, 60 A / 40 A stator limit | 7.07 / 5.78 m/s^2; front wheels slip first because load transfer unloads them |
| Load transfer at 9.24 m/s^2 | front pair 32 N / rear pair 213 N each (formula `m a h / L` predicts 30 / 215) |
| Plug braking 4 m/s at -12 V | 9.17 m/s^2 decel = `0.85 mu g` (skidding) |
| Swerve strafe (steer 90 deg) 12 V | 9.24 m/s^2, same as forward; steady 2.00 m/s command -> 2.000 m/s, yaw 0 |
| Swerve 45 deg diagonal 12 V | 9.25 m/s^2, drift 0.2 m in 1.5 s (a wheel unloads to 0 N) |
| Swerve spin in place | 8.485 rad/s vs ideal 8.485 |
| Tank 4-wheel straight 3/3 m/s | vx 3.000, vy 0, yaw 0 |
| Tank turn -1/+1 m/s (ideal 4.0 rad/s, track 0.5 m) | yaw 1.64 (traction wheels, `lateralScale` 1.0), 3.01 (0.5), 3.81 (0.15, omni): scrub steers the result as expected |
| Mecanum | cmd vx 2 -> 1.948; vy 2 -> 1.947; wz 3 -> 3.000 rad/s; mixed (1.5,1.5,1.0) -> (1.525,1.421,1.001) |
| Mecanum 12 V traction-limited | 5.49 m/s^2 forward **and** strafe (0.59 of swerve; wheel forces are 45 deg off-axis) |
| 15 deg ramp (rotated static box) | chassis pitch 14.4-15.2 deg while climbing at 1.6-2.1 m/s; summed load tracks `m g cos(15)` (475 vs 474 N) |

**Normal load source comparison.** Box3D contact impulses on the pads (`normalImpulse / (dt/substeps)`) reproduce the
quasi-static loads within about 2-4 % in steady cases (`1 V`: front 106/108 N, rear 139/138 N vs 103/143 N; accel
results identical to 0.01 m/s^2). But the contact source is solver-dependent for statically indeterminate
layouts and produces **spikes** when a pad hits an edge: 13.8 kN on the first ramp contact, which turned into a
huge friction impulse and flipped the chassis (pitch -84 deg). The quasi-static source has no such failure and is
deterministic and layout-independent. Decision: quasi-static primary; use contact impulses only for debugging or
for a future optional "contact" mode with a clamp (e.g. `N <= 3 m g`).

**Tire solver stability.** An explicit tire force with a stiff slip curve is unstable at 200 Hz
(`K dt / m_wheel-share` is about 2 near standstill). A hand-tuned cap `F <= m u / dt` fixed the explosion but made
the drive wheels lag the body and lost about 40 % of drive force (rear wheels 33 N instead of 84 N). The coupled
implicit solve above (wheel + chassis share) removes that, needs no tuning, and gives the correct tractive
force at 1 V through full-voltage slip.

**Sensors / events / filters**

- `isSensor` + `enableSensorEvents` on both the sensor and the visitor. Sensor on a *dynamic* chassis body (intake
  zone, `b3CreateHullShape` with `isSensor`) and on a *static* body (hub volume) both produce begin/end events.
  Spike: 20 fuel on the path + 10 dropped into a hub volume + 1 fast shot: intake begin 30, absorbed 30 (bodies
  destroyed after the event loop, 30 end events had invalid visitor ids as documented), hub begin 12 / end 12.
- The same two-way category/mask rule gates sensors: when the fuel mask lacked the `Sensor` bit, **zero** events
  were produced (silent failure). Fuel masks must include the sensor categories they are meant to trigger.
- Sensors have no CCD; a 25 m/s ball moves 0.125 m/step at 200 Hz, so sensor volumes must be >= about 0.3 m
  thick or the shot must also be detected by a ray/shape cast.
- Continuous collision is on by default: a 30 m/s ball against a 2 cm static wall was stopped and bounced
  (default `maximumLinearSpeed` 400 m/s is not a limit for shots).
- Restitution: ball 0.6 dropped 1.0 m rebounds to 0.356 m (`e^2 = 0.36`) with floor restitution 0 (mix = max).
  Rolling resistance: launched at 3 m/s on the floor, rolls 21 m with 0, stops after 9.8 m / 10 s with 0.02 and
  2 m with 0.1. Pick about 0.02-0.05 for foam on carpet and tune later.
- Static triangle mesh collider: ray hit at the correct height with the correct slope normal; a fuel sphere rolled
  down the mesh ramp. Z-axis cylinder through `b3CreateTransformedHullShape` works (AABB padding 2 cm per side is
  Box3D's margin).

**Performance** (single step incl. our 4-wheel tire code, fixed 200 Hz, 4 substeps, 50 kg robot driving at 4 m/s
through the pile; milliseconds per `step`; budget at 200 Hz = 5.0 ms)

| Scenario | workers | mean | p99 |
|---|---|---|---|
| 200 fuel in a pile, asleep | 1 | 0.01 | 0.05 |
| 200 fuel dropped from 0.2-2.2 m, settling | 1 / 4 | 0.18 / 0.11 | 0.44 / 0.33 |
| 200 fuel, robot plowing through | 1 / 4 | 0.12 / 0.09 | 0.20 / 0.31 |
| 200 fuel, sleeping **off** (worst case, all awake) | 1 / 2 / 4 | 0.22 / 0.17 / 0.14 | 0.27 / 0.38 / 0.35 |
| 600 fuel, all awake | 1 / 4 / 8 | 0.71 / 0.33 / 0.39 | 0.83 / 0.58 / 0.71 |
| 1000 fuel, 4 workers (settling, 981 awake) | 4 | 0.59 | 1.10 |
| 200 fuel all awake, substeps 1 / 2 / 4 | 1 | 0.12 / 0.14 / 0.22 | |

Other numbers: ray cast down against the field-only mask 0.15 us, through a 200-ball pile with all categories
0.47 us (so 4 rays/wheel-step is irrelevant); create a fuel body 0.7 us, destroy 0.15 us (1000 spawn burst = under
1 ms). Even a several-times slower laptop core (not measured here) leaves most of the 5 ms budget free, so **single-threaded is the default**;
raise `workerCount` to 2-4 only when more than about 500 awake bodies are expected. More than 4 workers did not help.

**Determinism.** The same 12 s scenario (robot driving through 200 fuel, contact + hit events included in the hash)
hashes bit-identical across two runs and for `workerCount` 1, 2, 4 and 8, both with 4 substeps and 1 substep
(`ac86186471c9bb22` and `bf4c239fc9cb0b0c`). Different substep counts give different (equally valid) trajectories,
so the substep count is part of the sim configuration. Cross-OS/compiler bit equality is claimed by Box3D (own CI
test) but was only verified on Windows/MSVC here; add our own cross-platform hash test to CI.

## Pitfalls

1. Default gravity is `{0,-10,0}`; always set it. Default body type is **static**. A dynamic body needs a shape
   with density > 0 or an explicit `b3Body_SetMassData`.
2. `b3MakeBoxHull` takes **half**-extents; `b3World_CastRayClosest` takes a **translation vector**, not an end point.
3. `b3Body_ApplyForce` accumulates only until the next `b3World_Step`; apply every step. Pass `wake=true`.
4. Zero pad friction is mandatory; otherwise Box3D contact friction fights the tire model. Chassis hull bottom
   must stay clear of the floor; the pads carry the vehicle.
5. A shape cannot become a sensor later. Sensor events need `enableSensorEvents` on both shapes, and the filter
   must pass in both directions (see sensor finding).
6. Event arrays are transient and end events can reference destroyed shapes: copy, check `b3Shape_IsValid`, mutate
   the world only after the loop.
7. Meshes/height fields/compounds are static-only; a mesh must outlive its shape.
8. `b3CreateCylinder` axis is Y with its base at `yOffset`.
9. `normalImpulse` is the final-substep impulse: divide by `dt/substeps`; it can spike on edges.
10. CMake: Box3D forces a static MSVC CRT in its own scope (override the target property); `-ffp-contract=off` is
    scoped to Box3D, add it to our own physics code for cross-platform determinism.
11. `workerCount > 1` spawns Box3D's own threads even though the docs say it does not. Do not call
    `b3World_Step` from inside another job system's job (it blocks waiting for its own tasks).
12. Quasi-static loads use the measured acceleration low-passed with 20 ms; wall hits show up in it too (correct
    physically). A 3-wheel-loaded case (e.g. 45 deg diagonal full throttle) drops one wheel to 0 N; the chassis
    would tip about the diagonal in reality, the model only reproduces the grip loss.
13. `lateralScale` also couples the axes; do not set it to 0 (zero lateral stiffness). Use 0.1-0.15 for rollers.
14. Float32 everywhere (no `BOX3D_DOUBLE_PRECISION`); field is 16.5 m, precision is a non-issue.

## Does anything block the plan?

No. Box3D v0.1.0 builds headless with CMake in about 20 s, has rigid bodies, spheres/hulls/static meshes, filters,
sensor and contact events, ray casts, optional threading and is bit-deterministic across worker counts. The chassis
traction approach works without suspension and without Box3D friction. Remaining risk is only API churn on a v0.x
library (mitigated by the pin and `IPhysicsWorld`) and unverified cross-OS bit equality of our own code (CI task).

## Consequences

- `core/physics` contains `TireModel` (above), `QuasiStaticLoads`, drivetrain mappings (swerve per-module heading,
  tank left/right groups, mecanum `phi = +-45 deg`), and a Box3D adapter implementing `IPhysicsWorld`.
- The game plugin builds the field with `ShapeDesc` boxes (rotated for ramps), meshes for non-box geometry, and uses
  `layer::Fuel`/`layer::Sensor` bits; intake and hub volumes are sensor shapes (intake attached to the chassis body).
- The editor's chassis dimensions map directly to the hull, wheel positions to pads/tire frames, and COM height and
  mass to `setMassProperties`.
- Add unit tests: static load split, 1 V no-slip acceleration, full-voltage traction limit (`0.85 mu g`), mecanum
  kinematics round trip, determinism hash across worker counts.
