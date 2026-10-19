#include "wpiemu/plugins/game_2026/field_layout.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "wpiemu/plugins/game_2026/field_dimensions.h"

namespace wpiemu::plugins::game_2026 {

namespace {

using sdk::Pose3;
using sdk::Quat;
using sdk::Vec3;

constexpr float kPi = 3.14159265358979f;
constexpr float kRampAngleRad = 0.2617994f;  // 15 degrees

const char* const kRedColor = "#b3261e";
const char* const kBlueColor = "#1f4fb3";
const char* const kSteelColor = "#8a8f99";
const char* const kRailColor = "#cfe3f0";
const char* const kCarpetColor = "#5a5f6b";

const char* allianceColor(Alliance a) { return a == Alliance::Red ? kRedColor : kBlueColor; }

// Blue geometry is authored once; red is the same field rotated 180 degrees about the field centre.
class Frame {
public:
    explicit Frame(Alliance a) : red_(a == Alliance::Red) {}
    Vec3 point(const Vec3& p) const { return red_ ? Vec3{dim::kFieldLengthM - p.x, dim::kFieldWidthM - p.y, p.z} : p; }
    Vec3 vector(const Vec3& v) const { return red_ ? Vec3{-v.x, -v.y, v.z} : v; }
    Pose3 pose(const Pose3& p) const {
        if (!red_) return p;
        return {point(p.position), (Quat::fromYaw(kPi) * p.rotation).normalized()};
    }
    float xDir() const { return red_ ? -1.f : 1.f; }

private:
    bool red_;
};

Pose3 at(float x, float y, float z) { return Pose3{{x, y, z}, Quat{}}; }

void addBox(FieldLayout& l, const Frame& f, std::string name, const Pose3& pose, const Vec3& size, const char* color,
            const sdk::Material& material = {}) {
    l.boxes.push_back({std::move(name), f.pose(pose), size, color, material, true});
}

void addAllianceWall(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("wall_") + allianceName(a);
    const float t = d::kAllianceWallThicknessM, h = d::kAllianceWallHeightM, w = d::kFieldWidthM;
    const float lo = d::kOutpostCenterYM - d::kCorralOpeningWidthM * 0.5f;
    const float hi = d::kOutpostCenterYM + d::kCorralOpeningWidthM * 0.5f;
    const float gapBottom = d::kCorralOpeningBottomM, gapTop = gapBottom + d::kCorralOpeningHeightM;
    sdk::Material wall;
    wall.restitution = 0.3f;
    addBox(l, f, n + "_low_y", at(-t / 2, lo / 2, h / 2), {t, lo, h}, kSteelColor, wall);
    addBox(l, f, n + "_outpost_lip", at(-t / 2, (lo + hi) / 2, gapBottom / 2), {t, hi - lo, gapBottom},
           allianceColor(a), wall);
    addBox(l, f, n + "_outpost_top", at(-t / 2, (lo + hi) / 2, (gapTop + h) / 2), {t, hi - lo, h - gapTop},
           allianceColor(a), wall);
    addBox(l, f, n + "_high_y", at(-t / 2, (hi + w) / 2, h / 2), {t, w - hi, h}, kSteelColor, wall);
}

void addHub(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("hub_") + allianceName(a);
    const float s = d::kHubSizeM, t = d::kHubWallThicknessM, top = d::kHubTopZM;
    const float cx = d::kHubCenterXM, cy = d::kHubCenterYM;
    const char* color = allianceColor(a);
    addBox(l, f, n + "_base", at(cx, cy, d::kHubBaseHeightM / 2), {s, s, d::kHubBaseHeightM}, color);
    addBox(l, f, n + "_wall_front", at(cx + s / 2 - t / 2, cy, top / 2), {t, s, top}, color);
    addBox(l, f, n + "_wall_back", at(cx - s / 2 + t / 2, cy, top / 2), {t, s, top}, color);
    addBox(l, f, n + "_wall_left", at(cx, cy + s / 2 - t / 2, top / 2), {s - 2 * t, t, top}, color);
    addBox(l, f, n + "_wall_right", at(cx, cy - s / 2 + t / 2, top / 2), {s - 2 * t, t, top}, color);

    l.sensors.push_back({n + "_scoring_sensor", f.pose(at(cx, cy, d::kHubSensorCenterZM)),
                         {d::kHubSensorHalfXYM, d::kHubSensorHalfXYM, d::kHubSensorHalfZM},
                         makeTag(EntityKind::HubSensor, a)});

    HubSpec& hub = l.hubs[index(a)];
    hub.alliance = a;
    hub.center = f.point({cx, cy, 0.f});
    // Exits face the neutral zone (blue: +X). Lateral offsets and fan angles after MapleSim, see field_dimensions.h.
    const float lateral[4] = {1.5f, 0.5f, -0.5f, -1.5f};
    const float fan[4] = {3.f, 1.f, -1.f, -3.f};
    for (std::size_t i = 0; i < 4; ++i) {
        const float yaw = fan[i] * d::kHubExitFanRad, pitch = d::kHubExitPitchRad;
        const Vec3 dir{std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
        hub.exits[i].position = f.point({cx + s / 2 + 0.12f, cy + lateral[i] * d::kHubExitLateralM, d::kHubExitZM});
        hub.exits[i].direction = f.vector(dir);
        hub.exits[i].speedMps = d::kHubExitSpeedMps;
    }
}

void addBumpsAndTrenches(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const float cx = d::kHubCenterXM, cy = d::kHubCenterYM;
    const float bumpOffsetY = d::kHubSizeM / 2 + d::kBumpWidthM / 2;
    const char* color = allianceColor(a);
    for (const float side : {-1.f, 1.f}) {
        BumpSpec b;
        b.name = std::string("bump_") + allianceName(a) + (side < 0 ? "_low_y" : "_high_y");
        b.center = f.point({cx, cy + side * bumpOffsetY, 0.f});
        b.widthM = d::kBumpWidthM;
        b.depthM = d::kBumpDepthM;
        b.ridgeHeightM = d::kBumpHeightM;
        b.edgeHeightM = d::kBumpEdgeHeightM;
        b.color = color;
        l.bumps.push_back(std::move(b));
    }

    // Trench: arm from the guardrail to the bump, post next to the bump; the opening under the arm is free.
    const float armLen = cy - d::kHubSizeM / 2 - d::kBumpWidthM;  // the low-Y bump starts here
    const float postY = d::kTrenchOpeningM + d::kTrenchPostWidthM / 2;
    const float armBottom = d::kTrenchClearanceM, armTop = d::kTrenchArmTopM;
    const std::string n = std::string("trench_") + allianceName(a);
    for (const bool high : {false, true}) {
        const std::string tag = n + (high ? "_high_y" : "_low_y");
        auto yOf = [&](float y) { return high ? d::kFieldWidthM - y : y; };
        addBox(l, f, tag + "_arm", at(cx, yOf(armLen / 2), (armBottom + armTop) / 2),
               {d::kTrenchDepthM, armLen, armTop - armBottom}, color);
        addBox(l, f, tag + "_post", at(cx, yOf(postY), armTop / 2), {d::kTrenchDepthM, d::kTrenchPostWidthM, armTop},
               color);
    }
}

void addTower(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("tower_") + allianceName(a);
    const char* color = allianceColor(a);
    const float cy = d::kTowerCenterYM;
    addBox(l, f, n + "_base", at(d::kTowerBaseDepthM / 2, cy, d::kTowerBaseHeightM / 2),
           {d::kTowerBaseDepthM, d::kTowerBaseWidthM, d::kTowerBaseHeightM}, kSteelColor);
    for (const float side : {-1.f, 1.f})
        addBox(l, f, n + (side < 0 ? "_upright_low_y" : "_upright_high_y"),
               at(d::kUprightCenterXM, cy + side * d::kUprightSpacingM / 2, d::kUprightHeightM / 2),
               {d::kUprightDepthM, d::kUprightThicknessM, d::kUprightHeightM}, color);
    const float rungZ[3] = {d::kLowRungZM, d::kMidRungZM, d::kHighRungZM};
    const char* const rungName[3] = {"_rung_low", "_rung_mid", "_rung_high"};
    for (int i = 0; i < 3; ++i) {
        FieldCylinder c;
        c.name = n + rungName[i];
        // cylinder axis is local Z; rotate it onto the field Y axis
        c.pose = f.pose({{d::kUprightCenterXM, cy, rungZ[i]}, Quat::fromAxisAngle({1.f, 0.f, 0.f}, kPi / 2)});
        c.radiusM = d::kRungRadiusM;
        c.heightM = d::kRungLengthM;
        c.color = color;
        l.cylinders.push_back(std::move(c));
    }
}

void addDepot(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("depot_") + allianceName(a);
    const float cy = d::kDepotCenterYM, bw = d::kDepotBarrierWidthM, h = d::kDepotBarrierHeightM;
    const float half = d::kDepotWidthM / 2;
    addBox(l, f, n + "_front", at(d::kDepotDepthM - bw / 2, cy, h / 2), {bw, d::kDepotWidthM, h}, kSteelColor);
    addBox(l, f, n + "_side_low_y", at(d::kDepotDepthM / 2, cy - half + bw / 2, h / 2), {d::kDepotDepthM, bw, h},
           kSteelColor);
    addBox(l, f, n + "_side_high_y", at(d::kDepotDepthM / 2, cy + half - bw / 2, h / 2), {d::kDepotDepthM, bw, h},
           kSteelColor);
    DepotSpec& spec = l.depots[index(a)];
    spec.alliance = a;
    spec.center = f.point({(d::kDepotDepthM - bw) / 2, cy, 0.f});
    spec.xDir = f.xDir();
}

void addOutpost(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("outpost_") + allianceName(a);
    const float y = d::kOutpostCenterYM;
    // The corral is the region behind the alliance wall where robots push fuel through the base opening.
    const float depth = d::kCorralDepthM, halfW = d::kCorralWidthM / 2;
    l.sensors.push_back({n + "_corral_sensor", f.pose(at(-depth / 2, y, 0.125f)), {depth / 2, halfW, 0.125f},
                         makeTag(EntityKind::CorralSensor, a)});

    OutpostSpec& o = l.outposts[index(a)];
    o.alliance = a;
    const float chuteZ = d::kChuteBottomM + d::kChuteHeightM / 2;
    o.chuteExit = f.point({0.15f, y, chuteZ});
    o.chuteVelocity = f.vector({d::kChuteExitSpeedMps * std::cos(kRampAngleRad), 0.f,
                                -d::kChuteExitSpeedMps * std::sin(kRampAngleRad)});
    o.chuteHalfWidthM = d::kChuteWidthM / 2 - d::kFuelRadiusM;
    o.inwardY = f.vector({0.f, 1.f, 0.f});
}

void addBumpVisuals(sdk::FieldDescription& desc, const BumpSpec& b) {
    const float a = b.depthM * 0.5f;
    const float slabLen = a / std::cos(kRampAngleRad);
    const float thickness = 0.02f;
    const float midZ = (b.edgeHeightM + b.ridgeHeightM) * 0.5f;
    for (const float side : {-1.f, 1.f}) {
        sdk::FieldVisual v;
        v.name = b.name + (side < 0 ? "_ramp_a" : "_ramp_b");
        v.kind = sdk::FieldVisual::Kind::Box;
        v.pose = {{b.center.x + side * a / 2, b.center.y, midZ},         Quat::fromAxisAngle({0.f, 1.f, 0.f}, side * kRampAngleRad)};
        v.sizeM = {slabLen, b.widthM, thickness};
        v.color = b.color;
        desc.visuals.push_back(std::move(v));
    }
    sdk::FieldVisual base;
    base.name = b.name + "_base";
    base.pose = at(b.center.x, b.center.y, b.edgeHeightM / 2);
    base.sizeM = {b.depthM, b.widthM, b.edgeHeightM};
    base.color = b.color;
    desc.visuals.push_back(std::move(base));
}

}  // namespace

FieldLayout buildFieldLayout() {
    namespace d = dim;
    FieldLayout l;
    const float len = d::kFieldLengthM, wid = d::kFieldWidthM, m = d::kGroundMarginM;

    sdk::Material carpet;
    carpet.friction = 0.8f;
    carpet.restitution = 0.f;
    l.boxes.push_back({"ground", at(len / 2, wid / 2, -0.25f), {len + 2 * m, wid + 2 * m, 0.5f}, kCarpetColor, carpet, true});

    sdk::Material rail;
    rail.restitution = 0.3f;
    for (const bool high : {false, true}) {
        const float y = high ? wid + d::kGuardrailThicknessM / 2 : -d::kGuardrailThicknessM / 2;
        l.boxes.push_back({high ? "guardrail_high_y" : "guardrail_low_y", at(len / 2, y, d::kGuardrailHeightM / 2),
                           {len, d::kGuardrailThicknessM, d::kGuardrailHeightM}, kRailColor, rail, true});
    }

    for (const Alliance a : kAlliances) {
        const Frame f(a);
        addAllianceWall(l, f, a);
        addHub(l, f, a);
        addBumpsAndTrenches(l, f, a);
        addTower(l, f, a);
        addDepot(l, f, a);
        addOutpost(l, f, a);
    }

    l.description.name = "REBUILT (2026)";
    l.description.lengthM = len;
    l.description.widthM = wid;
    for (const FieldBox& b : l.boxes) {
        sdk::FieldVisual v;
        v.name = b.name;
        v.kind = sdk::FieldVisual::Kind::Box;
        v.pose = b.pose;
        v.sizeM = b.sizeM;
        v.color = b.color;
        l.description.visuals.push_back(std::move(v));
    }
    for (const FieldCylinder& c : l.cylinders) {
        sdk::FieldVisual v;
        v.name = c.name;
        v.kind = sdk::FieldVisual::Kind::Cylinder;
        v.pose = c.pose;
        v.sizeM = {c.radiusM * 2, c.radiusM * 2, c.heightM};
        v.color = c.color;
        l.description.visuals.push_back(std::move(v));
    }
    for (const BumpSpec& b : l.bumps) addBumpVisuals(l.description, b);
    return l;
}

BumpMesh buildBumpMesh(const BumpSpec& b) {
    const float a = b.depthM * 0.5f, hw = b.widthM * 0.5f;
    // profile (x, z), counter-clockwise seen from +Y
    const float prof[5][2] = {{-a, 0.f}, {a, 0.f}, {a, b.edgeHeightM}, {0.f, b.ridgeHeightM}, {-a, b.edgeHeightM}};
    BumpMesh m;
    m.xyz.reserve(10 * 3);
    for (const float y : {-hw, hw})
        for (const auto& p : prof) {
            m.xyz.push_back(p[0]);
            m.xyz.push_back(y);
            m.xyz.push_back(p[1]);
        }
    auto vtx = [](std::uint32_t side, std::uint32_t i) { return side * 5u + i; };
    auto tri = [&](std::uint32_t i0, std::uint32_t i1, std::uint32_t i2) {
        // wind the triangle so that its normal points away from the (convex) prism's interior
        auto p = [&](std::uint32_t i) { return Vec3{m.xyz[i * 3], m.xyz[i * 3 + 1], m.xyz[i * 3 + 2]}; };
        const Vec3 p0 = p(i0), p1 = p(i1), p2 = p(i2);
        const Vec3 interior{0.f, 0.f, b.edgeHeightM * 0.5f};
        const Vec3 n = sdk::cross(p1 - p0, p2 - p0);
        const Vec3 c = (p0 + p1 + p2) / 3.f;
        if (sdk::dot(n, c - interior) >= 0.f) m.triangles.insert(m.triangles.end(), {i0, i1, i2});
        else m.triangles.insert(m.triangles.end(), {i0, i2, i1});
    };
    for (std::uint32_t i = 1; i + 1 < 5; ++i) {  // end caps: fans of the convex pentagon
        tri(vtx(0, 0), vtx(0, i), vtx(0, i + 1));
        tri(vtx(1, 0), vtx(1, i), vtx(1, i + 1));
    }
    for (std::uint32_t i = 0; i < 5; ++i) {  // side quads
        const std::uint32_t j = (i + 1) % 5;
        tri(vtx(0, i), vtx(0, j), vtx(1, j));
        tri(vtx(0, i), vtx(1, j), vtx(1, i));
    }
    return m;
}

Game2026Field::Game2026Field(sdk::IPhysicsWorld& world, FieldLayout layout, sdk::BodyId body)
    : world_(world), layout_(std::move(layout)), body_(body) {}

Game2026Field::~Game2026Field() {
    if (body_.valid()) world_.destroyBody(body_);
}

sdk::Result<std::unique_ptr<Game2026Field>> Game2026Field::create(sdk::IPhysicsWorld& world, FieldLayout layout) {
    sdk::BodyDesc bd;
    bd.type = sdk::BodyType::Static;
    const sdk::BodyId body = world.createBody(bd);
    if (!body.valid()) return sdk::makeError(sdk::ErrorCode::Internal, "physics world refused the static field body");

    std::unique_ptr<Game2026Field> field(new Game2026Field(world, std::move(layout), body));
    const FieldLayout& l = field->layout_;

    auto add = [&](const sdk::ShapeDesc& desc) -> bool {
        if (!world.addShape(body, desc).valid()) return false;
        ++field->shapeCount_;
        return true;
    };
    auto fail = [](const std::string& what) {
        return sdk::Result<std::unique_ptr<Game2026Field>>(
            sdk::makeError(sdk::ErrorCode::Internal, "physics world refused field shape: " + what));
    };

    for (const FieldBox& b : l.boxes) {
        if (!b.collides) continue;
        sdk::ShapeDesc sd;
        sd.geometry = sdk::BoxShape{b.sizeM * 0.5f};
        sd.local = b.pose;
        sd.material = b.material;
        sd.category = sdk::layer::Field;
        if (!add(sd)) return fail(b.name);
    }
    for (const FieldCylinder& c : l.cylinders) {
        sdk::ShapeDesc sd;
        sd.geometry = sdk::CylinderShape{c.radiusM, c.heightM * 0.5f};
        sd.local = c.pose;
        sd.material = c.material;
        sd.category = sdk::layer::Field;
        if (!add(sd)) return fail(c.name);
    }
    for (const BumpSpec& b : l.bumps) {
        const BumpMesh mesh = buildBumpMesh(b);
        sdk::ShapeDesc sd;
        sd.geometry =
            sdk::MeshShape{mesh.xyz.data(), mesh.xyz.size() / 3, mesh.triangles.data(), mesh.triangles.size() / 3};
        sd.local = at(b.center.x, b.center.y, 0.f);
        sd.category = sdk::layer::Field;
        if (world.addShape(body, sd).valid()) {
            ++field->shapeCount_;
            continue;
        }
        // Adapter without mesh support: base block plus two thick tilted slabs reproduce the profile.
        const float a = b.depthM * 0.5f, slabLen = a / std::cos(kRampAngleRad), thickness = 0.1f;
        sdk::ShapeDesc base;
        base.geometry = sdk::BoxShape{{a, b.widthM * 0.5f, b.edgeHeightM * 0.5f}};
        base.local = at(b.center.x, b.center.y, b.edgeHeightM * 0.5f);
        base.category = sdk::layer::Field;
        if (!add(base)) return fail(b.name);
        for (const float side : {-1.f, 1.f}) {
            const Quat q = Quat::fromAxisAngle({0.f, 1.f, 0.f}, side * kRampAngleRad);
            const Vec3 surface{b.center.x + side * a / 2, b.center.y, (b.edgeHeightM + b.ridgeHeightM) * 0.5f};
            sdk::ShapeDesc slab;
            slab.geometry = sdk::BoxShape{{slabLen * 0.5f, b.widthM * 0.5f, thickness * 0.5f}};
            slab.local = {surface - q.rotate({0.f, 0.f, thickness * 0.5f}), q};
            slab.category = sdk::layer::Field;
            if (!add(slab)) return fail(b.name);
        }
    }
    for (const SensorVolume& s : l.sensors) {
        sdk::ShapeDesc sd;
        sd.geometry = sdk::BoxShape{s.halfExtentsM};
        sd.local = s.pose;
        sd.sensor = true;
        sd.sensorEvents = true;
        sd.category = sdk::layer::Sensor;
        sd.mask = sdk::layer::Fuel;
        sd.userTag = s.tag;
        if (!add(sd)) return fail(s.name);
    }
    return field;
}

}  // namespace wpiemu::plugins::game_2026
