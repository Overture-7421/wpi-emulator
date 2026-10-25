#include "wpiemu/plugins/game_2026/field_layout.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::plugins::game_2026 {

namespace {

using sdk::Pose3;
using sdk::Quat;
using sdk::Vec3;

constexpr float kChuteAngleRad = 0.2617994f;  // 15 degrees

const char* const kRedColor = "#b3261e";
const char* const kBlueColor = "#1f4fb3";
const char* const kSteelColor = "#8a8f99";
const char* const kRailColor = "#cfe3f0";
const char* const kCarpetColor = "#5a5f6b";
const char* const kPlasticColor = "#a9cde6";
const char* const kAluminumColor = "#b9bec7";
const char* const kSponsorColor = "#2b303a";

const char* allianceColor(Alliance a) { return a == Alliance::Red ? kRedColor : kBlueColor; }

// Blue geometry is authored once; red is the same field rotated 180 degrees about the field centre.
class Frame {
public:
    explicit Frame(Alliance a) : red_(a == Alliance::Red) {}
    Vec3 point(const Vec3& p) const { return red_ ? Vec3{dim::kFieldLengthM - p.x, dim::kFieldWidthM - p.y, p.z} : p; }
    Vec3 vector(const Vec3& v) const { return red_ ? Vec3{-v.x, -v.y, v.z} : v; }
    Pose3 pose(const Pose3& p) const {
        if (!red_) return p;
        return {point(p.position), (Quat::fromYaw(sdk::kPiF) * p.rotation).normalized()};
    }
    float xDir() const { return red_ ? -1.f : 1.f; }

private:
    bool red_;
};

Pose3 at(float x, float y, float z) { return Pose3{{x, y, z}, Quat{}}; }

float bumpRampAngleRad(const BumpSpec& b) {
    return std::atan2(b.ridgeHeightM, b.depthM * 0.5f);
}

void addBox(FieldLayout& l, const Frame& f, std::string name, const Pose3& pose, const Vec3& size, const char* color,
            const sdk::Material& material = sdk::materials::kFieldStructure) {
    l.boxes.push_back({std::move(name), f.pose(pose), size, color, material, true});
}

void addAllianceWall(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("wall_") + sdk::allianceKey(a);
    const float t = d::kAllianceWallThicknessM, h = d::kAllianceWallHeightM, w = d::kFieldWidthM;
    const float lo = d::kOutpostCenterYM - d::kCorralOpeningWidthM * 0.5f;
    const float hi = d::kOutpostCenterYM + d::kCorralOpeningWidthM * 0.5f;
    const float gapBottom = d::kCorralOpeningBottomM, gapTop = gapBottom + d::kCorralOpeningHeightM;
    addBox(l, f, n + "_low_y", at(-t / 2, lo / 2, h / 2), {t, lo, h}, kSteelColor);
    addBox(l, f, n + "_outpost_lip", at(-t / 2, (lo + hi) / 2, gapBottom / 2), {t, hi - lo, gapBottom},
           allianceColor(a));
    addBox(l, f, n + "_outpost_top", at(-t / 2, (lo + hi) / 2, (gapTop + h) / 2), {t, hi - lo, h - gapTop},
           allianceColor(a));
    addBox(l, f, n + "_high_y", at(-t / 2, (hi + w) / 2, h / 2), {t, w - hi, h}, kSteelColor);
    l.boxes.back().visible = false;  // drawn by addAllianceWallVisuals: opaque base, see-through sheet, opaque tower wall
}

void addDecor(FieldLayout& l, const Frame& f, std::string name, const Pose3& pose, const Vec3& size, const char* color,
              float opacity = 1.f) {
    sdk::FieldVisual v;
    v.name = std::move(name);
    v.pose = f.pose(pose);
    v.sizeM = size;
    v.color = color;
    v.opacity = opacity;
    l.decor.push_back(std::move(v));
}

std::vector<float> stationCenters(float y0, float y1, int count) {
    std::vector<float> centers;
    centers.reserve(static_cast<std::size_t>(count));
    const float pitch = (y1 - y0) / static_cast<float>(count);
    for (int i = 0; i < count; ++i) centers.push_back(y0 + pitch * (static_cast<float>(i) + 0.5f));
    return centers;
}

void addDriverStationSpots(sdk::FieldDescription& description, const Frame& f) {
    namespace d = dim;
    const float towerLo = d::kTowerCenterYM - d::kTowerWallWidthM * 0.5f;
    const float towerHi = towerLo + d::kTowerWallWidthM;
    const float eyeX = -d::kStationBaseThicknessM - d::kStationShelfDepthM - d::kDriverEyeRoomBehindShelfM;
    const std::vector<float> farCenters = stationCenters(towerHi, d::kFieldWidthM, d::kStationsFarSide);
    for (int i = 0; i < d::kStationsFarSide; ++i) {
        const int number = d::kStationsFarSide - i;
        const float y = farCenters[static_cast<std::size_t>(i)];
        description.driverStations.push_back(
            {Alliance::Blue, number, f.point({eyeX, y, sdk::kDriverEyeHeightM})});
    }
    const std::vector<float> nearCenters = stationCenters(d::kOutpostWidthM, towerLo, d::kStationsOutpostSide);
    for (int i = 0; i < d::kStationsOutpostSide; ++i) {
        const float y = nearCenters[static_cast<std::size_t>(i)];
        const int number = d::kStationsFarSide + i + 1;
        description.driverStations.push_back(
            {Alliance::Blue, number, Frame(Alliance::Blue).point({eyeX, y, sdk::kDriverEyeHeightM})});
    }
    const std::size_t blueCount = description.driverStations.size();
    for (std::size_t i = 0; i < blueCount; ++i) {
        const sdk::DriverStationSpot& blue = description.driverStations[i];
        description.driverStations.push_back(
            {Alliance::Red, blue.number, Frame(Alliance::Red).point(blue.eyeM)});
    }
}

// What the solid `_high_y` collider looks like: stations (base, sponsor band, clear sheet, top rail, shelf), the opaque
// tower wall and the rest of the outpost. The colliders themselves do not change.
void addAllianceWallVisuals(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("wall_") + sdk::allianceKey(a);
    const float t = d::kAllianceWallThicknessM, h = d::kAllianceWallHeightM, w = d::kFieldWidthM;
    const float base = d::kStationBaseHeightM, sponsorTop = d::kStationSponsorTopM, rail = d::kStationRailHeightM;
    const float tb = d::kStationBaseThicknessM, ts = d::kPlasticThicknessM;
    const float openingEnd = d::kOutpostCenterYM + d::kCorralOpeningWidthM * 0.5f;
    const float towerLo = d::kTowerCenterYM - d::kTowerWallWidthM * 0.5f, towerHi = towerLo + d::kTowerWallWidthM;

    addDecor(l, f, n + "_outpost_side", at(-t / 2, (openingEnd + d::kOutpostWidthM) / 2, h / 2),
             {t, d::kOutpostWidthM - openingEnd, h}, kSteelColor);
    addDecor(l, f, n + "_tower_wall", at(-t / 2, (towerLo + towerHi) / 2, d::kTowerWallHeightM / 2),
             {t, d::kTowerWallWidthM, d::kTowerWallHeightM}, kSteelColor);

    auto stations = [&](const std::string& name, float y0, float y1, int count) {
        const float yc = (y0 + y1) / 2, len = y1 - y0, sheetTop = h - rail;
        addDecor(l, f, name + "_base", at(-tb / 2, yc, base / 2), {tb, len, base}, kSteelColor);
        addDecor(l, f, name + "_sponsor", at(-tb / 2, yc, (base + sponsorTop) / 2), {tb, len, sponsorTop - base},
                 kSponsorColor);
        addDecor(l, f, name + "_sheet", at(-ts / 2, yc, (sponsorTop + sheetTop) / 2), {ts, len, sheetTop - sponsorTop},
                 kPlasticColor, d::kPlasticOpacity);
        addDecor(l, f, name + "_rail", at(-tb / 2, yc, h - rail / 2), {tb, len, rail}, kAluminumColor);
        const float pitch = len / static_cast<float>(count);
        const std::vector<float> centers = stationCenters(y0, y1, count);
        const float shelfWidth = std::min(d::kStationShelfWidthM, pitch - 0.05f);
        for (int i = 0; i < count; ++i)
            addDecor(l, f, name + "_shelf_" + std::to_string(i + 1),
                     at(-tb - d::kStationShelfDepthM / 2, centers[static_cast<std::size_t>(i)],
                        base - d::kStationShelfThicknessM / 2),
                     {d::kStationShelfDepthM, shelfWidth, d::kStationShelfThicknessM}, kAluminumColor);
    };
    stations(n + "_stations_near", d::kOutpostWidthM, towerLo, d::kStationsOutpostSide);
    stations(n + "_stations_far", towerHi, w, d::kStationsFarSide);
}

// The guardrail collider is a thin solid box; the real rail is clear polycarbonate between aluminum extrusions.
void addGuardrailVisuals(FieldLayout& l, const std::string& name, float y) {
    namespace d = dim;
    const Frame f(Alliance::Blue);
    const float len = d::kFieldLengthM, h = d::kGuardrailHeightM, e = d::kGuardrailExtrusionM;
    addDecor(l, f, name + "_plastic", at(len / 2, y, h / 2), {len, d::kGuardrailPlasticThicknessM, h - 2 * e},
             kPlasticColor, d::kPlasticOpacity);
    addDecor(l, f, name + "_bottom", at(len / 2, y, e / 2), {len, d::kGuardrailThicknessM, e}, kAluminumColor);
    addDecor(l, f, name + "_top", at(len / 2, y, h - e / 2), {len, d::kGuardrailThicknessM, e}, kAluminumColor);
}

void addHub(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("hub_") + sdk::allianceKey(a);
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
        b.name = std::string("bump_") + sdk::allianceKey(a) + (side < 0 ? "_low_y" : "_high_y");
        b.center = f.point({cx, cy + side * bumpOffsetY, 0.f});
        b.widthM = d::kBumpWidthM;
        b.depthM = d::kBumpDepthM;
        b.ridgeHeightM = d::kBumpHeightM;
        b.color = color;
        l.bumps.push_back(std::move(b));
    }

    // Trench: short arm across the opening, with a post next to the bump.
    const float postY = d::kTrenchOpeningM + d::kTrenchPostWidthM / 2;
    const float armBottom = d::kTrenchClearanceM, armTop = d::kTrenchArmTopM;
    const std::string n = std::string("trench_") + sdk::allianceKey(a);
    for (const bool high : {false, true}) {
        const std::string tag = n + (high ? "_high_y" : "_low_y");
        auto yOf = [&](float y) { return high ? d::kFieldWidthM - y : y; };
        addBox(l, f, tag + "_arm", at(cx, yOf(d::kTrenchOpeningM / 2), (armBottom + armTop) / 2),
               {d::kTrenchArmLengthM, d::kTrenchOpeningM, armTop - armBottom}, color);
        addBox(l, f, tag + "_post", at(cx, yOf(postY), armTop / 2), {d::kTrenchDepthM, d::kTrenchPostWidthM, armTop},
               color);
    }
}

void addTower(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("tower_") + sdk::allianceKey(a);
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
        c.pose = f.pose({{d::kUprightCenterXM, cy, rungZ[i]}, Quat::fromAxisAngle({1.f, 0.f, 0.f}, sdk::kPiF / 2)});
        c.radiusM = d::kRungRadiusM;
        c.heightM = d::kRungLengthM;
        c.color = color;
        l.cylinders.push_back(std::move(c));
    }
}

void addDepot(FieldLayout& l, const Frame& f, Alliance a) {
    namespace d = dim;
    const std::string n = std::string("depot_") + sdk::allianceKey(a);
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
    const std::string n = std::string("outpost_") + sdk::allianceKey(a);
    const float y = d::kOutpostCenterYM;
    // The corral is the region behind the alliance wall where robots push fuel through the base opening.
    const float depth = d::kCorralDepthM, halfW = d::kCorralWidthM / 2;
    l.sensors.push_back({n + "_corral_sensor", f.pose(at(-depth / 2, y, 0.125f)), {depth / 2, halfW, 0.125f},
                         makeTag(EntityKind::CorralSensor, a)});

    OutpostSpec& o = l.outposts[index(a)];
    o.alliance = a;
    const float chuteZ = d::kChuteBottomM + d::kChuteHeightM / 2;
    o.chuteExit = f.point({0.15f, y, chuteZ});
    o.chuteVelocity = f.vector({d::kChuteExitSpeedMps * std::cos(kChuteAngleRad), 0.f,
                                -d::kChuteExitSpeedMps * std::sin(kChuteAngleRad)});
    o.chuteHalfWidthM = d::kChuteWidthM / 2 - d::kFuelRadiusM;
    o.inwardY = f.vector({0.f, 1.f, 0.f});
}

void addBumpVisuals(sdk::FieldDescription& desc, const BumpSpec& b) {
    const float a = b.depthM * 0.5f;
    const float angle = bumpRampAngleRad(b);
    const float slabLen = a / std::cos(angle);
    const float thickness = 0.02f;
    for (const float side : {-1.f, 1.f}) {
        sdk::FieldVisual v;
        v.name = b.name + (side < 0 ? "_ramp_a" : "_ramp_b");
        v.kind = sdk::FieldVisual::Kind::Box;
        const Quat q = Quat::fromAxisAngle({0.f, 1.f, 0.f}, side * angle);
        const Vec3 surface{b.center.x + side * a / 2, b.center.y, b.ridgeHeightM * 0.5f};
        v.pose = {surface - q.rotate({0.f, 0.f, thickness * 0.5f}), q};
        v.sizeM = {slabLen, b.widthM, thickness};
        v.color = b.color;
        desc.visuals.push_back(std::move(v));
    }
}

}  // namespace

FieldLayout buildFieldLayout() {
    namespace d = dim;
    FieldLayout l;
    const float len = d::kFieldLengthM, wid = d::kFieldWidthM, m = d::kGroundMarginM;

    l.boxes.push_back({"ground", at(len / 2, wid / 2, -0.25f), {len + 2 * m, wid + 2 * m, 0.5f}, kCarpetColor,
                       sdk::materials::kCarpet, true});

    for (const bool high : {false, true}) {
        const float y = high ? wid + d::kGuardrailThicknessM / 2 : -d::kGuardrailThicknessM / 2;
        const std::string name = high ? "guardrail_high_y" : "guardrail_low_y";
        l.boxes.push_back({name, at(len / 2, y, d::kGuardrailHeightM / 2),
                           {len, d::kGuardrailThicknessM, d::kGuardrailHeightM}, kRailColor,
                           sdk::materials::kFieldStructure, true, false});
        addGuardrailVisuals(l, name, y);
    }

    for (const Alliance a : kAlliances) {
        const Frame f(a);
        addAllianceWall(l, f, a);
        addAllianceWallVisuals(l, f, a);
        addHub(l, f, a);
        addBumpsAndTrenches(l, f, a);
        addTower(l, f, a);
        addDepot(l, f, a);
        addOutpost(l, f, a);
    }

    l.description.name = "REBUILT (2026)";
    l.description.lengthM = len;
    l.description.widthM = wid;
    addDriverStationSpots(l.description, Frame(Alliance::Blue));
    for (const FieldBox& b : l.boxes) {
        if (!b.visible) continue;
        sdk::FieldVisual v;
        v.name = b.name;
        v.kind = sdk::FieldVisual::Kind::Box;
        v.pose = b.pose;
        v.sizeM = b.sizeM;
        v.color = b.color;
        l.description.visuals.push_back(std::move(v));
    }
    l.description.visuals.insert(l.description.visuals.end(), l.decor.begin(), l.decor.end());
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
    const float prof[3][2] = {{-a, 0.f}, {a, 0.f}, {0.f, b.ridgeHeightM}};
    BumpMesh m;
    m.xyz.reserve(6 * 3);
    for (const float y : {-hw, hw})
        for (const auto& p : prof) {
            m.xyz.push_back(p[0]);
            m.xyz.push_back(y);
            m.xyz.push_back(p[1]);
        }
    auto vtx = [](std::uint32_t side, std::uint32_t i) { return side * 3u + i; };
    auto tri = [&](std::uint32_t i0, std::uint32_t i1, std::uint32_t i2) {
        // wind the triangle so that its normal points away from the (convex) prism's interior
        auto p = [&](std::uint32_t i) { return Vec3{m.xyz[i * 3], m.xyz[i * 3 + 1], m.xyz[i * 3 + 2]}; };
        const Vec3 p0 = p(i0), p1 = p(i1), p2 = p(i2);
        const Vec3 interior{0.f, 0.f, b.ridgeHeightM / 3.f};
        const Vec3 n = sdk::cross(p1 - p0, p2 - p0);
        const Vec3 c = (p0 + p1 + p2) / 3.f;
        if (sdk::dot(n, c - interior) >= 0.f) m.triangles.insert(m.triangles.end(), {i0, i1, i2});
        else m.triangles.insert(m.triangles.end(), {i0, i2, i1});
    };
    tri(vtx(0, 0), vtx(0, 1), vtx(0, 2));
    tri(vtx(1, 0), vtx(1, 1), vtx(1, 2));
    for (std::uint32_t i = 0; i < 3; ++i) {  // side quads
        const std::uint32_t j = (i + 1) % 3;
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
        // Adapter without mesh support: two thick tilted slabs reproduce the driving surface.
        const float angle = bumpRampAngleRad(b);
        const float a = b.depthM * 0.5f, slabLen = a / std::cos(angle), thickness = 0.1f;
        for (const float side : {-1.f, 1.f}) {
            const Quat q = Quat::fromAxisAngle({0.f, 1.f, 0.f}, side * angle);
            const Vec3 surface{b.center.x + side * a / 2, b.center.y, b.ridgeHeightM * 0.5f};
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
        sd.mask = sdk::layer::Fuel | kShotFuelLayer;
        sd.userTag = s.tag;
        if (!add(sd)) return fail(s.name);
    }
    return field;
}

}  // namespace wpiemu::plugins::game_2026
