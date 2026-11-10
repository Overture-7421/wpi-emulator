// Robot-versus-robot contact through the real Box3D world: a remote robot (physicsRemoteChassisFactory) driven into the host
// robot (physicsChassisBuilder) head-on and at an angle, on open carpet, against a wall and over a field bump. Both chassis are
// the swerve sample's (0.775 x 0.80 x 0.15 m frame 0.03 m off the floor, 61 kg, four 2 in-radius wheel pads), so the numbers
// are the ones a practice match sees.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "remote_robot_rig.h"

using namespace remote_test;
using proto::AllianceStation;

namespace {

constexpr double kPiRad = 3.14159265358979;
constexpr float kFrameHeightM = 0.15f;
constexpr float kClearanceM = 0.03f;
constexpr float kHullTopM = kClearanceM + kFrameHeightM;          // top of a robot's frame above the floor
constexpr double kInscribedSumM = 0.3875 + 0.3875;                // two robots' inscribed radii: closer than this = overlapping

// REBUILT bump (field_dimensions.h kBump*): 6.513 in ridge, 15 degree ramps, 73 in wide. Two tilted slabs, as the 2026 field
// builds it when the world takes no meshes.
constexpr float kBumpHeightM = 0.16543f;
constexpr float kBumpHalfDepthM = kBumpHeightM / 0.26794919f;
constexpr float kBumpWidthM = 1.854f;

RobotConfig sampleSwerve(const char* name, float frameHeightM = kFrameHeightM) {
    RobotConfig c;
    c.name = name;
    c.chassis.sizeM = {0.775f, 0.80f, frameHeightM};
    c.chassis.massKg = 61.235f;
    c.chassis.comHeightM = 0.20f;
    c.chassis.groundClearanceM = kClearanceM;
    c.drivetrain.spec.kind = drivetrain_kind::kSwerve;
    c.drivetrain.spec.tireModel = "brush";
    const struct {
        const char* name;
        float x, y;
    } modules[] = {{"fl", 0.2826f, 0.2762f}, {"fr", 0.2826f, -0.2762f}, {"bl", -0.2826f, 0.2762f}, {"br", -0.2826f, -0.2762f}};
    for (const auto& m : modules) {
        WheelSpec w;
        w.name = m.name;
        w.kind = WheelKind::Swerve;
        w.steered = true;
        w.positionM = {m.x, m.y, 0.0508f};
        w.radiusM = 0.0508f;
        w.driveGearRatio = 7.03f;
        w.steerGearRatio = 287.0f / 11.0f;
        w.steerMotor = motors::falcon500();
        c.drivetrain.spec.wheels.push_back(w);
    }
    return c;
}

struct Scenario {
    double speed = 2.5;           // m/s along the approach direction, robot relative
    double approachDeg = 0.0;     // direction of the host from the remote (0 = straight ahead)
    double hostYawDeg = 0.0;
    double gapM = 3.0;            // host distance from the remote's start (along the approach direction)
    int ticks = 900;
    float frameHeightM = kFrameHeightM;
    double hostXM = 0.0;          // absolute host x when > 0 (otherwise `gapM` ahead of the remote)
    bool bump = false;            // a bump between them, its far foot `bumpFootToHostM` short of the host's centre
    double bumpFootToHostM = 0.0;
};

struct Outcome {
    double zRiseHost = 0, zRiseRemote = 0;   // peak height above the rest pose
    double tiltHost = 0, tiltRemote = 0;     // peak angle of the body's up axis from vertical, rad
    double minCentreDistance = 1e9;          // planar
    double minLevelDistance = 1e9;           // planar, over ticks where the two frames are at the same level
    double minDistanceBelowFlat = 1e9;       // smallest remote z rise while the centres are closer than the flat contact distance
    double finalCentreDistance = 0;
};

double tiltOf(const Pose3& p) {
    const Vec3 up = p.rotation.rotate({0.f, 0.f, 1.f});
    return std::acos(std::clamp<double>(up.z, -1.0, 1.0));
}

// A triangular prism (the 2026 field's bump mesh): ridge along Y, wound so every triangle faces away from the interior.
void addBump(IPhysicsWorld& world, float cx, float cy) {
    const float a = kBumpHalfDepthM, hw = kBumpWidthM * 0.5f;
    const float prof[3][2] = {{-a, 0.f}, {a, 0.f}, {0.f, kBumpHeightM}};
    std::vector<float> xyz;
    for (const float y : {-hw, hw})
        for (const auto& p : prof) xyz.insert(xyz.end(), {p[0], y, p[1]});
    std::vector<std::uint32_t> tris;
    auto tri = [&](std::uint32_t i0, std::uint32_t i1, std::uint32_t i2) {
        auto p = [&](std::uint32_t i) { return Vec3{xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]}; };
        const Vec3 n = cross(p(i1) - p(i0), p(i2) - p(i0));
        const Vec3 c = (p(i0) + p(i1) + p(i2)) / 3.f;
        if (dot(n, c - Vec3{0.f, 0.f, kBumpHeightM / 3.f}) >= 0.f) tris.insert(tris.end(), {i0, i1, i2});
        else tris.insert(tris.end(), {i0, i2, i1});
    };
    tri(0, 1, 2);
    tri(3, 4, 5);
    for (std::uint32_t i = 0; i < 3; ++i) {
        const std::uint32_t j = (i + 1) % 3;
        tri(i, j, 3 + j);
        tri(i, 3 + j, 3 + i);
    }
    BodyDesc bd;
    bd.type = BodyType::Static;
    const BodyId body = world.createBody(bd);
    REQUIRE(body.valid());
    ShapeDesc sd;
    sd.geometry = MeshShape{xyz.data(), xyz.size() / 3, tris.data(), tris.size() / 3};
    sd.local = Pose3{{cx, cy, 0.f}, Quat::identity()};
    sd.category = layer::Field;
    REQUIRE(world.addShape(body, sd).valid());
}
// The remote starts at its station and drives at `speed`; the host stands `gapM` ahead of it along the approach direction.
Outcome collide(const Scenario& s) {
    const RobotConfig remoteSpec = sampleSwerve("Remote", s.frameHeightM);
    const RobotConfig hostSpec = sampleSwerve("Host", s.frameHeightM);
    Vec3 start{};
    {
        Rig probe;
        probe.add(1, AllianceStation::Blue2, remoteSpec);
        start = probe.poseOf(1).position;
    }
    const double a = s.approachDeg * kPiRad / 180.0;
    RigOptions options;
    options.hostSpec = &hostSpec;
    options.hostYaw = static_cast<float>(s.hostYawDeg * kPiRad / 180.0);
    options.hostX = static_cast<float>(s.hostXM > 0.0 ? s.hostXM : start.x + s.gapM * std::cos(a));
    options.hostY = static_cast<float>(start.y + s.gapM * std::sin(a));
    Rig rig(options);
    rig.add(1, AllianceStation::Blue2, remoteSpec);
    if (s.bump) addBump(rig.session->world(), options.hostX - static_cast<float>(s.bumpFootToHostM) - kBumpHalfDepthM, options.hostY);
    ScriptedClient client(rig, 1, remoteSpec);
    IPhysicsWorld& world = rig.session->world();
    const BodyId hostBody = rig.session->chassisBody();
    const double hostRestZ = world.pose(hostBody).position.z;
    const double remoteRestZ = rig.poseOf(1).position.z;
    const ChassisSpeeds speeds{static_cast<float>(s.speed * std::cos(a)), static_cast<float>(s.speed * std::sin(a)), 0.f};
    Outcome out;
    for (int i = 0; i < s.ticks; ++i) {
        if (i % 4 == 0) REQUIRE(client.send(speeds, rig.nextTick()).ok());
        rig.tick();
        const Pose3 h = world.pose(hostBody);
        const Pose3 r = rig.poseOf(1);
        const double d = std::hypot(h.position.x - r.position.x, h.position.y - r.position.y);
        const double riseH = h.position.z - hostRestZ, riseR = r.position.z - remoteRestZ;
        out.zRiseHost = std::max(out.zRiseHost, riseH);
        out.zRiseRemote = std::max(out.zRiseRemote, riseR);
        out.tiltHost = std::max(out.tiltHost, tiltOf(h));
        out.tiltRemote = std::max(out.tiltRemote, tiltOf(r));
        out.minCentreDistance = std::min(out.minCentreDistance, d);
        if (std::abs(r.position.z - h.position.z) < 0.5 * kFrameHeightM) out.minLevelDistance = std::min(out.minLevelDistance, d);
        if (d < kInscribedSumM - 0.02) out.minDistanceBelowFlat = std::min<double>(out.minDistanceBelowFlat, r.position.z - h.position.z);
        out.finalCentreDistance = d;
        
    }
    return out;
}

}  // namespace

// On open carpet a robot driven into another one pushes against it: neither rides up (the wheel pads and frames meet at the
// same height), nothing pitches or rolls noticeably and the frames never overlap.
TEST_CASE("robot contact: on open carpet a remote robot pushes the host robot without riding up or overlapping") {
    for (const double speed : {1.0, 2.5, 3.5})
        for (const double approachDeg : {0.0, 20.0, 45.0})
            for (const double hostYawDeg : {0.0, 45.0}) {
                Scenario s;
                s.speed = speed;
                s.approachDeg = approachDeg;
                s.hostYawDeg = hostYawDeg;
                const Outcome o = collide(s);
                CAPTURE(speed);
                CAPTURE(approachDeg);
                CAPTURE(hostYawDeg);
                CHECK(o.zRiseRemote < 0.03);
                CHECK(o.zRiseHost < 0.03);
                CHECK(o.tiltRemote < 5.0 * kPiRad / 180.0);
                CHECK(o.tiltHost < 5.0 * kPiRad / 180.0);
                CHECK(o.minCentreDistance > kInscribedSumM - 0.035);
            }
}

TEST_CASE("robot contact: a host robot pinned against the wall is pushed without riding up or overlapping") {
    for (const double speed : {2.5, 3.5})
        for (const double hostYawDeg : {0.0, 45.0, 90.0}) {
            Scenario s;
            s.speed = speed;
            s.hostYawDeg = hostYawDeg;
            s.hostXM = default_field::kLengthM - 0.45;
            s.ticks = 2400;
            const Outcome o = collide(s);
            CAPTURE(speed);
            CAPTURE(hostYawDeg);
            CHECK(o.zRiseRemote < 0.02);
            CHECK(o.zRiseHost < 0.05);   // a corner against the wall tips the host a little
            CHECK(o.tiltRemote < 5.0 * kPiRad / 180.0);
            CHECK(o.tiltHost < 8.0 * kPiRad / 180.0);
            CHECK(o.minCentreDistance > kInscribedSumM - 0.035);
        }
}

// What a practice match showed (a remote robot "riding up onto" the host robot, centres 0.44 m apart, remote z 0.35, host z 0.20):
// it needs a lifting field element. A robot that climbs the 0.165 m bump ridge has its frame bottom at 0.165 + 0.03 m, above the
// 0.18 m top of the sample's frame, so it passes over a robot standing at the ramp's foot; it is not a robot-to-robot contact
// defect (see the open carpet cases above). A frame as tall as a bumper zone (0.20 m, top 0.23 m) stops the robot instead.
TEST_CASE("robot contact: a robot that climbs a bump rides over a lower robot at the ramp foot unless its frame is taller") {
    SUBCASE("the sample's 0.15 m frame: the remote goes over the host") {
        for (const double foot : {-0.2, 0.0, 0.2}) {
            Scenario s;
            s.bump = true;
            s.gapM = 8.0;
            s.ticks = 1800;
            s.bumpFootToHostM = foot;
            const Outcome o = collide(s);
            CAPTURE(foot);
            CHECK(o.zRiseRemote > kBumpHeightM + 0.01);       // higher than the ridge alone: it is resting on the host
            CHECK(o.zRiseRemote < kBumpHeightM + 0.1);
            CHECK(o.minCentreDistance < kInscribedSumM - 0.07);
            CHECK(o.minDistanceBelowFlat > kFrameHeightM - 0.06);   // whenever the centres overlap the remote's frame is above the host's
            CHECK(o.minDistanceBelowFlat < 1e8);
        }
    }
    SUBCASE("a 0.20 m frame: the remote pushes the host and never overlaps it") {
        for (const double foot : {-0.2, 0.0, 0.2, 0.4}) {
            Scenario s;
            s.bump = true;
            s.gapM = 8.0;
            s.ticks = 1800;
            s.bumpFootToHostM = foot;
            s.frameHeightM = 0.20f;
            const Outcome o = collide(s);
            CAPTURE(foot);
            CHECK(o.zRiseRemote < kBumpHeightM + 0.01);
            CHECK(o.minCentreDistance > kInscribedSumM - 0.035);
        }
    }
}