#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "wpiemu/sdk/match/robot_appearance.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"

using namespace wpiemu::sdk;
using namespace wpiemu::sdk::match;

namespace {

constexpr float kMm = 0.0006f;  // a little over one wire step (0.5 mm) plus float rounding

ChassisSpec chassis() {
    ChassisSpec c;
    c.sizeM = {0.7f, 0.6f, 0.2f};
    c.comHeightM = 0.25f;
    c.groundClearanceM = 0.05f;
    c.comOffsetM = {0.05f, -0.1f, 0.f};
    PrimitiveSpec bumper;
    bumper.name = "bumper";
    bumper.kind = PrimitiveKind::Box;
    bumper.local = {Vec3{0.3f, 0.f, -0.1f}, Quat::fromEuler(0.f, 0.f, 0.5f)};
    bumper.sizeM = {0.8f, 0.04f, 0.09f};
    bumper.color = "#ff3300";
    PrimitiveSpec drum;
    drum.name = "drum";
    drum.kind = PrimitiveKind::Cylinder;
    drum.radiusM = 0.05f;
    drum.heightM = 0.25f;
    drum.color = "#0A0b0C99";
    c.primitives = {bumper, drum};
    return c;
}

WheelSpec wheel(WheelKind kind, float x, float y, float r = 0.0508f) {
    WheelSpec w;
    w.kind = kind;
    w.positionM = {x, y, 0.f};
    w.radiusM = r;
    w.widthM = 0.038f;
    return w;
}

ZoneConfig zone(const char* kind, Pose3 local, Vec3 size) {
    ZoneConfig z;
    z.id = kind;
    z.kind = kind;
    z.local = local;
    z.sizeM = size;
    return z;
}

float angleBetween(const Quat& a, const Quat& b) {
    const float dot = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.f * std::acos(std::min(1.f, dot));
}

}  // namespace

TEST_CASE("appearance: the encoded sizes are the documented constants") {
    CHECK(kWireAppearanceChassisBytes == 14);
    CHECK(kWireAppearancePrimitiveBytes == 23);
    CHECK(kWireAppearanceWheelBytes == 12);
    CHECK(kWireAppearanceZoneMinBytes == 21);
}

TEST_CASE("appearance: colors parse as #rrggbb or #rrggbbaa and anything else is the default grey") {
    std::uint8_t r = 0, g = 0, b = 0;
    parseAppearanceColor("#ff3300", r, g, b);
    CHECK((r == 0xFF && g == 0x33 && b == 0x00));
    parseAppearanceColor("#0A0b0C99", r, g, b);
    CHECK((r == 0x0A && g == 0x0B && b == 0x0C));
    for (const char* bad : {"", "ff3300", "#ff33", "#gg0000", "#ff33000", "#ff3300ff0", "red"}) {
        CAPTURE(bad);
        parseAppearanceColor(bad, r, g, b);
        CHECK((r == 0x80 && g == 0x80 && b == 0x80));
    }
}

TEST_CASE("appearance: the builder describes the chassis box in the COM-origin chassis frame") {
    const ChassisSpec c = chassis();
    const std::vector<WheelSpec> wheels = {wheel(WheelKind::Swerve, 0.25f, 0.25f)};
    const std::vector<Vec3> hubs = {Vec3{0.25f, 0.25f, -0.2f}};
    const RobotAppearance a = makeRobotAppearance(c, wheels, hubs, {});
    CHECK(a.sizeXMm == 700);
    CHECK(a.sizeYMm == 600);
    CHECK(a.sizeZMm == 200);
    CHECK(a.centreXMm == -50);
    CHECK(a.centreYMm == 100);
    CHECK(a.centreZMm == -100);  // clearance 0.05 + half height 0.1 - COM height 0.25
    CHECK(a.comHeightMm == 250);
    CHECK_FALSE(a.empty());
    CHECK(appearanceWellFormed(a));
}

TEST_CASE("appearance: wheels keep the spec order, kind, steering, hub, radius and width") {
    const std::vector<WheelSpec> wheels = {wheel(WheelKind::Standard, 0.3f, 0.25f), wheel(WheelKind::Omni, 0.3f, -0.25f, 0.04f),
                                           wheel(WheelKind::Mecanum, -0.3f, 0.25f), wheel(WheelKind::Swerve, -0.3f, -0.25f)};
    std::vector<WheelSpec> steered = wheels;
    steered[0].steered = true;
    std::vector<Vec3> hubs;
    for (const WheelSpec& w : wheels) hubs.push_back({w.positionM.x, w.positionM.y, w.radiusM - 0.25f});
    const RobotAppearance a = makeRobotAppearance(chassis(), steered, hubs, {});
    REQUIRE(a.wheels.size() == 4);
    CHECK(a.wheels[0].kind == WheelKind::Standard);
    CHECK(a.wheels[0].steered);   // flagged
    CHECK_FALSE(a.wheels[1].steered);
    CHECK_FALSE(a.wheels[2].steered);
    CHECK(a.wheels[3].kind == WheelKind::Swerve);
    CHECK(a.wheels[3].steered);   // implied by the kind
    CHECK(a.wheels[1].xMm == 300);
    CHECK(a.wheels[1].yMm == -250);
    CHECK(a.wheels[1].zMm == -210);
    CHECK(a.wheels[1].radiusMm == 40);
    CHECK(a.wheels[1].widthMm == 38);
    CHECK(a.wheels[0].radiusMm == 51);
}

TEST_CASE("appearance: primitives are described in the chassis frame with their extents and color") {
    const RobotAppearance a = makeRobotAppearance(chassis(), {}, {}, {});
    REQUIRE(a.primitives.size() == 2);
    CHECK(a.primitives[0].kind == PrimitiveKind::Box);
    CHECK(a.primitives[0].pose.zoneIndex == 0);
    CHECK(a.primitives[0].pose.xMm == 300);
    CHECK(a.primitives[0].pose.zMm == -100);
    CHECK(a.primitives[0].extentXMm == 800);
    CHECK(a.primitives[0].extentYMm == 40);
    CHECK(a.primitives[0].extentZMm == 90);
    CHECK(a.primitives[0].r == 0xFF);
    CHECK(a.primitives[1].kind == PrimitiveKind::Cylinder);
    CHECK(a.primitives[1].pose.zoneIndex == 1);
    CHECK(a.primitives[1].extentXMm == 100);  // diameter
    CHECK(a.primitives[1].extentYMm == 100);
    CHECK(a.primitives[1].extentZMm == 250);  // height
    CHECK(a.primitives[1].b == 0x0C);
    // What the rig gets back is the primitive, with its color, to wire precision.
    const PrimitiveSpec back = toPrimitiveSpec(a.primitives[0]);
    CHECK(back.kind == PrimitiveKind::Box);
    CHECK(back.color == "#ff3300");
    CHECK(back.sizeM.x == doctest::Approx(0.8f));
    CHECK(back.local.position.x == doctest::Approx(0.3f).epsilon(0.002));
    CHECK(angleBetween(back.local.rotation, chassis().primitives[0].local.rotation) < 3.f * 4.8e-5f + 1e-4f);
    const PrimitiveSpec cylinder = toPrimitiveSpec(a.primitives[1]);
    CHECK(cylinder.kind == PrimitiveKind::Cylinder);
    CHECK(cylinder.radiusM == doctest::Approx(0.05f));
    CHECK(cylinder.heightM == doctest::Approx(0.25f));
    CHECK(cylinder.color == "#0a0b0c");
}

TEST_CASE("appearance: lists are cut to their wire caps in spec order") {
    ChassisSpec c = chassis();
    c.primitives.assign(kMaxAppearancePrimitives + 5, PrimitiveSpec{});
    std::vector<WheelSpec> wheels(kMaxWheelsPerRobot + 3, wheel(WheelKind::Omni, 0.f, 0.f));
    const std::vector<Vec3> hubs(wheels.size(), Vec3{});
    std::vector<ZoneConfig> zones(kMaxZonesPerRobot + 4, zone("k", Pose3{}, Vec3{}));
    const RobotAppearance a = makeRobotAppearance(c, wheels, hubs, zones);
    CHECK(a.primitives.size() == kMaxAppearancePrimitives);
    CHECK(a.wheels.size() == kMaxWheelsPerRobot);
    CHECK(a.zones.size() == kMaxZonesPerRobot);
    CHECK(appearanceWellFormed(a));
    // Wheels without a hub position are not described.
    CHECK(makeRobotAppearance(c, wheels, std::vector<Vec3>(2), {}).wheels.size() == 2);
}

TEST_CASE("appearance: building is total for non-finite and huge values") {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    ChassisSpec c;
    c.sizeM = {nan, inf, 1e9f};
    c.comHeightM = -inf;
    c.comOffsetM = {nan, 1e9f, 0.f};
    const std::vector<WheelSpec> wheels = {wheel(WheelKind::Standard, nan, inf, 1e9f)};
    const std::vector<ZoneConfig> zones = {zone("k", Pose3{Vec3{nan, inf, -inf}, Quat{nan, 0.f, 0.f, 0.f}}, Vec3{nan, -1.f, 1e9f})};
    const RobotAppearance a = makeRobotAppearance(c, wheels, std::vector<Vec3>{Vec3{nan, inf, -inf}}, zones);
    CHECK(a.sizeXMm == 0);
    CHECK(a.sizeYMm == 0);  // a non-finite value is 0 (quantize.h)
    CHECK(a.sizeZMm == 0xFFFF);
    CHECK(a.wheels[0].radiusMm == 0xFFFF);
    CHECK(a.wheels[0].xMm == 0);
    CHECK(a.zones[0].sizeYMm == 0);
    CHECK(appearanceWellFormed(a));
    // And so is using it: every metre value is finite.
    const ChassisSpec back = toChassisSpec(a);
    CHECK(std::isfinite(back.sizeM.x));
    CHECK(std::isfinite(back.groundClearanceM));
    CHECK(std::isfinite(appearanceZoneRestPose(a.zones[0]).position.x));
    CHECK(toWheelSpecs(a)[0].radiusM >= 0.001f);
}

TEST_CASE("appearance: a long zone kind is cut at kMaxNameBytes") {
    const std::vector<ZoneConfig> zones = {zone(std::string(kMaxNameBytes + 20, 'k').c_str(), Pose3{}, Vec3{})};
    const RobotAppearance a = makeRobotAppearance(chassis(), {}, {}, zones);
    CHECK(a.zones[0].kind.size() == kMaxNameBytes);
    CHECK(appearanceWellFormed(a));
}

TEST_CASE("appearance: a zone's rest pose is remoteZoneSpecPose of the zone, on the chassis or on a mechanism") {
    ZoneConfig onChassis = zone("game.pickup", Pose3{Vec3{0.4f, -0.05f, 0.1f}, Quat::fromEuler(0.1f, -0.2f, 0.7f)}, Vec3{0.3f, 0.6f, 0.15f});
    ZoneConfig onArm = zone("game.muzzle", Pose3{Vec3{-0.1f, 0.f, 0.5f}, Quat::fromEuler(0.f, -0.5f, 0.f)}, Vec3{0.1f, 0.1f, 0.1f});
    onArm.attachment.parentMechanismId = "arm";
    onArm.attachment.parentLink = "tip";
    const std::vector<ZoneConfig> zones = {onChassis, onArm};
    const RobotAppearance a = makeRobotAppearance(chassis(), {}, {}, zones);
    REQUIRE(a.zones.size() == 2);
    CHECK_FALSE(a.zones[0].onMechanism);
    CHECK(a.zones[1].onMechanism);
    CHECK(a.zones[0].kind == "game.pickup");
    CHECK(a.zones[1].pose.zoneIndex == 1);
    CHECK(a.zones[0].sizeYMm == 600);

    const std::vector<Pose3> rest = appearanceZoneRestPoses(a);
    REQUIRE(rest.size() == 2);
    for (std::size_t i = 0; i < zones.size(); ++i) {
        CAPTURE(i);
        const Pose3 expected = remoteZoneSpecPose(zones[i]);
        CHECK(length(rest[i].position - expected.position) < kMm * 2.f);
        CHECK(angleBetween(rest[i].rotation, expected.rotation) < static_cast<float>(kMaxZonePoseRotationErrorRad) + 1e-4f);
        // The same pose through the one wire form: nothing differs after quantization.
        CHECK(makeZonePoseInput(i, rest[i]) == makeZonePoseInput(i, expected));
    }
    // toZoneConfig is what remoteZoneSpecPose runs over; a mechanism zone is pinned at its local pose.
    CHECK(toZoneConfig(a.zones[1]).local.position.z == doctest::Approx(0.5f));
}

TEST_CASE("appearance: zones at their rest pose are never reported as moved by the snapshot selection") {
    std::vector<ZoneConfig> zones;
    for (int i = 0; i < 6; ++i)
        zones.push_back(zone("k", Pose3{Vec3{0.1f * static_cast<float>(i) + 0.00037f, -0.2f, 0.3f}, Quat::fromEuler(0.01f * static_cast<float>(i), 0.3f, -1.f)}, Vec3{0.1f, 0.1f, 0.1f}));
    const RobotAppearance a = makeRobotAppearance(chassis(), {}, {}, zones);
    std::vector<Pose3> hostRest;  // what the host holds: the spec's own poses
    for (const ZoneConfig& z : zones) hostRest.push_back(remoteZoneSpecPose(z));
    const std::vector<Pose3> clientRest = appearanceZoneRestPoses(a);
    CHECK(selectSnapshotZonePoses(hostRest, hostRest, 0).empty());
    // A client that took its rest poses from the roster agrees with the host about which zones moved.
    std::vector<Pose3> effective = hostRest;
    effective[2].position.x += 0.2f;
    const std::vector<ZonePoseInput> moved = selectSnapshotZonePoses(hostRest, effective, 0);
    REQUIRE(moved.size() == 1);
    CHECK(moved[0].zoneIndex == 2);
    CHECK(selectSnapshotZonePoses(clientRest, hostRest, 0).empty());
}

TEST_CASE("appearance: toChassisSpec and toWheelSpecs recover what a rig draws, to wire precision") {
    const ChassisSpec c = chassis();
    std::vector<WheelSpec> wheels = {wheel(WheelKind::Swerve, 0.25f, 0.25f), wheel(WheelKind::Omni, -0.25f, -0.25f, 0.04f)};
    wheels[1].positionM.z = 0.05f;  // an explicit hub height
    std::vector<Vec3> hubs = {Vec3{0.25f, 0.25f, 0.0508f - c.comHeightM}, Vec3{-0.25f, -0.25f, 0.05f - c.comHeightM}};
    const RobotAppearance a = makeRobotAppearance(c, wheels, hubs, {});

    const ChassisSpec back = toChassisSpec(a);
    CHECK(back.sizeM.x == doctest::Approx(c.sizeM.x));
    CHECK(back.sizeM.z == doctest::Approx(c.sizeM.z));
    CHECK(back.comHeightM == doctest::Approx(c.comHeightM));
    CHECK(back.comOffsetM.x == doctest::Approx(c.comOffsetM.x));
    CHECK(back.comOffsetM.y == doctest::Approx(c.comOffsetM.y));
    CHECK(back.groundClearanceM == doctest::Approx(c.groundClearanceM).epsilon(0.02));
    CHECK(back.primitives.size() == 2);

    const std::vector<WheelSpec> specs = toWheelSpecs(a);
    REQUIRE(specs.size() == 2);
    CHECK(specs[0].kind == WheelKind::Swerve);
    CHECK(specs[0].name != specs[1].name);
    CHECK(specs[0].positionM.x == doctest::Approx(0.25f));
    CHECK(specs[0].positionM.z == doctest::Approx(0.0508f).epsilon(0.02));  // hub height above the floor
    CHECK(specs[1].positionM.z == doctest::Approx(0.05f).epsilon(0.02));
    CHECK(specs[1].radiusM == doctest::Approx(0.04f));
    CHECK(specs[1].widthM == doctest::Approx(0.038f));
}

TEST_CASE("appearance: well formed means in-cap lists and elements whose index equals their position") {
    const std::vector<WheelSpec> oneWheel = {wheel(WheelKind::Omni, 0.f, 0.f)};
    const std::vector<Vec3> oneHub(1);
    RobotAppearance a = makeRobotAppearance(chassis(), oneWheel, oneHub, {});
    CHECK(appearanceWellFormed(RobotAppearance{}));
    a.primitives[1].pose.zoneIndex = 0;
    CHECK_FALSE(appearanceWellFormed(a));
    a.primitives[1].pose.zoneIndex = 1;
    CHECK(appearanceWellFormed(a));

    a.zones = {makeAppearanceZone(0, zone("k", Pose3{}, Vec3{}))};
    CHECK(appearanceWellFormed(a));
    a.zones[0].pose.zoneIndex = 3;
    CHECK_FALSE(appearanceWellFormed(a));
    a.zones[0].pose.zoneIndex = 0;
    a.zones[0].kind.assign(kMaxNameBytes + 1, 'x');
    CHECK_FALSE(appearanceWellFormed(a));
    a.zones[0].kind.clear();
    a.wheels.assign(kMaxWheelsPerRobot + 1, AppearanceWheel{});
    CHECK_FALSE(appearanceWellFormed(a));
}
