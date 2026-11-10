#pragma once

// What a game client needs to DRAW another participant's robot and to place that robot's zones: the host's sanitized view of
// the robot it accepted (RosterEntry::appearance, see join_messages.h). Built from the ACCEPTED, clamped spec, never from the
// client's JSON, and bounded by the caps below, so a roster stays small whatever a client sent.
//
// Everything is in the CHASSIS frame of the robot (origin = centre of mass, +X forward, +Y left, +Z up; the floor is
// comHeightMm below the origin). Positions are 1 mm i16 and sizes 1 mm u16 (quantize.h); poses are the ZonePoseInput wire
// form of zone_pose_wire.h, whose `zoneIndex` here is the element's index in its list and must equal its position (a decoder
// refuses anything else, so a zone index in a snapshot's zone poses means the same zone here).
//
// Lists keep the order of the accepted spec: wheel i is the wheel that RobotWireState::wheels[i] reports and
// MatchInput::wheels[i] commands; zone i is the zone that zoneFlags bit i and ZonePoseInput::zoneIndex i address.
// Zones past kMaxZonesPerRobot, wheels past kMaxWheelsPerRobot and primitives past kMaxAppearancePrimitives are not sent
// (the host's own robot is the only one that can have more: a remote spec is refused beyond its wheel cap and has zones
// capped by the runtime).
//
// A client rebuilds a zone's REST pose with appearanceZoneRestPose(), which is sdk::remoteZoneSpecPose over the zone, the
// one rule the host uses (game/zone_pose.h); toChassisSpec / toWheelSpec / toPrimitiveSpec give the drawing code the specs
// it already knows. An empty `wheels` list means "no appearance known" (an older host).

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/zone.h"
#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/game/zone_pose.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/quantize.h"
#include "wpiemu/sdk/match/zone_pose_wire.h"
#include "wpiemu/sdk/physics/chassis_spec.h"

namespace wpiemu::sdk::match {

struct AppearancePrimitive {
    PrimitiveKind kind = PrimitiveKind::Box;
    ZonePoseInput pose;  // chassis frame; zoneIndex = index in RobotAppearance::primitives
    // PrimitiveSpec::extentsM(): the box size, or (diameter, diameter, height) of a cylinder (axis = local Z).
    std::uint16_t extentXMm = 0;
    std::uint16_t extentYMm = 0;
    std::uint16_t extentZMm = 0;
    std::uint8_t r = 0x80;
    std::uint8_t g = 0x80;
    std::uint8_t b = 0x80;

    bool operator==(const AppearancePrimitive&) const = default;
};

struct AppearanceWheel {
    WheelKind kind = WheelKind::Standard;
    bool steered = false;  // has an azimuth (isSteered): its steerAngleQ in a snapshot is meaningful
    // The hub centre in the chassis frame (z is the hub height above the floor minus comHeightMm).
    std::int16_t xMm = 0;
    std::int16_t yMm = 0;
    std::int16_t zMm = 0;
    std::uint16_t radiusMm = 0;
    std::uint16_t widthMm = 0;

    bool operator==(const AppearanceWheel&) const = default;
};

struct AppearanceZone {
    ZonePoseInput pose;     // the zone's REST pose in the chassis frame; zoneIndex = index in RobotAppearance::zones
    std::string kind;       // plugin-owned kind key ("game_2026.pickup"), <= kMaxNameBytes
    std::uint16_t sizeXMm = 0;  // ZoneConfig::sizeM, full extents (meaningful for volume kinds only)
    std::uint16_t sizeYMm = 0;
    std::uint16_t sizeZMm = 0;
    bool onMechanism = false;   // hangs on a mechanism link; a remote robot pins it at `pose` (its local pose)

    bool operator==(const AppearanceZone&) const = default;
};

struct RobotAppearance {
    // The frame box (ChassisSpec::sizeM) and its centre in the chassis frame.
    std::uint16_t sizeXMm = 0;
    std::uint16_t sizeYMm = 0;
    std::uint16_t sizeZMm = 0;
    std::int16_t centreXMm = 0;
    std::int16_t centreYMm = 0;
    std::int16_t centreZMm = 0;
    std::uint16_t comHeightMm = 0;  // the floor is this far below the chassis origin
    std::vector<AppearancePrimitive> primitives;  // <= kMaxAppearancePrimitives, visual only
    std::vector<AppearanceWheel> wheels;          // <= kMaxWheelsPerRobot
    std::vector<AppearanceZone> zones;            // <= kMaxZonesPerRobot

    bool empty() const { return wheels.empty(); }

    bool operator==(const RobotAppearance&) const = default;
};

// Encoded sizes (the codec writes the structs field by field in the order above).
inline constexpr std::size_t kWireAppearanceChassisBytes = 3 * 2 + 3 * 2 + 2;
inline constexpr std::size_t kWireAppearancePrimitiveBytes = 1 + kWireZonePoseBytes + 3 * 2 + 3;
inline constexpr std::size_t kWireAppearanceWheelBytes = 1 + 1 + 3 * 2 + 2 * 2;
// Zone: pose, kind (1-byte length and the bytes), size, flag. The minimum is an empty kind.
inline constexpr std::size_t kWireAppearanceZoneMinBytes = kWireZonePoseBytes + 1 + 3 * 2 + 1;

// True when the lists are within their caps, every list element's index equals its position and every zone kind is within
// kMaxNameBytes (the codec refuses anything else).
inline bool appearanceWellFormed(const RobotAppearance& a) {
    if (a.primitives.size() > kMaxAppearancePrimitives || a.wheels.size() > kMaxWheelsPerRobot ||
        a.zones.size() > kMaxZonesPerRobot)
        return false;
    for (std::size_t i = 0; i < a.primitives.size(); ++i)
        if (a.primitives[i].pose.zoneIndex != i) return false;
    for (std::size_t i = 0; i < a.zones.size(); ++i)
        if (a.zones[i].pose.zoneIndex != i || a.zones[i].kind.size() > kMaxNameBytes) return false;
    return true;
}

// ---- building (host side): total over any finite or non-finite input; values saturate at the wire ranges ----------------

namespace appearance_detail {

inline std::uint8_t hexNibble(char c, bool& ok) {
    if (c >= '0' && c <= '9') return static_cast<std::uint8_t>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<std::uint8_t>(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return static_cast<std::uint8_t>(c - 'A' + 10);
    ok = false;
    return 0;
}

}  // namespace appearance_detail

// "#rrggbb" or "#rrggbbaa" (the alpha is dropped); anything else is the default primitive grey.
inline void parseAppearanceColor(const std::string& css, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b) {
    r = g = b = 0x80;
    if ((css.size() != 7 && css.size() != 9) || css[0] != '#') return;
    bool ok = true;
    std::uint8_t v[3]{};
    for (std::size_t i = 0; i < 3; ++i)
        v[i] = static_cast<std::uint8_t>(appearance_detail::hexNibble(css[1 + 2 * i], ok) * 16 +
                                         appearance_detail::hexNibble(css[2 + 2 * i], ok));
    if (!ok) return;
    r = v[0];
    g = v[1];
    b = v[2];
}

inline AppearancePrimitive makeAppearancePrimitive(std::size_t index, const PrimitiveSpec& p) {
    AppearancePrimitive out;
    out.kind = p.kind == PrimitiveKind::Cylinder ? PrimitiveKind::Cylinder : PrimitiveKind::Box;
    out.pose = makeZonePoseInput(index, p.local);
    const Vec3 e = p.extentsM();
    out.extentXMm = quantizeMilliU16(e.x);
    out.extentYMm = quantizeMilliU16(e.y);
    out.extentZMm = quantizeMilliU16(e.z);
    parseAppearanceColor(p.color, out.r, out.g, out.b);
    return out;
}

// `hub` is the hub centre in the chassis frame (the physics' wheelHubLocal).
inline AppearanceWheel makeAppearanceWheel(const WheelSpec& wheel, const Vec3& hub) {
    AppearanceWheel out;
    out.kind = wheel.kind;
    out.steered = isSteered(wheel);
    out.xMm = quantizeMilliI16(hub.x);
    out.yMm = quantizeMilliI16(hub.y);
    out.zMm = quantizeMilliI16(hub.z);
    out.radiusMm = quantizeMilliU16(wheel.radiusM);
    out.widthMm = quantizeMilliU16(wheel.widthM);
    return out;
}

// A zone whose kind is longer than kMaxNameBytes is cut there.
inline AppearanceZone makeAppearanceZone(std::size_t index, const ZoneConfig& zone) {
    AppearanceZone out;
    out.pose = makeZonePoseInput(index, remoteZoneSpecPose(zone));
    out.kind = zone.kind.substr(0, kMaxNameBytes);
    out.sizeXMm = quantizeMilliU16(zone.sizeM.x);
    out.sizeYMm = quantizeMilliU16(zone.sizeM.y);
    out.sizeZMm = quantizeMilliU16(zone.sizeM.z);
    out.onMechanism = !zone.attachment.onChassis();
    return out;
}

// The appearance of a robot. `wheelHubs[i]` is the chassis-frame hub centre of `wheels[i]` (core/physics wheelHubLocal;
// the SDK has no copy of that rule): wheels beyond min(wheels.size(), wheelHubs.size()) are not described. The lists are cut
// to their caps in spec order.
inline RobotAppearance makeRobotAppearance(const ChassisSpec& chassis, std::span<const WheelSpec> wheels,
                                           std::span<const Vec3> wheelHubs, std::span<const ZoneConfig> zones) {
    RobotAppearance out;
    out.sizeXMm = quantizeMilliU16(chassis.sizeM.x);
    out.sizeYMm = quantizeMilliU16(chassis.sizeM.y);
    out.sizeZMm = quantizeMilliU16(chassis.sizeM.z);
    // The frame box centre in the COM-origin chassis frame (core/physics hullCenterLocal).
    out.centreXMm = quantizeMilliI16(-chassis.comOffsetM.x);
    out.centreYMm = quantizeMilliI16(-chassis.comOffsetM.y);
    out.centreZMm = quantizeMilliI16(chassis.groundClearanceM + 0.5f * chassis.sizeM.z - chassis.comHeightM);
    out.comHeightMm = quantizeMilliU16(chassis.comHeightM);
    const std::size_t prims = std::min(chassis.primitives.size(), kMaxAppearancePrimitives);
    for (std::size_t i = 0; i < prims; ++i) out.primitives.push_back(makeAppearancePrimitive(i, chassis.primitives[i]));
    const std::size_t wheelCount = std::min({wheels.size(), wheelHubs.size(), kMaxWheelsPerRobot});
    for (std::size_t i = 0; i < wheelCount; ++i) out.wheels.push_back(makeAppearanceWheel(wheels[i], wheelHubs[i]));
    const std::size_t zoneCount = std::min(zones.size(), kMaxZonesPerRobot);
    for (std::size_t i = 0; i < zoneCount; ++i) out.zones.push_back(makeAppearanceZone(i, zones[i]));
    return out;
}

// ---- using (client side): finite values whatever the bytes were ---------------------------------------------------------

inline float appearanceMetres(std::int32_t mm) { return static_cast<float>(dequantizeMilli(mm)); }

// The pose a remote robot's zone has when its client does not move it: sdk::remoteZoneSpecPose of the zone.
inline ZoneConfig toZoneConfig(const AppearanceZone& zone) {
    ZoneConfig out;
    out.id = "zone" + std::to_string(zone.pose.zoneIndex);
    out.kind = zone.kind;
    if (zone.onMechanism) out.attachment.parentMechanismId = "remote";
    out.local = toPose3(zone.pose);
    out.sizeM = {appearanceMetres(zone.sizeXMm), appearanceMetres(zone.sizeYMm), appearanceMetres(zone.sizeZMm)};
    return out;
}

inline Pose3 appearanceZoneRestPose(const AppearanceZone& zone) { return remoteZoneSpecPose(toZoneConfig(zone)); }

// Rest pose of every zone, in zone order: what a snapshot's ZonePoseInput entries override (the host sends only the zones
// whose effective pose differs from this one, compared after quantization).
inline std::vector<Pose3> appearanceZoneRestPoses(const RobotAppearance& appearance) {
    std::vector<Pose3> out;
    out.reserve(appearance.zones.size());
    for (const AppearanceZone& z : appearance.zones) out.push_back(appearanceZoneRestPose(z));
    return out;
}

inline PrimitiveSpec toPrimitiveSpec(const AppearancePrimitive& p) {
    PrimitiveSpec out;
    out.name = "prim" + std::to_string(p.pose.zoneIndex);
    out.kind = p.kind;
    out.local = toPose3(p.pose);
    const float x = appearanceMetres(p.extentXMm);
    const float y = appearanceMetres(p.extentYMm);
    const float z = appearanceMetres(p.extentZMm);
    out.sizeM = {x, y, z};
    out.radiusM = 0.5f * x;
    out.heightM = z;
    static constexpr char kHex[] = "0123456789abcdef";
    out.color = {'#', kHex[p.r >> 4], kHex[p.r & 15], kHex[p.g >> 4], kHex[p.g & 15], kHex[p.b >> 4], kHex[p.b & 15]};
    return out;
}

// A WheelSpec carrying what a rig draws: kind, steered, radius, width and a hub position whose x/y are the chassis-frame hub
// and whose z is the hub height above the floor (the persisted meaning). Nothing else is known or needed.
inline WheelSpec toWheelSpec(const AppearanceWheel& w, std::size_t index, float comHeightM) {
    WheelSpec out;
    out.name = "wheel" + std::to_string(index);
    out.kind = w.kind;
    out.steered = w.steered;
    out.radiusM = std::max(appearanceMetres(w.radiusMm), 0.001f);
    out.widthM = appearanceMetres(w.widthMm);
    out.positionM = {appearanceMetres(w.xMm), appearanceMetres(w.yMm), appearanceMetres(w.zMm) + comHeightM};
    return out;
}

inline std::vector<WheelSpec> toWheelSpecs(const RobotAppearance& a) {
    std::vector<WheelSpec> out;
    out.reserve(a.wheels.size());
    const float comHeightM = appearanceMetres(a.comHeightMm);
    for (std::size_t i = 0; i < a.wheels.size(); ++i) out.push_back(toWheelSpec(a.wheels[i], i, comHeightM));
    return out;
}

// The chassis a rig draws: the frame box, the COM offset and ground clearance implied by its centre, and the primitives.
inline ChassisSpec toChassisSpec(const RobotAppearance& a) {
    ChassisSpec out;
    out.sizeM = {appearanceMetres(a.sizeXMm), appearanceMetres(a.sizeYMm), appearanceMetres(a.sizeZMm)};
    out.comHeightM = appearanceMetres(a.comHeightMm);
    out.comOffsetM = {-appearanceMetres(a.centreXMm), -appearanceMetres(a.centreYMm), 0.f};
    out.groundClearanceM = std::max(0.f, appearanceMetres(a.centreZMm) + out.comHeightM - 0.5f * out.sizeM.z);
    out.primitives.reserve(a.primitives.size());
    for (const AppearancePrimitive& p : a.primitives) out.primitives.push_back(toPrimitiveSpec(p));
    return out;
}

}  // namespace wpiemu::sdk::match
