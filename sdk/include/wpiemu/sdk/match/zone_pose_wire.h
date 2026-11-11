#pragma once

// The one wire form of a zone pose, shared by MatchInput::zonePoses (client -> host: where the client wants its zones, see
// input_messages.h) and RobotWireState::zonePoses (host -> clients: where a robot's moved zones are, see
// snapshot_messages.h). Pure functions over the fixed-point schemes of quantize.h; the codec in ipc/ writes the struct.
//
// A zone pose is in the CHASSIS frame of its robot (origin = centre of mass, +X forward, +Y left, +Z up). Zones are
// addressed by their index in the robot's zone list (the accepted Join spec).

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/quantize.h"

namespace wpiemu::sdk::match {

struct ZonePoseInput {
    std::uint8_t zoneIndex = 0;  // index in the robot's zone list, < kMaxZonesPerRobot
    // Position in the chassis frame, 1 mm units.
    std::int16_t xMm = 0;
    std::int16_t yMm = 0;
    std::int16_t zMm = 0;
    // Intrinsic Z-Y-X orientation (Quat::fromEuler), quantizeAngle()/dequantizeAngle() each.
    std::int16_t yawQ = 0;
    std::int16_t pitchQ = 0;
    std::int16_t rollQ = 0;

    bool operator==(const ZonePoseInput&) const = default;
};

// Encoded size of one ZonePoseInput: u8 index and six i16 (the codec writes the struct field by field in this order).
inline constexpr std::size_t kWireZonePoseBytes = 1 + 6 * 2;

// Zone pose fixed point: positions saturate at +-kMaxZonePoseM and are exact to kMaxZonePosePositionErrorM; the rotation
// is exact to kMaxZonePoseRotationErrorRad (the geodesic angle between a pose and its round trip), one quantization step
// per Euler angle and so three in the worst case. Float rounding adds less than 1e-5 of either. Near pitch = +-90 degrees
// (a shooter pointing straight up or down) roll and yaw are individually ambiguous; zoneEulerOf resolves that so the
// ROTATION still meets the bound.
inline constexpr double kMaxZonePoseM = 32.767;
inline constexpr double kMaxZonePosePositionErrorM = 0.0005;
inline constexpr double kMaxZonePoseRotationErrorRad = 3.0 * kMaxAngleQuantizationErrorRad;

// Z-Y-X Euler angles of a rotation, computed in double. Unlike Quat::toEuler it keeps the ROTATION exact up to double
// rounding at and near pitch = +-90 degrees, where roll and yaw are individually undefined: there yaw is 0 and roll takes
// the whole rotation about the shared axis, and the pitch comes from atan2 (asin is ill-conditioned next to +-1).
struct ZoneEuler {
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
};
inline ZoneEuler zoneEulerOf(const Quat& rotation) {
    double x = rotation.x, y = rotation.y, z = rotation.z, w = rotation.w;
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!(norm > 0.0) || !std::isfinite(norm)) return {};
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
    const double yawSin = 2.0 * (w * z + x * y);
    const double yawCos = 1.0 - 2.0 * (y * y + z * z);
    const double cosPitch = std::hypot(yawSin, yawCos);
    ZoneEuler e;
    e.pitch = std::atan2(2.0 * (w * y - z * x), cosPitch);
    if (cosPitch < 1e-7) {
        e.roll = 2.0 * std::atan2(x, w);
    } else {
        e.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        e.yaw = std::atan2(yawSin, yawCos);
    }
    return e;
}

// The wire form of `pose` for zone `index` (values past 255 saturate and are refused by zonePosesWellFormed). Total: a
// non-finite position coordinate becomes 0 and a non-finite or zero rotation the identity; positions beyond
// +-kMaxZonePoseM saturate.
inline ZonePoseInput makeZonePoseInput(std::size_t index, const Pose3& pose) {
    ZonePoseInput z;
    z.zoneIndex = static_cast<std::uint8_t>(std::min<std::size_t>(index, 0xFF));
    z.xMm = quantizeMilliI16(pose.position.x);
    z.yMm = quantizeMilliI16(pose.position.y);
    z.zMm = quantizeMilliI16(pose.position.z);
    const ZoneEuler e = zoneEulerOf(finite(pose.rotation) ? pose.rotation : Quat{});
    z.yawQ = quantizeAngle(e.yaw);
    z.pitchQ = quantizeAngle(e.pitch);
    z.rollQ = quantizeAngle(e.roll);
    return z;
}

// The chassis-frame pose a ZonePoseInput describes (always finite, unit rotation).
inline Pose3 toPose3(const ZonePoseInput& z) {
    const auto metres = [](std::int16_t mm) { return static_cast<float>(dequantizeMilli(mm)); };
    return {Vec3{metres(z.xMm), metres(z.yMm), metres(z.zMm)},
            Quat::fromEuler(dequantizeAngle(z.rollQ), dequantizeAngle(z.pitchQ), dequantizeAngle(z.yawQ)).normalized()};
}

// A zone pose list is well formed when it has at most `maxEntries` entries whose zoneIndex values are strictly increasing
// and below kMaxZonesPerRobot (the codec refuses anything else).
inline bool zonePosesWellFormed(const std::vector<ZonePoseInput>& poses, std::size_t maxEntries = kMaxZonesPerRobot) {
    if (poses.size() > maxEntries) return false;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        if (poses[i].zoneIndex >= kMaxZonesPerRobot) return false;
        if (i > 0 && poses[i].zoneIndex <= poses[i - 1].zoneIndex) return false;
    }
    return true;
}

// What the host puts in RobotWireState::zonePoses: the zones whose EFFECTIVE pose differs from their rest pose (compared
// after quantization, so a change below the wire resolution is no change), at most kMaxSnapshotZonePosesPerRobot of them.
// When more qualify, active zones (bit i of `activeFlags`) win over inactive ones and lower indices over higher ones.
// Ordered by zoneIndex. Zones beyond min(restPoses.size(), effectivePoses.size(), kMaxZonesPerRobot) are not considered.
inline std::vector<ZonePoseInput> selectSnapshotZonePoses(std::span<const Pose3> restPoses,
                                                          std::span<const Pose3> effectivePoses,
                                                          std::uint32_t activeFlags) {
    const std::size_t n = std::min({restPoses.size(), effectivePoses.size(), kMaxZonesPerRobot});
    std::vector<ZonePoseInput> moved;
    std::uint32_t movedActive = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const ZonePoseInput effective = makeZonePoseInput(i, effectivePoses[i]);
        if (effective == makeZonePoseInput(i, restPoses[i])) continue;
        moved.push_back(effective);
        if (((activeFlags >> i) & 1u) != 0) movedActive |= std::uint32_t{1} << i;
    }
    if (moved.size() <= kMaxSnapshotZonePosesPerRobot) return moved;
    std::stable_partition(moved.begin(), moved.end(),
                          [movedActive](const ZonePoseInput& z) { return ((movedActive >> z.zoneIndex) & 1u) != 0; });
    moved.resize(kMaxSnapshotZonePosesPerRobot);
    std::sort(moved.begin(), moved.end(),
              [](const ZonePoseInput& a, const ZonePoseInput& b) { return a.zoneIndex < b.zoneIndex; });
    return moved;
}

}  // namespace wpiemu::sdk::match
