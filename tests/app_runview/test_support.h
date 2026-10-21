#pragma once

// Shared fixtures for the run view scene and camera tests.

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace runview_test {

namespace sdk = wpiemu::sdk;

inline constexpr float kEps = 1e-3f;

inline bool near(const sdk::Vec3& a, const sdk::Vec3& b, float eps = kEps) { return sdk::length(a - b) < eps; }

// A connected robot standing at (3, 2, 0.2), facing +X.
inline sdk::SimSnapshotData baseSnapshot() {
    sdk::SimSnapshotData d;
    d.chassis.pose = {{3.f, 2.f, 0.2f}, sdk::Quat::fromYaw(0.f)};
    d.robotConnected = true;
    return d;
}

}  // namespace runview_test
