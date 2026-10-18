#pragma once

// Cameras: simulated vision coprocessors (Limelight, PhotonVision) persisted in RobotConfig::cameras (ADR 0017).
//
// A camera is attached to the chassis or to a mechanism link, exactly like a zone or a mechanism mount:
//   pose_in_chassis(camera) = pose_in_chassis(attachment parent link) * camera.local
// so a camera mounted on a turret moves with the turret. Camera frame: +X is the optical axis (forward),
// +Y left, +Z up (the same convention as the robot frame, which is how WPILib/Limelight/PhotonVision express
// robot-to-camera transforms).
//
// The model is deliberately simple (no occlusion, lens distortion or exposure): a tag is seen when it is inside
// the field of view, within `maxRangeM` and facing the camera. Results are delivered `latencyMs` after capture.
//
// Stored in SI units except the explicitly suffixed degrees / milliseconds. There is no migration (ADR 0013).

#include <cstdint>
#include <string>

#include "wpiemu/sdk/config/mechanism_attachment.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

enum class CameraKind : std::uint8_t {
    Limelight = 0,
    PhotonVision = 1,
};

inline constexpr const char* cameraKindKey(CameraKind kind) {
    return kind == CameraKind::PhotonVision ? "photonvision" : "limelight";
}

struct CameraConfig {
    std::string id;       // unique among cameras, distinct from mechanism and zone ids
    std::string label;    // display name
    // The name the robot code uses. Limelight: the table is "limelight" when empty, else "limelight-<name>"
    // (LimelightHelpers' limelightName). PhotonVision: the PhotonCamera name (/photonvision/<name>). Unique per kind.
    std::string name;
    CameraKind kind = CameraKind::Limelight;
    MechanismAttachment attachment;   // chassis (default) or mechanism + link
    Pose3 local;                      // relative to the attachment link frame (camera frame, see above)

    double hFovDeg = 75.0;            // horizontal field of view
    double vFovDeg = 50.0;            // vertical field of view
    double maxRangeM = 6.0;           // tags farther than this are not seen
    double maxIncidenceDeg = 75.0;    // tags viewed more obliquely than this (angle to the tag normal) are not seen
    double fps = 30.0;                // capture rate; the sim captures at the next step boundary >= 1/fps
    double latencyMs = 40.0;          // capture -> publish delay (Limelight tl+cl, PhotonVision publish - capture)
    int widthPx = 1280;               // pixel space for corners / area
    int heightPx = 800;
    double positionNoiseM = 0.0;      // 1-sigma noise on the estimated robot position (x, y)
    double yawNoiseDeg = 0.0;         // 1-sigma noise on the estimated robot yaw

    bool operator==(const CameraConfig&) const = default;
};

}  // namespace wpiemu::sdk
