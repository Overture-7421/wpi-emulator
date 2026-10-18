#pragma once

// Vision cameras: virtual PhotonVision / Limelight coprocessors persisted in RobotConfig::vision
// (ADR 0017).
//
// A camera is NAME-keyed (CameraConfig::id), not a DeviceKey / slot. It hangs on the chassis or on
// a mechanism link with the same MechanismAttachment and link rules as a mechanism mount or a zone:
//   pose_in_chassis(camera) = pose_in_chassis(attachment parent link) * camera.local
// so a camera on a turret moves with the turret.
//
// Frames and image conventions (the same everywhere in the vision code):
//  * World = SDK Z-up NWU = WPILib blue origin (+X blue to red, +Y left).
//  * CAMERA frame (vendor neutral): +X forward (optical axis), +Y left, +Z up (the WPILib /
//  PhotonLib convention).
//    CameraConfig::local is the pose of this frame in its parent link frame. Limelight's own
//    conventions (camera space X right / Y down / Z out, robot space Y right) exist only inside the
//    Limelight publisher.
//  * IMAGE: continuous pixel coordinates, origin at the top-left corner, x right, y down, spanning
//  [0,w] x [0,h].
//    A camera-frame point (x, y, z) with x > 0 projects to  u = cx - fx * (y / x),  v = cy - fy *
//    (z / x).
//
// Stored in SI units (rad, m, us); pixels are doubles. There is no migration (ADR 0013).

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/mechanism_attachment.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

enum class CameraVendor : std::uint8_t {
    PhotonVision = 0,
    Limelight = 1,
};

struct CameraIntrinsics {
    int widthPx = 1280;
    int heightPx = 800;
    double hfovRad = 1.5708;  // horizontal FOV; fx is derived from it unless fxPx > 0
    double fxPx = 0.0;        // > 0 overrides the derived value
    double fyPx = 0.0;        // 0 => fx
    double cxPx = 0.0;        // 0 => widthPx / 2
    double cyPx = 0.0;        // 0 => heightPx / 2
    // k1 k2 p1 p2 k3: published to PhotonVision NT, NOT applied in v1
    std::array<double, 5> distortion{};

    // Effective pinhole parameters. Only meaningful for validated intrinsics (hfovRad in (0, pi)
    // unless fxPx is set).
    double fx() const { return fxPx > 0.0 ? fxPx : (0.5 * widthPx) / std::tan(0.5 * hfovRad); }
    double fy() const { return fyPx > 0.0 ? fyPx : fx(); }
    double cx() const { return cxPx > 0.0 ? cxPx : 0.5 * widthPx; }
    double cy() const { return cyPx > 0.0 ? cyPx : 0.5 * heightPx; }
    double vfovRad() const { return 2.0 * std::atan((0.5 * heightPx) / fy()); }

    bool operator==(const CameraIntrinsics&) const = default;
};

// All zero = IDEAL (the default): exact geometry, no randomness drawn. Frame rate and latency are
// device behaviour and always apply, they are not part of realism.
struct CameraRealism {
    double cornerNoisePx = 0.0;  // sigma of the Gaussian noise on every detected corner coordinate
    std::int64_t latencyJitterUs = 0;  // uniform +-jitter on the capture-to-publish latency
    double dropoutProbability = 0.0;   // chance that a frame detects nothing

    bool ideal() const {
        return cornerNoisePx == 0.0 && latencyJitterUs == 0 && dropoutProbability == 0.0;
    }
    bool operator==(const CameraRealism&) const = default;
};

struct CameraConfig {
    std::string id;  // unique among cameras; distinct from mechanism/zone ids and "chassis"/"root"
    std::string label;
    CameraVendor vendor = CameraVendor::PhotonVision;
    // PV: nickname -> /photonvision/<ntName>; LL: table name ("limelight", "limelight-left")
    std::string ntName;
    bool enabled = true;
    MechanismAttachment attachment;  // chassis (default) or a mechanism link; same rules as zones
    Pose3 local;                     // camera frame in the attachment link frame
    CameraIntrinsics intrinsics;
    double frameRateHz = 30.0;       // (0, 240]; at most one capture per tick
    std::int64_t latencyUs = 25000;  // total capture-to-publish latency (>= 0)
    // Limelight "cl" share of latencyUs (clamped to latencyUs); "tl" = the rest
    std::int64_t captureLatencyUs = 10000;
    double tagSizeM = 0.1651;        // 6.5 in; 0 => the layout's tag size
    double minTagPixels = 12.0;      // shortest projected tag edge that is still detected
    double maxViewAngleRad = 1.309;  // 75 deg between the optical ray and the tag normal
    bool occlusion = true;           // line of sight against layer::Field
    bool multiTag = true;            // PV multitag result / Limelight MT1 from all tags
    CameraRealism realism;
    // Limelight: what the device believes (web UI), CHASSIS frame; unset => the physical pose at
    // session start
    std::optional<Pose3> assumedPose;

    // The name the camera has on NetworkTables: ntName when set, else "limelight" for a Limelight,
    // else the id.
    std::string networkName() const {
        if (!ntName.empty()) return ntName;
        return vendor == CameraVendor::Limelight ? std::string("limelight") : id;
    }

    bool operator==(const CameraConfig&) const = default;
};

struct VisionConfig {
    std::string layoutId;    // FiducialLayout::id; "" => the plugin's first layout
    std::string layoutFile;  // WPILib AprilTagFieldLayout JSON path; non-empty overrides layoutId
    // "" => derive from the project's photonlib version; else force this PhotonVision message UUID
    std::string photonMessageUuid;
    std::vector<CameraConfig> cameras;

    const CameraConfig* findCamera(std::string_view cameraId) const {
        for (const CameraConfig& c : cameras)
            if (c.id == cameraId) return &c;
        return nullptr;
    }
    bool empty() const { return cameras.empty(); }  // no cameras: the vision subsystem is inert

    bool operator==(const VisionConfig&) const = default;
};

}  // namespace wpiemu::sdk
