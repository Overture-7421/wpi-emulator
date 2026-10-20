#pragma once

// Vendor-neutral output of the simulated cameras (emulator -> extension) and what the extension is told about
// them at connect time. The core VisionSystem produces CameraFrames; the vision bridges (bridges/vision)
// translate them into Limelight / PhotonVision NetworkTables data.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/camera.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::sdk {

// One tag a camera saw in one frame.
struct TagDetection {
    int id = 0;
    Pose3 cameraToTag;                      // tag pose in the CAMERA frame (+X forward); tag +X points out of its face
    double yawDeg = 0.0;                    // bearing of the tag centre: + = right of the optical axis (Limelight tx / Photon yaw)
    double pitchDeg = 0.0;                  // + = above the optical axis (Limelight ty / Photon pitch)
    double areaPercent = 0.0;               // of the image (Limelight ta / Photon area)
    double ambiguity = 0.0;                 // pose ambiguity in [0, 1]; the simulation is unambiguous (0)
    std::array<double, 8> cornersPx{};      // x0,y0 .. x3,y3 of the tag corners in the image
    double distanceM = 0.0;                 // camera to tag centre
};

// One captured frame of one camera, delivered at publish time (capture + latency).
struct CameraFrame {
    std::uint32_t cameraIndex = 0;          // index into VisionSetup::cameras
    std::uint64_t sequence = 0;             // frames captured by this camera since the session started
    SimTimeUs captureTimeUs = 0;            // sim time the image was captured
    SimTimeUs publishTimeUs = 0;            // sim time the result is published (>= capture)
    Pose3 robotToCamera;                    // chassis frame -> camera frame at capture time
    std::vector<TagDetection> tags;         // empty = nothing seen (a frame is still published)
    // Robot pose in the FIELD frame the camera's pipeline would report (ground truth + configured noise).
    // Valid when at least one tag was seen.
    bool hasRobotPose = false;
    Pose3 robotPoseField;
    double avgTagDistM = 0.0;
    double avgTagAreaPercent = 0.0;
    double tagSpanM = 0.0;                  // largest distance between two seen tags (0 for one tag)

    // How long the result took to be published (publish - capture); never negative.
    SimTimeUs latencyUs() const { return std::max<SimTimeUs>(0, publishTimeUs - captureTimeUs); }
    double latencyMs() const { return usToMs(latencyUs()); }
};

// What the extension needs to create the NetworkTables topics of one camera before the first frame.
struct CameraSetup {
    CameraConfig config;
};

struct VisionSetup {
    std::vector<CameraSetup> cameras;
    std::string photonVersion;              // robot project's photonlib version ("v2026.3.4"); empty = unknown (default used)
    std::string layoutName;                 // informational: the AprilTag layout the simulation uses
    // Size of the field of that layout (the season's data, from AprilTagLayout). The Limelight bridge needs it to
    // re-express our blue-origin poses in the red-origin and center-origin conventions; 0 = unknown.
    double fieldLengthM = 0.0;
    double fieldWidthM = 0.0;

    // True when both dimensions are finite and positive, i.e. poses can be moved to another field origin.
    bool hasFieldSize() const {
        return std::isfinite(fieldLengthM) && fieldLengthM > 0.0 && std::isfinite(fieldWidthM) && fieldWidthM > 0.0;
    }
};

}  // namespace wpiemu::sdk
