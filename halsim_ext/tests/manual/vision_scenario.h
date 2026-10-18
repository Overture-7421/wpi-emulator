#pragma once

// Synthetic camera data for the `--vision` scenario of wpiemu_halsim_fake_emulator (and its unit
// test): what the emulator's vision engine will produce, computed with plain pinhole math, so the
// real vision bridge can be exercised end to end without the engine. SDK types only. Conventions
// as in sdk/vision/vision_types.h (camera frame X forward / Y left / Z up, image origin top-left,
// corners BL BR TR TL).
//
// Scene: the robot stands still at (2.0, 4.0) facing +X. The cameras are the ones of the sample
// robots (samples/vision_robot/README.md, "Constants"): PhotonVision "photon-front" at (0.30, 0,
// 0.50) m pitched 20 deg up and Limelight "limelight-rear" at (-0.30, 0, 0.45) m, turned 180 deg
// and pitched 25 deg up. The front camera sees the hub tags 25 and 26, the rear one the alliance
// wall tags 31 and 32 of the 2026 welded layout (plugins/game_2026/src/fiducial_layouts.cpp,
// 2026-rebuilt-welded.json).
//
// Timing is the contract of the vision spec: a camera captures when its accumulator reaches one
// frame period (at most once per tick), after the step of tick N0 at T(N0); the frame rides
// SensorFrame N0 + 1 + k with k = ceil(latency / step) and VisionFrame::latencyUs = k * step.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::halsim::manual {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTagSizeM = 0.1651;  // 6.5 in

struct ScenarioTag {
    int id = 0;
    sdk::Pose3 fieldToTag;
};

struct ScenarioCamera {
    sdk::CameraSetup setup;
    sdk::Pose3 robotToCamera;
    std::int64_t captureLatencyUs = 0;  // the Limelight "cl" share of setup.latencyUs
    std::vector<ScenarioTag> tags;      // the tags to try to see, in any order
};

inline sdk::Pose3 mountPose(double xM, double yM, double zM, double rollDeg, double pitchDeg,
                            double yawDeg) {
    const auto rad = [](double deg) { return static_cast<float>(deg * kPi / 180.0); };
    return sdk::Pose3({static_cast<float>(xM), static_cast<float>(yM), static_cast<float>(zM)},
                      sdk::Quat::fromEuler(rad(rollDeg), rad(pitchDeg), rad(yawDeg)));
}

inline sdk::Pose3 robotPose() {
    return sdk::Pose3({2.0f, 4.0f, 0.0f}, sdk::Quat::fromYaw(0.0f));
}

inline void setIntrinsics(sdk::CameraSetup& setup, int widthPx, int heightPx, double hfovDeg) {
    setup.widthPx = widthPx;
    setup.heightPx = heightPx;
    setup.fxPx = setup.fyPx = (widthPx / 2.0) / std::tan(hfovDeg * kPi / 360.0);
    setup.cxPx = widthPx / 2.0;
    setup.cyPx = heightPx / 2.0;
}

inline std::vector<ScenarioCamera> makeScenarioCameras() {
    // Tag normal -X: faces the blue alliance wall. Tag normal +X: faces the field.
    const sdk::Quat facingBack(0.0f, 0.0f, 1.0f, 0.0f);
    const sdk::Quat facingField(0.0f, 0.0f, 0.0f, 1.0f);

    ScenarioCamera front;
    front.setup.cameraId = "photon-front";
    front.setup.vendor = sdk::CameraVendor::PhotonVision;
    front.setup.ntName = "photon-front";
    setIntrinsics(front.setup, 1280, 800, 75.0);
    front.setup.frameRateHz = 30.0;
    front.setup.latencyUs = 25000;
    front.robotToCamera = mountPose(0.30, 0.0, 0.50, 0.0, -20.0, 0.0);
    front.setup.defaultAssumedRobotToCamera = front.robotToCamera;
    front.captureLatencyUs = 10000;
    front.tags = {{25, sdk::Pose3({4.0218614f, 4.3902376f, 1.12395f}, facingBack)},
                  {26, sdk::Pose3({4.0218614f, 4.0346376f, 1.12395f}, facingBack)}};

    ScenarioCamera rear;
    rear.setup.cameraId = "limelight-rear";
    rear.setup.vendor = sdk::CameraVendor::Limelight;
    rear.setup.ntName = "limelight-rear";
    setIntrinsics(rear.setup, 1280, 800, 90.0);
    rear.setup.frameRateHz = 90.0;
    rear.setup.latencyUs = 25000;
    rear.robotToCamera = mountPose(-0.30, 0.0, 0.45, 0.0, -25.0, 180.0);
    rear.setup.defaultAssumedRobotToCamera = rear.robotToCamera;
    rear.captureLatencyUs = 10000;
    rear.tags = {{31, sdk::Pose3({0.0080772f, 3.7457126f, 0.55245f}, facingField)},
                 {32, sdk::Pose3({0.0080772f, 4.1775126f, 0.55245f}, facingField)}};
    return {front, rear};
}

// One processed frame of `camera` for a robot at `fieldToRobot`: what is visible, where its corners
// fall in the image and the camera-to-tag transforms. No noise and no alternate solution (ambiguity
// 0, alt = best).
inline sdk::VisionFrame captureFrame(const ScenarioCamera& camera, const sdk::Pose3& fieldToRobot,
                                     std::uint64_t sequence, std::int64_t captureTimeUs,
                                     std::int64_t latencyUs) {
    const sdk::CameraSetup& cam = camera.setup;
    sdk::VisionFrame frame;
    frame.cameraId = cam.cameraId;
    frame.vendor = cam.vendor;
    frame.ntName = cam.ntName;
    frame.sequence = sequence;
    frame.captureTimeUs = captureTimeUs;
    frame.latencyUs = latencyUs;
    frame.captureLatencyUs = std::min(camera.captureLatencyUs, latencyUs);
    frame.robotToCamera = camera.robotToCamera;

    const sdk::Pose3 fieldToCamera = fieldToRobot * camera.robotToCamera;
    const sdk::Pose3 cameraToField = fieldToCamera.inverse();
    const double half = kTagSizeM / 2.0;
    const sdk::Vec3 tagCorners[4] = {
        {0.0f, static_cast<float>(-half), static_cast<float>(-half)},  // BL
        {0.0f, static_cast<float>(half), static_cast<float>(-half)},   // BR
        {0.0f, static_cast<float>(half), static_cast<float>(half)},    // TR
        {0.0f, static_cast<float>(-half), static_cast<float>(half)}};  // TL

    // u = cx - fx * (y / x), v = cy - fy * (z / x); false when the point is behind the camera.
    const auto project = [&](const sdk::Vec3& p, sdk::PixelPoint& out) {
        if (p.x <= 0.0f) return false;
        out.x = cam.cxPx - cam.fxPx * (p.y / p.x);
        out.y = cam.cyPx - cam.fyPx * (p.z / p.x);
        return true;
    };

    for (const ScenarioTag& tag : camera.tags) {
        const sdk::Pose3 cameraToTag = cameraToField * tag.fieldToTag;
        const sdk::Vec3 tagNormal = tag.fieldToTag.transformVector({1.0f, 0.0f, 0.0f});
        if (sdk::dot(tagNormal, fieldToCamera.position - tag.fieldToTag.position) <= 0.0f) continue;

        sdk::FiducialObservation o;
        o.id = tag.id;
        bool visible = project(cameraToTag.position, o.centerPx);
        for (int i = 0; i < 4 && visible; ++i) {
            visible = project(cameraToTag.transformPoint(tagCorners[i]), o.cornersPx[i]) &&
                      o.cornersPx[i].x >= 0.0 && o.cornersPx[i].x <= cam.widthPx &&
                      o.cornersPx[i].y >= 0.0 && o.cornersPx[i].y <= cam.heightPx;
        }
        if (!visible) continue;

        double area2 = 0.0;  // shoelace: twice the polygon area
        for (int i = 0; i < 4; ++i) {
            const sdk::PixelPoint& a = o.cornersPx[i];
            const sdk::PixelPoint& b = o.cornersPx[(i + 1) % 4];
            area2 += a.x * b.y - b.x * a.y;
        }
        const sdk::PixelPoint& bl = o.cornersPx[0];
        const sdk::PixelPoint& br = o.cornersPx[1];
        o.yawRad = std::atan((o.centerPx.x - cam.cxPx) / cam.fxPx);
        o.pitchRad = -std::atan((o.centerPx.y - cam.cyPx) / cam.fyPx);
        // Bottom edge, counter-clockwise on screen.
        o.skewRad = std::atan2(-(br.y - bl.y), br.x - bl.x);
        o.areaPct =
            100.0 * std::abs(area2) / 2.0 / (static_cast<double>(cam.widthPx) * cam.heightPx);
        o.distanceM = sdk::length(cameraToTag.position);
        o.fieldToTag = tag.fieldToTag;
        o.cameraToTagBest = cameraToTag;
        o.cameraToTagAlt = cameraToTag;
        frame.observations.push_back(o);
    }
    std::sort(frame.observations.begin(), frame.observations.end(),
              [](const sdk::FiducialObservation& a, const sdk::FiducialObservation& b) {
                  return a.areaPct > b.areaPct;
              });

    if (frame.observations.size() >= 2) {
        frame.multiTag.valid = true;
        frame.multiTag.fieldToCamera = fieldToCamera;
        for (const sdk::FiducialObservation& o : frame.observations)
            frame.multiTag.tagIds.push_back(o.id);
    }
    return frame;
}

// The frame schedule: call framesFor() for ticks 1, 2, 3, ... in order. The sim time after tick N
// is N * stepUs.
class VisionScenario {
public:
    explicit VisionScenario(std::uint32_t stepUs) : stepUs_(stepUs) {
        for (ScenarioCamera& camera : makeScenarioCameras())
            cameras_.push_back({std::move(camera)});
    }

    sdk::VisionSetup setup() const {
        sdk::VisionSetup out;
        for (const Runtime& c : cameras_) out.cameras.push_back(c.camera.setup);
        return out;
    }

    // The frames that ride SensorFrame `tick` (captured k + 1 ticks earlier), then this tick's
    // captures are queued behind them.
    std::vector<sdk::VisionFrame> framesFor(std::uint64_t tick) {
        std::vector<sdk::VisionFrame> due;
        for (Runtime& c : cameras_) {
            while (!c.pending.empty() && c.pending.front().publishTick <= tick) {
                due.push_back(std::move(c.pending.front().frame));
                c.pending.pop_front();
            }
        }
        const std::int64_t step = stepUs_;
        for (Runtime& c : cameras_) {
            const double periodUs = 1e6 / c.camera.setup.frameRateHz;
            c.accumulatorUs += static_cast<double>(stepUs_);
            if (c.accumulatorUs < periodUs) continue;
            c.accumulatorUs -= periodUs;
            const std::int64_t ticksOfLatency = (c.camera.setup.latencyUs + step - 1) / step;
            c.pending.push_back(
                {tick + 1 + static_cast<std::uint64_t>(ticksOfLatency),
                 captureFrame(c.camera, robotPose(), ++c.sequence,
                              static_cast<std::int64_t>(tick) * step, ticksOfLatency * step)});
        }
        return due;
    }

private:
    struct Pending {
        std::uint64_t publishTick = 0;
        sdk::VisionFrame frame;
    };
    struct Runtime {
        ScenarioCamera camera;
        double accumulatorUs = 0.0;
        std::uint64_t sequence = 0;
        std::deque<Pending> pending;
    };

    std::uint32_t stepUs_;
    std::vector<Runtime> cameras_;
};

}  // namespace wpiemu::halsim::manual
