#pragma once

// Shared fixtures of the Limelight tests: the sample robot's rear Limelight scene from the nt spike, a recording
// logger and a rig that wires FakeNtApi -> LimelightPublisher the way the bridge core does (vendor -> createPublisher
// -> start / poll / publish).
//
// Scene (samples/vision_robot/README.md and the spike's findings): robot at (2, 4) facing +X, Limelight
// "limelight-rear" at (-0.30, 0, 0.45) m, WPILib pitch -25 deg (tilted up) and yaw 180 deg (looking backwards),
// 1280x800, 90 deg hfov. Tags 31 and 32 sit on the blue wall (x = 0.0081 m, y = 3.7457 / 4.1775 m, z = 0.5525 m,
// facing +X), where the real 2026 welded layout has them. From the spike: tx -9.16 / +6.42 deg, ty -21.53 deg,
// ta 0.471 %, 1.71 m, and MegaTag2 with a 45 deg yaw written by the robot = (1.237, 5.334) when tag 31 is the primary.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vision_frame_support.h"
#include "wpiemu/bridges/vision/limelight_keys.h"
#include "wpiemu/bridges/vision/limelight_math.h"
#include "wpiemu/bridges/vision/limelight_vendor.h"
#include "wpiemu/bridges/vision/testing/fake_nt_api.h"
#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::tests::limelight {

namespace ll = wpiemu::bridges::vision::limelight;
namespace vision = wpiemu::tests::vision;  // the frame builder, for the test files that use this namespace
using wpiemu::bridges::vision::ICameraPublisher;
using wpiemu::bridges::vision::NtType;
using wpiemu::bridges::vision::testing::FakeNtApi;

inline constexpr const char* kTable = "limelight-rear";
inline constexpr double kDeg = vision::kPi / 180.0;

// The six botpose arrays, MegaTag1 then MegaTag2.
inline constexpr std::array<std::string_view, 6> kBotposeKeys = {
    ll::kBotpose,           ll::kBotposeWpiBlue,    ll::kBotposeWpiRed,
    ll::kBotposeOrb,        ll::kBotposeOrbWpiBlue, ll::kBotposeOrbWpiRed,
};

inline std::string topic(std::string_view key) { return ll::topicOf(kTable, key); }

// The sample robot's rear Limelight mount, WPILib frame.
inline sdk::Pose3 rearMount() {
    return sdk::Pose3({-0.30f, 0.0f, 0.45f},
                      sdk::Quat::fromEuler(0.f, static_cast<float>(-25.0 * kDeg), static_cast<float>(vision::kPi)));
}

inline sdk::Pose3 robotAt(double x, double y, double yawDeg) {
    const sdk::Vec3 position(static_cast<float>(x), static_cast<float>(y), 0.0f);
    return sdk::Pose3(position, sdk::Quat::fromYaw(static_cast<float>(yawDeg * kDeg)));
}

// Tags 31 and 32 of the 2026 welded layout (both facing +X).
inline sdk::FiducialLayout sceneLayout() {
    sdk::FiducialLayout layout;
    layout.id = "limelight.test";
    layout.tagSizeM = 0.1651;
    layout.fieldLengthM = 16.541f;
    layout.fieldWidthM = 8.069f;
    layout.tags.push_back({31, sdk::Pose3({0.0080772f, 3.7457126f, 0.55245f}, sdk::Quat::identity()), 0.0});
    layout.tags.push_back({32, sdk::Pose3({0.0080772f, 4.1775126f, 0.55245f}, sdk::Quat::identity()), 0.0});
    return layout;
}
inline constexpr double kTagX = 0.0080772;  // tags 31 and 32 share it
inline constexpr double kTag31Y = 3.7457126;
inline constexpr double kTag32Y = 4.1775126;

// A frame of the rear Limelight. Tags 31 and 32 tie in area (they have the same depth), so the order of `tagIds`
// decides which one is the primary (first in the frame): the builder alone would leave it to floating point noise.
inline sdk::VisionFrame rearFrame(const std::vector<int>& tagIds,
                                  const sdk::Pose3& fieldToRobot = robotAt(2.0, 4.0, 0.0),
                                  std::int64_t latencyUs = 32000, std::int64_t captureLatencyUs = 12000) {
    vision::FrameSpec spec;
    spec.cameraId = "ll-rear";
    spec.vendor = sdk::CameraVendor::Limelight;
    spec.ntName = kTable;
    spec.fieldToRobot = fieldToRobot;
    spec.robotToCamera = rearMount();
    spec.tagIds = tagIds;
    spec.latencyUs = latencyUs;
    spec.captureLatencyUs = captureLatencyUs;
    sdk::VisionFrame frame = vision::makeFrame(spec, sceneLayout());
    const auto rank = [&tagIds](int id) { return std::find(tagIds.begin(), tagIds.end(), id) - tagIds.begin(); };
    std::stable_sort(frame.observations.begin(), frame.observations.end(),
                     [&rank](const sdk::FiducialObservation& a, const sdk::FiducialObservation& b) {
                         return rank(a.id) < rank(b.id);
                     });
    return frame;
}

inline sdk::CameraSetup rearSetup() {
    sdk::CameraSetup setup;
    setup.cameraId = "ll-rear";
    setup.vendor = sdk::CameraVendor::Limelight;
    setup.ntName = kTable;
    setup.widthPx = 1280;
    setup.heightPx = 800;
    setup.fxPx = setup.fyPx = 640.0;
    setup.cxPx = 640.0;
    setup.cyPx = 400.0;
    setup.frameRateHz = 30.0;
    setup.latencyUs = 32000;
    setup.defaultAssumedRobotToCamera = rearMount();
    return setup;
}

class RecordingLogger final : public sdk::ILogger {
public:
    struct Entry {
        sdk::LogLevel level;
        std::string category;
        std::string message;
    };
    void log(sdk::LogLevel level, std::string_view category, std::string_view message) override {
        entries.push_back({level, std::string(category), std::string(message)});
    }
    bool has(sdk::LogLevel level, std::string_view part) const {
        for (const Entry& e : entries)
            if (e.level == level && e.message.find(part) != std::string::npos) return true;
        return false;
    }
    std::vector<Entry> entries;
};

// FakeNtApi + the publisher of one rear Limelight, clock at 1 s.
struct Rig {
    FakeNtApi nt;
    RecordingLogger log;
    std::unique_ptr<bridges::vision::IVisionVendor> vendor = bridges::vision::makeLimelightVendor();
    std::unique_ptr<ICameraPublisher> camera;

    explicit Rig(const sdk::CameraSetup& setup = rearSetup()) {
        nt.setNowUs(1'000'000);
        camera = vendor->createPublisher(nt, log, setup);
    }
    // Publishes `frame` with the clock at `nowUs`.
    void publishAt(std::int64_t nowUs, const sdk::VisionFrame& frame) {
        nt.setNowUs(nowUs);
        camera->publish(frame);
    }
};

// ---- reading what the publisher wrote --------------------------------------------------------------------------
inline double number(const FakeNtApi& nt, std::string_view key) {
    const auto value = nt.latest(topic(key));
    return value ? value->number : std::nan("");
}
inline std::vector<double> array(const FakeNtApi& nt, std::string_view key) {
    const auto value = nt.latest(topic(key));
    return value ? value->numbers : std::vector<double>{};
}
// Number of accepted values of `key` stamped exactly `timeUs`.
inline int countAt(const FakeNtApi& nt, std::string_view key, std::int64_t timeUs) {
    int n = 0;
    for (const auto& v : nt.history(topic(key)))
        if (v.timeUs == timeUs) ++n;
    return n;
}

// |a - b| in degrees, modulo 360.
inline double angleDiffDeg(double a, double b) { return std::fabs(ll::wrapDegrees(a - b)); }

// What the robot writes through LimelightHelpers (the NT time is the robot's own FPGA time).
inline void robotSetArray6(FakeNtApi& nt, std::string_view key, const std::array<double, 6>& v, std::int64_t timeUs) {
    nt.robotSetDoubleArray(topic(key), {v[0], v[1], v[2], v[3], v[4], v[5]}, timeUs);
}
inline void robotSetOrientation(FakeNtApi& nt, double yawDeg, std::int64_t timeUs) {
    robotSetArray6(nt, ll::kRobotOrientationSet, {yawDeg, 0.0, 0.0, 0.0, 0.0, 0.0}, timeUs);
}
inline void robotSetMount(FakeNtApi& nt, const std::array<double, 6>& robotSpace, std::int64_t timeUs) {
    robotSetArray6(nt, ll::kCameraPoseRobotSpaceSet, robotSpace, timeUs);
}
inline void robotSetNumber(FakeNtApi& nt, std::string_view key, double value, std::int64_t timeUs) {
    nt.robotSetDouble(topic(key), value, timeUs);
}

// r' = p - Rz(psi - theta) (p - r): MegaTag2's translation for a LEVEL robot at r with true yaw theta whose assumed
// mount equals the physical one, solved from a tag at p with the yaw psi the robot supplied. Derived by hand: the
// camera orientation becomes Rz(psi) * mount rotation and the camera-to-tag vector keeps its camera-frame components.
// The tests thus do not call the code under test. Positions are x, y.
inline std::array<double, 2> expectedMegaTag2Xy(double tagX, double tagY, double robotX, double robotY,
                                                double trueYawDeg, double suppliedYawDeg) {
    const double delta = (suppliedYawDeg - trueYawDeg) * kDeg;
    const double dx = tagX - robotX;
    const double dy = tagY - robotY;
    return {tagX - (std::cos(delta) * dx - std::sin(delta) * dy),
            tagY - (std::sin(delta) * dx + std::cos(delta) * dy)};
}

}  // namespace wpiemu::tests::limelight
