#pragma once

// From the player's joystick to what the match client sends, as pure functions and a small state holder: chassis speeds with a
// deadband (the shared drive kinematics then turn them into wheel setpoints inside MatchClient), the zone buttons and the zone
// pose controls. The joystick is the DS joystick the run view's JoystickManager produces (keyboard keys or a gamepad, WPILib
// Xbox layout), so one binding profile serves both screens.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"

namespace wpiemu::app::client {

// ---- driving ---------------------------------------------------------------------------------------------------------

struct DriveLimits {
    float maxLinearMps = 0.f;          // the slowest wheel's top ground speed
    float maxAngularRadPerS = 0.f;     // spinning on the spot at that wheel speed
};
// The limits of an accepted spec (what the host simulates).
DriveLimits driveLimitsOf(const sdk::RobotConfig& spec);

// WPILib Xbox layout: left stick X strafes, left stick Y drives (up is negative), right stick X turns (left is negative).
inline constexpr int kStrafeAxis = 0;
inline constexpr int kForwardAxis = 1;
inline constexpr int kRotateAxis = 4;
inline constexpr int kPoseZAxis = 5;       // right stick Y: raises and lowers the selected zone
inline constexpr int kPoseYawLeftAxis = 2;   // left trigger turns the selected zone counter-clockwise
inline constexpr int kPoseYawRightAxis = 3;  // right trigger turns it clockwise
inline constexpr int kMaxZoneButtons = 12;   // buttons of a DS joystick profile (joy::kButtonCount)

struct DriveInputConfig {
    float deadband = 0.10f;            // stick travel around the centre that counts as zero, 0 to 0.9
    float linearScale = 1.f;           // fraction of top speed at full deflection, 0.05 to 1
    float angularScale = 1.f;
    bool fieldRelative = false;        // the sticks are field directions (the heading comes from the prediction)

    bool operator==(const DriveInputConfig&) const = default;
};
// The config with every value brought into range (a stored setting may be anything).
DriveInputConfig normalizedDriveConfig(DriveInputConfig config);

// 0 inside the deadband, then a straight line from 0 at its edge to +-1 at full travel (no jump at the edge).
float applyDeadband(float value, float deadband);

// The chassis speeds of the joystick state: the stick vector is limited to the unit circle so a diagonal is not faster, each
// part has the deadband, and full deflection is `limits` times the scale. A joystick that is not present drives nothing.
sdk::ChassisSpeeds mapDrive(const sdk::proto::JoystickState& stick, const DriveInputConfig& config, const DriveLimits& limits);

// ---- zones -----------------------------------------------------------------------------------------------------------

// Zone i is held active while button i of the joystick is held (A, B, X, Y, LB, RB, Back, Start ... or Z, X, C, V, Q, E, R, F
// on the keyboard profile), for the first `zoneCount` zones up to kMaxZoneButtons. Zones beyond that use the on-screen buttons.
std::uint32_t zoneFlagsOf(const sdk::proto::JoystickState& stick, std::size_t zoneCount);

// Moves the zones of the player's robot (chassis frame: +x forward, +y left, +z up), inside the envelope the host allows. Holds
// the pose of every zone it moved so a slider and the keys edit the same value; a zone that was never moved is not sent.
class ZonePoseControls {
public:
    static constexpr float kNudgeSpeedMps = 0.4f;
    static constexpr float kNudgeYawRadPerS = 1.5f;

    // A new robot: its chassis (the envelope) and where each zone rests. Forgets every moved zone and the selection.
    void configure(const sdk::ChassisSpec& chassis, std::vector<sdk::Pose3> restPoses);

    std::size_t zoneCount() const { return rest_.size(); }
    std::size_t selected() const { return selected_; }
    void select(std::size_t zone);
    bool moved(std::size_t zone) const { return zone < moved_.size() && moved_[zone] != 0; }
    const sdk::Pose3& pose(std::size_t zone) const;
    const sdk::Pose3& restPose(std::size_t zone) const;
    float yawOffsetRad(std::size_t zone) const { return zone < yaw_.size() ? yaw_[zone] : 0.f; }
    // The chassis-frame box a zone may be in (sliders' ranges).
    const sdk::Vec3& minPosition() const { return min_; }
    const sdk::Vec3& maxPosition() const { return max_; }

    // Sets the position (clamped into the envelope) and the yaw relative to the rest pose; marks the zone moved. False for a
    // zone that does not exist.
    bool set(std::size_t zone, const sdk::Vec3& position, float yawOffsetRad);
    // Back to the rest pose; the zone is no longer sent. True when it had been moved.
    bool reset(std::size_t zone);

    // The d-pad moves the selected zone (up forward, right to the robot's right), the right stick raises and lowers it, the
    // triggers turn it. Returns the zone that moved, or -1.
    int nudge(double dtSeconds, const sdk::proto::JoystickState& stick, float deadband);

private:
    void apply(std::size_t zone, const sdk::Vec3& position, float yaw);

    std::vector<sdk::Pose3> rest_;
    std::vector<sdk::Pose3> pose_;
    std::vector<float> yaw_;
    std::vector<char> moved_;
    std::size_t selected_ = 0;
    sdk::Vec3 min_, max_;
    sdk::ChassisSpec chassis_;
};

}  // namespace wpiemu::app::client
