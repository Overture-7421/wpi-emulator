#pragma once

// Qt-free camera rig for the run view (SDK frame: Z-up NWU). Three modes: free orbit around a target,
// chase camera that follows the robot, and a top-down field view.

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::app::runview {

enum class CameraMode { Orbit, Follow, TopDown };

struct CameraPose {
    sdk::Vec3 position;
    sdk::Quat rotation;   // camera looks along its local -Z, +Y up (Qt Quick 3D convention)
    float fieldOfViewDeg = 45.f;
};

class CameraController {
public:
    CameraController();

    void setMode(CameraMode mode);
    CameraMode mode() const { return mode_; }

    // Playing-area centre and extent: used for the default framing.
    void setField(const sdk::Vec3& center, float lengthM, float widthM);
    // Robot pose in the world, updated every frame (drives Follow and the orbit "recenter" action).
    void setRobot(const sdk::Vec3& position, float yawRad);

    // Pointer input in pixels. Left drag: orbit (Orbit/Follow) or pan (TopDown). Pan: shift/right/middle drag.
    void orbit(float dxPx, float dyPx);
    void pan(float dxPx, float dyPx);
    void zoom(float steps);   // >0 zooms in
    void reset();             // default framing for the current mode
    void centerOnRobot();     // Orbit: move the orbit target to the robot

    // Smooth the follow target. dt in seconds.
    void advance(double dtSeconds);
    CameraPose pose() const;

    float distance() const { return distance_; }
    float yaw() const { return yaw_; }
    float pitch() const { return pitch_; }
    sdk::Vec3 target() const { return target_; }

    static sdk::Quat lookRotation(const sdk::Vec3& forward, const sdk::Vec3& upHint);

private:
    void clampState();

    CameraMode mode_ = CameraMode::Orbit;
    sdk::Vec3 fieldCenter_{8.2f, 4.1f, 0.f};
    float fieldLength_ = 16.46f;
    float fieldWidth_ = 8.23f;
    sdk::Vec3 robotPos_;
    float robotYaw_ = 0.f;

    // Orbit
    sdk::Vec3 target_{8.2f, 4.1f, 0.f};
    float yaw_ = 3.1415926f;   // azimuth of the camera around the target
    float pitch_ = 0.7f;
    float distance_ = 12.f;
    // Follow
    sdk::Vec3 followTarget_;
    float followYawOffset_ = 0.f;
    float followPitch_ = 0.45f;
    float followDistance_ = 3.5f;
    float followYaw_ = 0.f;    // smoothed robot heading
    // Top-down
    sdk::Vec3 topCenter_{8.2f, 4.1f, 0.f};
    float topHeight_ = 14.f;
};

}  // namespace wpiemu::app::runview
