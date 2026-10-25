#pragma once

#include <memory>
#include <optional>
#include <string>

#include "common/orbit_camera.h"
#include "wpiemu/sdk/game/field.h"

namespace wpiemu::app::runview {

struct CameraView {
    enum class Kind { Home, DriverPov, Station };
    Kind kind = Kind::Home;
    sdk::Alliance alliance = sdk::Alliance::Blue;
    int number = 1;
};

std::string cameraViewKey(const CameraView& view);
std::optional<CameraView> parseCameraViewKey(const std::string& key);

class CameraRig {
public:
    OrbitCamera& camera() { return camera_; }
    const OrbitCamera& camera() const { return camera_; }
    const CameraView& view() const { return view_; }
    bool follow() const { return follow_; }

    void setField(float lengthM, float widthM, const sdk::Vec3& center,
                  std::shared_ptr<const sdk::FieldDescription> description = {});
    void showHome();
    void showDriverPov();
    void showStation(sdk::Alliance alliance, int number);
    void restoreView(const CameraView& view);
    void setDriverStation(sdk::Alliance alliance, int number);
    void setFollow(bool follow) { follow_ = follow; }
    void setRobot(std::optional<sdk::Vec3> robot);
    void advance(double dtSeconds);
    void orbit(float dxPx, float dyPx);
    void pan(float dxPx, float dyPx, float viewHeightPx);
    void zoom(float steps);

private:
    bool isDriverView() const { return view_.kind != CameraView::Kind::Home; }
    sdk::Vec3 eyePosition() const;
    void applyView(bool resetFov);
    void aimAt(const sdk::Vec3& point);
    void updateCamera();
    void show(CameraView view, bool resetFov);

    OrbitCamera camera_;
    CameraView view_;
    float fieldLength_ = 16.541f;
    float fieldWidth_ = 8.069f;
    sdk::Vec3 fieldCenter_{8.2f, 4.1f, 0.f};
    std::shared_ptr<const sdk::FieldDescription> field_;
    sdk::Alliance driverAlliance_ = sdk::Alliance::Blue;
    int driverStation_ = 1;
    std::optional<sdk::Vec3> robot_;
    bool follow_ = true;
    float gazeYawRad_ = 0.f;
    float gazePitchRad_ = 0.f;
};

}  // namespace wpiemu::app::runview
