#include "camera_rig.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::app::runview {
namespace {

constexpr float kFollowTauSeconds = 0.15f;
constexpr float kDriverFovDeg = 45.f;
constexpr float kDriverMinFovDeg = 15.f;
constexpr float kDriverMaxFovDeg = 75.f;
constexpr float kGazePivotDistanceM = 10.f;

float wrapYaw(float yaw) {
    return static_cast<float>(sdk::wrapAngleRad(static_cast<double>(yaw)));
}

}  // namespace

std::string cameraViewKey(const CameraView& view) {
    if (view.kind == CameraView::Kind::Home) return "home";
    if (view.kind == CameraView::Kind::DriverPov) return "driver";
    const int number = std::clamp(view.number, 1, 3);
    return std::string(view.alliance == sdk::Alliance::Blue ? "blue" : "red") + std::to_string(number);
}

std::optional<CameraView> parseCameraViewKey(const std::string& key) {
    if (key == "home") return CameraView{};
    if (key == "driver") return CameraView{CameraView::Kind::DriverPov, sdk::Alliance::Blue, 1};
    const bool blue = key.size() == 5 && key.starts_with("blue");
    const bool red = key.size() == 4 && key.starts_with("red");
    if ((blue || red) && key.back() >= '1' && key.back() <= '3') {
        const sdk::Alliance alliance = blue ? sdk::Alliance::Blue : sdk::Alliance::Red;
        return CameraView{CameraView::Kind::Station, alliance, key.back() - '0'};
    }
    return std::nullopt;
}

void CameraRig::setField(float lengthM, float widthM, const sdk::Vec3& center,
                         std::shared_ptr<const sdk::FieldDescription> description) {
    fieldLength_ = lengthM > 0.f ? lengthM : 16.541f;
    fieldWidth_ = widthM > 0.f ? widthM : 8.069f;
    fieldCenter_ = center;
    field_ = std::move(description);
    applyView(false);
}

void CameraRig::showHome() {
    show(CameraView{}, false);
}

void CameraRig::showDriverPov() {
    show(CameraView{CameraView::Kind::DriverPov, driverAlliance_, driverStation_}, true);
}

void CameraRig::showStation(sdk::Alliance alliance, int number) {
    show(CameraView{CameraView::Kind::Station, alliance, std::clamp(number, 1, 3)}, true);
}

void CameraRig::restoreView(const CameraView& view) {
    if (view.kind == CameraView::Kind::Home) showHome();
    else if (view.kind == CameraView::Kind::DriverPov) showDriverPov();
    else showStation(view.alliance, view.number);
}

void CameraRig::setDriverStation(sdk::Alliance alliance, int number) {
    const int clamped = std::clamp(number, 1, 3);
    if (driverAlliance_ == alliance && driverStation_ == clamped) return;
    driverAlliance_ = alliance;
    driverStation_ = clamped;
    if (view_.kind == CameraView::Kind::DriverPov) {
        view_.alliance = alliance;
        view_.number = clamped;
        applyView(false);
    }
}

void CameraRig::setRobot(std::optional<sdk::Vec3> robot) {
    robot_ = robot;
}

void CameraRig::advance(double dtSeconds) {
    if (!isDriverView() || !follow_ || !robot_ || !std::isfinite(dtSeconds) || dtSeconds <= 0.0) return;
    const double dt = std::clamp(dtSeconds, 0.0, 0.1);
    if (dt == 0.0) return;
    const sdk::Vec3 delta = *robot_ - eyePosition();
    const float horizontal = std::hypot(delta.x, delta.y);
    const float targetYaw = std::atan2(delta.y, delta.x);
    const float targetPitch = std::atan2(delta.z, horizontal);
    const float alpha = static_cast<float>(1.0 - std::exp(-dt / kFollowTauSeconds));
    gazeYawRad_ = wrapYaw(gazeYawRad_ + wrapYaw(targetYaw - gazeYawRad_) * alpha);
    gazePitchRad_ += (targetPitch - gazePitchRad_) * alpha;
    updateCamera();
}

void CameraRig::orbit(float dxPx, float dyPx) {
    if (!isDriverView()) {
        camera_.orbit(dxPx, dyPx);
        return;
    }
    if (follow_) return;
    camera_.orbit(dxPx, dyPx);
    gazeYawRad_ = wrapYaw(camera_.yawRad - sdk::kPiF);
    gazePitchRad_ = -camera_.pitchRad;
}

void CameraRig::pan(float dxPx, float dyPx, float viewHeightPx) {
    if (!isDriverView()) {
        camera_.pan(dxPx, dyPx, viewHeightPx);
        return;
    }
    orbit(dxPx, dyPx);
}

void CameraRig::zoom(float steps) {
    if (!isDriverView()) {
        camera_.zoom(steps);
        return;
    }
    camera_.fovDeg = std::clamp(camera_.fovDeg * std::pow(0.88f, steps), kDriverMinFovDeg, kDriverMaxFovDeg);
}

sdk::Vec3 CameraRig::eyePosition() const {
    if (view_.kind == CameraView::Kind::DriverPov) {
        const auto spot = field_ ? sdk::driverStationSpot(*field_, driverAlliance_, driverStation_) : std::nullopt;
        if (spot) return spot->eyeM;
        sdk::FieldDescription generic{};
        generic.lengthM = fieldLength_;
        generic.widthM = fieldWidth_;
        const auto fallback = sdk::driverStationSpot(generic, driverAlliance_, driverStation_);
        return fallback ? fallback->eyeM : sdk::Vec3{};
    }
    if (field_) {
        const auto spot = sdk::driverStationSpot(*field_, view_.alliance, view_.number);
        if (spot) return spot->eyeM;
    }
    sdk::FieldDescription generic{};
    generic.lengthM = fieldLength_;
    generic.widthM = fieldWidth_;
    const auto fallback = sdk::driverStationSpot(generic, view_.alliance, view_.number);
    return fallback ? fallback->eyeM : sdk::Vec3{};
}

void CameraRig::applyView(bool resetFov) {
    if (!isDriverView()) {
        const float span = std::max(fieldLength_, fieldWidth_);
        camera_ = OrbitCamera{};
        camera_.target = fieldCenter_;
        camera_.yawRad = -sdk::degToRadF(90.f);
        camera_.pitchRad = 0.4f;
        camera_.distance = span * 1.05f;
        camera_.maxDistance = std::max(camera_.maxDistance, span * 6.f);
        return;
    }
    if (resetFov) camera_.fovDeg = kDriverFovDeg;
    aimAt(robot_.value_or(fieldCenter_));
    updateCamera();
}

void CameraRig::aimAt(const sdk::Vec3& point) {
    const sdk::Vec3 delta = point - eyePosition();
    const float horizontal = std::hypot(delta.x, delta.y);
    if (horizontal <= 1e-5f && std::abs(delta.z) <= 1e-5f) {
        gazeYawRad_ = view_.alliance == sdk::Alliance::Blue ? 0.f : sdk::kPiF;
        gazePitchRad_ = 0.f;
        return;
    }
    gazeYawRad_ = std::atan2(delta.y, delta.x);
    gazePitchRad_ = std::atan2(delta.z, horizontal);
}

void CameraRig::updateCamera() {
    if (isDriverView()) camera_.lookFrom(eyePosition(), gazeYawRad_, gazePitchRad_, kGazePivotDistanceM);
}

void CameraRig::show(CameraView view, bool resetFov) {
    if (view.kind == CameraView::Kind::Station) view.number = std::clamp(view.number, 1, 3);
    view_ = view;
    applyView(resetFov);
}

}  // namespace wpiemu::app::runview
