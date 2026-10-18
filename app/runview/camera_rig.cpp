#include "camera_rig.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::app::runview {
namespace {

constexpr const char* kTagDimColor = "#8fa3c0";
constexpr const char* kTagSeenColor = "#22d3ee";
constexpr const char* kRayColor = "#22d3ee";
constexpr float kTagDimOpacity = 0.45f;
constexpr float kTagSeenOpacity = 0.95f;
constexpr float kRayOpacity = 0.8f;
constexpr float kDefaultTagSizeM = 0.1651f;
constexpr float kTagLabelLiftM = 0.06f;     // above the top edge of the plate
constexpr float kCameraLabelLiftM = 0.14f;  // above the camera body

// Rotation that takes +X to the unit vector `d`.
sdk::Quat rotationFromX(const sdk::Vec3& d) {
    const float yaw = std::atan2(d.y, d.x);
    const float elevation = std::asin(std::clamp(d.z, -1.f, 1.f));
    return sdk::Quat::fromYaw(yaw) * sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, -elevation);
}

bool contains(const std::vector<int>& ids, int id) {
    return std::find(ids.begin(), ids.end(), id) != ids.end();
}

// Assigns only when different, so a vector keeps its storage in steady state.
template <typename T>
bool assign(T& dst, const T& src) {
    if (dst == src) return false;
    dst = src;
    return true;
}

float edgeOf(const sdk::FiducialSample& tag) {
    return tag.sizeM > 0.f ? tag.sizeM : kDefaultTagSizeM;
}

}  // namespace

CameraState cameraStateOf(const sdk::CameraSample& camera) {
    if (!camera.enabled || camera.status == "disabled") return CameraState::Disabled;
    if (!camera.status.empty()) return CameraState::Problem;
    return camera.robotConnected ? CameraState::Ok : CameraState::Waiting;
}

LabelPlacement placeLabel(const OverlayLabel& label, const OrbitCamera& eye, float widthPx,
                          float heightPx) {
    LabelPlacement out;
    const auto pixel = eye.project(label.anchor, widthPx, heightPx);
    if (!pixel) return out;  // behind the camera
    const auto [x, y] = *pixel;
    if (x < kLabelInsetPx || x > widthPx - kLabelInsetPx || y < kLabelInsetPx ||
        y > heightPx - kLabelInsetPx)
        return out;  // off screen
    // A tag a camera sees is always labelled; the other labels are for a viewer who is close enough
    // to read them.
    const bool alwaysShown = label.kind == LabelKind::Tag && label.emphasized;
    const float rangeM = label.kind == LabelKind::Tag ? kTagLabelRangeM : kCameraLabelRangeM;
    if (!alwaysShown && sdk::length(label.anchor - eye.position()) > rangeM) return out;
    out.visible = true;
    out.xPx = x;
    out.yPx = y;
    return out;
}

bool CameraRig::sameTags(std::span<const sdk::FiducialSample> tags) const {
    if (tags.size() != tags_.size()) return false;
    for (std::size_t i = 0; i < tags.size(); ++i)
        if (tags[i].id != tags_[i].id || !(tags[i].pose == tags_[i].pose) ||
            tags[i].sizeM != tags_[i].sizeM)
            return false;
    return true;
}

bool CameraRig::sameCameras(std::span<const sdk::CameraSample> cameras) const {
    if (cameras.size() != cameras_.size()) return false;
    for (std::size_t i = 0; i < cameras.size(); ++i)
        if (cameras[i].id != cameras_[i].id || cameras[i].hfovRad != cameras_[i].hfovRad ||
            cameras[i].vfovRad != cameras_[i].vfovRad)
            return false;
    return true;
}

void CameraRig::rebuildTags(std::span<const sdk::FiducialSample> tags) {
    tags_.assign(tags.begin(), tags.end());
    tagParts_.clear();
    tagParts_.reserve(tags_.size());
    for (const sdk::FiducialSample& tag : tags_) {
        ScenePart plate;
        plate.name = "tag " + std::to_string(tag.id);
        const float edge = edgeOf(tag);
        plate.size = {kTagPlateThicknessM, edge,
                      edge};  // thin along the tag's +X, its outward normal
        plate.pose = tag.pose * sdk::Pose3({kTagPlateThicknessM * 0.5f, 0.f, 0.f},
                                           sdk::Quat{});  // proud of the surface
        plate.color = kTagDimColor;
        plate.translucent = true;
        plate.opacity = kTagDimOpacity;
        tagParts_.push_back(std::move(plate));
    }
}

void CameraRig::rebuildCameras(std::span<const sdk::CameraSample> cameras) {
    cameras_.assign(cameras.size(), CameraOverlay{});
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        cameras_[i].id = cameras[i].id;
        cameras_[i].hfovRad = cameras[i].hfovRad;
        cameras_[i].vfovRad = cameras[i].vfovRad;
    }
}

bool CameraRig::updateCamera(std::size_t index, const sdk::CameraSample& sample) {
    CameraOverlay& overlay = cameras_[index];
    bool changed = assign(overlay.label, sample.label);
    changed |= assign(overlay.vendor, sample.vendor);
    changed |= assign(overlay.state, cameraStateOf(sample));
    changed |= assign(overlay.seesTags, !sample.visibleTagIds.empty());
    changed |= assign(overlay.pose, sample.pose);
    changed |= assign(overlay.visibleTagIds, sample.visibleTagIds);
    return changed;
}

bool CameraRig::seen(std::size_t camera, std::size_t tag) const {
    return contains(cameras_[camera].visibleTagIds, tags_[tag].id);
}

bool CameraRig::updateTag(std::size_t index) {
    bool highlighted = false;
    for (std::size_t c = 0; c < cameras_.size() && !highlighted; ++c) highlighted = seen(c, index);
    ScenePart& plate = tagParts_[index];
    if (plate.highlighted == highlighted) return false;
    plate.highlighted = highlighted;
    plate.opacity = highlighted ? kTagSeenOpacity : kTagDimOpacity;
    plate.color = highlighted ? kTagSeenColor : kTagDimColor;
    return true;
}

bool CameraRig::updateRay(std::size_t camera, std::size_t tag) {
    ScenePart& ray = rayParts_[camera * tags_.size() + tag];
    const sdk::Vec3 delta = tags_[tag].pose.position - cameras_[camera].pose.position;
    const float length = sdk::length(delta);
    if (!seen(camera, tag) || length < 1e-3f) {
        if (!ray.visible) return false;
        ray.visible = false;
        return true;
    }
    const sdk::Pose3 pose{cameras_[camera].pose.position + delta * 0.5f,
                          rotationFromX(delta / length)};
    const sdk::Vec3 size{length, kRayThicknessM, kRayThicknessM};
    if (ray.visible && ray.pose == pose && ray.size == size) return false;
    ray.visible = true;
    ray.pose = pose;
    ray.size = size;
    return true;
}

bool CameraRig::updateLabel(std::size_t index) {
    OverlayLabel& label = labels_[index];
    if (index < tagParts_.size()) return assign(label.emphasized, tagParts_[index].highlighted);
    const CameraOverlay& camera = cameras_[index - tagParts_.size()];
    bool changed = assign(label.text, camera.label.empty() ? camera.id : camera.label);
    changed |= assign(label.anchor, camera.pose.position + sdk::Vec3{0.f, 0.f, kCameraLabelLiftM});
    changed |= assign(label.emphasized, camera.seesTags);
    return changed;
}

bool CameraRig::update(const sdk::ISimSnapshot* snapshot) {
    const std::span<const sdk::CameraSample> cameras =
        snapshot != nullptr ? snapshot->cameras() : std::span<const sdk::CameraSample>{};
    const std::span<const sdk::FiducialSample> tags =
        snapshot != nullptr ? snapshot->fiducials() : std::span<const sdk::FiducialSample>{};

    tagsRebuilt_ = !sameTags(tags);
    camerasRebuilt_ = !sameCameras(cameras);
    if (tagsRebuilt_) rebuildTags(tags);
    if (camerasRebuilt_) rebuildCameras(cameras);
    const bool rebuilt = tagsRebuilt_ || camerasRebuilt_;
    if (rebuilt) {
        ScenePart ray;
        ray.name = "ray";
        ray.color = kRayColor;
        ray.translucent = true;
        ray.opacity = kRayOpacity;
        ray.visible = false;
        rayParts_.assign(cameras_.size() * tags_.size(), ray);

        labels_.clear();
        for (const sdk::FiducialSample& tag : tags_) {
            OverlayLabel label;
            label.kind = LabelKind::Tag;
            label.text = std::to_string(tag.id);
            label.anchor =
                tag.pose.position + sdk::Vec3{0.f, 0.f, edgeOf(tag) * 0.5f + kTagLabelLiftM};
            labels_.push_back(std::move(label));
        }
        for (std::size_t i = 0; i < cameras_.size(); ++i) {
            OverlayLabel label;
            label.kind = LabelKind::Camera;
            labels_.push_back(std::move(label));
        }
    }
    camerasChanged_.assign(cameras_.size(), camerasRebuilt_ ? 1 : 0);
    tagsChanged_.assign(tagParts_.size(), tagsRebuilt_ ? 1 : 0);
    raysChanged_.assign(rayParts_.size(), rebuilt ? 1 : 0);
    labelsChanged_.assign(labels_.size(), rebuilt ? 1 : 0);

    bool any = rebuilt;
    const auto mark = [&any](std::vector<char>& flags, std::size_t index) {
        flags[index] = 1;
        any = true;
    };
    for (std::size_t c = 0; c < cameras_.size(); ++c)
        if (updateCamera(c, cameras[c])) mark(camerasChanged_, c);
    for (std::size_t t = 0; t < tagParts_.size(); ++t)
        if (updateTag(t)) mark(tagsChanged_, t);
    for (std::size_t c = 0; c < cameras_.size(); ++c)
        for (std::size_t t = 0; t < tagParts_.size(); ++t)
            if (updateRay(c, t)) mark(raysChanged_, c * tagParts_.size() + t);
    for (std::size_t l = 0; l < labels_.size(); ++l)
        if (updateLabel(l)) mark(labelsChanged_, l);
    return any;
}

}  // namespace wpiemu::app::runview
