#pragma once

// Qt-free vision overlay logic (ADR 0017): turns the camera and AprilTag part of an ISimSnapshot
// into what the run view draws in the world frame (SDK Z-up; the Y-up conversion stays in
// RunScene.qml). Mirrors ZoneRig: update() is called per snapshot, does not allocate in steady
// state and reports what changed, so the models only emit for rows that moved.

#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include "common/orbit_camera.h"
#include "scene_parts.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::runview {

// Depth of the drawn viewing pyramid, and the dimensions of the thin parts.
constexpr double kFrustumDepthM = 2.0;
constexpr float kTagPlateThicknessM = 0.02f;
constexpr float kRayThicknessM = 0.012f;
// Labels of tags nobody sees and of cameras are shown only while the viewer is this close (the home
// view would be a wall of text); a tag a camera sees is always labelled.
constexpr float kTagLabelRangeM = 6.f;
constexpr float kCameraLabelRangeM = 12.f;
// Labels whose anchor is closer than this to the edge of the view are hidden instead of being cut
// off.
constexpr float kLabelInsetPx = 8.f;

// What colour class a camera is drawn in.
enum class CameraState {
    Ok,        // enabled and the robot process publishes it
    Waiting,   // enabled, not published (yet): the robot code has not created or used the camera
    Disabled,  // switched off in the project
    Problem,   // the robot process reports a status (unsupported PhotonLib, foreign publisher, ...)
};

CameraState cameraStateOf(const sdk::CameraSample& camera);

struct CameraOverlay {
    std::string id;
    std::string label;  // may be empty: the id is shown
    sdk::CameraVendor vendor = sdk::CameraVendor::PhotonVision;
    CameraState state = CameraState::Waiting;
    bool seesTags = false;  // the latest capture contains a tag: drawn brighter
    sdk::Pose3 pose;        // camera frame in the world frame (+X = optical axis)
    double hfovRad = 0.0;
    double vfovRad = 0.0;
    std::vector<int> visibleTagIds;

    bool operator==(const CameraOverlay&) const = default;
};

enum class LabelKind { Tag, Camera };

// A text the view draws as a 2D item at the screen position of a world point.
struct OverlayLabel {
    LabelKind kind = LabelKind::Tag;
    std::string text;
    sdk::Vec3 anchor;         // world frame
    bool emphasized = false;  // a tag a camera sees / a camera that sees a tag

    bool operator==(const OverlayLabel&) const = default;
};

struct LabelPlacement {
    bool visible = false;
    float xPx = 0.f;
    float yPx = 0.f;
};

// Where `label` lands in a `widthPx` x `heightPx` view seen from `eye`. Hidden when the anchor is
// behind the camera or off screen, for a tag no camera sees while the viewer is farther than
// kTagLabelRangeM, and for a camera farther than kCameraLabelRangeM.
LabelPlacement placeLabel(const OverlayLabel& label, const OrbitCamera& eye, float widthPx,
                          float heightPx);

// The cameras, the field's AprilTags and the rays between them. Everything is derived from the
// snapshot alone (ISimSnapshot::cameras() / fiducials()), so a project without cameras yields empty
// lists.
class CameraRig {
public:
    // Returns true if anything changed. nullptr (no run) empties the rig.
    bool update(const sdk::ISimSnapshot* snapshot);

    // One entry per CameraSample. `camerasRebuilt()`: the list or the optics changed in the last
    // update(), so the models must be rebuilt rather than updated row by row.
    const std::vector<CameraOverlay>& cameras() const { return cameras_; }
    const std::vector<char>& camerasChanged() const { return camerasChanged_; }
    bool camerasRebuilt() const { return camerasRebuilt_; }

    // One thin plate per tag in the snapshot's order; `highlighted` while any camera sees it.
    const std::vector<ScenePart>& tagParts() const { return tagParts_; }
    const std::vector<char>& tagsChanged() const { return tagsChanged_; }
    bool tagsRebuilt() const { return tagsRebuilt_; }

    // A pool with one thin box per (camera, tag) pair: ray (c, t) is rayParts()[c *
    // tagParts().size() + t], `visible` only while camera c sees tag t. It is rebuilt whenever the
    // cameras or the tags are.
    const std::vector<ScenePart>& rayParts() const { return rayParts_; }
    const std::vector<char>& raysChanged() const { return raysChanged_; }

    // The tag ids first (snapshot order), then the camera labels.
    const std::vector<OverlayLabel>& labels() const { return labels_; }
    const std::vector<char>& labelsChanged() const { return labelsChanged_; }

private:
    bool sameTags(std::span<const sdk::FiducialSample> tags) const;
    bool sameCameras(std::span<const sdk::CameraSample> cameras) const;
    void rebuildTags(std::span<const sdk::FiducialSample> tags);
    void rebuildCameras(std::span<const sdk::CameraSample> cameras);
    bool updateCamera(std::size_t index, const sdk::CameraSample& sample);
    bool updateTag(std::size_t index);
    bool updateRay(std::size_t camera, std::size_t tag);
    bool updateLabel(std::size_t index);
    bool seen(std::size_t camera, std::size_t tag) const;

    std::vector<CameraOverlay> cameras_;
    std::vector<sdk::FiducialSample> tags_;
    std::vector<ScenePart> tagParts_;
    std::vector<ScenePart> rayParts_;
    std::vector<OverlayLabel> labels_;
    std::vector<char> camerasChanged_;
    std::vector<char> tagsChanged_;
    std::vector<char> raysChanged_;
    std::vector<char> labelsChanged_;
    bool camerasRebuilt_ = false;
    bool tagsRebuilt_ = false;
};

}  // namespace wpiemu::app::runview
