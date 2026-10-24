#pragma once

// Qt-free scene logic: turns the robot config + live snapshot into renderable primitives in SDK
// coordinates (Z-up NWU). The view layer (SceneModel / QML) converts to Qt Quick 3D; nothing here
// knows about Qt or about any game.

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/game/default_field.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/vision/apriltag.h"

namespace wpiemu::app::runview {

struct ScenePart {
    std::string name;
    sdk::PrimitiveKind shape = sdk::PrimitiveKind::Box;
    sdk::Pose3 pose;          // parent frame (chassis frame for robot parts, world frame for field parts)
    sdk::Vec3 size{1.f, 1.f, 1.f};  // Box: full extents; Cylinder: (diameter, diameter, height), axis = local Z
    std::string color = sdk::kDefaultPrimitiveColor;
    bool translucent = false;
    float opacity = 0.28f;      // alpha when translucent
    bool highlighted = false;   // drawn brighter (an active zone)
};

// The robot rig: chassis frame/primitives, wheels (with a rotation marker) and mechanism primitives. All primitives
// of a mechanism follow its first link (the moving part), or stay on the mechanism root when it has none.
class RobotRig {
public:
    void setConfig(std::shared_ptr<const sdk::RobotConfig> config);
    // Recomputes every part's pose for `snapshot` (nullptr = rest pose). Returns true if any pose changed.
    bool update(const sdk::ISimSnapshot* snapshot);

    const std::vector<ScenePart>& parts() const { return parts_; }
    // Parts [0, staticCount) never move (chassis frame primitives).
    std::size_t staticCount() const { return staticCount_; }
    // Set after update(): which parts moved since the previous update.
    const std::vector<char>& changed() const { return changed_; }

private:
    struct Binding {
        enum class Kind { Static, Wheel, WheelMarker, Mechanism } kind = Kind::Static;
        std::string owner;        // mechanism id / wheel name
        std::string link = sdk::kRootLink;   // mechanism link
        sdk::Pose3 local;         // primitive pose in the owner frame
        float wheelRadius = 0.05f;
    };
    std::shared_ptr<const sdk::RobotConfig> config_;
    std::vector<ScenePart> parts_;
    std::vector<Binding> bindings_;
    std::vector<char> changed_;
    std::size_t staticCount_ = 0;
};

// A translucent volume or muzzle in the chassis frame: either an EditorPanelDescriptor overlay (tracksState
// false, always drawn the same) or a robot zone (RobotConfig::zones) whose `active` flag comes from the snapshot.
struct ZoneOverlay {
    std::string id;          // ZoneConfig::id; empty for panel overlays
    std::string label;
    std::string color = "#4488ff";
    sdk::Pose3 pose;         // chassis frame
    sdk::Vec3 size;          // Volume only
    bool muzzle = false;     // point + direction along the pose's local +X, drawn as a marker and an arrow
    bool tracksState = false;
    bool active = false;     // ISimSnapshot::zones(); inactive when the snapshot does not list the zone
};

// Resolves the overlays of every panel descriptor against the config (plugin sections and mechanism
// properties) in the chassis frame. Plugin-scope boxes are chassis-frame, mechanism-scope boxes are
// relative to the mechanism root frame (resolved through the attachment chain).
std::vector<ZoneOverlay> resolveZoneOverlays(const sdk::RobotConfig& config,
                                             const std::vector<sdk::EditorPanelDescriptor>& panels,
                                             const sdk::ISimSnapshot* snapshot);

// Resolves RobotConfig::zones whose kind is in `kinds` (others are ignored) in the chassis frame:
// pose = pose of the attachment link * zone.local, shape and colour from the kind, `active` from the snapshot.
std::vector<ZoneOverlay> resolveRobotZones(const sdk::RobotConfig& config, const std::vector<sdk::ZoneKindInfo>& kinds,
                                           const sdk::ISimSnapshot* snapshot);

// Drawable parts of every panel overlay and robot zone. Mirrors RobotRig: build once per scene, then update()
// per snapshot without allocating and report which parts changed (pose or active state).
class ZoneRig {
public:
    void setScene(std::shared_ptr<const sdk::RobotConfig> config, std::vector<sdk::EditorPanelDescriptor> panels,
                  std::vector<sdk::ZoneKindInfo> kinds);
    // Returns true if any part changed. nullptr = rest pose, every zone inactive.
    bool update(const sdk::ISimSnapshot* snapshot);

    const std::vector<ScenePart>& parts() const { return parts_; }
    // Grouping label of each part (the zone / overlay label), for the per-zone visibility toggles.
    const std::vector<std::string>& labels() const { return labels_; }
    const std::vector<char>& changed() const { return changed_; }
    // One entry per panel overlay / robot zone (muzzles own two parts), as of the last update().
    const std::vector<ZoneOverlay>& overlays() const { return overlays_; }

private:
    struct Binding {
        std::string mechanism;                 // empty = chassis frame
        std::string link = sdk::kRootLink;
        sdk::Pose3 local;
        std::size_t firstPart = 0;
    };
    std::shared_ptr<const sdk::RobotConfig> config_;
    std::vector<ZoneOverlay> overlays_;
    std::vector<Binding> bindings_;
    std::vector<ScenePart> parts_;
    std::vector<std::string> labels_;
    std::vector<char> changed_;
};

// A field AprilTag; `seen` = some camera listed its id in the latest snapshot.
struct TagOverlay {
    int id = 0;
    sdk::Pose3 pose;     // field frame, +X out of the face
    float sizeM = static_cast<float>(sdk::kFrcAprilTagSizeM);
    bool seen = false;
};

// The printed tag: a quad on the tag's face plane that the view textures with the 36h11 image for `id` (10x10 cells =
// 1 white margin cell + the 8x8 black square). Quad frame: +Z = face normal (tag +X), +X = image right (tag +Y),
// +Y = image up (tag +Z).
struct TagDecal {
    int id = 0;
    sdk::Pose3 pose;
    float sizeM = 0.f;   // side of the whole image
};

// Field-frame drawing of the AprilTag layout: one backing panel per tag (parts(), same order as tags(); grey, green
// while any camera lists the tag's id in the snapshot: the union of CameraState::visibleTagIds) plus the decals.
class TagRig {
public:
    void setLayout(const sdk::AprilTagLayout& layout);
    bool update(const sdk::ISimSnapshot* snapshot);

    const std::vector<ScenePart>& parts() const { return parts_; }
    const std::vector<char>& changed() const { return changed_; }
    const std::vector<TagOverlay>& tags() const { return tags_; }
    const std::vector<TagDecal>& decals() const { return decals_; }

private:
    std::vector<TagOverlay> tags_;
    std::vector<TagDecal> decals_;
    std::vector<ScenePart> parts_;
    std::vector<char> changed_;
};

// The floor drawn when no field description is given. Same size as the default core-only field (sdk/game/default_field.h).
inline constexpr float kGenericFieldLengthM = sdk::default_field::kLengthM;
inline constexpr float kGenericFieldWidthM = sdk::default_field::kWidthM;

// Static field parts (floor + visuals) and the XY centre of the playing area.
struct FieldParts {
    std::vector<ScenePart> parts;
    sdk::Vec3 center;       // z = 0
    float lengthM = kGenericFieldLengthM;
    float widthM = kGenericFieldWidthM;
};
FieldParts buildFieldParts(const sdk::FieldDescription* field);

// Formats NamedValue for display (radians shown as degrees, rad/s as rpm).
struct DisplayValue {
    std::string name;
    double value = 0.0;
    std::string unit;
};
DisplayValue toDisplayValue(const sdk::NamedValue& v);

}  // namespace wpiemu::app::runview
