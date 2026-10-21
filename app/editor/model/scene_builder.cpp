#include "model/scene_builder.h"

#include <algorithm>
#include <cmath>

#include "common/camera_frustum.h"
#include "model/config_ops.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::editor {

using sdk::Pose3;
using sdk::Quat;
using sdk::Vec3;

namespace {

// Sizes (metres) and opacities of the markers, overlays and lines the editor adds around the robot's own shapes.
constexpr float kHullOpacity = 0.9f;
constexpr float kNoseLengthM = 0.024f;
constexpr float kNoseMaxWidthM = 0.25f;
constexpr float kNoseMinHeightM = 0.02f;
constexpr float kComMarkerM = 0.05f;
constexpr float kBareAnchorM = 0.1f;   // anchor of a mechanism without geometry
constexpr float kAnchorM = 0.03f;
constexpr float kBareAnchorOpacity = 0.6f;
constexpr float kAnchorOpacity = 0.9f;
constexpr float kAttachMinLengthM = 0.02f;
constexpr float kAttachLineM = 0.008f;
constexpr float kLinkMarkerM = 0.035f;
constexpr float kVolumeOverlayOpacity = 0.22f;
constexpr float kZoneVolumeOpacity = 0.28f;
constexpr float kMuzzleMarkerM = 0.06f;
constexpr float kMuzzleArrowLengthM = 0.25f;
constexpr float kMuzzleShaftM = 0.012f;
constexpr Vec3 kMuzzleHeadM{0.05f, 0.035f, 0.035f};
constexpr Vec3 kCameraBodyM{0.04f, 0.09f, 0.04f};
constexpr float kFrustumEdgeM = 0.006f;
constexpr float kFrustumPlaneThicknessM = 0.004f;
constexpr float kFrustumPlaneOpacity = 0.14f;

SceneItem fromPrimitive(const sdk::PrimitiveSpec& p, const Pose3& frame, std::string id, const SelectionRef& ref,
                        const SelectionRef& group, const QColor& fallback) {
    SceneItem item;
    item.id = std::move(id);
    item.ref = ref;
    item.group = group;
    item.pickable = true;
    item.role = ItemRole::Shape;
    item.world = frame * p.local;
    item.shape = p.kind;
    item.size = p.extentsM();
    item.color = ui::parseColor(p.color, fallback);
    return item;
}

SceneItem lineBetween(std::string id, const Vec3& a, const Vec3& b, const QColor& color, float thickness) {
    SceneItem item;
    item.id = std::move(id);
    item.role = ItemRole::Link;
    const Vec3 d = b - a;
    const float len = sdk::length(d);
    item.world = Pose3{(a + b) * 0.5f, rotationFromX(d)};
    item.size = Vec3{len, thickness, thickness};
    item.color = color;
    return item;
}

}  // namespace

SceneContent buildScene(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SceneTheme& theme) {
    SceneContent out;
    const Pose3 chassisFrame = ops::chassisFrameWorld(c.chassis);
    const SelectionRef chassisRef = SelectionRef::chassis();

    // Hull
    {
        SceneItem hull;
        hull.id = "hull";
        hull.ref = hull.group = chassisRef;
        hull.role = ItemRole::Hull;
        hull.world = ops::hullWorld(c.chassis);
        hull.size = c.chassis.sizeM;
        hull.color = theme.hull;
        hull.opacity = kHullOpacity;
        hull.pickable = true;
        out.items.push_back(hull);

        // A nose marker so the robot's forward (+X) is unmistakable.
        SceneItem nose;
        nose.id = "nose";
        nose.role = ItemRole::Marker;
        nose.world = Pose3{Vec3{c.chassis.sizeM.x * 0.5f + kNoseLengthM * 0.5f, 0.f, hull.world.position.z}, Quat{}};
        nose.size = Vec3{kNoseLengthM, std::min(kNoseMaxWidthM, c.chassis.sizeM.y * 0.5f), std::max(kNoseMinHeightM, c.chassis.sizeM.z * 0.5f)};
        nose.color = theme.accent;
        out.items.push_back(nose);
    }
    for (std::size_t i = 0; i < c.chassis.primitives.size(); ++i)
        out.items.push_back(fromPrimitive(c.chassis.primitives[i], chassisFrame, "chassis:" + std::to_string(i),
                                          SelectionRef::primitive(ops::kChassisOwner, static_cast<int>(i)), chassisRef, theme.shape));

    // Wheels
    for (const auto& w : c.drivetrain.spec.wheels) {
        SceneItem item;
        item.id = "wheel:" + w.name;
        item.ref = SelectionRef::wheel(w.name);
        item.group = item.ref;
        item.role = ItemRole::Wheel;
        item.shape = sdk::PrimitiveKind::Cylinder;
        item.world = Pose3{ops::wheelWorld(c.chassis, w).position, Quat::fromAxisAngle({1.f, 0.f, 0.f}, static_cast<float>(sdk::kPi * 0.5))};
        item.size = Vec3{w.radiusM * 2.f, w.radiusM * 2.f, w.widthM};
        item.color = theme.wheel;
        item.pickable = true;
        out.items.push_back(item);
    }

    // COM marker
    {
        SceneItem com;
        com.id = "com";
        com.ref = com.group = SelectionRef::com();
        com.role = ItemRole::Com;
        com.world = chassisFrame;
        com.size = Vec3{kComMarkerM, kComMarkerM, kComMarkerM};
        com.color = theme.com;
        com.pickable = true;
        out.items.push_back(com);
    }

    // Mechanisms
    for (const auto& m : c.mechanisms) {
        const Pose3 root = ops::mechanismRootWorld(c, catalog, m.id);
        const Pose3 parent = ops::mechanismParentWorld(c, catalog, m);
        const SelectionRef mechRef = SelectionRef::mechanism(m.id);

        for (std::size_t i = 0; i < m.geometry.size(); ++i)
            out.items.push_back(fromPrimitive(m.geometry[i], root, "mech:" + m.id + ":" + std::to_string(i),
                                              SelectionRef::primitive(m.id, static_cast<int>(i)), mechRef, theme.mechanism));
        SceneItem anchor;
        anchor.id = "anchor:" + m.id;
        anchor.ref = anchor.group = mechRef;
        anchor.role = ItemRole::Marker;
        anchor.world = root;
        anchor.size = m.geometry.empty() ? Vec3{kBareAnchorM, kBareAnchorM, kBareAnchorM} : Vec3{kAnchorM, kAnchorM, kAnchorM};
        anchor.color = theme.mechanism;
        anchor.opacity = m.geometry.empty() ? kBareAnchorOpacity : kAnchorOpacity;
        anchor.pickable = true;
        out.items.push_back(anchor);

        if (sdk::length(root.position - parent.position) > kAttachMinLengthM)
            out.items.push_back(lineBetween("attach:" + m.id, parent.position, root.position, theme.link, kAttachLineM));

        if (const auto* info = catalog.findTemplate(m.type))
            for (const auto& link : info->links) {
                if (const auto p = ops::linkInChassis(c, catalog, m.id, link.name)) {
                    SceneItem marker;
                    marker.id = "link:" + m.id + ":" + link.name;
                    marker.role = ItemRole::Marker;
                    marker.world = chassisFrame * *p;
                    marker.size = Vec3{kLinkMarkerM, kLinkMarkerM, kLinkMarkerM};
                    marker.color = theme.link;
                    out.items.push_back(marker);
                }
            }

        for (const auto& vol : ops::volumesFor(c, catalog, m.id)) {
            const sdk::Value* store = ops::volumeStore(c, catalog, m.id);
            sdk::Box3 box;
            const sdk::Value* raw = store != nullptr ? store->find(vol.key) : nullptr;
            if (!sdk::fromValue(raw != nullptr ? *raw : vol.defaultValue, box)) continue;
            SceneItem ov;
            ov.id = "overlay:" + m.id + ":" + vol.key;
            ov.ref = SelectionRef::overlay(m.id, vol.key);
            ov.group = mechRef;   // highlighted together with its mechanism
            ov.role = ItemRole::Overlay;
            ov.world = root * box.pose;
            ov.size = box.sizeM;
            ov.color = ui::parseColor(vol.color, theme.accent);
            ov.opacity = kVolumeOverlayOpacity;
            ov.pickable = true;
            out.items.push_back(ov);
        }
    }

    // Plugin-level volumes live in the chassis frame.
    for (const auto& vol : ops::volumesFor(c, catalog, "")) {
        const sdk::Value* store = ops::volumeStore(c, catalog, "");
        sdk::Box3 box;
        const sdk::Value* raw = store != nullptr ? store->find(vol.key) : nullptr;
        if (!sdk::fromValue(raw != nullptr ? *raw : vol.defaultValue, box)) continue;
        SceneItem ov;
        ov.id = "overlay::" + vol.key;
        ov.ref = SelectionRef::overlay({}, vol.key);
        ov.group = SelectionRef::plugin();
        ov.role = ItemRole::Overlay;
        ov.world = chassisFrame * box.pose;
        ov.size = box.sizeM;
        ov.color = ui::parseColor(vol.color, theme.accent);
        ov.opacity = kVolumeOverlayOpacity;
        ov.pickable = true;
        out.items.push_back(ov);
    }
    // Zones: translucent volumes, or a marker with a direction arrow (+X) for muzzles.
    for (const auto& z : c.zones) {
        const sdk::ZoneKindInfo* info = catalog.findZoneKind(z.kind);
        const SelectionRef ref = SelectionRef::zone(z.id);
        const QColor color = info != nullptr ? ui::parseColor(info->color, theme.accent) : theme.accent;
        const Pose3 world = ops::zoneWorld(c, catalog, z);
        if (info == nullptr || info->shape == sdk::ZoneShape::Volume) {
            SceneItem item;
            item.id = "zone:" + z.id;
            item.ref = item.group = ref;
            item.role = ItemRole::Overlay;
            item.world = world;
            item.size = z.sizeM;
            item.color = color;
            item.opacity = kZoneVolumeOpacity;
            item.pickable = true;
            out.items.push_back(item);
            continue;
        }
        SceneItem marker;
        marker.id = "zone:" + z.id;
        marker.ref = marker.group = ref;
        marker.role = ItemRole::Marker;
        marker.world = world;
        marker.size = Vec3{kMuzzleMarkerM, kMuzzleMarkerM, kMuzzleMarkerM};
        marker.color = color;
        marker.pickable = true;
        out.items.push_back(marker);

        const Vec3 tip = world.transformPoint(Vec3{kMuzzleArrowLengthM, 0.f, 0.f});
        SceneItem shaft = lineBetween("zone:" + z.id + ":dir", world.position, tip, color, kMuzzleShaftM);
        shaft.ref = shaft.group = ref;
        shaft.pickable = true;
        out.items.push_back(shaft);
        SceneItem head;
        head.id = "zone:" + z.id + ":head";
        head.ref = head.group = ref;
        head.role = ItemRole::Marker;
        head.world = Pose3{tip, world.rotation};
        head.size = kMuzzleHeadM;
        head.color = color;
        head.pickable = true;
        out.items.push_back(head);
    }

    // Cameras: a small body box at the mount and the field of view as 8 edge lines plus a translucent far plane.
    for (const auto& cam : c.cameras) {
        const SelectionRef ref = SelectionRef::camera(cam.id);
        const QColor color = cam.kind == sdk::CameraKind::PhotonVision ? theme.photonVision : theme.limelight;
        const Pose3 world = ops::cameraWorld(c, catalog, cam);

        SceneItem body;
        body.id = "camera:" + cam.id;
        body.ref = body.group = ref;
        body.role = ItemRole::Marker;
        body.world = world;
        body.size = kCameraBodyM;
        body.color = color;
        body.pickable = true;
        out.items.push_back(body);

        const FrustumGeometry frustum = cameraFrustum(cam);
        for (std::size_t i = 0; i < kFrustumEdges.size(); ++i) {
            const auto [a, b] = kFrustumEdges[i];
            SceneItem edge = lineBetween("camera:" + cam.id + ":edge:" + std::to_string(i), world.transformPoint(frustum.pts[a]),
                                         world.transformPoint(frustum.pts[b]), color, kFrustumEdgeM);
            edge.ref = edge.group = ref;
            out.items.push_back(edge);
        }
        SceneItem plane;
        plane.id = "camera:" + cam.id + ":fov";
        plane.ref = plane.group = ref;
        plane.role = ItemRole::Overlay;
        plane.world = world * Pose3{Vec3{frustum.lengthM, 0.f, 0.f}, Quat{}};
        plane.size = Vec3{kFrustumPlaneThicknessM, std::fabs(frustum.pts[kFrustumTopLeft].y) * 2.f, std::fabs(frustum.pts[kFrustumTopLeft].z) * 2.f};
        plane.color = color;
        plane.opacity = kFrustumPlaneOpacity;
        plane.pickable = true;
        out.items.push_back(plane);
    }
    return out;
}

}  // namespace wpiemu::app::editor
