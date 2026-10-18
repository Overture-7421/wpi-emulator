#include "model/scene_builder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "common/camera_frustum.h"
#include "model/config_ops.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::editor {

using sdk::Pose3;
using sdk::Quat;
using sdk::Vec3;

namespace {

constexpr float kCameraBodyDepthM = 0.05f;
constexpr float kCameraBodyWidthM = 0.09f;
constexpr float kCameraBodyHeightM = 0.05f;
constexpr float kCameraLensDiameterM = 0.035f;
constexpr float kCameraLensLengthM = 0.02f;
constexpr float kTagPlateThicknessM = 0.012f;
constexpr float kTagRangeM = 7.0f;   // tags farther than this from the robot are left out: the floor patch ends there
constexpr double kMinCameraDepthM = 1e-3;

QColor parseColor(const std::string& s, const QColor& fallback) {
    const QColor c(QString::fromStdString(s));
    return c.isValid() ? c : fallback;
}

SceneItem fromPrimitive(const sdk::PrimitiveSpec& p, const Pose3& frame, std::string id, const SelectionRef& ref,
                        const SelectionRef& group, const QColor& fallback) {
    SceneItem item;
    item.id = std::move(id);
    item.ref = ref;
    item.group = group;
    item.pickable = true;
    item.role = ItemRole::Shape;
    item.world = frame * p.local;
    item.shape = p.kind == sdk::PrimitiveKind::Box ? ItemShape::Box : ItemShape::Cylinder;
    item.size = p.kind == sdk::PrimitiveKind::Box ? p.sizeM : Vec3{p.radiusM * 2.f, p.radiusM * 2.f, p.heightM};
    item.color = parseColor(p.color, fallback);
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

// A camera: a small body behind the lens, the lens, and the view pyramid (four edges to the far corners, the rectangle of the far plane
// and a translucent far plane). All of it belongs to the camera (it is highlighted with it); the body and the plane select it.
void addCameraItems(SceneContent& out, const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam, const SceneTheme& theme,
                    float depthM) {
    const SelectionRef ref = SelectionRef::camera(cam.id);
    const Pose3 world = ops::cameraWorld(c, catalog, cam);
    const QColor color = cam.enabled ? theme.camera : theme.cameraOff;
    const float strength = cam.enabled ? 1.f : 0.45f;   // a camera that is switched off is dimmed

    SceneItem body;
    body.id = "camera:" + cam.id;
    body.ref = body.group = ref;
    body.role = ItemRole::Marker;
    body.world = world * Pose3{Vec3{-0.5f * kCameraBodyDepthM, 0.f, 0.f}, Quat{}};   // the camera origin is the lens
    body.size = Vec3{kCameraBodyDepthM, kCameraBodyWidthM, kCameraBodyHeightM};
    body.color = color;
    body.opacity = 0.95f * strength + 0.05f;
    body.pickable = true;
    out.items.push_back(body);

    SceneItem lens;
    lens.id = "camera:" + cam.id + ":lens";
    lens.ref = lens.group = ref;
    lens.role = ItemRole::Marker;
    lens.shape = ItemShape::Cylinder;
    lens.world = world * Pose3{Vec3{0.5f * kCameraLensLengthM, 0.f, 0.f}, Quat::fromAxisAngle({0.f, 1.f, 0.f}, static_cast<float>(sdk::kPi * 0.5))};   // axis +Z -> +X
    lens.size = Vec3{kCameraLensDiameterM, kCameraLensDiameterM, kCameraLensLengthM};
    lens.color = color.darker(190);
    lens.opacity = body.opacity;
    lens.pickable = true;
    out.items.push_back(lens);

    const double hfov = ops::effectiveHfovRad(cam.intrinsics);
    const double vfov = hfov > 0.0 ? cam.intrinsics.vfovRad() : 0.0;
    if (!(hfov > 0.0 && std::isfinite(vfov) && vfov > 0.0) || !(depthM > 0.f)) return;   // an invalid lens: the validator reports it
    const camera::Frustum f = camera::frustum(world, hfov, vfov, depthM);

    const float edge = 0.006f;
    for (std::size_t i = 0; i < 4; ++i) {
        SceneItem ray = lineBetween("camera:" + cam.id + ":ray" + std::to_string(i), f.apex, f.farCorners[i], color, edge);
        SceneItem rim = lineBetween("camera:" + cam.id + ":rim" + std::to_string(i), f.farCorners[i], f.farCorners[(i + 1) % 4], color, edge);
        for (SceneItem* line : {&ray, &rim}) {
            line->role = ItemRole::Frustum;
            line->ref = line->group = ref;
            line->opacity = 0.42f * strength;
            out.items.push_back(*line);
        }
    }
    SceneItem plane;
    plane.id = "camera:" + cam.id + ":view";
    plane.ref = plane.group = ref;
    plane.role = ItemRole::Frustum;
    const Vec3 centre = (f.farCorners[0] + f.farCorners[2]) * 0.5f;
    plane.world = Pose3{centre, world.rotation};
    plane.size = Vec3{0.004f, sdk::length(f.farCorners[0] - f.farCorners[1]), sdk::length(f.farCorners[0] - f.farCorners[3])};
    plane.color = color;
    plane.opacity = 0.08f * strength;
    plane.pickable = true;
    out.items.push_back(plane);
}

}  // namespace

Quat rotationFromX(const Vec3& direction) {
    const Vec3 d = sdk::normalized(direction);
    if (sdk::length(d) < 0.5f) return Quat{};
    const float cosA = std::clamp(d.x, -1.f, 1.f);
    if (cosA > 0.99999f) return Quat{};
    if (cosA < -0.99999f) return Quat::fromAxisAngle({0.f, 0.f, 1.f}, static_cast<float>(sdk::kPi));
    const Vec3 axis = sdk::normalized(sdk::cross(Vec3{1.f, 0.f, 0.f}, d));
    return Quat::fromAxisAngle(axis, std::acos(cosA));
}

SceneContent buildScene(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SceneTheme& theme, const SceneOptions& options) {
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
        hull.opacity = 0.9f;
        hull.pickable = true;
        out.items.push_back(hull);

        // A nose marker so the robot's forward (+X) is unmistakable.
        SceneItem nose;
        nose.id = "nose";
        nose.role = ItemRole::Marker;
        nose.world = Pose3{Vec3{c.chassis.sizeM.x * 0.5f + 0.012f, 0.f, hull.world.position.z}, Quat{}};
        nose.size = Vec3{0.024f, std::min(0.25f, c.chassis.sizeM.y * 0.5f), std::max(0.02f, c.chassis.sizeM.z * 0.5f)};
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
        item.shape = ItemShape::Cylinder;
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
        com.size = Vec3{0.05f, 0.05f, 0.05f};
        com.color = theme.com;
        com.pickable = true;
        out.items.push_back(com);
    }

    // Start pose: the hull's footprint on the floor where the robot starts (fainter while that is only the default placement)
    // and a heading arrow whose tip turns it. Both select, and dragging them edits SimSettings::startPose.
    {
        const sdk::StartPose start = ops::effectiveStartPose(c, catalog);
        const Pose3 body = sdk::toPose3(start, 0.f);   // the chassis origin on the floor
        const Vec3 hullCentre{-c.chassis.comOffsetM.x, -c.chassis.comOffsetM.y, 0.f};   // in the chassis frame
        const SelectionRef ref = SelectionRef::startPose();

        SceneItem foot;
        foot.id = kStartPoseFootprintId;
        foot.ref = foot.group = ref;
        foot.role = ItemRole::Marker;
        foot.world = Pose3{body.transformPoint(hullCentre + Vec3{0.f, 0.f, 0.008f}), body.rotation};
        foot.size = Vec3{c.chassis.sizeM.x, c.chassis.sizeM.y, 0.012f};
        foot.color = theme.startPose;
        foot.opacity = c.sim.startPose ? 0.4f : 0.22f;
        foot.pickable = true;
        out.items.push_back(foot);

        const float front = hullCentre.x + c.chassis.sizeM.x * 0.5f;
        const Vec3 tail = body.transformPoint(Vec3{0.f, 0.f, 0.02f});
        const Vec3 tip = body.transformPoint(Vec3{front + 0.2f, 0.f, 0.02f});
        SceneItem shaft = lineBetween(std::string(kStartPoseNoseId) + ":shaft", tail, tip, theme.startPose, 0.02f);
        shaft.ref = shaft.group = ref;
        shaft.pickable = true;
        out.items.push_back(shaft);
        SceneItem head;
        head.id = kStartPoseNoseId;
        head.ref = head.group = ref;
        head.role = ItemRole::Marker;
        head.world = Pose3{tip, body.rotation};
        head.size = Vec3{0.1f, 0.1f, 0.03f};
        head.color = theme.startPose;
        head.pickable = true;
        out.items.push_back(head);
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
        anchor.size = m.geometry.empty() ? Vec3{0.1f, 0.1f, 0.1f} : Vec3{0.03f, 0.03f, 0.03f};
        anchor.color = theme.mechanism;
        anchor.opacity = m.geometry.empty() ? 0.6f : 0.9f;
        anchor.pickable = true;
        out.items.push_back(anchor);

        if (sdk::length(root.position - parent.position) > 0.02f)
            out.items.push_back(lineBetween("attach:" + m.id, parent.position, root.position, theme.link, 0.008f));

        if (const auto* info = catalog.findTemplate(m.type))
            for (const auto& link : info->links) {
                if (const auto p = ops::linkInChassis(c, catalog, m.id, link.name)) {
                    SceneItem marker;
                    marker.id = "link:" + m.id + ":" + link.name;
                    marker.role = ItemRole::Marker;
                    marker.world = chassisFrame * *p;
                    marker.size = Vec3{0.035f, 0.035f, 0.035f};
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
            ov.color = parseColor(vol.color, theme.accent);
            ov.opacity = 0.22f;
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
        ov.color = parseColor(vol.color, theme.accent);
        ov.opacity = 0.22f;
        ov.pickable = true;
        out.items.push_back(ov);
    }
    // Zones: translucent volumes, or a marker with a direction arrow (+X) for muzzles.
    for (const auto& z : c.zones) {
        const sdk::ZoneKindInfo* info = catalog.findZoneKind(z.kind);
        const SelectionRef ref = SelectionRef::zone(z.id);
        const QColor color = info != nullptr ? parseColor(info->color, theme.accent) : theme.accent;
        const Pose3 world = ops::zoneWorld(c, catalog, z);
        if (info == nullptr || info->shape == sdk::ZoneShape::Volume) {
            SceneItem item;
            item.id = "zone:" + z.id;
            item.ref = item.group = ref;
            item.role = ItemRole::Overlay;
            item.world = world;
            item.size = z.sizeM;
            item.color = color;
            item.opacity = 0.28f;
            item.pickable = true;
            out.items.push_back(item);
            continue;
        }
        SceneItem marker;
        marker.id = "zone:" + z.id;
        marker.ref = marker.group = ref;
        marker.role = ItemRole::Marker;
        marker.world = world;
        marker.size = Vec3{0.06f, 0.06f, 0.06f};
        marker.color = color;
        marker.pickable = true;
        out.items.push_back(marker);

        const Vec3 tip = world.transformPoint(Vec3{0.25f, 0.f, 0.f});
        SceneItem shaft = lineBetween("zone:" + z.id + ":dir", world.position, tip, color, 0.012f);
        shaft.ref = shaft.group = ref;
        shaft.pickable = true;
        out.items.push_back(shaft);
        SceneItem head;
        head.id = "zone:" + z.id + ":head";
        head.ref = head.group = ref;
        head.role = ItemRole::Marker;
        head.world = Pose3{tip, world.rotation};
        head.size = Vec3{0.05f, 0.035f, 0.035f};
        head.color = color;
        head.pickable = true;
        out.items.push_back(head);
    }

    // Cameras: a body with a lens and the pyramid they look into.
    const float depth = cameraFrustumDepthM(c, catalog);
    for (const auto& cam : c.vision.cameras) addCameraItems(out, c, catalog, cam, theme, depth);

    // AprilTags of the active layout, in the field frame as the robot sees it from its start pose: the tags near the robot appear where
    // it would find them. Plates are never picked; the ones the selected camera would see are tinted.
    if (options.showTags)
        if (const sdk::FiducialLayout* layout = catalog.activeFiducialLayout(c.vision)) {
            const Pose3 fieldInDisplay = ops::fieldToDisplay(c, catalog);
            const sdk::CameraConfig* viewer = options.cameraId.empty() ? nullptr : c.findCamera(options.cameraId);
            const Pose3 viewerWorld = viewer != nullptr ? ops::cameraWorld(c, catalog, *viewer) : Pose3{};
            for (const sdk::FiducialTag& tag : layout->tags) {
                const Pose3 world = fieldInDisplay * tag.pose;
                if (std::hypot(world.position.x, world.position.y) > kTagRangeM) continue;
                const float size = tag.sizeM > 0.f ? tag.sizeM : layout->tagSizeM;
                SceneItem plate;
                plate.id = "tag:" + std::to_string(tag.id);
                plate.role = ItemRole::Tag;
                plate.world = world * Pose3{Vec3{-0.5f * kTagPlateThicknessM, 0.f, 0.f}, Quat{}};   // behind the face, which is the tag pose
                plate.size = Vec3{kTagPlateThicknessM, size, size};
                const bool seen = viewer != nullptr && cameraSeesTag(*viewer, viewerWorld, world, viewer->tagSizeM > 0.0 ? viewer->tagSizeM : static_cast<double>(size));
                plate.color = seen ? theme.tagSeen : theme.tag;
                plate.opacity = 0.92f;
                plate.text = QString::number(tag.id);
                out.items.push_back(plate);
            }
        }
    return out;
}

float cameraFrustumDepthM(const sdk::RobotConfig& config, const EditorCatalog& catalog) {
    const sdk::FiducialLayout* layout = catalog.activeFiducialLayout(config.vision);
    if (layout == nullptr || !(layout->fieldLengthM > 0.f)) return 1.5f;
    return std::clamp(0.1f * layout->fieldLengthM, 1.f, 3.f);
}

bool cameraSeesTag(const sdk::CameraConfig& camera, const Pose3& cameraWorld, const Pose3& tagWorld, double tagSizeM) {
    const sdk::CameraIntrinsics& in = camera.intrinsics;
    const double fx = in.fx(), fy = in.fy();
    if (!(std::isfinite(fx) && fx > 0.0 && std::isfinite(fy) && fy > 0.0) || !(tagSizeM > 0.0)) return false;

    // Facing: the tag's outward normal against the direction from the tag to the camera.
    const Vec3 toCamera = cameraWorld.position - tagWorld.position;
    const float distance = sdk::length(toCamera);
    if (!(distance > 0.f)) return false;
    const double facing = static_cast<double>(sdk::dot(tagWorld.rotation.rotate(Vec3{1.f, 0.f, 0.f}), toCamera)) / static_cast<double>(distance);
    if (!(facing > 0.0) || facing < std::cos(camera.maxViewAngleRad)) return false;

    // The four corners (tag frame: +X is the normal, +Y the viewer's right, +Z up) in the image.
    const float h = static_cast<float>(0.5 * tagSizeM);
    const Vec3 corners[4] = {{0.f, -h, -h}, {0.f, h, -h}, {0.f, h, h}, {0.f, -h, h}};
    const Pose3 toCameraFrame = cameraWorld.inverse();
    std::array<std::array<double, 2>, 4> px{};
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec3 p = toCameraFrame.transformPoint(tagWorld.transformPoint(corners[i]));
        if (!(static_cast<double>(p.x) >= kMinCameraDepthM)) return false;
        px[i] = {in.cx() - fx * static_cast<double>(p.y) / static_cast<double>(p.x), in.cy() - fy * static_cast<double>(p.z) / static_cast<double>(p.x)};
        if (px[i][0] < 0.0 || px[i][0] > in.widthPx || px[i][1] < 0.0 || px[i][1] > in.heightPx) return false;
    }
    double shortestEdge = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < 4; ++i)
        shortestEdge = std::min(shortestEdge, std::hypot(px[i][0] - px[(i + 1) % 4][0], px[i][1] - px[(i + 1) % 4][1]));
    return shortestEdge >= camera.minTagPixels;
}

}  // namespace wpiemu::app::editor
