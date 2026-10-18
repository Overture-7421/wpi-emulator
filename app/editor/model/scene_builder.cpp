#include "model/scene_builder.h"

#include <algorithm>
#include <cmath>

#include "model/config_ops.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::editor {

using sdk::Pose3;
using sdk::Quat;
using sdk::Vec3;

namespace {

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
    return out;
}

}  // namespace wpiemu::app::editor
