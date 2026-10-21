#include "model/transform_access.h"

#include <algorithm>
#include <cmath>

#include "model/config_ops.h"
#include "wpiemu/sdk/config/value_codec.h"

namespace wpiemu::app::editor {

using sdk::Pose3;
using sdk::Vec3;

namespace {

constexpr float kMinSize = 0.005f;

Vec3 clampSize(Vec3 v) { return {std::max(v.x, kMinSize), std::max(v.y, kMinSize), std::max(v.z, kMinSize)}; }

bool changed(float a, float b) { return std::fabs(a - b) > 1e-6f; }

const ops::VolumeRef* findVolume(const std::vector<ops::VolumeRef>& list, const std::string& key) {
    for (const auto& v : list)
        if (v.key == key) return &v;
    return nullptr;
}

}  // namespace

std::optional<TransformTarget> readTransform(const sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel) {
    TransformTarget out;
    EditableTransform& t = out.transform;
    switch (sel.kind) {
        case SelKind::Chassis:
            out.parentWorld = ops::hullWorld(c.chassis);
            t.size = c.chassis.sizeM;
            t.sizeKind = SizeKind::Box;
            t.canMove = t.canRotate = false;
            t.canScale = true;
            return out;
        case SelKind::ChassisCom:
            t.local.position = Vec3{c.chassis.comOffsetM.x, c.chassis.comOffsetM.y, c.chassis.comHeightM};
            t.canRotate = false;
            return out;
        case SelKind::Wheel: {
            const sdk::WheelSpec* w = ops::findWheel(c, sel.sub);
            if (w == nullptr) return std::nullopt;
            out.parentWorld = Pose3{Vec3{c.chassis.comOffsetM.x, c.chassis.comOffsetM.y, 0.f}, sdk::Quat{}};
            t.local.position = w->positionM;
            t.size = Vec3{w->radiusM * 2.f, w->widthM, w->radiusM * 2.f};
            t.sizeKind = SizeKind::Wheel;
            t.canRotate = false;
            t.canScale = true;
            return out;
        }
        case SelKind::Primitive: {
            const auto* list = ops::primitivesOf(c, sel.owner);
            if (list == nullptr || sel.index < 0 || sel.index >= static_cast<int>(list->size())) return std::nullopt;
            const sdk::PrimitiveSpec& p = (*list)[static_cast<std::size_t>(sel.index)];
            out.parentWorld = sel.owner == ops::kChassisOwner ? ops::chassisFrameWorld(c.chassis) : ops::mechanismRootWorld(c, catalog, sel.owner);
            t.local = p.local;
            t.canScale = true;
            t.size = p.extentsM();
            t.sizeKind = p.kind == sdk::PrimitiveKind::Box ? SizeKind::Box : SizeKind::Cylinder;
            return out;
        }
        case SelKind::Mechanism: {
            const sdk::MechanismConfig* m = c.findMechanism(sel.owner);
            if (m == nullptr) return std::nullopt;
            out.parentWorld = ops::mechanismParentWorld(c, catalog, *m);
            t.local = m->mount;
            return out;
        }
        case SelKind::Zone: {
            const sdk::ZoneConfig* z = c.findZone(sel.owner);
            if (z == nullptr) return std::nullopt;
            const sdk::ZoneKindInfo* info = catalog.findZoneKind(z->kind);
            out.parentWorld = ops::zoneParentWorld(c, catalog, *z);
            t.local = z->local;
            if (info == nullptr || info->shape == sdk::ZoneShape::Volume) {
                t.size = z->sizeM;
                t.sizeKind = SizeKind::Box;
                t.canScale = true;
            }
            return out;
        }
        case SelKind::Camera: {
            const sdk::CameraConfig* cam = c.findCamera(sel.owner);
            if (cam == nullptr) return std::nullopt;
            out.parentWorld = ops::cameraParentWorld(c, catalog, *cam);
            t.local = cam->local;
            return out;
        }
        case SelKind::Overlay: {
            const auto volumes = ops::volumesFor(c, catalog, sel.owner);
            const ops::VolumeRef* vol = findVolume(volumes, sel.sub);
            const sdk::Value* store = ops::volumeStore(c, catalog, sel.owner);
            if (vol == nullptr || store == nullptr) return std::nullopt;
            sdk::Box3 box;
            const sdk::Value* raw = store->find(vol->key);
            if (!sdk::fromValue(raw != nullptr ? *raw : vol->defaultValue, box)) return std::nullopt;
            out.parentWorld = sel.owner.empty() ? ops::chassisFrameWorld(c.chassis) : ops::mechanismRootWorld(c, catalog, sel.owner);
            t.local = box.pose;
            t.size = box.sizeM;
            t.sizeKind = SizeKind::Box;
            t.canMove = t.canRotate = t.canScale = vol->editable;
            return out;
        }
        default: return std::nullopt;
    }
}

bool writeTransform(sdk::RobotConfig& c, const EditorCatalog& catalog, const SelectionRef& sel, const EditableTransform& t) {
    const auto current = readTransform(c, catalog, sel);
    if (!current) return false;
    const EditableTransform& old = current->transform;
    const Vec3 size = clampSize(t.size);

    switch (sel.kind) {
        case SelKind::Chassis:
            if (!old.canScale) return false;
            c.chassis.sizeM = size;
            return true;
        case SelKind::ChassisCom:
            ops::setCom(c, t.local.position.x, t.local.position.y, std::max(t.local.position.z, 0.02f), true, &catalog);
            return true;
        case SelKind::Wheel: {
            sdk::WheelSpec* w = ops::findWheel(c, sel.sub);
            if (w == nullptr) return false;
            w->positionM = t.local.position;
            if (changed(size.x, old.size.x)) w->radiusM = size.x * 0.5f;
            else if (changed(size.z, old.size.z)) w->radiusM = size.z * 0.5f;
            if (changed(size.y, old.size.y)) w->widthM = size.y;
            return true;
        }
        case SelKind::Primitive: {
            auto* list = ops::primitivesOf(c, sel.owner);
            if (list == nullptr || sel.index < 0 || sel.index >= static_cast<int>(list->size())) return false;
            sdk::PrimitiveSpec& p = (*list)[static_cast<std::size_t>(sel.index)];
            p.local = Pose3{t.local.position, t.local.rotation.normalized()};
            if (old.sizeKind == SizeKind::Box) {
                p.sizeM = size;
            } else {
                if (changed(size.x, old.size.x)) p.radiusM = size.x * 0.5f;
                else if (changed(size.y, old.size.y)) p.radiusM = size.y * 0.5f;
                if (changed(size.z, old.size.z)) p.heightM = size.z;
            }
            return true;
        }
        case SelKind::Mechanism: {
            for (auto& m : c.mechanisms)
                if (m.id == sel.owner) {
                    m.mount = Pose3{t.local.position, t.local.rotation.normalized()};
                    return true;
                }
            return false;
        }
        case SelKind::Zone: {
            sdk::ZoneConfig* z = c.findZone(sel.owner);
            if (z == nullptr) return false;
            z->local = Pose3{t.local.position, t.local.rotation.normalized()};
            if (old.canScale) z->sizeM = size;
            return true;
        }
        case SelKind::Camera: {
            sdk::CameraConfig* cam = c.findCamera(sel.owner);
            if (cam == nullptr) return false;
            cam->local = Pose3{t.local.position, t.local.rotation.normalized()};
            return true;
        }
        case SelKind::Overlay: {
            if (!old.canMove) return false;
            sdk::Value* store = ops::volumeStore(c, catalog, sel.owner);
            if (store == nullptr) return false;
            if (!store->isObject()) *store = sdk::Value::makeObject();
            sdk::Box3 box;
            box.pose = Pose3{t.local.position, t.local.rotation.normalized()};
            box.sizeM = size;
            (*store)[sel.sub] = sdk::toValue(box);
            return true;
        }
        default: return false;
    }
}

}  // namespace wpiemu::app::editor
