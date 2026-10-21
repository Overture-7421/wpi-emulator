#include <algorithm>

#include "model/config_ops.h"
#include "model/config_ops_internal.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/physics/wheel_layout.h"

namespace wpiemu::app::editor::ops {

using sdk::MechanismConfig;
using sdk::RobotConfig;
using sdk::SlotAddress;
using sdk::Vec3;

// ---- chassis / drivetrain ---------------------------------------------------------------------
void setCom(RobotConfig& c, float offsetX, float offsetY, float height, bool compensate, const EditorCatalog* catalog) {
    const float dx = offsetX - c.chassis.comOffsetM.x;
    const float dy = offsetY - c.chassis.comOffsetM.y;
    const float dz = height - c.chassis.comHeightM;
    c.chassis.comOffsetM.x = offsetX;
    c.chassis.comOffsetM.y = offsetY;
    c.chassis.comHeightM = height;
    if (!compensate) return;

    for (auto& w : c.drivetrain.spec.wheels) {
        w.positionM.x -= dx;   // hub height (z) is floor-referenced and does not move
        w.positionM.y -= dy;
    }
    for (auto& p : c.chassis.primitives) p.local.position -= Vec3{dx, dy, dz};
    for (auto& m : c.mechanisms)
        if (m.attachment.onChassis()) m.mount.position -= Vec3{dx, dy, dz};
    for (auto& z : c.zones)
        if (z.attachment.onChassis()) z.local.position -= Vec3{dx, dy, dz};
    for (auto& cam : c.cameras)
        if (cam.attachment.onChassis()) cam.local.position -= Vec3{dx, dy, dz};
    if (catalog != nullptr) {
        if (sdk::Value* store = volumeStore(c, *catalog, "")) {
            for (const auto& vol : volumesFor(c, *catalog, "")) {
                const sdk::Value* raw = store->find(vol.key);
                sdk::Box3 box;
                if (!sdk::fromValue(raw != nullptr ? *raw : vol.defaultValue, box)) continue;
                box.pose.position -= Vec3{dx, dy, dz};
                (*store)[vol.key] = sdk::toValue(box);
            }
        }
    }
}

void setDrivetrainKind(RobotConfig& c, const EditorCatalog& catalog, const std::string& kind) {
    c.drivetrain.spec.kind = kind;
    for (auto& w : c.drivetrain.spec.wheels) {
        if (kind == sdk::drivetrain_kind::kSwerve) {
            w.kind = sdk::WheelKind::Swerve;
            w.steered = true;
        } else {
            w.steered = false;
            if (w.kind == sdk::WheelKind::Swerve) w.kind = sdk::WheelKind::Standard;
            if (kind == sdk::drivetrain_kind::kMecanum) w.kind = sdk::WheelKind::Mecanum;
        }
    }
    auto& maps = c.drivetrain.mappings;
    const std::vector<sdk::SlotSpec> remaining = sdk::drivetrainSlotSpecs(catalog.findDrivetrain(kind), c.drivetrain.spec);
    maps.erase(std::remove_if(maps.begin(), maps.end(),
                              [&](const sdk::SlotMapping& m) {
                                  for (const auto& s : remaining)
                                      if (s.name == m.slot.slot) return false;
                                  return true;
                              }),
               maps.end());
}

sdk::WheelSpec* findWheel(RobotConfig& c, std::string_view name) {
    for (auto& w : c.drivetrain.spec.wheels)
        if (w.name == name) return &w;
    return nullptr;
}
const sdk::WheelSpec* findWheel(const RobotConfig& c, std::string_view name) {
    for (const auto& w : c.drivetrain.spec.wheels)
        if (w.name == name) return &w;
    return nullptr;
}

std::string uniqueWheelName(const RobotConfig& c, std::string base) {
    return detail::uniqueSuffixed(std::move(base), "wheel", "", [&](const std::string& n) { return findWheel(c, n) != nullptr; });
}

std::string addWheel(RobotConfig& c) {
    sdk::WheelSpec w = sdk::drivetrainWheel(c.drivetrain.spec.kind);
    w.name = uniqueWheelName(c, "wheel");
    w.positionM = Vec3{0.f, c.chassis.sizeM.y * 0.5f - 0.05f, w.radiusM};
    if (!c.drivetrain.spec.wheels.empty()) w.positionM.z = c.drivetrain.spec.wheels.front().positionM.z;
    c.drivetrain.spec.wheels.push_back(w);
    return w.name;
}

bool removeWheel(RobotConfig& c, const std::string& name) {
    auto& wheels = c.drivetrain.spec.wheels;
    const auto it = std::find_if(wheels.begin(), wheels.end(), [&](const sdk::WheelSpec& w) { return w.name == name; });
    if (it == wheels.end()) return false;
    wheels.erase(it);
    auto& maps = c.drivetrain.mappings;
    maps.erase(std::remove_if(maps.begin(), maps.end(),
                              [&](const sdk::SlotMapping& m) {
                                  return wheelOfSlot(m.slot.slot) == name;
                              }),
               maps.end());
    return true;
}

bool renameWheel(RobotConfig& c, const std::string& oldName, const std::string& newName) {
    if (newName.empty() || oldName == newName || findWheel(c, newName) != nullptr) return false;
    sdk::WheelSpec* w = findWheel(c, oldName);
    if (w == nullptr) return false;
    w->name = newName;
    for (auto& link : c.motionLinks) {
        if (link.source == SlotAddress{sdk::slot_owner::kDrivetrain, sdk::driveSlotName(oldName, 0)})
            link.source.slot = sdk::driveSlotName(newName, 0);
        if (link.target == SlotAddress{sdk::slot_owner::kDrivetrain, sdk::driveSlotName(oldName, 0)})
            link.target.slot = sdk::driveSlotName(newName, 0);
    }
    for (auto& m : c.drivetrain.mappings)
        if (wheelOfSlot(m.slot.slot) == oldName) m.slot.slot = newName + m.slot.slot.substr(oldName.size());
    return true;
}

// ---- primitives -------------------------------------------------------------------------------
std::vector<sdk::PrimitiveSpec>* primitivesOf(RobotConfig& c, std::string_view owner) {
    if (owner == kChassisOwner) return &c.chassis.primitives;
    if (MechanismConfig* m = c.findMechanism(owner)) return &m->geometry;
    return nullptr;
}
const std::vector<sdk::PrimitiveSpec>* primitivesOf(const RobotConfig& c, std::string_view owner) {
    if (owner == kChassisOwner) return &c.chassis.primitives;
    if (const MechanismConfig* m = c.findMechanism(owner)) return &m->geometry;
    return nullptr;
}

namespace {
std::string uniquePrimitiveName(const std::vector<sdk::PrimitiveSpec>& list, const std::string& base) {
    auto taken = [&](const std::string& n) {
        return std::any_of(list.begin(), list.end(), [&](const sdk::PrimitiveSpec& p) { return p.name == n; });
    };
    for (int n = 1;; ++n) {
        std::string candidate = base + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}
}  // namespace

int addPrimitive(RobotConfig& c, std::string_view owner, sdk::PrimitiveKind kind) {
    auto* list = primitivesOf(c, owner);
    if (list == nullptr) return -1;
    sdk::PrimitiveSpec p;
    p.kind = kind;
    p.name = uniquePrimitiveName(*list, kind == sdk::PrimitiveKind::Box ? "box" : "cylinder");
    if (owner == kChassisOwner) p.collides = false;
    list->push_back(p);
    return static_cast<int>(list->size()) - 1;
}

bool removePrimitive(RobotConfig& c, std::string_view owner, int index) {
    auto* list = primitivesOf(c, owner);
    if (list == nullptr || index < 0 || index >= static_cast<int>(list->size())) return false;
    list->erase(list->begin() + index);
    return true;
}

int duplicatePrimitive(RobotConfig& c, std::string_view owner, int index) {
    auto* list = primitivesOf(c, owner);
    if (list == nullptr || index < 0 || index >= static_cast<int>(list->size())) return -1;
    sdk::PrimitiveSpec copy = (*list)[static_cast<std::size_t>(index)];
    copy.name = uniquePrimitiveName(*list, copy.kind == sdk::PrimitiveKind::Box ? "box" : "cylinder");
    copy.local.position.x += 0.05f;
    list->insert(list->begin() + index + 1, copy);
    return index + 1;
}

}  // namespace wpiemu::app::editor::ops
