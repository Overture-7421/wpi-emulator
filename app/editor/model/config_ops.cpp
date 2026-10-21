#include "model/config_ops.h"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/strings.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"
#include "wpiemu/sdk/physics/wheel_layout.h"

namespace wpiemu::app::editor::ops {

using sdk::DeviceKey;
using sdk::MechanismConfig;
using sdk::Pose3;
using sdk::RobotConfig;
using sdk::SlotAddress;
using sdk::Vec3;

namespace {

bool reservedOwner(std::string_view id) {
    return id == sdk::kChassisParent || id == sdk::slot_owner::kDrivetrain || id == sdk::slot_owner::kRobot;
}

bool onChassis(const std::string& parent) { return parent.empty() || parent == sdk::kChassisParent; }

const char* const kVolumeColors[] = {"#4488ff", "#ff9944", "#44cc88", "#cc55bb"};

void addVolume(std::vector<VolumeRef>& out, const sdk::PropertySchema& schema,
               const std::vector<sdk::ViewportOverlayDescriptor>& overlays, std::string_view owner) {
    auto exists = [&](const std::string& key) {
        return std::any_of(out.begin(), out.end(), [&](const VolumeRef& v) { return v.key == key; });
    };
    for (const auto& ov : overlays) {
        const sdk::PropertySpec* spec = schema.find(ov.propertyKey);
        if (spec == nullptr || spec->type != sdk::PropertyType::Box3 || exists(ov.propertyKey)) continue;
        out.push_back({std::string(owner), ov.propertyKey, ov.label.empty() ? spec->label : ov.label, ov.color,
                       ov.editable, spec->defaultValue});
    }
    for (const auto& spec : schema.properties) {
        if (spec.type != sdk::PropertyType::Box3 || exists(spec.key)) continue;
        out.push_back({std::string(owner), spec.key, spec.label, kVolumeColors[out.size() % 4], true, spec.defaultValue});
    }
}

}  // namespace

// ---- naming -----------------------------------------------------------------------------------
std::string baseIdForType(std::string_view type) {
    const auto dot = type.rfind('.');
    std::string base(dot == std::string_view::npos ? type : type.substr(dot + 1));
    for (char& ch : base)
        if (!std::isalnum(static_cast<unsigned char>(ch))) ch = '_';
    return base.empty() ? std::string("mechanism") : base;
}

std::string uniqueMechanismId(const RobotConfig& c, std::string base) {
    if (base.empty()) base = "mechanism";
    auto taken = [&](const std::string& id) {
        return reservedOwner(id) || c.findMechanism(id) != nullptr || c.findZone(id) != nullptr || c.findCamera(id) != nullptr ||
               c.findSensor(id) != nullptr;
    };
    if (!taken(base)) return base;
    for (int n = 2;; ++n) {
        std::string candidate = base + "_" + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}

std::string uniqueWheelName(const RobotConfig& c, std::string base) {
    auto taken = [&](const std::string& n) { return findWheel(c, n) != nullptr; };
    if (!taken(base)) return base;
    for (int n = 2;; ++n) {
        std::string candidate = base + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}

std::string displayNameFor(const MechanismConfig& m, const EditorCatalog& catalog) {
    if (!m.label.empty()) return m.label;
    if (const auto* t = catalog.findTemplate(m.type)) return t->displayName + " (" + m.id + ")";
    return m.id;
}

// ---- frames -----------------------------------------------------------------------------------
Pose3 chassisFrameWorld(const sdk::ChassisSpec& s) {
    return Pose3{Vec3{s.comOffsetM.x, s.comOffsetM.y, s.comHeightM}, sdk::Quat{}};
}

Pose3 hullWorld(const sdk::ChassisSpec& s) {
    return Pose3{Vec3{0.f, 0.f, s.groundClearanceM + s.sizeM.z * 0.5f}, sdk::Quat{}};
}

Pose3 wheelWorld(const sdk::ChassisSpec& chassis, const sdk::WheelSpec& wheel) {
    return Pose3{Vec3{chassis.comOffsetM.x + wheel.positionM.x, chassis.comOffsetM.y + wheel.positionM.y, wheel.positionM.z},
                 sdk::Quat{}};
}

sdk::LinkPoseSource restLinkPoses(const RobotConfig& c, const EditorCatalog& catalog) {
    return [&c, &catalog](std::string_view id, std::string_view link) -> std::optional<Pose3> {
        if (link == sdk::kRootLink) return Pose3{};
        const MechanismConfig* m = c.findMechanism(id);
        if (m == nullptr) return std::nullopt;
        const sdk::MechanismTemplateInfo* info = catalog.findTemplate(m->type);
        if (info == nullptr) return std::nullopt;
        for (const auto& l : info->links)
            if (l.name == link) return l.local;
        return std::nullopt;
    };
}

std::optional<Pose3> linkInChassis(const RobotConfig& c, const EditorCatalog& catalog, std::string_view id,
                                   std::string_view link) {
    return sdk::resolveLinkPoseInChassis(c, id, link, restLinkPoses(c, catalog));
}

Pose3 mechanismRootWorld(const RobotConfig& c, const EditorCatalog& catalog, std::string_view id) {
    const Pose3 chassis = chassisFrameWorld(c.chassis);
    if (const auto p = linkInChassis(c, catalog, id)) return chassis * *p;
    if (const MechanismConfig* m = c.findMechanism(id)) return chassis * m->mount;
    return chassis;
}

Pose3 mechanismParentWorld(const RobotConfig& c, const EditorCatalog& catalog, const MechanismConfig& m) {
    const Pose3 chassis = chassisFrameWorld(c.chassis);
    const sdk::MechanismAttachment att = m.attachment;
    if (att.onChassis()) return chassis;
    if (const auto p = linkInChassis(c, catalog, att.parentMechanismId, att.parentLink)) return chassis * *p;
    return chassis;
}

// ---- mechanisms -------------------------------------------------------------------------------
MechanismConfig makeMechanism(const RobotConfig& c, const sdk::MechanismTemplateInfo& info,
                              const sdk::MechanismAttachment& attachment, const MechanismVariant& variant) {
    MechanismConfig m;
    std::string baseId = baseIdForType(info.type);
    std::string baseLabel = info.displayName;
    if (!variant.first.empty())
        for (const auto& p : info.properties.properties)
            if (p.key == variant.first)
                for (const auto& o : p.options)
                    if (o.value == variant.second) {
                        baseId = o.value;
                        baseLabel = o.label;
                    }
    m.id = uniqueMechanismId(c, baseId);
    m.type = info.type;
    m.label = baseLabel;
    for (const auto& other : c.mechanisms)
        if (other.id == baseId || other.label == baseLabel) { m.label = baseLabel + " " + std::to_string(c.mechanisms.size() + 1); break; }
    m.geometry = info.defaultGeometry;
    m.properties = sdk::defaultsOf(info.properties);
    if (!variant.first.empty()) m.properties[variant.first] = variant.second;
    m.attachTo(attachment.parentMechanismId, attachment.parentLink);
    if (m.attachment.onChassis()) {
        int siblings = 0;
        for (const auto& other : c.mechanisms)
            if (other.attachment.onChassis()) ++siblings;
        const float step = 0.15f;
        const float x = ((siblings % 2 == 0) ? 1.f : -1.f) * step * static_cast<float>((siblings + 1) / 2);
        const float hullTop = c.chassis.groundClearanceM + c.chassis.sizeM.z - c.chassis.comHeightM;
        m.mount.position = Vec3{x, 0.f, hullTop + 0.06f};
    }
    return m;
}

std::string addMechanism(RobotConfig& c, const sdk::MechanismTemplateInfo& info, const sdk::MechanismAttachment& attachment,
                         const MechanismVariant& variant) {
    MechanismConfig m = makeMechanism(c, info, attachment, variant);
    std::string id = m.id;
    c.mechanisms.push_back(std::move(m));
    normalizeOrder(c);
    return id;
}

std::vector<std::string> childrenOf(const RobotConfig& c, std::string_view id) {
    std::vector<std::string> out;
    for (const auto& m : c.mechanisms) {
        const sdk::MechanismAttachment a = m.attachment;
        if (!a.onChassis() && a.parentMechanismId == id && m.id != id) out.push_back(m.id);
    }
    return out;
}

std::vector<std::string> descendantsOf(const RobotConfig& c, std::string_view id) {
    std::vector<std::string> out;
    std::vector<std::string> frontier{std::string(id)};
    while (!frontier.empty()) {
        const std::string cur = frontier.back();
        frontier.pop_back();
        for (const auto& child : childrenOf(c, cur)) {
            if (std::find(out.begin(), out.end(), child) != out.end() || child == id) continue;
            out.push_back(child);
            frontier.push_back(child);
        }
    }
    return out;
}

bool canReparent(const RobotConfig& c, std::string_view id, std::string_view newParent) {
    if (c.findMechanism(id) == nullptr) return false;
    if (onChassis(std::string(newParent))) return true;
    if (c.findMechanism(newParent) == nullptr || newParent == id) return false;
    const auto below = descendantsOf(c, id);
    return std::find(below.begin(), below.end(), newParent) == below.end();
}

bool reparentMechanism(RobotConfig& c, const EditorCatalog& catalog, std::string_view id, const std::string& newParent,
                       const std::string& newLink, bool keepWorldPose) {
    if (!canReparent(c, id, newParent)) return false;
    MechanismConfig* m = c.findMechanism(id);
    const std::string link = newLink.empty() ? std::string(sdk::kRootLink) : newLink;
    if (onChassis(newParent)) {
        if (link != sdk::kRootLink) return false;
    } else if (const auto* info = catalog.findTemplate(c.findMechanism(newParent)->type);
               info != nullptr && !sdk::hasLink(*info, link)) {
        return false;
    }

    const std::optional<Pose3> before = linkInChassis(c, catalog, id);
    m->attachTo(newParent, link);
    if (keepWorldPose && before) {
        Pose3 parentLink;
        if (!onChassis(newParent))
            if (const auto p = linkInChassis(c, catalog, newParent, link)) parentLink = *p;
        m->mount = parentLink.inverse() * *before;
    }
    normalizeOrder(c);
    return true;
}

bool removeMechanism(RobotConfig& c, const EditorCatalog& catalog, std::string_view id) {
    MechanismConfig* m = c.findMechanism(id);
    if (m == nullptr) return false;
    const sdk::MechanismAttachment att = m->attachment;
    for (const auto& child : childrenOf(c, id)) {
        if (!reparentMechanism(c, catalog, child, att.parentMechanismId, att.parentLink)) {
            if (MechanismConfig* cm = c.findMechanism(child)) cm->attachTo(sdk::kChassisParent);
        }
    }
    // Zones and cameras hang on the removed mechanism's parent now, in the same place.
    for (auto& z : c.zones)
        if (z.attachment.parentMechanismId == id && !z.attachment.onChassis()) {
            const std::string zoneId = z.id;
            if (!reattachZone(c, catalog, zoneId, att.parentMechanismId, att.parentLink))
                if (sdk::ZoneConfig* zc = c.findZone(zoneId)) zc->attachment = sdk::MechanismAttachment{};
        }
    for (auto& cam : c.cameras)
        if (cam.attachment.parentMechanismId == id && !cam.attachment.onChassis()) {
            const std::string cameraId = cam.id;
            if (!reattachCamera(c, catalog, cameraId, att.parentMechanismId, att.parentLink))
                if (sdk::CameraConfig* cc = c.findCamera(cameraId)) cc->attachment = sdk::MechanismAttachment{};
        }
    const std::string removed(id);
    c.mechanisms.erase(std::remove_if(c.mechanisms.begin(), c.mechanisms.end(),
                                      [&](const MechanismConfig& x) { return x.id == removed; }),
                       c.mechanisms.end());
    return true;
}

std::string duplicateMechanism(RobotConfig& c, std::string_view id) {
    if (c.findMechanism(id) == nullptr) return {};
    std::vector<std::string> subtree{std::string(id)};
    for (auto& d : descendantsOf(c, id)) subtree.push_back(std::move(d));

    std::vector<std::pair<std::string, std::string>> renamed;   // old -> new
    std::vector<MechanismConfig> copies;
    auto taken = [&](const std::string& candidate) {
        if (reservedOwner(candidate) || c.findMechanism(candidate) != nullptr || c.findZone(candidate) != nullptr ||
            c.findCamera(candidate) != nullptr || c.findSensor(candidate) != nullptr)
            return true;
        return std::any_of(renamed.begin(), renamed.end(), [&](const auto& r) { return r.second == candidate; });
    };
    for (const auto& oldId : subtree) {
        MechanismConfig copy = *c.findMechanism(oldId);
        std::string base = oldId;
        std::string fresh = base;
        for (int n = 2; taken(fresh); ++n) fresh = base + "_" + std::to_string(n);
        copy.id = fresh;
        renamed.emplace_back(oldId, fresh);
        copy.mappings.clear();
        copies.push_back(std::move(copy));
    }
    auto mapped = [&](const std::string& old) -> const std::string* {
        for (const auto& [o, n] : renamed)
            if (o == old) return &n;
        return nullptr;
    };
    for (std::size_t i = 0; i < copies.size(); ++i) {
        MechanismConfig& copy = copies[i];
        const sdk::MechanismAttachment att = copy.attachment;
        if (i == 0) {
            copy.label = (copy.label.empty() ? subtree[0] : copy.label) + " (copy)";
            copy.mount.position.y += 0.1f;
        } else if (const std::string* np = mapped(att.parentMechanismId)) {
            copy.attachTo(*np, att.parentLink);
        }
    }
    std::string rootId = copies.front().id;
    for (auto& copy : copies) c.mechanisms.push_back(std::move(copy));
    const auto originalLinks = c.motionLinks;
    for (const auto& link : originalLinks)
        if (const auto* target = mapped(link.target.owner)) {
            auto copy = link;
            copy.target.owner = *target;
            if (const auto* source = mapped(link.source.owner)) copy.source.owner = *source;
            copy.id += "_copy";
            while (std::any_of(c.motionLinks.begin(), c.motionLinks.end(),
                               [&](const auto& existing) { return existing.id == copy.id; }))
                copy.id += "_2";
            c.motionLinks.push_back(std::move(copy));
        }
    normalizeOrder(c);
    return rootId;
}

void normalizeOrder(RobotConfig& c) {
    std::vector<MechanismConfig> pending = std::move(c.mechanisms);
    std::vector<MechanismConfig> ordered;
    ordered.reserve(pending.size());
    auto emitted = [&](const std::string& id) {
        return std::any_of(ordered.begin(), ordered.end(), [&](const MechanismConfig& m) { return m.id == id; });
    };
    bool progress = true;
    while (!pending.empty() && progress) {
        progress = false;
        for (auto it = pending.begin(); it != pending.end();) {
            const sdk::MechanismAttachment a = it->attachment;
            if (a.onChassis() || emitted(a.parentMechanismId)) {
                ordered.push_back(std::move(*it));
                it = pending.erase(it);
                progress = true;
            } else {
                ++it;
            }
        }
    }
    for (auto& m : pending) ordered.push_back(std::move(m));   // cycles / missing parents: validator reports them
    c.mechanisms = std::move(ordered);
}

// ---- zones ------------------------------------------------------------------------------------
std::string uniqueZoneId(const RobotConfig& c, std::string base) {
    if (base.empty()) base = "zone";
    auto taken = [&](const std::string& id) {
        return reservedOwner(id) || c.findZone(id) != nullptr || c.findMechanism(id) != nullptr || c.findCamera(id) != nullptr ||
               c.findSensor(id) != nullptr;
    };
    if (!taken(base)) return base;
    for (int n = 2;; ++n) {
        std::string candidate = base + "_" + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}

std::string zoneDisplayName(const sdk::ZoneConfig& z, const EditorCatalog& catalog) {
    if (!z.label.empty()) return z.label;
    if (const auto* k = catalog.findZoneKind(z.kind)) return k->displayName;
    return z.id;
}

Pose3 zoneParentWorld(const RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneConfig& z) {
    const Pose3 chassis = chassisFrameWorld(c.chassis);
    if (z.attachment.onChassis()) return chassis;
    if (const auto p = linkInChassis(c, catalog, z.attachment.parentMechanismId, z.attachment.parentLink)) return chassis * *p;
    return chassis;
}

Pose3 zoneWorld(const RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneConfig& z) {
    return zoneParentWorld(c, catalog, z) * z.local;
}

sdk::ZoneConfig makeZone(const RobotConfig& c, const sdk::ZoneKindInfo& info, const sdk::MechanismAttachment& attachment) {
    sdk::ZoneConfig z;
    z.id = uniqueZoneId(c, baseIdForType(info.kind));
    z.kind = info.kind;
    z.label = info.displayName;
    for (const auto& other : c.zones)
        if (other.kind == info.kind && other.label == z.label) { z.label = info.displayName + " " + std::to_string(c.zones.size() + 1); break; }
    z.attachment.parentMechanismId = attachment.onChassis() ? std::string(sdk::kChassisParent) : attachment.parentMechanismId;
    z.attachment.parentLink = attachment.parentLink.empty() ? std::string(sdk::kRootLink) : attachment.parentLink;
    z.local = info.defaultLocal;
    z.sizeM = info.defaultSizeM;
    z.properties = sdk::defaultsOf(info.properties);
    return z;
}

std::string addZone(RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneKindInfo& info, const sdk::MechanismAttachment& attachment) {
    sdk::MechanismAttachment att = attachment;
    if (!att.onChassis()) {
        const MechanismConfig* parent = c.findMechanism(att.parentMechanismId);
        const sdk::MechanismTemplateInfo* tpl = parent != nullptr ? catalog.findTemplate(parent->type) : nullptr;
        const std::string link = att.parentLink.empty() ? std::string(sdk::kRootLink) : att.parentLink;
        if (parent == nullptr || (tpl != nullptr && !sdk::hasLink(*tpl, link))) att = sdk::MechanismAttachment{};
    } else {
        att = sdk::MechanismAttachment{};
    }
    sdk::ZoneConfig z = makeZone(c, info, att);
    std::string id = z.id;
    c.zones.push_back(std::move(z));
    return id;
}

bool removeZone(RobotConfig& c, std::string_view id) {
    const auto it = std::find_if(c.zones.begin(), c.zones.end(), [&](const sdk::ZoneConfig& z) { return z.id == id; });
    if (it == c.zones.end()) return false;
    c.zones.erase(it);
    return true;
}

std::string duplicateZone(RobotConfig& c, std::string_view id) {
    const sdk::ZoneConfig* original = c.findZone(id);
    if (original == nullptr) return {};
    sdk::ZoneConfig copy = *original;
    copy.id = uniqueZoneId(c, copy.id);
    copy.label = (copy.label.empty() ? original->id : copy.label) + " (copy)";
    copy.local.position.y += 0.1f;
    std::string newId = copy.id;
    c.zones.push_back(std::move(copy));
    return newId;
}

bool reattachZone(RobotConfig& c, const EditorCatalog& catalog, std::string_view id, const std::string& newParent,
                  const std::string& newLink, bool keepWorldPose) {
    sdk::ZoneConfig* z = c.findZone(id);
    if (z == nullptr) return false;
    const std::string link = newLink.empty() ? std::string(sdk::kRootLink) : newLink;
    if (onChassis(newParent)) {
        if (link != sdk::kRootLink) return false;
    } else {
        const MechanismConfig* parent = c.findMechanism(newParent);
        if (parent == nullptr) return false;
        if (const auto* info = catalog.findTemplate(parent->type); info != nullptr && !sdk::hasLink(*info, link)) return false;
    }
    const Pose3 before = zoneWorld(c, catalog, *z);
    z->attachment.parentMechanismId = onChassis(newParent) ? std::string(sdk::kChassisParent) : newParent;
    z->attachment.parentLink = link;
    if (keepWorldPose) z->local = zoneParentWorld(c, catalog, *z).inverse() * before;
    return true;
}

int addZoneCondition(RobotConfig& c, std::string_view zoneId) {
    sdk::ZoneConfig* z = c.findZone(zoneId);
    if (z == nullptr) return -1;
    sdk::ZoneCondition cond;
    if (!c.mechanisms.empty()) cond.mechanismId = c.mechanisms.front().id;
    z->activation.conditions.push_back(std::move(cond));
    return static_cast<int>(z->activation.conditions.size()) - 1;
}

bool removeZoneCondition(RobotConfig& c, std::string_view zoneId, int index) {
    sdk::ZoneConfig* z = c.findZone(zoneId);
    if (z == nullptr || index < 0 || index >= static_cast<int>(z->activation.conditions.size())) return false;
    z->activation.conditions.erase(z->activation.conditions.begin() + index);
    return true;
}

bool setZoneCondition(RobotConfig& c, std::string_view zoneId, int index, const sdk::ZoneCondition& condition) {
    sdk::ZoneConfig* z = c.findZone(zoneId);
    if (z == nullptr || index < 0 || index >= static_cast<int>(z->activation.conditions.size())) return false;
    sdk::ZoneCondition& slot = z->activation.conditions[static_cast<std::size_t>(index)];
    if (slot == condition) return false;
    slot = condition;
    return true;
}

bool setZoneActivationMode(RobotConfig& c, std::string_view zoneId, sdk::ActivationMode mode) {
    sdk::ZoneConfig* z = c.findZone(zoneId);
    if (z == nullptr || z->activation.mode == mode) return false;
    z->activation.mode = mode;
    return true;
}

// ---- cameras ----------------------------------------------------------------------------------
namespace {

const char* cameraKindName(sdk::CameraKind kind) { return kind == sdk::CameraKind::PhotonVision ? "PhotonVision" : "Limelight"; }

const char* cameraBaseId(sdk::CameraKind kind) { return kind == sdk::CameraKind::PhotonVision ? "photon" : "limelight"; }

bool nameTaken(const RobotConfig& c, sdk::CameraKind kind, const std::string& name, std::string_view exceptId) {
    return std::any_of(c.cameras.begin(), c.cameras.end(), [&](const sdk::CameraConfig& other) {
        return other.kind == kind && other.name == name && other.id != exceptId;
    });
}

}  // namespace

std::string uniqueCameraId(const RobotConfig& c, std::string base) {
    if (base.empty()) base = "camera";
    auto taken = [&](const std::string& id) {
        return reservedOwner(id) || c.findCamera(id) != nullptr || c.findZone(id) != nullptr || c.findMechanism(id) != nullptr ||
               c.findSensor(id) != nullptr;
    };
    if (!taken(base)) return base;
    for (int n = 2;; ++n) {
        std::string candidate = base + "_" + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}

std::string cameraDisplayName(const sdk::CameraConfig& cam) {
    if (!cam.label.empty()) return cam.label;
    return cameraKindName(cam.kind) + std::string(" (") + cam.id + ")";
}

Pose3 cameraParentWorld(const RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    const Pose3 chassis = chassisFrameWorld(c.chassis);
    if (cam.attachment.onChassis()) return chassis;
    if (const auto p = linkInChassis(c, catalog, cam.attachment.parentMechanismId, cam.attachment.parentLink)) return chassis * *p;
    return chassis;
}

Pose3 cameraWorld(const RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    return cameraParentWorld(c, catalog, cam) * cam.local;
}

Pose3 cameraRobotPose(const RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    return cameraWorld(c, catalog, cam);
}

std::string uniqueCameraName(const RobotConfig& c, sdk::CameraKind kind, std::string base, std::string_view exceptId) {    if (!nameTaken(c, kind, base, exceptId)) return base;
    const std::string stem = base.empty() ? std::string("cam") : base;
    for (int n = 2;; ++n) {
        std::string candidate = stem + std::to_string(n);
        if (!nameTaken(c, kind, candidate, exceptId)) return candidate;
    }
}

sdk::CameraConfig makeCamera(const RobotConfig& c, sdk::CameraKind kind, const sdk::MechanismAttachment& attachment) {
    sdk::CameraConfig cam;
    cam.kind = kind;
    cam.id = uniqueCameraId(c, cameraBaseId(kind));
    cam.label = cameraKindName(kind);
    int sameKind = 0;
    for (const auto& other : c.cameras) sameKind += other.kind == kind ? 1 : 0;
    if (sameKind > 0) cam.label += " " + std::to_string(sameKind + 1);
    cam.name = uniqueCameraName(c, kind, kind == sdk::CameraKind::PhotonVision ? "photon" : "");
    cam.attachment.parentMechanismId = attachment.onChassis() ? std::string(sdk::kChassisParent) : attachment.parentMechanismId;
    cam.attachment.parentLink = attachment.parentLink.empty() ? std::string(sdk::kRootLink) : attachment.parentLink;
    if (cam.attachment.onChassis()) {
        // Just behind the front face, high on the hull, looking forward (chassis frame = origin at the COM).
        const sdk::ChassisSpec& s = c.chassis;
        const Vec3 hullTopFront{s.sizeM.x * 0.5f - 0.05f, 0.f, s.groundClearanceM + s.sizeM.z * 0.8f};
        cam.local.position = hullTopFront - Vec3{s.comOffsetM.x, s.comOffsetM.y, s.comHeightM};
    }
    return cam;
}

std::string addCamera(RobotConfig& c, const EditorCatalog& catalog, sdk::CameraKind kind, const sdk::MechanismAttachment& attachment) {
    sdk::MechanismAttachment att = attachment;
    if (!att.onChassis()) {
        const MechanismConfig* parent = c.findMechanism(att.parentMechanismId);
        const sdk::MechanismTemplateInfo* tpl = parent != nullptr ? catalog.findTemplate(parent->type) : nullptr;
        const std::string link = att.parentLink.empty() ? std::string(sdk::kRootLink) : att.parentLink;
        if (parent == nullptr || (tpl != nullptr && !sdk::hasLink(*tpl, link))) att = sdk::MechanismAttachment{};
    } else {
        att = sdk::MechanismAttachment{};
    }
    sdk::CameraConfig cam = makeCamera(c, kind, att);
    std::string id = cam.id;
    c.cameras.push_back(std::move(cam));
    return id;
}

bool removeCamera(RobotConfig& c, std::string_view id) {
    const auto it = std::find_if(c.cameras.begin(), c.cameras.end(), [&](const sdk::CameraConfig& cam) { return cam.id == id; });
    if (it == c.cameras.end()) return false;
    c.cameras.erase(it);
    return true;
}

std::string duplicateCamera(RobotConfig& c, std::string_view id) {
    const sdk::CameraConfig* original = c.findCamera(id);
    if (original == nullptr) return {};
    sdk::CameraConfig copy = *original;
    copy.id = uniqueCameraId(c, copy.id);
    copy.label = (copy.label.empty() ? original->id : copy.label) + " (copy)";
    copy.name = uniqueCameraName(c, copy.kind, copy.name);
    copy.local.position.y += 0.1f;
    std::string newId = copy.id;
    c.cameras.push_back(std::move(copy));
    return newId;
}

bool reattachCamera(RobotConfig& c, const EditorCatalog& catalog, std::string_view id, const std::string& newParent,
                    const std::string& newLink, bool keepWorldPose) {
    sdk::CameraConfig* cam = c.findCamera(id);
    if (cam == nullptr) return false;
    const std::string link = newLink.empty() ? std::string(sdk::kRootLink) : newLink;
    if (onChassis(newParent)) {
        if (link != sdk::kRootLink) return false;
    } else {
        const MechanismConfig* parent = c.findMechanism(newParent);
        if (parent == nullptr) return false;
        if (const auto* info = catalog.findTemplate(parent->type); info != nullptr && !sdk::hasLink(*info, link)) return false;
    }
    const Pose3 before = cameraWorld(c, catalog, *cam);
    cam->attachment.parentMechanismId = onChassis(newParent) ? std::string(sdk::kChassisParent) : newParent;
    cam->attachment.parentLink = link;
    if (keepWorldPose) cam->local = cameraParentWorld(c, catalog, *cam).inverse() * before;
    return true;
}

// ---- sensors ----------------------------------------------------------------------------------
std::string uniqueSensorId(const RobotConfig& c, std::string base) {
    if (base.empty()) base = "sensor";
    auto taken = [&](const std::string& id) {
        return reservedOwner(id) || c.findSensor(id) != nullptr || c.findMechanism(id) != nullptr || c.findZone(id) != nullptr ||
               c.findCamera(id) != nullptr;
    };
    if (!taken(base)) return base;
    for (int n = 2;; ++n) {
        std::string candidate = base + "_" + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}

std::string sensorDisplayName(const sdk::SensorConfig& s) {
    if (!s.label.empty()) return s.label;
    return sdk::sensorKindInfo(s.kind).displayName + std::string(" (") + s.id + ")";
}

std::string addSensor(RobotConfig& c, sdk::SensorKind kind) {
    sdk::SensorConfig s;
    s.kind = kind;
    s.id = uniqueSensorId(c, sdk::sensorKindKey(kind));
    s.label = sdk::sensorKindInfo(kind).displayName;
    s.properties = sdk::Value::makeObject();
    for (const auto& p : sdk::sensorResponseSchema(kind).properties) s.properties[p.key] = p.defaultValue;
    std::string id = s.id;
    c.sensors.push_back(std::move(s));
    return id;
}

bool removeSensor(RobotConfig& c, std::string_view id) {
    const auto it = std::find_if(c.sensors.begin(), c.sensors.end(), [&](const sdk::SensorConfig& s) { return s.id == id; });
    if (it == c.sensors.end()) return false;
    c.sensors.erase(it);
    return true;
}

std::string duplicateSensor(RobotConfig& c, std::string_view id) {
    const sdk::SensorConfig* original = c.findSensor(id);
    if (original == nullptr) return {};
    sdk::SensorConfig copy = *original;
    copy.id = uniqueSensorId(c, copy.id);
    copy.label = (copy.label.empty() ? original->id : copy.label) + " (copy)";
    copy.mappings.clear();   // a device drives one thing
    std::string newId = copy.id;
    c.sensors.push_back(std::move(copy));
    return newId;
}

sdk::SensorConfig* findSensor(RobotConfig& c, std::string_view id) {
    for (auto& s : c.sensors)
        if (s.id == id) return &s;
    return nullptr;
}

// ---- start positions --------------------------------------------------------------------------
std::string uniqueStartPositionId(const RobotConfig& c, std::string base) {
    if (base.empty()) base = "start";
    if (c.findStartPosition(base) == nullptr) return base;
    for (int n = 2;; ++n) {
        std::string candidate = base + "_" + std::to_string(n);
        if (c.findStartPosition(candidate) == nullptr) return candidate;
    }
}

std::string startPositionDisplayName(const sdk::StartPosition& p) { return p.label.empty() ? p.id : p.label; }

std::string addStartPosition(RobotConfig& c) {
    sdk::StartPosition p;
    p.id = uniqueStartPositionId(c, "start");
    p.label = "Start " + std::to_string(c.startPositions.size() + 1);
    p.pose = sdk::RobotStartPose{2.f, 4.f, 0.f};
    std::string id = p.id;
    c.startPositions.push_back(std::move(p));
    return id;
}

bool removeStartPosition(RobotConfig& c, std::string_view id) {
    const auto it = std::find_if(c.startPositions.begin(), c.startPositions.end(), [&](const sdk::StartPosition& p) { return p.id == id; });
    if (it == c.startPositions.end()) return false;
    c.startPositions.erase(it);
    return true;
}

std::string duplicateStartPosition(RobotConfig& c, std::string_view id) {
    const sdk::StartPosition* original = c.findStartPosition(id);
    if (original == nullptr) return {};
    sdk::StartPosition copy = *original;
    copy.id = uniqueStartPositionId(c, copy.id);
    copy.label = startPositionDisplayName(*original) + " (copy)";
    std::string newId = copy.id;
    c.startPositions.push_back(std::move(copy));
    return newId;
}

namespace {

// "Blue left" -> "Red left", "BLUE" -> "RED", "blue" -> "red"; nothing to swap: nullopt.
std::optional<std::string> swapAllianceWord(const std::string& label, sdk::Alliance from, sdk::Alliance to) {
    const std::string needle(sdk::allianceKey(from));
    const std::size_t at = sdk::asciiLower(label).find(needle);
    if (at == std::string::npos) return std::nullopt;
    std::string word(sdk::allianceKey(to));
    const std::string found = label.substr(at, needle.size());
    if (std::all_of(found.begin(), found.end(), [](unsigned char ch) { return std::isupper(ch) != 0; })) {
        std::transform(word.begin(), word.end(), word.begin(), [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    } else if (std::isupper(static_cast<unsigned char>(found.front())) != 0) {
        word[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(word[0])));
    }
    std::string out = label;
    out.replace(at, needle.size(), word);
    return out;
}

}  // namespace

std::string copyStartPositionToOtherAlliance(RobotConfig& c, std::string_view id, const sdk::FieldFrame& frame) {
    const sdk::StartPosition* original = c.findStartPosition(id);
    if (original == nullptr) return {};
    const auto flipped = sdk::flipToOtherAlliance(original->pose, frame);
    if (!flipped) return {};
    sdk::StartPosition copy;
    copy.alliance = sdk::otherAlliance(original->alliance);
    copy.pose = *flipped;
    const std::string base = startPositionDisplayName(*original);
    copy.label = swapAllianceWord(base, original->alliance, copy.alliance)
                     .value_or(base + (copy.alliance == sdk::Alliance::Red ? " (Red)" : " (Blue)"));
    copy.id = uniqueStartPositionId(c, original->id + (copy.alliance == sdk::Alliance::Red ? "_red" : "_blue"));
    std::string newId = copy.id;
    c.startPositions.push_back(std::move(copy));
    return newId;
}

sdk::StartPosition* findStartPosition(RobotConfig& c, std::string_view id) {
    for (auto& p : c.startPositions)
        if (p.id == id) return &p;
    return nullptr;
}

// ---- device mappings --------------------------------------------------------------------------
std::vector<sdk::SlotMapping>* mappingList(RobotConfig& c, std::string_view owner) {
    if (owner == sdk::slot_owner::kRobot) return &c.robotMappings;
    if (owner == sdk::slot_owner::kDrivetrain) return &c.drivetrain.mappings;
    if (MechanismConfig* m = c.findMechanism(owner)) return &m->mappings;
    if (sdk::SensorConfig* s = findSensor(c, owner)) return &s->mappings;
    return nullptr;
}

const sdk::SlotMapping* findMapping(const RobotConfig& c, const SlotAddress& slot) {
    const auto& list = [&]() -> const std::vector<sdk::SlotMapping>& {
        static const std::vector<sdk::SlotMapping> empty;
        if (slot.owner == sdk::slot_owner::kRobot) return c.robotMappings;
        if (slot.owner == sdk::slot_owner::kDrivetrain) return c.drivetrain.mappings;
        if (const MechanismConfig* m = c.findMechanism(slot.owner)) return m->mappings;
        if (const sdk::SensorConfig* s = c.findSensor(slot.owner)) return s->mappings;
        return empty;
    }();
    for (const auto& m : list)
        if (m.slot == slot) return &m;
    return nullptr;
}

DeviceKey declaredKeyFor(const RobotConfig& c, const DeviceKey& proposed) {
    for (const auto& m : c.allMappings())
        if (m.device.physical() == proposed.physical()) return m.device;
    return proposed;
}

bool setMapping(RobotConfig& c, const SlotAddress& slot, const DeviceKey& device) {
    auto* list = mappingList(c, slot.owner);
    if (list == nullptr) return false;
    for (auto& m : *list)
        if (m.slot == slot) {
            m.device = device;
            return true;
        }
    list->push_back({slot, device});
    return true;
}

bool setMappingReversed(RobotConfig& c, const SlotAddress& slot, bool reversed) {
    auto* list = mappingList(c, slot.owner);
    if (list == nullptr) return false;
    for (auto& m : *list)
        if (m.slot == slot) {
            m.reversed = reversed;
            return true;
        }
    return false;
}

bool clearMapping(RobotConfig& c, const SlotAddress& slot) {
    auto* list = mappingList(c, slot.owner);
    if (list == nullptr) return false;
    const auto before = list->size();
    list->erase(std::remove_if(list->begin(), list->end(), [&](const sdk::SlotMapping& m) { return m.slot == slot; }),
                list->end());
    return list->size() != before;
}

std::vector<SlotAddress> slotsUsing(const RobotConfig& c, const sdk::PhysicalDeviceId& device) {
    std::vector<SlotAddress> out;
    for (const auto& m : c.allMappings())
        if (m.device.physical() == device) out.push_back(m.slot);
    return out;
}

// ---- slots ------------------------------------------------------------------------------------
namespace {
constexpr std::string_view kWheelCountSuffix = ".driveMotorCount";

// Current and maximum motor count behind `countKey`; false when the key is unknown for the owner.
bool motorCountOf(const RobotConfig& c, const EditorCatalog& catalog, std::string_view owner, std::string_view countKey, int& count, int& maxCount) {
    if (owner == sdk::slot_owner::kDrivetrain) {
        if (countKey.size() <= kWheelCountSuffix.size() || countKey.substr(countKey.size() - kWheelCountSuffix.size()) != kWheelCountSuffix) return false;
        const sdk::WheelSpec* w = findWheel(c, countKey.substr(0, countKey.size() - kWheelCountSuffix.size()));
        if (w == nullptr) return false;
        maxCount = sdk::kMaxMotorsPerAxis;
        count = std::clamp(w->driveMotorCount, 1, maxCount);
        return true;
    }
    const MechanismConfig* m = c.findMechanism(owner);
    const auto* info = m != nullptr ? catalog.findTemplate(m->type) : nullptr;
    if (info == nullptr) return false;
    maxCount = 0;
    for (const auto& spec : info->slotSpecs)
        if (spec.countKey == countKey) maxCount = std::max(maxCount, spec.ordinal);
    if (maxCount == 0) return false;
    count = 1;
    if (const sdk::PropertySpec* p = info->properties.find(std::string(countKey))) count = static_cast<int>(p->defaultValue.asInt(1));
    if (const sdk::Value* v = m->properties.find(std::string(countKey))) count = static_cast<int>(v->asInt(count));
    count = std::clamp(count, 1, maxCount);
    return true;
}
}  // namespace

std::string wheelOfSlot(std::string_view slotName) {
    const auto dot = slotName.rfind('.');
    if (dot == std::string_view::npos || dot == 0) return {};
    std::string_view tail = slotName.substr(dot + 1);
    if (tail == sdk::kSteerSlot || tail == sdk::kSteerEncoderSlot) return std::string(slotName.substr(0, dot));
    if (tail.substr(0, 5) != "drive") return {};
    tail.remove_prefix(5);
    for (const char ch : tail)
        if (ch < '0' || ch > '9') return {};
    return std::string(slotName.substr(0, dot));
}

std::vector<MotorGroup> motorGroups(const RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    std::vector<MotorGroup> out;
    for (const auto& spec : slotSpecsFor(c, catalog, owner)) {
        if (spec.type != sdk::SlotType::Motor || spec.countKey.empty()) continue;
        auto it = std::find_if(out.begin(), out.end(), [&](const MotorGroup& g) { return g.countKey == spec.countKey; });
        if (it == out.end()) {
            MotorGroup g;
            g.countKey = spec.countKey;
            if (!motorCountOf(c, catalog, owner, g.countKey, g.count, g.maxCount)) continue;
            out.push_back(std::move(g));
            it = std::prev(out.end());
        }
        it->slotNames.push_back(spec.name);
    }
    return out;
}

void pruneInactiveMappings(RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    auto* list = mappingList(c, owner);
    if (list == nullptr) return;
    std::vector<std::string> active;
    for (const auto& spec : slotSpecsFor(c, catalog, owner)) active.push_back(spec.name);
    list->erase(std::remove_if(list->begin(), list->end(),
                               [&](const sdk::SlotMapping& m) {
                                   if (std::find(active.begin(), active.end(), m.slot.slot) != active.end()) return false;
                                   // Only motor slots that exist for a higher motor number are pruned; unknown slots are left for the validator.
                                   if (owner == sdk::slot_owner::kDrivetrain) {
                                       const std::string wheel = wheelOfSlot(m.slot.slot);
                                       return !wheel.empty() && findWheel(c, wheel) != nullptr &&
                                              m.slot.slot != sdk::steerSlotName(wheel) &&
                                              m.slot.slot != sdk::steerEncoderSlotName(wheel);
                                   }
                                   const MechanismConfig* mech = c.findMechanism(owner);
                                   const auto* info = mech != nullptr ? catalog.findTemplate(mech->type) : nullptr;
                                   if (info == nullptr) return false;
                                   return std::any_of(info->slotSpecs.begin(), info->slotSpecs.end(),
                                                      [&](const sdk::SlotSpec& s) { return s.name == m.slot.slot && !s.countKey.empty(); });
                               }),
                list->end());
}

bool setMotorCount(RobotConfig& c, const EditorCatalog& catalog, std::string_view owner, const std::string& countKey, int count) {
    int current = 0, maxCount = 0;
    if (!motorCountOf(c, catalog, owner, countKey, current, maxCount)) return false;
    count = std::clamp(count, 1, maxCount);
    if (count == current) return false;
    if (owner == sdk::slot_owner::kDrivetrain) {
        findWheel(c, countKey.substr(0, countKey.size() - kWheelCountSuffix.size()))->driveMotorCount = count;
    } else {
        MechanismConfig* m = c.findMechanism(owner);
        if (!m->properties.isObject()) m->properties = sdk::Value::makeObject();
        m->properties[countKey] = count;
    }
    pruneInactiveMappings(c, catalog, owner);
    return true;
}

std::vector<sdk::SlotSpec> slotSpecsFor(const RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    std::vector<sdk::SlotSpec> out;
    if (owner == sdk::slot_owner::kRobot)
        out = sdk::robotSlotSpecs();
    else if (owner == sdk::slot_owner::kDrivetrain)
        out = sdk::drivetrainSlotSpecs(catalog.findDrivetrain(c.drivetrain.spec.kind), c.drivetrain.spec);
    else if (const sdk::SensorConfig* s = c.findSensor(owner))
        out.push_back(sdk::sensorSlotSpec(s->kind));
    else if (const MechanismConfig* m = c.findMechanism(owner))
        if (const auto* info = catalog.findTemplate(m->type)) {
            for (const auto& spec : info->slotSpecs) {
                int count = 0, maxCount = 0;
                if (!spec.countKey.empty() && motorCountOf(c, catalog, owner, spec.countKey, count, maxCount) && spec.ordinal > count) continue;
                out.push_back(spec);
            }
        }
    const auto axes =
        sdk::motionAxisInfos(c, [&](std::string_view type) { return catalog.findTemplate(type); });
    for (auto& slot : out)
        if (sdk::motionFollowerMotor(c, axes, {std::string(owner), slot.name})) {
            slot.required = false;
            slot.description = "Driven by a motion link; no motor mapping is needed.";
        }
    return out;
}

std::vector<std::string> slotOwners(const RobotConfig& c) {
    std::vector<std::string> out{sdk::slot_owner::kRobot, sdk::slot_owner::kDrivetrain};
    for (const auto& m : c.mechanisms) out.push_back(m.id);
    for (const auto& s : c.sensors) out.push_back(s.id);
    return out;
}

std::string slotDisplayName(const RobotConfig& c, const EditorCatalog& catalog, const SlotAddress& slot) {
    std::string ownerName = slot.owner == sdk::slot_owner::kRobot        ? "Robot"
                            : slot.owner == sdk::slot_owner::kDrivetrain ? "Drivetrain"
                                                                         : slot.owner;
    if (const MechanismConfig* m = c.findMechanism(slot.owner)) ownerName = displayNameFor(*m, catalog);
    if (const sdk::SensorConfig* s = c.findSensor(slot.owner)) ownerName = sensorDisplayName(*s);
    std::string slotName = slot.slot;
    for (const auto& s : slotSpecsFor(c, catalog, slot.owner))
        if (s.name == slot.slot) { slotName = s.label.empty() ? s.name : s.label; break; }
    return ownerName + " \xE2\x80\xBA " + slotName;
}

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

// ---- plugin-defined volumes -------------------------------------------------------------------
std::vector<VolumeRef> volumesFor(const RobotConfig& c, const EditorCatalog& catalog, std::string_view mechanismId) {
    std::vector<VolumeRef> out;
    if (mechanismId.empty()) {
        if (!catalog.hasPlugin()) return out;
        for (const auto& panel : catalog.pluginPanels)
            if (panel.scope == sdk::PanelScope::PluginConfig) addVolume(out, panel.properties, panel.overlays, "");
        addVolume(out, catalog.pluginSchema.properties, {}, "");
        return out;
    }
    const MechanismConfig* m = c.findMechanism(mechanismId);
    if (m == nullptr) return out;
    for (const auto& panel : catalog.pluginPanels) {
        if (panel.scope != sdk::PanelScope::Mechanism) continue;
        if (std::find(panel.mechanismTypes.begin(), panel.mechanismTypes.end(), m->type) == panel.mechanismTypes.end()) continue;
        addVolume(out, panel.properties, panel.overlays, mechanismId);
    }
    if (const auto* info = catalog.findTemplate(m->type)) addVolume(out, info->properties, {}, mechanismId);
    return out;
}

sdk::Value* volumeStore(RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    if (owner.empty()) {
        if (!catalog.hasPlugin()) return nullptr;
        return &c.pluginSection(catalog.pluginId).data;
    }
    if (MechanismConfig* m = c.findMechanism(owner)) return &m->properties;
    return nullptr;
}

const sdk::Value* volumeStore(const RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    if (owner.empty()) {
        if (!catalog.hasPlugin()) return nullptr;
        const sdk::PluginSection* s = c.findPlugin(catalog.pluginId);
        return s != nullptr ? &s->data : nullptr;
    }
    if (const MechanismConfig* m = c.findMechanism(owner)) return &m->properties;
    return nullptr;
}

}  // namespace wpiemu::app::editor::ops
