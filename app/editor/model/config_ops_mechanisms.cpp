#include <algorithm>
#include <cctype>

#include "model/config_ops.h"
#include "model/config_ops_internal.h"

namespace wpiemu::app::editor::ops {

using detail::onChassis;
using detail::reservedOwner;
using sdk::MechanismConfig;
using sdk::Pose3;
using sdk::RobotConfig;
using sdk::Vec3;

// ---- naming -----------------------------------------------------------------------------------
std::string baseIdForType(std::string_view type) {
    const auto dot = type.rfind('.');
    std::string base(dot == std::string_view::npos ? type : type.substr(dot + 1));
    for (char& ch : base)
        if (!std::isalnum(static_cast<unsigned char>(ch))) ch = '_';
    return base.empty() ? std::string("mechanism") : base;
}

std::string uniqueMechanismId(const RobotConfig& c, std::string base) {
    return detail::uniqueEntityId(c, std::move(base), "mechanism");
}

std::string displayNameFor(const MechanismConfig& m, const EditorCatalog& catalog) {
    if (!m.label.empty()) return m.label;
    if (const auto* t = catalog.findTemplate(m.type)) return t->displayName + " (" + m.id + ")";
    return m.id;
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
        if (reservedOwner(candidate) || c.hasEntityId(candidate)) return true;
        return std::any_of(renamed.begin(), renamed.end(), [&](const auto& r) { return r.second == candidate; });
    };
    for (const auto& oldId : subtree) {
        MechanismConfig copy = *c.findMechanism(oldId);
        copy.id = detail::uniqueSuffixed(oldId, "mechanism", "_", taken);
        renamed.emplace_back(oldId, copy.id);
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

}  // namespace wpiemu::app::editor::ops
