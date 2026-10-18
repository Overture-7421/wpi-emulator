#include "scene_parts.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

#include "common/camera_frustum.h"

namespace wpiemu::app::runview {
namespace {

constexpr float kHalfPi = static_cast<float>(sdk::kPi * 0.5);

const sdk::MechanismState* findState(const sdk::ISimSnapshot* snapshot, std::string_view id) {
    if (snapshot == nullptr) return nullptr;
    for (const auto& m : snapshot->mechanisms())
        if (m.id == id) return &m;
    return nullptr;
}

sdk::LinkPoseSource linkSource(const sdk::RobotConfig& config, const sdk::ISimSnapshot* snapshot) {
    return [&config, snapshot](std::string_view mechanism, std::string_view link) -> std::optional<sdk::Pose3> {
        if (config.findMechanism(mechanism) == nullptr) return std::nullopt;
        if (link == sdk::kRootLink) return sdk::Pose3{};
        if (const sdk::MechanismState* st = findState(snapshot, mechanism))
            for (const auto& l : st->links)
                if (l.name == link) return l.pose;
        return sdk::Pose3{};  // unknown / not yet published: rest pose
    };
}

ScenePart partFromPrimitive(const sdk::PrimitiveSpec& p) {
    ScenePart part;
    part.name = p.name;
    part.pose = p.local;
    part.color = p.color;
    if (p.kind == sdk::PrimitiveKind::Cylinder) {
        part.shape = PartShape::Cylinder;
        part.size = {p.radiusM * 2.f, p.radiusM * 2.f, p.heightM};
    } else {
        part.shape = PartShape::Box;
        part.size = p.sizeM;
    }
    return part;
}

}  // namespace

void RobotRig::setConfig(std::shared_ptr<const sdk::RobotConfig> config) {
    config_ = std::move(config);
    parts_.clear();
    bindings_.clear();
    changed_.clear();
    staticCount_ = 0;
    if (!config_) return;

    const sdk::ChassisSpec& ch = config_->chassis;
    auto addStatic = [&](ScenePart part) {
        Binding b;
        b.local = part.pose;
        parts_.push_back(std::move(part));
        bindings_.push_back(std::move(b));
    };
    if (ch.primitives.empty()) {
        ScenePart frame;
        frame.name = "chassis";
        frame.size = ch.sizeM;
        frame.pose.position = {-ch.comOffsetM.x, -ch.comOffsetM.y, ch.groundClearanceM + ch.sizeM.z * 0.5f - ch.comHeightM};
        frame.color = "#5d6b86";
        addStatic(std::move(frame));
    } else {
        for (const auto& p : ch.primitives) addStatic(partFromPrimitive(p));
    }
    {
        ScenePart front;
        front.name = "front.marker";
        front.size = {0.05f, ch.sizeM.y * 0.6f, 0.02f};
        front.pose.position = {ch.sizeM.x * 0.5f - 0.04f - ch.comOffsetM.x, -ch.comOffsetM.y,
                               ch.groundClearanceM + ch.sizeM.z - ch.comHeightM + 0.012f};
        front.color = "#ffc933";
        addStatic(std::move(front));
    }
    staticCount_ = parts_.size();

    for (const auto& w : config_->drivetrain.spec.wheels) {
        ScenePart tire;
        tire.name = w.name;
        tire.shape = PartShape::Cylinder;
        tire.size = {w.radiusM * 2.f, w.radiusM * 2.f, w.widthM};
        tire.color = "#20242b";
        tire.pose.position = {w.positionM.x, w.positionM.y, w.positionM.z - ch.comHeightM};
        ScenePart marker;
        marker.name = w.name + ".marker";
        marker.size = {w.radiusM * 1.8f, w.radiusM * 0.3f, w.widthM * 1.05f};
        marker.color = "#aab3c2";
        marker.pose = tire.pose;
        Binding b;
        b.kind = Binding::Kind::Wheel;
        b.owner = w.name;
        b.wheelRadius = w.radiusM;
        b.local = tire.pose;
        parts_.push_back(std::move(tire));
        bindings_.push_back(b);
        b.kind = Binding::Kind::WheelMarker;
        parts_.push_back(std::move(marker));
        bindings_.push_back(b);
    }

    for (const auto& m : config_->mechanisms) {
        for (const auto& p : m.geometry) {
            Binding b;
            b.kind = Binding::Kind::Mechanism;
            b.owner = m.id;
            b.local = p.local;
            parts_.push_back(partFromPrimitive(p));
            bindings_.push_back(std::move(b));
        }
    }
    changed_.assign(parts_.size(), 1);
    update(nullptr);
}

bool RobotRig::update(const sdk::ISimSnapshot* snapshot) {
    changed_.assign(parts_.size(), 0);
    if (!config_) return false;
    const sdk::LinkPoseSource source = linkSource(*config_, snapshot);
    const sdk::Pose3 chassisInv = snapshot != nullptr ? snapshot->chassis().pose.inverse() : sdk::Pose3{};
    bool any = false;

    for (std::size_t i = staticCount_; i < parts_.size(); ++i) {
        Binding& b = bindings_[i];
        sdk::Pose3 next = parts_[i].pose;
        switch (b.kind) {
            case Binding::Kind::Static: continue;
            case Binding::Kind::Wheel:
            case Binding::Kind::WheelMarker: {
                sdk::Vec3 hub = b.local.position;
                float steer = 0.f, spin = 0.f;
                if (snapshot != nullptr) {
                    for (const auto& w : snapshot->wheels()) {
                        if (w.name != b.owner) continue;
                        if (!w.airborne) hub = chassisInv.transformPoint(w.contactPoint + sdk::Vec3{0.f, 0.f, b.wheelRadius});
                        steer = static_cast<float>(w.steerAngleRad);
                        spin = static_cast<float>(w.spinAngleRad);
                        break;
                    }
                }
                next.position = hub;
                // Cylinder axis (local Z) -> chassis +Y, steered about chassis Z, spun about its own axle.
                next.rotation = sdk::Quat::fromYaw(steer) * sdk::Quat::fromAxisAngle({1.f, 0.f, 0.f}, -kHalfPi) *
                                sdk::Quat::fromAxisAngle({0.f, 0.f, 1.f}, spin);
                break;
            }
            case Binding::Kind::Mechanism: {
                // All of a mechanism's shapes move together with its first (moving) link; without one they stay on the root.
                if (const sdk::MechanismState* st = findState(snapshot, b.owner); st != nullptr && !st->links.empty())
                    b.link = st->links.front().name;
                const auto linkInChassis = sdk::resolveLinkPoseInChassis(*config_, b.owner, b.link, source);
                next = (linkInChassis ? *linkInChassis : sdk::Pose3{}) * b.local;
                break;
            }
        }
        if (!(next == parts_[i].pose)) {
            parts_[i].pose = next;
            changed_[i] = 1;
            any = true;
        }
    }
    return any;
}

namespace {

// An overlay whose pose is `frame(mechanism, link) * local`; an empty mechanism means the chassis frame.
struct ZoneBinding {
    ZoneOverlay overlay;
    std::string mechanism;
    std::string link = sdk::kRootLink;
    sdk::Pose3 local;
};

void collectPanelBindings(const sdk::RobotConfig& config, const std::vector<sdk::EditorPanelDescriptor>& panels,
                          std::vector<ZoneBinding>& out) {
    for (const auto& panel : panels) {
        for (const auto& overlay : panel.overlays) {
            auto add = [&](const sdk::Value* v, const std::string& mechanism) {
                sdk::Box3 box;
                if (v == nullptr || !sdk::fromValue(*v, box)) return;
                ZoneBinding b;
                b.overlay.label = overlay.label.empty() ? overlay.propertyKey : overlay.label;
                b.overlay.color = overlay.color;
                b.overlay.size = box.sizeM;
                b.mechanism = mechanism;
                b.local = box.pose;
                out.push_back(std::move(b));
            };
            if (panel.scope == sdk::PanelScope::PluginConfig) {
                const std::string pluginId = panel.id.substr(0, panel.id.find('.'));
                if (const sdk::PluginSection* section = config.findPlugin(pluginId))
                    add(section->data.findPath(overlay.propertyKey), {});
            } else {
                for (const auto& m : config.mechanisms) {
                    if (std::find(panel.mechanismTypes.begin(), panel.mechanismTypes.end(), m.type) == panel.mechanismTypes.end())
                        continue;
                    add(m.properties.findPath(overlay.propertyKey), m.id);
                }
            }
        }
    }
}

void collectZoneBindings(const sdk::RobotConfig& config, const std::vector<sdk::ZoneKindInfo>& kinds,
                         std::vector<ZoneBinding>& out) {
    for (const auto& zone : config.zones) {
        const auto kind = std::find_if(kinds.begin(), kinds.end(), [&](const sdk::ZoneKindInfo& k) { return k.kind == zone.kind; });
        if (kind == kinds.end()) continue;
        ZoneBinding b;
        b.overlay.id = zone.id;
        b.overlay.label = zone.label.empty() ? zone.id : zone.label;
        b.overlay.color = kind->color;
        b.overlay.size = zone.sizeM;
        b.overlay.muzzle = kind->shape == sdk::ZoneShape::Muzzle;
        b.overlay.tracksState = true;
        if (!zone.attachment.onChassis()) {
            b.mechanism = zone.attachment.parentMechanismId;
            b.link = zone.attachment.parentLink;
        }
        b.local = zone.local;
        out.push_back(std::move(b));
    }
}

// Pose of the frame a binding hangs on (identity on the chassis, or when the chain cannot be resolved).
sdk::Pose3 frameOf(const sdk::RobotConfig& config, const sdk::LinkPoseSource& source, const ZoneBinding& b) {
    if (b.mechanism.empty()) return {};
    const auto frame = sdk::resolveLinkPoseInChassis(config, b.mechanism, b.link, source);
    return frame ? *frame : sdk::Pose3{};
}

bool zoneActive(const sdk::ISimSnapshot* snapshot, const std::string& id) {
    if (snapshot == nullptr) return false;
    for (const auto& z : snapshot->zones())
        if (z.id == id) return z.active;
    return false;
}

constexpr float kMuzzleMarkerM = 0.08f;
constexpr float kMuzzleArrowLengthM = 0.3f;
constexpr float kMuzzleArrowThicknessM = 0.025f;
constexpr float kActiveOpacity = 0.55f;
constexpr float kInactiveOpacity = 0.12f;
constexpr float kActiveMuzzleOpacity = 0.95f;
constexpr float kInactiveMuzzleOpacity = 0.4f;

std::size_t partCount(const ZoneOverlay& z) { return z.muzzle ? 2 : 1; }

}  // namespace

std::vector<ZoneOverlay> resolveZoneOverlays(const sdk::RobotConfig& config,
                                             const std::vector<sdk::EditorPanelDescriptor>& panels,
                                             const sdk::ISimSnapshot* snapshot) {
    std::vector<ZoneBinding> bindings;
    collectPanelBindings(config, panels, bindings);
    const sdk::LinkPoseSource source = linkSource(config, snapshot);
    std::vector<ZoneOverlay> out;
    out.reserve(bindings.size());
    for (auto& b : bindings) {
        b.overlay.pose = frameOf(config, source, b) * b.local;
        out.push_back(std::move(b.overlay));
    }
    return out;
}

std::vector<ZoneOverlay> resolveRobotZones(const sdk::RobotConfig& config, const std::vector<sdk::ZoneKindInfo>& kinds,
                                           const sdk::ISimSnapshot* snapshot) {
    std::vector<ZoneBinding> bindings;
    collectZoneBindings(config, kinds, bindings);
    const sdk::LinkPoseSource source = linkSource(config, snapshot);
    std::vector<ZoneOverlay> out;
    out.reserve(bindings.size());
    for (auto& b : bindings) {
        b.overlay.pose = frameOf(config, source, b) * b.local;
        b.overlay.active = zoneActive(snapshot, b.overlay.id);
        out.push_back(std::move(b.overlay));
    }
    return out;
}

void ZoneRig::setScene(std::shared_ptr<const sdk::RobotConfig> config, std::vector<sdk::EditorPanelDescriptor> panels,
                       std::vector<sdk::ZoneKindInfo> kinds) {
    config_ = std::move(config);
    overlays_.clear();
    bindings_.clear();
    parts_.clear();
    labels_.clear();
    changed_.clear();
    if (!config_) return;

    std::vector<ZoneBinding> bound;
    collectPanelBindings(*config_, panels, bound);
    collectZoneBindings(*config_, kinds, bound);
    for (auto& zb : bound) {
        Binding b;
        b.mechanism = zb.mechanism;
        b.link = zb.link;
        b.local = zb.local;
        b.firstPart = parts_.size();
        auto addPart = [&](sdk::Vec3 size) {
            ScenePart p;
            p.name = zb.overlay.label;
            p.size = size;
            p.color = zb.overlay.color;
            p.translucent = true;
            parts_.push_back(std::move(p));
            labels_.push_back(zb.overlay.label);
        };
        if (zb.overlay.muzzle) {
            addPart({kMuzzleMarkerM, kMuzzleMarkerM, kMuzzleMarkerM});
            addPart({kMuzzleArrowLengthM, kMuzzleArrowThicknessM, kMuzzleArrowThicknessM});
        } else {
            addPart(zb.overlay.size);
        }
        bindings_.push_back(std::move(b));
        // Chassis-frame panel overlays never move: place them once.
        if (!zb.overlay.tracksState && zb.mechanism.empty()) {
            zb.overlay.pose = zb.local;
            parts_[bindings_.back().firstPart].pose = zb.local;
        }
        overlays_.push_back(std::move(zb.overlay));
    }
    changed_.assign(parts_.size(), 1);
    update(nullptr);
    changed_.assign(parts_.size(), 1);
}

bool ZoneRig::update(const sdk::ISimSnapshot* snapshot) {
    changed_.assign(parts_.size(), 0);
    if (!config_) return false;
    const sdk::LinkPoseSource source = linkSource(*config_, snapshot);
    bool any = false;

    for (std::size_t i = 0; i < overlays_.size(); ++i) {
        ZoneOverlay& z = overlays_[i];
        const Binding& b = bindings_[i];
        if (!z.tracksState && b.mechanism.empty()) continue;

        sdk::Pose3 frame;
        if (!b.mechanism.empty()) {
            const auto resolved = sdk::resolveLinkPoseInChassis(*config_, b.mechanism, b.link, source);
            if (resolved) frame = *resolved;
        }
        z.pose = frame * b.local;
        z.active = z.tracksState && zoneActive(snapshot, z.id);

        const float opacity = !z.tracksState ? 0.28f
                              : z.muzzle     ? (z.active ? kActiveMuzzleOpacity : kInactiveMuzzleOpacity)
                                             : (z.active ? kActiveOpacity : kInactiveOpacity);
        for (std::size_t k = 0; k < partCount(z); ++k) {
            const std::size_t idx = b.firstPart + k;
            ScenePart& part = parts_[idx];
            sdk::Pose3 pose = z.pose;
            if (z.muzzle && k == 1) pose = z.pose * sdk::Pose3({kMuzzleArrowLengthM * 0.5f + kMuzzleMarkerM * 0.5f, 0.f, 0.f}, sdk::Quat{});
            if (pose == part.pose && opacity == part.opacity && z.active == part.highlighted) continue;
            part.pose = pose;
            part.opacity = opacity;
            part.highlighted = z.active;
            changed_[idx] = 1;
            any = true;
        }
    }
    return any;
}

namespace {

constexpr float kFrustumEdgeThicknessM = 0.02f;
constexpr const char* kLimelightColor = "#ff8a00";
constexpr const char* kPhotonVisionColor = "#3b82f6";
constexpr const char* kSeesTagsColor = "#22e06b";
constexpr float kFrustumActiveOpacity = 0.75f;
constexpr float kFrustumInactiveOpacity = 0.3f;

// The 8 edges of the shared frustum geometry (common/camera_frustum.h) as box poses whose local +X runs along the edge.
std::vector<sdk::Pose3> frustumEdgePoses(const sdk::CameraConfig& camera, float lengthM, std::vector<float>& edgeLengths) {
    const FrustumGeometry g = cameraFrustum(camera, lengthM);
    std::vector<sdk::Pose3> poses;
    edgeLengths.clear();
    for (const auto& [from, to] : kFrustumEdges) {
        const sdk::Vec3 a = g.pts[from], b = g.pts[to];
        const sdk::Vec3 d = b - a;
        const sdk::Vec3 dir = sdk::normalized(d);
        // Shortest rotation taking +X onto dir.
        const sdk::Vec3 axis = sdk::cross({1.f, 0.f, 0.f}, dir);
        const sdk::Quat rot = dir.x < -0.9999f ? sdk::Quat::fromYaw(static_cast<float>(sdk::kPi))
                                               : sdk::Quat{axis.x, axis.y, axis.z, 1.f + dir.x}.normalized();
        poses.emplace_back((a + b) * 0.5f, rot);
        edgeLengths.push_back(sdk::length(d));
    }
    return poses;
}

const char* cameraKindColor(sdk::CameraKind kind) {
    return kind == sdk::CameraKind::PhotonVision ? kPhotonVisionColor : kLimelightColor;
}

bool cameraActive(const sdk::ISimSnapshot* snapshot, const std::string& id) {
    if (snapshot == nullptr) return false;
    for (const auto& c : snapshot->cameras())
        if (c.id == id) return c.tagsVisible;
    return false;
}

sdk::Pose3 attachmentFrame(const sdk::RobotConfig& config, const sdk::LinkPoseSource& source,
                           const sdk::MechanismAttachment& attachment) {
    if (attachment.onChassis()) return {};
    const auto frame = sdk::resolveLinkPoseInChassis(config, attachment.parentMechanismId, attachment.parentLink, source);
    return frame ? *frame : sdk::Pose3{};
}

CameraOverlay makeCameraOverlay(const sdk::CameraConfig& c) {
    CameraOverlay o;
    o.id = c.id;
    o.label = c.label.empty() ? c.id : c.label;
    o.kind = c.kind;
    o.hFovDeg = static_cast<float>(c.hFovDeg);
    o.vFovDeg = static_cast<float>(c.vFovDeg);
    o.frustumLengthM = frustumDisplayLengthM(c);
    return o;
}

}  // namespace

std::vector<CameraOverlay> resolveRobotCameras(const sdk::RobotConfig& config, const sdk::ISimSnapshot* snapshot) {
    const sdk::LinkPoseSource source = linkSource(config, snapshot);
    std::vector<CameraOverlay> out;
    out.reserve(config.cameras.size());
    for (const auto& c : config.cameras) {
        CameraOverlay o = makeCameraOverlay(c);
        o.pose = attachmentFrame(config, source, c.attachment) * c.local;
        o.active = cameraActive(snapshot, c.id);
        out.push_back(std::move(o));
    }
    return out;
}

void CameraRig::setScene(std::shared_ptr<const sdk::RobotConfig> config) {
    config_ = std::move(config);
    cameras_.clear();
    bindings_.clear();
    parts_.clear();
    labels_.clear();
    changed_.clear();
    if (!config_) return;

    for (const auto& c : config_->cameras) {
        CameraOverlay o = makeCameraOverlay(c);
        Binding b;
        if (!c.attachment.onChassis()) {
            b.mechanism = c.attachment.parentMechanismId;
            b.link = c.attachment.parentLink;
        }
        b.local = c.local;
        std::vector<float> lengths;
        b.edges = frustumEdgePoses(c, o.frustumLengthM, lengths);

        ScenePart body;
        body.name = o.label;
        body.size = {0.05f, 0.1f, 0.04f};
        body.color = cameraKindColor(o.kind);
        parts_.push_back(std::move(body));
        for (const float len : lengths) {
            ScenePart edge;
            edge.name = o.label;
            edge.size = {len, kFrustumEdgeThicknessM, kFrustumEdgeThicknessM};
            edge.color = cameraKindColor(o.kind);
            edge.translucent = true;
            edge.opacity = kFrustumInactiveOpacity;
            parts_.push_back(std::move(edge));
        }
        labels_.insert(labels_.end(), kPartsPerCamera, o.label);
        bindings_.push_back(std::move(b));
        cameras_.push_back(std::move(o));
    }
    changed_.assign(parts_.size(), 1);
    update(nullptr);
    changed_.assign(parts_.size(), 1);
}

bool CameraRig::update(const sdk::ISimSnapshot* snapshot) {
    changed_.assign(parts_.size(), 0);
    if (!config_) return false;
    const sdk::LinkPoseSource source = linkSource(*config_, snapshot);
    bool any = false;

    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        CameraOverlay& cam = cameras_[i];
        const Binding& b = bindings_[i];
        const sdk::CameraConfig& cfg = config_->cameras[i];
        cam.pose = attachmentFrame(*config_, source, cfg.attachment) * b.local;
        cam.active = cameraActive(snapshot, cam.id);

        const char* frustumColor = cam.active ? kSeesTagsColor : cameraKindColor(cam.kind);
        const float opacity = cam.active ? kFrustumActiveOpacity : kFrustumInactiveOpacity;
        for (std::size_t k = 0; k < kPartsPerCamera; ++k) {
            const std::size_t idx = i * kPartsPerCamera + k;
            ScenePart& part = parts_[idx];
            const sdk::Pose3 pose = k == 0 ? cam.pose : cam.pose * b.edges[k - 1];
            const bool edge = k != 0;
            const float nextOpacity = edge ? opacity : part.opacity;
            const char* nextColor = edge ? frustumColor : part.color.c_str();
            if (pose == part.pose && nextOpacity == part.opacity && cam.active == part.highlighted && part.color == nextColor)
                continue;
            part.pose = pose;
            part.opacity = nextOpacity;
            part.highlighted = cam.active;
            if (edge) part.color = nextColor;
            changed_[idx] = 1;
            any = true;
        }
    }
    return any;
}

void TagRig::setLayout(const sdk::AprilTagLayout& layout) {
    tags_.clear();
    parts_.clear();
    changed_.clear();
    tagChanged_.clear();
    for (const auto& t : layout.tags) {
        TagOverlay o;
        o.id = t.id;
        o.pose = t.pose;
        o.sizeM = static_cast<float>(t.sizeM);
        tags_.push_back(o);
        // +X is the face normal, so the plates are thin along X; thicker layers stand proud of the thinner ones.
        const auto plate = [&](const char* suffix, float side, float thicknessM, const char* color) {
            ScenePart p;
            p.name = "tag " + std::to_string(t.id) + suffix;
            p.pose = t.pose;
            p.size = {thicknessM, side, side};
            p.color = color;
            parts_.push_back(std::move(p));
        };
        plate(".border", o.sizeM * 1.4f, 0.006f, "#6b7280");
        plate(".plate", o.sizeM * 1.2f, 0.01f, "#f4f4f5");
        plate(".black", o.sizeM, 0.014f, "#111114");
    }
    changed_.assign(parts_.size(), 1);
    tagChanged_.assign(tags_.size(), 1);
}

bool TagRig::update(const sdk::ISimSnapshot* snapshot) {
    changed_.assign(parts_.size(), 0);
    tagChanged_.assign(tags_.size(), 0);
    bool any = false;
    for (std::size_t i = 0; i < tags_.size(); ++i) {
        bool seen = false;
        if (snapshot != nullptr) {
            for (const auto& cam : snapshot->cameras()) {
                seen = std::find(cam.visibleTagIds.begin(), cam.visibleTagIds.end(), tags_[i].id) != cam.visibleTagIds.end();
                if (seen) break;
            }
        }
        if (seen == tags_[i].seen) continue;
        tags_[i].seen = seen;
        tagChanged_[i] = 1;
        ScenePart& border = parts_[i * kPartsPerTag];
        border.color = seen ? kSeesTagsColor : "#6b7280";
        changed_[i * kPartsPerTag] = 1;
        any = true;
    }
    return any;
}

FieldParts buildFieldParts(const sdk::FieldDescription* field) {
    FieldParts out;
    if (field == nullptr) return out;
    if (field->lengthM > 0.f) out.lengthM = field->lengthM;
    if (field->widthM > 0.f) out.widthM = field->widthM;

    float minX = std::numeric_limits<float>::max(), minY = minX, maxX = -minX, maxY = -minX;
    for (const auto& v : field->visuals) {
        ScenePart p;
        p.name = v.name;
        p.pose = v.pose;
        p.color = v.color;
        if (v.kind == sdk::FieldVisual::Kind::Mesh) continue;  // mesh assets are not loaded yet; colliders still exist
        p.shape = v.kind == sdk::FieldVisual::Kind::Cylinder ? PartShape::Cylinder : PartShape::Box;
        p.size = v.sizeM;
        const float ex = std::max(v.sizeM.x, v.sizeM.y) * 0.5f;
        minX = std::min(minX, v.pose.position.x - ex);
        maxX = std::max(maxX, v.pose.position.x + ex);
        minY = std::min(minY, v.pose.position.y - ex);
        maxY = std::max(maxY, v.pose.position.y + ex);
        out.parts.push_back(std::move(p));
    }
    if (maxX > minX && maxY > minY)
        out.center = {(minX + maxX) * 0.5f, (minY + maxY) * 0.5f, 0.f};
    else
        out.center = {out.lengthM * 0.5f, out.widthM * 0.5f, 0.f};

    ScenePart floor;
    floor.name = "field.floor";
    floor.size = {out.lengthM, out.widthM, 0.04f};
    // Top face sits just below z=0 so it never shares a plane with a field's own ground/carpet box.
    floor.pose.position = {out.center.x, out.center.y, -0.03f};
    floor.color = "#2d333c";
    out.parts.insert(out.parts.begin(), std::move(floor));
    return out;
}

DisplayValue toDisplayValue(const sdk::NamedValue& v) {
    if (v.unit == "rad") return {v.name, sdk::radToDeg(v.value), "°"};
    if (v.unit == "rad/s") return {v.name, sdk::radPerSecToRpm(v.value), "rpm"};
    return {v.name, v.value, v.unit};
}

}  // namespace wpiemu::app::runview
