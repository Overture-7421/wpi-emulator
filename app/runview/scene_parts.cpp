#include "scene_parts.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

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

bool isSeparator(char c) { return c == '.' || c == '/' || c == ':' || c == '_' || c == '-'; }

}  // namespace

std::string linkForPrimitive(const std::string& name, const std::vector<std::string>& linkNames) {
    std::string best = sdk::kRootLink;
    std::size_t bestLen = 0;
    for (const auto& link : linkNames) {
        if (link.empty() || link == sdk::kRootLink || link.size() <= bestLen || name.size() < link.size()) continue;
        if (name.compare(0, link.size(), link) != 0) continue;
        if (name.size() == link.size() || isSeparator(name[link.size()])) {
            best = link;
            bestLen = link.size();
        }
    }
    return best;
}

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
            b.link = p.link.empty() ? sdk::kRootLink : p.link;
            b.linkResolved = !p.link.empty();  // explicit link: skip the name-prefix derivation
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
                if (const sdk::MechanismState* st = findState(snapshot, b.owner); st != nullptr && !b.linkResolved) {
                    std::vector<std::string> names;
                    for (const auto& l : st->links) names.push_back(l.name);
                    b.link = linkForPrimitive(parts_[i].name, names);
                    b.linkResolved = true;
                }
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

std::vector<ZoneOverlay> resolveZoneOverlays(const sdk::RobotConfig& config,
                                             const std::vector<sdk::EditorPanelDescriptor>& panels,
                                             const sdk::ISimSnapshot* snapshot) {
    std::vector<ZoneOverlay> out;
    const sdk::LinkPoseSource source = linkSource(config, snapshot);
    for (const auto& panel : panels) {
        for (const auto& overlay : panel.overlays) {
            auto addOverlay = [&](const sdk::Value* v, const sdk::Pose3& frame) {
                sdk::Box3 box;
                if (v == nullptr || !sdk::fromValue(*v, box)) return;
                ZoneOverlay z;
                z.label = overlay.label.empty() ? overlay.propertyKey : overlay.label;
                z.color = overlay.color;
                z.pose = frame * box.pose;
                z.size = box.sizeM;
                out.push_back(std::move(z));
            };
            if (panel.scope == sdk::PanelScope::PluginConfig) {
                const std::string pluginId = panel.id.substr(0, panel.id.find('.'));
                if (const sdk::PluginSection* section = config.findPlugin(pluginId))
                    addOverlay(section->data.findPath(overlay.propertyKey), sdk::Pose3{});
            } else {
                for (const auto& m : config.mechanisms) {
                    if (std::find(panel.mechanismTypes.begin(), panel.mechanismTypes.end(), m.type) == panel.mechanismTypes.end())
                        continue;
                    const auto frame = sdk::resolveLinkPoseInChassis(config, m.id, sdk::kRootLink, source);
                    addOverlay(m.properties.findPath(overlay.propertyKey), frame ? *frame : sdk::Pose3{});
                }
            }
        }
    }
    return out;
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
    floor.pose.position = {out.center.x, out.center.y, -0.02f};
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
