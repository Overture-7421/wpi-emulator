#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/config/validation_codes.h"
#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/registry.h"
#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/mechanisms/mechanism_state.h"

namespace wpiemu::sdk {

class SimContext;

// A named attachment frame a mechanism exposes to children (SDK 1.1), e.g. an elevator's "carriage" or an
// arm's "tip". EVERY mechanism has the implicit, stable link kRootLink ("root") = its own origin
// (identity pose); it is never listed here and must not be redeclared. Other links are optional.
struct LinkSpec {
    std::string name;          // unique within the template, not "root"
    std::string label;         // UI text
    Pose3 local;               // rest pose relative to the mechanism root frame
    std::string description;
};

// Metadata that fully describes a mechanism TEMPLATE to the editor (palette, inspector, slot
// panel, default geometry) without the editor knowing the concrete class.
struct MechanismTemplateInfo {
    std::string type;          // registry key, namespaced for plugins: "roller", "game_2026.fuel_intake"
    std::string displayName;
    std::string category;      // palette group: "Intake", "Shooter", "Arm", "Generic", ...
    std::string description;
    std::string origin = "core";   // "core" or the registering plugin id
    std::vector<SlotSpec> slotSpecs;
    PropertySchema properties;
    std::vector<PrimitiveSpec> defaultGeometry;
    std::vector<LinkSpec> links;   // SDK 1.1: attachable links besides the implicit "root"
    std::vector<MotionAxisSpec> motionAxes;
};

// True for kRootLink and for every link the template declares.
inline bool hasLink(const MechanismTemplateInfo& info, std::string_view link) {
    if (link == kRootLink) return true;
    for (const LinkSpec& l : info.links)
        if (l.name == link) return true;
    return false;
}

// Template-level link contract: non-empty unique names, "root" never redeclared.
inline std::vector<Issue> templateLinkIssues(const MechanismTemplateInfo& info) {
    std::vector<Issue> out;
    for (std::size_t i = 0; i < info.links.size(); ++i) {
        const std::string& name = info.links[i].name;
        const std::string path = info.type + ".links[" + std::to_string(i) + "]";
        if (name.empty())
            out.push_back({Severity::Error, path, "link name must not be empty", validation_code::kTemplateLinkInvalid});
        else if (name == kRootLink)
            out.push_back({Severity::Error, path, "'root' is implicit and reserved", validation_code::kTemplateLinkInvalid});
        for (std::size_t j = 0; j < i; ++j)
            if (info.links[j].name == name) {
                out.push_back({Severity::Error, path, "duplicate link '" + name + "'", validation_code::kTemplateLinkInvalid});
                break;
            }
    }
    return out;
}

// What a mechanism receives at construction (constructor injection).
struct MechanismBuildContext {
    const MechanismConfig& config;   // instance id, mount, properties (already defaulted + validated)
    SimContext& sim;
};

// One user-placed mechanism instance (an intake, a shooter, an elevator...).
//
// Lifecycle & threading: created by MechanismFactory on the sim thread, ticked once per sim tick
// on the sim thread in config order; destroyed before the physics world. Not thread-safe; the UI
// only sees state() copies through ISimSnapshot.
//
// tick() contract:
//   1. read commands from the mapped slots (ctx.devices().motorOutput({id, "motor"}) ...),
//   2. advance its IPlant(s) by dt (mechanism-local physics only: NO rigid-body forces except
//      via ctx.physics() when the template documents it, e.g. a shooter spawning a ball, or via
//      ctx.coupling() (SDK 1.1: chassis wrenches, anchored constraints, payload mass): submit before
//      the physics step, read the PREVIOUS step's constraint loads),
//   3. write sensor feedback to the slots (encoders, limit switches, beam breaks),
//   4. update internal state() for display.
// Errors: no exceptions; invalid property values are rejected at construction by the factory.
class IMechanism {
    WPIEMU_INTERFACE(IMechanism)
public:
    virtual const std::string& id() const = 0;
    virtual std::string_view type() const = 0;
    virtual void tick(SimContext& ctx, double dtSeconds) = 0;
    virtual void reset() = 0;                       // back to the initial pose; keep config
    virtual MechanismState state() const = 0;       // cheap copy for the snapshot
    // Total electrical current drawn from the battery during the last tick (A), >= 0.
    virtual double batteryCurrentA() const { return 0.0; }
    virtual std::vector<MotionAxisBinding> motionAxes() { return {}; }
    // Called instead of tick after the runtime advanced the advertised axes exactly once.
    virtual void finishMotionTick(SimContext& ctx, double dtSeconds) { tick(ctx, dtSeconds); }
    // SDK 1.1. Pose of `link` relative to this mechanism's ROOT frame (the mount), as of the last tick.
    // "root" is always the identity. Default: looks `link` up in state().links; mechanisms SHOULD publish
    // every link their template declares there (moving links update each tick). nullopt = unknown link.
    virtual std::optional<Pose3> linkPose(std::string_view link) const {
        if (link == kRootLink) return Pose3{};
        for (const LinkPose& l : state().links)
            if (l.name == link) return l.pose;
        return std::nullopt;
    }
};

// Read access for game plugins / HUD to look at other mechanisms (e.g. "is the intake active?").
class IMechanismDirectory {
    WPIEMU_INTERFACE(IMechanismDirectory)
public:
    virtual IMechanism* find(std::string_view id) = 0;
    virtual std::vector<IMechanism*> all() = 0;
    // SDK 1.1. Read-only RESOLVED transform: pose of `link` of mechanism `mechanismId` in the CHASSIS frame,
    // composed through the whole attachment chain (mount of each mechanism * the parent link's current
    // pose, see mechanisms/attachment.h resolveLinkPoseInChassis). Reflects the mechanisms' poses as of
    // the moment of the call: mechanisms ticked earlier in the same pass show this tick, later ones the
    // previous tick (list parents before children in RobotConfig::mechanisms for a consistent view).
    // nullopt: unknown mechanism/link, or a broken chain (missing parent / cycle). Default: unsupported.
    virtual std::optional<Pose3> linkPoseInChassis(std::string_view /*mechanismId*/, std::string_view /*link*/) const {
        return std::nullopt;
    }
};

// Registry of mechanism templates. Core registers the generic set (roller, elevator, arm, ...);
// year plugins add theirs in IGameModule::registerMechanisms. Keys are unique across both.
class MechanismFactory : public Registry<MechanismTemplateInfo, IMechanism, const MechanismBuildContext&> {
public:
    // InvalidArgument when the template's links violate templateLinkIssues() (e.g. redeclares "root").
    Status registerTemplate(MechanismTemplateInfo info, Factory factory) {
        if (const auto issues = templateLinkIssues(info); !issues.empty())
            return makeError(ErrorCode::InvalidArgument, "mechanism template '" + info.type + "': " + issues.front().message);
        std::string key = info.type;
        return add(std::move(key), std::move(info), std::move(factory));
    }
    const MechanismTemplateInfo* templateInfo(std::string_view type) const { return info(type); }
};

}  // namespace wpiemu::sdk
