# 10. Robot interaction

## 10.1 Objectives

After this chapter the reader can compose a zone's world pose, evaluate a
mechanism-based activation condition, and calculate release velocity. Chapter
9 is the prerequisite. The zone component addresses R9.

## 10.2 Problem statement

An emulator cannot infer an intake from a motor controller's name. A motor
may drive a roller, elevator or flywheel. The robot model therefore declares
where an interaction occurs and which published mechanism reading enables it.

## 10.3 Background and definitions

**Definition 10.1 (interaction zone).** A zone combines an attachment, local pose,
shape and activation predicate. Its world transform is the chassis transform
times the attachment-link transform times the local transform.

A chassis attachment contributes the identity transform. A mechanism attachment
uses the named link pose from the mechanism directory. The pickup volume uses
full extents from configuration; the release muzzle uses a point and local
positive-X direction.

The shared SDK evaluator handles All and Any conditions and unit conversion.
An unresolved mechanism makes its condition false. A resolved mechanism with
no matching value uses the mechanism's active flag, as specified by the
[activation contract](../reference/game-plugin.md#zones-and-game-fed-sensors).

## 10.4 Design

A physical intake could use a sensor and simulated rollers. A geometric intake
instead tests cube centres against an attached box. The geometric model is
selected because it can be configured from generic mechanisms without a
new game-specific mechanism class.

**Invariant 10.1 (single custody transition).** A pickup consumes a field cube
and fills one held slot; release empties that same slot. A zone cannot release
an absent cube. Overlapping pickup candidates are resolved by pool index.

## 10.5 Implementation

`CubeZones` borrows the pool and world. Its warning set belongs to the session
and is cleared on reset. A missing attachment produces one diagnostic per
identifier, rather than one message on every 5 ms tick.

Listing 10.1. Zone orchestration and diagnostic state. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\cube_zones.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\cube_zones.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/cube_pool.h"

namespace wpiemu::plugins::game_2027 {

class CubeZones {
public:
    CubeZones(CubePool& pool, s::IPhysicsWorld& world) : pool_(pool), world_(world) {}
    void reset() { missingWarnings_.clear(); }
    void update(s::SimContext& ctx);

private:
    void warnMissing(s::SimContext& ctx, const std::string& id);
    CubePool& pool_;
    s::IPhysicsWorld& world_;
    std::vector<std::string> missingWarnings_;
};

}  // namespace wpiemu::plugins::game_2027
```

`update` first resolves attachment and activation, then computes the world
pose. Release adds the chassis point velocity to an ejection velocity of
0.2 m/s along the muzzle. Point velocity includes the contribution from
chassis rotation; copying only the chassis centre velocity would omit it.

Listing 10.2. Pickup and release from configured zones. <code>seasons\2027\game\src\cube_zones.cpp</code>.

<!-- book-file: seasons\2027\game\src\cube_zones.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/cube_zones.h"

namespace wpiemu::plugins::game_2027 {
void CubeZones::warnMissing(s::SimContext& ctx, const std::string& id) {
    if (std::find(missingWarnings_.begin(), missingWarnings_.end(), id) != missingWarnings_.end())
        return;
    missingWarnings_.push_back(id);
    ctx.logger().warn("game_2027",
                      "Power UP zone has an unresolved attachment or mechanism: " + id);
}

void CubeZones::update(s::SimContext& ctx) {
    if (!ctx.chassisBody().valid()) return;
    const auto chassis = world_.pose(ctx.chassisBody());
    for (const auto& zone : ctx.config().zones) {
        if (zone.kind != kPickup && zone.kind != kRelease) continue;
        s::Pose3 attachment;
        if (!zone.attachment.onChassis()) {
            const auto pose = ctx.mechanisms() == nullptr ? std::optional<s::Pose3>{}
                                                          : ctx.mechanisms()->linkPoseInChassis(
                                                                zone.attachment.parentMechanismId,
                                                                zone.attachment.parentLink);
            if (!pose) {
                warnMissing(ctx, zone.attachment.parentMechanismId);
                continue;
            }
            attachment = *pose;
        }

        std::vector<std::string> missing;
        const bool active = s::evaluateActivation(zone.activation, ctx.mechanisms(), &missing);
        for (const auto& id : missing) warnMissing(ctx, id);
        if (!active) continue;
        const auto pose = chassis * attachment * zone.local;

        if (zone.kind == kRelease) {
            const auto velocity = world_.pointVelocity(ctx.chassisBody(), pose.position) +
                                  pose.transformVector({0.2f, 0.f, 0.f});
            pool_.release(pose, velocity);
        } else {
            pool_.pickup(pose, zone.sizeM * 0.5f);
        }
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 10.3. Library sources through Chapter 10. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp
    src/field_layout.cpp
    src/cube_layout.cpp
    src/cube_pool.cpp
    src/plate_build.cpp
    src/plate_rules.cpp
    src/plates.cpp
    src/match_clock.cpp
    src/cube_zones.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

The ordering of configured zones is significant. The sample declares one
release zone. A model with multiple always-active pickup and release zones
would express a different policy and needs tests for that policy.

## 10.6 Verification

The pool tests already reject phantom releases. The zone tests in Chapter 12
add an unresolved mechanism, check that the preload remains held, and require
one warning. Another test picks up a real field cube, releases it and checks
that the body count is unchanged.

## 10.7 Reproduction

Extract through Chapter 10 and build the plugin target with warnings as errors.
Run the Chapter 7 conservation case. After assembly in Chapter 12, run the
pickup and unresolved-activation cases listed there.

## 10.8 Discussion

The sampled-volume interpretation is part of F-03 and F-08's physical model.
The SDK evaluator is reused rather than approximated with a comparison against
motor voltage. This preserves existing activation behavior for configured units
and missing readings.

## 10.9 Summary

Robot outputs now have a route to game actions through generic mechanisms and
configured zones. Geometry, activation and custody remain distinct concerns.
Release preserves the chassis contribution to velocity.

## 10.10 Exercises

1. Derive the release direction for a chassis facing positive Y.
2. Test an inactive zone with an otherwise valid preload.
3. Explain how zone ordering could produce a pickup and release in the same tick.

[Chapter 11](11-observability.md)
