#pragma once

// Reusable contract for a game plugin that declares zone kinds: it must NOTICE the remote robots of a practice match.
// A plugin whose zone kinds are meant for robots (pickups, shooters, scoring mouths...) but whose game state never
// looks at ctx.robots() plays one-robot rules for everybody else; that silently makes a practice match meaningless.
//
//   const std::string violation = sdk::testing::remoteRobotContractViolation(module);
//   CHECK(violation.empty());   // doctest prints the reason through CHECK_MESSAGE(violation.empty(), violation)
//
// It drives the module like the runtime does (createField, createGameState, onRobotAdded for the host, reset, ticks,
// onRobotAdded for two remote robots that carry one zone of every kind the module declares with their flags set,
// publish + publishRobot) over FakeSimEnvironment, and reports the first rule that does not hold:
//   1. REACTS: with the remote robots present the snapshot differs from the run without them, either through a per-robot HUD
//      value (SnapshotBuilder::setRobotHud, from IGameState::publishRobot) or through the shared publish() output.
//   2. DECLARES IT: a plugin that reacts sets GameMetadata::supportsMultiRobot (the host reads it).
//   3. ORDER: the outcome does not depend on the order in which the robots were announced (iterate by ascending id:
//      sdk::PerRobotState).
//   4. LEAVES: removing the robots again and ticking on works.
// A plugin without zone kinds has nothing to react to and passes. It cannot see rules that need a specific field layout
// (a remote robot has to stand next to the game pieces to change them); those get their own plugin tests.
// Contract tests registered once over every built-in plugin (tests/sdk/test_multi_robot_contract.cpp) cover a new season
// as soon as its plugin is registered.

#include <algorithm>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/config/zone.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/robot_view.h"
#include "wpiemu/sdk/game/zone_pose.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/testing/fake_robot_directory.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

namespace wpiemu::sdk::testing {

namespace multi_robot_contract_detail {

inline void appendValue(std::string& out, const Value& v) {
    if (v.isNull()) out += "null";
    else if (v.isBool()) out += v.asBool() ? "true" : "false";
    else if (v.isNumber()) {
        char buf[48];
        std::snprintf(buf, sizeof buf, "%.9g", v.asDouble());
        out += buf;
    } else if (v.isString()) out += '"' + v.asString() + '"';
    else if (v.isArray()) {
        out += '[';
        for (const Value& e : v.array()) { appendValue(out, e); out += ','; }
        out += ']';
    } else {
        out += '{';
        const Value::Object& o = v.object();
        for (std::size_t i = 0; i < o.size(); ++i) { out += o.keyAt(i) + '='; appendValue(out, o.valueAt(i)); out += ','; }
        out += '}';
    }
}

inline void appendHud(std::string& out, const std::vector<HudValue>& hud) {
    for (const HudValue& h : hud) { out += h.key + '='; appendValue(out, h.value); out += ';'; }
}

// What the plugin published for everybody (HUD, entity groups, host zones).
inline std::string sharedDigest(const ISimSnapshot& snap) {
    std::string out;
    for (const HudValue& h : snap.hud()) { out += h.key + '='; appendValue(out, h.value); out += ';'; }
    for (const EntityGroupSample& g : snap.entityGroups()) {
        out += "group " + g.group + ':';
        char buf[96];
        for (const Vec3& p : g.positions) {
            std::snprintf(buf, sizeof buf, "(%.5g,%.5g,%.5g)", static_cast<double>(p.x), static_cast<double>(p.y),
                          static_cast<double>(p.z));
            out += buf;
        }
        for (const std::uint32_t id : g.ids) out += '#' + std::to_string(id);
        out += ';';
    }
    for (const ZoneState& z : snap.zones()) out += "zone " + z.id + (z.active ? "+" : "-") + ';';
    return out;
}

// What the plugin published per remote robot (publishRobot), keyed by id.
inline std::string robotDigest(const ISimSnapshot& snap, bool& anyHud) {
    std::vector<const RobotSample*> robots;
    for (const RobotSample& r : snap.remoteRobots()) robots.push_back(&r);
    std::sort(robots.begin(), robots.end(),
              [](const RobotSample* a, const RobotSample* b) { return a->participantId < b->participantId; });
    std::string out;
    for (const RobotSample* r : robots) {
        out += "robot " + std::to_string(r->participantId) + ':';
        appendHud(out, r->hud);
        anyHud = anyHud || !r->hud.empty();
        out += '|';
    }
    return out;
}

struct Outcome {
    std::string error;       // the module could not even be driven
    std::string shared;
    std::string perRobot;
    bool robotHud = false;   // some remote robot got a per-robot HUD value
};

class Scenario {
public:
    explicit Scenario(IGameModule& module) : module_(module) {}

    // Runs the module with the remote robots `joinOrder` (ids), announced in that order, then removes them again.
    Outcome run(const std::vector<ParticipantId>& joinOrder) {
        Outcome outcome;
        FakeSimEnvironment env;
        MechanismFactory factory;
        if (const Status st = module_.registerMechanisms(factory); !st.ok()) return fail("registerMechanisms: " + st.error().message);
        auto field = module_.createField(env.physics);
        if (!field.ok()) return fail("createField: " + field.error().message);

        BodyDesc chassisDesc;
        chassisDesc.type = BodyType::Kinematic;
        const BodyId chassis = env.physics.createBody(chassisDesc);
        env.ctx.bindChassisBody(chassis);
        FakeRobotDirectory directory;
        env.ctx.bindRobots(&directory);
        RobotView host;
        host.participantId = match::kHostParticipantId;
        host.name = "Host";
        host.body = chassis;
        host.enabled = true;
        host.isHost = true;
        directory.add(host);

        auto created = module_.createGameState(env.ctx);
        if (!created.ok()) return fail("createGameState: " + created.error().message);
        std::unique_ptr<IGameState> state = std::move(created).value();
        env.ctx.bindGameState(state.get());
        state->onRobotAdded(env.ctx, directory.list.front());
        state->reset(env.ctx);

        const auto tick = [&](int n) {
            for (int i = 0; i < n; ++i) {
                env.advance();
                state->step(env.ctx, env.clock.fixedStepSeconds());
                module_.onTick(env.ctx, env.clock.fixedStepSeconds());
            }
        };
        tick(3);
        for (const ParticipantId id : joinOrder) join(env, directory, *state, id);
        tick(10);

        auto snapshot = publish(*state, directory);
        outcome.shared = sharedDigest(*snapshot);
        outcome.perRobot = robotDigest(*snapshot, outcome.robotHud);

        for (const ParticipantId id : joinOrder) {
            directory.remove(id);
            state->onRobotRemoved(env.ctx, id);
        }
        tick(3);
        (void)publish(*state, directory);
        return outcome;
    }

private:
    struct Remote {
        std::vector<ZoneConfig> zones;
        std::vector<Pose3> poses;
    };

    Outcome fail(std::string message) {
        Outcome o;
        o.error = std::move(message);
        return o;
    }

    void join(FakeSimEnvironment& env, FakeRobotDirectory& directory, IGameState& state, ParticipantId id) {
        auto remote = std::make_unique<Remote>();
        std::size_t index = 0;
        for (const ZoneKindInfo& kind : module_.contributeZoneKinds()) {
            ZoneConfig zone;
            zone.id = "contract_zone_" + std::to_string(index++);
            zone.kind = kind.kind;
            zone.properties = defaultsOf(kind.properties);
            zone.local = kind.defaultLocal;
            zone.sizeM = kind.defaultSizeM;
            remote->poses.push_back(remoteZoneSpecPose(zone));
            remote->zones.push_back(std::move(zone));
        }
        BodyDesc desc;
        desc.type = BodyType::Kinematic;
        desc.pose = Pose3{Vec3{2.f * static_cast<float>(id), 1.f, 0.f}, Quat::fromYaw(0.f)};

        RobotView view;
        view.participantId = id;
        view.name = "Remote" + std::to_string(id);
        view.station = id % 2 == 0 ? proto::AllianceStation::Blue1 : proto::AllianceStation::Red2;
        view.body = env.physics.createBody(desc);
        view.zones = &remote->zones;
        view.zoneFlags = remote->zones.size() >= 32 ? ~0u : (1u << remote->zones.size()) - 1u;
        view.zonePoses = remote->poses;
        view.enabled = true;
        remotes_[id] = std::move(remote);

        // The runtime keeps its roster ordered by id; announce in the caller's order.
        const auto at = std::find_if(directory.list.begin(), directory.list.end(),
                                     [id](const RobotView& r) { return r.participantId > id; });
        directory.list.insert(at, std::move(view));
        const RobotView* added = directory.find(id);
        state.onRobotAdded(env.ctx, *added);
    }

    static std::shared_ptr<const ISimSnapshot> publish(const IGameState& state, const FakeRobotDirectory& directory) {
        SnapshotBuilder out;
        state.publish(out);
        for (const RobotView& r : directory.list) {
            if (r.isHost) continue;
            RobotSample sample;
            sample.participantId = r.participantId;
            sample.name = r.name;
            sample.station = r.station;
            out.addRobot(std::move(sample));
        }
        for (const RobotView& r : directory.list)
            if (!r.isHost) state.publishRobot(out, r);
        return out.build();
    }

    IGameModule& module_;
    std::map<ParticipantId, std::unique_ptr<Remote>> remotes_;
};

}  // namespace multi_robot_contract_detail

// "" when `module` honours the contract above, else the first violated rule in words.
inline std::string remoteRobotContractViolation(IGameModule& module) {
    using namespace multi_robot_contract_detail;
    const GameMetadata meta = module.metadata();
    if (module.contributeZoneKinds().empty()) return {};

    const std::string who = "plugin '" + meta.id + "': ";
    const Outcome alone = Scenario(module).run({});
    if (!alone.error.empty()) return who + alone.error;
    const Outcome ascending = Scenario(module).run({1, 2});
    if (!ascending.error.empty()) return who + ascending.error;
    const Outcome descending = Scenario(module).run({2, 1});
    if (!descending.error.empty()) return who + descending.error;

    const bool reacts = ascending.robotHud || ascending.shared != alone.shared;
    if (!reacts)
        return who + "declares zone kinds but ignores remote robots: with two remote robots (zone flags set, enabled) the "
                     "snapshot equals the run without them. Create per-robot state in onRobotAdded (sdk::PerRobotState), act on "
                     "RobotView::zoneFlags / zonePoses in step() and publish per-robot values from publishRobot.";
    if (!meta.supportsMultiRobot)
        return who + "reacts to remote robots but GameMetadata::supportsMultiRobot is false: set it so hosts know the rules apply";
    if (ascending.shared != descending.shared || ascending.perRobot != descending.perRobot)
        return who + "the outcome depends on the order in which the remote robots were announced: iterate robots by ascending "
                     "participant id (sdk::PerRobotState::forEach)";
    return {};
}

}  // namespace wpiemu::sdk::testing
